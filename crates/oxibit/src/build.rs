//! `oxibit fasm2bit` and `oxibit build-port`: FASM -> full-configuration
//! `.bit`, through `oxibit-fasm` against a prjxray database. Configuration
//! data is word-for-word what prjxray's `fasm2frames` + `xc7frames2bit`
//! produce; the `.bit` header carries no date or time, so the same FASM
//! always yields the same file.
//!
//! ## build-port
//!
//! `{:packet, 4}` framing, as `oxibit port`. The database is read once at
//! start. Request: FASM text. Reply: `0x00` + `.bit` bytes, or `0x01` +
//! UTF-8 error text. The port exits when stdin closes.

use std::io::{self, Read, Write};
use std::path::Path;
use std::process::ExitCode;
use std::time::Instant;

use oxibit_bitstream::{BitHeader, Bitstream, Container};
use oxibit_fasm::db::Database;

use crate::input::Result;
use crate::report::ReportArgs;
use oxibit_report::{Event, Progress, Reporter};

fn open(db_root: &Path, part: &str) -> Result<Database> {
    Database::open(db_root, part).map_err(|e| format!("database: {e}"))
}

pub(crate) fn to_bit(db: &Database, fasm: &str, design: &str) -> Result<Vec<u8>> {
    let words = oxibit_fasm::build(db, fasm).map_err(|e| e.to_string())?;
    let header = BitHeader {
        design: Some(format!("{design};Generator=oxibit")),
        part: Some(db.part.clone()),
        date: None,
        time: None,
    };
    let bs = Bitstream { container: Container::Bit, header: Some(header.clone()), words };
    bs.to_bit(&header).map_err(|e| format!("{e:?}"))
}

/// Exit status: 0 written; 1 the FASM does not assemble; 3 environment
/// (database, input, output). The status line on stderr is unchanged unless
/// `--progress json` or `none`; events go to `--events` as well.
pub fn fasm2bit(input: &Path, output: &Path, db_root: &Path, part: &str, frames: Option<&Path>, rep: &ReportArgs) -> Result<ExitCode> {
    let r = rep.reporter(false)?;
    r.emit(Event::Start { cmd: "fasm2bit".into(), version: env!("CARGO_PKG_VERSION").into(), args: std::env::args().skip(1).collect() });
    let fail = |code: u8, e: String| -> Result<ExitCode> {
        r.emit(Event::Warn { code: if code == 1 { "assemble".into() } else { "environment".into() }, msg: e.clone() });
        done(&r, false, code, serde_json::json!({}), vec![]);
        if rep.text() {
            eprintln!("oxibit: {e}");
        }
        Ok(ExitCode::from(code))
    };

    let t0 = Instant::now();
    r.begin("db");
    let db = match open(db_root, part) {
        Ok(db) => db,
        Err(e) => return fail(3, e),
    };
    r.end("db");
    let t1 = Instant::now();
    let fasm = match std::fs::read_to_string(input) {
        Ok(f) => f,
        Err(e) => return fail(3, format!("{}: {e}", input.display())),
    };

    r.begin("assemble");
    if let Some(frm) = frames {
        let f = match oxibit_fasm::assemble(&db, &fasm) {
            Ok(f) => f,
            Err(e) => return fail(1, e.to_string()),
        };
        if let Err(e) = std::fs::write(frm, oxibit_fasm::to_frm(&f)) {
            return fail(3, format!("{}: {e}", frm.display()));
        }
    }
    let design = input.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
    let bit = match to_bit(&db, &fasm, &design) {
        Ok(b) => b,
        Err(e) => return fail(1, e),
    };
    r.end("assemble");
    r.begin("write");
    if let Err(e) = std::fs::write(output, &bit) {
        return fail(3, format!("{}: {e}", output.display()));
    }
    r.end("write");
    if rep.text() {
        eprintln!(
            "oxibit: {} -> {} ({} bytes; database {} ms, build {} ms)",
            input.display(),
            output.display(),
            bit.len(),
            (t1 - t0).as_millis(),
            t1.elapsed().as_millis()
        );
    }
    done(&r, true, 0, serde_json::json!({"bytes": bit.len()}), vec![output.display().to_string()]);
    Ok(ExitCode::SUCCESS)
}

fn done(r: &Reporter, ok: bool, exit: u8, stats: serde_json::Value, outputs: Vec<String>) {
    r.emit(Event::Done { ok, exit: exit as i32, cancelled: false, phases: r.phases(), peak_rss_mb: None, stats, check: serde_json::json!({}), outputs });
    r.finish();
}

pub fn serve(db_root: &Path, part: &str) -> ExitCode {
    let db = match open(db_root, part) {
        Ok(db) => db,
        Err(e) => {
            eprintln!("oxibit build-port: {e}");
            return ExitCode::from(69);
        }
    };
    let mut stdin = io::stdin().lock();
    let mut stdout = io::stdout().lock();

    loop {
        let mut len = [0u8; 4];
        if stdin.read_exact(&mut len).is_err() {
            return ExitCode::SUCCESS; // the BEAM closed the port
        }
        let mut req = vec![0u8; u32::from_be_bytes(len) as usize];
        if stdin.read_exact(&mut req).is_err() {
            return ExitCode::from(70);
        }
        let reply = match std::str::from_utf8(&req).map_err(|e| e.to_string()).and_then(|f| to_bit(&db, f, "fabfarm")) {
            Ok(bit) => [vec![0x00], bit].concat(),
            Err(e) => [vec![0x01], e.into_bytes()].concat(),
        };
        let ok = stdout
            .write_all(&(reply.len() as u32).to_be_bytes())
            .and_then(|_| stdout.write_all(&reply))
            .and_then(|_| stdout.flush());
        if ok.is_err() {
            return ExitCode::from(70);
        }
    }
}
