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

pub fn fasm2bit(input: &Path, output: &Path, db_root: &Path, part: &str, frames: Option<&Path>) -> Result<ExitCode> {
    let t0 = Instant::now();
    let db = open(db_root, part)?;
    let t1 = Instant::now();
    let fasm = std::fs::read_to_string(input).map_err(|e| format!("{}: {e}", input.display()))?;

    if let Some(frm) = frames {
        let f = oxibit_fasm::assemble(&db, &fasm).map_err(|e| e.to_string())?;
        std::fs::write(frm, oxibit_fasm::to_frm(&f)).map_err(|e| format!("{}: {e}", frm.display()))?;
    }

    let design = input.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
    let bit = to_bit(&db, &fasm, &design)?;
    std::fs::write(output, &bit).map_err(|e| format!("{}: {e}", output.display()))?;
    eprintln!(
        "oxibit: {} -> {} ({} bytes; database {} ms, build {} ms)",
        input.display(),
        output.display(),
        bit.len(),
        (t1 - t0).as_millis(),
        t1.elapsed().as_millis()
    );
    Ok(ExitCode::SUCCESS)
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
