//! ECP5 bitstream -> Trellis `.config`, the oxibit counterpart of `ecpunpack`.
//!
//!   cargo run --release -p oxibit-ecp5 --example ecp5_unpack -- \
//!       [--db <trellis/database>] in.bit out.config
//!
//! The device is found by the bitstream's IDCODE. The database defaults to
//! $OXIBIT_TRELLIS_DB, then ~/oss-cad-suite/share/trellis/database.

#![expect(
    clippy::print_stderr,
    reason = "a command-line example reports its errors on stderr"
)]

use std::path::PathBuf;
use std::process::ExitCode;

fn default_db() -> PathBuf {
    if let Some(p) = std::env::var_os("OXIBIT_TRELLIS_DB") {
        return PathBuf::from(p);
    }
    let home = std::env::var_os("HOME")
        .map(PathBuf::from)
        .unwrap_or_default();
    home.join("oss-cad-suite/share/trellis/database")
}

fn run() -> Result<(), String> {
    let mut db = default_db();
    let mut files = Vec::new();
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--db" => db = PathBuf::from(args.next().ok_or("--db needs a directory")?),
            _ => files.push(a),
        }
    }
    let [bit, out] = files.as_slice() else {
        return Err("usage: ecp5_unpack [--db DIR] in.bit out.config".into());
    };
    let bytes = std::fs::read(bit).map_err(|e| format!("{bit}: {e}"))?;
    let cc = oxibit_ecp5::unpack(&db, &bytes).map_err(|e| format!("{bit}: {e}"))?;
    std::fs::write(out, cc.to_text()).map_err(|e| format!("{out}: {e}"))
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("ecp5_unpack: {e}");
            ExitCode::FAILURE
        }
    }
}
