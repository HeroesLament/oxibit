//! FASM -> Nexus bitstream.
//!
//!   cargo run --release -p oxibit-nexus --example nexus_pack -- \
//!       [--db <prjoxide/database>] [--compress] [--background] [--multiboot] \
//!       in.fasm out.bit
//!
//! The database defaults to $OXIBIT_PRJOXIDE_DB, then ~/src/prjoxide/database.

use std::path::PathBuf;
use std::process::ExitCode;

use oxibit_nexus::{Chip, DeviceDb, ParsedFasm, serialise};

pub fn default_db() -> PathBuf {
    if let Some(p) = std::env::var_os("OXIBIT_PRJOXIDE_DB") {
        return PathBuf::from(p);
    }
    let home = std::env::var_os("HOME").map(PathBuf::from).unwrap_or_default();
    home.join("src/prjoxide/database")
}

fn run() -> Result<(), String> {
    let mut db = default_db();
    let mut settings = oxibit_nexus::Settings::default();
    let mut files = Vec::new();
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--db" => db = PathBuf::from(args.next().ok_or("--db needs a directory")?),
            "--compress" => settings.compress = true,
            "--background" => settings.background = true,
            "--multiboot" => settings.multiboot = true,
            _ => files.push(a),
        }
    }
    let [fasm_path, out] = files.as_slice() else {
        return Err("usage: nexus_pack [--db DIR] [--compress] [--background] [--multiboot] in.fasm out.bit".into());
    };
    let text = std::fs::read_to_string(fasm_path).map_err(|e| format!("{fasm_path}: {e}"))?;
    let fasm = ParsedFasm::parse(&text).map_err(|e| format!("{fasm_path}: {e}"))?;
    let device = DeviceDb::device_of(&fasm).map_err(|e| e.to_string())?;
    let ddb = DeviceDb::open(&db, device).map_err(|e| e.to_string())?;
    let mut chip = Chip::from_fasm(&ddb, &fasm).map_err(|e| e.to_string())?;
    chip.settings = settings;
    let bits = serialise(&chip).map_err(|e| e.to_string())?;
    std::fs::write(out, &bits).map_err(|e| format!("{out}: {e}"))?;
    eprintln!("{out}: {} bytes, {device}", bits.len());
    Ok(())
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("nexus_pack: {e}");
            ExitCode::FAILURE
        }
    }
}
