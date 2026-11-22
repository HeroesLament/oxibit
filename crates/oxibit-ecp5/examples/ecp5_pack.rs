//! Trellis `.config` -> ECP5 bitstream, the oxibit counterpart of `ecppack`.
//!
//!   cargo run --release -p oxibit-ecp5 --example ecp5_pack -- \
//!       [--db <trellis/database>] [--freq MHZ] [--spimode MODE] \
//!       [--usercode N] [--idcode N] in.config out.bit
//!
//! The database defaults to $OXIBIT_TRELLIS_DB, then
//! ~/oss-cad-suite/share/trellis/database.

#![expect(
    clippy::print_stderr,
    reason = "a command-line example reports its errors on stderr"
)]
use std::path::PathBuf;
use std::process::ExitCode;

use oxibit_ecp5::{Chip, ChipConfig, DeviceDb, Options, serialise};

fn default_db() -> PathBuf {
    if let Some(p) = std::env::var_os("OXIBIT_TRELLIS_DB") {
        return PathBuf::from(p);
    }
    let home = std::env::var_os("HOME")
        .map(PathBuf::from)
        .unwrap_or_default();
    home.join("oss-cad-suite/share/trellis/database")
}

fn parse_u32(s: &str) -> Result<u32, String> {
    let r = match s.strip_prefix("0x") {
        Some(h) => u32::from_str_radix(h, 16),
        None => s.parse(),
    };
    r.map_err(|e| format!("{s:?}: {e}"))
}

fn run() -> Result<(), String> {
    let mut db = default_db();
    let mut over = Options::default();
    let mut files = Vec::new();
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        let mut val = |flag: &str| args.next().ok_or_else(|| format!("{flag} needs a value"));
        match a.as_str() {
            "--db" => db = PathBuf::from(val("--db")?),
            "--freq" => over.freq = Some(val("--freq")?),
            "--spimode" => over.spimode = Some(val("--spimode")?),
            "--usercode" => over.usercode = Some(parse_u32(&val("--usercode")?)?),
            "--idcode" => over.idcode = Some(parse_u32(&val("--idcode")?)?),
            "--compress" => over.compress = true,
            _ => files.push(a),
        }
    }
    let [cfg_path, out] = files.as_slice() else {
        return Err("usage: ecp5_pack [--db DIR] [--freq MHZ] [--spimode MODE] [--usercode N] [--idcode N] in.config out.bit".into());
    };
    let text = std::fs::read_to_string(cfg_path).map_err(|e| format!("{cfg_path}: {e}"))?;
    let cc = ChipConfig::parse(&text).map_err(|e| format!("{cfg_path}: {e}"))?;
    let ddb = DeviceDb::load(&db, &cc.device).map_err(|e| e.to_string())?;
    let chip = Chip::from_config(&ddb, &cc).map_err(|e| e.to_string())?;
    let opts = Options::from_sysconfig(&cc.sysconfig, &over);
    let bits = serialise(&chip, &opts).map_err(|e| e.to_string())?;
    std::fs::write(out, bits).map_err(|e| format!("{out}: {e}"))
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("ecp5_pack: {e}");
            ExitCode::FAILURE
        }
    }
}
