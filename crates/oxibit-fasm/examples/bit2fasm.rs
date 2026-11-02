//! The features a bitstream's frames set, decoded through the segbits
//! database (`disassemble::decode`), one per line, sorted.
//!
//!   bit2fasm DB_ROOT PART DESIGN.bit|.bin

use std::path::Path;
use std::process::ExitCode;

use oxibit_bitstream::{Bitstream, reader::read_frames};
use oxibit_fasm::db::Database;
use oxibit_fasm::disassemble;

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("bit2fasm: {e}");
            ExitCode::from(3)
        }
    }
}

fn run() -> Result<(), String> {
    let args: Vec<String> = std::env::args().collect();
    let [_, db_root, part, bit_path] = &args[..] else {
        return Err("usage: bit2fasm DB_ROOT PART DESIGN.bit".into());
    };
    let db = Database::open(Path::new(db_root), part)?;
    let data = std::fs::read(bit_path).map_err(|e| format!("{bit_path}: {e}"))?;
    let bs = Bitstream::parse(&data).map_err(|e| format!("{bit_path}: {e:?}"))?;
    let got = read_frames(&bs.words, &db.layout)?;
    let dec = disassemble::decode(&db, &got.frames);
    let mut f: Vec<&String> = dec.features.iter().collect();
    f.sort();
    for l in f {
        println!("{l}");
    }
    Ok(())
}
