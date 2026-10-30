//! A bitstream against the FASM it was built from, read back through the
//! segbits database:
//!
//! - frames: every frame of the `.bit` equals the frame the assembler makes
//!   from the FASM (ECC recomputed), and the pad frames between rows are
//!   zero;
//! - features: decoding the `.bit`'s frames (`disassemble::decode`) gives
//!   exactly the features the FASM enabled. `implied` is a feature the bits
//!   enable that the FASM did not set (the silicon sees it: its bits are a
//!   subset of the set ones'), `lost` one the FASM set that does not read
//!   back;
//! - `unclaimed`: set bits no decoded feature sets.
//!
//! cargo run --release -p oxibit-fasm --example bit_roundtrip -- DB_ROOT PART DESIGN.fasm DESIGN.bit
//!
//! stdout: one JSON line. Exit 0 when all three are clean, 1 otherwise, 3 on
//! an error.

use std::collections::{BTreeMap, HashSet};
use std::path::Path;
use std::process::ExitCode;

use oxibit_bitstream::{Bitstream, FRAME_WORDS, ecc::update_ecc, reader::read_frames};
use oxibit_fasm::db::Database;
use oxibit_fasm::{assemble_named, disassemble};
use serde_json::json;

fn main() -> ExitCode {
    match run() {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::from(1),
        Err(e) => {
            eprintln!("bit_roundtrip: {e}");
            ExitCode::from(3)
        }
    }
}

/// `CLBLL_L_X2Y3.SLICEL_X0.ALUT.INIT[05]` -> `CLBLL_L.SLICEL_X0.ALUT.INIT[#]`:
/// the tile's type for its name, digits in brackets folded.
fn kind(db: &Database, f: &str) -> String {
    let (tile, rest) = f.split_once('.').unwrap_or((f, ""));
    let tt = db.tiles.get(tile).map(|t| t.tile_type.as_str()).unwrap_or("?");
    let mut out = String::new();
    let mut in_br = false;
    for c in rest.chars() {
        match c {
            '[' => in_br = true,
            ']' => in_br = false,
            d if in_br && d.is_ascii_digit() => {
                if !out.ends_with('#') {
                    out.push('#');
                }
                continue;
            }
            _ => {}
        }
        out.push(c);
    }
    format!("{tt}.{out}")
}

fn run() -> Result<bool, String> {
    let args: Vec<String> = std::env::args().collect();
    let [_, db_root, part, fasm_path, bit_path] = &args[..] else {
        return Err("usage: bit_roundtrip DB_ROOT PART DESIGN.fasm DESIGN.bit".into());
    };
    let db = Database::open(Path::new(db_root), part)?;
    let fasm = std::fs::read_to_string(fasm_path).map_err(|e| format!("{fasm_path}: {e}"))?;
    let (mut want, named) = assemble_named(&db, &fasm).map_err(|e| e.to_string())?;
    for f in want.values_mut() {
        update_ecc(f);
    }
    let data = std::fs::read(bit_path).map_err(|e| format!("{bit_path}: {e}"))?;
    let bs = Bitstream::parse(&data).map_err(|e| format!("{bit_path}: {e:?}"))?;
    let got = read_frames(&bs.words, &db.layout)?;

    // Frames.
    let zero = [0u32; FRAME_WORDS];
    let (mut frames_differ, mut words_differ, mut examples) = (0usize, 0usize, Vec::new());
    let all = db.layout.all();
    for a in &all {
        let w = want.get(a).unwrap_or(&zero);
        let g = got.frames.get(a).unwrap_or(&zero);
        let n = w.iter().zip(g).filter(|(x, y)| x != y).count();
        if n > 0 {
            frames_differ += 1;
            words_differ += n;
            if examples.len() < 6 {
                let i = w.iter().zip(g).position(|(x, y)| x != y).unwrap();
                examples.push(format!("{:#010X} word {i}: fasm {:#010X} bit {:#010X}", a.0, w[i], g[i]));
            }
        }
    }
    let in_layout: HashSet<_> = all.iter().collect();
    let extra_frames: Vec<String> = got.frames.keys().filter(|a| !in_layout.contains(a)).take(6).map(|a| format!("{:#010X}", a.0)).collect();

    // Features.
    let dec = disassemble::decode(&db, &got.frames);
    let lost: Vec<&String> = named.difference(&dec.features).collect();
    let implied: Vec<&String> = dec.features.difference(&named).collect();
    let mut implied_kinds: BTreeMap<String, usize> = BTreeMap::new();
    for f in &implied {
        *implied_kinds.entry(kind(&db, f)).or_default() += 1;
    }
    let mut lost_kinds: BTreeMap<String, usize> = BTreeMap::new();
    for f in &lost {
        *lost_kinds.entry(kind(&db, f)).or_default() += 1;
    }
    let unclaimed: Vec<String> = dec
        .unclaimed
        .iter()
        .take(12)
        .map(|&b| format!("{:#010X} word {} bit {} ({})", b.0, b.1, b.2, disassemble::tile_of(&db, b).unwrap_or_default()))
        .collect();

    let ok = frames_differ == 0 && extra_frames.is_empty() && got.nonzero_pads == 0 && lost.is_empty() && implied.is_empty() && dec.unclaimed.is_empty();
    println!(
        "{}",
        json!({
            "ok": ok,
            "frames": {"read": got.frames.len(), "layout": all.len(), "differ": frames_differ, "words_differ": words_differ,
                       "examples": examples, "not_in_layout": extra_frames, "pad_frames": got.pad_frames, "nonzero_pads": got.nonzero_pads},
            "features": {"fasm": named.len(), "decoded": dec.features.len(),
                         "lost": lost.len(), "lost_kinds": lost_kinds, "lost_examples": lost.iter().take(8).collect::<Vec<_>>(),
                         "implied": implied.len(), "implied_kinds": implied_kinds, "implied_examples": implied.iter().take(8).collect::<Vec<_>>()},
            "unclaimed": {"bits": dec.unclaimed.len(), "examples": unclaimed},
        })
    );
    Ok(ok)
}
