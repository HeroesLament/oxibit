//! Check `lutinit::cell_inits` against nextpnr: compute every LUT's INIT
//! from a placed (or routed) design's cells and compare with the INIT lines
//! of the FASM nextpnr wrote for it. Needs no routing graph.
//!
//! cargo run --release -p oxibit-pnr --example lut_init_check -- \
//!     DB_ROOT PART DESIGN.JSON DESIGN.FASM
//!
//! One JSON object on stdout: counts, and a few examples of each kind of
//! difference.

use std::collections::BTreeMap;
use std::path::Path;

use oxibit_fasm::db::Database;
use oxibit_pnr::lutinit::{self, LutKey};
use serde_json::{Value, json};

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().collect();
    let [_, db_root, part, design, fasm] = &args[..] else {
        return Err("usage: lut_init_check DB_ROOT PART DESIGN.JSON DESIGN.FASM".into());
    };
    let db = Database::open(Path::new(db_root), part)?;
    let loc = lutinit::Locator::new(&db)?;
    let locate = |bel: &str| loc.locate(bel);

    let text = std::fs::read_to_string(design).map_err(|e| format!("{design}: {e}"))?;
    let v: Value = serde_json::from_str(&text).map_err(|e| format!("{design}: {e}"))?;
    drop(text);
    let (ours, skipped) = lutinit::cell_inits(&v, &locate)?;
    let fasm_text = std::fs::read_to_string(fasm).map_err(|e| format!("{fasm}: {e}"))?;
    let theirs = lutinit::fasm_inits(&fasm_text)?;

    let mut counts: BTreeMap<&str, usize> = BTreeMap::new();
    let mut examples: BTreeMap<&str, Vec<Value>> = BTreeMap::new();
    let mut note = |kind: &'static str, ex: Value| {
        *counts.entry(kind).or_default() += 1;
        let e = examples.entry(kind).or_default();
        if e.len() < 5 {
            e.push(ex);
        }
    };
    let name = |k: &LutKey| format!("{}.{}.{}LUT", k.0, k.1, k.2);
    for (k, &a) in &ours {
        match theirs.get(k) {
            Some(&b) if a == b => note("match", Value::Null),
            Some(&b) => note(
                "differ",
                json!({"lut": name(k), "ours": format!("{a:016X}"), "nextpnr": format!("{b:016X}")}),
            ),
            None if a == 0 => note("zero_both", Value::Null),
            None => note(
                "only_ours",
                json!({"lut": name(k), "ours": format!("{a:016X}")}),
            ),
        }
    }
    for (k, &b) in &theirs {
        if !ours.contains_key(k) {
            note(
                "only_nextpnr",
                json!({"lut": name(k), "nextpnr": format!("{b:016X}")}),
            );
        }
    }
    examples.remove("match");
    examples.remove("zero_both");
    println!(
        "{}",
        json!({"design": design, "luts_ours": ours.len(), "luts_nextpnr": theirs.len(), "skipped": skipped, "counts": counts, "examples": examples})
    );
    Ok(())
}
