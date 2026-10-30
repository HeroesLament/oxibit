//! Check `cellcfg::slice_config` against a nextpnr-xilinx FASM: every slice
//! configuration line (FF ZINI/ZRST, control set, LUT mode, PRECYINIT)
//! computed from the placed design's cells must be the line nextpnr wrote,
//! and none missing.
//!
//! cargo run --release -p oxibit-pnr --example cell_cfg_check -- \
//!     DB_ROOT PART DESIGN_routed.json DESIGN.fasm
//!
//! Prints one JSON line: counts, and per kind of line (its last part) the
//! lines only one side has, with examples. Two differences are explained,
//! counted apart, and do not fail the check:
//!
//! - `precyinit_const_via_ax`: nextpnr's `PRECYINIT.AX` where this writes
//!   `PRECYINIT.C0`/`C1` (a constant carry-in). nextpnr-himbaechel's xilinx
//!   FASM writer routes the constant in on AX (and can leave AX undriven);
//!   nextpnr-xilinx's `write_carry_config` selects C0/C1, as this does.
//! - `cellless_half`: a control-set line (`NOCLKINV`) on a half slice of a
//!   tile with no placed cell, which only nextpnr's routing uses (a
//!   route-through LUT).

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

use oxibit_fasm::db::Database;
use oxibit_pnr::cellcfg;
use oxibit_pnr::lutinit::Locator;
use serde_json::{json, Value};

type R<T> = Result<T, String>;

fn main() -> R<()> {
    let args: Vec<String> = std::env::args().collect();
    let [_, db_root, part, design, fasm] = &args[..] else {
        return Err("usage: cell_cfg_check DB_ROOT PART DESIGN.JSON DESIGN.FASM".into());
    };
    let db = Database::open(Path::new(db_root), part)?;
    let loc = Locator::new(&db)?;
    let text = std::fs::read_to_string(design).map_err(|e| format!("{design}: {e}"))?;
    let placed: Value = serde_json::from_str(&text).map_err(|e| format!("{design}: {e}"))?;
    let halves = cellcfg::slice_config(&placed, &|b| loc.locate(b))?;
    let our_tiles: BTreeSet<String> = halves.keys().map(|(t, _)| t.clone()).collect();
    let mut ours: BTreeSet<String> = halves.into_values().flatten().collect();
    ours.extend(cellcfg::dsp_ties(&placed, &|b| loc.locate(b))?);
    ours.extend(cellcfg::bufg_config(&placed, &|b| loc.locate(b))?);
    ours.extend(cellcfg::dsp_config(&placed, &|b| loc.locate(b))?);

    let fasm_text = std::fs::read_to_string(fasm).map_err(|e| format!("{fasm}: {e}"))?;
    let mut theirs: BTreeSet<String> = fasm_text
        .lines()
        .map(|l| l.split('#').next().unwrap_or("").trim())
        .filter(|l| cellcfg::is_slice_config(l) || cellcfg::is_dsp_tie(l) || cellcfg::is_bufg_config(l) || cellcfg::is_dsp_config(l))
        .map(String::from)
        .collect();

    // The explained differences.
    let mut via_ax = 0usize;
    let ax: Vec<String> = theirs.iter().filter(|l| l.ends_with(".PRECYINIT.AX")).cloned().collect();
    for l in ax {
        let stem = &l[..l.len() - 2];
        let ours_const: Vec<String> = ["C0", "C1"].iter().map(|c| format!("{stem}{c}")).filter(|c| ours.contains(c)).collect();
        if let [c] = &ours_const[..] {
            theirs.remove(&l);
            ours.remove(c);
            via_ax += 1;
        }
    }
    let cellless: Vec<String> = theirs.difference(&ours).filter(|l| !our_tiles.contains(l.split('.').next().unwrap_or(""))).cloned().collect();
    for l in &cellless {
        theirs.remove(l);
    }

    let kind = |l: &str| -> String {
        let p: Vec<&str> = l.split('.').collect();
        if cellcfg::is_dsp_tie(l) {
            return format!("DSP tie {}", &p[2][..7]);
        }
        if cellcfg::is_bufg_config(l) {
            return format!("BUFGCTRL.{}", p[3]);
        }
        if cellcfg::is_dsp_config(l) {
            let f = p[3].split(['[', ' ']).next().unwrap_or("");
            return format!("DSP48.{f}");
        }
        match p[2..] {
            [x] => x.to_string(),
            [a, b] if a == "PRECYINIT" => format!("PRECYINIT.{b}"),
            [a, b] => format!("{}.{b}", if a.ends_with("FF") { "xFF" } else { "xLUT" }),
            _ => l.to_string(),
        }
    };
    let mut by_kind: BTreeMap<String, (usize, Vec<String>, Vec<String>)> = BTreeMap::new();
    for l in ours.intersection(&theirs) {
        by_kind.entry(kind(l)).or_default().0 += 1;
    }
    for l in ours.difference(&theirs) {
        by_kind.entry(kind(l)).or_default().1.push(l.clone());
    }
    for l in theirs.difference(&ours) {
        by_kind.entry(kind(l)).or_default().2.push(l.clone());
    }
    let kinds: BTreeMap<String, Value> = by_kind
        .into_iter()
        .map(|(k, (same, o, t))| {
            (k, json!({"same": same, "only_ours": o.len(), "only_nextpnr": t.len(),
                       "ours_examples": o.iter().take(4).collect::<Vec<_>>(), "nextpnr_examples": t.iter().take(4).collect::<Vec<_>>()}))
        })
        .collect();
    let equal = ours == theirs;
    println!("{}", json!({"equal": equal, "ours": ours.len(), "nextpnr": theirs.len(),
                          "explained": {"precyinit_const_via_ax": via_ax, "cellless_half": cellless.len(), "cellless_examples": cellless.iter().take(4).collect::<Vec<_>>()},
                          "kinds": kinds}));
    if !equal {
        std::process::exit(1);
    }
    Ok(())
}
