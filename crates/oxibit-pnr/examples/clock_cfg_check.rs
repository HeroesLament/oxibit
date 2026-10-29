//! Check `clocking::clock_enables` against a nextpnr FASM: the clock
//! enables computed from the FASM's own pips must be exactly the enable
//! lines nextpnr wrote.
//!
//! cargo run --release -p oxibit-pnr --example clock_cfg_check -- \
//!     DB_ROOT PART DESIGN.fasm
//!
//! Prints one JSON line: counts, and the lines only one side has.

use std::collections::BTreeSet;
use std::path::Path;

use oxibit_fasm::db::Database;
use oxibit_pnr::clocking;
use oxibit_pnr::lutinit::Locator;
use serde_json::json;

type R<T> = Result<T, String>;

fn main() -> R<()> {
    let args: Vec<String> = std::env::args().collect();
    let [_, db_root, part, fasm] = &args[..] else {
        return Err("usage: clock_cfg_check DB_ROOT PART DESIGN.FASM".into());
    };
    let db = Database::open(Path::new(db_root), part)?;
    let loc = Locator::new(&db)?;
    let text = std::fs::read_to_string(fasm).map_err(|e| format!("{fasm}: {e}"))?;
    let ours = clocking::clock_enables(clocking::fasm_pips(&text, &loc), &loc);
    let theirs: BTreeSet<String> = text
        .lines()
        .map(|l| l.split('#').next().unwrap_or("").trim())
        .filter(|l| clocking::is_clock_enable(l, &loc))
        .map(String::from)
        .collect();
    let only_ours: Vec<&String> = ours.difference(&theirs).collect();
    let only_theirs: Vec<&String> = theirs.difference(&ours).collect();
    let equal = only_ours.is_empty() && only_theirs.is_empty();
    println!("{}", json!({"equal": equal, "ours": ours.len(), "nextpnr": theirs.len(),
                          "only_ours": only_ours.len(), "only_nextpnr": only_theirs.len(),
                          "ours_examples": only_ours.iter().take(6).collect::<Vec<_>>(),
                          "nextpnr_examples": only_theirs.iter().take(6).collect::<Vec<_>>()}));
    if !equal {
        std::process::exit(1);
    }
    Ok(())
}
