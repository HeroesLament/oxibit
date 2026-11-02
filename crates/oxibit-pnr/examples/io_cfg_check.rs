//! Check `iocfg` and `bramcfg` against nextpnr: the pad, IOLOGIC,
//! HCLK_IOI3 bank and block RAM lines written from a routed design JSON,
//! against those lines of nextpnr's FASM for it (`iocfg::is_io_config`,
//! `bramcfg::is_bram_config`).
//!
//! cargo run --release -p oxibit-pnr --example io_cfg_check -- \
//!     DB_ROOT PART ROUTED.json NEXTPNR.fasm
//!
//! nextpnr's lines that only its routing writes (the I/O pseudo-pips' bits,
//! `route::io_pseudo`) are counted apart as `routing_only`; every other
//! difference is listed. Block RAM lines are a second JSON object
//! (`bram`). Exit 0 when the only differences are routing-only lines; 1
//! otherwise; 3 on an error.

use std::collections::BTreeSet;
use std::path::Path;
use std::process::ExitCode;

use oxibit_fasm::db::Database;
use oxibit_fasm::route::{Graph, GraphOpts};
use oxibit_pnr::{bramcfg, iocfg};
use oxibit_pnr::lutinit::Locator;
use serde_json::json;

fn main() -> ExitCode {
    match run() {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::from(1),
        Err(e) => {
            eprintln!("io_cfg_check: {e}");
            ExitCode::from(3)
        }
    }
}

/// A setting a routed I/O pseudo-pip makes (nextpnr's pseudo-pip table).
fn routing_only(line: &str) -> bool {
    let rest: Vec<&str> = line.splitn(3, '.').collect();
    let [tile, _, setting] = rest[..] else { return false };
    tile.contains("IOI") && matches!(setting, "OMUX.D1" | "OQUSED" | "OSERDES.DATA_RATE_TQ.BUF" | "IDELAY_TYPE_FIXED" | "ZINV_D" | "IDELMUXE3.P0" | "ZINV_T1")
}

fn run() -> Result<bool, String> {
    let args: Vec<String> = std::env::args().collect();
    let [_, db_root, part, design, np] = &args[..] else {
        return Err("usage: io_cfg_check DB_ROOT PART ROUTED.json NEXTPNR.fasm".into());
    };
    let db = Database::open(Path::new(db_root), part)?;
    let g = Graph::build_with(&db, None, GraphOpts { defaults: false })?;
    let loc = Locator::new(&db)?;
    let text = std::fs::read_to_string(design).map_err(|e| format!("{design}: {e}"))?;
    let placed: serde_json::Value = serde_json::from_str(&text).map_err(|e| format!("{design}: {e}"))?;
    let np_text = std::fs::read_to_string(np).map_err(|e| format!("{np}: {e}"))?;

    let io = iocfg::io_config(&placed, &loc, &g)?;
    let mut ours: BTreeSet<String> = io.lines.clone();
    // The bypass inversion, from nextpnr's own pips.
    let pips: Vec<(String, String)> = np_text
        .lines()
        .map(|l| l.split('#').next().unwrap_or("").trim())
        .filter(|l| !l.is_empty() && !l.contains('='))
        .filter_map(|l| {
            let p: Vec<&str> = l.split('.').collect();
            (p.len() == 3).then(|| (p[0].to_string(), p[1].to_string()))
        })
        .collect();
    ours.extend(iocfg::ilogic_bypass_inversion(pips.iter().map(|(t, d)| (t.as_str(), d.as_str()))));

    let theirs: BTreeSet<String> = np_text
        .lines()
        .map(|l| l.split('#').next().unwrap_or("").trim().to_string())
        .filter(|l| !l.is_empty() && iocfg::is_io_config(l))
        .collect();
    let only_ours: Vec<&String> = ours.difference(&theirs).collect();
    let only_theirs: Vec<&String> = theirs.difference(&ours).collect();
    let (routing, other): (Vec<&String>, Vec<&String>) = only_theirs.into_iter().partition(|l| routing_only(l));
    let same = ours.intersection(&theirs).count();
    println!(
        "{}",
        json!({"design": design, "ours": ours.len(), "theirs": theirs.len(), "same": same,
               "only_ours": only_ours.len(), "only_theirs": other.len(), "routing_only": routing.len(),
               "top_sing": io.top_sing.len(), "ologic_registered": io.ologic_registered.len(),
               "only_ours_lines": only_ours.iter().take(40).collect::<Vec<_>>(),
               "only_theirs_lines": other.iter().take(40).collect::<Vec<_>>(),
               "routing_only_lines": routing.iter().take(12).collect::<Vec<_>>()})
    );

    // Block RAM, with the cascade bits from nextpnr's pips.
    let mut bram = bramcfg::bram_config(&placed, &|b| loc.locate(b))?;
    bram.extend(bramcfg::cascade_lines(pips.iter().map(|(t, d)| (t.as_str(), d.as_str()))));
    let theirs_b: BTreeSet<String> = np_text
        .lines()
        .map(|l| l.split('#').next().unwrap_or("").trim().to_string())
        .filter(|l| !l.is_empty() && bramcfg::is_bram_config(l))
        .collect();
    let ob: Vec<&String> = bram.difference(&theirs_b).collect();
    let tb: Vec<&String> = theirs_b.difference(&bram).collect();
    let short = |v: &[&String]| v.iter().take(30).map(|l| l.chars().take(120).collect::<String>()).collect::<Vec<_>>();
    println!(
        "{}",
        json!({"bram": {"ours": bram.len(), "theirs": theirs_b.len(), "same": bram.intersection(&theirs_b).count(),
                        "only_ours": ob.len(), "only_theirs": tb.len(),
                        "only_ours_lines": short(&ob), "only_theirs_lines": short(&tb)}})
    );
    Ok(only_ours.is_empty() && other.is_empty() && ob.is_empty() && tb.is_empty())
}
