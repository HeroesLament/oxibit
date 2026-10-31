//! Route a placed design and write its FASM with oxibit alone: no FASM from
//! another tool. The input is a placed design JSON (nextpnr `--write` after
//! placement, e.g. `--no-route`); every line of the output is written by a
//! `flow::standalone` writer.
//!
//! cargo run --release -p oxibit-pnr --example standalone -- \
//!     DB_ROOT PART META_DIR PLACED.json OUT.fasm
//!
//! OXIBIT_DELAY_WEIGHT=w (0..=1): the router's `Opts::delay_weight`.
//! OXIBIT_BBOX_MARGIN=m: the router's `Opts::bbox_margin` (negative: no box).
//! OXIBIT_ASTAR=a: the router's `Estimate::per_tile`.
//! OXIBIT_PRES_FAC / OXIBIT_PRES_GROWTH / OXIBIT_PRES_MAX: the router's
//! `Schedule::Geometric` p0 / growth / cap. OXIBIT_HIST_FAC: history factor.
//! OXIBIT_ENTRY_BOUND=0|1: `Estimate::entry`. OXIBIT_OVERUSE_DUMP=path: the
//! nodes still overused after routing.
//!
//! stdout: one JSON line per phase. Exit 0 when the routing is complete
//! and clean and no two writers conflict; 1 otherwise; 3 on an error.

use std::io::Write;
use std::path::Path;
use std::process::ExitCode;
use std::sync::atomic::AtomicBool;

use oxibit_fasm::db::Database;
use oxibit_fasm::route::{Graph, GraphOpts};
use oxibit_pnr::flow;
use oxibit_pnr::router::Opts;
use oxibit_pnr::placed::{self, Meta, Perm};
use oxibit_report::{Human, HumanStyle, Reporter};
use serde_json::json;

static CANCEL: AtomicBool = AtomicBool::new(false);

fn main() -> ExitCode {
    let _prof = oxibit_prof::init();
    match run() {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::from(1),
        Err(e) => {
            eprintln!("standalone: {e}");
            ExitCode::from(3)
        }
    }
}

fn run() -> Result<bool, String> {
    let args: Vec<String> = std::env::args().collect();
    let [_, db_root, part, meta_dir, design, out_path] = &args[..] else {
        return Err("usage: standalone DB_ROOT PART META_DIR PLACED.json OUT.fasm".into());
    };
    let line = |v: serde_json::Value| {
        let mut o = std::io::stdout().lock();
        let _ = writeln!(o, "{v}");
        let _ = o.flush();
    };
    let r = Reporter::new(vec![Box::new(Human::new(std::io::stderr(), HumanStyle::Plain))], &CANCEL);
    let db = oxibit_prof::hot_block!("db.open", { Database::open(Path::new(db_root), part)? });
    let g = Graph::build_with(&db, None, GraphOpts { defaults: true })?;
    let placed_v: serde_json::Value = oxibit_prof::hot_block!("placed.read", {
        let text = std::fs::read_to_string(design).map_err(|e| format!("{design}: {e}"))?;
        serde_json::from_str(&text).map_err(|e| format!("{design}: {e}"))?
    });
    let mut meta = Meta::open(Path::new(meta_dir));
    // A placer leaves signals on A6 of fractured LUTs; routing moves them.
    let imp = placed::import(&g, &placed::site_index(&db), &mut meta, &placed_v, Perm::A5)?;
    line(json!({"import": {"nets": imp.nets.len(), "fixed_sinks": imp.fixed_sinks, "internal_sinks": imp.internal_sinks,
                           "skipped": imp.skipped, "unresolved": imp.unresolved.counts, "unresolved_examples": imp.unresolved.examples}}));
    let mut opts = Opts::default();
    if let Some(w) = std::env::var("OXIBIT_DELAY_WEIGHT").ok().and_then(|w| w.parse().ok()) {
        opts.delay_weight = w;
    }
    if let Some(m) = std::env::var("OXIBIT_BBOX_MARGIN").ok().and_then(|m| m.parse().ok()) {
        opts.bbox_margin = m;
    }
    if let Some(a) = std::env::var("OXIBIT_ASTAR").ok().and_then(|a| a.parse().ok()) {
        opts.estimate.per_tile = a;
    }
    if let Ok(v) = std::env::var("OXIBIT_ENTRY_AIM") {
        opts.entry_aim = v != "0";
    }
    if let Some(n) = std::env::var("OXIBIT_MAX_ITERS").ok().and_then(|n| n.parse().ok()) {
        opts.max_iters = n;
    }
    let envf = |k: &str| std::env::var(k).ok().and_then(|v| v.parse::<f32>().ok());
    let oxibit_pnr::router::Schedule::Geometric { mut p0, mut growth, mut cap } = opts.schedule;
    p0 = envf("OXIBIT_PRES_FAC").unwrap_or(p0);
    growth = envf("OXIBIT_PRES_GROWTH").unwrap_or(growth);
    cap = envf("OXIBIT_PRES_MAX").unwrap_or(cap);
    opts.schedule = oxibit_pnr::router::Schedule::Geometric { p0, growth, cap };
    if let Some(v) = envf("OXIBIT_HIST_FAC") {
        opts.hist_fac = v;
    }
    opts.overuse_dump = std::env::var("OXIBIT_OVERUSE_DUMP").ok();
    if let Ok(v) = std::env::var("OXIBIT_ENTRY_BOUND") {
        opts.estimate.entry = v != "0";
    }
    let sa = flow::standalone_with(&db, &g, &placed_v, &imp, "", opts, &r)?;
    for v in &sa.report {
        line(v.clone());
    }
    // Each short node, and the nets whose routes touch it.
    let shorts: Vec<serde_json::Value> = sa
        .check
        .shorts
        .iter()
        .take(10)
        .map(|&n| {
            let on: Vec<String> = sa
                .nets
                .iter()
                .zip(&sa.routes)
                .filter(|(net, hops)| net.source == n || net.sinks.contains(&n) || hops.iter().any(|h| h.src == n || h.dst == n))
                .map(|(net, _)| g.node_name(net.source))
                .collect();
            json!({"node": g.node_name(n), "nets": on})
        })
        .collect();
    if !shorts.is_empty() {
        line(json!({ "shorts": shorts }));
    }
    std::fs::write(out_path, sa.fasm.text()).map_err(|e| format!("{out_path}: {e}"))?;
    let ok = sa.ok() && imp.unresolved.counts.is_empty();
    line(json!({"ok": ok, "out": out_path}));
    Ok(ok)
}
