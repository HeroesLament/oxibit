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
//! OXIBIT_ASTAR_DELAY, OXIBIT_DELAY_NORM: `Opts::astar_delay`, `delay_norm`.
//! OXIBIT_SPAN_AWARE=0|1: `Opts::span_aware`. OXIBIT_SKIP_STALE=0|1:
//! `Opts::skip_stale`. OXIBIT_DETOUR=FACTOR,MARGIN: `Opts::detour`.
//! OXIBIT_BIDIR=0|1: `Opts::bidir`. OXIBIT_CLEANUP=N: `Opts::cleanup_rounds`.
//! OXIBIT_ENTRY_BOUND=0|1: `Estimate::entry`. OXIBIT_OVERUSE_DUMP=path: the
//! nodes still overused after routing.
//!
//! OXIBIT_PERM=none|a5|a6 (default a5): which LUT inputs the router may
//! move between (`placed::Perm`); `none` keeps every LUT pin as placed, so
//! LUT INITs stay the placer's. OXIBIT_ROUTES_DUMP=path: per routed net,
//! one JSON line `{"net", "class", "source", "lines"}` with the FASM lines
//! of its pips (for splicing one tool's routes into another's FASM).
//!
//! OXIBIT_TIMING="pb_clk=50000,axi_clk=8000" (clock periods in ps, `*` for
//! the rest): route for timing (`Opts::timing`). OXIBIT_TAU, OXIBIT_DISCOUNT, OXIBIT_EXACT,
//! OXIBIT_HOT_MAX, OXIBIT_MIN_RISE, OXIBIT_TIMING_PASSES: the
//! `sta::TimingView` fields. With or without it, the routed design's
//! slowest path per clock pair is reported (`timing` lines;
//! OXIBIT_TIMING_STEPS=1 adds each path's steps), and each clock's
//! tightest same-clock hold check at the fast corner (`hold` lines).
//!
//! stdout: one JSON line per phase. Exit 0 when the routing is complete
//! and clean, no two writers conflict and the import resolved every pin;
//! 1 when the routing is not (overuse, missing sinks, shorts, writer
//! conflicts); 2 when it is, but the import left pins unresolved (the
//! `import` line's `unresolved`: those connections are not in the FASM);
//! 3 on an error.

use std::io::Write;
use std::path::Path;
use std::process::ExitCode;
use std::sync::atomic::AtomicBool;

use oxibit_fasm::db::Database;
use oxibit_fasm::route::{Graph, GraphOpts};
use oxibit_pnr::flow;
use oxibit_pnr::delay::Model;
use oxibit_pnr::router::Opts;
use oxibit_pnr::placed::{self, Meta, Perm};
use oxibit_pnr::sta::{self, TimingSource};
use oxibit_report::{Human, HumanStyle, Reporter};
use serde_json::json;

static CANCEL: AtomicBool = AtomicBool::new(false);

/// Hold checks with less slack than this are counted (`hold` lines): a
/// budget for the clock skew `sta::Timer::hold` does not model.
const HOLD_MARGIN_PS: i64 = 300;

/// How a run ended, short of an error.
enum Outcome {
    Clean,
    RouteFailed,
    Unresolved,
}

fn main() -> ExitCode {
    let _prof = oxibit_prof::init();
    match run() {
        Ok(Outcome::Clean) => ExitCode::SUCCESS,
        Ok(Outcome::RouteFailed) => ExitCode::from(1),
        Ok(Outcome::Unresolved) => ExitCode::from(2),
        Err(e) => {
            eprintln!("standalone: {e}");
            ExitCode::from(3)
        }
    }
}

fn run() -> Result<Outcome, String> {
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
    let sites = placed::site_index(&db);
    let perm = match std::env::var("OXIBIT_PERM").as_deref() {
        Ok("none") => Perm::None,
        Ok("a6") => Perm::A6,
        Ok("a5") | Err(_) => Perm::A5,
        Ok(other) => return Err(format!("OXIBIT_PERM={other}: expected none, a5 or a6")),
    };
    let imp = placed::import(&g, &sites, &mut meta, &placed_v, perm)?;
    line(json!({"import": {"nets": imp.nets.len(), "fixed_sinks": imp.fixed_sinks, "internal_sinks": imp.internal_sinks,
                           "skipped": imp.skipped, "const_internal": imp.const_internal, "const_empty_lut": imp.const_empty_lut,
                           "unresolved": imp.unresolved.counts, "unresolved_examples": imp.unresolved.examples}}));
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
    // OXIBIT_ENDGAME=OVERUSE,RADIUS or off: the router's `Opts::endgame`.
    if let Ok(v) = std::env::var("OXIBIT_ENDGAME") {
        opts.endgame = match v.split_once(',') {
            Some((o, r)) => Some(oxibit_pnr::router::Endgame {
                overuse: o.trim().parse().map_err(|_| format!("OXIBIT_ENDGAME: {o} is not a count"))?,
                radius: r.trim().parse().map_err(|_| format!("OXIBIT_ENDGAME: {r} is not a radius"))?,
            }),
            None if v == "off" => None,
            None => return Err(format!("OXIBIT_ENDGAME={v}: want OVERUSE,RADIUS or off")),
        };
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
    if let Ok(v) = std::env::var("OXIBIT_SPAN_AWARE") {
        opts.span_aware = v != "0";
    }
    if let Ok(v) = std::env::var("OXIBIT_SKIP_STALE") {
        opts.skip_stale = v != "0";
    }
    if let Ok(v) = std::env::var("OXIBIT_BIDIR") {
        opts.bidir = v != "0";
    }
    if let Some(n) = std::env::var("OXIBIT_CLEANUP").ok().and_then(|n| n.parse().ok()) {
        opts.cleanup_rounds = n;
    }
    // OXIBIT_HOLD_FIX=PS: pad block RAM captures up to this hold slack.
    if let Some(n) = std::env::var("OXIBIT_HOLD_FIX").ok().and_then(|n| n.parse().ok()) {
        opts.hold_fix_ps = Some(n);
    }
    // OXIBIT_HOLD_BUDGET=S: seconds the hold pass may take (default 300).
    if let Some(s) = std::env::var("OXIBIT_HOLD_BUDGET").ok().and_then(|s| s.parse().ok()) {
        opts.hold_budget_s = s;
    }
    if let Ok(v) = std::env::var("OXIBIT_DETOUR") {
        let (f, m) = v.split_once(',').ok_or("OXIBIT_DETOUR: want FACTOR,MARGIN")?;
        let num = |s: &str| s.trim().parse::<f32>().map_err(|_| format!("OXIBIT_DETOUR: {s} is not a number"));
        opts.detour = Some((num(f)?, num(m)?));
    }
    if let Some(v) = envf("OXIBIT_ASTAR_DELAY") {
        opts.astar_delay = v;
    }
    if let Some(v) = envf("OXIBIT_DELAY_NORM") {
        opts.delay_norm = v;
    }
    opts.overuse_dump = std::env::var("OXIBIT_OVERUSE_DUMP").ok();
    opts.hist_dump = std::env::var("OXIBIT_HIST_DUMP").ok();
    if let Ok(v) = std::env::var("OXIBIT_ENTRY_BOUND") {
        opts.estimate.entry = v != "0";
    }
    // Timing: always measured after routing; steers it when periods are given.
    let cons = match std::env::var("OXIBIT_TIMING") {
        Ok(s) => Some(sta::Constraints::parse(&s)?),
        Err(_) => None,
    };
    let design = sta::Design::build(&g, &sites, &placed_v, &imp)?;
    line(json!({"timing_graph": {"cells": design.cell_counts(), "ignored": design.ignored, "clocks": design.clocks(),
                                 "in_loops": design.in_loops, "unplaced": design.unplaced}}));
    let timer = std::rc::Rc::new(std::cell::RefCell::new(sta::Timer::new(design, &cons.clone().unwrap_or_default(), Model::Intrinsic)));
    if cons.is_some() {
        let mut view = sta::TimingView::new(timer.clone());
        view.tau = envf("OXIBIT_TAU").unwrap_or(view.tau);
        view.discount = envf("OXIBIT_DISCOUNT").unwrap_or(view.discount);
        view.exact = envf("OXIBIT_EXACT").unwrap_or(view.exact);
        view.min_rise = envf("OXIBIT_MIN_RISE").unwrap_or(view.min_rise);
        view.hot_max = std::env::var("OXIBIT_HOT_MAX").ok().and_then(|v| v.parse().ok()).unwrap_or(view.hot_max);
        view.passes = std::env::var("OXIBIT_TIMING_PASSES").ok().and_then(|v| v.parse().ok()).unwrap_or(view.passes);
        line(json!({"timing_view": format!("{view:?}")}));
        opts.timing = Some(view);
    }
    let sa = flow::standalone_with(&db, &g, &placed_v, &imp, "", opts, &r)?;
    for v in &sa.report {
        line(v.clone());
    }
    {
        let mut t = timer.borrow_mut();
        let all: Vec<usize> = (0..sa.nets.len()).collect();
        let steps = std::env::var("OXIBIT_TIMING_STEPS").is_ok();
        let table = t.update(&g, &sa.nets, &sa.routes, &all);
        for p in &table.paths {
            line(json!({"timing": p.to_json(steps)}));
        }
        let summary = table.summary();
        let arcs = t.arc_counts();
        line(json!({"timing_summary": summary, "arcs": {"timed": arcs.timed, "unknown": arcs.unknown, "unreached": arcs.unreached}}));
        // Hold, fast corner, no skew; counted against a 300 ps skew budget.
        for h in t.hold(oxibit_pnr::delay::HOLD_ROUTE_SCALE, HOLD_MARGIN_PS) {
            line(json!({"hold": h.to_json(), "margin_ns": HOLD_MARGIN_PS as f64 / 1000.0}));
        }
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
    if let Ok(path) = std::env::var("OXIBIT_ROUTES_DUMP") {
        let mut f = std::io::BufWriter::new(std::fs::File::create(&path).map_err(|e| format!("{path}: {e}"))?);
        for (k, (net, hops)) in sa.nets.iter().zip(&sa.routes).enumerate() {
            let lines: Vec<String> = hops.iter().flat_map(|h| g.edge_pip(h.tile, h.pi).fasm_lines()).collect();
            let name = imp.names.get(k).cloned().unwrap_or_default();
            let v = json!({"net": name, "class": format!("{:?}", net.class), "source": g.node_name(net.source), "lines": lines});
            writeln!(f, "{v}").map_err(|e| format!("{path}: {e}"))?;
        }
    }
    std::fs::write(out_path, sa.fasm.text()).map_err(|e| format!("{out_path}: {e}"))?;
    let (routed, resolved) = (sa.ok(), imp.unresolved.counts.is_empty());
    line(json!({"ok": routed && resolved, "routed": routed, "resolved": resolved, "out": out_path}));
    Ok(match (routed, resolved) {
        (false, _) => Outcome::RouteFailed,
        (true, false) => Outcome::Unresolved,
        (true, true) => Outcome::Clean,
    })
}
