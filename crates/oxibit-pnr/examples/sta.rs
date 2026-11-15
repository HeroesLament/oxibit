//! Setup timing of a routed design's FASM (`oxibit_pnr::sta`): the slowest
//! register-to-register path of every clock pair.
//!
//! cargo run --release -p oxibit-pnr --example sta -- \
//!     DB_ROOT PART META_DIR PLACED.json ROUTED.fasm [a5|none]
//!
//! PLACED.json is the placed design the FASM was routed from (`a5`: routed
//! by oxibit, LUT inputs free to move; `none`: another tool's routing of
//! its own placement, pins as placed). `standalone` prints the same lines
//! for its own routes; this is for a FASM from elsewhere.
//!
//! DELAY_MODEL=intrinsic (default) | fast | full. STA_SITE_PIP: ps per
//! site pip (default 20). STA_PERIODS="clk=ps,...": slack against these.
//! STA_TRACE=1: each path's steps. STA_ARCS=path: every timed arc as
//! `driver cell, sink cell, ps, tile distance` (tab-separated).
//! STA_SHOW=path: for each `driver cell <tab> sink cell` line of that
//! file, the arc's pips and their delays (a `show` line each, first).
//!
//! stdout: one JSON line of counts, then one per clock pair, slowest first,
//! then each clock's tightest same-clock hold check (`hold`, fast corner,
//! no skew; `sta::Timer::hold`).

use std::path::Path;

use oxibit_fasm::db::Database;
use oxibit_fasm::route::{Graph, GraphOpts};
use oxibit_pnr::delay::Model;
use oxibit_pnr::placed::{self, Meta, Perm};
use oxibit_pnr::sta::{Constraints, Design, FasmRouting, Timer};
use serde_json::{Value, json};

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().collect();
    let (db_root, part, meta_dir, placed_path, fasm_path, perm) = match &args[1..] {
        [a, b, c, d, e] => (a, b, c, d, e, Perm::A5),
        [a, b, c, d, e, p] if p == "a5" => (a, b, c, d, e, Perm::A5),
        [a, b, c, d, e, p] if p == "none" => (a, b, c, d, e, Perm::None),
        _ => return Err("usage: sta DB_ROOT PART META_DIR PLACED.json ROUTED.fasm [a5|none]".into()),
    };
    let model = match std::env::var("DELAY_MODEL").as_deref() {
        Ok("fast") => Model::Fast,
        Ok("full") => Model::Full,
        _ => Model::Intrinsic,
    };
    let cons = Constraints::parse(&std::env::var("STA_PERIODS").unwrap_or_default())?;
    let db = Database::open(Path::new(db_root), part)?;
    let g = Graph::build_with(&db, None, GraphOpts { defaults: true })?;
    let read = |p: &str| std::fs::read_to_string(p).map_err(|e| format!("{p}: {e}"));
    let design: Value = serde_json::from_str(&read(placed_path)?).map_err(|e| format!("{placed_path}: {e}"))?;
    let sites = placed::site_index(&db);
    let imp = placed::import(&g, &sites, &mut Meta::open(Path::new(meta_dir)), &design, perm)?;

    let mut timer = Timer::new(Design::build(&g, &sites, &design, &imp)?, &cons, model);
    if let Some(ps) = std::env::var("STA_SITE_PIP").ok().and_then(|s| s.parse().ok()) {
        timer.site_pip = ps;
    }
    let routing = FasmRouting::new(&g, &read(fasm_path)?, model);
    let counts = routing.annotate(&mut timer, &imp.nets, perm == Perm::None);
    // The pips of chosen arcs, from the net's source as the import has it.
    if let Ok(path) = std::env::var("STA_SHOW") {
        for line in read(&path)?.lines() {
            let Some((from, to)) = line.split_once('\t') else { continue };
            let pips = timer.find_arc(from, to).map(|(net, nodes)| routing.trace(imp.nets[net].source, &nodes)).unwrap_or_default();
            println!("{}", json!({"show": [from, to], "ps": pips.iter().map(|p| p.1).sum::<u32>(), "pips": pips}));
        }
    }
    let trace = std::env::var("STA_TRACE").is_ok();
    let paths: Vec<Value> = timer.analyze().paths.iter().map(|p| p.to_json(trace)).collect();
    if let Ok(path) = std::env::var("STA_ARCS") {
        let mut text = String::new();
        for (from, to, ps, dist) in timer.arcs(&|tile| g.tile_xy(tile)) {
            text.push_str(&format!("{from}\t{to}\t{ps}\t{}\n", dist.map_or(-1, |d| d)));
        }
        std::fs::write(&path, text).map_err(|e| format!("{path}: {e}"))?;
    }
    let d = timer.design();
    println!(
        "{}",
        json!({"sta": {"model": format!("{model:?}"), "site_pip_ps": timer.site_pip, "cells": d.cell_counts(), "ignored": d.ignored, "unplaced": d.unplaced,
                       "in_loops": d.in_loops, "clocks": d.clocks(), "endpoints": timer.table().endpoints, "summary": timer.table().summary(),
                       "arcs": {"from_source": counts.from_source, "from_sink_tree": counts.from_sink_tree, "unreached": counts.unreached}}})
    );
    for p in paths {
        println!("{p}");
    }
    for h in timer.hold(oxibit_pnr::delay::HOLD_ROUTE_SCALE, 300) {
        println!("{}", json!({"hold": h.to_json(), "margin_ns": 0.3}));
    }
    Ok(())
}
