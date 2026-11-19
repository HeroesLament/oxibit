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
//! `driver cell, sink cell, ps, tile distance, dx, dy, driver kind,
//! sink kind` (tab-separated; distance -1 and dx, dy 0 where a cell is
//! unplaced). `oxibit-place`'s `delay_grid` turns it into the placer's
//! delay table.
//! STA_SHOW=path: for each `driver cell <tab> sink cell` line of that
//! file, the arc's pips and their delays (a `show` line each, first).
//!
//! stdout: one JSON line of counts, then one per clock pair, slowest first,
//! then each clock's tightest same-clock hold check (`hold`, fast corner,
//! no skew; `sta::Timer::hold`), then a `sypher` line per clock
//! (`docs/sypher.md`): the worst same-clock path as routed, with arcs held
//! to the packing's floor and to the netlist's, and Sy = floor / routed
//! with its losses in dB. STA_W0: the minimal routed hop, ps (default 301).
//!
//! ROUTED.fasm given as `-`: no routing is read. Only `sypher_floor` lines
//! are printed, the netlist's floor and the packing's from the placement
//! alone, so two syntheses of a design can be compared unrouted.

use std::path::Path;

use oxibit_fasm::db::Database;
use oxibit_fasm::route::{Graph, GraphOpts};
use oxibit_pnr::delay::Model;
use oxibit_pnr::placed::{self, Meta, Perm};
use oxibit_pnr::sta::{Constraints, Design, FasmRouting, Floor, SlackTable, Timer};
use serde_json::{Value, json};

/// Per clock, the slowest same-clock path: (delay ps, logic ps, levels).
fn worst(t: &SlackTable) -> std::collections::BTreeMap<String, (i64, i64, usize)> {
    let mut m = std::collections::BTreeMap::new();
    for p in t.paths.iter().filter(|p| p.from_clk == p.to_clk) {
        let e = m.entry(p.from_clk.clone()).or_insert((p.delay_ps, p.logic_ps, p.levels));
        if p.delay_ps > e.0 {
            *e = (p.delay_ps, p.logic_ps, p.levels);
        }
    }
    m
}

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
    if fasm_path == "-" {
        // Unrouted: the floors alone.
        let w0: i64 = std::env::var("STA_W0").ok().and_then(|s| s.parse().ok()).unwrap_or(301);
        let packed_arcs = timer.floor_unrouted(Floor::Packed, w0);
        let pack = worst(timer.analyze());
        let floor_arcs = timer.floor_unrouted(Floor::Netlist, w0);
        let floor = worst(timer.analyze());
        let d = timer.design();
        println!("{}", json!({"sta": {"unrouted": true, "cells": d.cell_counts(), "ignored": d.ignored, "unplaced": d.unplaced, "in_loops": d.in_loops, "clocks": d.clocks()}}));
        for (clk, &(t_floor, logic, levels)) in &floor {
            let Some(&(t_pack, ..)) = pack.get(clk) else { continue };
            if t_floor <= 0 || t_pack <= 0 {
                continue;
            }
            println!(
                "{}",
                json!({"sypher_floor": {"clock": clk, "w0_ps": w0, "t_floor_ns": t_floor as f64 / 1000.0, "t_pack_ns": t_pack as f64 / 1000.0,
                                         "f_floor_mhz": (1e9 / t_floor as f64).round() / 1000.0,
                                         "sy_pack": (1000.0 * t_floor as f64 / t_pack as f64).round() / 1000.0,
                                         "floor_path": {"logic_ns": logic as f64 / 1000.0, "levels": levels},
                                         "arcs": {"packed_in_slice": packed_arcs.in_slice, "could_be_in_slice": floor_arcs.in_slice,
                                                  "dedicated": floor_arcs.dedicated, "routed": floor_arcs.routed}}})
            );
        }
        return Ok(());
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
        for a in timer.arcs(&|tile| g.tile_xy(tile)) {
            // The first four columns as before: readers of the old form still work.
            let dist = a.delta.map_or(-1, |(dx, dy)| dx.abs() + dy.abs());
            let (dx, dy) = a.delta.unwrap_or((0, 0));
            text.push_str(&format!("{}\t{}\t{}\t{dist}\t{dx}\t{dy}\t{:?}\t{:?}\n", a.from, a.to, a.ps, a.from_kind, a.to_kind));
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

    // The Sypher number. Last: the floors overwrite the routed delays.
    let w0: i64 = std::env::var("STA_W0").ok().and_then(|s| s.parse().ok()).unwrap_or(301);
    let crit = worst(timer.table());
    let packed_arcs = timer.floor(Floor::Packed, w0);
    let pack = worst(timer.analyze());
    let floor_arcs = timer.floor(Floor::Netlist, w0);
    let floor = worst(timer.analyze());
    // Loss in dB, two places; `+ 0.0` turns a negative zero positive.
    let db = |ratio: f64| (-1000.0 * ratio.log10()).round() / 100.0 + 0.0;
    let r3 = |v: f64| (v * 1000.0).round() / 1000.0;
    for (clk, &(t_crit, crit_logic, crit_levels)) in &crit {
        let (Some(&(t_pack, ..)), Some(&(t_floor, floor_logic, floor_levels))) = (pack.get(clk), floor.get(clk)) else { continue };
        if t_crit <= 0 || t_pack <= 0 || t_floor <= 0 {
            continue;
        }
        let (sy, sy_pack, sy_pr) = (t_floor as f64 / t_crit as f64, t_floor as f64 / t_pack as f64, t_pack as f64 / t_crit as f64);
        let mut v = json!({"clock": clk, "w0_ps": w0,
                           "t_floor_ns": t_floor as f64 / 1000.0, "t_pack_ns": t_pack as f64 / 1000.0, "t_crit_ns": t_crit as f64 / 1000.0,
                           "f_floor_mhz": r3(1e6 / t_floor as f64), "fmax_mhz": r3(1e6 / t_crit as f64),
                           "sy": r3(sy), "sy_pack": r3(sy_pack), "sy_place_route": r3(sy_pr),
                           "loss_db": {"pack": db(sy_pack), "place_route": db(sy_pr), "total": db(sy)},
                           "floor_path": {"logic_ns": floor_logic as f64 / 1000.0, "levels": floor_levels},
                           "crit_path": {"logic_ns": crit_logic as f64 / 1000.0, "levels": crit_levels},
                           "arcs": {"packed_in_slice": packed_arcs.in_slice, "could_be_in_slice": floor_arcs.in_slice,
                                    "dedicated": floor_arcs.dedicated, "routed": floor_arcs.routed}});
        if let Some(period) = cons.period(clk) {
            let req = t_floor as f64 / period as f64;
            v["target"] = json!({"period_ns": period as f64 / 1000.0, "sy_req": r3(req), "budget_db": db(req), "short_db": r3(db(sy) - db(req))});
        }
        println!("{}", json!({"sypher": v}));
    }
    Ok(())
}
