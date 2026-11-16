//! Check the placed-design import and the router against a design nextpnr
//! routed: import its placement (`--write` JSON; routing attributes are
//! ignored), compare every imported net with the net nextpnr's FASM routes
//! from the same source, then route the imported nets with oxibit and write
//! the design back with oxibit's routing in place of nextpnr's.
//!
//! cargo run --release -p oxibit-pnr --example placed_check -- \
//!     DB_ROOT PART META_DIR DESIGN.JSON DESIGN.FASM OUT.FASM [OUT.JSON]
//!
//! OXIBIT_PERM=a5 (or a6) lets LUT inputs move between pins (the comparison
//! then shows LUT pins as differences). A placed-only JSON needs a5: the
//! placer leaves signals on A6 of fractured LUTs, which routing moves off.
//!
//! With OUT.JSON, constant nets keep nextpnr's routing too, and the design
//! JSON is written back with each imported net's `ROUTING` attribute set to
//! oxibit's routing (pips at strength 3, placer: router2 treats those wires
//! as reserved for the net and extends them, where any stronger binding
//! makes them unavailable even to their own net): nextpnr can load it with
//! `--no-place`, finish the routing inside the sites, analyse timing and
//! write the FASM.
//!
//! OXIBIT_STANDALONE=1: `flow::standalone` routes the whole design from
//! the netlist and writes the FASM, each line owned by one of its writers;
//! DESIGN.FASM is used only for the cell configuration no writer covers
//! yet (its `base` lines, counted in the report and written to
//! OUT.FASM.base).
//!
//! With `Perm::None` pins stay where nextpnr put them, so the LUT INITs in
//! the FASM stay valid. Otherwise every LUT INIT is computed from the
//! design's cells for the pins the router chose (`lutinit`). Clock and default nets keep nextpnr's routing; constant
//! nets are routed again by oxibit.
//! stdout: one JSON line per phase. Exit 0 when the comparison is exact and
//! the new FASM reaches every sink of nextpnr's nets with no short.

use std::collections::{HashMap, HashSet};
use std::io::Write;
use std::path::Path;
use std::process::ExitCode;
use std::sync::atomic::AtomicBool;

use oxibit_fasm::db::Database;
use oxibit_fasm::route::{Graph, GraphOpts};
use oxibit_pnr::fasm_out::Owner;
use oxibit_pnr::{flow, lutinit};
use oxibit_pnr::nets::{self, NetClass};
use oxibit_pnr::placed::{self, Meta, Perm};
use oxibit_pnr::reroute;
use oxibit_pnr::router::{Hop, Opts, Router};
use oxibit_report::{Human, HumanStyle, Reporter};
use serde_json::json;

static CANCEL: AtomicBool = AtomicBool::new(false);

fn main() -> ExitCode {
    match run() {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::from(1),
        Err(e) => {
            eprintln!("placed_check: {e}");
            ExitCode::from(3)
        }
    }
}

fn run() -> Result<bool, String> {
    let args: Vec<String> = std::env::args().collect();
    let (db_root, part, meta_dir, design, fasm_path, out_path, out_json) = match &args[..] {
        [_, a, b, c, d, e, f] => (a, b, c, d, e, f, None),
        [_, a, b, c, d, e, f, j] => (a, b, c, d, e, f, Some(j)),
        _ => return Err("usage: placed_check DB_ROOT PART META_DIR DESIGN.JSON DESIGN.FASM OUT.FASM [OUT.JSON]".into()),
    };
    let line = |v: serde_json::Value| {
        let mut o = std::io::stdout().lock();
        let _ = writeln!(o, "{v}");
        let _ = o.flush();
    };

    let r = Reporter::new(vec![Box::new(Human::new(std::io::stderr(), HumanStyle::Plain))], &CANCEL);
    let db = Database::open(Path::new(db_root), part)?;
    let g = Graph::build_with(&db, None, GraphOpts { defaults: true })?;
    let fasm = std::fs::read_to_string(fasm_path).map_err(|e| format!("{fasm_path}: {e}"))?;
    let text = std::fs::read_to_string(design).map_err(|e| format!("{design}: {e}"))?;
    let placed_v: serde_json::Value = serde_json::from_str(&text).map_err(|e| format!("{design}: {e}"))?;
    let mut meta = Meta::open(Path::new(meta_dir));
    let perm = match std::env::var("OXIBIT_PERM").as_deref() {
        Ok("a5") => Perm::A5,
        Ok("a6") => Perm::A6,
        _ => Perm::None,
    };
    let imp = placed::import(&g, &placed::site_index(&db), &mut meta, &placed_v, perm)?;
    let (fnets, fun) = nets::from_fasm(&g, &fasm);
    line(json!({
        "import": {"nets": imp.nets.len(), "fixed_sinks": imp.fixed_sinks, "internal_sinks": imp.internal_sinks,
                   "skipped": imp.skipped, "unresolved": imp.unresolved.counts, "unresolved_examples": imp.unresolved.examples},
        "fasm": {"nets": fnets.len(), "missing_pips": fun.missing.len(), "multi_driven": fun.multi_driven.len()},
    }));

    // nextpnr passes some nets through an unused LUT (in at X1..X6, out at
    // X), which a FASM shows as two pip trees. Join them: a tree ending on a
    // LUT input whose LUT output starts a tree no imported net drives.
    let mut lut_out: HashMap<u32, u32> = HashMap::new();
    // LUT output X -> the site's XMUX: nextpnr can send part of an O6 net
    // out through each.
    let mut out_mux: HashMap<u32, u32> = HashMap::new();
    let clb_tiles: Vec<String> = g.tile_names().filter(|t| t.starts_with("CLB")).map(String::from).collect();
    for tile in &clb_tiles {
        for site in g.sites(tile) {
            for l in ["A", "B", "C", "D"] {
                let Some(o) = g.site_pin(tile, &site, l) else { continue };
                if let Some(m) = g.site_pin(tile, &site, &format!("{l}MUX")) {
                    out_mux.insert(o, m);
                }
                for k in 1..=6 {
                    if let Some(i) = g.site_pin(tile, &site, &format!("{l}{k}")) {
                        lut_out.insert(i, o);
                    }
                }
            }
        }
    }
    let by_src: HashMap<u32, usize> = fnets.iter().enumerate().map(|(i, n)| (n.source, i)).collect();
    let ours_src: HashSet<u32> = imp.nets.iter().map(|n| n.source).collect();
    let mut absorbed: HashSet<usize> = HashSet::new();
    let mut through = 0usize;
    let expanded: Vec<Vec<u32>> = fnets
        .iter()
        .map(|n| {
            let mut out = Vec::new();
            let mut work: Vec<u32> = n.sinks.clone();
            let mut seen: HashSet<u32> = HashSet::new();
            while let Some(s) = work.pop() {
                if !seen.insert(s) {
                    continue;
                }
                match lut_out.get(&s).and_then(|o| by_src.get(o).filter(|_| !ours_src.contains(o))) {
                    Some(&fi) => {
                        absorbed.insert(fi);
                        through += 1;
                        work.extend(fnets[fi].sinks.iter().copied());
                    }
                    None => out.push(s),
                }
            }
            out.sort_unstable();
            out
        })
        .collect();

    // Compare each imported net with nextpnr's net from the same source, or,
    // where nextpnr left the site by another output pin, with the net that
    // reaches exactly the same sinks.
    let by_sinks: HashMap<Vec<u32>, usize> =
        fnets.iter().enumerate().filter(|(i, n)| n.class == NetClass::Signal && !absorbed.contains(i)).map(|(i, _)| (expanded[i].clone(), i)).collect();
    let (mut exact, mut other_exit, mut split_exit, mut no_match, mut short_sinks, mut extra_sinks) = (0usize, 0usize, 0usize, 0usize, 0usize, 0usize);
    let mut examples = Vec::new();
    let mut claimed: HashSet<usize> = HashSet::new();
    for (i, n) in imp.nets.iter().enumerate() {
        let fi = match by_src.get(&n.source) {
            Some(&fi) => fi,
            None => match by_sinks.get(&n.sinks) {
                Some(&fi) => {
                    other_exit += 1;
                    fi
                }
                None => {
                    no_match += 1;
                    if examples.len() < 16 {
                        examples.push(json!({"why": "no nextpnr net from source or with these sinks", "net": imp.names[i], "source": g.node_name(n.source),
                                             "sinks": n.sinks.iter().take(4).map(|&s| g.node_name(s)).collect::<Vec<_>>()}));
                    }
                    continue;
                }
            },
        };
        claimed.insert(fi);
        let fs: HashSet<u32> = expanded[fi].iter().copied().collect();
        let ours: HashSet<u32> = n.sinks.iter().copied().collect();
        let mut missing: Vec<u32> = ours.difference(&fs).copied().collect();
        // nextpnr's LUT input pins are ours when they are in the net's pin
        // groups: the router picks one pin of each group (OXIBIT_PERM).
        let grouped: HashSet<u32> = n.pin_groups.iter().flatten().copied().collect();
        let extra: Vec<u32> = fs.difference(&ours).filter(|s| !grouped.contains(s)).copied().collect();
        // The rest of an O6 net that nextpnr sent out through XMUX.
        if !missing.is_empty() {
            missing.sort_unstable();
            if let Some(&mi) = out_mux.get(&n.source).and_then(|m| by_src.get(m)) {
                if expanded[mi] == missing && !claimed.contains(&mi) {
                    claimed.insert(mi);
                    split_exit += 1;
                    missing.clear();
                }
            }
        }
        short_sinks += missing.len();
        extra_sinks += extra.len();
        if missing.is_empty() && extra.is_empty() {
            exact += 1;
        } else if examples.len() < 16 {
            examples.push(json!({
                "why": "sinks differ", "net": imp.names[i], "source": g.node_name(n.source),
                "only_ours": missing.iter().take(4).map(|&s| g.node_name(s)).collect::<Vec<_>>(),
                "only_nextpnr": extra.iter().take(4).map(|&s| g.node_name(s)).collect::<Vec<_>>(),
            }));
        }
    }
    let mut unclaimed = 0usize;
    for (fi, n) in fnets.iter().enumerate() {
        if n.class == NetClass::Signal && !claimed.contains(&fi) && !absorbed.contains(&fi) {
            unclaimed += 1;
            if examples.len() < 24 {
                examples.push(json!({"why": "nextpnr signal net not imported", "source": g.node_name(n.source),
                                     "sinks": expanded[fi].iter().take(4).map(|&s| g.node_name(s)).collect::<Vec<_>>()}));
            }
        }
    }
    let compare_ok = no_match == 0 && short_sinks == 0 && extra_sinks == 0 && unclaimed == 0;
    line(json!({"compare": {"exact": exact, "other_exit": other_exit, "split_exit": split_exit, "no_match": no_match, "sinks_only_ours": short_sinks, "sinks_only_nextpnr": extra_sinks,
                            "nextpnr_signal_nets_not_imported": unclaimed, "lut_route_throughs_joined": through, "examples": examples}}));

    // Standalone: the library flow routes and writes everything.
    if std::env::var("OXIBIT_STANDALONE").is_ok_and(|v| v == "1") {
        let sa = flow::standalone(&db, &g, &placed_v, &imp, &fasm, &r)?;
        for v in &sa.report {
            line(v.clone());
        }
        std::fs::write(out_path, sa.fasm.text()).map_err(|e| format!("{out_path}: {e}"))?;
        // The lines still taken from DESIGN.FASM, for the writers to come.
        let base: String = sa.fasm.of(Owner::Base).map(|l| format!("{l}\n")).collect();
        let base_path = format!("{out_path}.base");
        std::fs::write(&base_path, base).map_err(|e| format!("{base_path}: {e}"))?;
        let ok = sa.ok();
        line(json!({"ok": ok, "compare_ok": compare_ok, "out": out_path}));
        return Ok(ok);
    }

    // Route: the imported signal nets, plus nextpnr's clock and default nets
    // kept as routed (the router reserves their nodes) and its constant nets
    // routed again by oxibit, as `pnr route` does.
    let mut all = imp.nets.clone();
    for n in fnets.iter().filter(|n| n.class != NetClass::Signal) {
        let mut k = n.clone();
        // Exporting to nextpnr: its constant routing stays (kept nets).
        if out_json.is_some() && k.class == NetClass::Constant {
            k.class = NetClass::Clock;
        }
        all.push(k);
    }
    let mut router = Router::new(&g, Opts::default());
    let (routes, stats) = router.run_with(&all, &r);
    line(json!({"route": stats.to_json(), "nets": all.len()}));

    // Write: drop nextpnr's signal and constant pips, add oxibit's.
    let rerouted = |c: NetClass| c == NetClass::Signal || c == NetClass::GlobalClock || (c == NetClass::Constant && out_json.is_none());
    let drop: HashSet<String> =
        fnets.iter().filter(|n| rerouted(n.class)).flat_map(|n| n.edges.iter().map(|e| g.edge_pip(e.tile, e.pi).name())).collect();
    let mut out = String::with_capacity(fasm.len());
    let mut removed = 0usize;
    for l in fasm.lines() {
        if drop.contains(l.split('#').next().unwrap_or("").trim()) {
            removed += 1;
            continue;
        }
        out.push_str(l);
        out.push('\n');
    }
    let mut seen = HashSet::new();
    let mut added = 0usize;
    for (n, hops) in all.iter().zip(&routes) {
        if !rerouted(n.class) {
            continue;
        }
        for l in hops.iter().flat_map(|h: &Hop| g.edge_pip(h.tile, h.pi).fasm_lines()) {
            if seen.insert(l.clone()) {
                out.push_str(&l);
                out.push('\n');
                added += 1;
            }
        }
    }
    // LUT INITs: with pins allowed to move, every LUT's INIT is computed
    // from its cells' logical INITs for the pins the router chose.
    if perm != Perm::None {
        let loc = lutinit::Locator::new(&db)?;
        let (cells, skipped) = lutinit::cell_inits(&placed_v, &|b| loc.locate(b))?;
        let moves = lutinit::moves(&imp, &routes[..imp.nets.len()])?;
        let (text, st) = lutinit::apply(&out, &cells, &moves)?;
        out = text;
        line(json!({"lut_init": {"cells": cells.len(), "skipped": skipped, "written": st.written, "moved": st.moved,
                                 "routing_dropped": st.routing_dropped, "cellless_kept": st.cellless_kept}}));
    }
    // Slice routing-bel settings for the site paths the import chose (FF
    // data and output muxes, carry inputs, WEMUX), in place of nextpnr's.
    let (text, sm) = placed::apply_site_muxes(&out, &imp);
    out = text;
    line(json!({"site_muxes": {"set": sm.set, "written": sm.written, "dropped": sm.dropped}}));
    std::fs::write(out_path, &out).map_err(|e| format!("{out_path}: {e}"))?;

    // Check over the new FASM alone: every imported sink reached, and no node
    // reached from two sources (imported nets plus the kept and constant
    // nets).
    let after = reroute::check(&g, &all, &out);
    line(json!({"write": {"pips_removed": removed, "lines_added": added, "out": out_path}, "check": after.to_json(),
                "missing_examples": after.missing.iter().take(12).map(|&(s, k)| format!("{} -> {}", g.node_name(s), g.node_name(k))).collect::<Vec<_>>()}));
    if let Some(out_json) = out_json {
        let mut v = placed_v.clone();
        let (set, routing, unnamed) = placed::write_routing(&g, &imp, &routes, &mut v)?;
        std::fs::write(out_json, serde_json::to_string(&v).map_err(|e| e.to_string())?).map_err(|e| format!("{out_json}: {e}"))?;
        line(json!({"export": {"json": out_json, "nets_with_routing": set, "bits": routing.len(), "hops_without_tile": unnamed}}));
    }
    // The import matches nextpnr's nets only while every pin is the one
    // nextpnr chose; with LUT pins free (OXIBIT_PERM) or the whole design
    // routed here, the comparison is information and `fasm_equiv` is the
    // proof of equivalence.
    let compare_gates = perm == Perm::None;
    let ok = (compare_ok || !compare_gates) && stats.overused == 0 && stats.unroutable_sinks == 0 && after.sinks_missing == 0 && after.shorts.is_empty();
    line(json!({"ok": ok, "compare_ok": compare_ok}));
    Ok(ok)
}
