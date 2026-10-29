//! Route a placed, unrouted nextpnr design (`--no-route --write`) on
//! oxibit's graph and report congestion each iteration, to measure what LUT
//! input permutation (`PERM`) changes on the same placement.
//!
//! cargo run --release -p oxibit-pnr --example placed_route -- \
//!     DB_ROOT PART META_DIR PLACED.JSON PERM ITERS
//!
//! PERM is none, a5 (A1..A5 permute: nextpnr today) or a6. stdout carries
//! one JSON object per line: the import, one per iteration (overused nodes
//! by wire class), and the end; progress goes to stderr.

use std::collections::{BTreeMap, HashSet};
use std::io::Write;
use std::path::Path;
use std::sync::atomic::AtomicBool;

use oxibit_fasm::db::Database;
use oxibit_fasm::route::{Graph, GraphOpts};
use oxibit_pnr::placed::{self, Meta, Perm};
use oxibit_pnr::router::{self, Opts, Router};
use oxibit_report::{Human, HumanStyle, Reporter};
use serde_json::json;

static CANCEL: AtomicBool = AtomicBool::new(false);

/// Wire classes, first match wins: (class, wire name parts).
const CLASSES: &[(&str, &[&str])] = &[
    ("IMUX", &["IMUX"]),
    ("BYP", &["BYP"]),
    ("GFAN", &["GFAN"]),
    ("FAN", &["FAN"]),
    ("CTRL", &["CTRL"]),
    ("CLK", &["CLK"]),
    ("LOGIC_OUTS", &["LOGIC_OUTS"]),
    ("SINGLE", &["1BEG", "1END"]),
    ("DOUBLE", &["2BEG", "2END"]),
    ("QUAD", &["4BEG", "4END"]),
    ("LONG", &["LH", "LV"]),
    ("CLB", &["CLBLL_", "CLBLM_"]),
    ("BRAM", &["BRAM"]),
    ("DSP", &["DSP"]),
];

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().collect();
    let [_, db_root, part, meta_dir, placed_path, perm, iters] = &args[..] else {
        return Err("usage: placed_route DB_ROOT PART META_DIR PLACED.JSON none|a5|a6 ITERS".into());
    };
    let perm = match perm.as_str() {
        "none" => Perm::None,
        "a5" => Perm::A5,
        "a6" => Perm::A6,
        p => return Err(format!("PERM {p}: none, a5 or a6")),
    };
    let iters: u32 = iters.parse().map_err(|e| format!("ITERS: {e}"))?;
    let out = std::io::stdout();
    let line = |v: serde_json::Value| {
        let mut o = out.lock();
        let _ = writeln!(o, "{v}");
        let _ = o.flush();
    };

    let r = Reporter::new(vec![Box::new(Human::new(std::io::stderr(), HumanStyle::Plain))], &CANCEL);
    let db = Database::open(Path::new(db_root), part)?;
    let g = Graph::build_with(&db, None, GraphOpts::default())?;
    eprintln!("graph: {} nodes, {} edges", g.nodes(), g.edges());
    let text = std::fs::read_to_string(placed_path).map_err(|e| format!("{placed_path}: {e}"))?;
    let placed_v: serde_json::Value = serde_json::from_str(&text).map_err(|e| format!("{placed_path}: {e}"))?;
    drop(text);
    let mut meta = Meta::open(Path::new(meta_dir));
    let imp = placed::import(&g, &placed::site_index(&db), &mut meta, &placed_v, perm)?;
    drop(placed_v);

    let sinks: usize = imp.nets.iter().map(|n| n.sinks.len() + n.pin_groups.len()).sum();
    line(json!({
        "import": {
            "perm": format!("{perm:?}"),
            "nets": imp.nets.len(),
            "sinks": sinks,
            "fixed_sinks": imp.fixed_sinks,
            "internal_sinks": imp.internal_sinks,
            "group_sizes": imp.group_sizes,
            "luts": imp.luts,
            "skipped": imp.skipped,
            "unresolved": imp.unresolved.counts,
            "unresolved_examples": imp.unresolved.examples,
        }
    }));

    // Node classes: the import's own terminals first, then wire names.
    let mut class = vec![u8::MAX; g.nodes()];
    let mut names: Vec<&str> = vec!["LUT_IN", "SITE_IN", "SITE_OUT"];
    for n in &imp.nets {
        for p in n.pin_groups.iter().flatten() {
            class[*p as usize] = 0;
        }
    }
    for n in &imp.nets {
        for &s in &n.sinks {
            if class[s as usize] == u8::MAX {
                class[s as usize] = 1;
            }
        }
        if class[n.source as usize] == u8::MAX {
            class[n.source as usize] = 2;
        }
    }
    for (name, parts) in CLASSES {
        let id = names.len() as u8;
        names.push(*name);
        let pats: Vec<String> = parts.iter().map(|s| s.to_string()).collect();
        for n in g.nodes_with_wire(&pats) {
            if class[n as usize] == u8::MAX {
                class[n as usize] = id;
            }
        }
    }

    let mut router = Router::new(&g, Opts { max_iters: iters, ..Opts::default() });
    let (routes, stats) = router.run_observed(&imp.nets, &r, &mut |iter, occ| {
        let mut by: BTreeMap<&str, (usize, u64)> = BTreeMap::new();
        let (mut over, mut overuse) = (0usize, 0u64);
        for (n, &o) in occ.iter().enumerate() {
            if o > 1 {
                over += 1;
                overuse += (o - 1) as u64;
                let c = class[n];
                let e = by.entry(if c == u8::MAX { "OTHER" } else { names[c as usize] }).or_default();
                e.0 += 1;
                e.1 += (o - 1) as u64;
            }
        }
        let by: BTreeMap<&str, serde_json::Value> = by.into_iter().map(|(k, (n, u))| (k, json!({"nodes": n, "overuse": u}))).collect();
        line(json!({"iter": iter, "overused": over, "overuse": overuse, "by_class": by}));
    });

    // Which member of each six-pin group the router took (index 5 is A6).
    let mut pos6 = [0usize; 6];
    let mut unreached = 0usize;
    let mut used: HashSet<u32> = HashSet::new();
    for (n, hops) in imp.nets.iter().zip(&routes) {
        for (grp, pin) in n.pin_groups.iter().zip(router::chosen_pins(n, hops)) {
            match pin {
                Some(p) => {
                    used.insert(p);
                    if grp.len() == 6 {
                        if let Some(i) = grp.iter().position(|&x| x == p) {
                            pos6[i] += 1;
                        }
                    }
                }
                None => unreached += 1,
            }
        }
    }
    // Sinks the routing does not reach, by name (a few).
    let mut missed = Vec::new();
    for (i, (n, hops)) in imp.nets.iter().zip(&routes).enumerate() {
        let reached: HashSet<u32> = hops.iter().map(|h| h.dst).chain([n.source]).collect();
        for s in n.sinks.iter().filter(|s| !reached.contains(s)) {
            if missed.len() < 30 {
                missed.push(json!([imp.names[i], g.node_name(n.source), g.node_name(*s)]));
            }
        }
    }
    line(json!({"done": stats.to_json(), "six_pin_groups_by_pin": pos6, "groups_unreached": unreached, "group_pins_used": used.len(), "sinks_unreached": missed}));
    Ok(())
}
