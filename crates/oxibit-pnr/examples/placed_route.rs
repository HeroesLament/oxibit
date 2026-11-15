//! Route a placed, unrouted nextpnr design (`--no-route --write`) on
//! oxibit's graph and report congestion each iteration, to measure what LUT
//! input permutation (`PERM`) changes on the same placement.
//!
//! cargo run --release -p oxibit-pnr --example placed_route -- \
//!     DB_ROOT PART META_DIR PLACED.JSON PERM ITERS [OUT.JSON]
//!
//! With OUT.JSON, the placed design is written back with oxibit's routing as
//! nextpnr ROUTING attributes (`placed::write_routing`), for nextpnr to load
//! with --no-place, route what is left (constants, clocks, anything oxibit
//! did not finish), analyse timing and write the FASM.
//!
//! PERM is none, a5 (A1..A5 permute: nextpnr today) or a6. stdout carries
//! one JSON object per line: the import, one per iteration (overused nodes
//! by wire class), the sinks on the PS (`ps_sinks`), what the routing uses
//! (`routed`), and the end; progress goes to stderr.

use std::collections::{BTreeMap, HashMap, HashSet};
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
    let (db_root, part, meta_dir, placed_path, perm, iters, out_json) = match &args[..] {
        [_, a, b, c, d, e, f] => (a, b, c, d, e, f, None),
        [_, a, b, c, d, e, f, j] => (a, b, c, d, e, f, Some(j)),
        _ => return Err("usage: placed_route DB_ROOT PART META_DIR PLACED.JSON none|a5|a6 ITERS [OUT.JSON]".into()),
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

    let sinks: usize = imp.nets.iter().map(|n| n.sinks.len() + n.pin_groups.len()).sum();
    let conflicts = placed::pin_conflicts(&imp.nets);
    let conflict_examples: Vec<serde_json::Value> = conflicts
        .iter()
        .take(12)
        .map(|(node, ns)| json!({"pin": g.node_name(*node), "nets": ns.iter().map(|&i| imp.names[i].as_str()).collect::<Vec<_>>()}))
        .collect();
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
            "pin_conflicts": conflicts.len(),
            "pin_conflict_examples": conflict_examples,
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
    // Unreached pin groups and the nodes still shared, by name (a few).
    for (i, (n, hops)) in imp.nets.iter().zip(&routes).enumerate() {
        for (grp, pin) in n.pin_groups.iter().zip(router::chosen_pins(n, hops)) {
            if pin.is_none() && missed.len() < 40 {
                missed.push(json!([imp.names[i], g.node_name(n.source), format!("group {}", g.node_name(grp[0]))]));
            }
        }
    }
    let shared: Vec<serde_json::Value> = if stats.overused <= 50 {
        router
            .audit(&imp.nets, &routes)
            .iter()
            .take(30)
            .map(|(node, ns)| json!({"node": g.node_name(*node), "nets": ns.iter().map(|&i| imp.names[i].as_str()).collect::<Vec<_>>()}))
            .collect()
    } else {
        Vec::new()
    };
    // What the routing uses: pips, the distinct wires of each net, and
    // those wires' nominal span in tiles (7-series segment lengths: single
    // 1, double 2, quad 4, hex 6, horizontal long 12, vertical long 18;
    // site pins and wires within a tile 0).
    let span_of = |wire: &str| -> u64 {
        let has = |parts: &[&str]| parts.iter().any(|p| wire.contains(p));
        if wire.starts_with("LV") {
            18
        } else if wire.starts_with("LH") {
            12
        } else if has(&["6BEG", "6END"]) {
            6
        } else if has(&["4BEG", "4END"]) {
            4
        } else if has(&["2BEG", "2END"]) {
            2
        } else if has(&["1BEG", "1END"]) {
            1
        } else {
            0
        }
    };
    let (mut pips, mut wires, mut span) = (0u64, 0u64, 0u64);
    let mut wires_by: BTreeMap<&str, u64> = BTreeMap::new();
    for hops in &routes {
        pips += hops.len() as u64;
        let nodes: HashSet<u32> = hops.iter().map(|h| h.dst).collect();
        for &n in &nodes {
            wires += 1;
            let c = class[n as usize];
            *wires_by.entry(if c == u8::MAX { "OTHER" } else { names[c as usize] }).or_default() += 1;
            let name = g.node_name(n).to_string();
            span += span_of(name.rsplit('/').next().unwrap_or(""));
        }
    }
    // Sinks on the PS: where the net starts, where the PS pin's node is, and
    // where the route leaves the general interconnect for it (the node the
    // last interconnect pip on the way reads). For checking a placer's
    // idea of where PS pins are against where their nets have to arrive.
    let mut ps_sinks: Vec<serde_json::Value> = Vec::new();
    for (n, hops) in imp.nets.iter().zip(&routes) {
        let into: HashMap<u32, &router::Hop> = hops.iter().map(|h| (h.dst, h)).collect();
        for &s in n.sinks.iter().filter(|&&s| g.node_name(s).to_string().starts_with("PSS")) {
            let mut at = s;
            let mut entry = None;
            for _ in 0..hops.len() {
                let Some(h) = into.get(&at) else { break };
                if g.edge_pip(h.tile, h.pi).tile.starts_with("INT") {
                    // The interconnect side: the PS wire the pip drives sits
                    // at its PSS tile's anchor, not on the row it enters at.
                    entry = Some(g.node_xy(h.src));
                    break;
                }
                at = h.src;
            }
            let ((sx, sy), (px, py)) = (g.node_xy(n.source), g.node_xy(s));
            ps_sinks.push(json!({"source": g.node_name(n.source).to_string(), "source_xy": [sx, sy], "pin_xy": [px, py], "entry_xy": entry.map(|(x, y)| [x, y])}));
        }
    }
    line(json!({ "ps_sinks": ps_sinks }));
    line(json!({"routed": {"pips": pips, "wires": wires, "span_tiles": span, "wires_by_class": wires_by}}));
    line(json!({"done": stats.to_json(), "six_pin_groups_by_pin": pos6, "groups_unreached": unreached, "group_pins_used": used.len(), "sinks_unreached": missed, "overused_nodes": shared}));
    if let Some(out_json) = out_json {
        let mut v = placed_v;
        let (set, routing, unnamed) = placed::write_routing(&g, &imp, &routes, &mut v)?;
        std::fs::write(out_json, serde_json::to_string(&v).map_err(|e| e.to_string())?).map_err(|e| format!("{out_json}: {e}"))?;
        line(json!({"export": {"json": out_json, "nets_with_routing": set, "bits": routing.len(), "hops_without_tile": unnamed}}));
    }
    Ok(())
}
