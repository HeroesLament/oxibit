//! Interconnect delay of two routings of one placement, on nextpnr's
//! critical paths.
//!
//! cargo run --release -p oxibit-pnr --example delay_check -- \
//!     DB_ROOT PART META_DIR ROUTED.json REPORT.json NEXTPNR.fasm [OTHER.fasm OTHER.json]
//!
//! ROUTED.json and REPORT.json are nextpnr's (`--write`, `--report
//! --detailed-timing-report`), NEXTPNR.fasm its routing. For every routing
//! segment of the report's critical paths (net, sink location and port,
//! delay), `delay::arrivals_with` on nextpnr's routing gives the model's
//! delay to the net's sinks at that location; the closest is the segment's
//! sink, and `reported - model` its offset (the site pips, 20 ps each,
//! which the model does not count, and what the model misses).
//!
//! With OTHER.fasm (oxibit's routing of the same placement), each segment
//! is timed again on it. OTHER.json is the placed design it was routed
//! from: its import (as `flow::standalone` makes it) gives each net's
//! source, the root of OTHER's tree for the net. On that tree: the same
//! sink node where that net reaches it,
//! else the slowest of its sinks at the location (LUT inputs can move
//! between pins), plus the segment's offset. Each path is then re-timed
//! with OTHER's routing segments and nextpnr's logic delays. Also, over
//! every net the two routings share a root: the sum and the maximum of the
//! sink delays.
//!
//! DELAY_MODEL=intrinsic (default) | fast | full. DELAY_TRACE=1: the pips of
//! each matched segment on stderr.
//!
//! stdout: one JSON line per segment, one per path, then a summary.

use std::collections::{BTreeMap, HashMap, HashSet};
use std::path::Path;

use oxibit_fasm::db::Database;
use oxibit_fasm::route::{EdgeKind, Graph, GraphOpts};
use oxibit_pnr::delay::{self, Model, TreeEdge};
use oxibit_pnr::nets::{self, Edge, Net, NetClass};
use oxibit_pnr::placed::{self, Meta, Perm};
use serde_json::{Value, json};

type R<T> = Result<T, String>;

/// One routing's trees, by root, with arrivals computed on demand.
struct Routing<'g> {
    g: &'g Graph,
    model: Model,
    trees: Vec<Net>,
    by_root: HashMap<u32, usize>,
    arr: HashMap<usize, HashMap<u32, u64>>,
}

impl<'g> Routing<'g> {
    fn new(g: &'g Graph, fasm: &str, model: Model) -> Self {
        let (mut trees, _) = nets::from_fasm(g, fasm);
        // A FASM that omits the fixed hops (they have no bits; nextpnr's
        // lists them): carry each tree's sinks on along fixed edges to the
        // site pins.
        for t in &mut trees {
            let mut sinks = Vec::new();
            let mut stack = t.sinks.clone();
            let mut seen: HashSet<u32> = stack.iter().copied().collect();
            while let Some(n) = stack.pop() {
                let fixed: Vec<_> = g.out_edges(n).iter().filter(|&&(_, tile, pi)| g.edge_kind(tile, pi) == EdgeKind::Fixed).copied().collect();
                if fixed.is_empty() {
                    sinks.push(n);
                }
                for (d, tile, pi) in fixed {
                    if seen.insert(d) {
                        t.edges.push(Edge { src: n, dst: d, tile, pi, kind: EdgeKind::Fixed });
                        stack.push(d);
                    }
                }
            }
            sinks.sort_unstable();
            t.sinks = sinks;
        }
        let by_root = trees.iter().enumerate().map(|(i, t)| (t.source, i)).collect();
        Routing { g, model, trees, by_root, arr: HashMap::new() }
    }

    /// The tree a net with source `src` has: rooted at `src`, or at a node
    /// `src` reaches over fixed edges (the hops a FASM may omit).
    fn root_from(&self, src: u32) -> Option<u32> {
        let mut frontier = vec![src];
        for _ in 0..4 {
            if let Some(&r) = frontier.iter().find(|r| self.by_root.contains_key(r)) {
                return Some(r);
            }
            frontier = frontier
                .iter()
                .flat_map(|&n| self.g.out_edges(n).iter().filter(|&&(_, tile, pi)| self.g.edge_kind(tile, pi) == EdgeKind::Fixed).map(|e| e.0).collect::<Vec<_>>())
                .collect();
        }
        None
    }

    fn arrivals(&mut self, t: usize) -> &HashMap<u32, u64> {
        let (g, model, tree) = (self.g, self.model, &self.trees[t]);
        self.arr.entry(t).or_insert_with(|| {
            let edges: Vec<TreeEdge> = tree.edges.iter().map(|e| (e.src, e.dst, e.tile, e.pi)).collect();
            delay::arrivals_with(g, tree.source, &edges, model)
        })
    }

    /// The tree's sinks in or beside the tile at `loc`, with their delays.
    fn at_loc(&mut self, root: u32, loc: (i64, i64), wires: &HashMap<u32, Vec<(String, String)>>) -> Vec<(u32, String, i64)> {
        let Some(&t) = self.by_root.get(&root) else { return vec![] };
        let sinks = self.trees[t].sinks.clone();
        let g = self.g;
        let arr = self.arrivals(t).clone();
        sinks
            .into_iter()
            .filter_map(|s| {
                let (tile, w) = wires.get(&s)?.iter().find(|(tile, _)| g.tile_xy(tile).is_some_and(|(x, y)| y as i64 == loc.1 && (x as i64 - loc.0).abs() <= 1))?;
                Some((s, format!("{tile}/{w}"), arr.get(&s).copied().unwrap_or(0) as i64))
            })
            .collect()
    }
}

fn main() -> R<()> {
    let args: Vec<String> = std::env::args().collect();
    let (db_root, part, meta_dir, routed, report_path, np_fasm, other) = match &args[1..] {
        [a, b, c, d, e, f] => (a, b, c, d, e, f, None),
        [a, b, c, d, e, f, o, oj] => (a, b, c, d, e, f, Some((o, oj))),
        _ => return Err("usage: delay_check DB_ROOT PART META_DIR ROUTED.json REPORT.json NEXTPNR.fasm [OTHER.fasm OTHER.json]".into()),
    };
    let model = match std::env::var("DELAY_MODEL").as_deref() {
        Ok("fast") => Model::Fast,
        Ok("full") => Model::Full,
        _ => Model::Intrinsic,
    };
    let trace = std::env::var("DELAY_TRACE").is_ok();
    let db = Database::open(Path::new(db_root), part)?;
    let g = Graph::build_with(&db, None, GraphOpts { defaults: true })?;
    let read_text = |p: &str| std::fs::read_to_string(p).map_err(|e| format!("{p}: {e}"));
    let read_json = |p: &str| -> R<Value> { serde_json::from_str(&read_text(p)?).map_err(|e| format!("{p}: {e}")) };
    let placed_v = read_json(routed)?;
    let report = read_json(report_path)?;
    let mut meta = Meta::open(Path::new(meta_dir));
    let sites = placed::site_index(&db);
    // Report net name -> its bit (any of the JSON's names for it) -> the
    // imported net of that bit -> its source node.
    let roots = |design: &Value, perm: Perm, meta: &mut Meta| -> R<HashMap<String, u32>> {
        let imp = placed::import(&g, &sites, meta, design, perm)?;
        let src_of_bit: HashMap<i64, u32> = imp.bits.iter().zip(&imp.nets).map(|(&b, net)| (b, net.source)).collect();
        let top = design["modules"].as_object().and_then(|m| m.values().find(|m| m["attributes"].get("top").is_some())).ok_or("no top module")?;
        Ok(top["netnames"]
            .as_object()
            .into_iter()
            .flatten()
            .filter_map(|(n, v)| v["bits"].get(0).and_then(Value::as_i64).and_then(|b| src_of_bit.get(&b)).map(|&s| (n.clone(), s)))
            .collect())
    };
    let root_of = roots(&placed_v, Perm::None, &mut meta)?;

    let mut np = Routing::new(&g, &read_text(np_fasm)?, model);
    let (mut ox, other_root_of) = match other {
        Some((p, j)) => (Some(Routing::new(&g, &read_text(p)?, model)), roots(&read_json(j)?, Perm::A5, &mut meta)?),
        None => (None, HashMap::new()),
    };

    // Every sink node's wires (for locations), both routings.
    let mut leaves: HashSet<u32> = np.trees.iter().flat_map(|t| t.sinks.iter().copied()).collect();
    if let Some(o) = &ox {
        leaves.extend(o.trees.iter().flat_map(|t| t.sinks.iter().copied()));
    }
    let wires = g.node_wires(&leaves);

    // OTHER's root for each of nextpnr's, through the net's name.
    let np_root: HashMap<u32, u32> = root_of.values().filter_map(|&r| np.root_from(r).map(|t| (r, t))).collect();
    let other_root: HashMap<u32, u32> = match &ox {
        Some(o) => root_of.iter().filter_map(|(n, &r)| other_root_of.get(n).and_then(|&s| o.root_from(s)).and_then(|t| np_root.get(&r).map(|&nr| (nr, t)))).collect(),
        None => HashMap::new(),
    };
    let root_of: HashMap<String, u32> = root_of.iter().filter_map(|(n, r)| np_root.get(r).map(|&t| (n.clone(), t))).collect();

    let mut offsets: BTreeMap<i64, usize> = BTreeMap::new();
    let (mut segs_total, mut segs_matched, mut segs_other) = (0usize, 0usize, 0usize);
    let mut path_lines = Vec::new();
    for (pi, cp) in report["critical_paths"].as_array().into_iter().flatten().enumerate() {
        let (mut total, mut routing_np, mut routing_ox, mut complete) = (0i64, 0i64, 0i64, true);
        for s in cp["path"].as_array().into_iter().flatten() {
            let d = (s["delay"].as_f64().unwrap_or(0.0) * 1000.0).round() as i64;
            total += d;
            if s["type"].as_str() != Some("routing") {
                continue;
            }
            segs_total += 1;
            routing_np += d;
            let net = s["net"].as_str().unwrap_or("");
            let loc = (s["to"]["loc"][0].as_i64().unwrap_or(-1), s["to"]["loc"][1].as_i64().unwrap_or(-1));
            let root = root_of.get(net).copied();
            let cands = root.map(|r| np.at_loc(r, loc, &wires)).unwrap_or_default();
            let best = cands.iter().min_by_key(|c| (d - c.2).abs()).cloned();
            let mut line = json!({"path": pi, "net": net, "loc": [loc.0, loc.1], "port": s["to"]["port"], "reported_ps": d,
                                   "nextpnr_model": best.as_ref().map(|b| b.2), "sink": best.as_ref().map(|b| b.1.clone())});
            match &best {
                Some(b) => {
                    segs_matched += 1;
                    *offsets.entry(d - b.2).or_default() += 1;
                    if trace {
                        if let (Some(r), Some(&t)) = (root, root.and_then(|r| np.by_root.get(&r))) {
                            trace_arc(&g, &np.trees[t], r, b.0, net, d);
                        }
                    }
                }
                None => complete = false,
            }
            if let (Some(o), Some(r), Some(b)) = (ox.as_mut(), root.and_then(|r| other_root.get(&r).copied()), &best) {
                let oc = o.at_loc(r, loc, &wires);
                let pick = oc.iter().find(|c| c.0 == b.0).or_else(|| oc.iter().max_by_key(|c| c.2)).cloned();
                match pick {
                    Some(p) => {
                        segs_other += 1;
                        let t = p.2 + (d - b.2);
                        routing_ox += t;
                        line["other_ps"] = json!(t);
                        line["other_sink"] = json!(p.1);
                    }
                    None => complete = false,
                }
            } else if ox.is_some() {
                complete = false;
            }
            println!("{line}");
        }
        let mut pl = json!({"path": pi, "from": cp["from"], "to": cp["to"], "nextpnr_ps": total, "nextpnr_routing_ps": routing_np});
        if ox.is_some() {
            pl["other_ps"] = if complete { json!(total - routing_np + routing_ox) } else { Value::Null };
            pl["other_routing_ps"] = json!(routing_ox);
        }
        println!("{pl}");
        path_lines.push(pl);
    }

    // Every net both routings share a root: sum and max of sink delays.
    let mut totals = json!(null);
    if let Some(o) = ox.as_mut() {
        let (mut sum_np, mut sum_ox, mut max_np, mut max_ox, mut nets_n, mut pips_np, mut pips_ox) = (0u64, 0u64, 0u64, 0u64, 0usize, 0usize, 0usize);
        let roots: Vec<(u32, usize)> = np.by_root.iter().map(|(&r, &t)| (r, t)).collect();
        for (r, t) in roots {
            if np.trees[t].class != NetClass::Signal {
                continue;
            }
            let Some(&u) = other_root.get(&r).and_then(|r| o.by_root.get(r)) else { continue };
            nets_n += 1;
            pips_np += np.trees[t].edges.len();
            pips_ox += o.trees[u].edges.len();
            let sinks_np = np.trees[t].sinks.clone();
            let a = np.arrivals(t);
            for s in &sinks_np {
                let v = a.get(s).copied().unwrap_or(0);
                sum_np += v;
                max_np = max_np.max(v);
            }
            let sinks_ox = o.trees[u].sinks.clone();
            let b = o.arrivals(u);
            for s in &sinks_ox {
                let v = b.get(s).copied().unwrap_or(0);
                sum_ox += v;
                max_ox = max_ox.max(v);
            }
        }
        totals = json!({"signal_nets": nets_n, "edges": [pips_np, pips_ox], "sink_delay_sum_ps": [sum_np, sum_ox], "sink_delay_max_ps": [max_np, max_ox]});
    }
    println!(
        "{}",
        json!({"summary": {"model": format!("{model:?}"), "segments": segs_total, "matched": segs_matched, "other_matched": segs_other,
                           "offsets_ps": offsets, "paths": path_lines, "nextpnr_vs_other": totals}})
    );
    Ok(())
}

/// The pips from `root` to `sink` in `tree`, with their timing, on stderr.
fn trace_arc(g: &Graph, tree: &Net, root: u32, sink: u32, net: &str, reported: i64) {
    let mut into: HashMap<u32, (u32, u32, u32)> = HashMap::new();
    for e in &tree.edges {
        into.insert(e.dst, (e.src, e.tile, e.pi));
    }
    let mut n = sink;
    let mut steps = Vec::new();
    while let Some(&(src, tile, pi)) = into.get(&n) {
        let tm = g.edge_timing(tile, pi);
        steps.push(json!({"pip": g.edge_pip(tile, pi).name(), "kind": format!("{:?}", g.edge_kind(tile, pi)), "delay": tm.delay, "in_cap": tm.in_cap,
            "out_res": tm.out_res, "buffered": tm.buffered, "src_rc": g.node_rc(src).map(|r| [r.res, r.cap]), "dst_rc": g.node_rc(n).map(|r| [r.res, r.cap]),
            "fanout_from_src": tree.edges.iter().filter(|e| e.src == src).count()}));
        if n == root {
            break;
        }
        n = src;
    }
    steps.reverse();
    eprintln!("{}", json!({"trace": net, "reported": reported, "steps": steps}));
}
