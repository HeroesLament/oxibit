//! Splice one routing into another: a base FASM with some of its signal
//! nets routed the way a donor FASM of the same placement routes them. For
//! bisecting a routing that fails on silicon against one that works: each
//! step swaps half of the remaining candidates and the board says which
//! half holds the fault.
//!
//! cargo run --release -p oxibit-pnr --example splice -- \
//!     DB_ROOT PART BASE.fasm DONOR.fasm FROM TO OUT.fasm
//!
//! Both FASMs' nets are recovered from their pips (`nets::from_fasm`),
//! after dropping every `always` pseudo-pip line (a hard edge: no bits).
//! One tool writes those at the ends of a route and the other does not,
//! so without them both trees start and end at the same interconnect
//! nodes, and the output's bits are unchanged. A net is matched by its
//! sinks. A candidate is a signal
//! net with the same sinks in both whose donor routing uses no node a base
//! net that stays uses: so a splice never shorts two nets, whichever candidates it
//! takes. Candidates are numbered in source-node name order; the splice
//! takes those numbered FROM to TO (exclusive). The base's pips of those
//! nets go, the donor's come in; every other line (cell settings, LUT
//! INITs, site muxes) is the base's. The donor must keep LUT pins as
//! placed (`standalone` with OXIBIT_PERM=none) for its INITs to match.
//!
//! stdout: one JSON line: nets in each, candidates (and why the others are
//! not), the range taken, and the result checked over its own FASM
//! (`reroute::check`: sinks missing, shorts). Exit 1 if that check fails.

use std::collections::{BTreeMap, HashMap, HashSet};
use std::path::Path;
use std::process::ExitCode;

use oxibit_fasm::db::Database;
use oxibit_fasm::route::{EdgeKind, Graph, GraphOpts, PipLookup};
use oxibit_pnr::nets::{self, Net, NetClass};
use oxibit_pnr::reroute;
use serde_json::json;

type R<T> = Result<T, String>;

/// Nodes a net's routing touches: its source and every pip's ends.
fn nodes(n: &Net) -> HashSet<u32> {
    std::iter::once(n.source).chain(n.edges.iter().flat_map(|e| [e.src, e.dst])).collect()
}

/// `fasm` without its `always` pseudo-pips (hard edges, no bits), and how
/// many lines went.
fn strip_always(g: &Graph, fasm: &str) -> (String, usize) {
    let mut out = String::with_capacity(fasm.len());
    let mut dropped = 0;
    for line in fasm.lines() {
        let f = line.split('#').next().unwrap_or("").trim();
        if let PipLookup::Edge { tile, pi, .. } = g.lookup_pip(f)
            && g.edge_kind(tile, pi) == EdgeKind::Fixed
        {
            dropped += 1;
            continue;
        }
        out.push_str(line);
        out.push('\n');
    }
    (out, dropped)
}

fn sorted_sinks(n: &Net) -> Vec<u32> {
    let mut s = n.sinks.clone();
    s.sort_unstable();
    s.dedup();
    s
}

fn main() -> ExitCode {
    match run() {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::from(1),
        Err(e) => {
            eprintln!("splice: {e}");
            ExitCode::from(3)
        }
    }
}

fn run() -> R<bool> {
    let args: Vec<String> = std::env::args().collect();
    let [_, db_root, part, base_path, donor_path, from, to, out_path] = &args[..] else {
        return Err("usage: splice DB_ROOT PART BASE.fasm DONOR.fasm FROM TO OUT.fasm".into());
    };
    let from: usize = from.parse().map_err(|e| format!("FROM: {e}"))?;
    let to: usize = to.parse().map_err(|e| format!("TO: {e}"))?;
    let read = |p: &str| std::fs::read_to_string(p).map_err(|e| format!("{p}: {e}"));
    let db = Database::open(Path::new(db_root), part)?;
    let g = Graph::build_with(&db, None, GraphOpts { defaults: true })?;
    let (base, always_base) = strip_always(&g, &read(base_path)?);
    let (donor, always_donor) = strip_always(&g, &read(donor_path)?);

    let (bnets, _) = nets::from_fasm(&g, &base);
    let (dnets, _) = nets::from_fasm(&g, &donor);
    let by_sinks: HashMap<Vec<u32>, &Net> = dnets.iter().filter(|n| n.class == NetClass::Signal).map(|n| (sorted_sinks(n), n)).collect();

    // Base node -> how many base nets use it.
    let mut use_count: HashMap<u32, u32> = HashMap::new();
    let bnodes: Vec<HashSet<u32>> = bnets.iter().map(nodes).collect();
    for ns in &bnodes {
        for &n in ns {
            *use_count.entry(n).or_default() += 1;
        }
    }

    // Candidates: matched signal nets, same sinks. Collisions with the base
    // are settled below, once the set to swap is known.
    let mut why: BTreeMap<&str, usize> = BTreeMap::new();
    let mut cands: Vec<(String, usize, &Net)> = Vec::new();
    for (i, b) in bnets.iter().enumerate() {
        if b.class != NetClass::Signal {
            continue;
        }
        let Some(d) = by_sinks.get(&sorted_sinks(b)) else {
            *why.entry("no_donor_net").or_default() += 1;
            continue;
        };
        if nodes(b) == nodes(d) {
            *why.entry("same_route").or_default() += 1;
            continue;
        }
        cands.push((g.node_name(b.source), i, d));
    }
    cands.sort_by(|a, b| a.0.cmp(&b.0));

    // Take FROM..TO, each only if its donor nodes are free of every base
    // net that stays (nets already taken free theirs).
    let mut taken: Vec<usize> = Vec::new();
    let mut taken_nodes: HashSet<u32> = HashSet::new();
    let mut collided = 0usize;
    for (_, i, d) in cands.iter().skip(from).take(to.saturating_sub(from)) {
        for &n in &bnodes[*i] {
            *use_count.get_mut(&n).expect("counted") -= 1;
        }
        let dn = nodes(d);
        let clash = dn.iter().any(|n| use_count.get(n).is_some_and(|&c| c > 0) || taken_nodes.contains(n));
        if clash {
            for &n in &bnodes[*i] {
                *use_count.get_mut(&n).expect("counted") += 1;
            }
            collided += 1;
            continue;
        }
        taken_nodes.extend(dn);
        taken.push(*i);
    }

    // The base's pips of the taken nets go; the donor's come in.
    let drop: HashSet<String> = taken.iter().flat_map(|&i| bnets[i].edges.iter().map(|e| g.edge_pip(e.tile, e.pi).name())).collect();
    let mut out = String::with_capacity(base.len());
    let mut removed = 0usize;
    for line in base.lines() {
        if drop.contains(line.split('#').next().unwrap_or("").trim()) {
            removed += 1;
            continue;
        }
        out.push_str(line);
        out.push('\n');
    }
    let mut added = 0usize;
    for &i in &taken {
        let d = by_sinks[&sorted_sinks(&bnets[i])];
        for e in &d.edges {
            for l in g.edge_pip(e.tile, e.pi).fasm_lines() {
                out.push_str(&l);
                out.push('\n');
                added += 1;
            }
        }
    }
    std::fs::write(out_path, &out).map_err(|e| format!("{out_path}: {e}"))?;

    let check = reroute::check(&g, &bnets, &out);
    let base_check = reroute::check(&g, &bnets, &base);
    let new_shorts = check.shorts.difference(&base_check.shorts).count();
    let ok = check.sinks_missing == base_check.sinks_missing && new_shorts == 0;
    println!(
        "{}",
        json!({
            "always_dropped": [always_base, always_donor],
            "base_nets": bnets.len(), "donor_nets": dnets.len(), "candidates": cands.len(), "not_candidates": why,
            "range": [from, to], "taken": taken.len(), "collided": collided,
            "pips_removed": removed, "lines_added": added,
            "check": check.to_json(), "base_check": base_check.to_json(), "new_shorts": new_shorts, "ok": ok, "out": out_path,
        })
    );
    Ok(ok)
}
