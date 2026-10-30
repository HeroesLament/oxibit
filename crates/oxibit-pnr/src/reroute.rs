//! Route an already placed and routed design again, on its own placement:
//! recover its nets from its FASM, strip the signal and constant routing,
//! route those nets with `Router`, and write the design back with the new
//! routing. Clock nets and default tie-offs stay as the design had them.
//!
//! The result is checked over the new FASM alone (`check`), the same way the
//! design's own FASM is: every sink reached, no node driven from two
//! sources.

use std::collections::{BTreeMap, HashMap, HashSet};

use oxibit_fasm::route::{EdgeKind, Graph, PipLookup, pip_lines};
use oxibit_report::{Event, PhaseState, Progress};
use serde_json::{Value, json};

use crate::nets::{self, Net, NetClass};
use crate::router::{Opts, Router, Stats};

/// Outcome of `run`.
pub struct Reroute {
    pub fasm: String,
    pub stats: Stats,
    /// Per net class: (nets, sinks).
    pub classes: BTreeMap<String, (usize, usize)>,
    pub pips_removed: usize,
    pub pips_added: usize,
    /// The check on the design's own FASM, then on the new one.
    pub check_design: Check,
    pub check_new: Check,
}

#[derive(Debug, Clone, Default)]
pub struct Check {
    pub sinks: usize,
    pub sinks_missing: usize,
    /// (net source, sink) of every sink not reached.
    pub missing: Vec<(u32, u32)>,
    pub shorts: HashSet<u32>,
}

impl Check {
    pub fn to_json(&self) -> Value {
        json!({"sinks": self.sinks, "sinks_missing": self.sinks_missing, "shorts": self.shorts.len()})
    }
}

impl Reroute {
    /// Shorts in the new routing that the design's own routing does not have.
    pub fn new_shorts(&self) -> usize {
        self.check_new.shorts.difference(&self.check_design.shorts).count()
    }

    /// The routing is complete and clean.
    pub fn ok(&self) -> bool {
        !self.stats.cancelled
            && self.stats.overused == 0
            && self.stats.unroutable_sinks == 0
            && self.check_new.sinks_missing == 0
            && self.new_shorts() == 0
    }

    pub fn stats_json(&self) -> Value {
        let classes: serde_json::Map<String, Value> =
            self.classes.iter().map(|(k, (n, s))| (k.clone(), json!({"nets": n, "sinks": s}))).collect();
        json!({
            "router": self.stats.to_json(),
            "classes": classes,
            "pips_removed": self.pips_removed,
            "pips_added": self.pips_added,
        })
    }

    pub fn check_json(&self) -> Value {
        json!({"design": self.check_design.to_json(), "new": self.check_new.to_json(), "new_shorts": self.new_shorts()})
    }
}

fn phase(p: &dyn Progress, name: &str, state: PhaseState) {
    p.emit(Event::Phase { phase: name.into(), state, ms: None });
}

fn class_name(c: NetClass) -> &'static str {
    match c {
        NetClass::Signal => "signal",
        NetClass::Constant => "constant",
        NetClass::Default => "default",
        NetClass::Clock => "clock",
        NetClass::GlobalClock => "global_clock",
    }
}

/// Re-route `fasm` on `g` (built with `GraphOpts { defaults: true }`).
pub fn run(g: &Graph, fasm: &str, opts: Opts, p: &dyn Progress) -> Reroute {
    phase(p, "nets", PhaseState::Begin);
    let (nets, un) = nets::from_fasm(g, fasm);
    let mut classes: BTreeMap<String, (usize, usize)> = BTreeMap::new();
    for n in &nets {
        let e = classes.entry(class_name(n.class).into()).or_default();
        e.0 += 1;
        e.1 += n.sinks.len();
    }
    for l in &un.missing {
        p.emit(Event::Warn { code: "pip_unresolved".into(), msg: format!("{l} is not a usable pip in the graph") });
    }
    if !un.multi_driven.is_empty() {
        p.emit(Event::Warn { code: "multi_driven".into(), msg: format!("{} nodes have two listed drivers in the design", un.multi_driven.len()) });
    }
    p.emit(Event::progress("nets", json!({"nets": nets.len(), "site_lines": un.not_pips})));
    phase(p, "nets", PhaseState::End);

    phase(p, "route", PhaseState::Begin);
    let mut router = Router::new(g, opts);
    let (routes, stats) = router.run_with(&nets, p);
    phase(p, "route", PhaseState::End);

    phase(p, "fasm", PhaseState::Begin);
    let routed = |n: &Net| matches!(n.class, NetClass::Signal | NetClass::Constant);
    let drop: HashSet<String> = nets.iter().filter(|n| routed(n)).flat_map(|n| n.edges.iter().map(|e| g.edge_pip(e.tile, e.pi).name())).collect();
    let ours: Vec<String> = routes.iter().flatten().flat_map(|h| g.edge_pip(h.tile, h.pi).fasm_lines()).collect();
    let mut out = String::with_capacity(fasm.len());
    let mut removed = 0;
    for line in fasm.lines() {
        if drop.contains(line.split('#').next().unwrap_or("").trim()) {
            removed += 1;
            continue;
        }
        out.push_str(line);
        out.push('\n');
    }
    for l in &ours {
        out.push_str(l);
        out.push('\n');
    }
    phase(p, "fasm", PhaseState::End);

    phase(p, "check", PhaseState::Begin);
    let check_design = check(g, &nets, fasm);
    let check_new = check(g, &nets, &out);
    phase(p, "check", PhaseState::End);

    Reroute { fasm: out, stats, classes, pips_removed: removed, pips_added: ours.len(), check_design, check_new }
}

/// Over a FASM alone: from each net's source, follow the pips the FASM sets
/// and hard wiring, where a hard edge into a node a set pip also drives is a
/// mux's unselected input and does not conduct. Clock nets also cross the
/// clock buffers the FASM configures (`Graph::trace` with relays).
pub fn check(g: &Graph, nets: &[Net], fasm: &str) -> Check {
    let on: HashSet<String> = pip_lines(fasm);
    // Every bare line: a clock buffer relay's lines are site settings.
    let mut all: HashSet<String> = fasm
        .lines()
        .map(|l| l.split('#').next().unwrap_or("").trim())
        .filter(|f| !f.is_empty() && !f.contains('=') && !f.contains('['))
        .map(String::from)
        .collect();
    // An I/O site route-through on a top SING IOI tile is written for its
    // site's `_Y1` (`iocfg::IoCfg::route_line`); the graph names it `_Y0`.
    let sing: Vec<String> = all
        .iter()
        .filter(|l| l.contains("IOI") && l.contains("_SING_"))
        .filter_map(|l| {
            let (tile, c) = l.split_once('.')?;
            c.contains("Y1").then(|| format!("{tile}.{}", c.replacen("Y1", "Y0", 1)))
        })
        .collect();
    all.extend(sing);
    let set_into: HashSet<u32> = on
        .iter()
        .filter_map(|l| match g.lookup_pip(l) {
            PipLookup::Edge { dst, tile, pi, .. } if g.edge_kind(tile, pi) == EdgeKind::Feature => Some(dst),
            _ => None,
        })
        .collect();
    let mut owner: HashMap<u32, usize> = HashMap::new();
    let mut c = Check::default();
    for (i, n) in nets.iter().enumerate() {
        if n.class == NetClass::Default {
            continue;
        }
        let nodes = if matches!(n.class, NetClass::Clock | NetClass::GlobalClock) { g.trace(n.source, &all, true).0 } else { reach(g, n.source, &on, &all, &set_into) };
        c.sinks += n.sinks.len();
        for &s in n.sinks.iter().filter(|s| !nodes.contains(s)) {
            c.sinks_missing += 1;
            c.missing.push((n.source, s));
        }
        for m in nodes {
            match owner.get(&m) {
                Some(&j) if j != i => {
                    c.shorts.insert(m);
                }
                Some(_) => {}
                None => {
                    owner.insert(m, i);
                }
            }
        }
    }
    c
}

/// Nodes `src` reaches over set pips, fixed edges no set pip overrides,
/// and the I/O site route-throughs whose settings (in `lines`) are set.
fn reach(g: &Graph, src: u32, on: &HashSet<String>, lines: &HashSet<String>, set_into: &HashSet<u32>) -> HashSet<u32> {
    let mut seen = HashSet::from([src]);
    let mut stack = vec![src];
    while let Some(n) = stack.pop() {
        for &(d, tile, pi) in g.out_edges(n) {
            let conducts = match g.edge_kind(tile, pi) {
                EdgeKind::Feature => on.contains(&g.edge_pip(tile, pi).name()),
                EdgeKind::Fixed => !set_into.contains(&d),
                // A tie's default into an interconnect input no pip sets.
                EdgeKind::Default => n == src && !set_into.contains(&d),
                EdgeKind::Relay => g.edge_site_through(tile, pi) && g.relay_conducts(tile, pi, lines),
            };
            if conducts && seen.insert(d) {
                stack.push(d);
            }
        }
    }
    seen
}
