//! Negotiated-congestion routing (PathFinder) over oxibit's routing graph.
//!
//! Every node carries one net. Nets are routed one sink at a time with A*
//! from the net's whole tree so far; a node another net uses costs more each
//! iteration (present congestion), and a node that stays contested gets
//! more expensive for good (history). Nets that touch an overused node are
//! ripped up and routed again until no node is shared.
//!
//! A node's fixed fan-out (hard wires, `always` pseudo-pips) is driven
//! whenever the node is, so it counts as used by the net too.
//!
//! A net's `pin_groups` are sinks that any one node of the group satisfies:
//! the inputs of a LUT, which the LUT's INIT can be permuted to read in any
//! order. The search ends on whichever member it reaches first, and the
//! members compete for nets like any other node, so congestion decides
//! which net gets which pin.
//!
//! The present factor follows `Opts::schedule`, which always has a
//! ceiling, and the A* estimate is `Opts::estimate`: a per-tile distance
//! term (`Bound::Greedy` above `ADMISSIBLE_PER_TILE`) plus the target's
//! unavoidable congestion cost of entry (beyond the uncongested cost the
//! distance term stands for), a lower bound on its last
//! hops. Together they keep a search for a sink behind a contested entrance
//! from clearing the whole cheaper device first.
//!
//! With `Opts::delay_weight` w > 0 an edge costs, as in VPR,
//! `(1 - w) * congestion + w * delay / delay_norm`: the congestion term as
//! above, and the pip's intrinsic delay (`Graph::edge_timing`, the model
//! `delay` checks against nextpnr). w = 0 is congestion alone. A sink's
//! search then starts from each tree node at `w * arrival / delay_norm`,
//! its delay from the source, not 0: the sink joins the tree where it
//! arrives first, not where the tree is nearest.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::time::Instant;

use oxibit_fasm::route::{EdgeKind, Fabric, Graph};
use oxibit_report::{Event, Progress, Silent};
use serde_json::json;

use crate::nets::{Driver, Identity, Logic, Net, NetClass};

/// The present-congestion factor over iterations. Every variant has a
/// ceiling: unbounded, the cost of crossing a contested node outgrows the
/// A* estimate until one-sink searches flood the device.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Schedule {
    /// p = min(p0 * growth^(iter - 1), cap).
    Geometric { p0: f32, growth: f32, cap: f32 },
}

impl Schedule {
    pub fn first(&self) -> f32 {
        match *self {
            Schedule::Geometric { p0, cap, .. } => p0.min(cap),
        }
    }
    pub fn next(&self, p: f32) -> f32 {
        match *self {
            Schedule::Geometric { growth, cap, .. } => (p * growth).min(cap),
        }
    }
}

/// What an estimate guarantees about the cost still to pay.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Bound {
    /// Never more than the true remaining cost: searches find cheapest paths.
    Lower,
    /// May exceed it, by up to the given factor over the lower bound per
    /// tile: faster, paths possibly dearer.
    Greedy(f32),
}

/// Cost per tile no path can beat: the longest wire (an 18-tile long line)
/// costs 1 like any node.
pub const ADMISSIBLE_PER_TILE: f32 = 1.0 / 18.0;

/// The A* estimate: `per_tile` x Manhattan distance to the target, plus,
/// with `entry`, the congestion cost the target's entry cannot avoid
/// (a lower bound on the last hops, `Router::entry_bound`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Estimate {
    pub per_tile: f32,
    pub entry: bool,
}

impl Estimate {
    pub fn bound(&self) -> Bound {
        if self.per_tile <= ADMISSIBLE_PER_TILE {
            Bound::Lower
        } else {
            Bound::Greedy(self.per_tile / ADMISSIBLE_PER_TILE)
        }
    }
}

/// The end of a negotiation, when few nodes are left overused: the nets
/// on them keep trading the same wires because the nets that block their
/// alternatives are never ripped up. Once at most `overuse` nodes are
/// overused, every signal net with a node within `radius` tiles of one is
/// rerouted too, its branches through that area dropped, so the
/// neighbourhood can rearrange.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Endgame {
    pub overuse: usize,
    pub radius: i16,
}

#[derive(Debug, Clone)]
pub struct Opts {
    pub max_iters: u32,
    /// `None`: only the nets on overused nodes are rerouted, to the end.
    pub endgame: Option<Endgame>,
    pub schedule: Schedule,
    /// History added per unit of overuse per iteration.
    pub hist_fac: f32,
    pub estimate: Estimate,
    /// Weight of delay against congestion in an edge's cost, 0..=1.
    pub delay_weight: f32,
    /// ps of delay per cost unit.
    pub delay_norm: f32,
    /// A* weight of the delay term (delay cost units per tile).
    pub astar_delay: f32,
    /// A sink is searched for inside the bounding box of its net's
    /// terminals grown by this many tiles, and without the box only when
    /// none is found there. A net rerouted again has its margin doubled,
    /// so a net that stays congested soon searches unbounded. Negative:
    /// no box.
    pub bbox_margin: i32,
    /// Aim the A* heuristic at where a sink is entered from the routing
    /// (the configurable muxes its fixed chain leads back to), not at the
    /// sink node's own tile: a PS7 pin's node runs across the PSS tiles,
    /// tens of tiles from its interconnect.
    /// Off by default: on the PHY placement it cut iteration time 6-12%
    /// but left the single-sink floods; on spi_probe it doubled the pops of
    /// iteration 1.
    pub entry_aim: bool,
    /// After routing, write the nodes still used by two or more nets
    /// (name, occupancy, history) to this file.
    pub overuse_dump: Option<String>,
}

impl Default for Opts {
    fn default() -> Self {
        Opts {
            max_iters: 200,
            // picoBEAM (onehot, engine floorplan): without it 3-13 nodes
            // traded places from iteration 11 to 200.
            endgame: Some(Endgame {
                overuse: 64,
                radius: 3,
            }),
            // Cap 64: on the PHY, cap 8 settles into 1-3 overused nodes
            // that trade places for good and cap 1000 oscillates; 64
            // converges (0 at iteration 151 unclaimed, 30 with claims).
            schedule: Schedule::Geometric {
                p0: 0.5,
                growth: 1.5,
                cap: 64.0,
            },
            hist_fac: 1.0,
            estimate: Estimate {
                per_tile: 0.6,
                entry: true,
            },
            delay_weight: 0.0,
            delay_norm: 100.0,
            astar_delay: 0.3,
            bbox_margin: -1,
            entry_aim: false,
            overuse_dump: None,
        }
    }
}

/// One routed edge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Hop {
    pub src: u32,
    pub dst: u32,
    pub tile: u32,
    pub pi: u32,
}

#[derive(Debug, Default, Clone)]
pub struct Stats {
    pub iterations: u32,
    pub overused: usize,
    pub routed_nets: usize,
    pub unroutable_sinks: usize,
    pub heap_pops: u64,
    pub seconds: f64,
    /// Stopped early through `Progress::cancelled`.
    pub cancelled: bool,
    /// Nodes claimed for one net (see `Claim`).
    pub claimed_nodes: usize,
}

impl Stats {
    pub fn to_json(&self) -> serde_json::Value {
        json!({
            "iterations": self.iterations,
            "overused": self.overused,
            "routed_nets": self.routed_nets,
            "unroutable_sinks": self.unroutable_sinks,
            "heap_pops": self.heap_pops,
            "seconds": self.seconds,
            "cancelled": self.cancelled,
            "claimed_nodes": self.claimed_nodes,
        })
    }
}

#[derive(Clone, Copy)]
struct Item {
    f: f32,
    node: u32,
}
impl PartialEq for Item {
    fn eq(&self, o: &Self) -> bool {
        self.f == o.f
    }
}
impl Eq for Item {}
impl PartialOrd for Item {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for Item {
    fn cmp(&self, o: &Self) -> Ordering {
        o.f.total_cmp(&self.f) // min-heap
    }
}

/// The pin of each of `net`'s pin groups that `hops` reach, in group order;
/// `None` for a group the routing does not reach.
pub fn chosen_pins(net: &Net, hops: &[Hop]) -> Vec<Option<u32>> {
    let reached: HashSet<u32> = hops.iter().map(|h| h.dst).collect();
    net.pin_groups
        .iter()
        .map(|g| g.iter().copied().find(|p| reached.contains(p)))
        .collect()
}

pub struct Router<'g> {
    g: &'g Graph,
    opts: Opts,
    /// Electrical nets using each node: once per signal net, once per
    /// constant value however many tie-driven nets of it share the node.
    occ: Vec<u16>,
    /// Tie-driven nets of each value (`Logic::index`) on each node.
    const_on: Vec<[u16; 2]>,
    /// The value of the constant net being routed, if it is one.
    cur_const: Option<Logic>,
    hist: Vec<f32>,
    /// Nodes no routed net may enter (clock and default nets, kept as is).
    reserved: Vec<bool>,
    /// Nodes only one net may enter (see `Claim`).
    claim: Vec<Claim>,
    /// Nodes some configurable pip drives. A hard edge into such a node is
    /// a mux's default input (DSP_FAN -> DSP_1_D17 next to the GND and VCC
    /// tie pips), not a wire, so fixed fan-out stops there.
    muxed: Vec<bool>,
    // Search scratch.
    cost: Vec<f32>,
    prev: Vec<(u32, u32, u32)>, // (from node, tile, pi); from == u32::MAX at a start node
    touched: Vec<u32>,
    /// Net-local tree membership: tag[n] == current net tag.
    tag: Vec<u32>,
    cur_tag: u32,
    /// The costliest single search of the last routed net.
    diag: SearchDiag,
    /// OXIBIT_DUMP_NET: write every pop of this net's searches (debug).
    dump_net: Vec<usize>,
    cur_net: usize,
}

/// One target search: how far it wandered and what it paid.
#[derive(Debug, Clone, Copy, Default)]
pub struct SearchDiag {
    pub pops: u64,
    /// Heuristic at the start (astar x tiles to the target).
    pub h0: f32,
    /// Path cost at the target (infinite: not reached).
    pub cost: f32,
    /// Farthest popped node from the target, in tiles.
    pub far: i32,
    /// Tiles from the nearest tree node to the target.
    pub dist: i32,
    /// Popped nodes another net already uses.
    pub congested_pops: u64,
    pub hops: usize,
    /// Pops by tiles from the target: 0-2, 3-5, 6-10, 11-20, 21-40, 41+.
    pub by_dist: [u64; 6],
    /// Pops by f (heuristic plus cost) relative to the reached cost: <0.5x, <1x, <2x, >=2x.
    pub by_f: [u64; 4],
}

impl SearchDiag {
    pub fn to_json(&self) -> serde_json::Value {
        json!({"pops": self.pops, "h0": self.h0, "cost": if self.cost.is_finite() { json!(self.cost) } else { json!(null) },
               "far": self.far, "dist": self.dist, "congested_pops": self.congested_pops, "hops": self.hops,
               "by_dist": self.by_dist, "by_f": self.by_f})
    }
}

const NONE: u32 = u32::MAX;

/// Who may enter a node: a node whose fixed fan-out drives a net's fixed
/// sink belongs to that net. Driving it for another net would drive the
/// pin too: `FAN_ALT7 -> FAN7 -> CLBLL_LL_CE` with `FAN_BOUNCE7` off the
/// same node, so a route bouncing through `FAN_BOUNCE7` shorts the CE pin
/// it passes. Negotiation finds this only by failing on it; the claim
/// keeps other nets off from the start.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Claim {
    Free,
    Net(Identity),
    /// Two nets' sinks behind one node: left to negotiation.
    Contested,
}

impl<'g> Router<'g> {
    pub fn new(g: &'g Graph, opts: Opts) -> Self {
        let n = g.nodes();
        let mut muxed = vec![false; n];
        for m in 0..n as u32 {
            for &(d, tile, pi) in g.out_edges(m) {
                if g.edge_kind(tile, pi) == EdgeKind::Feature {
                    muxed[d as usize] = true;
                }
            }
        }
        Router {
            muxed,
            g,
            opts,
            occ: vec![0; n],
            const_on: vec![[0; 2]; n],
            cur_const: None,
            hist: vec![0.0; n],
            reserved: vec![false; n],
            claim: vec![Claim::Free; n],
            cost: vec![f32::INFINITY; n],
            prev: vec![(NONE, 0, 0); n],
            touched: Vec::new(),
            tag: vec![0; n],
            cur_tag: 0,
            diag: SearchDiag::default(),
            dump_net: std::env::var("OXIBIT_DUMP_NET")
                .map(|v| v.split(',').filter_map(|x| x.parse().ok()).collect())
                .unwrap_or_default(),
            cur_net: usize::MAX,
        }
    }

    /// Keep every net off `nodes` (as off a kept net's).
    pub fn forbid(&mut self, nodes: impl IntoIterator<Item = u32>) {
        for n in nodes {
            self.reserved[n as usize] = true;
        }
    }

    /// Count node `n` as used by a net of identity `id`.
    fn occupy(&mut self, id: Identity, n: u32) {
        let n = n as usize;
        match id {
            Identity::Own(_) => self.occ[n] += 1,
            Identity::Const(v) => {
                let c = &mut self.const_on[n][v.index()];
                if *c == 0 {
                    self.occ[n] += 1;
                }
                *c += 1;
            }
        }
    }

    /// Undo `occupy`.
    fn vacate(&mut self, id: Identity, n: u32) {
        let n = n as usize;
        match id {
            Identity::Own(_) => self.occ[n] -= 1,
            Identity::Const(v) => {
                let c = &mut self.const_on[n][v.index()];
                *c -= 1;
                if *c == 0 {
                    self.occ[n] -= 1;
                }
            }
        }
    }

    /// Electrical nets on `n` other than the one being routed.
    fn others(&self, n: u32) -> u16 {
        let o = self.occ[n as usize];
        match self.cur_const {
            Some(v) if self.const_on[n as usize][v.index()] > 0 => o - 1,
            _ => o,
        }
    }

    /// The node and everything its fixed fan-out drives.
    fn closure(&self, n: u32, out: &mut Vec<u32>, seen: &mut HashSet<u32>) {
        let mut stack = vec![n];
        while let Some(m) = stack.pop() {
            if !seen.insert(m) {
                continue;
            }
            out.push(m);
            for &(d, tile, pi) in self.g.out_edges(m) {
                if self.g.edge_kind(tile, pi) == EdgeKind::Fixed
                    && !self.muxed[d as usize]
                    && !seen.contains(&d)
                {
                    stack.push(d);
                }
            }
        }
    }

    /// Nodes a net occupies: its source, its sinks, every node it routes
    /// through, and the fixed fan-out of the source and of every node it
    /// routes through. A sink's own fixed fan-out is not counted: a site pin
    /// behind it can be a mux whose other input is a constant (a DSP
    /// control pin tied in the tile), which nextpnr uses as such. A pin
    /// group's members are occupied only once a hop reaches one.
    #[oxibit_prof::hot(name = "Router::footprint")]
    fn footprint(&self, net: &Net, hops: &[Hop]) -> Vec<u32> {
        let mut out = Vec::new();
        let mut seen = HashSet::new();
        match &net.driver {
            // The ties a route leaves from count once it does (below).
            Driver::Ties(_) => {}
            // A tie-off source's hard fan-out (DSP_GND_L to every DSP
            // control pin) is a default the pin's own routing overrides.
            Driver::Node(s) if net.class == NetClass::Constant => {
                seen.insert(*s);
                out.push(*s);
            }
            Driver::Node(s) => self.closure(*s, &mut out, &mut seen),
        }
        if let Driver::Ties(_) = &net.driver {
            for h in hops {
                if net.driver.contains(h.src) && seen.insert(h.src) {
                    out.push(h.src);
                }
            }
        }
        for &s in &net.sinks {
            if seen.insert(s) {
                out.push(s);
            }
        }
        let mut group_pins: Vec<u32> = net.pin_groups.iter().flatten().copied().collect();
        group_pins.sort_unstable();
        for h in hops {
            let terminal =
                net.sinks.binary_search(&h.dst).is_ok() || group_pins.binary_search(&h.dst).is_ok();
            if !terminal {
                self.closure(h.dst, &mut out, &mut seen);
            } else if seen.insert(h.dst) {
                out.push(h.dst);
            }
        }
        out
    }

    fn node_cost(&self, n: u32, pres: f32) -> f32 {
        let others = self.others(n) as f32;
        (1.0 + self.hist[n as usize]) * (1.0 + pres * others)
    }

    /// Congestion part of entering `n` during the current net's search: its
    /// cost as `route_net` charges it, less the uncongested cost of 1 that
    /// the distance term already stands for (tree nodes are free).
    fn enter_excess(&self, n: u32, pres: f32, tag: u32) -> f32 {
        if self.tag[n as usize] == tag {
            0.0
        } else {
            self.node_cost(n, pres) - 1.0
        }
    }

    /// Lower bound on the congestion cost still to pay, beyond the
    /// uncongested cost the distance term covers, to reach a node of `set`:
    /// 0 on the set, the target's own excess on its drivers, two hops'
    /// worth on theirs; every other node pays at least three hops' worth
    /// (`far`). Delay terms are ignored (a lower bound).
    fn entry_bound(&self, set: &[u32], pres: f32, tag: u32) -> (HashMap<u32, f32>, f32) {
        let mut near: HashMap<u32, f32> = HashMap::new();
        let mut far = f32::INFINITY;
        for &t in set {
            near.insert(t, 0.0);
        }
        for &t in set {
            let ct = self.enter_excess(t, pres, tag);
            for (p1, _, _) in self.g.in_edges(t) {
                let c1 = self.enter_excess(p1, pres, tag);
                let e1 = near.entry(p1).or_insert(f32::INFINITY);
                *e1 = e1.min(ct);
                let mut any = false;
                for (p2, _, _) in self.g.in_edges(p1) {
                    any = true;
                    let e2 = near.entry(p2).or_insert(f32::INFINITY);
                    *e2 = e2.min(ct + c1);
                    far = far.min(ct + c1 + self.enter_excess(p2, pres, tag));
                }
                if !any {
                    far = far.min(ct + c1);
                }
            }
        }
        // Set members stay at 0 even when they also drive each other.
        for &t in set {
            near.insert(t, 0.0);
        }
        if !far.is_finite() {
            far = 0.0;
        }
        (near, far)
    }

    /// Tiles from `n` to the nearest of `aims`.
    fn to_aims(&self, n: u32, aims: &[(i16, i16)]) -> f32 {
        let (x, y) = self.g.node_xy(n);
        aims.iter()
            .map(|&(ax, ay)| ((x - ax).abs() + (y - ay).abs()) as i32)
            .min()
            .unwrap_or(0) as f32
    }

    /// Where a target set is entered: the muxed nodes its fixed in-edges
    /// lead back to (the target itself when it is muxed), at most 16.
    fn aims(&self, set: &[u32]) -> Vec<(i16, i16)> {
        if !self.opts.entry_aim {
            return vec![self.g.node_xy(set[0])];
        }
        let mut out: Vec<(i16, i16)> = Vec::new();
        let mut seen: HashSet<u32> = HashSet::new();
        let mut stack: Vec<(u32, u32)> = set.iter().map(|&t| (t, 0)).collect();
        while let Some((n, depth)) = stack.pop() {
            if !seen.insert(n) || out.len() >= 16 {
                continue;
            }
            let mut back = 0;
            if !self.muxed[n as usize] && depth < 8 {
                for (s, tile, pi) in self.g.in_edges(n) {
                    if self.g.edge_kind(tile, pi) == EdgeKind::Fixed {
                        stack.push((s, depth + 1));
                        back += 1;
                    }
                }
            }
            if back == 0 {
                let xy = self.g.node_xy(n);
                if !out.contains(&xy) {
                    out.push(xy);
                }
            }
        }
        // Only a sink entered far from its own tile (a PS7 pin) changes
        // aim: a slice pin one column from its IMUX keeps its own, which
        // gives the search a direction inside the switchbox.
        let own = self.g.node_xy(set[0]);
        let near = |&(x, y): &(i16, i16)| (x - own.0).abs() + (y - own.1).abs() <= 2;
        if out.is_empty() || out.iter().all(near) {
            return vec![own];
        }
        out
    }

    fn manhattan(&self, a: u32, b: u32) -> f32 {
        let (ax, ay) = self.g.node_xy(a);
        let (bx, by) = self.g.node_xy(b);
        ((ax - bx).abs() + (ay - by).abs()) as f32
    }

    fn reset_scratch(&mut self) {
        for &n in &self.touched {
            self.cost[n as usize] = f32::INFINITY;
            self.prev[n as usize] = (NONE, 0, 0);
        }
        self.touched.clear();
    }

    /// The hops of `hops` (a net's routed tree) worth keeping on a reroute:
    /// those on the path from the source to a sink that crosses no node in
    /// `bad` (a node the net shares with another), counting the fixed
    /// fan-out of every node it routes through as the footprint does.
    #[oxibit_prof::hot(name = "Router::keep_hops")]
    fn keep_hops(&self, net: &Net, hops: &[Hop], bad: &HashSet<u32>) -> Vec<Hop> {
        if bad.is_empty() {
            return hops.to_vec();
        }
        let mut terminals: HashSet<u32> = net.sinks.iter().copied().collect();
        terminals.extend(net.pin_groups.iter().flatten().copied());
        // node -> the hop into it.
        let parent: HashMap<u32, usize> =
            hops.iter().enumerate().map(|(k, h)| (h.dst, k)).collect();
        let hop_bad: Vec<bool> = hops
            .iter()
            .map(|h| {
                if bad.contains(&h.dst) {
                    return true;
                }
                if terminals.contains(&h.dst) {
                    return false;
                }
                let (mut out, mut seen) = (Vec::new(), HashSet::new());
                self.closure(h.dst, &mut out, &mut seen);
                out.iter().any(|n| bad.contains(n))
            })
            .collect();
        let mut keep = vec![false; hops.len()];
        for &t in &terminals {
            let mut path = Vec::new();
            let mut n = t;
            let mut ok = true;
            while let Some(&k) = parent.get(&n) {
                if hop_bad[k] {
                    ok = false;
                    break;
                }
                path.push(k);
                n = hops[k].src;
                if path.len() > hops.len() {
                    ok = false; // not a tree
                    break;
                }
            }
            if ok && !path.is_empty() && net.driver.contains(n) {
                for k in path {
                    keep[k] = true;
                }
            }
        }
        hops.iter()
            .zip(&keep)
            .filter(|(_, k)| **k)
            .map(|(h, _)| *h)
            .collect()
    }

    /// Route one net, starting from the hops in `keep` (a subtree from the
    /// source; empty to route from scratch). Its own previous occupancy
    /// must already be removed.
    #[allow(clippy::too_many_arguments)]
    fn route_net(
        &mut self,
        net: &Net,
        id: Identity,
        pres: f32,
        pops: &mut u64,
        outside: &mut u64,
        keep: &[Hop],
        margin: i32,
    ) -> (Vec<Hop>, usize) {
        self.cur_tag += 1;
        self.diag = SearchDiag::default();
        let tag = self.cur_tag;
        self.cur_const = match &net.driver {
            Driver::Ties(t) => Some(t.value()),
            Driver::Node(_) => None,
        };
        let roots = net.driver.nodes();
        let mut tree: Vec<u32> = roots.to_vec();
        for &r in roots {
            self.tag[r as usize] = tag;
        }
        // Delay (ps) from the driver to each tree node.
        let mut arrival: std::collections::HashMap<u32, f32> =
            roots.iter().map(|&r| (r, 0.0)).collect();
        let mut hops: Vec<Hop> = keep.to_vec();
        let mut failed = 0;
        // The kept subtree, its arrivals in order out from the source.
        if !keep.is_empty() {
            let mut out: HashMap<u32, Vec<usize>> = HashMap::new();
            for (k, h) in keep.iter().enumerate() {
                out.entry(h.src).or_default().push(k);
            }
            let mut stack = roots.to_vec();
            while let Some(n) = stack.pop() {
                let a = arrival.get(&n).copied().unwrap_or(0.0);
                for &k in out.get(&n).map(Vec::as_slice).unwrap_or(&[]) {
                    let h = keep[k];
                    if self.tag[h.dst as usize] == tag {
                        continue;
                    }
                    self.tag[h.dst as usize] = tag;
                    tree.push(h.dst);
                    arrival.insert(h.dst, a + self.g.edge_timing(h.tile, h.pi).delay as f32);
                    stack.push(h.dst);
                }
            }
        }

        // Every sink is a set of nodes any one of which completes it: a
        // fixed sink is a set of one. Nearest first.
        let mut targets: Vec<Vec<u32>> = net.sinks.iter().map(|&s| vec![s]).collect();
        targets.extend(net.pin_groups.iter().filter(|g| !g.is_empty()).cloned());
        let anchor = net.driver.anchor();
        targets.sort_by(|a, b| {
            self.manhattan(anchor, a[0])
                .total_cmp(&self.manhattan(anchor, b[0]))
        });

        // The terminals' bounding box, grown by the margin.
        let bbox = (margin >= 0).then(|| {
            let m = margin;
            let (sx, sy) = self.g.node_xy(anchor);
            let (mut x0, mut y0, mut x1, mut y1) = (sx as i32, sy as i32, sx as i32, sy as i32);
            for &t in targets.iter().flatten().chain(roots) {
                let (x, y) = self.g.node_xy(t);
                x0 = x0.min(x as i32);
                y0 = y0.min(y as i32);
                x1 = x1.max(x as i32);
                y1 = y1.max(y as i32);
            }
            (x0 - m, y0 - m, x1 + m, y1 + m)
        });

        for set in &targets {
            if set.iter().any(|&t| self.tag[t as usize] == tag) {
                continue; // already on the tree
            }
            let aims = self.aims(set);
            // A global clock keeps to the clock network all the way to a
            // clock pin. It crosses general interconnect only to reach a
            // sink that is interconnect itself (a clock read as data).
            let dedicated = net.class == NetClass::GlobalClock
                && set
                    .iter()
                    .all(|&t| self.g.node_fabric(t) == Fabric::Dedicated);
            let (near, far) = if self.opts.estimate.entry {
                self.entry_bound(set, pres, tag)
            } else {
                (HashMap::new(), 0.0)
            };
            let eb = |n: u32| near.get(&n).copied().unwrap_or(far);
            let w = self.opts.delay_weight;
            let astar = (1.0 - w) * self.opts.estimate.per_tile + w * self.opts.astar_delay;
            let mut reached = None;
            let mut sd = SearchDiag {
                cost: f32::INFINITY,
                dist: i32::MAX,
                ..Default::default()
            };
            for &t in &tree {
                sd.dist = sd.dist.min(self.to_aims(t, &aims) as i32);
            }
            sd.h0 = astar * sd.dist as f32
                + tree
                    .iter()
                    .map(|&t| eb(t))
                    .fold(f32::INFINITY, f32::min)
                    .min(1e30);
            for boxed in [bbox, None] {
                if boxed.is_none() && bbox.is_some() {
                    // The boxed search found nothing: search again unbounded.
                    self.reset_scratch();
                    *outside += 1;
                }
                let inside = |n: u32| {
                    boxed.is_none_or(|(x0, y0, x1, y1)| {
                        let (x, y) = self.g.node_xy(n);
                        let (x, y) = (x as i32, y as i32);
                        x >= x0 && x <= x1 && y >= y0 && y <= y1
                    })
                };
                let mut heap = BinaryHeap::new();
                for &t in tree
                    .iter()
                    .filter(|&&t| !dedicated || self.g.node_fabric(t) == Fabric::Dedicated)
                {
                    let start = w * arrival.get(&t).copied().unwrap_or(0.0) / self.opts.delay_norm;
                    self.cost[t as usize] = start;
                    self.prev[t as usize] = (NONE, 0, 0);
                    self.touched.push(t);
                    heap.push(Item {
                        f: start + astar * self.to_aims(t, &aims) + eb(t),
                        node: t,
                    });
                }
                let mut fs: Vec<f32> = Vec::new();
                let mut dump: Option<std::io::BufWriter<std::fs::File>> = self
                    .dump_net
                    .contains(&self.cur_net)
                    .then_some(self.cur_net)
                    .and_then(|d| {
                        std::fs::File::create(format!("/tmp/oxibit_pops_{d}_{}.csv", sd.dist))
                            .ok()
                            .map(std::io::BufWriter::new)
                    });
                while let Some(Item { f, node }) = heap.pop() {
                    *pops += 1;
                    sd.pops += 1;
                    let dd = self.to_aims(node, &aims) as i32;
                    sd.far = sd.far.max(dd);
                    sd.by_dist[match dd {
                        0..=2 => 0,
                        3..=5 => 1,
                        6..=10 => 2,
                        11..=20 => 3,
                        21..=40 => 4,
                        _ => 5,
                    }] += 1;
                    fs.push(f);
                    if let Some(w) = dump.as_mut() {
                        use std::io::Write;
                        let (x, y) = self.g.node_xy(node);
                        let _ = writeln!(
                            w,
                            "{x},{y},{},{f},{},{}",
                            self.cost[node as usize],
                            self.occ[node as usize],
                            self.g.node_name(node)
                        );
                    }
                    if self.occ[node as usize] > 0 && self.tag[node as usize] != tag {
                        sd.congested_pops += 1;
                    }
                    if set.contains(&node) {
                        reached = Some(node);
                        break;
                    }
                    let base = self.cost[node as usize];
                    for &(d, tile, pi) in self.g.out_edges(node) {
                        match self.g.edge_kind(tile, pi) {
                            EdgeKind::Feature | EdgeKind::Fixed => {}
                            EdgeKind::Relay if self.g.edge_site_through(tile, pi) => {}
                            EdgeKind::Relay
                                if net.class == NetClass::GlobalClock
                                    && self.g.edge_pip(tile, pi).tile.starts_with("CLK_") => {}
                            // A constant net's VCC tie reads into an interconnect
                            // input through its default (no line).
                            EdgeKind::Default
                                if net.class == NetClass::Constant && net.driver.contains(node) => {
                            }
                            EdgeKind::Relay | EdgeKind::Default => continue,
                        }
                        if self.reserved[d as usize] && !set.contains(&d) {
                            continue;
                        }
                        if dedicated && self.g.node_fabric(d) == Fabric::Interconnect {
                            continue;
                        }
                        if matches!(self.claim[d as usize], Claim::Net(owner) if owner != id) {
                            continue;
                        }
                        if !set.contains(&d) && !inside(d) {
                            continue;
                        }
                        let step = if self.tag[d as usize] == tag {
                            0.0
                        } else if w > 0.0 {
                            (1.0 - w) * self.node_cost(d, pres)
                                + w * self.g.edge_timing(tile, pi).delay as f32
                                    / self.opts.delay_norm
                        } else {
                            self.node_cost(d, pres)
                        };
                        let c = base + step;
                        if c < self.cost[d as usize] {
                            if self.cost[d as usize].is_infinite() {
                                self.touched.push(d);
                            }
                            self.cost[d as usize] = c;
                            self.prev[d as usize] = (node, tile, pi);
                            heap.push(Item {
                                f: c + astar * self.to_aims(d, &aims) + eb(d),
                                node: d,
                            });
                        }
                    }
                }
                if let Some(t) = reached {
                    let c = self.cost[t as usize].max(1e-3);
                    for f in fs {
                        let r = f / c;
                        sd.by_f[if r < 0.5 {
                            0
                        } else if r < 1.0 {
                            1
                        } else if r < 2.0 {
                            2
                        } else {
                            3
                        }] += 1;
                    }
                }
                if reached.is_some() || boxed.is_none() {
                    break;
                }
            }
            if let Some(t) = reached {
                sd.cost = self.cost[t as usize];
            }
            match reached {
                Some(target) => {
                    let mut n = target;
                    let first = hops.len();
                    while self.prev[n as usize].0 != NONE && self.tag[n as usize] != tag {
                        let (from, tile, pi) = self.prev[n as usize];
                        hops.push(Hop {
                            src: from,
                            dst: n,
                            tile,
                            pi,
                        });
                        self.tag[n as usize] = tag;
                        tree.push(n);
                        n = from;
                    }
                    self.tag[target as usize] = tag;
                    sd.hops = hops.len() - first;
                    // The new branch's arrivals, from where it left the tree.
                    for h in hops[first..].iter().rev() {
                        let a = arrival.get(&h.src).copied().unwrap_or(0.0)
                            + self.g.edge_timing(h.tile, h.pi).delay as f32;
                        arrival.insert(h.dst, a);
                    }
                }
                None => failed += 1,
            }
            if sd.pops > self.diag.pops {
                self.diag = sd;
            }
            self.reset_scratch();
        }
        (hops, failed)
    }

    /// Nodes two or more nets occupy under `routes` (one hop list per net),
    /// with the nets on each.
    pub fn audit(&self, nets: &[Net], routes: &[Vec<Hop>]) -> Vec<(u32, Vec<usize>)> {
        let mut on: std::collections::HashMap<u32, Vec<usize>> = std::collections::HashMap::new();
        for (i, (net, hops)) in nets.iter().zip(routes).enumerate() {
            for n in self.footprint(net, hops) {
                on.entry(n).or_default().push(i);
            }
        }
        // Two or more electrical nets: constants of one value are one.
        let distinct = |v: &[usize]| {
            v.iter()
                .map(|&i| nets[i].identity(i))
                .collect::<HashSet<_>>()
                .len()
        };
        let mut out: Vec<(u32, Vec<usize>)> =
            on.into_iter().filter(|(_, v)| distinct(v) > 1).collect();
        out.sort_unstable();
        out
    }

    /// Route `nets`; returns each net's hops (empty for nets left as the
    /// design routed them) and the run's statistics.
    pub fn run(&mut self, nets: &[Net]) -> (Vec<Vec<Hop>>, Stats) {
        self.run_with(nets, &Silent)
    }

    /// `run`, reporting a `progress` event per iteration (phase `route`) and
    /// stopping between nets once `progress.cancelled()`; a cancelled run
    /// returns the routes it has.
    pub fn run_with(&mut self, nets: &[Net], progress: &dyn Progress) -> (Vec<Vec<Hop>>, Stats) {
        self.run_observed(nets, progress, &mut |_, _| {})
    }

    /// `run_with`, handing `observe` each iteration's number and the nets
    /// on every node once the iteration's nets are routed.
    #[oxibit_prof::hot(name = "Router::run")]
    pub fn run_observed(
        &mut self,
        nets: &[Net],
        progress: &dyn Progress,
        observe: &mut dyn FnMut(u32, &[u16]),
    ) -> (Vec<Vec<Hop>>, Stats) {
        let t0 = Instant::now();
        let mut stats = Stats::default();
        let routed: Vec<bool> = nets
            .iter()
            .map(|n| {
                matches!(
                    n.class,
                    NetClass::Signal | NetClass::Constant | NetClass::GlobalClock
                )
            })
            .collect();

        // Kept nets reserve their nodes; routed nets own their terminals.
        for (net, &r) in nets.iter().zip(&routed) {
            if !r {
                let hops: Vec<Hop> = net
                    .edges
                    .iter()
                    .map(|e| Hop {
                        src: e.src,
                        dst: e.dst,
                        tile: e.tile,
                        pi: e.pi,
                    })
                    .collect();
                for n in self.footprint(net, &hops) {
                    self.reserved[n as usize] = true;
                }
            }
        }
        // Claims: every node whose fixed fan-out reaches a routed net's
        // fixed sink (walking back over the edges `closure` follows).
        let mut claimed = 0usize;
        for (i, net) in nets.iter().enumerate() {
            if !routed[i] {
                continue;
            }
            let id = net.identity(i);
            for &s in &net.sinks {
                let mut stack = vec![s];
                let mut seen: HashSet<u32> = HashSet::new();
                while let Some(q) = stack.pop() {
                    if !seen.insert(q) {
                        continue;
                    }
                    if q != s {
                        let c = &mut self.claim[q as usize];
                        *c = match *c {
                            Claim::Free => {
                                claimed += 1;
                                Claim::Net(id)
                            }
                            Claim::Net(o) if o == id => Claim::Net(o),
                            _ => Claim::Contested,
                        };
                    }
                    // `closure` crosses a fixed edge only into an unmuxed node.
                    if self.muxed[q as usize] {
                        continue;
                    }
                    for (p, tile, pi) in self.g.in_edges(q) {
                        if self.g.edge_kind(tile, pi) == EdgeKind::Fixed {
                            stack.push(p);
                        }
                    }
                }
            }
        }
        stats.claimed_nodes = claimed;
        let mut routes: Vec<Vec<Hop>> = vec![Vec::new(); nets.len()];
        let mut prints: Vec<Vec<u32>> = vec![Vec::new(); nets.len()];
        for (i, net) in nets.iter().enumerate() {
            if routed[i] {
                prints[i] = self.footprint(net, &[]);
                for &n in &prints[i] {
                    self.occupy(net.identity(i), n);
                }
            }
        }

        let mut pres = self.opts.schedule.first();
        let mut margin: Vec<i32> = vec![self.opts.bbox_margin; nets.len()];
        let mut todo: Vec<usize> = (0..nets.len()).filter(|&i| routed[i]).collect();
        stats.routed_nets = todo.len();
        // Tiles near an overused node, once the end game is on.
        let mut hot: HashSet<(i16, i16)> = HashSet::new();
        // Sinks each net could not reach when last routed. A net keeps its
        // count until it is routed again: counting only the nets of the
        // last iteration once reported an MMCM-to-BUFG net that failed in
        // iteration 1, and was never retried, as routed.
        let mut unreached: Vec<usize> = vec![0; nets.len()];
        for iter in 1..=self.opts.max_iters {
            oxibit_prof::hot_scope!("route.iter", iter, nets = todo.len());
            let ti = Instant::now();
            let mut pops = 0u64;
            let mut failed = 0;
            let mut kept = 0usize;
            let mut outside = 0u64;
            // (pops, net) of the costliest nets this iteration.
            let mut costly: Vec<(u64, usize, SearchDiag)> = Vec::new();
            for &i in &todo {
                if progress.cancelled() {
                    stats.cancelled = true;
                    break;
                }
                // Keep the branches that share no node with another net.
                let bad: HashSet<u32> = prints[i]
                    .iter()
                    .copied()
                    .filter(|&n| {
                        self.occ[n as usize] > 1
                            || (!hot.is_empty() && hot.contains(&self.g.node_xy(n)))
                    })
                    .collect();
                let keep = if iter == 1 {
                    Vec::new()
                } else {
                    self.keep_hops(&nets[i], &routes[i], &bad)
                };
                kept += keep.len();
                for &n in &prints[i] {
                    self.vacate(nets[i].identity(i), n);
                }
                let before = pops;
                self.cur_net = if iter == 1 { i } else { usize::MAX };
                let (hops, f) = oxibit_prof::hot_block!(
                    "route.net",
                    [
                        net = i,
                        sinks = nets[i].sinks.len(),
                        pops = ::tracing::field::Empty
                    ],
                    {
                        let r = self.route_net(
                            &nets[i],
                            nets[i].identity(i),
                            pres,
                            &mut pops,
                            &mut outside,
                            &keep,
                            margin[i],
                        );
                        oxibit_prof::hot_record!(pops, pops - before);
                        r
                    }
                );
                costly.push((pops - before, i, self.diag));
                if margin[i] >= 0 {
                    margin[i] = margin[i].saturating_mul(2).max(1);
                }
                failed += f;
                unreached[i] = f;
                prints[i] = self.footprint(&nets[i], &hops);
                for &n in &prints[i] {
                    self.occupy(nets[i].identity(i), n);
                }
                routes[i] = hops;
            }
            stats.heap_pops += pops;
            stats.unroutable_sinks = unreached.iter().sum();
            let _ = failed;

            // Overuse and history.
            let over = oxibit_prof::hot_block!("route.history", {
                let mut over = 0usize;
                for (n, &o) in self.occ.iter().enumerate() {
                    if o > 1 {
                        over += 1;
                        self.hist[n] += self.opts.hist_fac * (o - 1) as f32;
                    }
                }
                over
            });
            stats.iterations = iter;
            stats.overused = over;
            observe(iter, &self.occ);
            costly.sort_unstable_by(|a, b| (b.0, b.1).cmp(&(a.0, a.1)));
            let costliest: Vec<_> = costly
                .iter()
                .take(5)
                .map(|&(p, i, d)| {
                    json!({"pops": p, "net": i, "sinks": nets[i].sinks.len(), "groups": nets[i].pin_groups.len(),
                           "source": self.g.node_name(nets[i].driver.anchor()), "worst_search": d.to_json()})
                })
                .collect();
            progress.emit(Event::progress(
                "route",
                json!({
                    "iter": iter,
                    "pres": pres,
                    "estimate": format!("{:?}", self.opts.estimate.bound()),
                    "routed": todo.len(),
                    "overused": over,
                    "unroutable": stats.unroutable_sinks,
                    "pops": pops,
                    "kept_hops": kept,
                    "unboxed_searches": outside,
                    "costliest": costliest,
                    "top100_pops": costly.iter().take(100).map(|c| c.0).sum::<u64>(),
                    // [pops, sinks, h0, cost, far, dist, congested_pops, hops] of the 40 costliest.
                    "worst40": costly.iter().take(40).map(|&(p, i, d)| json!([p, nets[i].sinks.len() + nets[i].pin_groups.len(), d.h0, if d.cost.is_finite() { d.cost } else { -1.0 }, d.far, d.dist, d.congested_pops, d.hops, d.by_dist, d.by_f])).collect::<Vec<_>>(),
                    "iter_ms": ti.elapsed().as_millis() as u64,
                }),
            ));
            if over == 0 || stats.cancelled {
                break;
            }
            hot.clear();
            if let Some(e) = self.opts.endgame
                && over <= e.overuse
            {
                for (n, &o) in self.occ.iter().enumerate() {
                    if o > 1 {
                        let (x, y) = self.g.node_xy(n as u32);
                        for dx in -e.radius..=e.radius {
                            for dy in -e.radius..=e.radius {
                                hot.insert((x + dx, y + dy));
                            }
                        }
                    }
                }
            }
            todo = (0..nets.len())
                .filter(|&i| {
                    routed[i]
                        && (unreached[i] > 0
                            || prints[i].iter().any(|&n| {
                                self.occ[n as usize] > 1
                                    || (nets[i].class == NetClass::Signal
                                        && !hot.is_empty()
                                        && hot.contains(&self.g.node_xy(n)))
                            }))
                })
                .collect();
            pres = self.opts.schedule.next(pres);
        }
        stats.seconds = t0.elapsed().as_secs_f64();
        if let Some(path) = &self.opts.overuse_dump {
            // Per overused node, the nets on it: class, source, and whether
            // the node is that net's source, a sink, or on its route.
            let mut on: HashMap<u32, Vec<String>> = HashMap::new();
            for (i, p) in prints.iter().enumerate() {
                for &n in p {
                    if self.occ[n as usize] > 1 {
                        let role = if nets[i].driver.contains(n) {
                            "src"
                        } else if nets[i].sinks.contains(&n) {
                            "sink"
                        } else if nets[i].pin_groups.iter().any(|g| g.contains(&n)) {
                            "group"
                        } else {
                            "hop"
                        };
                        on.entry(n).or_default().push(format!(
                            "{:?}:{}:{role}",
                            nets[i].class,
                            self.g.node_name(nets[i].driver.anchor())
                        ));
                    }
                }
            }
            let mut text = String::new();
            for (n, &o) in self.occ.iter().enumerate() {
                if o > 1 {
                    let who = on.get(&(n as u32)).map(|v| v.join(" ")).unwrap_or_default();
                    text.push_str(&format!(
                        "{}\t{o}\t{}\t{who}\n",
                        self.g.node_name(n as u32),
                        self.hist[n]
                    ));
                }
            }
            let _ = std::fs::write(path, text);
        }
        (routes, stats)
    }
}
