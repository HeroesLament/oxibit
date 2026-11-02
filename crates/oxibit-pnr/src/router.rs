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
use std::sync::atomic::Ordering::Relaxed;
use std::sync::atomic::{AtomicU16, AtomicUsize};
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

#[derive(Debug, Clone)]
pub struct Opts {
    pub max_iters: u32,
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
    /// Route the nets whose search box and current footprint lie in one
    /// cell of an `rx` x `ry` grid over the device in parallel, one worker
    /// per cell at a time, the others serially after them. Workers touch
    /// disjoint nodes, so the result does not depend on `threads`. Needs
    /// `bbox_margin` >= 0 (an unboxed search is never confined). None:
    /// every net serially, as before.
    pub regions: Option<(u16, u16)>,
    /// Region workers; 0 = available parallelism.
    pub threads: usize,
}

impl Default for Opts {
    fn default() -> Self {
        Opts {
            max_iters: 200,
            // Cap 64: on the PHY, cap 8 settles into 1-3 overused nodes
            // that trade places for good and cap 1000 oscillates; 64
            // converges (0 at iteration 151 unclaimed, 30 with claims).
            schedule: Schedule::Geometric { p0: 0.5, growth: 1.5, cap: 64.0 },
            hist_fac: 1.0,
            estimate: Estimate { per_tile: 0.6, entry: true },
            delay_weight: 0.0,
            delay_norm: 100.0,
            astar_delay: 0.3,
            bbox_margin: -1,
            entry_aim: false,
            overuse_dump: None,
            regions: None,
            threads: 0,
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
    net.pin_groups.iter().map(|g| g.iter().copied().find(|p| reached.contains(p))).collect()
}

pub struct Router<'g> {
    core: Core<'g>,
    /// Search scratch of the serial pass.
    buf: Buffers,
    /// Search scratch of the region workers, allocated on first use.
    workers: Vec<Buffers>,
}

/// What every search reads, and the occupancy they all write. `occ` and
/// `const_on` are atomics so region workers can share them: by
/// construction (`Grid`, `Router::partition`) no two workers touch the
/// same node, so `Relaxed` is enough and results do not depend on timing.
struct Core<'g> {
    g: &'g Graph,
    opts: Opts,
    /// Electrical nets using each node: once per signal net, once per
    /// constant value however many tie-driven nets of it share the node.
    occ: Vec<AtomicU16>,
    /// Tie-driven nets of each value (`Logic::index`) on each node.
    const_on: Vec<[AtomicU16; 2]>,
    hist: Vec<f32>,
    /// Nodes no routed net may enter (clock and default nets, kept as is).
    reserved: Vec<bool>,
    /// Nodes only one net may enter (see `Claim`).
    claim: Vec<Claim>,
    /// Nodes some configurable pip drives. A hard edge into such a node is
    /// a mux's default input (DSP_FAN -> DSP_1_D17 next to the GND and VCC
    /// tie pips), not a wire, so fixed fan-out stops there.
    muxed: Vec<bool>,
    /// OXIBIT_DUMP_NET: write every pop of this net's searches (debug).
    dump_net: Vec<usize>,
    /// `Opts::regions` over the device, if set.
    grid: Option<Grid>,
    /// `Opts::threads` resolved.
    threads: usize,
}

/// One search's scratch, sized to the graph.
struct Buffers {
    cost: Vec<f32>,
    prev: Vec<(u32, u32, u32)>, // (from node, tile, pi); from == u32::MAX at a start node
    touched: Vec<u32>,
    /// Net-local tree membership: tag[n] == current net tag.
    tag: Vec<u32>,
    cur_tag: u32,
    /// The value of the constant net being routed, if it is one.
    cur_const: Option<Logic>,
    /// The costliest single search of the last routed net.
    diag: SearchDiag,
    cur_net: usize,
}

impl Buffers {
    fn new(n: usize) -> Self {
        Buffers {
            cost: vec![f32::INFINITY; n],
            prev: vec![(NONE, 0, 0); n],
            touched: Vec::new(),
            tag: vec![0; n],
            cur_tag: 0,
            cur_const: None,
            diag: SearchDiag::default(),
            cur_net: usize::MAX,
        }
    }
}

/// The device cut into `rx` x `ry` cells of tiles. A node belongs to the
/// cell of its anchor tile (`Graph::node_xy`), so the cells partition the
/// nodes.
#[derive(Debug, Clone, Copy)]
struct Grid {
    rx: i32,
    ry: i32,
    /// Device extent in tiles (one past the largest coordinate).
    w: i32,
    h: i32,
}

impl Grid {
    fn count(&self) -> usize {
        (self.rx * self.ry) as usize
    }

    fn cell(&self, x: i32, y: i32) -> usize {
        let cx = (x.clamp(0, self.w - 1) * self.rx / self.w).min(self.rx - 1);
        let cy = (y.clamp(0, self.h - 1) * self.ry / self.h).min(self.ry - 1);
        (cy * self.rx + cx) as usize
    }

    fn of_node(&self, g: &Graph, n: u32) -> usize {
        let (x, y) = g.node_xy(n);
        self.cell(x as i32, y as i32)
    }

    /// The cell holding the whole box (clipped to the device), if one does.
    fn of_box(&self, (x0, y0, x1, y1): (i32, i32, i32, i32)) -> Option<usize> {
        let a = self.cell(x0, y0);
        (a == self.cell(x1, y1)).then_some(a)
    }
}

/// One search over the shared state with its own scratch; in a region
/// (`region` Some) it enters only that cell's nodes.
struct Search<'a, 'g> {
    core: &'a Core<'g>,
    b: &'a mut Buffers,
    region: Option<usize>,
}

/// What a region worker did with one net.
enum Outcome {
    Routed { hops: Vec<Hop>, print: Vec<u32> },
    /// A sink not found inside the box: vacated, nothing occupied; the
    /// serial pass routes it from `keep`.
    HandedBack { keep: Vec<Hop> },
}

struct NetRun {
    i: usize,
    pops: u64,
    diag: SearchDiag,
    outcome: Outcome,
}

/// One region's nets, in net order.
#[derive(Default)]
struct RegionRun {
    nets: Vec<NetRun>,
    /// Footprint nodes outside the cell (fixed fan-out across its edge),
    /// occupied after the workers join.
    deferred: Vec<(Identity, u32)>,
    pops: u64,
    outside: u64,
    kept: usize,
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

impl<'g> Core<'g> {
    fn new(g: &'g Graph, opts: Opts) -> Self {
        let n = g.nodes();
        let mut muxed = vec![false; n];
        let (mut w, mut h) = (1i32, 1i32);
        for m in 0..n as u32 {
            for &(d, tile, pi) in g.out_edges(m) {
                if g.edge_kind(tile, pi) == EdgeKind::Feature {
                    muxed[d as usize] = true;
                }
            }
            let (x, y) = g.node_xy(m);
            w = w.max(x as i32 + 1);
            h = h.max(y as i32 + 1);
        }
        let grid = opts.regions.map(|(rx, ry)| Grid { rx: (rx as i32).clamp(1, w), ry: (ry as i32).clamp(1, h), w, h });
        let threads = match opts.threads {
            0 => std::thread::available_parallelism().map_or(1, |t| t.get()),
            t => t,
        };
        Core {
            muxed,
            g,
            occ: (0..n).map(|_| AtomicU16::new(0)).collect(),
            const_on: (0..n).map(|_| [AtomicU16::new(0), AtomicU16::new(0)]).collect(),
            hist: vec![0.0; n],
            reserved: vec![false; n],
            claim: vec![Claim::Free; n],
            dump_net: std::env::var("OXIBIT_DUMP_NET").map(|v| v.split(',').filter_map(|x| x.parse().ok()).collect()).unwrap_or_default(),
            grid,
            threads,
            opts,
        }
    }

    /// Count node `n` as used by a net of identity `id`.
    fn occupy(&self, id: Identity, n: u32) {
        let n = n as usize;
        match id {
            Identity::Own(_) => {
                self.occ[n].fetch_add(1, Relaxed);
            }
            Identity::Const(v) => {
                if self.const_on[n][v.index()].fetch_add(1, Relaxed) == 0 {
                    self.occ[n].fetch_add(1, Relaxed);
                }
            }
        }
    }

    /// Undo `occupy`.
    fn vacate(&self, id: Identity, n: u32) {
        let n = n as usize;
        match id {
            Identity::Own(_) => {
                self.occ[n].fetch_sub(1, Relaxed);
            }
            Identity::Const(v) => {
                if self.const_on[n][v.index()].fetch_sub(1, Relaxed) == 1 {
                    self.occ[n].fetch_sub(1, Relaxed);
                }
            }
        }
    }

    /// Electrical nets on node `n`.
    fn occ_at(&self, n: u32) -> u16 {
        self.occ[n as usize].load(Relaxed)
    }

    fn occ_snapshot(&self) -> Vec<u16> {
        self.occ.iter().map(|o| o.load(Relaxed)).collect()
    }

    /// The bounding box of a net's terminals (source, sinks, pin group
    /// members) grown by `margin`; None for a negative margin (no box).
    fn net_box(&self, net: &Net, margin: i32) -> Option<(i32, i32, i32, i32)> {
        (margin >= 0).then(|| {
            let (sx, sy) = self.g.node_xy(net.driver.anchor());
            let (mut x0, mut y0, mut x1, mut y1) = (sx as i32, sy as i32, sx as i32, sy as i32);
            let groups = net.pin_groups.iter().filter(|g| !g.is_empty()).flatten();
            for &t in net.sinks.iter().chain(groups).chain(net.driver.nodes()) {
                let (x, y) = self.g.node_xy(t);
                x0 = x0.min(x as i32);
                y0 = y0.min(y as i32);
                x1 = x1.max(x as i32);
                y1 = y1.max(y as i32);
            }
            (x0 - margin, y0 - margin, x1 + margin, y1 + margin)
        })
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
                if self.g.edge_kind(tile, pi) == EdgeKind::Fixed && !self.muxed[d as usize] && !seen.contains(&d) {
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
            let terminal = net.sinks.binary_search(&h.dst).is_ok() || group_pins.binary_search(&h.dst).is_ok();
            if !terminal {
                self.closure(h.dst, &mut out, &mut seen);
            } else if seen.insert(h.dst) {
                out.push(h.dst);
            }
        }
        out
    }


    /// Tiles from `n` to the nearest of `aims`.
    fn to_aims(&self, n: u32, aims: &[(i16, i16)]) -> f32 {
        let (x, y) = self.g.node_xy(n);
        aims.iter().map(|&(ax, ay)| ((x - ax).abs() + (y - ay).abs()) as i32).min().unwrap_or(0) as f32
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
        let parent: HashMap<u32, usize> = hops.iter().enumerate().map(|(k, h)| (h.dst, k)).collect();
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
        hops.iter().zip(&keep).filter(|(_, k)| **k).map(|(h, _)| *h).collect()
    }


    /// Nodes two or more nets occupy under `routes` (one hop list per net),
    /// with the nets on each.
    fn audit(&self, nets: &[Net], routes: &[Vec<Hop>]) -> Vec<(u32, Vec<usize>)> {
        let mut on: std::collections::HashMap<u32, Vec<usize>> = std::collections::HashMap::new();
        for (i, (net, hops)) in nets.iter().zip(routes).enumerate() {
            for n in self.footprint(net, hops) {
                on.entry(n).or_default().push(i);
            }
        }
        // Two or more electrical nets: constants of one value are one.
        let distinct = |v: &[usize]| v.iter().map(|&i| nets[i].identity(i)).collect::<HashSet<_>>().len();
        let mut out: Vec<(u32, Vec<usize>)> = on.into_iter().filter(|(_, v)| distinct(v) > 1).collect();
        out.sort_unstable();
        out
    }
}

impl<'a, 'g> Search<'a, 'g> {

    /// Electrical nets on `n` other than the one being routed.
    fn others(&self, n: u32) -> u16 {
        let o = self.core.occ_at(n);
        match self.b.cur_const {
            Some(v) if self.core.const_on[n as usize][v.index()].load(Relaxed) > 0 => o - 1,
            _ => o,
        }
    }


    fn node_cost(&self, n: u32, pres: f32) -> f32 {
        let others = self.others(n) as f32;
        (1.0 + self.core.hist[n as usize]) * (1.0 + pres * others)
    }


    /// Congestion part of entering `n` during the current net's search: its
    /// cost as `route_net` charges it, less the uncongested cost of 1 that
    /// the distance term already stands for (tree nodes are free).
    fn enter_excess(&self, n: u32, pres: f32, tag: u32) -> f32 {
        if self.b.tag[n as usize] == tag || !self.owns(n) { 0.0 } else { self.node_cost(n, pres) - 1.0 }
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
            for (p1, _, _) in self.core.g.in_edges(t) {
                let c1 = self.enter_excess(p1, pres, tag);
                let e1 = near.entry(p1).or_insert(f32::INFINITY);
                *e1 = e1.min(ct);
                let mut any = false;
                for (p2, _, _) in self.core.g.in_edges(p1) {
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


    fn reset_scratch(&mut self) {
        for &n in &self.b.touched {
            self.b.cost[n as usize] = f32::INFINITY;
            self.b.prev[n as usize] = (NONE, 0, 0);
        }
        self.b.touched.clear();
    }

    /// Node `n` may be entered (always, outside a region).
    fn owns(&self, n: u32) -> bool {
        match (self.region, self.core.grid) {
            (Some(r), Some(grid)) => grid.of_node(self.core.g, n) == r,
            _ => true,
        }
    }

    /// Route one region's nets in order (`Router::partition` chose them:
    /// box and footprint inside the cell).
    #[allow(clippy::too_many_arguments)]
    fn route_region(&mut self, nets: &[Net], iter: u32, pres: f32, list: &[usize], prints: &[Vec<u32>], routes: &[Vec<Hop>], margin: &[i32]) -> RegionRun {
        let mut run = RegionRun::default();
        for &i in list {
            let net = &nets[i];
            let id = net.identity(i);
            let bad: HashSet<u32> = prints[i].iter().copied().filter(|&n| self.core.occ_at(n) > 1).collect();
            let keep = if iter == 1 { Vec::new() } else { self.core.keep_hops(net, &routes[i], &bad) };
            for &n in &prints[i] {
                self.core.vacate(id, n);
            }
            self.b.cur_net = if iter == 1 { i } else { usize::MAX };
            let (mut pops, mut outside) = (0u64, 0u64);
            let (hops, failed) = self.route_net(net, id, pres, &mut pops, &mut outside, &keep, margin[i]);
            run.pops += pops;
            run.outside += outside;
            if failed > 0 {
                run.nets.push(NetRun { i, pops, diag: self.b.diag, outcome: Outcome::HandedBack { keep } });
                continue;
            }
            run.kept += keep.len();
            let print = self.core.footprint(net, &hops);
            for &n in &print {
                if self.owns(n) {
                    self.core.occupy(id, n);
                } else {
                    run.deferred.push((id, n));
                }
            }
            run.nets.push(NetRun { i, pops, diag: self.b.diag, outcome: Outcome::Routed { hops, print } });
        }
        run
    }



    /// Route one net, starting from the hops in `keep` (a subtree from the
    /// source; empty to route from scratch). Its own previous occupancy
    /// must already be removed.
    #[allow(clippy::too_many_arguments)]
    fn route_net(&mut self, net: &Net, id: Identity, pres: f32, pops: &mut u64, outside: &mut u64, keep: &[Hop], margin: i32) -> (Vec<Hop>, usize) {
        self.b.cur_tag += 1;
        self.b.diag = SearchDiag::default();
        let tag = self.b.cur_tag;
        self.b.cur_const = match &net.driver {
            Driver::Ties(t) => Some(t.value()),
            Driver::Node(_) => None,
        };
        let roots = net.driver.nodes();
        let mut tree: Vec<u32> = roots.to_vec();
        for &r in roots {
            self.b.tag[r as usize] = tag;
        }
        // Delay (ps) from the driver to each tree node.
        let mut arrival: std::collections::HashMap<u32, f32> = roots.iter().map(|&r| (r, 0.0)).collect();
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
                    if self.b.tag[h.dst as usize] == tag {
                        continue;
                    }
                    self.b.tag[h.dst as usize] = tag;
                    tree.push(h.dst);
                    arrival.insert(h.dst, a + self.core.g.edge_timing(h.tile, h.pi).delay as f32);
                    stack.push(h.dst);
                }
            }
        }

        // Every sink is a set of nodes any one of which completes it: a
        // fixed sink is a set of one. Nearest first.
        let mut targets: Vec<Vec<u32>> = net.sinks.iter().map(|&s| vec![s]).collect();
        targets.extend(net.pin_groups.iter().filter(|g| !g.is_empty()).cloned());
        let anchor = net.driver.anchor();
        targets.sort_by(|a, b| self.core.manhattan(anchor, a[0]).total_cmp(&self.core.manhattan(anchor, b[0])));

        // The terminals' bounding box, grown by the margin.
        let bbox = self.core.net_box(net, margin);

        for set in &targets {
            if set.iter().any(|&t| self.b.tag[t as usize] == tag) {
                continue; // already on the tree
            }
            let aims = self.core.aims(set);
            // A global clock keeps to the clock network all the way to a
            // clock pin. It crosses general interconnect only to reach a
            // sink that is interconnect itself (a clock read as data).
            let dedicated = net.class == NetClass::GlobalClock && set.iter().all(|&t| self.core.g.node_fabric(t) == Fabric::Dedicated);
            let (near, far) = if self.core.opts.estimate.entry { self.entry_bound(set, pres, tag) } else { (HashMap::new(), 0.0) };
            let eb = |n: u32| near.get(&n).copied().unwrap_or(far);
            let w = self.core.opts.delay_weight;
            let astar = (1.0 - w) * self.core.opts.estimate.per_tile + w * self.core.opts.astar_delay;
            let mut reached = None;
            let mut sd = SearchDiag { cost: f32::INFINITY, dist: i32::MAX, ..Default::default() };
            for &t in &tree {
                sd.dist = sd.dist.min(self.core.to_aims(t, &aims) as i32);
            }
            sd.h0 = astar * sd.dist as f32 + tree.iter().map(|&t| eb(t)).fold(f32::INFINITY, f32::min).min(1e30);
            // In a region the boxed search is the only one: a sink it
            // cannot reach hands the net back to the serial pass.
            let passes: &[Option<(i32, i32, i32, i32)>] = if self.region.is_some() { &[bbox] } else { &[bbox, None] };
            for &boxed in passes {
                if boxed.is_none() && bbox.is_some() {
                    // The boxed search found nothing: search again unbounded.
                    self.reset_scratch();
                    *outside += 1;
                }
                let inside = |n: u32| {
                    boxed.is_none_or(|(x0, y0, x1, y1)| {
                        let (x, y) = self.core.g.node_xy(n);
                        let (x, y) = (x as i32, y as i32);
                        x >= x0 && x <= x1 && y >= y0 && y <= y1
                    })
                };
                let mut heap = BinaryHeap::new();
                for &t in tree.iter().filter(|&&t| !dedicated || self.core.g.node_fabric(t) == Fabric::Dedicated) {
                    let start = w * arrival.get(&t).copied().unwrap_or(0.0) / self.core.opts.delay_norm;
                    self.b.cost[t as usize] = start;
                    self.b.prev[t as usize] = (NONE, 0, 0);
                    self.b.touched.push(t);
                    heap.push(Item { f: start + astar * self.core.to_aims(t, &aims) + eb(t), node: t });
                }
                let mut fs: Vec<f32> = Vec::new();
                let mut dump: Option<std::io::BufWriter<std::fs::File>> = self.core.dump_net.contains(&self.b.cur_net).then_some(self.b.cur_net).and_then(|d| {
                    std::fs::File::create(format!("/tmp/oxibit_pops_{d}_{}.csv", sd.dist)).ok().map(std::io::BufWriter::new)
                });
                while let Some(Item { f, node }) = heap.pop() {
                    *pops += 1;
                    sd.pops += 1;
                    let dd = self.core.to_aims(node, &aims) as i32;
                    sd.far = sd.far.max(dd);
                    sd.by_dist[match dd { 0..=2 => 0, 3..=5 => 1, 6..=10 => 2, 11..=20 => 3, 21..=40 => 4, _ => 5 }] += 1;
                    fs.push(f);
                    if let Some(w) = dump.as_mut() {
                        use std::io::Write;
                        let (x, y) = self.core.g.node_xy(node);
                        let _ = writeln!(w, "{x},{y},{},{f},{},{}", self.b.cost[node as usize], self.core.occ_at(node), self.core.g.node_name(node));
                    }
                    if self.core.occ_at(node) > 0 && self.b.tag[node as usize] != tag {
                        sd.congested_pops += 1;
                    }
                    if set.contains(&node) {
                        reached = Some(node);
                        break;
                    }
                    let base = self.b.cost[node as usize];
                    for &(d, tile, pi) in self.core.g.out_edges(node) {
                        match self.core.g.edge_kind(tile, pi) {
                            EdgeKind::Feature | EdgeKind::Fixed => {}
                            EdgeKind::Relay if self.core.g.edge_site_through(tile, pi) => {}
                            EdgeKind::Relay if net.class == NetClass::GlobalClock && self.core.g.edge_pip(tile, pi).tile.starts_with("CLK_") => {}
                            // A constant net's VCC tie reads into an interconnect
                            // input through its default (no line).
                            EdgeKind::Default if net.class == NetClass::Constant && net.driver.contains(node) => {}
                            EdgeKind::Relay | EdgeKind::Default => continue,
                        }
                        if self.core.reserved[d as usize] && !set.contains(&d) {
                            continue;
                        }
                        if dedicated && self.core.g.node_fabric(d) == Fabric::Interconnect {
                            continue;
                        }
                        if matches!(self.core.claim[d as usize], Claim::Net(owner) if owner != id) {
                            continue;
                        }
                        if !set.contains(&d) && !inside(d) {
                            continue;
                        }
                        let step = if self.b.tag[d as usize] == tag {
                            0.0
                        } else if w > 0.0 {
                            (1.0 - w) * self.node_cost(d, pres) + w * self.core.g.edge_timing(tile, pi).delay as f32 / self.core.opts.delay_norm
                        } else {
                            self.node_cost(d, pres)
                        };
                        let c = base + step;
                        if c < self.b.cost[d as usize] {
                            if self.b.cost[d as usize].is_infinite() {
                                self.b.touched.push(d);
                            }
                            self.b.cost[d as usize] = c;
                            self.b.prev[d as usize] = (node, tile, pi);
                            heap.push(Item { f: c + astar * self.core.to_aims(d, &aims) + eb(d), node: d });
                        }
                    }
                }
                if let Some(t) = reached {
                    let c = self.b.cost[t as usize].max(1e-3);
                    for f in fs {
                        let r = f / c;
                        sd.by_f[if r < 0.5 { 0 } else if r < 1.0 { 1 } else if r < 2.0 { 2 } else { 3 }] += 1;
                    }
                }
                if reached.is_some() || boxed.is_none() {
                    break;
                }
            }
            if let Some(t) = reached {
                sd.cost = self.b.cost[t as usize];
            }
            match reached {
                Some(target) => {
                    let mut n = target;
                    let first = hops.len();
                    while self.b.prev[n as usize].0 != NONE && self.b.tag[n as usize] != tag {
                        let (from, tile, pi) = self.b.prev[n as usize];
                        hops.push(Hop { src: from, dst: n, tile, pi });
                        self.b.tag[n as usize] = tag;
                        tree.push(n);
                        n = from;
                    }
                    self.b.tag[target as usize] = tag;
                    sd.hops = hops.len() - first;
                    // The new branch's arrivals, from where it left the tree.
                    for h in hops[first..].iter().rev() {
                        let a = arrival.get(&h.src).copied().unwrap_or(0.0) + self.core.g.edge_timing(h.tile, h.pi).delay as f32;
                        arrival.insert(h.dst, a);
                    }
                }
                None => failed += 1,
            }
            if sd.pops > self.b.diag.pops {
                self.b.diag = sd;
            }
            self.reset_scratch();
        }
        (hops, failed)
    }
}

impl<'g> Router<'g> {
    pub fn new(g: &'g Graph, opts: Opts) -> Self {
        let buf = Buffers::new(g.nodes());
        Router { core: Core::new(g, opts), buf, workers: Vec::new() }
    }

    /// Keep every net off `nodes` (as off a kept net's).
    pub fn forbid(&mut self, nodes: impl IntoIterator<Item = u32>) {
        for n in nodes {
            self.core.reserved[n as usize] = true;
        }
    }

    /// Nodes two or more nets occupy under `routes` (one hop list per net),
    /// with the nets on each.
    pub fn audit(&self, nets: &[Net], routes: &[Vec<Hop>]) -> Vec<(u32, Vec<usize>)> {
        self.core.audit(nets, routes)
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

    /// Split `todo` into per-cell lists (box and footprint inside one cell)
    /// and the rest, each in net order.
    fn partition(&self, nets: &[Net], todo: &[usize], prints: &[Vec<u32>], margin: &[i32]) -> (Vec<Vec<usize>>, Vec<usize>) {
        let Some(grid) = self.core.grid else {
            return (Vec::new(), todo.to_vec());
        };
        let mut by_region: Vec<Vec<usize>> = vec![Vec::new(); grid.count()];
        let mut serial = Vec::new();
        for &i in todo {
            let cell = self
                .core
                .net_box(&nets[i], margin[i])
                .and_then(|b| grid.of_box(b))
                .filter(|&r| prints[i].iter().all(|&n| grid.of_node(self.core.g, n) == r));
            match cell {
                Some(r) => by_region[r].push(i),
                None => serial.push(i),
            }
        }
        (by_region, serial)
    }

    /// Route every non-empty cell's list on the workers; results in cell order.
    #[allow(clippy::too_many_arguments)]
    fn route_regions(&mut self, nets: &[Net], iter: u32, pres: f32, by_region: &[Vec<usize>], prints: &[Vec<u32>], routes: &[Vec<Hop>], margin: &[i32]) -> Vec<RegionRun> {
        let jobs: Vec<usize> = (0..by_region.len()).filter(|&r| !by_region[r].is_empty()).collect();
        let threads = self.core.threads.clamp(1, jobs.len().max(1));
        while self.workers.len() < threads {
            self.workers.push(Buffers::new(self.core.g.nodes()));
        }
        let Router { core, workers, .. } = self;
        let core: &Core = core;
        let next = AtomicUsize::new(0);
        let mut done: Vec<(usize, RegionRun)> = std::thread::scope(|sc| {
            let handles: Vec<_> = workers[..threads]
                .iter_mut()
                .map(|b| {
                    let (next, jobs) = (&next, &jobs);
                    sc.spawn(move || {
                        let mut mine = Vec::new();
                        while let Some(&r) = jobs.get(next.fetch_add(1, Relaxed)) {
                            let mut s = Search { core, b: &mut *b, region: Some(r) };
                            mine.push((r, s.route_region(nets, iter, pres, &by_region[r], prints, routes, margin)));
                        }
                        mine
                    })
                })
                .collect();
            handles.into_iter().flat_map(|h| h.join().unwrap_or_else(|e| std::panic::resume_unwind(e))).collect()
        });
        done.sort_unstable_by_key(|d| d.0);
        done.into_iter().map(|d| d.1).collect()
    }



    /// `run_with`, handing `observe` each iteration's number and the nets
    /// on every node once the iteration's nets are routed.
    #[oxibit_prof::hot(name = "Router::run")]
    pub fn run_observed(&mut self, nets: &[Net], progress: &dyn Progress, observe: &mut dyn FnMut(u32, &[u16])) -> (Vec<Vec<Hop>>, Stats) {
        let t0 = Instant::now();
        let mut stats = Stats::default();
        let routed: Vec<bool> = nets.iter().map(|n| matches!(n.class, NetClass::Signal | NetClass::Constant | NetClass::GlobalClock)).collect();

        // Kept nets reserve their nodes; routed nets own their terminals.
        for (net, &r) in nets.iter().zip(&routed) {
            if !r {
                let hops: Vec<Hop> = net.edges.iter().map(|e| Hop { src: e.src, dst: e.dst, tile: e.tile, pi: e.pi }).collect();
                for n in self.core.footprint(net, &hops) {
                    self.core.reserved[n as usize] = true;
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
                        let c = &mut self.core.claim[q as usize];
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
                    if self.core.muxed[q as usize] {
                        continue;
                    }
                    for (p, tile, pi) in self.core.g.in_edges(q) {
                        if self.core.g.edge_kind(tile, pi) == EdgeKind::Fixed {
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
                prints[i] = self.core.footprint(net, &[]);
                for &n in &prints[i] {
                    self.core.occupy(net.identity(i), n);
                }
            }
        }

        let mut pres = self.core.opts.schedule.first();
        let mut margin: Vec<i32> = vec![self.core.opts.bbox_margin; nets.len()];
        let mut todo: Vec<usize> = (0..nets.len()).filter(|&i| routed[i]).collect();
        stats.routed_nets = todo.len();
        for iter in 1..=self.core.opts.max_iters {
            oxibit_prof::hot_scope!("route.iter", iter, nets = todo.len());
            let ti = Instant::now();
            let mut pops = 0u64;
            let mut failed = 0;
            let mut kept = 0usize;
            let mut outside = 0u64;
            // (pops, net) of the costliest nets this iteration.
            let mut costly: Vec<(u64, usize, SearchDiag)> = Vec::new();
            // Nets inside one region, in parallel; then, in net order, the
            // rest and any a region handed back.
            let (by_region, serial) = self.partition(nets, &todo, &prints, &margin);
            let mut handed: Vec<(usize, Vec<Hop>)> = Vec::new();
            if by_region.iter().any(|r| !r.is_empty()) {
                for run in self.route_regions(nets, iter, pres, &by_region, &prints, &routes, &margin) {
                    pops += run.pops;
                    outside += run.outside;
                    kept += run.kept;
                    for (id, n) in run.deferred {
                        self.core.occupy(id, n);
                    }
                    for nr in run.nets {
                        match nr.outcome {
                            Outcome::Routed { hops, print } => {
                                costly.push((nr.pops, nr.i, nr.diag));
                                if margin[nr.i] >= 0 {
                                    margin[nr.i] = margin[nr.i].saturating_mul(2).max(1);
                                }
                                prints[nr.i] = print;
                                routes[nr.i] = hops;
                            }
                            Outcome::HandedBack { keep } => {
                                // Vacated in the region; nothing of it occupied.
                                prints[nr.i].clear();
                                handed.push((nr.i, keep));
                            }
                        }
                    }
                }
            }
            let mut seq: Vec<(usize, Option<Vec<Hop>>)> = serial.into_iter().map(|i| (i, None)).chain(handed.into_iter().map(|(i, k)| (i, Some(k)))).collect();
            seq.sort_unstable_by_key(|e| e.0);
            for (i, given) in seq {
                if progress.cancelled() {
                    stats.cancelled = true;
                    break;
                }
                let keep = match given {
                    Some(keep) => keep,
                    None => {
                        // Keep the branches that share no node with another net.
                        let bad: HashSet<u32> = prints[i].iter().copied().filter(|&n| self.core.occ_at(n) > 1).collect();
                        let keep = if iter == 1 { Vec::new() } else { self.core.keep_hops(&nets[i], &routes[i], &bad) };
                        for &n in &prints[i] {
                            self.core.vacate(nets[i].identity(i), n);
                        }
                        keep
                    }
                };
                kept += keep.len();
                let before = pops;
                self.buf.cur_net = if iter == 1 { i } else { usize::MAX };
                let (hops, f) = oxibit_prof::hot_block!("route.net", [net = i, sinks = nets[i].sinks.len(), pops = ::tracing::field::Empty], {
                    let mut s = Search { core: &self.core, b: &mut self.buf, region: None };
                    let r = s.route_net(&nets[i], nets[i].identity(i), pres, &mut pops, &mut outside, &keep, margin[i]);
                    oxibit_prof::hot_record!(pops, pops - before);
                    r
                });
                costly.push((pops - before, i, self.buf.diag));
                if margin[i] >= 0 {
                    margin[i] = margin[i].saturating_mul(2).max(1);
                }
                failed += f;
                prints[i] = self.core.footprint(&nets[i], &hops);
                for &n in &prints[i] {
                    self.core.occupy(nets[i].identity(i), n);
                }
                routes[i] = hops;
            }
            stats.heap_pops += pops;
            stats.unroutable_sinks = failed;

            // Overuse and history.
            let over = oxibit_prof::hot_block!("route.history", {
                let mut over = 0usize;
                for n in 0..self.core.occ.len() {
                    let o = self.core.occ[n].load(Relaxed);
                    if o > 1 {
                        over += 1;
                        self.core.hist[n] += self.core.opts.hist_fac * (o - 1) as f32;
                    }
                }
                over
            });
            stats.iterations = iter;
            stats.overused = over;
            observe(iter, &self.core.occ_snapshot());
            costly.sort_unstable_by(|a, b| (b.0, b.1).cmp(&(a.0, a.1)));
            let costliest: Vec<_> = costly
                .iter()
                .take(5)
                .map(|&(p, i, d)| {
                    json!({"pops": p, "net": i, "sinks": nets[i].sinks.len(), "groups": nets[i].pin_groups.len(),
                           "source": self.core.g.node_name(nets[i].driver.anchor()), "worst_search": d.to_json()})
                })
                .collect();
            progress.emit(Event::progress(
                "route",
                json!({
                    "iter": iter,
                    "pres": pres,
                    "estimate": format!("{:?}", self.core.opts.estimate.bound()),
                    "routed": todo.len(),
                    "overused": over,
                    "unroutable": failed,
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
            todo = (0..nets.len()).filter(|&i| routed[i] && prints[i].iter().any(|&n| self.core.occ_at(n) > 1)).collect();
            pres = self.core.opts.schedule.next(pres);
        }
        stats.seconds = t0.elapsed().as_secs_f64();
        if let Some(path) = &self.core.opts.overuse_dump {
            // Per overused node, the nets on it: class, source, and whether
            // the node is that net's source, a sink, or on its route.
            let mut on: HashMap<u32, Vec<String>> = HashMap::new();
            for (i, p) in prints.iter().enumerate() {
                for &n in p {
                    if self.core.occ_at(n) > 1 {
                        let role = if nets[i].driver.contains(n) {
                            "src"
                        } else if nets[i].sinks.contains(&n) {
                            "sink"
                        } else if nets[i].pin_groups.iter().any(|g| g.contains(&n)) {
                            "group"
                        } else {
                            "hop"
                        };
                        on.entry(n).or_default().push(format!("{:?}:{}:{role}", nets[i].class, self.core.g.node_name(nets[i].driver.anchor())));
                    }
                }
            }
            let mut text = String::new();
            for (n, o) in self.core.occ_snapshot().into_iter().enumerate() {
                if o > 1 {
                    let who = on.get(&(n as u32)).map(|v| v.join(" ")).unwrap_or_default();
                    text.push_str(&format!("{}\t{o}\t{}\t{who}\n", self.core.g.node_name(n as u32), self.core.hist[n]));
                }
            }
            let _ = std::fs::write(path, text);
        }
        (routes, stats)
    }
}

#[cfg(test)]
mod tests {
    use super::Grid;

    #[test]
    fn grid_cells_partition_the_device() {
        let g = Grid { rx: 4, ry: 3, w: 100, h: 151 };
        let mut seen = vec![0usize; g.count()];
        for y in 0..g.h {
            for x in 0..g.w {
                seen[g.cell(x, y)] += 1;
            }
        }
        assert!(seen.iter().all(|&c| c > 0), "{seen:?}");
        assert_eq!(seen.iter().sum::<usize>(), (g.w * g.h) as usize);
        // Off-device coordinates clip to the edge cells.
        assert_eq!(g.cell(-5, -5), g.cell(0, 0));
        assert_eq!(g.cell(1000, 1000), g.cell(g.w - 1, g.h - 1));
    }

    #[test]
    fn a_box_belongs_to_a_cell_only_when_inside_it() {
        let g = Grid { rx: 2, ry: 2, w: 100, h: 100 };
        assert_eq!(g.of_box((10, 10, 40, 40)), Some(g.cell(10, 10)));
        assert_eq!(g.of_box((40, 10, 60, 40)), None);
        assert_eq!(g.of_box((-3, -3, 20, 20)), Some(g.cell(0, 0)));
    }
}
