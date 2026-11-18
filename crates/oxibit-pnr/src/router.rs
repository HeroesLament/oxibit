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
//!
//! With `Opts::timing` (a `sta::TimingView`) the delay term is per sink
//! instead: an edge costs `tau * criticality * delay / delay_norm` on top
//! of the above, criticality that of the sink being searched for, read
//! from the view's table after every iteration (only the nets just routed
//! are re-timed). A sink that meets its clock has criticality 0 and routes
//! on congestion alone. Besides the nets on overused nodes, an iteration
//! then also rips up the failing nets whose criticality has risen since
//! they were last routed (`TimingView::hot`): only their critical sinks'
//! branches, searched for first. After `TimingView::passes` such
//! iterations only congestion rips nets up, so the run ends.
//!
//! A global clock net stays on the clock network: it may not enter a
//! node of general routing (`Router::fabric`: the singles to long lines,
//! bounce and alternate wires, bypass and logic-output wires). Left free
//! to, the router took picoBEAM's pb_clk off a GCLK line through GFAN,
//! BYP_BOUNCE and FAN_BOUNCE onto slice CLK pins: the same source, so
//! every static check passed, and on silicon the skew broke hold at any
//! clock rate. A CLK pin is then reached only from its GCLK line; GFAN and
//! IMUX stay open for clock inputs no GCLK line reaches.
//!
//! With `Opts::cleanup_rounds` > 0, a legal timed route then gets a
//! cleanup pass (`Router::cleanup_pass`): fresh slack, and every sink that
//! fails it searched again alone, for delay only, through nodes no other
//! net uses, its new branch kept only if it arrives earlier. No other
//! connection changes, so nothing gets slower and nothing overlaps.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::fx::{HashMap, HashSet};
use std::time::Instant;

use oxibit_fasm::route::{EdgeKind, Graph};
use oxibit_report::{Event, Progress, Silent};
use serde_json::json;

use crate::nets::{Net, NetClass};
use crate::sta::{CriticalityMap, TimingView};

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
    /// Route for timing: where criticalities come from and how they weigh.
    pub timing: Option<TimingView>,
    /// Measure the A* distance from the nearest tile of a node, not from
    /// one arbitrary tile of it (`Router::to_aims`). On by default: on
    /// picoBEAM it made long lines usable (2060 to 3296 long-line
    /// entries), took the untimed worst path from 74.9 to 70.4 ns and
    /// convergence on a foreign placement from 143 iterations to 12, for
    /// about 60% more route time (docs/clock-improvement.md).
    pub span_aware: bool,
    /// Drop a popped heap entry whose node has been reached more cheaply
    /// since it was pushed. Changes no route: its expansion would only
    /// repeat the cheaper entry's.
    pub skip_stale: bool,
    /// Detour bound `(factor, margin)`: a search pushes a node only if its
    /// distance to the target (`Router::to_aims`) is at most `factor` x
    /// the least distance of any node popped so far, plus `margin` tiles.
    /// A search that finds nothing so is run again without it. Only a
    /// net's first route is bounded: a bounded search still finds a path
    /// through a hot spot, just a congested one, so bounding reroutes
    /// keeps negotiation from moving nets around it (picoBEAM on
    /// nextpnr's placement, (1.5, 6) on every route: 200 iterations, still
    /// overused). `None`: no bound.
    pub detour: Option<(f32, f32)>,
    /// Search each connection from both ends at once (`Router::search_bidir`):
    /// a forward front from the tree as today, a backward front from the
    /// sink over reversed edges, alternating one pop each and stopping at
    /// the first node both have reached. Not exact, like the forward
    /// search. `detour` and the bounding box do not apply to it; a
    /// connection whose fronts never meet is searched again forward.
    pub bidir: bool,
    /// Rounds of the post-route cleanup pass (`Router::cleanup_pass`), run
    /// after a legal route with `timing` set. 0: none.
    pub cleanup_rounds: u32,
    /// Hold slack (ps, fast corner, no skew) every block RAM capture is
    /// padded up to after a legal route with `timing` set
    /// (`Router::hold_pass`). None: no padding.
    pub hold_fix_ps: Option<i64>,
    /// Seconds the hold pass may take; when spent it stops between
    /// targets and says so (`budget_hit`).
    pub hold_budget_s: f64,
}

/// The end of a negotiation, when few nodes are left overused: the nets
/// on them keep trading the same wires because the nets that block their
/// alternatives are never ripped up. Once at most `overuse` nodes are
/// overused, every signal net with a node within `radius` tiles of one is
/// rerouted too, its branches through that area dropped, so the
/// neighbourhood can rearrange. (From main, f3090db.)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Endgame {
    pub overuse: usize,
    pub radius: i16,
}

impl Default for Opts {
    fn default() -> Self {
        Opts {
            max_iters: 200,
            // picoBEAM, 32k LUTs, Nesterov placement: without it 43-49
            // nodes traded places from pass 30 and 2 were left at 200.
            endgame: Some(Endgame { overuse: 64, radius: 3 }),
            schedule: Schedule::Geometric { p0: 0.5, growth: 1.5, cap: 8.0 },
            hist_fac: 1.0,
            estimate: Estimate { per_tile: 0.6, entry: true },
            delay_weight: 0.0,
            delay_norm: 100.0,
            astar_delay: 0.3,
            bbox_margin: -1,
            entry_aim: false,
            overuse_dump: None,
            timing: None,
            span_aware: true,
            skip_stale: false,
            detour: None,
            bidir: false,
            cleanup_rounds: 0,
            hold_fix_ps: None,
            hold_budget_s: 300.0,
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
    /// Heap pushes: a node is pushed again each time its cost improves,
    /// so pushes over pops is how much of the heap is stale entries.
    pub heap_pushes: u64,
    pub seconds: f64,
    /// Stopped early through `Progress::cancelled`.
    pub cancelled: bool,
    /// Iterations that ripped nets up for timing.
    pub timing_passes: u32,
    /// Nets ripped up for timing, over all iterations.
    pub timing_ripups: usize,
    /// What the cleanup pass did (`Cleanup::to_json`); null without one.
    pub cleanup: serde_json::Value,
}

impl Stats {
    pub fn to_json(&self) -> serde_json::Value {
        json!({
            "iterations": self.iterations,
            "overused": self.overused,
            "routed_nets": self.routed_nets,
            "unroutable_sinks": self.unroutable_sinks,
            "heap_pops": self.heap_pops,
            "heap_pushes": self.heap_pushes,
            "seconds": self.seconds,
            "cancelled": self.cancelled,
            "timing_passes": self.timing_passes,
            "timing_ripups": self.timing_ripups,
            "cleanup": self.cleanup,
        })
    }
}

#[derive(Clone, Copy)]
struct Item {
    f: f32,
    /// The node's path cost when pushed: above its cost now, the entry is
    /// stale (a cheaper path was pushed since) and its expansion would
    /// repeat that of the cheaper one.
    g: f32,
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
    g: &'g Graph,
    opts: Opts,
    /// Nets using each node (counted once per net).
    occ: Vec<u16>,
    hist: Vec<f32>,
    /// Nodes no routed net may enter (clock and default nets, kept as is).
    reserved: Vec<bool>,
    /// Nodes of the kept default nets: interconnect inputs reading VCC
    /// because nothing drives them, and the hard wires behind them. Off
    /// limits to every net but a VCC constant net, which puts the same
    /// value there (a VCC tie bounced through FAN_ALT to a control input
    /// also feeds the pin behind FAN, which a default net holds at VCC).
    vcc_kept: Vec<bool>,
    /// Nodes some configurable pip drives. A hard edge into such a node is
    /// a mux's default input (DSP_FAN -> DSP_1_D17 next to the GND and VCC
    /// tie pips), not a wire, so fixed fan-out stops there.
    muxed: Vec<bool>,
    /// Nodes of general routing, off limits to a global clock net
    /// (`is_fabric_wire`).
    fabric: Vec<bool>,
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
    /// Heap pushes so far (`Stats::heap_pushes`).
    pushes: u64,
    cur_net: usize,
    /// Backward-front scratch (`Opts::bidir`; empty otherwise): cost from
    /// a node to the sink, the hop out of it towards the sink, touched.
    cost_b: Vec<f32>,
    next_b: Vec<(u32, u32, u32)>,
    touched_b: Vec<u32>,
    /// The search (by `search_no`) that last popped each node, per front:
    /// a node a front has popped is settled for it.
    popped_f: Vec<u32>,
    popped_b: Vec<u32>,
    search_no: u32,
    /// `route_net` runs for the cleanup pass: cost is delay alone, and
    /// nodes other nets occupy are off limits.
    cleanup: bool,
    /// OXIBIT_BY_F=1: keep every popped f of a search for `SearchDiag::by_f`
    /// (one push per heap pop, on the hottest line; off by default).
    by_f: bool,
}

/// Delay cost units per tile of distance in a cleanup search: just under
/// a long line's (301 ps over 18 tiles, 0.167 per `delay_norm` of 100 ps),
/// so the estimate stays close to a lower bound and the search close to
/// the fastest path.
const CLEANUP_ASTAR: f32 = 0.15;

/// Added per node entered in a cleanup search, so of two equally fast
/// paths the shorter wins.
const CLEANUP_NODE: f32 = 0.01;

/// What `Router::cleanup_pass` did.
#[derive(Debug, Default, Clone)]
pub struct Cleanup {
    pub rounds: u32,
    /// Failing sinks searched again, over all rounds.
    pub tried: usize,
    /// Of those, the ones whose new branch arrives earlier, kept.
    pub improved: usize,
    /// Sum of the arrival gains kept, ps.
    pub ps_saved: f64,
    pub pops: u64,
    pub slack_before: Option<i64>,
    pub slack_after: Option<i64>,
    pub seconds: f64,
}

impl Cleanup {
    pub fn to_json(&self) -> serde_json::Value {
        json!({"rounds": self.rounds, "tried": self.tried, "improved": self.improved, "ps_saved": self.ps_saved.round(),
               "pops": self.pops, "worst_slack_ns_before": self.slack_before.map(|s| s as f64 / 1000.0),
               "worst_slack_ns_after": self.slack_after.map(|s| s as f64 / 1000.0), "seconds": self.seconds})
    }
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

/// Whether a node (`TILE/WIRE`, as `Graph::node_name` names it) is general
/// routing: an interconnect single to long line (`..BEG..`, `..END..`,
/// `LV..`, `LH..`), a bounce or alternate wire, a bypass input or a logic
/// output. Not the clock network (GCLK, CLK, HCLK), not GFAN, not IMUX.
pub fn is_fabric_wire(node: &str) -> bool {
    let wire = node.rsplit('/').next().unwrap_or(node);
    if wire.contains("CLK") {
        return false;
    }
    wire.contains("BEG")
        || wire.contains("END")
        || wire.starts_with("LV")
        || wire.starts_with("LH")
        || wire.contains("FAN_ALT")
        || wire.contains("FAN_BOUNCE")
        || wire.contains("BYP")
        || wire.contains("LOGIC_OUTS")
}

/// How one net is to be routed this time.
struct Reroute<'a> {
    /// The hops to start from: a subtree from the source; empty to route
    /// from scratch.
    keep: &'a [Hop],
    /// `Opts::bbox_margin` as it stands for this net.
    margin: i32,
    /// The criticality of the sink a set of nodes completes (0:
    /// congestion alone).
    crit: &'a dyn Fn(&[u32]) -> f32,
    /// The net's first route: `Opts::detour` applies.
    first: bool,
}

/// What a two-front search of one connection needs from `route_net`.
struct Fronts<'a> {
    /// The sink: any one node of it completes the connection.
    set: &'a [u32],
    /// The net's tree so far, each node with its start cost.
    starts: &'a [(u32, f32)],
    tag: u32,
    vcc: bool,
    pres: f32,
    w: f32,
    kd: f32,
    yields: f32,
    /// Cost units per tile of distance, both fronts.
    astar: f32,
    aims: &'a [(i16, i16)],
    /// The forward front's lower bound on the sink's entry cost.
    eb: &'a dyn Fn(u32) -> f32,
}

/// An edge's cost with delay in it: `congestion` (the node's cost) against
/// `delay` (cost units) by the net-blind weight `w`, plus `delay` by the
/// sink's own weight `kd` (`tau * criticality`). `yields` scales the
/// congestion term: below 1 for a critical sink under
/// `TimingView::discount`, which then gives way to other nets less.
fn timed_step(congestion: f32, delay: f32, w: f32, kd: f32, yields: f32) -> f32 {
    (1.0 - w) * yields * congestion + (w + kd) * delay
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
        let fabric: Vec<bool> = (0..n as u32).map(|m| is_fabric_wire(&g.node_name(m))).collect();
        let opts_bidir = opts.bidir;
        Router {
            muxed,
            fabric,
            g,
            opts,
            occ: vec![0; n],
            hist: vec![0.0; n],
            reserved: vec![false; n],
            vcc_kept: vec![false; n],
            cost: vec![f32::INFINITY; n],
            prev: vec![(NONE, 0, 0); n],
            touched: Vec::new(),
            tag: vec![0; n],
            cur_tag: 0,
            diag: SearchDiag::default(),
            dump_net: std::env::var("OXIBIT_DUMP_NET").map(|v| v.split(',').filter_map(|x| x.parse().ok()).collect()).unwrap_or_default(),
            pushes: 0,
            cur_net: usize::MAX,
            cost_b: if opts_bidir { vec![f32::INFINITY; n] } else { Vec::new() },
            next_b: if opts_bidir { vec![(NONE, 0, 0); n] } else { Vec::new() },
            touched_b: Vec::new(),
            popped_f: if opts_bidir { vec![0; n] } else { Vec::new() },
            popped_b: if opts_bidir { vec![0; n] } else { Vec::new() },
            search_no: 0,
            cleanup: false,
            by_f: std::env::var_os("OXIBIT_BY_F").is_some(),
        }
    }

    /// Keep every net off `nodes` (as off a kept net's).
    pub fn forbid(&mut self, nodes: impl IntoIterator<Item = u32>) {
        for n in nodes {
            self.reserved[n as usize] = true;
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
        let mut seen = HashSet::default();
        if net.class == NetClass::Constant {
            // A tie-off source's hard fan-out (DSP_GND_L to every DSP
            // control pin) is a default the pin's own routing overrides.
            seen.insert(net.source);
            out.push(net.source);
        } else {
            self.closure(net.source, &mut out, &mut seen);
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

    fn node_cost(&self, n: u32, pres: f32) -> f32 {
        let others = self.occ[n as usize] as f32;
        (1.0 + self.hist[n as usize]) * (1.0 + pres * others)
    }

    /// Congestion part of entering `n` during the current net's search: its
    /// cost as `route_net` charges it, less the uncongested cost of 1 that
    /// the distance term already stands for (tree nodes are free).
    fn enter_excess(&self, n: u32, pres: f32, tag: u32) -> f32 {
        if self.tag[n as usize] == tag { 0.0 } else { self.node_cost(n, pres) - 1.0 }
    }

    /// Lower bound on the congestion cost still to pay, beyond the
    /// uncongested cost the distance term covers, to reach a node of `set`:
    /// 0 on the set, the target's own excess on its drivers, two hops'
    /// worth on theirs; every other node pays at least three hops' worth
    /// (`far`). Delay terms are ignored (a lower bound).
    fn entry_bound(&self, set: &[u32], pres: f32, tag: u32) -> (HashMap<u32, f32>, f32) {
        let mut near: HashMap<u32, f32> = HashMap::default();
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

    /// Tiles from `n` to the nearest of `aims`: from the nearest tile of
    /// the node with `Opts::span_aware`, else from `Graph::node_xy`'s one
    /// tile, which for a wire of several tiles is an arbitrary one (a long
    /// line towards the target then looks like no progress, or like 18
    /// tiles of it, by the order the tiles happen to come in).
    fn to_aims(&self, n: u32, aims: &[(i16, i16)]) -> f32 {
        if self.opts.span_aware {
            let (x0, y0, x1, y1) = self.g.node_box(n);
            let off = |a: i16, lo: i16, hi: i16| (lo - a).max(a - hi).max(0) as i32;
            return aims.iter().map(|&(ax, ay)| off(ax, x0, x1) + off(ay, y0, y1)).min().unwrap_or(0) as f32;
        }
        let (x, y) = self.g.node_xy(n);
        aims.iter().map(|&(ax, ay)| ((x - ax).abs() + (y - ay).abs()) as i32).min().unwrap_or(0) as f32
    }

    /// The tiles a node spans (`Opts::span_aware`), else its one tile.
    fn node_span(&self, n: u32) -> (i16, i16, i16, i16) {
        if self.opts.span_aware {
            self.g.node_box(n)
        } else {
            let (x, y) = self.g.node_xy(n);
            (x, y, x, y)
        }
    }

    /// Whether a search for `net` may take the edge `(tile, pi)` out of
    /// node `from`.
    fn edge_ok(&self, net: &Net, from: u32, tile: u32, pi: u32) -> bool {
        match self.g.edge_kind(tile, pi) {
            EdgeKind::Feature | EdgeKind::Fixed => true,
            EdgeKind::Relay if self.g.edge_site_through(tile, pi) => true,
            EdgeKind::Relay if net.class == NetClass::GlobalClock && self.g.edge_pip(tile, pi).tile.starts_with("CLK_") => true,
            // A constant net's VCC tie reads into an interconnect input
            // through its default (no line).
            EdgeKind::Default if net.class == NetClass::Constant && from == net.source => true,
            EdgeKind::Relay | EdgeKind::Default => false,
        }
    }

    /// One connection searched from both ends (`Opts::bidir`). The forward
    /// front is the forward search's: from the tree, its costs, its
    /// estimate. The backward front starts at the sink at cost 0 and
    /// follows edges in reverse; each step costs what the forward search
    /// pays to enter the node it steps from, so a path's two halves add up
    /// to its forward cost. Its estimate is the distance to the tree's box.
    /// The fronts take turns, one pop each, and stop at the first node
    /// both have popped. (Stopping where one pops a node the other has
    /// only reached, at whatever provisional cost, kept picoBEAM from
    /// converging: 7718 nodes still overused at iteration 141.)
    ///
    /// On success the backward half is written into `prev` (as if the
    /// forward search had found it) and the sink node reached is returned;
    /// the caller reconstructs and resets as after a forward search. None:
    /// the fronts never met, or the halves would form a loop; the forward
    /// scratch is then dirty and the caller resets it.
    fn search_bidir(&mut self, net: &Net, f: &Fronts, pops: &mut u64) -> Option<u32> {
        let g = self.g;
        let tag = f.tag;
        self.search_no += 1;
        let sno = self.search_no;
        let mut tb = (i16::MAX, i16::MAX, i16::MIN, i16::MIN);
        for &(t, _) in f.starts {
            let b = self.node_span(t);
            tb = (tb.0.min(b.0), tb.1.min(b.1), tb.2.max(b.2), tb.3.max(b.3));
        }
        // Tiles between a node's span and the tree's box.
        let gap = |lo: i16, hi: i16, tlo: i16, thi: i16| (tlo as i32 - hi as i32).max(lo as i32 - thi as i32).max(0);
        let to_tree = |b: (i16, i16, i16, i16)| (gap(b.0, b.2, tb.0, tb.2) + gap(b.1, b.3, tb.1, tb.3)) as f32;
        let step = |r: &Self, d: u32, tile: u32, pi: u32| {
            if r.tag[d as usize] == tag {
                0.0
            } else if f.w > 0.0 || f.kd > 0.0 {
                timed_step(r.node_cost(d, f.pres), g.edge_timing(tile, pi).delay as f32 / r.opts.delay_norm, f.w, f.kd, f.yields)
            } else {
                r.node_cost(d, f.pres)
            }
        };
        let clock = net.class == NetClass::GlobalClock;
        let blocked = |r: &Self, d: u32| {
            r.tag[d as usize] != tag && (r.reserved[d as usize] || (r.vcc_kept[d as usize] && !f.vcc) || (clock && r.fabric[d as usize])) && !f.set.contains(&d)
        };

        let mut fwd = BinaryHeap::new();
        for &(t, start) in f.starts {
            self.cost[t as usize] = start;
            self.prev[t as usize] = (NONE, 0, 0);
            self.touched.push(t);
            fwd.push(Item { f: start + f.astar * self.to_aims(t, f.aims) + (f.eb)(t), g: start, node: t });
        }
        let mut bwd = BinaryHeap::new();
        for &t in f.set {
            if self.cost_b[t as usize].is_infinite() {
                self.touched_b.push(t);
            }
            self.cost_b[t as usize] = 0.0;
            self.next_b[t as usize] = (NONE, 0, 0);
            bwd.push(Item { f: f.astar * to_tree(self.node_span(t)), g: 0.0, node: t });
        }

        let mut meet = None;
        let mut forward_turn = false;
        while !(fwd.is_empty() && bwd.is_empty()) {
            forward_turn = !forward_turn;
            let forward = if fwd.is_empty() { false } else if bwd.is_empty() { true } else { forward_turn };
            if forward {
                let Some(Item { g: gi, node, .. }) = fwd.pop() else { break };
                if gi > self.cost[node as usize] {
                    continue;
                }
                *pops += 1;
                self.popped_f[node as usize] = sno;
                if self.popped_b[node as usize] == sno {
                    meet = Some(node);
                    break;
                }
                let base = self.cost[node as usize];
                for &(d, tile, pi) in g.out_edges(node) {
                    if !self.edge_ok(net, node, tile, pi) || blocked(self, d) {
                        continue;
                    }
                    let c = base + step(self, d, tile, pi);
                    if c < self.cost[d as usize] {
                        if self.cost[d as usize].is_infinite() {
                            self.touched.push(d);
                        }
                        self.cost[d as usize] = c;
                        self.prev[d as usize] = (node, tile, pi);
                        self.pushes += 1;
                        fwd.push(Item { f: c + f.astar * self.to_aims(d, f.aims) + (f.eb)(d), g: c, node: d });
                    }
                }
            } else {
                let Some(Item { g: gi, node, .. }) = bwd.pop() else { break };
                if gi > self.cost_b[node as usize] {
                    continue;
                }
                *pops += 1;
                self.popped_b[node as usize] = sno;
                if self.popped_f[node as usize] == sno {
                    meet = Some(node);
                    break;
                }
                let base = self.cost_b[node as usize];
                for (p, tile, pi) in g.in_edges(node) {
                    if !self.edge_ok(net, p, tile, pi) || blocked(self, p) {
                        continue;
                    }
                    let c = base + step(self, node, tile, pi);
                    if c < self.cost_b[p as usize] {
                        if self.cost_b[p as usize].is_infinite() {
                            self.touched_b.push(p);
                        }
                        self.cost_b[p as usize] = c;
                        self.next_b[p as usize] = (node, tile, pi);
                        self.pushes += 1;
                        bwd.push(Item { f: c + f.astar * to_tree(self.node_span(p)), g: c, node: p });
                    }
                }
            }
        }

        // Splice the backward half into `prev`, unless it would cross the
        // forward half (a loop) or the tree.
        let mut out = None;
        if let Some(m) = meet {
            let total = self.cost[m as usize] + self.cost_b[m as usize];
            let mut forward_half: HashSet<u32> = HashSet::default();
            let mut n = m;
            while self.prev[n as usize].0 != NONE && forward_half.insert(n) {
                n = self.prev[n as usize].0;
            }
            let mut n = m;
            let mut ok = true;
            while self.next_b[n as usize].0 != NONE {
                let (to, tile, pi) = self.next_b[n as usize];
                if forward_half.contains(&to) || self.tag[to as usize] == tag {
                    ok = false;
                    break;
                }
                if self.cost[to as usize].is_infinite() {
                    self.touched.push(to);
                    self.cost[to as usize] = total;
                }
                self.prev[to as usize] = (n, tile, pi);
                n = to;
            }
            if ok {
                self.cost[n as usize] = total;
                out = Some(n);
            }
        }
        for &n in &self.touched_b {
            self.cost_b[n as usize] = f32::INFINITY;
            self.next_b[n as usize] = (NONE, 0, 0);
        }
        self.touched_b.clear();
        out
    }

    /// Where a target set is entered: the muxed nodes its fixed in-edges
    /// lead back to (the target itself when it is muxed), at most 16.
    fn aims(&self, set: &[u32]) -> Vec<(i16, i16)> {
        if !self.opts.entry_aim {
            return vec![self.g.node_xy(set[0])];
        }
        let mut out: Vec<(i16, i16)> = Vec::new();
        let mut seen: HashSet<u32> = HashSet::default();
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
                let (mut out, mut seen) = (Vec::new(), HashSet::default());
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
            if ok && !path.is_empty() && n == net.source {
                for k in path {
                    keep[k] = true;
                }
            }
        }
        hops.iter().zip(&keep).filter(|(_, k)| **k).map(|(h, _)| *h).collect()
    }

    /// Route one net as `how` says. Its own previous occupancy must
    /// already be removed.
    fn route_net(&mut self, net: &Net, pres: f32, pops: &mut u64, outside: &mut u64, how: Reroute) -> (Vec<Hop>, usize) {
        let Reroute { keep, margin, crit, first } = how;
        self.cur_tag += 1;
        self.diag = SearchDiag::default();
        let tag = self.cur_tag;
        let vcc = net.class == NetClass::Constant && self.g.node_name(net.source).to_string().contains("VCC");
        let clock = net.class == NetClass::GlobalClock;
        let mut tree: Vec<u32> = vec![net.source];
        self.tag[net.source as usize] = tag;
        // Delay (ps) from the source to each tree node.
        let mut arrival: HashMap<u32, f32> = HashMap::from_iter([(net.source, 0.0)]);
        let mut hops: Vec<Hop> = keep.to_vec();
        let mut failed = 0;
        // The kept subtree, its arrivals in order out from the source.
        if !keep.is_empty() {
            let mut out: HashMap<u32, Vec<usize>> = HashMap::default();
            for (k, h) in keep.iter().enumerate() {
                out.entry(h.src).or_default().push(k);
            }
            let mut stack = vec![net.source];
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
        targets.sort_by(|a, b| self.manhattan(net.source, a[0]).total_cmp(&self.manhattan(net.source, b[0])));
        // The critical ones before the rest (a stable sort: nearest first
        // within equals), so they take the tree's direct paths.
        targets.sort_by(|a, b| crit(b).total_cmp(&crit(a)));

        // The terminals' bounding box, grown by the margin.
        let bbox = (margin >= 0).then(|| {
            let m = margin;
            let (sx, sy) = self.g.node_xy(net.source);
            let (mut x0, mut y0, mut x1, mut y1) = (sx as i32, sy as i32, sx as i32, sy as i32);
            for &t in targets.iter().flatten() {
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
            let cleanup = self.cleanup;
            let (near, far) = if self.opts.estimate.entry && !cleanup { self.entry_bound(set, pres, tag) } else { (HashMap::default(), 0.0) };
            let w = self.opts.delay_weight;
            // The sink's own delay weight, on top of the net-blind `w`,
            // and how much of the congestion cost it still pays.
            let c = crit(set);
            let (kd, yields, greed) = self.opts.timing.as_ref().map_or((0.0, 1.0, 1.0), |v| (v.tau * c, 1.0 - v.discount * c, 1.0 - v.exact * c));
            let eb = |n: u32| yields * near.get(&n).copied().unwrap_or(far);
            // A critical sink's search leans less on the distance estimate
            // (`TimingView::exact`): slower, nearer the cheapest path.
            let astar = if cleanup { CLEANUP_ASTAR } else { greed * ((1.0 - w) * yields * self.opts.estimate.per_tile + (w + kd) * self.opts.astar_delay) };
            // Weight of a tree node's arrival in its start cost.
            let sw = if cleanup { 1.0 } else { w + kd };
            let mut reached = None;
            let mut sd = SearchDiag { cost: f32::INFINITY, dist: i32::MAX, ..Default::default() };
            for &t in &tree {
                sd.dist = sd.dist.min(self.to_aims(t, &aims) as i32);
            }
            sd.h0 = astar * sd.dist as f32 + tree.iter().map(|&t| eb(t)).fold(f32::INFINITY, f32::min).min(1e30);
            if self.opts.bidir && !cleanup {
                let starts: Vec<(u32, f32)> = tree.iter().map(|&t| (t, sw * arrival.get(&t).copied().unwrap_or(0.0) / self.opts.delay_norm)).collect();
                let fronts = Fronts { set, starts: &starts, tag, vcc, pres, w, kd, yields, astar, aims: &aims, eb: &eb };
                reached = self.search_bidir(net, &fronts, pops);
                if reached.is_none() {
                    // Searched again forward, from clean scratch.
                    self.reset_scratch();
                    *outside += 1;
                }
            }
            // The forward search, unless the two-front one found the sink.
            let passes: &[Option<(i32, i32, i32, i32)>] = if reached.is_some() { &[] } else { &[bbox, None] };
            for (pass, boxed) in passes.iter().copied().enumerate() {
                // The first pass is boxed and detour-bounded where those
                // are set; the second, run only when the first finds
                // nothing, is neither.
                let bounded = pass == 0 && first && self.opts.detour.is_some();
                if pass == 1 {
                    self.reset_scratch();
                    *outside += 1;
                }
                let (factor, slack) = self.opts.detour.unwrap_or((f32::INFINITY, 0.0));
                // Least distance to the target of any node popped so far.
                let mut best = sd.dist as f32;
                let inside = |n: u32| {
                    boxed.is_none_or(|(x0, y0, x1, y1)| {
                        let (x, y) = self.g.node_xy(n);
                        let (x, y) = (x as i32, y as i32);
                        x >= x0 && x <= x1 && y >= y0 && y <= y1
                    })
                };
                let mut heap = BinaryHeap::new();
                for &t in &tree {
                    let start = sw * arrival.get(&t).copied().unwrap_or(0.0) / self.opts.delay_norm;
                    self.cost[t as usize] = start;
                    self.prev[t as usize] = (NONE, 0, 0);
                    self.touched.push(t);
                    heap.push(Item { f: start + astar * self.to_aims(t, &aims) + eb(t), g: start, node: t });
                }
                let mut fs: Vec<f32> = Vec::new();
                let by_f = self.by_f;
                let mut dump: Option<std::io::BufWriter<std::fs::File>> = self.dump_net.contains(&self.cur_net).then_some(self.cur_net).and_then(|d| {
                    std::fs::File::create(format!("/tmp/oxibit_pops_{d}_{}.csv", sd.dist)).ok().map(std::io::BufWriter::new)
                });
                while let Some(Item { f, g, node }) = heap.pop() {
                    if self.opts.skip_stale && g > self.cost[node as usize] {
                        continue;
                    }
                    *pops += 1;
                    sd.pops += 1;
                    let dd = self.to_aims(node, &aims) as i32;
                    best = best.min(dd as f32);
                    sd.far = sd.far.max(dd);
                    sd.by_dist[match dd { 0..=2 => 0, 3..=5 => 1, 6..=10 => 2, 11..=20 => 3, 21..=40 => 4, _ => 5 }] += 1;
                    if by_f {
                        fs.push(f);
                    }
                    if let Some(w) = dump.as_mut() {
                        use std::io::Write;
                        let (x, y) = self.g.node_xy(node);
                        let _ = writeln!(w, "{x},{y},{},{f},{},{}", self.cost[node as usize], self.occ[node as usize], self.g.node_name(node));
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
                        if !self.edge_ok(net, node, tile, pi) {
                            continue;
                        }
                        if (self.reserved[d as usize] || (self.vcc_kept[d as usize] && !vcc) || (clock && self.fabric[d as usize])) && !set.contains(&d) {
                            continue;
                        }
                        if !set.contains(&d) && !inside(d) {
                            continue;
                        }
                        if cleanup && self.occ[d as usize] > 0 && self.tag[d as usize] != tag {
                            continue;
                        }
                        let step = if self.tag[d as usize] == tag {
                            0.0
                        } else if cleanup {
                            self.g.edge_timing(tile, pi).delay as f32 / self.opts.delay_norm + CLEANUP_NODE
                        } else if w > 0.0 || kd > 0.0 {
                            timed_step(self.node_cost(d, pres), self.g.edge_timing(tile, pi).delay as f32 / self.opts.delay_norm, w, kd, yields)
                        } else {
                            self.node_cost(d, pres)
                        };
                        let c = base + step;
                        if c < self.cost[d as usize] {
                            let hd = self.to_aims(d, &aims);
                            if bounded && hd > factor * best + slack && !set.contains(&d) {
                                continue;
                            }
                            if self.cost[d as usize].is_infinite() {
                                self.touched.push(d);
                            }
                            self.cost[d as usize] = c;
                            self.prev[d as usize] = (node, tile, pi);
                            self.pushes += 1;
                            heap.push(Item { f: c + astar * hd + eb(d), g: c, node: d });
                        }
                    }
                }
                if let Some(t) = reached {
                    let c = self.cost[t as usize].max(1e-3);
                    for f in fs {
                        let r = f / c;
                        sd.by_f[if r < 0.5 { 0 } else if r < 1.0 { 1 } else if r < 2.0 { 2 } else { 3 }] += 1;
                    }
                }
                if reached.is_some() || pass == 1 || (boxed.is_none() && !bounded) {
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
                        hops.push(Hop { src: from, dst: n, tile, pi });
                        self.tag[n as usize] = tag;
                        tree.push(n);
                        n = from;
                    }
                    self.tag[target as usize] = tag;
                    sd.hops = hops.len() - first;
                    // The new branch's arrivals, from where it left the tree.
                    for h in hops[first..].iter().rev() {
                        let a = arrival.get(&h.src).copied().unwrap_or(0.0) + self.g.edge_timing(h.tile, h.pi).delay as f32;
                        arrival.insert(h.dst, a);
                    }
                }
                None => {
                    failed += 1;
                    // The first few unreachable sinks by name: each is a
                    // placement or import fault, not congestion.
                    static SHOWN: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
                    if SHOWN.fetch_add(1, std::sync::atomic::Ordering::Relaxed) < 200 {
                        let names: Vec<String> = set.iter().map(|&n| self.g.node_name(n)).collect();
                        eprintln!("oxibit: route: unreachable: from {} to {:?} (dist {}, pops {})", self.g.node_name(net.source), names, sd.dist, sd.pops);
                    }
                }
            }
            if sd.pops > self.diag.pops {
                self.diag = sd;
            }
            self.reset_scratch();
        }
        (hops, failed)
    }

    /// Intrinsic delay (ps) from `source` to every node `hops` reach.
    fn arrivals(&self, source: u32, hops: &[Hop]) -> HashMap<u32, f32> {
        let mut out: HashMap<u32, Vec<usize>> = HashMap::default();
        for (k, h) in hops.iter().enumerate() {
            out.entry(h.src).or_default().push(k);
        }
        let mut at = HashMap::from_iter([(source, 0.0)]);
        let mut stack = vec![source];
        while let Some(n) = stack.pop() {
            let a = at[&n];
            for &k in out.get(&n).map(Vec::as_slice).unwrap_or(&[]) {
                let h = hops[k];
                if at.contains_key(&h.dst) {
                    continue;
                }
                at.insert(h.dst, a + self.g.edge_timing(h.tile, h.pi).delay as f32);
                stack.push(h.dst);
            }
        }
        at
    }

    /// The post-route cleanup pass (`Opts::cleanup_rounds`). Each round:
    ///
    /// 1. Fresh slack from `view` (the nets the last round changed
    ///    re-timed), not the criticality the router saw mid-negotiation.
    /// 2. Every sink with negative slack is a target, most critical first.
    /// 3. A target's branch (the hops no other sink of its net needs) is
    ///    ripped up and searched again for delay alone, through nodes no
    ///    other net occupies.
    /// 4. The new branch is kept only if it arrives earlier and its
    ///    footprint overlaps no other net; otherwise the old one stays.
    ///
    /// No other connection, of this net or another, changes: nothing gets
    /// slower and the route stays legal. Stops after a round that keeps
    /// nothing.
    fn cleanup_pass(&mut self, nets: &[Net], routes: &mut [Vec<Hop>], prints: &mut [Vec<u32>], routed: &[bool], view: &TimingView) -> Cleanup {
        let t0 = Instant::now();
        let mut st = Cleanup::default();
        let mut changed: Vec<usize> = Vec::new();
        for round in 0..self.opts.cleanup_rounds {
            let (crit, worst) = view.retime(self.g, nets, routes, &changed, |t| (t.criticality().clone(), t.worst_slack()));
            if round == 0 {
                st.slack_before = worst;
            }
            changed.clear();
            let mut targets: Vec<(f32, usize, Vec<u32>)> = Vec::new();
            for (i, net) in nets.iter().enumerate() {
                if !routed[i] || net.class != NetClass::Signal || crit.of_net(i) <= 0.0 {
                    continue;
                }
                let sets = net.sinks.iter().map(|&s| vec![s]).chain(net.pin_groups.iter().filter(|g| !g.is_empty()).cloned());
                for set in sets {
                    let c = crit.of_sink(i, &set);
                    if c > 0.0 {
                        targets.push((c, i, set));
                    }
                }
            }
            targets.sort_by(|a, b| b.0.total_cmp(&a.0).then(a.1.cmp(&b.1)));
            let mut kept = 0usize;
            for (_, i, set) in targets {
                let net = &nets[i];
                let at = self.arrivals(net.source, &routes[i]);
                let Some((node, old)) = set.iter().find_map(|n| at.get(n).map(|&a| (*n, a))) else { continue };
                st.tried += 1;
                let keep = self.keep_hops(net, &routes[i], &HashSet::from_iter([node]));
                for &n in &prints[i] {
                    self.occ[n as usize] -= 1;
                }
                self.cleanup = true;
                let (mut pops, mut outside) = (0u64, 0u64);
                let none = |_: &[u32]| 0.0_f32;
                let (hops, failed) = self.route_net(net, 1.0, &mut pops, &mut outside, Reroute { keep: &keep, margin: -1, crit: &none, first: false });
                self.cleanup = false;
                st.pops += pops;
                let print = self.footprint(net, &hops);
                let clash = print.iter().any(|&n| self.occ[n as usize] > 0);
                let new_at = self.arrivals(net.source, &hops);
                let new = set.iter().filter_map(|n| new_at.get(n)).copied().fold(f32::INFINITY, f32::min);
                if failed == 0 && !clash && new + 1.0 < old {
                    kept += 1;
                    st.ps_saved += (old - new) as f64;
                    routes[i] = hops;
                    prints[i] = print;
                    changed.push(i);
                }
                for &n in &prints[i] {
                    self.occ[n as usize] += 1;
                }
            }
            changed.sort_unstable();
            changed.dedup();
            st.improved += kept;
            st.rounds = round + 1;
            if kept == 0 {
                break;
            }
        }
        st.slack_after = view.retime(self.g, nets, routes, &changed, |t| t.worst_slack());
        st.seconds = t0.elapsed().as_secs_f64();
        st
    }

    /// The post-route hold pass (`Opts::hold_fix_ps`). A block RAM capture
    /// with less hold slack than the margin has its branch ripped up and
    /// laid again through a waypoint: the net's tree to a free node some
    /// hops off, that node to the pin, both legs through nodes no net
    /// occupies. The connection arrives later; no other connection, of
    /// this net or another, changes, and the route stays legal. Waypoints
    /// are tried nearest first (breadth first out of the old branch); the
    /// first that adds the delay asked for is kept, else the longest
    /// found. Up to three rounds, each on fresh timing. Returns counts.
    fn hold_pass(&mut self, nets: &[Net], routes: &mut [Vec<Hop>], prints: &mut [Vec<u32>], routed: &[bool], view: &TimingView, margin_ps: i64) -> serde_json::Value {
        const DEPTH: usize = 32;
        const PER_DEPTH: usize = 4;
        const FRONT: usize = 96;
        // Candidates in a row without a better detour before a target is
        // given up: a target nothing fixes would otherwise try all of
        // DEPTH x PER_DEPTH, two searches each.
        const GIVE_UP: usize = 24;
        let t0 = Instant::now();
        let (mut tried, mut met, mut partial, mut short_first, mut short_last) = (0usize, 0usize, 0usize, 0usize, 0usize);
        let (mut given_up, mut budget_hit) = (0usize, false);
        let budget = std::time::Duration::from_secs_f64(self.opts.hold_budget_s);
        let mut worst_first = 0i64;
        let mut changed: Vec<usize> = Vec::new();
        let none = |_: &[u32]| 0.0_f32;
        let empty: Vec<Hop> = Vec::new();
        for round in 0..3 {
            let mut shorts = view.hold_targets(self.g, nets, routes, &changed, margin_ps);
            changed.clear();
            shorts.sort_by(|a, b| b.add_ps.cmp(&a.add_ps).then(a.net.cmp(&b.net)).then(a.nodes.cmp(&b.nodes)));
            shorts.dedup_by(|a, b| a.net == b.net && a.nodes == b.nodes);
            if round == 0 {
                short_first = shorts.len();
                worst_first = shorts.iter().map(|s| s.slack_ps).min().unwrap_or(0);
            }
            short_last = shorts.len();
            for s in &shorts {
                if t0.elapsed() > budget {
                    budget_hit = true;
                    break;
                }
                let i = s.net;
                if !routed[i] || nets[i].class != NetClass::Signal {
                    continue;
                }
                let net = &nets[i];
                let at = self.arrivals(net.source, &routes[i]);
                let Some((node, old)) = s.nodes.iter().find_map(|n| at.get(n).map(|&a| (*n, a))) else { continue };
                if !net.sinks.contains(&node) {
                    continue; // a pin the router chooses: not a block RAM's
                }
                tried += 1;
                let want = old + s.add_ps as f32;
                let keep = self.keep_hops(net, &routes[i], &HashSet::from_iter([node]));
                for &n in &prints[i] {
                    self.occ[n as usize] -= 1;
                }
                // Waypoint candidates: free general-routing nodes, breadth
                // first out of the source and the branch being replaced.
                let mut seen: HashSet<u32> = keep.iter().map(|h| h.dst).collect();
                seen.insert(net.source);
                seen.insert(node);
                let mut front: Vec<u32> = vec![net.source];
                for h in routes[i].iter().filter(|h| h.dst != node) {
                    if seen.insert(h.dst) {
                        front.push(h.dst);
                    }
                }
                let mut cands: Vec<u32> = Vec::new();
                for _ in 0..DEPTH {
                    let mut next: Vec<u32> = Vec::new();
                    let mut took = 0;
                    for &n in &front {
                        for &(d, tile, pi) in self.g.out_edges(n) {
                            if !self.edge_ok(net, n, tile, pi) || self.occ[d as usize] > 0 || self.reserved[d as usize] || self.vcc_kept[d as usize] || !seen.insert(d) {
                                continue;
                            }
                            next.push(d);
                            if self.fabric[d as usize] && took < PER_DEPTH {
                                took += 1;
                                cands.push(d);
                            }
                        }
                    }
                    next.truncate(FRONT);
                    front = next;
                    if front.is_empty() {
                        break;
                    }
                }
                let mut best: Option<(f32, Vec<Hop>)> = None;
                self.cleanup = true;
                let mut dry = 0usize;
                for &w in &cands {
                    if dry >= GIVE_UP {
                        given_up += 1;
                        break;
                    }
                    dry += 1;
                    let (mut pops, mut outside) = (0u64, 0u64);
                    let leg1 = Net { source: net.source, sinks: vec![w], pin_groups: Vec::new(), edges: Vec::new(), class: net.class };
                    let (h1, f1) = self.route_net(&leg1, 1.0, &mut pops, &mut outside, Reroute { keep: &keep, margin: -1, crit: &none, first: false });
                    if f1 != 0 {
                        continue;
                    }
                    // The second leg may not touch the tree or the first.
                    let mine: Vec<u32> = std::iter::once(net.source).chain(h1.iter().map(|h| h.dst)).filter(|&n| n != w && !self.reserved[n as usize]).collect();
                    for &n in &mine {
                        self.reserved[n as usize] = true;
                    }
                    let leg2 = Net { source: w, sinks: vec![node], pin_groups: Vec::new(), edges: Vec::new(), class: net.class };
                    let (h2, f2) = self.route_net(&leg2, 1.0, &mut pops, &mut outside, Reroute { keep: &empty, margin: -1, crit: &none, first: false });
                    for &n in &mine {
                        self.reserved[n as usize] = false;
                    }
                    if f2 != 0 {
                        continue;
                    }
                    let mut hops = h1;
                    hops.extend(h2);
                    let new = self.arrivals(net.source, &hops).get(&node).copied().unwrap_or(0.0);
                    let clash = self.footprint(net, &hops).iter().any(|&n| self.occ[n as usize] > 0);
                    if clash || new <= old {
                        continue;
                    }
                    let done = new >= want;
                    if done || best.as_ref().is_none_or(|b| new > b.0) {
                        best = Some((new, hops));
                        dry = 0;
                    }
                    if done {
                        break;
                    }
                }
                self.cleanup = false;
                if let Some((new, hops)) = best {
                    if new >= want {
                        met += 1;
                    } else {
                        partial += 1;
                    }
                    prints[i] = self.footprint(net, &hops);
                    routes[i] = hops;
                    changed.push(i);
                }
                for &n in &prints[i] {
                    self.occ[n as usize] += 1;
                }
            }
            changed.sort_unstable();
            changed.dedup();
            if changed.is_empty() || budget_hit {
                break;
            }
        }
        let left = view.hold_targets(self.g, nets, routes, &changed, margin_ps);
        json!({"margin_ps": margin_ps, "short": short_first, "worst_ps": worst_first, "tried": tried, "met": met, "partial": partial,
               "given_up": given_up, "budget_s": self.opts.hold_budget_s, "budget_hit": budget_hit,
               "short_last_round": short_last, "left": left.len(), "left_worst_ps": left.iter().map(|s| s.slack_ps).min().unwrap_or(0),
               "seconds": t0.elapsed().as_secs_f64()})
    }

    /// Nodes two or more nets occupy under `routes` (one hop list per net),
    /// with the nets on each.
    pub fn audit(&self, nets: &[Net], routes: &[Vec<Hop>]) -> Vec<(u32, Vec<usize>)> {
        let mut on: HashMap<u32, Vec<usize>> = HashMap::default();
        for (i, (net, hops)) in nets.iter().zip(routes).enumerate() {
            for n in self.footprint(net, hops) {
                on.entry(n).or_default().push(i);
            }
        }
        let mut out: Vec<(u32, Vec<usize>)> = on.into_iter().filter(|(_, v)| v.len() > 1).collect();
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
    pub fn run_observed(&mut self, nets: &[Net], progress: &dyn Progress, observe: &mut dyn FnMut(u32, &[u16])) -> (Vec<Vec<Hop>>, Stats) {
        let t0 = Instant::now();
        let mut stats = Stats::default();
        let routed: Vec<bool> = nets.iter().map(|n| matches!(n.class, NetClass::Signal | NetClass::Constant | NetClass::GlobalClock)).collect();

        // Kept nets reserve their nodes; routed nets own their terminals.
        for (net, &r) in nets.iter().zip(&routed) {
            if !r {
                let hops: Vec<Hop> = net.edges.iter().map(|e| Hop { src: e.src, dst: e.dst, tile: e.tile, pi: e.pi }).collect();
                let default = net.class == NetClass::Default;
                for n in self.footprint(net, &hops) {
                    if default {
                        self.vcc_kept[n as usize] = true;
                    } else {
                        self.reserved[n as usize] = true;
                    }
                }
            }
        }
        let mut routes: Vec<Vec<Hop>> = vec![Vec::new(); nets.len()];
        let mut prints: Vec<Vec<u32>> = vec![Vec::new(); nets.len()];
        for (i, net) in nets.iter().enumerate() {
            if routed[i] {
                prints[i] = self.footprint(net, &[]);
                for &n in &prints[i] {
                    self.occ[n as usize] += 1;
                }
            }
        }

        let mut pres = self.opts.schedule.first();
        let mut margin: Vec<i32> = vec![self.opts.bbox_margin; nets.len()];
        let mut todo: Vec<usize> = (0..nets.len()).filter(|&i| routed[i]).collect();
        stats.routed_nets = todo.len();
        // Timing: the criticalities of the last analysis, what each net
        // was last routed with, and the nets this iteration rips up for it.
        let view = self.opts.timing.clone();
        let mut crit: Option<CriticalityMap> = None;
        let mut routed_with: Vec<f32> = vec![0.0; nets.len()];
        let mut hot: HashSet<usize> = HashSet::default();
        // The end game's area: tiles within its radius of an overused node.
        let mut near: HashSet<(i16, i16)> = HashSet::default();
        // With a handful of nodes left overused the capped present cost can
        // lose to a detour for good: one net then takes a different shared
        // node every pass (ms5: 1 overused from pass 187 to 200 at cap 8,
        // a new node each pass). Past the cap the cost doubles per pass
        // until sharing is never the cheaper path.
        let mut boost = 1.0f32;
        // Stagnation: passes in a row with a handful of nodes overused and
        // the count not falling. Negotiation has stopped moving them, so
        // each is settled exactly (`OXIBIT_STUCK` passes, 0 = never).
        let stuck_limit: u32 = std::env::var("OXIBIT_STUCK").ok().and_then(|v| v.parse().ok()).unwrap_or(16);
        let stuck_max: usize = std::env::var("OXIBIT_STUCK_MAX").ok().and_then(|v| v.parse().ok()).unwrap_or(32);
        let mut stuck = 0u32;
        let mut last_small = usize::MAX;
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
                // A net ripped up for timing also gives up the branches to
                // its critical sinks.
                let mut bad: HashSet<u32> = prints[i].iter().copied().filter(|&n| self.occ[n as usize] > 1).collect();
                if let (Some(c), true) = (&crit, hot.contains(&i)) {
                    bad.extend(c.hot_sinks(i));
                }
                // In the end game a signal net gives up its branches
                // through the area around the overuse.
                if !near.is_empty() && nets[i].class == NetClass::Signal {
                    bad.extend(prints[i].iter().copied().filter(|&n| near.contains(&self.g.node_xy(n))));
                }
                let keep = if iter == 1 { Vec::new() } else { self.keep_hops(&nets[i], &routes[i], &bad) };
                let weigh = |set: &[u32]| crit.as_ref().map_or(0.0, |c| c.of_sink(i, set));
                kept += keep.len();
                for &n in &prints[i] {
                    self.occ[n as usize] -= 1;
                }
                let before = pops;
                self.cur_net = if iter == 1 { i } else { usize::MAX };
                let (hops, f) = oxibit_prof::hot_block!("route.net", [net = i, sinks = nets[i].sinks.len(), pops = ::tracing::field::Empty], {
                    let r = self.route_net(&nets[i], pres * boost, &mut pops, &mut outside, Reroute { keep: &keep, margin: margin[i], crit: &weigh, first: iter == 1 });
                    oxibit_prof::hot_record!(pops, pops - before);
                    r
                });
                costly.push((pops - before, i, self.diag));
                if margin[i] >= 0 {
                    margin[i] = margin[i].saturating_mul(2).max(1);
                }
                failed += f;
                routed_with[i] = crit.as_ref().map_or(0.0, |c| c.of_net(i));
                prints[i] = self.footprint(&nets[i], &hops);
                for &n in &prints[i] {
                    self.occ[n as usize] += 1;
                }
                routes[i] = hops;
            }
            stats.heap_pops += pops;
            stats.unroutable_sinks = failed;

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
            // The last few contested nodes, and who holds them.
            if over > 0 && over <= 8 && std::env::var_os("OXIBIT_ROUTE_CONTEST").is_some() {
                for (n, &o) in self.occ.iter().enumerate().filter(|&(_, &o)| o > 1) {
                    let holders: Vec<String> = (0..nets.len())
                        .filter(|&i| prints[i].contains(&(n as u32)))
                        .map(|i| format!("{}({:?},{} sinks)", self.g.node_name(nets[i].source), nets[i].class, nets[i].sinks.len()))
                        .collect();
                    eprintln!("\noxibit: route: contest: iter {iter} {} occ {o} hist {:.1}: {holders:?}", self.g.node_name(n as u32), self.hist[n]);
                }
            }
            if over > 0 && over <= stuck_max {
                // Stagnation is no new minimum, so an oscillation (1, 4, 1, 4) counts.
                if over < last_small { last_small = over; stuck = 0; } else { stuck += 1; }
            }
            // The exact step. For each overused node, each net on it in
            // turn is taken out and routed again with every node any other
            // net holds forbidden outright. A route found that way is legal
            // as it stands and is kept: the conflict is gone. If none of a
            // node's nets has one, no routing of this placement frees the
            // node while the others stay put, and the report says which
            // sinks could not be reached: a demand to chase back to the
            // pack or the placement, not to negotiate over.
            let mut gave_up = false;
            let over = if stuck_limit > 0 && over > 0 && ((stuck >= stuck_limit && over <= last_small) || (iter == self.opts.max_iters && over <= stuck_max)) {
                let contested: Vec<u32> = self.occ.iter().enumerate().filter(|&(_, &o)| o > 1).map(|(n, _)| n as u32).collect();
                let mut report = Vec::new();
                for &n in &contested {
                    let holders: Vec<usize> = (0..nets.len()).filter(|&i| routed[i] && prints[i].contains(&n)).collect();
                    let mut tried = Vec::new();
                    for &i in &holders {
                        if self.occ[n as usize] <= 1 {
                            break;
                        }
                        for &m in &prints[i] {
                            self.occ[m as usize] -= 1;
                        }
                        let saved = self.reserved.clone();
                        for m in 0..self.occ.len() {
                            if self.occ[m] > 0 {
                                self.reserved[m] = true;
                            }
                        }
                        let none = |_: &[u32]| 0.0;
                        let (mut p, mut o) = (0u64, 0u64);
                        self.cur_net = usize::MAX;
                        let (hops, f) = self.route_net(&nets[i], 1.0, &mut p, &mut o, Reroute { keep: &Vec::new(), margin: 1 << 20, crit: &none, first: false });
                        self.reserved = saved;
                        stats.heap_pops += p;
                        let print = self.footprint(&nets[i], &hops);
                        let shared = print.iter().filter(|&&m| self.occ[m as usize] > 0).count();
                        if f == 0 && shared == 0 {
                            for &m in &print {
                                self.occ[m as usize] += 1;
                            }
                            routes[i] = hops;
                            prints[i] = print;
                            tried.push(json!({"net": self.g.node_name(nets[i].source), "moved": true, "pops": p}));
                        } else {
                            for &m in &prints[i] {
                                self.occ[m as usize] += 1;
                            }
                            tried.push(json!({"net": self.g.node_name(nets[i].source), "moved": false, "unreached_sinks": f, "still_shared": shared,
                                              "sinks": nets[i].sinks.len(), "groups": nets[i].pin_groups.len(), "pops": p}));
                        }
                    }
                    report.push(json!({"node": self.g.node_name(n), "settled": self.occ[n as usize] <= 1, "nets": tried}));
                }
                // The second ring. A node neither of its nets can leave alone
                // may still be freed by moving a third: each net on it in
                // turn is routed off the node at negotiation cost (it may
                // cross others), then every net it now crosses is routed
                // again with all else forbidden. Kept only if every one of
                // those finds a clean route; otherwise put back as it was.
                let still: Vec<u32> = contested.iter().copied().filter(|&n| self.occ[n as usize] > 1).collect();
                for &n in &still {
                    let holders: Vec<usize> = (0..nets.len()).filter(|&i| routed[i] && prints[i].contains(&n)).collect();
                    for &i in &holders {
                        if self.occ[n as usize] <= 1 {
                            break;
                        }
                        let none = |_: &[u32]| 0.0;
                        let (mut p, mut o) = (0u64, 0u64);
                        for &m in &prints[i] {
                            self.occ[m as usize] -= 1;
                        }
                        let was = std::mem::replace(&mut self.reserved[n as usize], true);
                        self.cur_net = usize::MAX;
                        let (hops, f) = self.route_net(&nets[i], 1.0e3, &mut p, &mut o, Reroute { keep: &Vec::new(), margin: 1 << 20, crit: &none, first: false });
                        self.reserved[n as usize] = was;
                        let print = self.footprint(&nets[i], &hops);
                        let crossed: HashSet<u32> = print.iter().copied().filter(|&m| self.occ[m as usize] > 0).collect();
                        let blockers: Vec<usize> = (0..nets.len()).filter(|&j| j != i && routed[j] && prints[j].iter().any(|m| crossed.contains(m))).collect();
                        if f != 0 || blockers.len() > 6 {
                            for &m in &prints[i] {
                                self.occ[m as usize] += 1;
                            }
                            report.push(json!({"node": self.g.node_name(n), "ring2": self.g.node_name(nets[i].source), "moved": false, "unreached_sinks": f, "blockers": blockers.len()}));
                            continue;
                        }
                        // Tentatively: the net on its new route, each blocker moved in turn.
                        let mut undo: Vec<(usize, Vec<Hop>, Vec<u32>)> = vec![(i, std::mem::replace(&mut routes[i], hops), std::mem::replace(&mut prints[i], print))];
                        for &m in &prints[i] {
                            self.occ[m as usize] += 1;
                        }
                        let mut ok = true;
                        for &j in &blockers {
                            for &m in &prints[j] {
                                self.occ[m as usize] -= 1;
                            }
                            let saved = self.reserved.clone();
                            for m in 0..self.occ.len() {
                                if self.occ[m] > 0 {
                                    self.reserved[m] = true;
                                }
                            }
                            let (hj, fj) = self.route_net(&nets[j], 1.0, &mut p, &mut o, Reroute { keep: &Vec::new(), margin: 1 << 20, crit: &none, first: false });
                            self.reserved = saved;
                            let pj = self.footprint(&nets[j], &hj);
                            if fj != 0 || pj.iter().any(|&m| self.occ[m as usize] > 0) {
                                for &m in &prints[j] {
                                    self.occ[m as usize] += 1;
                                }
                                ok = false;
                                break;
                            }
                            for &m in &pj {
                                self.occ[m as usize] += 1;
                            }
                            undo.push((j, std::mem::replace(&mut routes[j], hj), std::mem::replace(&mut prints[j], pj)));
                        }
                        stats.heap_pops += p;
                        let moved: Vec<String> = undo.iter().map(|u| self.g.node_name(nets[u.0].source).to_string()).collect();
                        if !ok {
                            for (j, h, pr) in undo.into_iter().rev() {
                                for &m in &prints[j] {
                                    self.occ[m as usize] -= 1;
                                }
                                routes[j] = h;
                                prints[j] = pr;
                                for &m in &prints[j] {
                                    self.occ[m as usize] += 1;
                                }
                            }
                        }
                        report.push(json!({"node": self.g.node_name(n), "ring2": moved, "moved": ok, "blockers": blockers.len(), "pops": p}));
                    }
                }
                let left = self.occ.iter().filter(|&&o| o > 1).count();
                eprintln!("\noxibit: route: stuck at {over} overused for {stuck} passes: exact step leaves {left}: {}", serde_json::Value::Array(report.clone()));
                progress.emit(Event::progress("route", json!({"iter": iter, "stuck": {"passes": stuck, "before": over, "after": left, "nodes": report}})));
                gave_up = left > 0;
                stats.overused = left;
                left
            } else {
                over
            };
            observe(iter, &self.occ);
            // Re-time the nets just routed; pick the next iteration's hot
            // nets while passes remain.
            let mut timing = serde_json::Value::Null;
            hot.clear();
            if let Some(v) = &view {
                let (c, summary) = v.retime(self.g, nets, &routes, &todo, |t| (t.criticality().clone(), t.summary()));
                if stats.timing_passes < v.passes && !stats.cancelled {
                    hot = v.hot(&c, &routed_with, |i| routed[i] && nets[i].class == NetClass::Signal).into_iter().collect();
                }
                if !hot.is_empty() {
                    stats.timing_passes += 1;
                    stats.timing_ripups += hot.len();
                }
                timing = json!({"table": summary, "rip_up_next": hot.len()});
                crit = Some(c);
            }
            costly.sort_unstable_by(|a, b| (b.0, b.1).cmp(&(a.0, a.1)));
            // Pops by what the net connects: its source's tile type, then
            // its sinks' (slices of either kind as CLB), as `SRC>DST+DST`.
            let tile_class = |n: u32| -> String {
                let name = self.g.node_name(n).to_string();
                let tile = name.split('/').next().unwrap_or("");
                let ty = tile.rsplit_once("_X").map_or(tile, |(t, _)| t);
                if ty.starts_with("CLB") { "CLB".to_string() } else { ty.to_string() }
            };
            let mut by_ends: std::collections::BTreeMap<String, (u64, u64, u64)> = std::collections::BTreeMap::new();
            for &(p, i, _) in &costly {
                let net = &nets[i];
                let mut to: Vec<String> = net.sinks.iter().copied().chain(net.pin_groups.iter().filter_map(|g| g.first().copied())).map(tile_class).collect();
                to.sort();
                to.dedup();
                let e = by_ends.entry(format!("{}>{}", tile_class(net.source), to.join("+"))).or_default();
                e.0 += p;
                e.1 += 1;
                e.2 += (net.sinks.len() + net.pin_groups.len()) as u64;
            }
            let by_ends: Vec<_> = by_ends.into_iter().map(|(k, (p, n, s))| json!([k, p, n, s])).collect();
            let costliest: Vec<_> = costly
                .iter()
                .take(5)
                .map(|&(p, i, d)| {
                    json!({"pops": p, "net": i, "sinks": nets[i].sinks.len(), "groups": nets[i].pin_groups.len(),
                           "source": self.g.node_name(nets[i].source), "worst_search": d.to_json()})
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
                    "timing": timing,
                    "unroutable": failed,
                    "pops": pops,
                    "kept_hops": kept,
                    "unboxed_searches": outside,
                    "costliest": costliest,
                    "top100_pops": costly.iter().take(100).map(|c| c.0).sum::<u64>(),
                    // [ends, pops, nets, sinks] per pair of tile classes.
                    "pops_by_ends": by_ends,
                    // [pops, sinks, h0, cost, far, dist, congested_pops, hops] of the 40 costliest.
                    "worst40": costly.iter().take(40).map(|&(p, i, d)| json!([p, nets[i].sinks.len() + nets[i].pin_groups.len(), d.h0, if d.cost.is_finite() { d.cost } else { -1.0 }, d.far, d.dist, d.congested_pops, d.hops, d.by_dist, d.by_f])).collect::<Vec<_>>(),
                    "iter_ms": ti.elapsed().as_millis() as u64,
                }),
            ));
            if (over == 0 && hot.is_empty()) || stats.cancelled || gave_up {
                break;
            }
            near.clear();
            if let Some(e) = self.opts.endgame
                && over > 0
                && over <= e.overuse
            {
                for (n, &o) in self.occ.iter().enumerate() {
                    if o > 1 {
                        let (x, y) = self.g.node_xy(n as u32);
                        for dx in -e.radius..=e.radius {
                            for dy in -e.radius..=e.radius {
                                near.insert((x + dx, y + dy));
                            }
                        }
                    }
                }
            }
            todo = (0..nets.len())
                .filter(|&i| {
                    routed[i]
                        && (hot.contains(&i)
                            || prints[i].iter().any(|&n| {
                                self.occ[n as usize] > 1 || (nets[i].class == NetClass::Signal && !near.is_empty() && near.contains(&self.g.node_xy(n)))
                            }))
                })
                .collect();
            pres = self.opts.schedule.next(pres);
            boost = if over <= 8 && std::env::var_os("OXIBIT_NO_BOOST").is_none() { (boost * 2.0).min(65536.0) } else { 1.0 };
        }
        if let (Some(v), Some(margin)) = (&view, self.opts.hold_fix_ps.filter(|_| stats.overused == 0 && !stats.cancelled)) {
            let h = self.hold_pass(nets, &mut routes, &mut prints, &routed, v, margin);
            eprintln!("oxibit: route: hold: {h}");
            progress.emit(Event::progress("route", json!({"hold_fix": h})));
        }
        if let (Some(v), true) = (&view, self.opts.cleanup_rounds > 0 && stats.overused == 0 && !stats.cancelled) {
            let c = self.cleanup_pass(nets, &mut routes, &mut prints, &routed, v);
            progress.emit(Event::progress("route", json!({"cleanup": c.to_json()})));
            stats.heap_pops += c.pops;
            stats.cleanup = c.to_json();
        }
        stats.seconds = t0.elapsed().as_secs_f64();
        stats.heap_pushes = self.pushes;
        if let Some(path) = &self.opts.overuse_dump {
            let mut text = String::new();
            for (n, &o) in self.occ.iter().enumerate() {
                if o > 1 {
                    // The nets on it: source, class, and its sinks in this
                    // node's tile (a pin two nets both need shows here).
                    let tile = self.g.node_name(n as u32).to_string();
                    let tile = tile.split('/').next().unwrap_or("").to_string();
                    let on: Vec<String> = (0..nets.len())
                        .filter(|&i| prints[i].contains(&(n as u32)))
                        .map(|i| {
                            let near: Vec<String> = nets[i]
                                .sinks
                                .iter()
                                .map(|&s| self.g.node_name(s).to_string())
                                .filter(|s| s.starts_with(&tile))
                                .collect();
                            format!("{} {:?} [{}]", self.g.node_name(nets[i].source), nets[i].class, near.join(" "))
                        })
                        .collect();
                    text.push_str(&format!("{}\t{o}\t{}\t{}\n", self.g.node_name(n as u32), self.hist[n], on.join(" | ")));
                }
            }
            let _ = std::fs::write(path, text);
        }
        (routes, stats)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sta::Stub;
    use std::cell::RefCell;
    use std::rc::Rc;

    #[test]
    fn a_sink_that_meets_its_clock_pays_congestion_alone() {
        assert_eq!(timed_step(3.0, 2.5, 0.0, 0.0, 1.0), 3.0);
    }

    #[test]
    fn a_critical_sink_pays_for_delay_in_proportion() {
        // tau 2, criticality 0.5: one cost unit per unit of delay.
        assert_eq!(timed_step(3.0, 2.5, 0.0, 2.0 * 0.5, 1.0), 5.5);
        // On top of the net-blind weight, which trades congestion away.
        assert_eq!(timed_step(4.0, 2.0, 0.25, 1.0, 1.0), 3.0 + 2.5);
    }

    #[test]
    fn a_discounted_critical_sink_gives_way_less() {
        // Discount 0.9 at criticality 1: a tenth of the congestion cost,
        // all of the delay cost.
        let yields = 1.0 - 0.9 * 1.0;
        assert!((timed_step(10.0, 2.0, 0.0, 3.0, yields) - (1.0 + 6.0)).abs() < 1e-5);
    }

    #[test]
    fn the_view_in_the_options_needs_no_design_or_clock() {
        let mut view = TimingView::new(Rc::new(RefCell::new(Stub::with_nets(vec![0.0, 0.5]))));
        view.tau = 2.0;
        let opts = Opts { timing: Some(view), ..Opts::default() };
        // Options stay cloneable and printable with a source behind them.
        let again = opts.clone();
        assert!(format!("{again:?}").contains("tau: 2.0"));
        assert!(Opts::default().timing.is_none());
    }

    #[test]
    fn a_clock_may_not_cross_general_routing() {
        // The path picoBEAM's pb_clk took off its GCLK line to slice CLK
        // pins: every step general routing.
        for n in ["INT_R_X19Y100/BYP_BOUNCE1", "INT_R_X19Y100/FAN_ALT5", "INT_R_X19Y100/FAN_BOUNCE5", "INT_R_X19Y100/BYP_ALT1"] {
            assert!(is_fabric_wire(n), "{n}");
        }
        for n in ["INT_L_X10Y10/NN6BEG0", "INT_L_X10Y10/EE2END1", "INT_L_X10Y10/LV0", "INT_L_X10Y10/LH12", "INT_L_X10Y10/LOGIC_OUTS_L4"] {
            assert!(is_fabric_wire(n), "{n}");
        }
        // The clock network, and the inputs a clock may still enter by.
        for n in [
            "INT_R_X19Y100/GCLK_B0_EAST",
            "INT_R_X19Y100/CLK0",
            "INT_L_X10Y10/CLK_L1",
            "HCLK_L_X10Y26/HCLK_CK_BUFHCLK3",
            "CLK_HROW_TOP_R_X87Y78/CLK_HROW_CK_MUX_OUT_L0",
            "INT_R_X19Y100/GFAN1",
            "INT_R_X19Y100/IMUX5",
        ] {
            assert!(!is_fabric_wire(n), "{n}");
        }
    }
}
