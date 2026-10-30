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

use oxibit_fasm::route::{EdgeKind, Graph};
use oxibit_report::{Event, Progress, Silent};
use serde_json::json;

use crate::nets::{Net, NetClass};

#[derive(Debug, Clone)]
pub struct Opts {
    pub max_iters: u32,
    /// Present-congestion factor at iteration 1, and its growth per iteration.
    pub pres_fac: f32,
    pub pres_growth: f32,
    /// History added per unit of overuse per iteration.
    pub hist_fac: f32,
    /// A* weight on the Manhattan distance (cost units per tile). Above the
    /// admissible bound on purpose: long wires make the true bound tiny.
    pub astar: f32,
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
}

impl Default for Opts {
    fn default() -> Self {
        Opts { max_iters: 200, pres_fac: 0.5, pres_growth: 1.5, hist_fac: 1.0, astar: 0.6, delay_weight: 0.0, delay_norm: 100.0, astar_delay: 0.3, bbox_margin: -1 }
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
    g: &'g Graph,
    opts: Opts,
    /// Nets using each node (counted once per net).
    occ: Vec<u16>,
    hist: Vec<f32>,
    /// Nodes no routed net may enter (clock and default nets, kept as is).
    reserved: Vec<bool>,
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
}

const NONE: u32 = u32::MAX;

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
            hist: vec![0.0; n],
            reserved: vec![false; n],
            cost: vec![f32::INFINITY; n],
            prev: vec![(NONE, 0, 0); n],
            touched: Vec::new(),
            tag: vec![0; n],
            cur_tag: 0,
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
    fn footprint(&self, net: &Net, hops: &[Hop]) -> Vec<u32> {
        let mut out = Vec::new();
        let mut seen = HashSet::new();
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
            if ok && !path.is_empty() && n == net.source {
                for k in path {
                    keep[k] = true;
                }
            }
        }
        hops.iter().zip(&keep).filter(|(_, k)| **k).map(|(h, _)| *h).collect()
    }

    /// Route one net, starting from the hops in `keep` (a subtree from the
    /// source; empty to route from scratch). Its own previous occupancy
    /// must already be removed.
    fn route_net(&mut self, net: &Net, pres: f32, pops: &mut u64, outside: &mut u64, keep: &[Hop], margin: i32) -> (Vec<Hop>, usize) {
        self.cur_tag += 1;
        let tag = self.cur_tag;
        let mut tree: Vec<u32> = vec![net.source];
        self.tag[net.source as usize] = tag;
        // Delay (ps) from the source to each tree node.
        let mut arrival: std::collections::HashMap<u32, f32> = std::collections::HashMap::from([(net.source, 0.0)]);
        let mut hops: Vec<Hop> = keep.to_vec();
        let mut failed = 0;
        // The kept subtree, its arrivals in order out from the source.
        if !keep.is_empty() {
            let mut out: HashMap<u32, Vec<usize>> = HashMap::new();
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
            let aim = set[0];
            let w = self.opts.delay_weight;
            let astar = (1.0 - w) * self.opts.astar + w * self.opts.astar_delay;
            let mut reached = None;
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
                for &t in &tree {
                    let start = w * arrival.get(&t).copied().unwrap_or(0.0) / self.opts.delay_norm;
                    self.cost[t as usize] = start;
                    self.prev[t as usize] = (NONE, 0, 0);
                    self.touched.push(t);
                    heap.push(Item { f: start + astar * self.manhattan(t, aim), node: t });
                }
                while let Some(Item { f: _, node }) = heap.pop() {
                    *pops += 1;
                    if set.contains(&node) {
                        reached = Some(node);
                        break;
                    }
                    let base = self.cost[node as usize];
                    for &(d, tile, pi) in self.g.out_edges(node) {
                        match self.g.edge_kind(tile, pi) {
                            EdgeKind::Feature | EdgeKind::Fixed => {}
                            EdgeKind::Relay if self.g.edge_site_through(tile, pi) => {}
                            EdgeKind::Relay if net.class == NetClass::GlobalClock && self.g.edge_pip(tile, pi).tile.starts_with("CLK_") => {}
                            // A constant net's VCC tie reads into an interconnect
                            // input through its default (no line).
                            EdgeKind::Default if net.class == NetClass::Constant && node == net.source => {}
                            EdgeKind::Relay | EdgeKind::Default => continue,
                        }
                        if self.reserved[d as usize] && !set.contains(&d) {
                            continue;
                        }
                        if !set.contains(&d) && !inside(d) {
                            continue;
                        }
                        let step = if self.tag[d as usize] == tag {
                            0.0
                        } else if w > 0.0 {
                            (1.0 - w) * self.node_cost(d, pres) + w * self.g.edge_timing(tile, pi).delay as f32 / self.opts.delay_norm
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
                            heap.push(Item { f: c + astar * self.manhattan(d, aim), node: d });
                        }
                    }
                }
                if reached.is_some() || boxed.is_none() {
                    break;
                }
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
                    // The new branch's arrivals, from where it left the tree.
                    for h in hops[first..].iter().rev() {
                        let a = arrival.get(&h.src).copied().unwrap_or(0.0) + self.g.edge_timing(h.tile, h.pi).delay as f32;
                        arrival.insert(h.dst, a);
                    }
                }
                None => failed += 1,
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
    pub fn run_observed(&mut self, nets: &[Net], progress: &dyn Progress, observe: &mut dyn FnMut(u32, &[u16])) -> (Vec<Vec<Hop>>, Stats) {
        let t0 = Instant::now();
        let mut stats = Stats::default();
        let routed: Vec<bool> = nets.iter().map(|n| matches!(n.class, NetClass::Signal | NetClass::Constant | NetClass::GlobalClock)).collect();

        // Kept nets reserve their nodes; routed nets own their terminals.
        for (net, &r) in nets.iter().zip(&routed) {
            if !r {
                let hops: Vec<Hop> = net.edges.iter().map(|e| Hop { src: e.src, dst: e.dst, tile: e.tile, pi: e.pi }).collect();
                for n in self.footprint(net, &hops) {
                    self.reserved[n as usize] = true;
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

        let mut pres = self.opts.pres_fac;
        let mut margin: Vec<i32> = vec![self.opts.bbox_margin; nets.len()];
        let mut todo: Vec<usize> = (0..nets.len()).filter(|&i| routed[i]).collect();
        stats.routed_nets = todo.len();
        for iter in 1..=self.opts.max_iters {
            let ti = Instant::now();
            let mut pops = 0u64;
            let mut failed = 0;
            let mut kept = 0usize;
            let mut outside = 0u64;
            // (pops, net) of the costliest nets this iteration.
            let mut costly: Vec<(u64, usize)> = Vec::new();
            for &i in &todo {
                if progress.cancelled() {
                    stats.cancelled = true;
                    break;
                }
                // Keep the branches that share no node with another net.
                let bad: HashSet<u32> = prints[i].iter().copied().filter(|&n| self.occ[n as usize] > 1).collect();
                let keep = if iter == 1 { Vec::new() } else { self.keep_hops(&nets[i], &routes[i], &bad) };
                kept += keep.len();
                for &n in &prints[i] {
                    self.occ[n as usize] -= 1;
                }
                let before = pops;
                let (hops, f) = self.route_net(&nets[i], pres, &mut pops, &mut outside, &keep, margin[i]);
                costly.push((pops - before, i));
                if margin[i] >= 0 {
                    margin[i] = margin[i].saturating_mul(2).max(1);
                }
                failed += f;
                prints[i] = self.footprint(&nets[i], &hops);
                for &n in &prints[i] {
                    self.occ[n as usize] += 1;
                }
                routes[i] = hops;
            }
            stats.heap_pops += pops;
            stats.unroutable_sinks = failed;

            // Overuse and history.
            let mut over = 0usize;
            for (n, &o) in self.occ.iter().enumerate() {
                if o > 1 {
                    over += 1;
                    self.hist[n] += self.opts.hist_fac * (o - 1) as f32;
                }
            }
            stats.iterations = iter;
            stats.overused = over;
            observe(iter, &self.occ);
            costly.sort_unstable_by(|a, b| b.cmp(a));
            let costliest: Vec<_> = costly
                .iter()
                .take(5)
                .map(|&(p, i)| {
                    json!({"pops": p, "net": i, "sinks": nets[i].sinks.len(), "groups": nets[i].pin_groups.len(),
                           "source": self.g.node_name(nets[i].source)})
                })
                .collect();
            progress.emit(Event::progress(
                "route",
                json!({
                    "iter": iter,
                    "routed": todo.len(),
                    "overused": over,
                    "unroutable": failed,
                    "pops": pops,
                    "kept_hops": kept,
                    "unboxed_searches": outside,
                    "costliest": costliest,
                    "top100_pops": costly.iter().take(100).map(|c| c.0).sum::<u64>(),
                    "iter_ms": ti.elapsed().as_millis() as u64,
                }),
            ));
            if over == 0 || stats.cancelled {
                break;
            }
            todo = (0..nets.len()).filter(|&i| routed[i] && prints[i].iter().any(|&n| self.occ[n as usize] > 1)).collect();
            pres *= self.opts.pres_growth;
        }
        stats.seconds = t0.elapsed().as_secs_f64();
        (routes, stats)
    }
}
