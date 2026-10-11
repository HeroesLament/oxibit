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

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashSet};
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
}

impl Default for Opts {
    fn default() -> Self {
        Opts { max_iters: 200, pres_fac: 0.5, pres_growth: 1.5, hist_fac: 1.0, astar: 0.6 }
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
    /// control pin tied in the tile), which nextpnr uses as such.
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
        for h in hops {
            if net.sinks.binary_search(&h.dst).is_err() {
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

    /// Route one net. Its own previous occupancy must already be removed.
    fn route_net(&mut self, net: &Net, pres: f32, pops: &mut u64) -> (Vec<Hop>, usize) {
        self.cur_tag += 1;
        let tag = self.cur_tag;
        let mut tree: Vec<u32> = vec![net.source];
        self.tag[net.source as usize] = tag;
        let mut hops = Vec::new();
        let mut failed = 0;

        // Nearest sinks first.
        let mut sinks = net.sinks.clone();
        sinks.sort_by(|&a, &b| self.manhattan(net.source, a).total_cmp(&self.manhattan(net.source, b)));

        for &target in &sinks {
            if self.tag[target as usize] == tag {
                continue; // already on the tree
            }
            let mut heap = BinaryHeap::new();
            for &t in &tree {
                self.cost[t as usize] = 0.0;
                self.prev[t as usize] = (NONE, 0, 0);
                self.touched.push(t);
                heap.push(Item { f: self.opts.astar * self.manhattan(t, target), node: t });
            }
            let mut found = false;
            while let Some(Item { f: _, node }) = heap.pop() {
                *pops += 1;
                if node == target {
                    found = true;
                    break;
                }
                let base = self.cost[node as usize];
                for &(d, tile, pi) in self.g.out_edges(node) {
                    match self.g.edge_kind(tile, pi) {
                        EdgeKind::Feature | EdgeKind::Fixed => {}
                        EdgeKind::Relay | EdgeKind::Default => continue,
                    }
                    if self.reserved[d as usize] && d != target {
                        continue;
                    }
                    let c = base + if self.tag[d as usize] == tag { 0.0 } else { self.node_cost(d, pres) };
                    if c < self.cost[d as usize] {
                        if self.cost[d as usize].is_infinite() {
                            self.touched.push(d);
                        }
                        self.cost[d as usize] = c;
                        self.prev[d as usize] = (node, tile, pi);
                        heap.push(Item { f: c + self.opts.astar * self.manhattan(d, target), node: d });
                    }
                }
            }
            if found {
                let mut n = target;
                while self.prev[n as usize].0 != NONE && self.tag[n as usize] != tag {
                    let (from, tile, pi) = self.prev[n as usize];
                    hops.push(Hop { src: from, dst: n, tile, pi });
                    self.tag[n as usize] = tag;
                    tree.push(n);
                    n = from;
                }
                self.tag[target as usize] = tag;
            } else {
                failed += 1;
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
        let t0 = Instant::now();
        let mut stats = Stats::default();
        let routed: Vec<bool> = nets.iter().map(|n| matches!(n.class, NetClass::Signal | NetClass::Constant)).collect();

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
        let mut todo: Vec<usize> = (0..nets.len()).filter(|&i| routed[i]).collect();
        stats.routed_nets = todo.len();
        for iter in 1..=self.opts.max_iters {
            let ti = Instant::now();
            let mut pops = 0u64;
            let mut failed = 0;
            for &i in &todo {
                if progress.cancelled() {
                    stats.cancelled = true;
                    break;
                }
                for &n in &prints[i] {
                    self.occ[n as usize] -= 1;
                }
                let (hops, f) = self.route_net(&nets[i], pres, &mut pops);
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
            progress.emit(Event::progress(
                "route",
                json!({
                    "iter": iter,
                    "routed": todo.len(),
                    "overused": over,
                    "unroutable": failed,
                    "pops": pops,
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
