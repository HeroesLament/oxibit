//! A routing graph of the device, built from the same prjxray database the
//! assembler uses, and a router that emits FASM.
//!
//! This is for test harnesses, not designs: it routes chosen nets through
//! chosen pips so that a test can exercise exactly the routing resources it
//! is meant to document, with no placer or router in between.
//!
//! ## Graph
//!
//! * Wires are tile-local (`tile_type_*.json` "wires").
//! * Nodes are the wires `tileconn.json` joins across tiles (union-find over
//!   every wire pair at every matching tile pair), as prjxray's
//!   `Connections` builds them.
//! * Edges are pips. A pip is usable when it can actually be made to
//!   conduct: its `TILE.DST.SRC` feature has segbits (and is emitted), or it
//!   is an `always` pseudo-pip, or it is a hard connection (`->`, not
//!   pseudo, no segbits: fixed wiring such as CLB BYP -> AX), or it is
//!   one of the I/O site route-throughs in `io_pseudo`. Other `hint` and
//!   `default` pseudo-pips are never used. `<<->>` pass pips are two edges,
//!   one per direction, each usable only if its own feature has segbits.
//!
//! The graph can be built over a window of the grid; nodes leaving the
//! window are cut there, which is fine for local test structures and keeps
//! memory to the region in use.

use std::collections::{HashMap, HashSet, VecDeque};
use std::fs;
use std::sync::{Arc, Mutex, OnceLock};

use serde_json::Value;

use crate::db::Database;

type R<T> = Result<T, String>;

/// A search result: the node reached and the `(node, tile, type pip)` steps
/// that reached it.
type Found = (u32, Vec<(u32, u32, u32)>);
/// A net's source node and each sink with its exclusion set.
type NetEnds = (u32, Vec<(u32, HashSet<u32>)>);
/// tileconn: (tile type, wire) -> [(dx, dy, other tile type, other wire)].
type Potential = HashMap<(String, String), Vec<(i32, i32, String, String)>>;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Window {
    pub x0: i32,
    pub y0: i32,
    pub x1: i32,
    pub y1: i32,
}

impl Window {
    pub fn contains(&self, x: i32, y: i32) -> bool {
        (self.x0..=self.x1).contains(&x) && (self.y0..=self.y1).contains(&y)
    }
}

/// How a usable pip is made to conduct.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Enable {
    /// Emit this FASM feature (`TILE.DST.SRC`).
    Feature(String),
    /// Always connected: an `always` pseudo-pip or a hard wire.
    Fixed,
    /// A `default` pseudo-pip: the mux's output when none of its pips is
    /// set. It conducts only while no other pip into `dst` is configured.
    /// Only in graphs built with `GraphOpts::defaults`.
    Default,
    /// A LUT route-through or output mux in a CLB site, made by these
    /// tile-relative FASM lines (`SITE.ALUT.INIT[63:0] = ...`,
    /// `SITE.AOUTMUX.O6`).
    Relay(Vec<String>),
}

#[derive(Debug, Clone)]
struct TypePip {
    src: u32,
    dst: u32,
    /// The type-level feature key (`TYPE.DST.SRC`) when it has segbits.
    feature: Option<String>,
    /// For a relay (LUT route-through, output mux): its site and the
    /// site-relative lines that make it.
    relay: Option<(String, Vec<String>)>,
    /// A `default` pseudo-pip (see `Enable::Default`).
    default: bool,
    /// An I/O site route-through (`io_pseudo`): a relay a design router may
    /// take, since only the I/O pin it serves can reach it.
    through: bool,
}

struct TypeData {
    wires: Vec<String>,
    index: HashMap<String, u32>,
    pips: Vec<TypePip>,
    /// CLB site pins: (site, pin, wire index).
    site_pins: Vec<(String, String, u32)>,
}

#[derive(Debug, Clone)]
pub struct Pip {
    pub tile: String,
    pub src: String,
    pub dst: String,
    pub enable: Enable,
}

impl Pip {
    /// `TILE.DST.SRC`, whatever enables it.
    pub fn name(&self) -> String {
        format!("{}.{}.{}", self.tile, self.dst, self.src)
    }

    /// The pip's own FASM line; `None` for fixed and relay edges.
    pub fn fasm(&self) -> Option<String> {
        match &self.enable {
            Enable::Feature(_) => Some(format!("{}.{}.{}", self.tile, self.dst, self.src)),
            Enable::Fixed | Enable::Default | Enable::Relay(_) => None,
        }
    }

    /// Every FASM line that makes this edge conduct.
    pub fn fasm_lines(&self) -> Vec<String> {
        match &self.enable {
            Enable::Feature(_) => vec![format!("{}.{}.{}", self.tile, self.dst, self.src)],
            Enable::Fixed | Enable::Default => vec![],
            Enable::Relay(lines) => lines.iter().map(|l| format!("{}.{l}", self.tile)).collect(),
        }
    }
}

pub struct Graph {
    tiles: Vec<(String, String)>, // (instance, type)
    tile_index: HashMap<String, u32>,
    wire_base: Vec<u32>, // global wire id of each tile's wire 0
    node_of: Vec<u32>,   // global wire -> node
    /// node -> [(dst node, tile, type pip index)]
    edges: Vec<Vec<(u32, u32, u32)>>,
    types: HashMap<String, TypeData>,
    coords: Vec<(i32, i32)>,
    /// node -> grid coordinates of one of its tiles (A* heuristic).
    node_xy: Vec<(i16, i16)>,
    /// Reverse adjacency, built on first use.
    rev: OnceLock<Vec<Vec<u32>>>,
    /// Reverse adjacency over fixed edges only, built on first use.
    rev_fixed: OnceLock<HashMap<u32, Vec<u32>>>,
    /// sink node -> bitset of nodes that can reach it.
    reach: Mutex<HashMap<u32, Arc<Vec<u64>>>>,
    opts: GraphOpts,
}

/// A dense node set (one bit per node): what the packer tests on every
/// edge it looks at.
#[derive(Clone)]
pub struct NodeSet {
    bits: Vec<u64>,
}

impl NodeSet {
    pub fn new(nodes: usize) -> Self {
        NodeSet { bits: vec![0; nodes.div_ceil(64)] }
    }
    pub fn contains(&self, n: &u32) -> bool {
        self.bits[(*n / 64) as usize] >> (*n % 64) & 1 == 1
    }
    pub fn insert(&mut self, n: u32) {
        self.bits[(n / 64) as usize] |= 1 << (n % 64);
    }
    pub fn remove(&mut self, n: &u32) {
        self.bits[(*n / 64) as usize] &= !(1 << (*n % 64));
    }
    pub fn count(&self) -> u64 {
        self.bits.iter().map(|w| w.count_ones() as u64).sum()
    }
    pub fn extend(&mut self, it: impl IntoIterator<Item = u32>) {
        for n in it {
            self.insert(n);
        }
    }
}

// Reusable search state (dense, stamped per search), one per thread so
// searches on different threads never wait on each other. Sized to the
// graph on first use; a graph of another size resizes it.
thread_local! {
    static SCRATCH: std::cell::RefCell<Scratch> = std::cell::RefCell::new(Scratch::default());
}

#[derive(Default)]
struct Scratch {
    epoch: u32,
    stamp: Vec<u32>,
    prev: Vec<(u32, u32, u32)>,
    depth: Vec<u16>,
    queue: VecDeque<u32>,
}

/// What `Graph::build_with` puts in the graph beyond the defaults.
#[derive(Debug, Clone, Copy, Default)]
pub struct GraphOpts {
    /// Add `default` pseudo-pips as edges (`Enable::Default`): the constant
    /// an unset interconnect input mux reads. A design router needs them for
    /// tie-offs; test-chain routing must not use them.
    pub defaults: bool,
}

impl Graph {
    pub fn build(db: &Database, window: Option<Window>) -> R<Self> {
        Self::build_with(db, window, GraphOpts::default())
    }

    pub fn build_with(db: &Database, window: Option<Window>, opts: GraphOpts) -> R<Self> {
        let grid: Value = read_json(&db.root.join(&db.fabric).join("tilegrid.json"))?;
        let tileconn: Value = read_json(&db.root.join(&db.fabric).join("tileconn.json"))?;
        let grid = grid.as_object().ok_or("tilegrid is not an object")?;

        // Tiles in the window, with coordinates.
        let mut tiles = Vec::new();
        let mut at: HashMap<(i32, i32), u32> = HashMap::new();
        for (name, t) in grid {
            let (x, y) = (t["grid_x"].as_i64().unwrap_or(-1) as i32, t["grid_y"].as_i64().unwrap_or(-1) as i32);
            if window.is_some_and(|w| !w.contains(x, y)) {
                continue;
            }
            at.insert((x, y), tiles.len() as u32);
            tiles.push((name.clone(), t["type"].as_str().unwrap_or("").to_string(), x, y));
        }

        // Tile types in use.
        let mut types: HashMap<String, TypeData> = HashMap::new();
        for (_, ty, _, _) in &tiles {
            if !types.contains_key(ty) {
                types.insert(ty.clone(), load_type(db, ty)?);
            }
        }

        // Global wire ids.
        let mut wire_base = Vec::with_capacity(tiles.len());
        let mut total = 0u32;
        for (_, ty, _, _) in &tiles {
            wire_base.push(total);
            total += types[ty].wires.len() as u32;
        }

        // Nodes: union-find over tileconn.
        let mut uf = UnionFind::new(total as usize);
        let mut potential: Potential = HashMap::new();
        for conn in tileconn.as_array().ok_or("tileconn is not a list")? {
            let d = &conn["grid_deltas"];
            let (dx, dy) = (d[0].as_i64().unwrap_or(0) as i32, d[1].as_i64().unwrap_or(0) as i32);
            let tt = &conn["tile_types"];
            let (t0, t1) = (tt[0].as_str().unwrap_or(""), tt[1].as_str().unwrap_or(""));
            for pair in conn["wire_pairs"].as_array().into_iter().flatten() {
                let (w0, w1) = (pair[0].as_str().unwrap_or(""), pair[1].as_str().unwrap_or(""));
                potential.entry((t0.to_string(), w0.to_string())).or_default().push((dx, dy, t1.to_string(), w1.to_string()));
            }
        }
        for (ti, (_, ty, x, y)) in tiles.iter().enumerate() {
            let td = &types[ty];
            for (wi, w) in td.wires.iter().enumerate() {
                let Some(targets) = potential.get(&(ty.clone(), w.clone())) else { continue };
                for (dx, dy, tty, tw) in targets {
                    let Some(&tj) = at.get(&(x + dx, y + dy)) else { continue };
                    let (_, jty, _, _) = &tiles[tj as usize];
                    if jty != tty {
                        continue;
                    }
                    if let Some(&wj) = types[jty].index.get(tw) {
                        uf.union(wire_base[ti] + wi as u32, wire_base[tj as usize] + wj);
                    }
                }
            }
        }
        let (node_of, nodes) = uf.dense();

        // Edges.
        let mut edges: Vec<Vec<(u32, u32, u32)>> = vec![Vec::new(); nodes];
        for (ti, (_, ty, _, _)) in tiles.iter().enumerate() {
            for (pi, p) in types[ty].pips.iter().enumerate() {
                if p.default && !opts.defaults {
                    continue;
                }
                let s = node_of[(wire_base[ti] + p.src) as usize];
                let d = node_of[(wire_base[ti] + p.dst) as usize];
                if s != d {
                    edges[s as usize].push((d, ti as u32, pi as u32));
                }
            }
        }

        let tile_index = tiles.iter().enumerate().map(|(i, (n, _, _, _))| (n.clone(), i as u32)).collect();
        let coords: Vec<(i32, i32)> = tiles.iter().map(|&(_, _, x, y)| (x, y)).collect();
        let mut node_xy = vec![(0i16, 0i16); nodes];
        for (ti, base) in wire_base.iter().enumerate() {
            let n_wires = types[&tiles[ti].1].wires.len() as u32;
            for wi in 0..n_wires {
                let n = node_of[(base + wi) as usize] as usize;
                node_xy[n] = (coords[ti].0 as i16, coords[ti].1 as i16);
            }
        }
        Ok(Graph {
            coords,
            node_xy,
            rev: OnceLock::new(),
            rev_fixed: OnceLock::new(),
            reach: Mutex::new(HashMap::new()),
            opts,
            tiles: tiles.into_iter().map(|(n, t, _, _)| (n, t)).collect(),
            tile_index,
            wire_base,
            node_of,
            edges,
            types,
        })
    }

    /// Tile type of an instance in the graph.
    pub fn tile_type(&self, tile: &str) -> Option<&str> {
        self.tile_index.get(tile).map(|&i| self.tiles[i as usize].1.as_str())
    }

    pub fn nodes(&self) -> usize {
        self.edges.len()
    }

    pub fn edges(&self) -> usize {
        self.edges.iter().map(Vec::len).sum()
    }

    /// The node of `TILE/WIRE`.
    pub fn node(&self, tile_wire: &str) -> R<u32> {
        let (tile, wire) = tile_wire.split_once('/').ok_or_else(|| format!("expected TILE/WIRE, got {tile_wire}"))?;
        let &ti = self.tile_index.get(tile).ok_or_else(|| format!("tile {tile} not in the graph (window?)"))?;
        let ty = &self.tiles[ti as usize].1;
        let &wi = self.types[ty].index.get(wire).ok_or_else(|| format!("{ty} has no wire {wire}"))?;
        Ok(self.node_of[(self.wire_base[ti as usize] + wi) as usize])
    }

    fn pip(&self, tile: u32, pi: u32) -> Pip {
        let (name, ty) = &self.tiles[tile as usize];
        let td = &self.types[ty];
        let p = &td.pips[pi as usize];
        Pip {
            tile: name.clone(),
            src: td.wires[p.src as usize].clone(),
            dst: td.wires[p.dst as usize].clone(),
            enable: match (&p.feature, &p.relay) {
                (Some(f), _) => Enable::Feature(f.clone()),
                (None, Some((_, lines))) => Enable::Relay(lines.clone()),
                (None, None) if p.default => Enable::Default,
                (None, None) => Enable::Fixed,
            },
        }
    }

    /// Nodes a FASM file's pips occupy (both ends of every emitted pip),
    /// so a router can keep clear of an existing design.
    pub fn occupied(&self, fasm: &str) -> HashSet<u32> {
        let mut used = HashSet::new();
        for line in fasm.lines() {
            let f = line.split('#').next().unwrap_or("").trim();
            let parts: Vec<&str> = f.split('.').collect();
            if parts.len() != 3 || f.contains('=') || f.contains('[') {
                continue;
            }
            let (tile, dst, src) = (parts[0], parts[1], parts[2]);
            for w in [dst, src] {
                if let Ok(n) = self.node(&format!("{tile}/{w}")) {
                    used.insert(n);
                }
            }
        }
        self.exclusion(used)
    }

    /// Fewest-pips route from `src` to `dst` avoiding `blocked` nodes
    /// (neither end may be blocked). Returns the pips in order.
    pub fn route(&self, src: u32, dst: u32, blocked: &HashSet<u32>) -> Option<Vec<Pip>> {
        let mut prev: HashMap<u32, (u32, u32, u32)> = HashMap::new();
        let mut seen = HashSet::from([src]);
        let mut q = VecDeque::from([src]);
        while let Some(n) = q.pop_front() {
            if n == dst {
                let mut path = Vec::new();
                let mut cur = dst;
                while cur != src {
                    let (p, tile, pi) = prev[&cur];
                    path.push(self.pip(tile, pi));
                    cur = p;
                }
                path.reverse();
                return Some(path);
            }
            for &(d, tile, pi) in &self.edges[n as usize] {
                if seen.contains(&d) || (blocked.contains(&d) && d != dst) {
                    continue;
                }
                seen.insert(d);
                prev.insert(d, (n, tile, pi));
                q.push_back(d);
            }
        }
        None
    }

    /// Route `src` -> through each of `via` (pips named `TILE.DST.SRC`, taken
    /// in order) -> `dst`, never reusing a node. The result conducts only if
    /// every listed pip does: the point of a test chain.
    pub fn route_via(&self, src: &str, via: &[String], dst: &str, blocked: &HashSet<u32>) -> R<Vec<Pip>> {
        let mut blocked = blocked.clone();
        let mut cur = self.node(src)?;
        let mut out = Vec::new();
        blocked.insert(cur);

        for v in via {
            let parts: Vec<&str> = v.split('.').collect();
            let [tile, pdst, psrc] = parts[..] else { return Err(format!("via pip must be TILE.DST.SRC: {v}")) };
            let (a, b) = (self.node(&format!("{tile}/{psrc}"))?, self.node(&format!("{tile}/{pdst}"))?);
            let leg = if cur == a { vec![] } else { self.route(cur, a, &blocked).ok_or_else(|| format!("no route to {tile}/{psrc}"))? };
            let &(_, ti, pi) = self.edges[a as usize]
                .iter()
                .find(|&&(d, t, p)| d == b && self.tiles[t as usize].0 == tile && {
                    let pp = self.pip(t, p);
                    pp.src == psrc && pp.dst == pdst
                })
                .ok_or_else(|| format!("{v} is not a usable pip"))?;
            for p in &leg {
                blocked.insert(self.node(&format!("{}/{}", p.tile, p.dst))?);
            }
            blocked.insert(a);
            blocked.insert(b);
            out.extend(leg);
            out.push(self.pip(ti, pi));
            cur = b;
        }

        let end = self.node(dst)?;
        let leg = self.route(cur, end, &blocked).ok_or_else(|| format!("no route to {dst}"))?;
        out.extend(leg);
        Ok(out)
    }

    /// The graph's view of one pip line `TILE.DST.SRC`.
    pub fn lookup_pip(&self, line: &str) -> PipLookup {
        let parts: Vec<&str> = line.split('.').collect();
        let [tile, dst, src] = parts[..] else { return PipLookup::Malformed };
        let Some(&ti) = self.tile_index.get(tile) else { return PipLookup::NoTile };
        let td = &self.types[&self.tiles[ti as usize].1];
        let (Some(&ws), Some(&wd)) = (td.index.get(src), td.index.get(dst)) else { return PipLookup::NoWire };
        let usable = |p: &TypePip| !p.default || self.opts.defaults;
        let Some(pi) = td.pips.iter().position(|p| p.src == ws && p.dst == wd && usable(p)) else { return PipLookup::NoPip };
        let a = self.node_of[(self.wire_base[ti as usize] + ws) as usize];
        let b = self.node_of[(self.wire_base[ti as usize] + wd) as usize];
        if a == b {
            return PipLookup::SameNode(a);
        }
        let pip = self.pip(ti, pi as u32);
        PipLookup::Edge { src: a, dst: b, tile: ti, pi: pi as u32, pip }
    }

    /// Out-edges of a node: `(dst node, tile, type pip index)`.
    pub fn out_edges(&self, n: u32) -> &[(u32, u32, u32)] {
        &self.edges[n as usize]
    }

    /// How an edge conducts, without building its names.
    pub fn edge_kind(&self, tile: u32, pi: u32) -> EdgeKind {
        let p = &self.types[&self.tiles[tile as usize].1].pips[pi as usize];
        match (&p.feature, &p.relay) {
            (Some(_), _) => EdgeKind::Feature,
            (None, Some(_)) => EdgeKind::Relay,
            (None, None) if p.default => EdgeKind::Default,
            (None, None) => EdgeKind::Fixed,
        }
    }

    /// Whether an edge is an I/O site route-through (see `io_pseudo`).
    pub fn edge_site_through(&self, tile: u32, pi: u32) -> bool {
        self.types[&self.tiles[tile as usize].1].pips[pi as usize].through
    }

    /// The pip behind an edge.
    pub fn edge_pip(&self, tile: u32, pi: u32) -> Pip {
        self.pip(tile, pi)
    }

    /// Grid coordinates of one of the node's tiles.
    pub fn node_xy(&self, n: u32) -> (i16, i16) {
        self.node_xy[n as usize]
    }

    /// `TILE/WIRE` names of a node's first wire (for reports).
    pub fn node_name(&self, n: u32) -> String {
        // Linear scan; reports only.
        for (ti, base) in self.wire_base.iter().enumerate() {
            let td = &self.types[&self.tiles[ti].1];
            for wi in 0..td.wires.len() as u32 {
                if self.node_of[(base + wi) as usize] == n {
                    return format!("{}/{}", self.tiles[ti].0, td.wires[wi as usize]);
                }
            }
        }
        format!("node{n}")
    }
}

/// How an edge conducts (`Enable` without the names).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EdgeKind {
    Feature,
    Fixed,
    Relay,
    Default,
}

/// What `Graph::lookup_pip` found for a pip line.
#[derive(Debug, Clone)]
pub enum PipLookup {
    /// A usable edge between two distinct nodes.
    Edge { src: u32, dst: u32, tile: u32, pi: u32, pip: Pip },
    /// Both wires are the same node, so the pip is not an edge.
    SameNode(u32),
    /// The tile type has both wires but no such pip (or it is not usable).
    NoPip,
    NoWire,
    NoTile,
    Malformed,
}


/// The pip lines (`TILE.DST.SRC`) of a FASM text.
pub fn pip_lines(fasm: &str) -> HashSet<String> {
    fasm.lines()
        .map(|l| l.split('#').next().unwrap_or("").trim())
        .filter(|f| f.split('.').count() == 3 && !f.contains('=') && !f.contains('['))
        .map(String::from)
        .collect()
}

/// One target pip resolved against the graph.
#[derive(Debug, Clone)]
struct Target {
    name: String,
    a: u32,
    b: u32,
    tile: u32,
    pi: u32,
}

/// Search limits for `pack`.
#[derive(Debug, Clone)]
pub struct PackOpts {
    /// Targets threaded per chain at most.
    pub max_via: usize,
    /// The next target is looked for this many pips away at most.
    pub target_hops: u16,
    /// Remember where target searches came up empty (see `pack`).
    pub dead_regions: bool,
}

/// Work counters for one `pack` call: where the time goes.
#[derive(Debug, Clone, Default)]
pub struct PackStats {
    /// Searches for the next target, those that found none, nodes dequeued.
    pub target_searches: u64,
    pub target_misses: u64,
    /// Nodes target searches reached but did not expand because they lie in
    /// a dead region, and nodes in dead regions at the end.
    pub target_pruned: u64,
    pub dead_nodes: u64,
    pub target_visited: u64,
    /// Nodes dequeued by searches that found a target: total and largest.
    pub target_hit_visited: u64,
    pub target_hit_max: u64,
    /// Searches to the capture sink, those that failed, nodes popped.
    pub finish_searches: u64,
    pub finish_misses: u64,
    pub finish_visited: u64,
    pub rollbacks: u64,
    /// Wall time per chain, in `ends` order.
    pub chain_ms: Vec<u64>,
}

/// Per-pack lookups the target search consults.
struct Lookup {
    /// Source node of some pool target.
    is_src: NodeSet,
    /// Dead regions: nodes from which an earlier target search, run to
    /// completion, reached no usable target. Blocking only grows and the
    /// pool only shrinks during a pack, so searching from there again would
    /// miss again, except that the region was found under another chain's
    /// own path and sink: a heuristic, measured by targets placed.
    dead: NodeSet,
}

/// One chain attempt: the chain, the nodes it owns, and the pool indices of
/// the targets it threads.
type Attempt = R<(Chain, Vec<u32>, Vec<usize>)>;

/// A packed test chain: launch -> targets -> capture.
#[derive(Debug, Clone, Default)]
pub struct Chain {
    /// Every pip in order (feature and fixed).
    pub pips: Vec<Pip>,
    /// The targets this chain threads, in order.
    pub targets: Vec<String>,
}

impl Graph {
    /// Every node reachable from `seeds` over fixed edges (hard wires and
    /// `always` pseudo-pips), seeds included: the nodes a driven node drives
    /// with no configuration at all.
    pub fn fixed_closure(&self, seeds: impl IntoIterator<Item = u32>) -> HashSet<u32> {
        let mut out: HashSet<u32> = HashSet::new();
        let mut stack: Vec<u32> = seeds.into_iter().collect();
        while let Some(n) = stack.pop() {
            if !out.insert(n) {
                continue;
            }
            for &(d, tile, pi) in &self.edges[n as usize] {
                if self.is_fixed(tile, pi) && !out.contains(&d) {
                    stack.push(d);
                }
            }
        }
        out
    }

    /// The fixed (always-on) pips around a site pin, walked away from the
    /// site: forward from an output pin, backward from an input pin, until
    /// the walk leaves the fixed region (a pip with bits, or a relay). For a
    /// hard block such as the PS7 these are the pseudo-pips between the pin
    /// and the first switchbox, which a working design through that pin
    /// exercises whatever route it took beyond them.
    pub fn fixed_pips(&self, wire: &str, backward: bool) -> R<Vec<String>> {
        let start = self.node(wire)?;
        let mut seen: HashSet<u32> = HashSet::from([start]);
        let mut stack = vec![start];
        let mut out = Vec::new();
        let mut preds: HashMap<u32, Vec<(u32, u32, u32)>> = HashMap::new();
        if backward {
            for (s, es) in self.edges.iter().enumerate() {
                for &(d, tile, pi) in es {
                    if self.is_fixed(tile, pi) {
                        preds.entry(d).or_default().push((s as u32, tile, pi));
                    }
                }
            }
        }
        while let Some(n) = stack.pop() {
            let next: Vec<(u32, u32, u32)> = if backward {
                preds.get(&n).cloned().unwrap_or_default()
            } else {
                self.edges[n as usize].iter().filter(|&&(_, t, pi)| self.is_fixed(t, pi)).copied().collect()
            };
            for (m, tile, pi) in next {
                out.push(self.pip_name(tile, pi));
                if seen.insert(m) {
                    stack.push(m);
                }
            }
        }
        out.sort();
        out.dedup();
        Ok(out)
    }

    /// `nodes` plus every node whose fixed fan-out reaches one of them: the
    /// nodes a new route may not touch if `nodes` are in use, since driving
    /// any of them would also drive a node in use.
    pub fn exclusion(&self, nodes: impl IntoIterator<Item = u32>) -> HashSet<u32> {
        let rf = self.rev_fixed.get_or_init(|| {
            let mut m: HashMap<u32, Vec<u32>> = HashMap::new();
            for (s, es) in self.edges.iter().enumerate() {
                for &(d, tile, pi) in es {
                    if self.is_fixed(tile, pi) {
                        m.entry(d).or_default().push(s as u32);
                    }
                }
            }
            m
        });
        let mut out: HashSet<u32> = HashSet::new();
        let mut stack: Vec<u32> = self.fixed_closure(nodes).into_iter().collect();
        while let Some(n) = stack.pop() {
            if out.insert(n) {
                stack.extend(rf.get(&n).into_iter().flatten().copied().filter(|p| !out.contains(p)));
            }
        }
        out
    }

    fn is_fixed(&self, tile: u32, pi: u32) -> bool {
        let p = &self.types[&self.tiles[tile as usize].1].pips[pi as usize];
        p.feature.is_none() && p.relay.is_none() && !p.default
    }

    /// A relay edge the design has configured: every line that makes it
    /// (`BUFHCE.BUFHCE_X0Y0.IN_USE`, ...) is in `on`.
    fn relay_on(&self, tile: u32, pi: u32, on: &HashSet<String>) -> bool {
        let (name, ty) = &self.tiles[tile as usize];
        match &self.types[ty].pips[pi as usize].relay {
            Some((_, lines)) => !lines.is_empty() && lines.iter().all(|l| on.contains(&format!("{name}.{l}"))),
            None => false,
        }
    }

    fn pip_name(&self, tile: u32, pi: u32) -> String {
        let (name, ty) = &self.tiles[tile as usize];
        let td = &self.types[ty];
        let p = &td.pips[pi as usize];
        format!("{name}.{}.{}", td.wires[p.dst as usize], td.wires[p.src as usize])
    }

    /// The net a design drives from `src`: every node reachable over fixed
    /// edges and over pips whose line is in `on`; with `relays`, also over
    /// relays whose lines all are in `on` (a clock buffer the design
    /// configures). A data net's trace must not cross relays: it would run
    /// through the FFs the net feeds. Returns the nodes and the
    /// pip lines the net uses.
    pub fn trace(&self, src: u32, on: &HashSet<String>, relays: bool) -> (HashSet<u32>, Vec<String>) {
        let mut nodes = HashSet::from([src]);
        let mut pips = Vec::new();
        let mut stack = vec![src];
        while let Some(n) = stack.pop() {
            for &(d, tile, pi) in &self.edges[n as usize] {
                // Fixed edges always conduct; a design may still write their
                // line (nextpnr writes `always` pseudo-pips), and that line
                // belongs to the net too.
                let name = self.pip_name(tile, pi);
                let listed = on.contains(&name);
                if listed {
                    pips.push(name);
                }
                let through = self.types[&self.tiles[tile as usize].1].pips[pi as usize].through;
                let take = listed || self.is_fixed(tile, pi) || ((relays || through) && self.relay_on(tile, pi, on));
                if take && nodes.insert(d) {
                    stack.push(d);
                }
            }
        }
        (nodes, pips)
    }

    /// Node of a CLB site pin (`CLK`, `AX`, ...).
    pub fn site_pin(&self, tile: &str, site: &str, pin: &str) -> Option<u32> {
        let &ti = self.tile_index.get(tile)?;
        let td = &self.types[&self.tiles[ti as usize].1];
        let &(_, _, w) = td.site_pins.iter().find(|(s, p, _)| s == site && p == pin)?;
        Some(self.node_of[(self.wire_base[ti as usize] + w) as usize])
    }

    /// Tile-local wire name of a site pin (`CLBLM_M_C`).
    pub fn site_pin_wire(&self, tile: &str, site: &str, pin: &str) -> Option<String> {
        let &ti = self.tile_index.get(tile)?;
        let td = &self.types[&self.tiles[ti as usize].1];
        let &(_, _, w) = td.site_pins.iter().find(|(s, p, _)| s == site && p == pin)?;
        Some(td.wires[w as usize].clone())
    }

    /// CLB sites of a tile.
    pub fn sites(&self, tile: &str) -> Vec<String> {
        let Some(&ti) = self.tile_index.get(tile) else { return vec![] };
        let mut s: Vec<String> = self.types[&self.tiles[ti as usize].1].site_pins.iter().map(|(s, _, _)| s.clone()).collect();
        s.dedup();
        s
    }

    /// Grid coordinates of a tile instance.
    pub fn tile_xy(&self, tile: &str) -> Option<(i32, i32)> {
        self.tile_index.get(tile).map(|&i| self.coords[i as usize])
    }

    /// Tile instance names.
    /// Nodes holding a tile wire whose name contains any of `pats`
    /// (`CLK_HROW_CK_INT`), or, for a `TILE/PART` pattern, a wire of that
    /// tile containing PART: what a request asks the router to stay off.
    pub fn nodes_with_wire(&self, pats: &[String]) -> Vec<u32> {
        let mut out = Vec::new();
        if pats.is_empty() {
            return out;
        }
        let pats: Vec<(Option<&str>, &str)> =
            pats.iter().map(|p| match p.split_once('/') {
                Some((t, w)) => (Some(t), w),
                None => (None, p.as_str()),
            }).collect();
        for (t, (name, ty)) in self.tiles.iter().enumerate() {
            let Some(td) = self.types.get(ty) else { continue };
            for (i, w) in td.wires.iter().enumerate() {
                if pats.iter().any(|(pt, pw)| pt.is_none_or(|pt| pt == name) && w.contains(pw)) {
                    out.push(self.node_of[self.wire_base[t] as usize + i]);
                }
            }
        }
        out.sort_unstable();
        out.dedup();
        out
    }

    pub fn tile_names(&self) -> impl Iterator<Item = &str> {
        self.tiles.iter().map(|(n, _)| n.as_str())
    }

    /// Feature pips driving `n` from a node in `from`, through any chain of
    /// fixed edges into `n` (e.g. INT `GCLK -> CLK_L0` for a slice CLK pin).
    pub fn feature_drivers(&self, n: u32, from: &HashSet<u32>) -> Vec<Pip> {
        let rev = self.rev.get_or_init(|| {
            let mut rev = vec![Vec::new(); self.edges.len()];
            for (s, es) in self.edges.iter().enumerate() {
                for &(d, _, _) in es {
                    rev[d as usize].push(s as u32);
                }
            }
            rev
        });
        let mut out = Vec::new();
        let mut stack = vec![n];
        let mut seen = HashSet::from([n]);
        while let Some(c) = stack.pop() {
            for &s in &rev[c as usize] {
                for &(d, tile, pi) in &self.edges[s as usize] {
                    if d != c {
                        continue;
                    }
                    if self.is_fixed(tile, pi) {
                        if seen.insert(s) {
                            stack.push(s);
                        }
                    } else if from.contains(&s) && self.types[&self.tiles[tile as usize].1].pips[pi as usize].feature.is_some() {
                        out.push(self.pip(tile, pi));
                    }
                }
            }
        }
        out
    }

    /// Output nodes and lines of every relay in one site of one tile.
    pub fn site_relays(&self, tile: &str, site: &str) -> Vec<(u32, Vec<String>)> {
        let Some(&ti) = self.tile_index.get(tile) else { return vec![] };
        let td = &self.types[&self.tiles[ti as usize].1];
        td.pips
            .iter()
            .filter_map(|p| match &p.relay {
                Some((s, lines)) if s == site => Some((self.node_of[(self.wire_base[ti as usize] + p.dst) as usize], lines.clone())),
                _ => None,
            })
            .collect()
    }

    /// Output nodes of the relays in one site of one tile: what to block
    /// when a design already uses that site.
    pub fn relay_outputs(&self, tile: &str, site: &str) -> Vec<u32> {
        let Some(&ti) = self.tile_index.get(tile) else { return vec![] };
        let td = &self.types[&self.tiles[ti as usize].1];
        td.pips
            .iter()
            .filter(|p| p.relay.as_ref().is_some_and(|(s, _)| s == site))
            .map(|p| self.node_of[(self.wire_base[ti as usize] + p.dst) as usize])
            .collect()
    }

    /// Bits a route needs for the clock wires it uses, as nextpnr writes them:
    /// CLK_HROW `<wire>_ACTIVE` for GCLK and CK_IN wires, HCLK
    /// `ENABLE_BUFFER.<wire>` for BUFHCLK / BUFRCLK leaves, CLK_BUFG_REBUF
    /// `GCLKn_ENABLE_ABOVE/BELOW` where a route crosses a rebuffer.
    pub fn wire_extras(&self, pips: &[Pip]) -> Vec<String> {
        let mut out = Vec::new();
        for p in pips {
            let Some(ty) = self.tile_type(&p.tile) else { continue };
            for w in [&p.src, &p.dst] {
                if ty.starts_with("CLK_HROW") {
                    if w.starts_with("CLK_HROW_R_CK_GCLK") || (w.starts_with("CLK_HROW_CK_IN_") && !w.contains("HROW_CK_INT")) {
                        out.push(format!("{}.{w}_ACTIVE", p.tile));
                    }
                } else if ty.starts_with("HCLK_L") || ty.starts_with("HCLK_R") {
                    if w.starts_with("HCLK_CK_BUFHCLK") || w.starts_with("HCLK_CK_BUFRCLK") {
                        out.push(format!("{}.ENABLE_BUFFER.{w}", p.tile));
                    }
                } else if ty.starts_with("BRAM_") {
                    // The BRAM address cascade drives up the column only with
                    // the lower tile's output enabled (nextpnr fasm.cc).
                    if w.starts_with("BRAM_CASCOUT_ADDRARDADDR") {
                        out.push(format!("{}.CASCOUT_ARD_ACTIVE", p.tile));
                    } else if w.starts_with("BRAM_CASCOUT_ADDRBWRADDR") {
                        out.push(format!("{}.CASCOUT_BWR_ACTIVE", p.tile));
                    }
                } else if ty.starts_with("HCLK_CMT") {
                    // nextpnr fasm.cc write_clocking: CCIO wires get _ACTIVE
                    // and _USED, regional buffer wires (BUFHCLK, BUFRCLK) _USED.
                    if w.starts_with("HCLK_CMT_CCIO") {
                        out.push(format!("{}.{w}_ACTIVE", p.tile));
                        out.push(format!("{}.{w}_USED", p.tile));
                    } else if w.starts_with("HCLK_CMT_CK_BUFHCLK") || w.starts_with("HCLK_CMT_CK_BUFRCLK") {
                        out.push(format!("{}.{w}_USED", p.tile));
                    }
                } else if ty == "CLK_BUFG_REBUF" {
                    if let Some(rest) = w.strip_prefix("CLK_BUFG_REBUF_R_CK_") {
                        if let Some(g) = rest.split('_').next() {
                            out.push(format!("{}.{g}_ENABLE_ABOVE", p.tile));
                            out.push(format!("{}.{g}_ENABLE_BELOW", p.tile));
                        }
                    }
                }
            }
        }
        out.sort();
        out.dedup();
        out
    }

    /// Every usable pip with bits in tiles of the given types, as
    /// `TILE.DST.SRC`.
    pub fn feature_pips(&self, types: &[String], window: Option<Window>) -> Vec<String> {
        let mut out = Vec::new();
        for (ti, (name, ty)) in self.tiles.iter().enumerate() {
            if !types.iter().any(|t| t == ty) {
                continue;
            }
            let (x, y) = self.coords[ti];
            if window.is_some_and(|w| !w.contains(x, y)) {
                continue;
            }
            let td = &self.types[ty];
            for (pi, p) in td.pips.iter().enumerate() {
                if p.feature.is_some() {
                    let _ = name;
                    out.push(self.pip_name(ti as u32, pi as u32));
                }
            }
        }
        out
    }

    fn resolve(&self, pip: &str) -> R<Target> {
        let parts: Vec<&str> = pip.split('.').collect();
        let [tile, pdst, psrc] = parts[..] else { return Err(format!("pip must be TILE.DST.SRC: {pip}")) };
        let (a, b) = (self.node(&format!("{tile}/{psrc}"))?, self.node(&format!("{tile}/{pdst}"))?);
        let &ti = self.tile_index.get(tile).ok_or_else(|| format!("tile {tile} not in the graph"))?;
        let &(_, _, pi) = self.edges[a as usize]
            .iter()
            .find(|&&(d, t, p)| d == b && t == ti && self.pip_name(t, p) == pip)
            .ok_or_else(|| format!("{pip} is not a usable pip"))?;
        Ok(Target { name: pip.to_string(), a, b, tile: ti, pi })
    }

    /// Breadth-first search from every node in `srcs` (a driven node and
    /// what it drives through fixed edges) avoiding `blocked`; stops at the
    /// first node `goal` accepts and returns it with the pip path from the
    /// source it left.
    ///
    /// Also returns how many nodes it dequeued, and how many of those it did
    /// not expand because they are in `dead`. With `region`, a miss that was
    /// not cut short by `max_depth` leaves there every node it reached: a
    /// region with no goal node reachable from it.
    #[allow(clippy::too_many_arguments)]
    fn search_counted(
        &self,
        srcs: &[u32],
        blocked: &NodeSet,
        allowed: Option<&[bool]>,
        max_depth: Option<u16>,
        dead: Option<&NodeSet>,
        mut region: Option<&mut Vec<u32>>,
        mut goal: impl FnMut(u32) -> bool,
    ) -> (Option<Found>, u64, u64) {
        let mut visited = 0u64;
        let mut pruned = 0u64;
        // Set when the depth limit stopped the search short of something:
        // a miss then proves nothing about the nodes beyond.
        let mut truncated = false;
        if let Some(r) = region.as_deref_mut() {
            r.clear();
        }
        const NONE: u32 = u32::MAX;
        SCRATCH.with_borrow_mut(|sc| {
            if sc.stamp.len() != self.edges.len() {
                sc.stamp = vec![0; self.edges.len()];
                sc.prev = vec![(NONE, 0, 0); self.edges.len()];
                sc.depth = vec![0; self.edges.len()];
                sc.epoch = 0;
            }
            sc.epoch = sc.epoch.wrapping_add(1);
            if sc.epoch == 0 {
                sc.stamp.iter_mut().for_each(|x| *x = 0);
                sc.epoch = 1;
            }
            let epoch = sc.epoch;
            sc.queue.clear();
            for &n in srcs {
                if sc.stamp[n as usize] != epoch {
                    sc.stamp[n as usize] = epoch;
                    sc.prev[n as usize] = (NONE, 0, 0);
                    sc.depth[n as usize] = 0;
                    sc.queue.push_back(n);
                }
            }
            while let Some(n) = sc.queue.pop_front() {
                visited += 1;
                if let Some(r) = region.as_deref_mut() {
                    r.push(n);
                }
                if goal(n) {
                    let mut path = Vec::new();
                    let mut cur = n;
                    while sc.prev[cur as usize].0 != NONE {
                        let (p, tile, pi) = sc.prev[cur as usize];
                        path.push((cur, tile, pi));
                        cur = p;
                    }
                    path.reverse();
                    if let Some(r) = region.as_deref_mut() {
                        r.clear();
                    }
                    return (Some((n, path)), visited, pruned);
                }
                // Known dead: nothing usable is reachable from here.
                if dead.is_some_and(|d| d.contains(&n)) {
                    pruned += 1;
                    continue;
                }
                let dn = sc.depth[n as usize];
                if max_depth.is_some_and(|m| dn >= m) {
                    truncated |= !self.edges[n as usize].is_empty();
                    continue;
                }
                for &(d, tile, pi) in &self.edges[n as usize] {
                    if sc.stamp[d as usize] == epoch || allowed.is_some_and(|a| !a[tile as usize]) || blocked.contains(&d) {
                        continue;
                    }
                    sc.stamp[d as usize] = epoch;
                    sc.prev[d as usize] = (n, tile, pi);
                    sc.depth[d as usize] = dn.saturating_add(1);
                    sc.queue.push_back(d);
                }
            }
            if truncated {
                if let Some(r) = region.as_mut() {
                    r.clear();
                }
            }
            (None, visited, pruned)
        })
    }

    /// A* from `srcs` to the single node `dst`, avoiding `blocked`; the
    /// heuristic is grid distance / 12 (no wire spans more than about a dozen
    /// tiles per pip), so it stays close to admissible and finds short routes
    /// without searching the device.
    fn search_to(&self, srcs: &[u32], dst: u32, blocked: &NodeSet, allowed: Option<&[bool]>) -> Option<Found> {
        self.search_to_counted(srcs, dst, blocked, allowed).0
    }

    /// `search_to`, also returning how many nodes it popped.
    fn search_to_counted(&self, srcs: &[u32], dst: u32, blocked: &NodeSet, allowed: Option<&[bool]>) -> (Option<Found>, u64) {
        let mut visited = 0u64;
        use std::cmp::Reverse;
        use std::collections::BinaryHeap;
        const NONE: u32 = u32::MAX;
        let (tx, ty) = self.node_xy[dst as usize];
        let h = |n: u32| -> u32 {
            let (x, y) = self.node_xy[n as usize];
            ((x as i32 - tx as i32).unsigned_abs() + (y as i32 - ty as i32).unsigned_abs()) / 12
        };
        SCRATCH.with_borrow_mut(|sc| {
            if sc.stamp.len() != self.edges.len() {
                sc.stamp = vec![0; self.edges.len()];
                sc.prev = vec![(NONE, 0, 0); self.edges.len()];
                sc.depth = vec![0; self.edges.len()];
                sc.epoch = 0;
            }
            sc.epoch = sc.epoch.wrapping_add(1);
            if sc.epoch == 0 {
                sc.stamp.iter_mut().for_each(|x| *x = 0);
                sc.epoch = 1;
            }
            let epoch = sc.epoch;
            let mut heap: BinaryHeap<Reverse<(u32, u32)>> = BinaryHeap::new();
            for &n in srcs {
                if sc.stamp[n as usize] != epoch {
                    sc.stamp[n as usize] = epoch;
                    sc.prev[n as usize] = (NONE, 0, 0);
                    sc.depth[n as usize] = 0;
                    heap.push(Reverse((h(n), n)));
                }
            }
            while let Some(Reverse((_, n))) = heap.pop() {
                visited += 1;
                if n == dst {
                    let mut path = Vec::new();
                    let mut cur = n;
                    while sc.prev[cur as usize].0 != NONE {
                        let (p, tile, pi) = sc.prev[cur as usize];
                        path.push((cur, tile, pi));
                        cur = p;
                    }
                    path.reverse();
                    return (Some((n, path)), visited);
                }
                let g = sc.depth[n as usize] as u32;
                for &(d, tile, pi) in &self.edges[n as usize] {
                    if allowed.is_some_and(|a| !a[tile as usize]) || blocked.contains(&d) {
                        continue;
                    }
                    if sc.stamp[d as usize] == epoch && (sc.depth[d as usize] as u32) <= g + 1 {
                        continue;
                    }
                    sc.stamp[d as usize] = epoch;
                    sc.prev[d as usize] = (n, tile, pi);
                    sc.depth[d as usize] = (g + 1).min(u16::MAX as u32) as u16;
                    heap.push(Reverse((g + 1 + h(d), d)));
                }
            }
            (None, visited)
        })
    }

    /// Nodes from which `dst` can be reached (ignoring blocking), as a
    /// bitset; cached per sink.
    fn reaches(&self, dst: u32) -> Arc<Vec<u64>> {
        if let Some(r) = self.reach.lock().unwrap().get(&dst) {
            return r.clone();
        }
        let r = Arc::new(self.reverse_closure(&[dst]));
        self.reach.lock().unwrap().insert(dst, r.clone());
        r
    }

    /// Bitset of the nodes from which any of `seeds` can be reached
    /// (ignoring blocking), seeds included.
    fn reverse_closure(&self, seeds: &[u32]) -> Vec<u64> {
        let rev = self.rev.get_or_init(|| {
            let mut rev = vec![Vec::new(); self.edges.len()];
            for (s, es) in self.edges.iter().enumerate() {
                for &(d, _, _) in es {
                    rev[d as usize].push(s as u32);
                }
            }
            rev
        });
        let mut bits = vec![0u64; self.edges.len().div_ceil(64)];
        let set = |bits: &mut Vec<u64>, n: u32| {
            let (w, b) = ((n / 64) as usize, n % 64);
            let was = bits[w] >> b & 1 == 1;
            bits[w] |= 1 << b;
            !was
        };
        let mut stack = Vec::new();
        for &n in seeds {
            if set(&mut bits, n) {
                stack.push(n);
            }
        }
        while let Some(n) = stack.pop() {
            for &s in &rev[n as usize] {
                if set(&mut bits, s) {
                    stack.push(s);
                }
            }
        }
        bits
    }

    /// For each target pip: can a chain from `from` reach its source
    /// (avoiding `blocked`, inside `tile_types`), and can its sink reach
    /// `to`? A target failing either test cannot sit in a launch -> capture
    /// chain at all and needs a different test structure.
    pub fn feasibility(&self, from: &str, to: &str, targets: &[String], tile_types: Option<&[String]>, blocked: &HashSet<u32>) -> R<Vec<(String, bool, bool)>> {
        let allowed: Option<Vec<bool>> = tile_types.map(|ts| self.tiles.iter().map(|(_, ty)| ts.iter().any(|t| t == ty)).collect());
        let (src, dst) = (self.node(from)?, self.node(to)?);
        // Backward reach from the sink through unblocked nodes only.
        self.reaches(dst); // builds the reverse adjacency
        let rev = self.rev.get().unwrap();
        let mut back = vec![0u64; self.edges.len().div_ceil(64)];
        back[(dst / 64) as usize] |= 1 << (dst % 64);
        let mut stack = vec![dst];
        while let Some(n) = stack.pop() {
            for &p in &rev[n as usize] {
                let (w, b) = ((p / 64) as usize, p % 64);
                if back[w] >> b & 1 == 0 && !blocked.contains(&p) {
                    back[w] |= 1 << b;
                    stack.push(p);
                }
            }
        }
        let mut fwd = vec![0u64; self.edges.len().div_ceil(64)];
        let mut stack: Vec<u32> = self.fixed_closure([src]).into_iter().collect();
        for &n in &stack {
            fwd[(n / 64) as usize] |= 1 << (n % 64);
        }
        while let Some(n) = stack.pop() {
            for &(d, tile, _) in &self.edges[n as usize] {
                if allowed.as_ref().is_some_and(|a| !a[tile as usize]) || blocked.contains(&d) {
                    continue;
                }
                let (w, b) = ((d / 64) as usize, d % 64);
                if fwd[w] >> b & 1 == 0 {
                    fwd[w] |= 1 << b;
                    stack.push(d);
                }
            }
        }
        let has = |bits: &[u64], n: u32| bits[(n / 64) as usize] >> (n % 64) & 1 == 1;
        Ok(targets
            .iter()
            .map(|t| match self.resolve(t) {
                Ok(r) => (t.clone(), has(&fwd, r.a), has(&back, r.b) && !blocked.contains(&r.b)),
                Err(_) => (t.clone(), false, false),
            })
            .collect())
    }

    /// The path a configured chain takes from `from` to `to`: a breadth-first
    /// walk over fixed edges, feature pips whose line is in `lines`, and
    /// relays whose lines are all in `lines`. Side branches never reach the
    /// sink, so the result is the chain's own path.
    pub fn configured_path(&self, from: &str, to: &str, lines: &HashSet<String>) -> R<Vec<Pip>> {
        let (src, dst) = (self.node(from)?, self.node(to)?);
        let mut prev: HashMap<u32, (u32, u32, u32)> = HashMap::new();
        let mut q = VecDeque::from([src]);
        let mut seen = HashSet::from([src]);
        while let Some(n) = q.pop_front() {
            if n == dst {
                let mut path = Vec::new();
                let mut cur = dst;
                while let Some(&(p, tile, pi)) = prev.get(&cur) {
                    path.push(self.pip(tile, pi));
                    cur = p;
                }
                path.reverse();
                return Ok(path);
            }
            for &(d, tile, pi) in &self.edges[n as usize] {
                if seen.contains(&d) {
                    continue;
                }
                let p = self.pip(tile, pi);
                let on = match &p.enable {
                    Enable::Fixed => true,
                    _ => p.fasm_lines().iter().all(|l| lines.contains(l)),
                };
                if on {
                    seen.insert(d);
                    prev.insert(d, (n, tile, pi));
                    q.push_back(d);
                }
            }
        }
        Err(format!("no configured path {from} -> {to}"))
    }

    /// Every pin node of one CLB site.
    pub fn site_pin_nodes(&self, tile: &str, site: &str) -> Vec<u32> {
        let Some(&ti) = self.tile_index.get(tile) else { return vec![] };
        let td = &self.types[&self.tiles[ti as usize].1];
        td.site_pins.iter().filter(|(s, _, _)| s == site).map(|&(_, _, w)| self.node_of[(self.wire_base[ti as usize] + w) as usize]).collect()
    }

    /// Fixed edges from `src` to `dst` (shortest), as pips.
    fn fixed_path(&self, src: u32, dst: u32) -> Vec<Pip> {
        let mut prev: HashMap<u32, (u32, u32, u32)> = HashMap::new();
        let mut q = VecDeque::from([src]);
        while let Some(n) = q.pop_front() {
            if n == dst {
                let mut out = Vec::new();
                let mut c = dst;
                while let Some(&(p, t, pi)) = prev.get(&c) {
                    out.push(self.pip(t, pi));
                    c = p;
                }
                out.reverse();
                return out;
            }
            for &(d, t, pi) in &self.edges[n as usize] {
                if self.is_fixed(t, pi) && d != src && !prev.contains_key(&d) {
                    prev.insert(d, (n, t, pi));
                    q.push_back(d);
                }
            }
        }
        vec![]
    }

    /// Route nets as trees: each `(from, [to..])` gets a tree from `from` to
    /// every sink, each sink reached by a search from everything the tree
    /// already drives. Every net's ends and every node in `reserve` are kept
    /// from other nets; nets never touch each other or `blocked`, side
    /// branches included. Returns each net's pips (fixed edges included).
    pub fn connect(&self, nets: &[(String, Vec<String>)], reserve: &[u32], tile_types: Option<&[String]>, blocked: &HashSet<u32>) -> Vec<R<Vec<Pip>>> {
        let allowed: Option<Vec<bool>> = tile_types.map(|ts| self.tiles.iter().map(|(_, ty)| ts.iter().any(|t| t == ty)).collect());
        let allowed = allowed.as_deref();
        let mut nb = NodeSet::new(self.edges.len());
        nb.extend(blocked.iter().copied());
        nb.extend(self.exclusion(reserve.iter().copied()));

        // Resolve and reserve every end first.
        let mut ends: Vec<R<NetEnds>> = Vec::new();
        for (from, to) in nets {
            let r = (|| {
                let s = self.node(from)?;
                let mut sinks = Vec::new();
                for t in to {
                    let d = self.node(t)?;
                    sinks.push((d, self.exclusion([d])));
                }
                Ok((s, sinks))
            })();
            if let Ok((s, sinks)) = &r {
                nb.extend(self.exclusion([*s]));
                for (_, ex) in sinks {
                    nb.extend(ex.iter().copied());
                }
            }
            ends.push(r);
        }

        let mut out = Vec::new();
        for ((from, _), e) in nets.iter().zip(ends) {
            let (src, sinks) = match e {
                Ok(x) => x,
                Err(err) => {
                    out.push(Err(err));
                    continue;
                }
            };
            let mut local = nb.clone();
            for (_, ex) in &sinks {
                for n in ex {
                    local.remove(n);
                }
            }
            let mut tree: Vec<u32> = self.fixed_closure([src]).into_iter().collect();
            let mut on_path: HashSet<u32> = HashSet::from([src]);
            let mut pips = Vec::new();
            let mut failed = None;
            for (d, _) in &sinks {
                if tree.contains(d) {
                    continue;
                }
                match self.search_to(&tree, *d, &local, allowed) {
                    Some((_, path)) => {
                        // The route leaves the tree at some node; if that node is
                        // only reached from the source over fixed edges (a site
                        // pin's hard wire to LOGIC_OUTS, say), those edges are on
                        // the path too.
                        if let Some(&(n1, t1, p1)) = path.first() {
                            let origin = tree.iter().copied().find(|&o| self.edges[o as usize].iter().any(|&e| e == (n1, t1, p1)));
                            if let Some(o) = origin.filter(|&o| o != src && !on_path.contains(&o)) {
                                pips.extend(self.fixed_path(src, o));
                            }
                        }
                        on_path.extend(path.iter().map(|&(n, _, _)| n));
                        let added: Vec<u32> = self.fixed_closure(path.iter().map(|&(n, _, _)| n)).into_iter().collect();
                        local.extend(added.iter().copied());
                        tree.extend(added);
                        pips.extend(path.iter().map(|&(_, t, p)| self.pip(t, p)));
                    }
                    None => {
                        failed = Some(format!("no route {from} -> sink node {d}"));
                        break;
                    }
                }
            }
            match failed {
                Some(f) => out.push(Err(f)),
                None => {
                    nb.extend(self.exclusion(tree.iter().copied()));
                    out.push(Ok(pips));
                }
            }
        }
        out
    }

    /// Pack test chains. Each `(from, to)` pair gets one chain that threads
    /// up to `max_via` of `targets` (nearest first), never touching a node in
    /// `blocked` or another chain, and never letting a fixed edge from its
    /// nodes reach anything already in use. Targets used are removed from
    /// `targets`. A pair that cannot be routed at all gets an error.
    #[allow(clippy::too_many_arguments)]
    pub fn pack(
        &self,
        ends: &[(String, String)],
        targets: &mut Vec<String>,
        opts: &PackOpts,
        tile_types: Option<&[String]>,
        blocked: &mut HashSet<u32>,
        stats: &mut PackStats,
    ) -> Vec<R<Chain>> {
        let allowed: Option<Vec<bool>> = tile_types.map(|ts| self.tiles.iter().map(|(_, ty)| ts.iter().any(|t| t == ty)).collect());
        let allowed = allowed.as_deref();
        let mut pool: Vec<Option<Target>> = Vec::new();
        let mut unresolved = Vec::new();
        for t in targets.drain(..) {
            match self.resolve(&t) {
                Ok(r) if allowed.is_none_or(|a| a[r.tile as usize]) => pool.push(Some(r)),
                _ => unresolved.push(t),
            }
        }
        let mut by_src: HashMap<u32, Vec<usize>> = HashMap::new();
        // The search tests every node it visits for "is this a target's
        // source"; a bit test answers almost all of them without hashing.
        let mut is_src = NodeSet::new(self.edges.len());
        for (i, t) in pool.iter().enumerate() {
            let a = t.as_ref().unwrap().a;
            by_src.entry(a).or_default().push(i);
            is_src.insert(a);
        }
        let mut is_src = Lookup { is_src, dead: NodeSet::new(self.edges.len()) };

        // Every chain's ends are reserved before any chain is routed: no
        // chain may touch another's launch fan-out, or anything that drives
        // another's capture pin. Each chain gets only its own sink back.
        let mut sinks: Vec<HashSet<u32>> = Vec::new();
        for (from, to) in ends {
            if let (Ok(s), Ok(d)) = (self.node(from), self.node(to)) {
                blocked.extend(self.exclusion([s]));
                let ex = self.exclusion([d]);
                blocked.extend(ex.iter().copied());
                sinks.push(ex);
            } else {
                sinks.push(HashSet::new());
            }
        }

        let mut nb = NodeSet::new(self.edges.len());
        nb.extend(blocked.iter().copied());
        let mut out = Vec::new();
        for ((from, to), own_sink) in ends.iter().zip(&sinks) {
            let t0 = std::time::Instant::now();
            let mut dead = Vec::new();
            let r = self.pack_one(from, to, own_sink, &mut pool, &by_src, &is_src, opts, allowed, &nb, stats, &mut dead).map(|(c, own, _)| {
                nb.extend(self.exclusion(own));
                c
            });
            is_src.dead.extend(dead);
            stats.chain_ms.push(t0.elapsed().as_millis() as u64);
            if std::env::var_os("OXIBIT_ROUTE_TRACE").is_some() {
                eprintln!("pack {from}: {} ms, {}", t0.elapsed().as_millis(), match &r {
                    Ok(c) => format!("{} targets, {} pips", c.targets.len(), c.pips.len()),
                    Err(e) => e.clone(),
                });
            }
            out.push(r);
        }
        stats.dead_nodes = is_src.dead.count();
        targets.extend(unresolved);
        targets.extend(pool.into_iter().flatten().map(|t| t.name));
        out
    }

    #[allow(clippy::too_many_arguments)]
    fn pack_one(
        &self,
        from: &str,
        to: &str,
        own_sink: &HashSet<u32>,
        pool: &mut [Option<Target>],
        by_src: &HashMap<u32, Vec<usize>>,
        is_src: &Lookup,
        opts: &PackOpts,
        allowed: Option<&[bool]>,
        blocked: &NodeSet,
        stats: &mut PackStats,
        dead_out: &mut Vec<u32>,
    ) -> Attempt {
        let mut region: Vec<u32> = Vec::new();
        struct Step {
            index: usize,
            target: Target,
            path: Vec<(u32, u32, u32)>,
            added: Vec<u32>,
        }

        let (src, dst) = (self.node(from)?, self.node(to)?);
        let can_finish = self.reaches(dst);
        // `blocked` is closed under `exclusion`, so a search that avoids it
        // never touches a node in use, side branches included. The sink and
        // whatever drives it through fixed edges are this chain's.
        let mut local = blocked.clone();
        for n in own_sink {
            local.remove(n);
        }

        let mut steps: Vec<Step> = Vec::new();
        let mut skip: HashSet<usize> = HashSet::new();
        let mut adding = true;
        let mut rollbacks = 0;
        const MAX_ROLLBACKS: usize = 16;
        // The next target is looked for `target_hops` pips away at most. At
        // 40 (the old fixed value) a miss covered the whole device: long
        // wires and clock spines cross a dozen or more tiles per pip.

        loop {
            let cur = steps.last().map_or(src, |s| s.target.b);
            // Sorted: HashSet order is per process, and the search seeds its
            // queue in this order, so unsorted seeds made plans irreproducible.
            let mut from_here: Vec<u32> = self.fixed_closure([cur]).into_iter().collect();
            from_here.sort_unstable();
            if adding && steps.len() < opts.max_via {
                let usable = |i: usize, local: &NodeSet, skip: &HashSet<usize>| {
                    !skip.contains(&i)
                        && pool[i].as_ref().is_some_and(|t| t.b != dst && !local.contains(&t.b) && (can_finish[(t.b / 64) as usize] >> (t.b % 64) & 1 == 1))
                };
                let want = opts.dead_regions.then_some(&mut region);
                let dead = opts.dead_regions.then_some(&is_src.dead);
                let (found, visited, pruned) = self.search_counted(&from_here, &local, allowed, Some(opts.target_hops), dead, want, |n| {
                    is_src.is_src.contains(&n) && by_src.get(&n).is_some_and(|ix| ix.iter().any(|&i| usable(i, &local, &skip)))
                });
                stats.target_searches += 1;
                stats.target_visited += visited;
                stats.target_pruned += pruned;
                if found.is_none() {
                    stats.target_misses += 1;
                    dead_out.append(&mut region);
                    if std::env::var_os("OXIBIT_ROUTE_TRACE").is_some() {
                        let left = (0..pool.len()).filter(|&i| usable(i, &local, &skip)).count();
                        eprintln!(
                            "  miss {from}: after {} targets (at {}), {visited} nodes, {left} usable targets in pool",
                            steps.len(),
                            steps.last().map_or("launch", |s| s.target.name.as_str())
                        );
                    }
                } else {
                    stats.target_hit_visited += visited;
                    stats.target_hit_max = stats.target_hit_max.max(visited);
                }
                if let Some((a, mut path)) = found {
                    let i = by_src[&a].iter().copied().find(|&i| usable(i, &local, &skip)).unwrap();
                    let b = pool[i].as_ref().unwrap().b;
                    if path.iter().any(|&(n, _, _)| n == b) {
                        skip.insert(i);
                        continue;
                    }
                    let target = pool[i].take().unwrap();
                    path.push((target.b, target.tile, target.pi));
                    let added: Vec<u32> =
                        self.fixed_closure(path.iter().map(|&(n, _, _)| n)).into_iter().filter(|n| !local.contains(n)).collect();
                    if added.contains(&dst) {
                        pool[i] = Some(target);
                        skip.insert(i);
                        continue;
                    }
                    local.extend(added.iter().copied());
                    steps.push(Step { index: i, target, path, added });
                    continue;
                }
            }

            // Finish to the sink; on failure give the last target back.
            let (last, visited) = self.search_to_counted(&from_here, dst, &local, allowed);
            let last = last.map(|(_, p)| p);
            stats.finish_searches += 1;
            stats.finish_visited += visited;
            if last.is_none() {
                stats.finish_misses += 1;
            }
            if let Some(last) = last {
                let mut pips = Vec::new();
                let mut targets = Vec::new();
                for s in &steps {
                    targets.push(s.target.name.clone());
                    pips.extend(s.path.iter().map(|&(_, t, p)| self.pip(t, p)));
                }
                pips.extend(last.iter().map(|&(_, t, p)| self.pip(t, p)));
                let own: Vec<u32> = std::iter::once(src)
                    .chain(steps.iter().flat_map(|s| s.path.iter().map(|&(n, _, _)| n)))
                    .chain(last.iter().map(|&(n, _, _)| n))
                    .chain([dst])
                    .collect();
                let taken = steps.iter().map(|s| s.index).collect();
                return Ok((Chain { pips, targets }, own, taken));
            }
            let Some(s) = steps.pop() else {
                return Err(format!("no route {from} -> {to}"));
            };
            if std::env::var_os("OXIBIT_ROUTE_TRACE").is_some() {
                eprintln!("  rollback {} at depth {}", s.target.name, steps.len());
            }
            for n in &s.added {
                local.remove(n);
            }
            skip.insert(s.index);
            pool[s.index] = Some(s.target);
            // Keep packing past a dead end, up to a budget; then just finish.
            rollbacks += 1;
            stats.rollbacks += 1;
            adding = rollbacks <= MAX_ROLLBACKS;
        }
    }
}

fn read_json(p: &std::path::Path) -> R<Value> {
    serde_json::from_str(&fs::read_to_string(p).map_err(|e| format!("{}: {e}", p.display()))?)
        .map_err(|e| format!("{}: {e}", p.display()))
}

/// The I/O tiles' site route-through pseudo-pips a design router may take,
/// with the tile-relative lines that make each conduct (as nextpnr's
/// xilinx pseudo-pip table configures them): the ILOGIC bypass from D or
/// DDLY to O, the OLOGIC bypass from D1 to OQ, and the IOB pair's output and
/// tristate pass-through. prjxray lists these as pseudo-pips with no segbits
/// (or as hints), so without this table an input pin or IDELAY output has no
/// way into the fabric and a differential output's slave no way to its
/// master. Lines the tile type has no segbits for are dropped by the caller.
fn io_pseudo(ty: &str, src: &str, dst: &str) -> Option<Vec<String>> {
    let side = ty.chars().next().filter(|c| *c == 'L' || *c == 'R')?;
    let fam = &ty[1..];
    let fam = fam.split('_').next().unwrap_or(fam);
    if fam == "IOI3" {
        let idx = |w: &str, pre: &str, suf: &str| -> Option<String> {
            let i = w.strip_prefix(pre)?.strip_suffix(suf)?;
            (i.len() <= 1 && i.chars().all(|c| c.is_ascii_digit())).then(|| i.to_string())
        };
        let s = side.to_string();
        if let Some(i) = idx(dst, "IOI_ILOGIC", "_O") {
            let y = if i.is_empty() { "0".to_string() } else { i.clone() };
            if src == format!("{s}IOI_ILOGIC{i}_D") {
                return Some(vec![format!("IDELAY_Y{y}.IDELAY_TYPE_FIXED"), format!("ILOGIC_Y{y}.ZINV_D")]);
            }
            if src == format!("{s}IOI_ILOGIC{i}_DDLY") {
                return Some(vec![format!("ILOGIC_Y{y}.IDELMUXE3.P0"), format!("ILOGIC_Y{y}.ZINV_D")]);
            }
        }
        if let Some(i) = idx(dst, &format!("{s}IOI_OLOGIC"), "_OQ") {
            let y = if i.is_empty() { "0".to_string() } else { i.clone() };
            if src == format!("IOI_OLOGIC{i}_D1") {
                return Some(vec![format!("OLOGIC_Y{y}.OMUX.D1"), format!("OLOGIC_Y{y}.OQUSED"), format!("OLOGIC_Y{y}.OSERDES.DATA_RATE_TQ.BUF")]);
            }
        }
        // T1 -> TQ conducts inverted unless ZINV_T1 is set.
        if let Some(i) = idx(dst, &format!("{s}IOI_OLOGIC"), "_TQ") {
            let y = if i.is_empty() { "0".to_string() } else { i.clone() };
            if src == format!("IOI_OLOGIC{i}_T1") {
                return Some(vec![format!("OLOGIC_Y{y}.ZINV_T1")]);
            }
        }
        return None;
    }
    if fam == "IOB33" || fam == "IOB18" {
        let pass = [("IOB_O_OUT0", "IOB_O_IN1"), ("IOB_O0", "IOB_O_OUT0"), ("IOB_T_OUT0", "IOB_T_IN1"), ("IOB_T0", "IOB_T_OUT0"), ("IOB_PADOUT1", "IOB_DIFFI_IN0")];
        return pass.iter().any(|&(a, b)| a == src && b == dst).then(Vec::new);
    }
    None
}

fn load_type(db: &Database, ty: &str) -> R<TypeData> {
    let path = db.root.join(format!("tile_type_{ty}.json"));
    let mut wires = Vec::new();
    let mut index = HashMap::new();
    let mut pips = Vec::new();
    if !path.is_file() {
        return Ok(TypeData { wires, index, pips, site_pins: vec![] }); // tile types with no routing data
    }
    let v = read_json(&path)?;

    let wire_names: Vec<String> = match &v["wires"] {
        Value::Array(a) => a.iter().filter_map(|w| w.get(0).and_then(Value::as_str).map(String::from)).collect(),
        Value::Object(o) => o.keys().cloned().collect(),
        _ => vec![],
    };
    for w in wire_names {
        index.insert(w.clone(), wires.len() as u32);
        wires.push(w);
    }

    // Site pins: wire -> (site, pin). CLB sites are named as in FASM
    // (SLICEL_X0); others TYPE_XxYy (RAMB18E1_X0Y1, DSP48E1_X0Y0).
    let clb = ty.starts_with("CLB");
    let mut pin_of: HashMap<String, (String, String)> = HashMap::new();
    {
        for site in v["sites"].as_array().into_iter().flatten() {
            let (st, x, y) = (site["type"].as_str().unwrap_or(""), site["x_coord"].as_i64().unwrap_or(0), site["y_coord"].as_i64().unwrap_or(0));
            let name = if clb { format!("{st}_X{x}") } else { format!("{st}_X{x}Y{y}") };
            for (pin, sp) in site["site_pins"].as_object().into_iter().flatten() {
                if let Some(w) = sp["wire"].as_str() {
                    pin_of.insert(w.to_string(), (name.clone(), pin.clone()));
                }
            }
        }
    }
    let relay = |src: &str, dst: &str| -> Option<(String, Vec<String>)> {
        if !clb {
            return None;
        }
        let ((ss, sp), (ds, dp)) = (pin_of.get(src)?, pin_of.get(dst)?);
        if ss != ds {
            return None;
        }
        let lut = |p: &str| p.len() == 1 && "ABCD".contains(p);
        // X_k -> X: the LUT passes input k to O6.
        if lut(dp) && sp.len() == 2 && sp.starts_with(dp.as_str()) {
            let k: u32 = sp[1..].parse().ok().filter(|k| (1..=6).contains(k))?;
            let init = (0..64u64).filter(|i| i >> (k - 1) & 1 == 1).fold(0u64, |acc, i| acc | 1 << i);
            return Some((ss.clone(), vec![format!("{ss}.{dp}LUT.INIT[63:0] = 64'h{init:016X}")]));
        }
        // X -> XMUX: the output mux selects O6.
        if lut(sp) && dp == &format!("{sp}MUX") {
            return Some((ss.clone(), vec![format!("{ss}.{sp}OUTMUX.O6")]));
        }
        None
    };

    let seg = db.segbits.get(&ty.to_uppercase());
    let has_bits = |f: &str| seg.is_some_and(|s| s.by_block.iter().any(|(_, m)| m.contains_key(f)));
    let always = |f: &str| seg.is_some_and(|s| s.ppips_always.contains(f));
    let pseudo_other = |f: &str| seg.is_some_and(|s| s.ppips.contains(f) && !s.ppips_always.contains(f));
    let default = |f: &str| seg.is_some_and(|s| s.ppips_default.contains(f));

    let mut add = |src: &str, dst: &str, hard: bool, index: &HashMap<String, u32>| {
        let (Some(&s), Some(&d)) = (index.get(src), index.get(dst)) else { return };
        // A BUFR or PHASER_IN passes I to O only when the site is
        // configured; prjxray still lists the pair as an `always` pseudo-pip.
        // No design route may run through one it did not place.
        if let (Some((ss, _)), Some((ds, _))) = (pin_of.get(src), pin_of.get(dst)) {
            if ss == ds && (ss.starts_with("BUFR_") || ss.starts_with("PHASER_IN_PHY_")) {
                return;
            }
        }
        let key = format!("{ty}.{dst}.{src}");
        let io = io_pseudo(ty, src, dst).filter(|_| !has_bits(&key));
        let (feature, relay, is_default) = if has_bits(&key) {
            (Some(key), None, false)
        } else if let Some(lines) = io.clone() {
            // An I/O site route-through: configured by the site's own bits
            // (those this tile type defines), or by none at all.
            let lines: Vec<String> = lines.into_iter().filter(|l| has_bits(&format!("{}.{l}", ty.to_uppercase()))).collect();
            match (lines.is_empty(), pin_of.get(dst).or(pin_of.get(src))) {
                (false, Some((site, _))) => (None, Some((site.clone(), lines)), false),
                _ => (None, None, false),
            }
        } else if always(&key) || (hard && !pseudo_other(&key)) {
            (None, None, false)
        } else if let Some(r) = relay(src, dst) {
            (None, Some(r), false)
        } else if default(&key) {
            (None, None, true) // kept, but an edge only with GraphOpts::defaults
        } else {
            return; // hint pseudo-pip, or a buffered pip with no known bits
        };
        let through = io.is_some() && relay.is_some();
        pips.push(TypePip { src: s, dst: d, feature, relay, default: is_default, through });
    };

    if let Some(o) = v["pips"].as_object() {
        for (name, p) in o {
            let body = name.split_once('.').map(|x| x.1).unwrap_or(name);
            let pseudo = p["is_pseudo"].as_str() == Some("1");
            if let Some((a, b)) = body.split_once("<<->>") {
                add(a, b, false, &index);
                add(b, a, false, &index);
            } else if let Some((a, b)) = body.split_once("->>") {
                add(a, b, false, &index);
            } else if let Some((a, b)) = body.split_once("->") {
                add(a, b, !pseudo, &index);
            }
        }
    }
    // FF relays: PX -> FF -> PQ. Clocked by the design's clock; the clock
    // pip is per instance, so the caller adds it (see `Graph::site_pin`).
    for (w, (site, pin)) in pin_of.iter().filter(|_| clb) {
        let lut = |p: &str| p.len() == 1 && "ABCD".contains(p);
        if pin.len() == 2 && pin.ends_with('X') && lut(&pin[..1]) {
            let l = &pin[..1];
            let q = pin_of.iter().find(|(_, (s2, p2))| s2 == site && p2 == &format!("{l}Q")).map(|(w, _)| w.clone());
            if let (Some(&src), Some(dst)) = (index.get(w), q.and_then(|q| index.get(&q).copied())) {
                let lines = vec![format!("{site}.{l}FFMUX.{l}X"), format!("{site}.{l}FF.ZINI"), format!("{site}.{l}FF.ZRST"), format!("{site}.FFSYNC")];
                pips.push(TypePip { src, dst, feature: None, relay: Some((site.clone(), lines)), default: false, through: false });
            }
        }
    }
    // Clock buffers as relays: a BUFGCTRL held on I0 (CE0 = S0 = 1, I1
    // side off, as nextpnr configures a BUFG) and a BUFHCE held enabled pass
    // a static level from input to output. The BUFGCTRL control pins are
    // wired to their IMUX, which reads 1 when nothing drives it.
    if (ty.starts_with("CLK_BUFG_") && ty != "CLK_BUFG_REBUF") || ty.starts_with("CLK_HROW") {
        let mut src_of: HashMap<String, Vec<String>> = HashMap::new();
        if let Some(o) = v["pips"].as_object() {
            for name in o.keys() {
                let body = name.split_once('.').map(|x| x.1).unwrap_or(name);
                for sep in ["->>", "->"] {
                    if let Some((a, b)) = body.split_once(sep) {
                        src_of.entry(b.to_string()).or_default().push(a.to_string());
                        break;
                    }
                }
            }
        }
        for site in v["sites"].as_array().into_iter().flatten() {
            let st = site["type"].as_str().unwrap_or("");
            let name = format!("{st}_X{}Y{}", site["x_coord"].as_i64().unwrap_or(0), site["y_coord"].as_i64().unwrap_or(0));
            let pin = |p: &str| site["site_pins"][p]["wire"].as_str().map(String::from);
            let (input, output, mut lines) = match st {
                "BUFGCTRL" => (
                    pin("I0"),
                    pin("O"),
                    vec![format!("BUFGCTRL.{name}.IN_USE"), format!("BUFGCTRL.{name}.ZINV_CE0"), format!("BUFGCTRL.{name}.ZINV_S0"), format!("BUFGCTRL.{name}.IS_IGNORE1_INVERTED")],
                ),
                "BUFHCE" => (pin("I"), pin("O"), vec![format!("BUFHCE.{name}.IN_USE"), format!("BUFHCE.{name}.ZINV_CE")]),
                _ => continue,
            };
            if st == "BUFGCTRL" {
                for cp in ["CE0", "S0", "IGNORE0", "IGNORE1", "CE1", "S1"] {
                    if let Some(w) = pin(cp) {
                        if let Some(imux) = src_of.get(&w).and_then(|ss| ss.iter().find(|s| s.contains("IMUX"))) {
                            lines.push(format!("{w}.{imux}"));
                        }
                    }
                }
            }
            if let (Some(i), Some(o)) = (input.and_then(|w| index.get(&w).copied()), output.and_then(|w| index.get(&w).copied())) {
                pips.push(TypePip { src: i, dst: o, feature: None, relay: Some((name.clone(), lines)), default: false, through: false });
            }
        }
    }

    // prjxray lists a clock buffer's I -> O as an `always` pseudo-pip (the
    // site route-through). It only conducts when the site is configured, so
    // where a relay exists for the pair the fixed edge is dropped and the
    // relay is the only way through.
    let relayed: HashSet<(u32, u32)> = pips.iter().filter(|p| p.relay.is_some()).map(|p| (p.src, p.dst)).collect();
    pips.retain(|p| p.relay.is_some() || p.feature.is_some() || !relayed.contains(&(p.src, p.dst)));

    let mut site_pins: Vec<(String, String, u32)> =
        pin_of.iter().filter_map(|(w, (site, pin))| index.get(w).map(|&i| (site.clone(), pin.clone(), i))).collect();
    site_pins.sort();
    Ok(TypeData { wires, index, pips, site_pins })
}

struct UnionFind {
    parent: Vec<u32>,
}

impl UnionFind {
    fn new(n: usize) -> Self {
        UnionFind { parent: (0..n as u32).collect() }
    }
    fn find(&mut self, mut x: u32) -> u32 {
        while self.parent[x as usize] != x {
            let p = self.parent[x as usize];
            self.parent[x as usize] = self.parent[p as usize];
            x = p;
        }
        x
    }
    fn union(&mut self, a: u32, b: u32) {
        let (ra, rb) = (self.find(a), self.find(b));
        if ra != rb {
            self.parent[ra as usize] = rb;
        }
    }
    /// Root of every element as a dense id 0..n_nodes.
    fn dense(mut self) -> (Vec<u32>, usize) {
        let n = self.parent.len();
        let mut id: HashMap<u32, u32> = HashMap::new();
        let mut out = Vec::with_capacity(n);
        for i in 0..n as u32 {
            let r = self.find(i);
            let next = id.len() as u32;
            out.push(*id.entry(r).or_insert(next));
        }
        let count = id.len();
        (out, count)
    }
}
