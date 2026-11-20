//! Routing for Nexus.
//!
//! - Clock nets (driven by a DCC) go first, on the global network, by
//!   nextpnr's rule: a backward breadth-first search from each sink to the
//!   net's tree that never enters general routing (`H0x`/`V0x` wires), and
//!   that only reaches DIMUX/DRMUX from their fabric inputs.
//! - Routes fixed during placement (clock source to DCC) are taken as is.
//! - Signal nets are routed by negotiated congestion (PathFinder): A* from
//!   the net's tree to each sink, a node's cost rising with present
//!   sharing and with its history of overuse, until nothing is shared.
//!   Signal nets never use the global clock network or Vcc.
//!
//! Occupancy is per electrical node (`Graph::group`): wires joined by
//! fixed connections are one wire.

use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap, HashSet, VecDeque};

use crate::graph::{Arc, Graph, NodeId};
use crate::netlist::{CellId, CellKind, NetId, Netlist};
use crate::place::Placement;
use crate::{Result, err};

#[derive(Debug)]
pub struct Routes {
    /// Arcs of each net's tree, source to sinks.
    pub arcs: Vec<Vec<Arc>>,
    pub iterations: usize,
}

/// Bel pin a cell pin lands on: an FF's D is DI when it sits on its LUT.
pub fn bel_pin<'a>(pl: &Placement, cell: CellId, pin: &'a str) -> &'a str {
    if pin == "M" && pl.ff_on_di.contains(&cell) { "DI" } else { pin }
}

pub fn pin_node(g: &Graph, nl: &Netlist, pl: &Placement, cell: CellId, pin: &str) -> Result<NodeId> {
    let b = &g.bels[pl.bel[cell]];
    let bp = bel_pin(pl, cell, pin);
    b.pin(bp).ok_or_else(|| err!("cell {} pin {pin}: bel {} has no pin {bp}", nl.cells[cell].name, b.group()))
}

fn is_global_name(s: &str) -> bool {
    ["HPBX", "VPSX", "HPRX"].iter().any(|p| s.starts_with(p))
        || s.contains("DCC")
        || s.contains("DCS")
        || s.contains("CMUX")
        || s.contains("MIDMUX")
}

fn is_general(s: &str) -> bool {
    let b = s.as_bytes();
    b.len() > 2 && (b[0] == b'H' || b[0] == b'V') && b[1] == b'0'
}

/// `?` matches any one character.
fn matches(s: &str, pat: &str) -> bool {
    s.len() == pat.len() && s.bytes().zip(pat.bytes()).all(|(a, b)| b == b'?' || a == b)
}

fn clock_arc_ok(g: &Graph, a: &Arc) -> bool {
    let to = g.name(g.nodes[a.to as usize].name);
    if is_general(to) {
        return false;
    }
    let from = g.name(g.nodes[a.from as usize].name);
    if matches(to, "JDI?_DIMUX") {
        return matches(from, "JM?_DIMUX");
    }
    if matches(to, "JDL?_DRMUX") {
        return matches(from, "JD?_DRMUX");
    }
    true
}

struct State<'g> {
    g: &'g Graph,
    /// Net holding each electrical node exclusively (clock trees, fixed
    /// routes, bel pins), or none.
    owner: HashMap<NodeId, NetId>,
    /// Signal nets on each electrical node.
    occ: Vec<u16>,
    hist: Vec<f32>,
}

impl<'g> State<'g> {
    fn grp(&self, n: NodeId) -> NodeId {
        self.g.group[n as usize]
    }
}

pub fn route(g: &Graph, nl: &Netlist, pl: &Placement) -> Result<Routes> {
    let nnodes = g.nodes.len();
    let mut st = State { g, owner: HashMap::new(), occ: vec![0; nnodes], hist: vec![0.0; nnodes] };
    let mut arcs: Vec<Vec<Arc>> = vec![Vec::new(); nl.nets.len()];

    // Endpoints.
    let mut src: Vec<Option<NodeId>> = vec![None; nl.nets.len()];
    let mut sinks: Vec<Vec<NodeId>> = vec![Vec::new(); nl.nets.len()];
    for (ni, net) in nl.nets.iter().enumerate() {
        if net.sinks.is_empty() {
            continue;
        }
        let (dc, dp) = net.driver.ok_or_else(|| err!("net {} has sinks but no driver", net.name))?;
        src[ni] = Some(pin_node(g, nl, pl, dc, dp)?);
        for &(c, p) in &net.sinks {
            sinks[ni].push(pin_node(g, nl, pl, c, p)?);
        }
    }
    // A DCC's CLKI and CLKO are joined by a fixed connection (its
    // route-through), so its input net and output net share that wire.
    let mut alias: Vec<Option<NetId>> = vec![None; nl.nets.len()];
    for (ni, net) in nl.nets.iter().enumerate() {
        if let Some((c, _)) = net.driver {
            if matches!(nl.cells[c].kind, CellKind::Dcc) {
                alias[ni] = nl.cells[c].input("CLKI");
            }
        }
    }
    let same = |a: NetId, b: NetId| a == b || alias[a] == Some(b) || alias[b] == Some(a);
    // Reserve every used pin for its net.
    for ni in 0..nl.nets.len() {
        for &n in src[ni].iter().chain(&sinks[ni]) {
            let gr = st.grp(n);
            if let Some(&o) = st.owner.get(&gr) {
                if same(o, ni) {
                    continue;
                }
                if o != ni {
                    return Err(err!(
                        "nets {} and {} share pin node {}",
                        nl.nets[o].name,
                        nl.nets[ni].name,
                        g.node_name(n)
                    ));
                }
            }
            st.owner.insert(gr, ni);
        }
    }

    // Fixed routes.
    for (&ni, path) in &pl.fixed_routes {
        for a in path {
            for n in [a.from, a.to] {
                let gr = st.grp(n);
                if st.owner.get(&gr).is_some_and(|&o| !same(o, ni)) {
                    return Err(err!("fixed route of {} crosses another net at {}", nl.nets[ni].name, g.node_name(n)));
                }
                st.owner.insert(gr, ni);
            }
        }
        arcs[ni] = path.clone();
    }

    // Clock nets.
    for (ni, net) in nl.nets.iter().enumerate() {
        let is_clk = net.driver.is_some_and(|(c, _)| matches!(nl.cells[c].kind, CellKind::Dcc));
        if !is_clk || pl.fixed_routes.contains_key(&ni) {
            continue;
        }
        let s = src[ni].ok_or_else(|| err!("clock net {} has no source", net.name))?;
        let mut tree: HashSet<NodeId> = HashSet::from([s]);
        for &t in &sinks[ni] {
            if tree.contains(&t) {
                continue;
            }
            // Backward BFS from the sink to the tree.
            let mut next: HashMap<NodeId, Arc> = HashMap::new();
            let mut q = VecDeque::from([t]);
            let mut seen = HashSet::from([t]);
            let mut hit = None;
            let mut iter = 0;
            while let Some(n) = q.pop_front() {
                iter += 1;
                if iter > 1_000_000 {
                    break;
                }
                for a in g.uphill(n) {
                    let p = a.from;
                    if seen.contains(&p) || !clock_arc_ok(g, a) {
                        continue;
                    }
                    if st.owner.get(&st.grp(p)).is_some_and(|&o| !same(o, ni)) {
                        continue;
                    }
                    seen.insert(p);
                    next.insert(p, *a);
                    if tree.contains(&p) {
                        hit = Some(p);
                        break;
                    }
                    q.push_back(p);
                }
                if hit.is_some() {
                    break;
                }
            }
            let mut cur = hit.ok_or_else(|| {
                err!("clock net {}: no global route to {}", net.name, g.node_name(t))
            })?;
            while cur != t {
                let a = next[&cur];
                arcs[ni].push(a);
                tree.insert(a.to);
                st.owner.insert(st.grp(a.to), ni);
                cur = a.to;
            }
        }
    }

    // Signal nets: PathFinder.
    let signal: Vec<NetId> = (0..nl.nets.len())
        .filter(|&ni| src[ni].is_some() && arcs[ni].is_empty() && !nl.nets[ni].clock)
        .filter(|&ni| !nl.nets[ni].driver.is_some_and(|(c, _)| matches!(nl.cells[c].kind, CellKind::Dcc)))
        .collect();
    let blocked: Vec<bool> = g
        .nodes
        .iter()
        .enumerate()
        .map(|(i, nd)| {
            let gi = g.group[i];
            gi == g.group[g.vcc as usize] || is_global_name(g.name(nd.name)) || (nd.x == 0 && nd.y == 0)
        })
        .collect();
    let mut pres = 0.5f32;
    let mut iterations = 0;
    let mut net_groups: Vec<HashSet<NodeId>> = vec![HashSet::new(); nl.nets.len()];
    for it in 0..60 {
        iterations = it + 1;
        for &ni in &signal {
            // Rip up.
            for gr in net_groups[ni].drain() {
                st.occ[gr as usize] = st.occ[gr as usize].saturating_sub(1);
            }
            arcs[ni].clear();
            let s = src[ni].unwrap_or_default();
            let mut tree: HashMap<NodeId, ()> = HashMap::from([(s, ())]);
            let sg = st.grp(s);
            net_groups[ni].insert(sg);
            st.occ[sg as usize] += 1;
            let mut order: Vec<NodeId> = sinks[ni].clone();
            let (sx, sy) = (g.nodes[s as usize].x, g.nodes[s as usize].y);
            order.sort_by_key(|&t| (g.nodes[t as usize].x - sx).abs() + (g.nodes[t as usize].y - sy).abs());
            for t in order {
                if tree.contains_key(&t) {
                    continue;
                }
                let path = astar(&st, &blocked, ni, &net_groups[ni], st.grp(s), &tree, t, pres)
                    .ok_or_else(|| err!("net {}: no route to {}", nl.nets[ni].name, g.node_name(t)))?;
                for a in path {
                    tree.insert(a.to, ());
                    let gr = st.grp(a.to);
                    if net_groups[ni].insert(gr) {
                        st.occ[gr as usize] += 1;
                    }
                    arcs[ni].push(a);
                }
            }
        }
        // Overuse: a group used by more than one signal net.
        let mut over = 0;
        for (i, &o) in st.occ.iter().enumerate() {
            if o > 1 {
                over += 1;
                st.hist[i] += (o - 1) as f32;
            }
        }
        if over == 0 {
            return Ok(Routes { arcs, iterations });
        }
        pres *= 1.8;
    }
    Err(err!("routing did not converge in {iterations} iterations"))
}

#[derive(PartialEq)]
struct Key(f32);
impl Eq for Key {}
impl PartialOrd for Key {
    fn partial_cmp(&self, o: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for Key {
    fn cmp(&self, o: &Self) -> std::cmp::Ordering {
        self.0.total_cmp(&o.0)
    }
}

#[allow(clippy::too_many_arguments)]
fn astar(
    st: &State,
    blocked: &[bool],
    ni: NetId,
    own: &HashSet<NodeId>,
    src_group: NodeId,
    tree: &HashMap<NodeId, ()>,
    t: NodeId,
    pres: f32,
) -> Option<Vec<Arc>> {
    let g = st.g;
    let (tx, ty) = (g.nodes[t as usize].x, g.nodes[t as usize].y);
    let h = |n: NodeId| {
        let nd = g.nodes[n as usize];
        0.25 * ((nd.x - tx).abs() + (nd.y - ty).abs()) as f32
    };
    let mut best: HashMap<NodeId, f32> = HashMap::new();
    let mut prev: HashMap<NodeId, Arc> = HashMap::new();
    let mut heap = BinaryHeap::new();
    for &n in tree.keys() {
        best.insert(n, 0.0);
        heap.push(Reverse((Key(h(n)), n)));
    }
    let tg = st.grp(t);
    let mut pops = 0usize;
    while let Some(Reverse((Key(f), n))) = heap.pop() {
        pops += 1;
        if pops > 3_000_000 {
            return None;
        }
        let gn = best[&n];
        if f > gn + h(n) + 1e-3 {
            continue;
        }
        if n == t {
            let mut path = Vec::new();
            let mut cur = t;
            while !tree.contains_key(&cur) {
                let a = prev[&cur];
                path.push(a);
                cur = a.from;
            }
            path.reverse();
            return Some(path);
        }
        for a in g.downhill(n) {
            let v = a.to;
            let gv = st.grp(v);
            if blocked[v as usize] && gv != tg {
                continue;
            }
            if let Some(&o) = st.owner.get(&gv) {
                if o != ni {
                    continue;
                }
                // Own pins: only the source's and this sink's wire.
                if gv != tg && gv != src_group {
                    continue;
                }
            }
            // Sharing with other nets only; this net's own wires are free.
            let others = st.occ[gv as usize] - u16::from(own.contains(&gv));
            let occ = others as f32;
            let cost = (1.0 + st.hist[gv as usize]) * (1.0 + pres * occ);
            let nv = gn + cost;
            if best.get(&v).is_none_or(|&b| nv < b) {
                best.insert(v, nv);
                prev.insert(v, *a);
                heap.push(Reverse((Key(nv + h(v)), v)));
            }
        }
    }
    None
}
