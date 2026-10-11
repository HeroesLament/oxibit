//! Nets recovered from a routed design's FASM.
//!
//! Every pip line the design writes is an edge of the routing graph. The
//! edges form one tree per net: a root no listed edge drives, and leaves no
//! listed edge leaves. Stripping a design's routing and keeping each tree's
//! root and leaves gives the router exactly the problem the original router
//! solved, on the same placement.

use std::collections::{HashMap, HashSet};

use oxibit_fasm::route::{EdgeKind, Graph, PipLookup, pip_lines};

/// One routed edge of the design.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Edge {
    pub src: u32,
    pub dst: u32,
    pub tile: u32,
    pub pi: u32,
    pub kind: EdgeKind,
}

#[derive(Debug, Clone)]
pub struct Net {
    /// The tree's root node.
    pub source: u32,
    /// Leaf nodes the tree reaches.
    pub sinks: Vec<u32>,
    /// The design's own routing of this net.
    pub edges: Vec<Edge>,
    pub class: NetClass,
}

/// How the router treats a net.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum NetClass {
    /// Ordinary signal: routed.
    Signal,
    /// Rooted at a VCC_WIRE or GND_WIRE tie-off or a DSP/site constant
    /// source: routed like a signal, but reported separately.
    Constant,
    /// Uses `default` pseudo-pips (an interconnect input reading VCC because
    /// nothing drives it). Needs no routing; its nodes are kept clear.
    Default,
    /// On the global clock network (BUFG, HROW, HCLK): kept as the design
    /// routed it.
    Clock,
}

/// What `from_fasm` could not place in a tree.
#[derive(Debug, Default)]
pub struct Unresolved {
    /// Lines that look like pips but name no wire (site features).
    pub not_pips: usize,
    /// Pip lines the graph has no usable edge for.
    pub missing: Vec<String>,
    /// Nodes with more than one listed driver.
    pub multi_driven: Vec<u32>,
}

/// Wire-name test for the clock network.
fn is_clock_wire(name: &str) -> bool {
    ["CLK_BUFG", "CLK_HROW", "HCLK_", "CLK_HCLK", "GCLK", "CLK_L", "CLK_R"].iter().any(|p| name.contains(p))
}

/// Split a design's FASM routing into nets.
pub fn from_fasm(g: &Graph, fasm: &str) -> (Vec<Net>, Unresolved) {
    let mut un = Unresolved::default();
    let mut out: HashMap<u32, Vec<Edge>> = HashMap::new();
    let mut drivers: HashMap<u32, u32> = HashMap::new(); // dst -> number of listed drivers
    let mut clockish: HashSet<u32> = HashSet::new(); // nodes on clock-network edges

    for line in pip_lines(fasm) {
        match g.lookup_pip(&line) {
            PipLookup::Edge { src, dst, tile, pi, pip } => {
                let kind = g.edge_kind(tile, pi);
                out.entry(src).or_default().push(Edge { src, dst, tile, pi, kind });
                *drivers.entry(dst).or_default() += 1;
                if is_clock_wire(&pip.src) || is_clock_wire(&pip.dst) || pip.tile.starts_with("CLK_") {
                    clockish.insert(src);
                    clockish.insert(dst);
                }
            }
            PipLookup::NoWire => un.not_pips += 1,
            PipLookup::SameNode(_) => {}
            _ => un.missing.push(line),
        }
    }
    un.multi_driven = drivers.iter().filter(|&(_, &c)| c > 1).map(|(&n, _)| n).collect();
    un.multi_driven.sort_unstable();

    let mut roots: Vec<u32> = out.keys().copied().filter(|n| !drivers.contains_key(n)).collect();
    roots.sort_unstable();

    let mut nets = Vec::with_capacity(roots.len());
    for root in roots {
        let mut edges = Vec::new();
        let mut sinks = Vec::new();
        let mut stack = vec![root];
        let mut seen = HashSet::from([root]);
        while let Some(n) = stack.pop() {
            match out.get(&n) {
                Some(es) => {
                    for e in es {
                        edges.push(*e);
                        if seen.insert(e.dst) {
                            stack.push(e.dst);
                        }
                    }
                }
                None => sinks.push(n),
            }
        }
        sinks.sort_unstable();
        let class = if edges.iter().any(|e| e.kind == EdgeKind::Default) {
            NetClass::Default
        } else if edges.iter().any(|e| clockish.contains(&e.src) || clockish.contains(&e.dst)) {
            NetClass::Clock
        } else {
            let root_wire = g.edge_pip(edges[0].tile, edges[0].pi).src;
            if root_wire.contains("VCC") || root_wire.contains("GND") {
                NetClass::Constant
            } else {
                NetClass::Signal
            }
        };
        nets.push(Net { source: root, sinks, edges, class });
    }
    (nets, un)
}
