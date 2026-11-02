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

/// A constant's logic value.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Logic {
    Low,
    High,
}

impl Logic {
    pub fn of(high: bool) -> Self {
        if high { Logic::High } else { Logic::Low }
    }

    /// 0 for `Low`, 1 for `High`: an index into per-value tables.
    pub fn index(self) -> usize {
        self as usize
    }
}

/// Tie nodes of one value (VCC_WIRE or GND_WIRE), any of which may drive a
/// constant net. Never empty.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Ties {
    value: Logic,
    nodes: Box<[u32]>,
}

impl Ties {
    /// `None` when `nodes` is empty: a constant with no tie is not routable.
    pub fn new(value: Logic, mut nodes: Vec<u32>) -> Option<Self> {
        nodes.sort_unstable();
        nodes.dedup();
        (!nodes.is_empty()).then(|| Ties { value, nodes: nodes.into_boxed_slice() })
    }

    pub fn value(&self) -> Logic {
        self.value
    }

    pub fn nodes(&self) -> &[u32] {
        &self.nodes
    }
}

/// What drives a net.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Driver {
    /// One node: a site output pin, a single tie, or a recovered tree's root.
    Node(u32),
    /// A constant from whichever of these ties the router finds cheapest.
    Ties(Ties),
}

impl Driver {
    /// Every node the net may start from.
    pub fn nodes(&self) -> &[u32] {
        match self {
            Driver::Node(n) => std::slice::from_ref(n),
            Driver::Ties(t) => t.nodes(),
        }
    }

    /// One representative node, for names, distances and reports.
    pub fn anchor(&self) -> u32 {
        self.nodes()[0]
    }

    pub fn contains(&self, n: u32) -> bool {
        match self {
            Driver::Node(m) => *m == n,
            Driver::Ties(t) => t.nodes.binary_search(&n).is_ok(),
        }
    }
}

/// The electrical net a routed net belongs to. Nets of one identity may
/// share nodes: every constant of one value is the same signal, so two
/// GND routes may merge where two signals may not.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Identity {
    /// The net at this index, alone.
    Own(usize),
    /// Every tie-driven net of this value.
    Const(Logic),
}

#[derive(Debug, Clone)]
pub struct Net {
    /// Where the tree starts.
    pub driver: Driver,
    /// Leaf nodes the tree reaches.
    pub sinks: Vec<u32>,
    /// Sinks any one node of which completes the net: the inputs of a LUT,
    /// whose INIT can be permuted to read them in any order. Empty for nets
    /// recovered from FASM, whose pins are already chosen.
    pub pin_groups: Vec<Vec<u32>>,
    /// The design's own routing of this net.
    pub edges: Vec<Edge>,
    pub class: NetClass,
}

impl Net {
    /// The electrical net this one belongs to, as net `index` of its list.
    pub fn identity(&self, index: usize) -> Identity {
        match &self.driver {
            Driver::Ties(t) => Identity::Const(t.value()),
            Driver::Node(_) => Identity::Own(index),
        }
    }
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
    /// A clock net of a placed design (driven by a clock buffer cell):
    /// routed, and may cross the clock buffers in the clock tiles (BUFHCE
    /// relays), which no other net takes.
    GlobalClock,
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
        nets.push(Net { driver: Driver::Node(root), sinks, pin_groups: Vec::new(), edges, class });
    }
    (nets, un)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ties_are_never_empty() {
        assert!(Ties::new(Logic::Low, Vec::new()).is_none());
        let t = Ties::new(Logic::High, vec![9, 3, 9]).expect("ties");
        assert_eq!(t.nodes(), &[3, 9]);
        let d = Driver::Ties(t);
        assert_eq!(d.anchor(), 3);
        assert!(d.contains(9) && !d.contains(4));
    }

    #[test]
    fn constants_of_one_value_share_an_identity() {
        let net = |driver| Net { driver, sinks: Vec::new(), pin_groups: Vec::new(), edges: Vec::new(), class: NetClass::Constant };
        let gnd = |n| Driver::Ties(Ties::new(Logic::Low, vec![n]).expect("tie"));
        assert_eq!(net(gnd(1)).identity(0), net(gnd(2)).identity(5));
        assert_ne!(net(gnd(1)).identity(0), net(Driver::Ties(Ties::new(Logic::High, vec![1]).expect("tie"))).identity(0));
        assert_ne!(net(Driver::Node(1)).identity(0), net(Driver::Node(1)).identity(1));
    }
}
