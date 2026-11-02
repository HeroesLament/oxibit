//! Where the routing meets a hard block's pins, for the placer's
//! wirelength.
//!
//! A slice is small enough to stand for all of its pins. The PS7 is not:
//! its site pins sit on nodes that run across the PSS tiles, and the
//! interconnect each one is driven from (or drives) is tens of tiles from
//! the site's own grid point. Costing every PS7 net at that one point let
//! the placer park the AXI HP address registers (`hp_awaddr`, `hp_araddr`)
//! ~90 tiles from their pins on the mimohalow board top, and those one-sink
//! nets became the router's worst floods (3-5M pops per search).
//!
//! `PinPoints` gives each pin the mean grid position of the interconnect
//! on the far side of its configurable pips: for an input pin, the source
//! nodes of the configurable pips into the chain of fixed edges that leads
//! to it; for an output pin, the destinations of the configurable pips out
//! of its fixed fan-out. The pin's own node, or a node of its fixed chain,
//! can span the PSS tiles; the interconnect node across a configurable pip
//! is the switchbox the router enters or leaves by. Grid coordinates are
//! the tilegrid's, as `Loc::Grid` and `Graph::node_xy`.

use std::collections::{HashMap, HashSet};

use oxibit_fasm::route::{EdgeKind, Graph};

/// Fixed-edge hops followed from a pin before giving up.
const MAX_DEPTH: u32 = 8;

/// Grid points of one site's pins, by pin name.
#[derive(Debug, Clone, Default)]
pub struct PinPoints {
    points: HashMap<String, (i32, i32)>,
}

impl PinPoints {
    /// The pins of `site` in `tile` that configurable routing reaches.
    pub fn of_site(g: &Graph, tile: &str, site: &str) -> PinPoints {
        let mut points = HashMap::new();
        for (s, pin, node) in g.tile_site_pins(tile) {
            if s != site {
                continue;
            }
            let ends: Vec<u32> = entries(g, node).into_iter().chain(exits(g, node)).collect();
            if ends.is_empty() {
                continue;
            }
            let (sx, sy) = ends.iter().fold((0i64, 0i64), |(ax, ay), &n| {
                let (x, y) = g.node_xy(n);
                (ax + i64::from(x), ay + i64::from(y))
            });
            let k = ends.len() as i64;
            points.insert(pin, ((sx / k) as i32, (sy / k) as i32));
        }
        PinPoints { points }
    }

    pub fn get(&self, pin: &str) -> Option<(i32, i32)> {
        self.points.get(pin).copied()
    }

    /// Pins and their points, by pin name.
    pub fn iter(&self) -> impl Iterator<Item = (&str, (i32, i32))> {
        let mut v: Vec<_> = self.points.iter().map(|(k, &p)| (k.as_str(), p)).collect();
        v.sort_unstable();
        v.into_iter()
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }
}

/// Source nodes of the configurable pips into `pin`'s chain of fixed edges.
fn entries(g: &Graph, pin: u32) -> Vec<u32> {
    let mut out = Vec::new();
    let mut seen = HashSet::new();
    let mut stack = vec![(pin, 0u32)];
    while let Some((n, depth)) = stack.pop() {
        if !seen.insert(n) {
            continue;
        }
        for (s, tile, pi) in g.in_edges(n) {
            match g.edge_kind(tile, pi) {
                EdgeKind::Feature => out.push(s),
                EdgeKind::Fixed if depth < MAX_DEPTH => stack.push((s, depth + 1)),
                _ => {}
            }
        }
    }
    out.sort_unstable();
    out.dedup();
    out
}

/// Destination nodes of the configurable pips out of `pin`'s fixed fan-out.
fn exits(g: &Graph, pin: u32) -> Vec<u32> {
    let mut out = Vec::new();
    let mut seen = HashSet::new();
    let mut stack = vec![(pin, 0u32)];
    while let Some((n, depth)) = stack.pop() {
        if !seen.insert(n) {
            continue;
        }
        for &(d, tile, pi) in g.out_edges(n) {
            match g.edge_kind(tile, pi) {
                EdgeKind::Feature => out.push(d),
                EdgeKind::Fixed if depth < MAX_DEPTH => stack.push((d, depth + 1)),
                _ => {}
            }
        }
    }
    out.sort_unstable();
    out.dedup();
    out
}
