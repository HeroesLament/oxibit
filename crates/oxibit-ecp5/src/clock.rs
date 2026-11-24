//! Clock nets on the global network.
//!
//! A clock reaches its flops over the dedicated network rather than general
//! routing. On ECP5 (Trellis `docs/architecture/global_routing.rst`):
//!
//! ```text
//! pad / PLL --mid mux--> DCC (CLKI -> CLKO, DCC_<s><n>.MODE DCCA)
//!   --centre mux--> G_<qq>PCLK<n>            one per quadrant and index
//!   ==span==> G_HPRX<n>00 at each spine tile of the quadrant
//!   --spine buffer--> G_VPTX<n>00
//!   ==span==> G_VPTX<n>00 at every TAP_DRIVE of the spine's column in the quadrant
//!   --tap buffer--> L_HPBX<n>00 / R_HPBX<n>00
//!   ==span==> G_HPBX<n>00 in each logic tile the tap serves (globals.json `taps`)
//!   --tile clock mux--> CLK0 / CLK1 -> MUXCLK<z> -> slice CLK
//! ```
//!
//! The `--` steps are pips (or the DCC bel) in the database; the `==span==`
//! steps are one physical wire that libtrellis's per-tile names leave
//! unconnected (its `globalise_net_ecp5` notes "TODO: quadrants and
//! TAP_DRIVE regions"). This module adds them from `globals.json`.
//!
//! The search is a Dijkstra from the source over global wires and spans,
//! plus general wires only within `near` tiles of the source or a sink,
//! with global wires much cheaper than general ones. One search settles
//! every sink; the union of the sinks' shortest paths is the tree.

use std::cmp::Reverse;
use std::collections::{BTreeMap, BinaryHeap, HashMap, HashSet};
use std::path::Path;

use crate::db::DeviceDb;
use crate::graph::{Arc, Graph, NameId, Wire};
use crate::route::is_global;
use crate::{Result, err};

/// A quadrant's rows and columns.
#[derive(Clone, Copy, Debug)]
struct Quadrant {
    x0: i16,
    x1: i16,
    y0: i16,
    y1: i16,
}

/// A tap column: its tile column and the logic columns its left (`L_`) and
/// right (`R_`) branches serve.
#[derive(Clone, Copy, Debug)]
struct Tap {
    col: i16,
    left: (i16, i16),
    right: (i16, i16),
}

/// A spine tile: its quadrant, location and the tap column it feeds.
#[derive(Clone, Debug)]
struct Spine {
    quadrant: String,
    x: i16,
    y: i16,
    tap_col: i16,
}

/// One step of a clock tree.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClockArc {
    /// A pip or fixed connection of the routing graph.
    Pip(Arc),
    /// The same physical wire at another tile: no configuration.
    Span { from: Wire, to: Wire },
    /// Through a DCC bel, CLKI to CLKO: `mid` tile, `DCC_<s><n>` enable.
    Dcc { from: Wire, to: Wire, side: char, index: u8 },
    /// Through a PLL's reference select, CLK0 to PLLCSOUT (the PLL's
    /// CLKI): its SEL (`JSEL_PLLREFCS`) is tied low.
    RefCs { from: Wire, to: Wire },
    /// Through an ECLKSYNC bel, ECLKI to ECLKO: no configuration (its
    /// MODE is only for STOP).
    Sync { from: Wire, to: Wire },
}

impl ClockArc {
    pub fn to(self) -> Wire {
        match self {
            ClockArc::Pip(a) => a.to,
            ClockArc::Span { to, .. }
            | ClockArc::Dcc { to, .. }
            | ClockArc::RefCs { to, .. }
            | ClockArc::Sync { to, .. } => to,
        }
    }
    pub fn from(self) -> Wire {
        match self {
            ClockArc::Pip(a) => a.from,
            ClockArc::Span { from, .. }
            | ClockArc::Dcc { from, .. }
            | ClockArc::RefCs { from, .. }
            | ClockArc::Sync { from, .. } => from,
        }
    }
}

/// The global network's spans for one device.
#[derive(Debug)]
pub struct ClockNetwork {
    quadrants: BTreeMap<String, Quadrant>,
    taps: Vec<Tap>,
    spines: Vec<Spine>,
    /// Per edge I/O bank (left / right): its PIO column and the rows of its
    /// PIC tiles, which carry `BNK_ECLK<n>`.
    banks: BTreeMap<u32, (i16, Vec<i16>)>,
}

fn num(v: &serde_json::Value, k: &str) -> Result<i16> {
    v[k].as_i64()
        .and_then(|n| i16::try_from(n).ok())
        .ok_or_else(|| err!("globals.json: no {k}"))
}

impl ClockNetwork {
    /// From `<root>/ECP5/<device>/globals.json`.
    pub fn load(root: &Path, device: &str) -> Result<ClockNetwork> {
        let path = root.join("ECP5").join(device).join("globals.json");
        let text =
            std::fs::read_to_string(&path).map_err(|e| err!("{}: {e}", path.display()))?;
        let v: serde_json::Value =
            serde_json::from_str(&text).map_err(|e| err!("{}: {e}", path.display()))?;
        let mut quadrants = BTreeMap::new();
        for (name, q) in v["quadrants"].as_object().ok_or(err!("globals.json: quadrants"))? {
            quadrants.insert(
                name.clone(),
                Quadrant {
                    x0: num(q, "x0")?,
                    x1: num(q, "x1")?,
                    y0: num(q, "y0")?,
                    y1: num(q, "y1")?,
                },
            );
        }
        let mut taps = Vec::new();
        for (name, t) in v["taps"].as_object().ok_or(err!("globals.json: taps"))? {
            let col = name
                .strip_prefix('C')
                .and_then(|c| c.parse().ok())
                .ok_or_else(|| err!("globals.json: tap {name}"))?;
            taps.push(Tap {
                col,
                left: (num(t, "lx0")?, num(t, "lx1")?),
                right: (num(t, "rx0")?, num(t, "rx1")?),
            });
        }
        let mut spines = Vec::new();
        for (name, s) in v["spines"].as_object().ok_or(err!("globals.json: spines"))? {
            let (quadrant, col) = name.split_at(2);
            spines.push(Spine {
                quadrant: quadrant.to_owned(),
                x: num(s, "x")?,
                y: num(s, "y")?,
                tap_col: col.parse().map_err(|_| err!("globals.json: spine {name}"))?,
            });
        }
        let banks = load_banks(&root.join("ECP5").join(device).join("iodb.json"))?;
        Ok(ClockNetwork {
            quadrants,
            taps,
            spines,
            banks,
        })
    }

    /// Steps out of `w` that the routing graph does not have: spans of
    /// the same wire, DCCs, and the PLL reference select.
    fn virtual_downhill(&self, g: &Graph, w: Wire, out: &mut Vec<ClockArc>) {
        let name = &g.names[w.name as usize];
        let id = |s: &str| g.name_id_of(s);
        if name == "CLK0_PLLREFCS" {
            if let Some(o) = id("PLLCSOUT_PLLREFCS") {
                out.push(ClockArc::RefCs {
                    from: w,
                    to: Wire { x: w.x, y: w.y, name: o },
                });
            }
        }
        let span = |to: Wire, out: &mut Vec<ClockArc>| out.push(ClockArc::Span { from: w, to });
        // ECLKSYNC<n>: ECLKI -> ECLKO at the bel.
        if let Some(n) = name.strip_prefix("ECLKI_ECLKSYNC") {
            if let Some(o) = id(&format!("JECLKO_ECLKSYNC{n}")) {
                out.push(ClockArc::Sync {
                    from: w,
                    to: Wire { x: w.x, y: w.y, name: o },
                });
            }
        }
        // G_BANK<b>ECLK<n> -> BNK_ECLK<n> at each PIC tile of bank b.
        if let Some(rest) = name.strip_prefix("G_BANK") {
            if let Some((b, n)) = rest.split_once("ECLK") {
                if let (Ok(b), Some(t)) = (b.parse::<u32>(), id(&format!("BNK_ECLK{n}"))) {
                    if let Some((col, rows)) = self.banks.get(&b) {
                        for &y in rows {
                            span(Wire { x: *col, y, name: t }, out);
                        }
                    }
                    // And the bank's corner DDRDLL tile (its CLK mux takes
                    // BNK_ECLK0/1 there).
                    let corner = match b {
                        2 => Some("DDRDLL_UR"),
                        7 => Some("DDRDLL_UL"),
                        3 => Some("DDRDLL_LR"),
                        6 => Some("DDRDLL_LL"),
                        _ => None,
                    };
                    if let Some(c) = corner {
                        for tile in g.tiles.iter().filter(|tile| tile.ty.starts_with(c)) {
                            span(Wire { x: tile.x, y: tile.y, name: t }, out);
                        }
                    }
                }
            }
        }
        // G_<qq>PCLK<n> at (0, 0) -> G_HPRX<n>00 at each spine of qq.
        if let Some(rest) = name.strip_prefix("G_") {
            if rest.len() > 6 && &rest[2..6] == "PCLK" {
                let qq = &rest[..2];
                if let (true, Ok(n)) = (self.quadrants.contains_key(qq), rest[6..].parse::<u8>()) {
                    if let Some(h) = id(&format!("G_HPRX{n:02}00")) {
                        for s in self.spines.iter().filter(|s| s.quadrant == qq) {
                            span(Wire { x: s.x, y: s.y, name: h }, out);
                        }
                    }
                }
            }
            // G_VPTX at a spine -> the tap column's rows in that quadrant.
            if rest.starts_with("VPTX") {
                if let Some(s) = self.spines.iter().find(|s| s.x == w.x && s.y == w.y) {
                    if let Some(q) = self.quadrants.get(&s.quadrant) {
                        for y in q.y0..=q.y1 {
                            if y != w.y || s.tap_col != w.x {
                                span(Wire { x: s.tap_col, y, name: w.name }, out);
                            }
                        }
                    }
                }
            }
            // DCC: G_CLKI_<s>DCC<n> -> G_CLKO_<s>DCC<n>.
            if let Some(d) = rest.strip_prefix("CLKI_") {
                let mut cs = d.chars();
                if let Some(side) = cs.next() {
                    let index = cs.as_str().strip_prefix("DCC").and_then(|n| n.parse::<u8>().ok());
                    if let (Some(index), Some(o)) = (index, id(&format!("G_CLKO_{d}"))) {
                        out.push(ClockArc::Dcc {
                            from: w,
                            to: Wire { x: w.x, y: w.y, name: o },
                            side,
                            index,
                        });
                    }
                }
            }
        }
        // L_/R_HPBX at a tap -> G_HPBX across the branch.
        for (prefix, left) in [("L_HPBX", true), ("R_HPBX", false)] {
            if let Some(n) = name.strip_prefix(prefix) {
                let Some(h) = id(&format!("G_HPBX{n}")) else {
                    continue;
                };
                if let Some(t) = self.taps.iter().find(|t| t.col == w.x) {
                    let (x0, x1) = if left { t.left } else { t.right };
                    for x in x0..=x1 {
                        span(Wire { x, y: w.y, name: h }, out);
                    }
                }
            }
        }
    }
}

/// Costs and limits of the clock search.
#[derive(Clone, Copy, Debug)]
pub struct ClockOptions {
    /// Cost of a global wire (and of a span).
    pub global_cost: u32,
    /// Cost of a general wire.
    pub general_cost: u32,
    /// General wires are allowed within this many tiles (Chebyshev) of
    /// the source or a sink.
    pub near: i16,
}

impl Default for ClockOptions {
    fn default() -> Self {
        ClockOptions {
            global_cost: 1,
            general_cost: 20,
            near: 2,
        }
    }
}

/// Statistics of one clock tree.
#[derive(Clone, Copy, Debug, Default)]
pub struct ClockStats {
    pub arcs: usize,
    pub spans: usize,
    pub dccs: usize,
    pub visited: usize,
}

/// A routed clock tree.
#[derive(Clone, Debug)]
pub struct ClockTree {
    pub arcs: Vec<ClockArc>,
    pub stats: ClockStats,
}

impl ClockTree {
    /// Every wire the tree holds, the source included.
    pub fn wires(&self) -> impl Iterator<Item = Wire> + '_ {
        self.arcs
            .iter()
            .flat_map(|a| [a.from(), a.to()])
    }
}

/// Route one clock net: source to every sink, not through `taken`.
pub fn route_clock(
    g: &Graph,
    net: &ClockNetwork,
    source: Wire,
    sinks: &[Wire],
    taken: &HashSet<Wire>,
    opts: ClockOptions,
) -> Result<ClockTree> {
    let near_any = |w: Wire| {
        let close = |s: Wire| (w.x - s.x).abs() <= opts.near && (w.y - s.y).abs() <= opts.near;
        close(source) || sinks.iter().any(|&s| close(s))
    };
    let wanted: HashSet<Wire> = sinks.iter().copied().collect();
    // Once a path is on the global network it stays off the fabric
    // interconnect (the H0x/V0x wires). A clock that leaves the spine for
    // fabric wires has a skew that the sinks on the spine do not see, and
    // hold breaks between the two.
    let mut spine: HashSet<Wire> = HashSet::new();
    let mut left = wanted.len();
    let mut found: HashSet<Wire> = HashSet::new();
    let mut dist: HashMap<Wire, u32> = HashMap::from([(source, 0)]);
    let mut parent: HashMap<Wire, ClockArc> = HashMap::new();
    let mut heap = BinaryHeap::from([(Reverse(0u32), source)]);
    let mut steps = Vec::new();
    while let Some((Reverse(d), w)) = heap.pop() {
        if dist.get(&w).is_some_and(|&b| d > b) {
            continue;
        }
        if wanted.contains(&w) && found.insert(w) {
            left -= 1;
            if left == 0 {
                break;
            }
        }
        steps.clear();
        steps.extend(g.downhill(w).into_iter().map(ClockArc::Pip));
        net.virtual_downhill(g, w, &mut steps);
        for &step in &steps {
            let v = step.to();
            if taken.contains(&v) {
                continue;
            }
            // A sink on an ECLKSYNC input is that bel's clock: the net
            // ends there, its output is another net.
            // (The database also has the bel as a fixed ECLKI -> ECLKO pip.)
            if wanted.contains(&w) && g.names[w.name as usize].contains("_ECLKSYNC") {
                continue;
            }
            let global = !matches!(step, ClockArc::Pip(_)) || is_global(g, v) || edge_clock_wire(g, v);
            let on = global || spine.contains(&w);
            if on && !global && fabric_wire(g, v) {
                continue;
            }
            if !on && (!near_any(v) || wanted.contains(&v)) {
                // Before the spine: general wires near the ends only, and
                // never straight into a sink.
                continue;
            }
            let c = d + if global {
                opts.global_cost
            } else {
                opts.general_cost
            };
            if dist.get(&v).is_none_or(|&b| c < b) {
                dist.insert(v, c);
                parent.insert(v, step);
                if on {
                    spine.insert(v);
                } else {
                    spine.remove(&v);
                }
                heap.push((Reverse(c), v));
            }
        }
    }
    if left > 0 {
        let missing: Vec<String> = sinks
            .iter()
            .filter(|s| !found.contains(s))
            .take(4)
            .map(|&s| g.wire_name(s))
            .collect();
        return Err(err!(
            "clock from {}: {left} of {} sinks unreachable (e.g. {})",
            g.wire_name(source),
            sinks.len(),
            missing.join(", ")
        ));
    }
    let mut arcs: Vec<ClockArc> = Vec::new();
    let mut on: HashSet<Wire> = HashSet::from([source]);
    for &s in sinks {
        let mut path = Vec::new();
        let mut x = s;
        while !on.contains(&x) {
            let a = parent[&x];
            path.push(a);
            x = a.from();
        }
        for a in path.into_iter().rev() {
            on.insert(a.to());
            arcs.push(a);
        }
    }
    let stats = ClockStats {
        arcs: arcs.len(),
        spans: arcs
            .iter()
            .filter(|a| matches!(a, ClockArc::Span { .. }))
            .count(),
        dccs: arcs
            .iter()
            .filter(|a| matches!(a, ClockArc::Dcc { .. }))
            .count(),
        visited: dist.len(),
    };
    Ok(ClockTree { arcs, stats })
}

/// The configuration a tree needs: the pips as tile arcs, and each DCC's
/// enable in the tile that has its `DCC_<s><n>.MODE` enum, plus each DCC's
/// clock enable tied high (an unrouted CE leaves the clock gated).
pub fn tree_config(
    g: &Graph,
    db: &DeviceDb,
    tree: &ClockTree,
    tiles: &mut BTreeMap<String, crate::config::TileConfig>,
) -> Result<()> {
    for a in &tree.arcs {
        match *a {
            ClockArc::Pip(arc) => {
                let p = g.pip(&arc);
                if p.kind == crate::graph::PipKind::Configurable {
                    tiles
                        .entry(g.tiles[arc.tile as usize].name.clone())
                        .or_default()
                        .arcs
                        .push((p.sink.clone(), p.source.clone()));
                }
            }
            ClockArc::Span { .. } | ClockArc::Sync { .. } => {}
            ClockArc::RefCs { from, .. } => {
                let sel = g
                    .name_id_of("JSEL_PLLREFCS")
                    .ok_or_else(|| err!("no JSEL_PLLREFCS wire"))?;
                crate::io::tie(
                    g,
                    db,
                    Wire { x: from.x, y: from.y, name: sel },
                    crate::chip::Level::Low,
                    tiles,
                )?;
            }
            ClockArc::Dcc { side, index, .. } => {
                let e = format!("DCC_{side}{index}.MODE");
                let t = g
                    .tiles
                    .iter()
                    .find(|t| db.tile_bits(&t.ty).is_ok_and(|b| b.enums.contains_key(&e)))
                    .ok_or_else(|| err!("no tile has {e}"))?;
                tiles
                    .entry(t.name.clone())
                    .or_default()
                    .enums
                    .push((e, "DCCA".to_owned()));
                let ce: Option<NameId> = g.name_id_of(&format!("G_JCE_{side}DCC{index}"));
                if let Some(ce) = ce {
                    crate::io::tie(
                        g,
                        db,
                        Wire { x: 0, y: 0, name: ce },
                        crate::chip::Level::High,
                        tiles,
                    )?;
                }
            }
        }
    }
    Ok(())
}

/// The edge clock wires (ECLKSYNC, the bank ECLKs, the IOLOGIC ECLK
/// muxes, CLKDIV inputs): clock wires the global name test misses.
fn edge_clock_wire(g: &Graph, w: Wire) -> bool {
    let n = &g.names[w.name as usize];
    n.contains("ECLK") || n.starts_with("CLKI") || n.contains("CLKDIV")
}

/// The fabric interconnect: the H00..H06 and V00..V06 wires, with or
/// without a neighbour prefix (`N1_`, `S3_`).
fn fabric_wire(g: &Graph, w: Wire) -> bool {
    let n = g.names[w.name as usize].as_bytes();
    let n = match n {
        [b'N' | b'S' | b'E' | b'W', d, b'_', rest @ ..] if d.is_ascii_digit() => rest,
        _ => n,
    };
    matches!(n, [b'H' | b'V', b'0', d, ..] if d.is_ascii_digit())
}

/// The left / right I/O banks: column and PIC rows, from `iodb.json`.
fn load_banks(path: &Path) -> Result<BTreeMap<u32, (i16, Vec<i16>)>> {
    let text = std::fs::read_to_string(path).map_err(|e| err!("{}: {e}", path.display()))?;
    let v: serde_json::Value =
        serde_json::from_str(&text).map_err(|e| err!("{}: {e}", path.display()))?;
    let mut banks: BTreeMap<u32, (i16, Vec<i16>)> = BTreeMap::new();
    for p in v["pio_metadata"].as_array().ok_or(err!("iodb.json: pio_metadata"))? {
        let (Some(bank), Some(col), Some(row)) = (p["bank"].as_u64(), p["col"].as_i64(), p["row"].as_i64()) else {
            continue;
        };
        // The top row has no ECLK (the bottom row drops out below: one row).
        if row == 0 {
            continue;
        }
        let (Ok(bank), Ok(col), Ok(row)) = (u32::try_from(bank), i16::try_from(col), i16::try_from(row)) else {
            continue;
        };
        let e = banks.entry(bank).or_insert((col, Vec::new()));
        if e.0 == col && !e.1.contains(&row) {
            e.1.push(row);
        }
    }
    // Only edge banks with one column (left 0, right) keep ECLK rows.
    banks.retain(|_, (_, rows)| rows.len() > 1);
    for (_, rows) in banks.values_mut() {
        rows.sort_unstable();
    }
    Ok(banks)
}
