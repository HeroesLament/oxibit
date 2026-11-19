//! Setup timing of a placed design, and what the router needs of it.
//!
//! A path's delay is clock-to-out, then cell and routing delays in turn,
//! then setup: no clock skew, one corner, which is what nextpnr's "Max
//! frequency" line is made of. Cell delays are `delay`'s table. PS7, MMCM
//! and clock buffer pins are neither start nor end points, and a LUT RAM's
//! clock-to-out is not modelled.
//!
//! - [`Design`]: the timing graph of a placed design (cells, pins, and one
//!   arc per net and sink cell, with the nodes the net may enter that
//!   cell's site by). Built once.
//! - [`Timer`]: the routing delay of every arc and the analysis over them.
//!   `annotate_with` re-times one net's arcs; `analyze` then propagates
//!   arrivals and required times over the whole graph (a few ms: only the
//!   annotation is incremental, which is where the time goes).
//! - [`SlackTable`]: per net slack and arrival, the slowest path of every
//!   clock pair, and the [`CriticalityMap`] the router's cost reads.
//! - [`TimingSource`] and [`TimingView`]: how the router asks. [`Stub`] is
//!   a source with no design or clock behind it, for tests.
//!
//! Slack exists only for a clock with a period in [`Constraints`], and only
//! between registers of that one clock. Criticality is
//! `x / (1 + x)` with `x = max(0, -slack)` in ns: 0 while a net meets its
//! clock, 0.5 at 1 ns short, towards 1 beyond.
//!
//! Checked against nextpnr on its own picoBEAM run (its placement and
//! routing, `FasmRouting` with fixed pins): the same end points on both
//! clocks, 50.1 ns against its reported 52.96 on one and 8.14 against 8.78
//! on the other. The shortfall is routing: the RC terms the intrinsic
//! model leaves out.

use std::cell::RefCell;
use std::collections::{BTreeMap, HashMap, HashSet};
use std::rc::Rc;

use oxibit_fasm::route::{EdgeKind, Graph};
use serde_json::{Value, json};

use crate::delay::{self, CellDelay, CellKind, Model, TreeEdge};
use crate::nets::{self, Edge, Net};
use crate::placed::Import;
use crate::router::Hop;

type R<T> = Result<T, String>;

const NONE: u32 = u32::MAX;
/// No arrival: nothing on the clock in hand reaches the bit.
const UNSET: i64 = i64::MIN;
/// No requirement: the bit reaches no constrained end point.
const FREE: i64 = i64::MAX;

#[derive(Debug, Clone)]
struct Pin {
    port: String,
    bit: u32,
    /// The routed arc into this pin; `NONE` inside one site or on no
    /// routed net.
    arc: u32,
}

#[derive(Debug, Clone)]
struct Cell {
    name: String,
    kind: CellKind,
    /// (tile, site as nextpnr names it).
    site: Option<(String, String)>,
    ins: Vec<Pin>,
    outs: Vec<(String, u32)>,
    /// (port, clock index).
    clocks: Vec<(String, u32)>,
    /// Clocked inputs of a cell that is otherwise combinational: a LUT
    /// RAM's write address, data and enable.
    caps: Vec<Pin>,
    /// A block RAM's output registers, port A and B.
    out_reg: [bool; 2],
}

impl Cell {
    fn seq(&self) -> bool {
        match self.kind {
            CellKind::Ff | CellKind::Bram => true,
            CellKind::Dsp => !self.clocks.is_empty(),
            CellKind::Lut | CellKind::Mux | CellKind::Carry => false,
        }
    }

    /// The clock a pin is launched or captured by.
    fn clock_of(&self, port: &str) -> Option<u32> {
        if self.kind == CellKind::Bram && self.clocks.len() > 1 {
            let a = bram_side_a(port);
            return self.clocks.iter().find(|(p, _)| p.contains("ARD") == a).map(|c| c.1);
        }
        self.clocks.first().map(|c| c.1)
    }

    fn launch(&self, port: &str) -> CellDelay {
        delay::clk_to_q(self.kind, self.kind == CellKind::Bram && self.out_reg[usize::from(!bram_side_a(port))])
    }
}

/// Whether a block RAM pin belongs to port A (else B), by its name.
fn bram_side_a(port: &str) -> bool {
    port.contains("ARD") || ["DOA", "DOPA", "DIA", "DIPA", "WEA", "RSTRAMA", "RSTREGA", "REGCEA"].iter().any(|p| port.starts_with(p))
}

fn is_clock_port(kind: CellKind, port: &str) -> bool {
    match kind {
        CellKind::Ff => matches!(port, "CK" | "C" | "CLK"),
        CellKind::Bram => port.starts_with("CLK"),
        CellKind::Dsp | CellKind::Lut => port == "CLK",
        CellKind::Mux | CellKind::Carry => false,
    }
}

/// A LUT RAM's write-side pins: captured by its clock, no path to its
/// output.
fn is_write_port(kind: CellKind, port: &str) -> bool {
    kind == CellKind::Lut && (port.starts_with("WA") || port.starts_with("DI") || port == "WE")
}

/// One net into one sink cell.
#[derive(Debug, Clone)]
struct Arc {
    net: u32,
    /// The sink cell.
    sink: u32,
    /// The net bit (`Design` numbering), whose driver is the source cell.
    bit: u32,
    /// The nodes the net may enter the cell's site by, each with the site
    /// pips behind it. More than one: a LUT's inputs before the router
    /// chooses, or a net entering one site twice (timed at the slowest).
    cands: Vec<(u32, i32)>,
    /// Site pips between the driver and its site's output pin.
    src_pips: i32,
}

/// The timing graph of a placed design.
#[derive(Debug, Clone, Default)]
pub struct Design {
    cells: Vec<Cell>,
    /// Combinational cells, inputs before outputs.
    order: Vec<u32>,
    /// Net bit -> the cell driving it.
    driver: Vec<u32>,
    clocks: Vec<String>,
    arcs: Vec<Arc>,
    /// Import net -> its arcs.
    net_arcs: Vec<Vec<u32>>,
    /// Import net -> its bit here.
    net_bit: Vec<u32>,
    /// Cell types left out, with counts.
    pub ignored: BTreeMap<String, usize>,
    /// Combinational cells on or behind a loop: not timed.
    pub in_loops: usize,
    pub unplaced: usize,
}

impl Design {
    /// The timing graph of `placed`, whose import is `imp` (`sites`:
    /// `placed::site_index`).
    pub fn build(g: &Graph, sites: &HashMap<String, (String, String)>, placed: &Value, imp: &Import) -> R<Design> {
        let mut tile_at: HashMap<(i32, i32), String> = HashMap::new();
        for t in g.tile_names() {
            if let Some(xy) = g.tile_xy(t) {
                tile_at.insert(xy, t.to_string());
            }
        }
        // `X14Y142/SLICE_X0Y0.A6LUT`, or `{absolute site}/{bel}`.
        let site_of = |bel: &str| -> Option<(String, String)> {
            let (head, rest) = bel.split_once('/')?;
            let grid = head.strip_prefix('X').and_then(|t| t.split_once('Y')).and_then(|(x, y)| Some((x.parse().ok()?, y.parse().ok()?)));
            match grid {
                Some(xy) => Some((tile_at.get(&xy)?.clone(), rest.split_once('.')?.0.to_string())),
                None => sites.get(head).cloned(),
            }
        };
        let mods = placed["modules"].as_object().ok_or("no modules")?;
        let top = mods.values().find(|m| m["attributes"]["top"].is_string() || m["attributes"]["top"].is_number()).or_else(|| mods.values().next()).ok_or("no top module")?;
        let cells_v = top["cells"].as_object().ok_or("no cells")?;
        let bits_of = |v: &Value| -> Vec<i64> { v.as_array().into_iter().flatten().filter_map(Value::as_i64).collect() };
        // The constant nets: a clock pin on one is no clock (an
        // unregistered DSP48E1's).
        let consts: HashSet<i64> = cells_v
            .values()
            .filter(|c| matches!(c["type"].as_str(), Some("PSEUDO_GND" | "PSEUDO_VCC")))
            .flat_map(|c| c["connections"].as_object().into_iter().flatten().flat_map(|(_, v)| bits_of(v)))
            .collect();
        let one = |v: &Value| v.as_str().map(|s| s.trim().ends_with('1')).or_else(|| v.as_i64().map(|n| n == 1)).unwrap_or(false);

        let net_of_bit: HashMap<i64, usize> = imp.bits.iter().enumerate().map(|(i, &b)| (b, i)).collect();
        let mut bit_id: HashMap<i64, u32> = HashMap::new();
        let mut clock_id: HashMap<i64, u32> = HashMap::new();
        let mut clock_bits: Vec<i64> = Vec::new();
        let mut arc_of: HashMap<(i64, usize), u32> = HashMap::new();
        let mut d = Design { net_arcs: vec![Vec::new(); imp.nets.len()], ..Default::default() };

        for (name, c) in cells_v {
            let ty = c["type"].as_str().unwrap_or("");
            let kind = match ty {
                "SLICE_LUTX" => CellKind::Lut,
                "SLICE_FFX" => CellKind::Ff,
                "SELMUX2_1" => CellKind::Mux,
                "CARRY4" => CellKind::Carry,
                t if t.starts_with("RAMB") => CellKind::Bram,
                t if t.starts_with("DSP48E1") => CellKind::Dsp,
                _ => {
                    *d.ignored.entry(ty.to_string()).or_default() += 1;
                    continue;
                }
            };
            let ci = d.cells.len();
            let site = c["attributes"]["NEXTPNR_BEL"].as_str().and_then(site_of);
            if site.is_none() {
                d.unplaced += 1;
            }
            let params = &c["parameters"];
            let mut cell = Cell {
                name: name.clone(),
                kind,
                site,
                ins: vec![],
                outs: vec![],
                clocks: vec![],
                caps: vec![],
                out_reg: [one(&params["DOA_REG"]), one(&params["DOB_REG"])],
            };
            // A fractured LUT's cell also carries its partner's nets, on
            // the pins the two share: the cell's own inputs are the pins
            // with an original port (when a packer records them at all).
            let attrs = c["attributes"].as_object();
            let orig = |p: &str| attrs.is_some_and(|a| a.contains_key(&format!("X_ORIG_PORT_{p}")));
            let records_orig = kind == CellKind::Lut && attrs.is_some_and(|a| a.keys().any(|k| k.starts_with("X_ORIG_PORT_A")));
            for (p, dir) in c["port_directions"].as_object().into_iter().flatten() {
                let output = dir.as_str() == Some("output");
                if records_orig && !output && p.starts_with('A') && !orig(p) {
                    continue;
                }
                for b in bits_of(&c["connections"][p]) {
                    let n = bit_id.len() as u32;
                    let id = *bit_id.entry(b).or_insert(n);
                    if output {
                        cell.outs.push((p.clone(), id));
                        continue;
                    }
                    if consts.contains(&b) {
                        continue;
                    }
                    if is_clock_port(kind, p) {
                        let k = *clock_id.entry(b).or_insert_with(|| {
                            clock_bits.push(b);
                            clock_bits.len() as u32 - 1
                        });
                        cell.clocks.push((p.clone(), k));
                        continue;
                    }
                    let arc = match arc_of.get(&(b, ci)) {
                        Some(&a) => a,
                        None => {
                            let a = match (net_of_bit.get(&b), &cell.site) {
                                (Some(&ni), Some((tile, site))) => {
                                    let pips = |wires: &[String]| wires.len().saturating_sub(1) as i32;
                                    let cands: Vec<(u32, i32)> =
                                        imp.sink_paths[ni].iter().filter(|(_, sp)| &sp.tile == tile && &sp.np_site == site).map(|(n, sp)| (*n, pips(&sp.site_wires))).collect();
                                    if cands.is_empty() {
                                        NONE
                                    } else {
                                        d.arcs.push(Arc { net: ni as u32, sink: ci as u32, bit: id, cands, src_pips: pips(&imp.sources[ni].site_wires) });
                                        d.net_arcs[ni].push(d.arcs.len() as u32 - 1);
                                        d.arcs.len() as u32 - 1
                                    }
                                }
                                _ => NONE,
                            };
                            arc_of.insert((b, ci), a);
                            a
                        }
                    };
                    let pin = Pin { port: p.clone(), bit: id, arc };
                    if is_write_port(kind, p) {
                        cell.caps.push(pin);
                    } else {
                        cell.ins.push(pin);
                    }
                }
            }
            d.cells.push(cell);
        }

        for (n, v) in top["netnames"].as_object().into_iter().flatten() {
            let Some(&k) = v["bits"].get(0).and_then(Value::as_i64).and_then(|b| clock_id.get(&b)) else { continue };
            if d.clocks.is_empty() {
                d.clocks = vec![String::new(); clock_bits.len()];
            }
            let e = &mut d.clocks[k as usize];
            if e.is_empty() || (n.contains('$'), n.len()) < (e.contains('$'), e.len()) {
                *e = n.clone();
            }
        }
        d.clocks.resize(clock_bits.len(), String::new());
        for (k, name) in d.clocks.iter_mut().enumerate().filter(|(_, n)| n.is_empty()) {
            *name = format!("bit{}", clock_bits[k]);
        }
        d.net_bit = imp.bits.iter().map(|b| bit_id.get(b).copied().unwrap_or(NONE)).collect();
        d.finish(bit_id.len());
        Ok(d)
    }

    /// Drivers and the combinational order, once the cells are in.
    fn finish(&mut self, bits: usize) {
        self.driver = vec![NONE; bits];
        for (i, c) in self.cells.iter().enumerate() {
            for o in &c.outs {
                self.driver[o.1 as usize] = i as u32;
            }
        }
        let comb_driven = |b: u32| self.driver[b as usize] != NONE && !self.cells[self.driver[b as usize] as usize].seq();
        let mut indeg: Vec<usize> = self.cells.iter().map(|c| if c.seq() { 0 } else { c.ins.iter().filter(|p| comb_driven(p.bit)).count() }).collect();
        let mut users: Vec<Vec<u32>> = vec![Vec::new(); bits];
        for (i, c) in self.cells.iter().enumerate().filter(|(_, c)| !c.seq()) {
            for p in &c.ins {
                users[p.bit as usize].push(i as u32);
            }
        }
        let mut stack: Vec<u32> = (0..self.cells.len()).filter(|&i| !self.cells[i].seq() && indeg[i] == 0).map(|i| i as u32).collect();
        let mut order = Vec::new();
        while let Some(c) = stack.pop() {
            order.push(c);
            for o in &self.cells[c as usize].outs {
                for &u in &users[o.1 as usize] {
                    indeg[u as usize] -= 1;
                    if indeg[u as usize] == 0 {
                        stack.push(u);
                    }
                }
            }
        }
        self.in_loops = self.cells.iter().filter(|c| !c.seq()).count() - order.len();
        self.order = order;
    }

    pub fn clocks(&self) -> &[String] {
        &self.clocks
    }

    /// A timing graph with no routing behind it, for a placer: `cells` in
    /// any order, pins by net bit (`0..bits`), `net_of_bit` the net (in
    /// `0..nets`) whose criticality a bit's arcs count towards. Every
    /// (bit, sink cell) pair a net reaches is an arc, delay 0 until
    /// `Timer::set_arc_ps`. Cell `i` of the result is `cells[i]`.
    pub fn from_cells(cells: Vec<CellSpec>, bits: usize, nets: usize, net_of_bit: &dyn Fn(u32) -> Option<u32>) -> Design {
        let mut d = Design { net_arcs: vec![Vec::new(); nets], ..Default::default() };
        let mut clock_id: HashMap<u32, u32> = HashMap::new();
        let mut arc_of: HashMap<(u32, usize), u32> = HashMap::new();
        for (ci, spec) in cells.into_iter().enumerate() {
            let mut cell = Cell { name: spec.name, kind: spec.kind, site: None, ins: vec![], outs: vec![], clocks: vec![], caps: vec![], out_reg: spec.out_reg };
            for (port, output, bit) in spec.pins {
                if output {
                    cell.outs.push((port, bit));
                    continue;
                }
                if is_clock_port(cell.kind, &port) {
                    let n = clock_id.len() as u32;
                    let k = *clock_id.entry(bit).or_insert_with(|| {
                        d.clocks.push(format!("bit{bit}"));
                        n
                    });
                    cell.clocks.push((port, k));
                    continue;
                }
                let arc = match (net_of_bit(bit), arc_of.get(&(bit, ci))) {
                    (_, Some(&a)) => a,
                    (Some(net), None) => {
                        d.arcs.push(Arc { net, sink: ci as u32, bit, cands: Vec::new(), src_pips: 0 });
                        let a = d.arcs.len() as u32 - 1;
                        d.net_arcs[net as usize].push(a);
                        arc_of.insert((bit, ci), a);
                        a
                    }
                    (None, None) => NONE,
                };
                let pin = Pin { port: port.clone(), bit, arc };
                if is_write_port(cell.kind, &port) {
                    cell.caps.push(pin);
                } else {
                    cell.ins.push(pin);
                }
            }
            d.cells.push(cell);
        }
        d.net_bit = vec![NONE; nets];
        for a in &d.arcs {
            d.net_bit[a.net as usize] = a.bit;
        }
        d.finish(bits);
        d
    }

    /// Arcs, in `Timer::set_arc_ps` numbering.
    pub fn arcs_len(&self) -> usize {
        self.arcs.len()
    }

    /// An arc's driver cell (if the bit has one) and sink cell, in
    /// `from_cells` order.
    pub fn arc_cells(&self, a: usize) -> (Option<usize>, usize) {
        let arc = &self.arcs[a];
        let drv = self.driver.get(arc.bit as usize).copied().filter(|&c| c != NONE).map(|c| c as usize);
        (drv, arc.sink as usize)
    }

    pub fn cell_kind(&self, c: usize) -> CellKind {
        self.cells[c].kind
    }
}

/// A cell for `Design::from_cells`.
#[derive(Debug, Clone)]
pub struct CellSpec {
    pub name: String,
    pub kind: CellKind,
    /// (port, is an output, net bit). Leave constant nets out.
    pub pins: Vec<(String, bool, u32)>,
    /// A block RAM's output registers, port A and B.
    pub out_reg: [bool; 2],
}

impl CellSpec {
    /// The kind of a cell type as the packer writes it; `None` for a cell
    /// the analysis leaves out. CARRY4 is still left out here (the
    /// placer's timing weights): timing it there changes every placement
    /// with carry chains, a change of its own to measure.
    pub fn kind_of(ty: &str) -> Option<CellKind> {
        match ty {
            "SLICE_LUTX" => Some(CellKind::Lut),
            "SLICE_FFX" => Some(CellKind::Ff),
            "SELMUX2_1" => Some(CellKind::Mux),
            t if t.starts_with("RAMB") => Some(CellKind::Bram),
            t if t.starts_with("DSP48E1") => Some(CellKind::Dsp),
            _ => None,
        }
    }
}

impl Design {
    /// Cells by kind.
    pub fn cell_counts(&self) -> BTreeMap<String, usize> {
        let mut out = BTreeMap::new();
        for c in &self.cells {
            *out.entry(format!("{:?}", c.kind).to_lowercase()).or_default() += 1;
        }
        out
    }
}

/// Clock periods to meet, in ps.
#[derive(Debug, Clone, Default)]
pub struct Constraints {
    /// For every clock not named.
    pub default_ps: Option<i64>,
    pub clocks: Vec<(String, i64)>,
}

impl Constraints {
    /// `pb_clk=50000,axi_clk=8000`; `*=20000` sets the default.
    pub fn parse(s: &str) -> R<Self> {
        let mut c = Constraints::default();
        for item in s.split(',').map(str::trim).filter(|i| !i.is_empty()) {
            let (name, ps) = item.split_once('=').ok_or_else(|| format!("{item}: want CLOCK=PS"))?;
            let ps: i64 = ps.trim().parse().map_err(|_| format!("{item}: period not a number of ps"))?;
            match name.trim() {
                "*" => c.default_ps = Some(ps),
                n => c.clocks.push((n.to_string(), ps)),
            }
        }
        Ok(c)
    }

    pub fn period(&self, clock: &str) -> Option<i64> {
        self.clocks.iter().find(|(n, _)| n == clock).map(|c| c.1).or(self.default_ps)
    }
}

/// `x / (1 + x)`, `x = max(0, -slack)` in ns.
pub fn criticality(slack_ps: i64) -> f32 {
    let x = (-slack_ps).max(0) as f32 / 1000.0;
    x / (1.0 + x)
}

/// How critical each net is, and each of its sinks.
#[derive(Debug, Clone, Default)]
pub struct CriticalityMap {
    net: Vec<f32>,
    /// Per net, the sink nodes with a criticality above 0, by node.
    sink: Vec<Vec<(u32, f32)>>,
}

impl CriticalityMap {
    /// Per-net values alone: every sink of a net is as critical as the net.
    pub fn from_nets(net: Vec<f32>) -> Self {
        CriticalityMap { sink: vec![Vec::new(); net.len()], net }
    }

    /// A net's own: that of its worst sink. 0 for a net the analysis does
    /// not know (a constant net, a clock).
    pub fn of_net(&self, net: usize) -> f32 {
        self.net.get(net).copied().unwrap_or(0.0)
    }

    /// That of the sink any of `nodes` completes. Where sinks are told
    /// apart, a sink not listed is not critical; otherwise the net's own.
    pub fn of_sink(&self, net: usize, nodes: &[u32]) -> f32 {
        match self.sink.get(net) {
            Some(s) if !s.is_empty() => {
                nodes.iter().filter_map(|n| s.binary_search_by_key(n, |e| e.0).ok().map(|i| s[i].1)).fold(0.0, f32::max)
            }
            _ => self.of_net(net),
        }
    }

    /// The nodes of a net's critical sinks (every candidate of each).
    pub fn hot_sinks(&self, net: usize) -> impl Iterator<Item = u32> + '_ {
        self.sink.get(net).into_iter().flatten().map(|e| e.0)
    }

    /// Nets with a criticality above 0.
    pub fn failing(&self) -> usize {
        self.net.iter().filter(|&&c| c > 0.0).count()
    }
}

/// What `Timer::floor` holds the arcs to: the two idealised ends of the
/// Sypher number's chain (`docs/sypher.md`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Floor {
    /// The netlist alone: a connection that could stay inside a slice does.
    Netlist,
    /// The packing as placed: only a connection inside one site does.
    Packed,
}

/// What one `Timer::floor` did, in arcs.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct FloorCounts {
    /// Held to 0: inside a slice.
    pub in_slice: usize,
    /// Left as routed: carry to carry, on dedicated wires.
    pub dedicated: usize,
    /// Held to one minimal hop.
    pub routed: usize,
    /// Arcs whose delay fell.
    pub lowered: usize,
}

/// One timed arc of an analysis (`Timer::arcs`).
#[derive(Debug, Clone, PartialEq)]
pub struct TimedArc {
    pub from: String,
    pub to: String,
    /// Its delay as analysed.
    pub ps: i64,
    /// Sink tile minus driver tile, (x, y); `None` unless both are placed.
    pub delta: Option<(i32, i32)>,
    pub from_kind: CellKind,
    pub to_kind: CellKind,
}

/// The slowest path between the registers of a clock pair.
#[derive(Debug, Clone)]
pub struct PathReport {
    pub from_clk: String,
    pub to_clk: String,
    pub delay_ps: i64,
    pub logic_ps: i64,
    pub route_ps: i64,
    pub levels: usize,
    pub from: String,
    pub to: String,
    pub from_kind: CellKind,
    pub to_kind: CellKind,
    /// The assumed cell delays the path uses.
    pub assumed: Vec<String>,
    /// Launch to capture.
    pub steps: Vec<Value>,
}

impl PathReport {
    pub fn fmax_mhz(&self) -> Option<f64> {
        (self.from_clk == self.to_clk && self.delay_ps > 0).then(|| (1e8 / self.delay_ps as f64).round() / 100.0)
    }

    pub fn to_json(&self, steps: bool) -> Value {
        let mut v = json!({"from_clk": self.from_clk, "to_clk": self.to_clk, "delay_ns": self.delay_ps as f64 / 1000.0,
                           "logic_ns": self.logic_ps as f64 / 1000.0, "routing_ns": self.route_ps as f64 / 1000.0, "levels": self.levels,
                           "from": self.from, "from_kind": format!("{:?}", self.from_kind), "to": self.to, "to_kind": format!("{:?}", self.to_kind),
                           "assumed": self.assumed});
        if let Some(f) = self.fmax_mhz() {
            v["fmax_mhz"] = json!(f);
        }
        if steps {
            v["steps"] = json!(self.steps);
        }
        v
    }
}

/// A clock's tightest hold check (`Timer::hold`).
#[derive(Debug, Clone)]
pub struct HoldReport {
    pub clock: String,
    /// Earliest arrival less the hold requirement; no skew.
    pub slack_ps: i64,
    pub arrival_ps: i64,
    pub hold_ps: i64,
    pub from: String,
    pub from_kind: CellKind,
    pub to: String,
    pub to_pin: String,
    pub to_kind: CellKind,
    /// Combinational cells on the path.
    pub cells: usize,
    /// The hold requirement is a placeholder (`delay::hold`).
    pub assumed: bool,
    /// Hold checks on this clock, and how many have less slack than the
    /// margin asked for.
    pub checked: usize,
    pub under_margin: usize,
}

impl HoldReport {
    pub fn to_json(&self) -> Value {
        json!({"clock": self.clock, "slack_ns": self.slack_ps as f64 / 1000.0, "arrival_ns": self.arrival_ps as f64 / 1000.0,
               "hold_ns": self.hold_ps as f64 / 1000.0, "from": self.from, "from_kind": format!("{:?}", self.from_kind),
               "to": self.to, "to_pin": self.to_pin, "to_kind": format!("{:?}", self.to_kind), "cells": self.cells,
               "assumed": self.assumed, "checked": self.checked, "under_margin": self.under_margin})
    }
}

/// One analysis.
#[derive(Debug, Clone, Default)]
pub struct SlackTable {
    /// Per import net, ps; `None`: unconstrained.
    pub net_slack: Vec<Option<i64>>,
    /// Per import net, the latest arrival at its driver, ps.
    pub net_arrival: Vec<Option<i64>>,
    /// Slowest path of every clock pair, slowest first.
    pub paths: Vec<PathReport>,
    /// Per net, the slowest path through it over its clock's slowest
    /// path, 0..=1, the worst over clocks with a period (VPR's
    /// criticality, against the current worst path rather than the
    /// period: a path does not have to fail to count). 0 where no
    /// constrained path runs.
    pub net_rel: Vec<f32>,
    pub endpoints: usize,
    crit: CriticalityMap,
}

impl SlackTable {
    /// A table that is nothing but criticalities (`Stub`).
    pub fn from_criticality(crit: CriticalityMap) -> Self {
        SlackTable { crit, ..Default::default() }
    }

    pub fn criticality(&self) -> &CriticalityMap {
        &self.crit
    }

    pub fn worst_slack(&self) -> Option<i64> {
        self.net_slack.iter().flatten().copied().min()
    }

    /// The slowest path launched and captured on `clock`.
    pub fn path(&self, clock: &str) -> Option<&PathReport> {
        self.paths.iter().find(|p| p.from_clk == clock && p.to_clk == clock)
    }

    /// The numbers a progress line wants.
    pub fn summary(&self) -> Value {
        let same: Vec<Value> = self.paths.iter().filter(|p| p.from_clk == p.to_clk).map(|p| json!([p.from_clk, p.delay_ps as f64 / 1000.0])).collect();
        json!({"worst_slack_ns": self.worst_slack().map(|s| s as f64 / 1000.0), "failing_nets": self.crit.failing(), "worst_path_ns": same})
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ArcState {
    /// Never annotated: delay 0.
    Unknown,
    Timed,
    /// Annotated, but the routing reaches none of its nodes: delay 0.
    Unreached,
}

/// Arcs by state, for a report.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ArcCounts {
    pub timed: usize,
    pub unknown: usize,
    pub unreached: usize,
}

/// Routing delays of a design's arcs, and the analysis over them.
pub struct Timer {
    d: Design,
    model: Model,
    /// ps per site pip (nextpnr: 20).
    pub site_pip: i64,
    period: Vec<Option<i64>>,
    arc_ps: Vec<i64>,
    arc_state: Vec<ArcState>,
    table: SlackTable,
}

impl Timer {
    pub fn new(d: Design, cons: &Constraints, model: Model) -> Self {
        let period = d.clocks.iter().map(|c| cons.period(c)).collect();
        let n = d.arcs.len();
        Timer { d, model, site_pip: 20, period, arc_ps: vec![0; n], arc_state: vec![ArcState::Unknown; n], table: SlackTable::default() }
    }

    pub fn design(&self) -> &Design {
        &self.d
    }

    pub fn model(&self) -> Model {
        self.model
    }

    /// Set one arc's delay directly (a placer's estimate).
    pub fn set_arc_ps(&mut self, arc: usize, ps: i64) {
        self.arc_ps[arc] = ps;
        self.arc_state[arc] = ArcState::Timed;
    }

    /// Hold every timed arc to its floor: the delay it would have were
    /// nothing lost to placement or routing. 0 for a LUT, mux or carry
    /// into a flop, mux or carry (at `Floor::Packed`, only where both sit
    /// in one site); its routed delay for carry to carry, whose wires
    /// are dedicated; `w0_ps`, one minimal routed hop, for the rest. An
    /// arc is never raised, so the path delays of routed, `Packed` and
    /// `Netlist` analyses fall in that order. `analyze` afterwards times
    /// the floor; the routed delays are gone from this timer.
    pub fn floor(&mut self, level: Floor, w0_ps: i64) -> FloorCounts {
        use CellKind::{Carry, Ff, Lut, Mux};
        let mut n = FloorCounts::default();
        for (a, arc) in self.d.arcs.iter().enumerate() {
            if self.arc_state[a] != ArcState::Timed {
                continue;
            }
            let sink = &self.d.cells[arc.sink as usize];
            let drv = self.d.driver.get(arc.bit as usize).copied().filter(|&c| c != NONE).map(|c| &self.d.cells[c as usize]);
            let dedicated = drv.is_some_and(|c| c.kind == Carry) && sink.kind == Carry;
            let could = drv.is_some_and(|c| matches!(c.kind, Lut | Mux | Carry)) && matches!(sink.kind, Ff | Mux | Carry);
            let together = drv.is_some_and(|c| c.site.is_some() && c.site == sink.site);
            let floor = if dedicated {
                n.dedicated += 1;
                self.arc_ps[a]
            } else if could && (level == Floor::Netlist || together) {
                n.in_slice += 1;
                0
            } else {
                n.routed += 1;
                w0_ps
            };
            if floor < self.arc_ps[a] {
                self.arc_ps[a] = floor;
                n.lowered += 1;
            }
        }
        n
    }

    /// Re-time the arcs of import net `net`: `arrival(node)` is the delay
    /// (ps) from the net's source to a node its routing reaches.
    pub fn annotate_with(&mut self, net: usize, arrival: &mut dyn FnMut(u32) -> Option<u64>) {
        let Some(arcs) = self.d.net_arcs.get(net) else { return };
        for &a in arcs {
            let arc = &self.d.arcs[a as usize];
            let best = arc.cands.iter().filter_map(|&(n, p)| arrival(n).map(|t| t as i64 + self.site_pip * p as i64)).max();
            (self.arc_ps[a as usize], self.arc_state[a as usize]) = match best {
                Some(t) => (t + self.site_pip * arc.src_pips as i64, ArcState::Timed),
                None => (0, ArcState::Unreached),
            };
        }
    }

    pub fn arc_counts(&self) -> ArcCounts {
        let n = |s| self.arc_state.iter().filter(|&&x| x == s).count();
        ArcCounts { timed: n(ArcState::Timed), unknown: n(ArcState::Unknown), unreached: n(ArcState::Unreached) }
    }

    /// The arcs of `net` its last annotation did not reach, as node lists.
    fn unreached(&self, net: usize) -> Vec<(u32, Vec<(u32, i32)>)> {
        let arcs = self.d.net_arcs.get(net).map(Vec::as_slice).unwrap_or(&[]);
        arcs.iter().filter(|&&a| self.arc_state[a as usize] == ArcState::Unreached).map(|&a| (a, self.d.arcs[a as usize].cands.clone())).collect()
    }

    pub fn table(&self) -> &SlackTable {
        &self.table
    }

    /// Every timed arc: (driver cell, sink cell, ps, the tile distance
    /// between their sites when `xy` knows both). For comparing two
    /// routings of one placement arc by arc.
    pub fn arcs(&self, xy: &dyn Fn(&str) -> Option<(i32, i32)>) -> Vec<TimedArc> {
        let d = &self.d;
        let at = |c: &Cell| c.site.as_ref().and_then(|s| xy(&s.0));
        let mut seen = vec![false; d.arcs.len()];
        let mut out = Vec::new();
        for c in &d.cells {
            for pin in c.ins.iter().chain(&c.caps) {
                if pin.arc == NONE || self.arc_state[pin.arc as usize] != ArcState::Timed || std::mem::replace(&mut seen[pin.arc as usize], true) {
                    continue;
                }
                let drv = d.driver[pin.bit as usize];
                if drv == NONE {
                    continue;
                }
                let from = &d.cells[drv as usize];
                let delta = at(from).zip(at(c)).map(|(a, b)| (b.0 - a.0, b.1 - a.1));
                out.push(TimedArc {
                    from: from.name.clone(),
                    to: c.name.clone(),
                    ps: self.arc_ps[pin.arc as usize],
                    delta,
                    from_kind: from.kind,
                    to_kind: c.kind,
                });
            }
        }
        out
    }

    /// The arc from cell `driver` into cell `sink`: its import net and
    /// the nodes it may enter the sink's site by.
    pub fn find_arc(&self, driver: &str, sink: &str) -> Option<(usize, Vec<u32>)> {
        let d = &self.d;
        let c = d.cells.iter().find(|c| c.name == sink)?;
        let pin = c.ins.iter().chain(&c.caps).find(|p| p.arc != NONE && d.driver[p.bit as usize] != NONE && d.cells[d.driver[p.bit as usize] as usize].name == driver)?;
        let arc = &d.arcs[pin.arc as usize];
        Some((arc.net as usize, arc.cands.iter().map(|c| c.0).collect()))
    }

    /// The tightest same-clock hold check of every clock, fast corner:
    /// earliest arrivals forward from the clock's registers
    /// (`delay::clk_to_q_min`, `delay::comb_min`, each arc's routing delay
    /// times `route_scale`) against `delay::hold` at every capture on the
    /// same clock. No clock skew is modelled: the slack is the skew
    /// between launch and capture the check can absorb. Paths between two
    /// clocks are not checked (how their edges relate is not modelled).
    /// `margin_ps`: the report counts the checks with less slack than it.
    pub fn hold(&self, route_scale: f64, margin_ps: i64) -> Vec<HoldReport> {
        let d = &self.d;
        let route = |a: u32| if a == NONE { 0 } else { (self.arc_ps[a as usize] as f64 * route_scale).round() as i64 };
        let mut out = Vec::new();
        for k in 0..d.clocks.len() as u32 {
            let arr = self.hold_arrivals(k, route_scale);
            let mut worst: Option<HoldReport> = None;
            let (mut checked, mut under) = (0usize, 0usize);
            for c in &d.cells {
                let pins = if c.seq() { &c.ins } else { &c.caps };
                for pin in pins.iter().filter(|p| c.clock_of(&p.port) == Some(k)) {
                    let a = arr[pin.bit as usize];
                    if a.0 == i64::MAX {
                        continue;
                    }
                    let h = delay::hold_pin(c.kind, &pin.port);
                    let at = a.0 + route(pin.arc);
                    let slack = at - h.ps;
                    checked += 1;
                    under += usize::from(slack < margin_ps);
                    if slack < 0 && std::env::var_os("OXIBIT_HOLD_DUMP").is_some() { eprintln!("oxibit: hold: {} {:?} {} {} <- {} {:?}", slack, c.kind, c.name, pin.port, d.cells[a.1 as usize].name, d.cells[a.1 as usize].kind); }
                    if worst.as_ref().is_none_or(|w| slack < w.slack_ps) {
                        let from = &d.cells[a.1 as usize];
                        worst = Some(HoldReport {
                            clock: d.clocks[k as usize].clone(),
                            slack_ps: slack,
                            arrival_ps: at,
                            hold_ps: h.ps,
                            from: from.name.clone(),
                            from_kind: from.kind,
                            to: c.name.clone(),
                            to_pin: pin.port.clone(),
                            to_kind: c.kind,
                            cells: a.2,
                            assumed: h.assumed,
                            checked: 0,
                            under_margin: 0,
                        });
                    }
                }
            }
            if let Some(mut w) = worst {
                (w.checked, w.under_margin) = (checked, under);
                out.push(w);
            }
        }
        out
    }

    /// Fast-corner earliest arrivals of clock `k`, per bit: the arrival,
    /// the register it left, cells since (`hold`).
    fn hold_arrivals(&self, k: u32, route_scale: f64) -> Vec<(i64, u32, usize)> {
        let d = &self.d;
        let route = |a: u32| if a == NONE { 0 } else { (self.arc_ps[a as usize] as f64 * route_scale).round() as i64 };
        let mut arr: Vec<(i64, u32, usize)> = vec![(i64::MAX, NONE, 0); d.driver.len()];
        for (ci, c) in d.cells.iter().enumerate().filter(|(_, c)| c.seq()) {
            for (p, b) in &c.outs {
                let t = delay::clk_to_q_min(c.kind).ps;
                if c.clock_of(p) == Some(k) && t < arr[*b as usize].0 {
                    arr[*b as usize] = (t, ci as u32, 0);
                }
            }
        }
        for &ci in &d.order {
            let c = &d.cells[ci as usize];
            let through = delay::comb_min(c.kind).ps;
            let best = c
                .ins
                .iter()
                .map(|p| (arr[p.bit as usize], p.arc))
                .filter(|(a, _)| a.0 != i64::MAX)
                .map(|(a, arc)| (a.0 + route(arc) + through, a.1, a.2 + 1))
                .min_by_key(|a| (a.0, a.1));
            if let Some(best) = best {
                for (_, ob) in &c.outs {
                    if best.0 < arr[*ob as usize].0 {
                        arr[*ob as usize] = best;
                    }
                }
            }
        }
        arr
    }

    /// Every capture of a cell of `kinds` whose hold check (`hold`) has
    /// less slack than `margin_ps`, as the routed arc into the pin and the
    /// slow-corner routing delay that arc must gain to clear the margin.
    pub fn hold_shorts(&self, route_scale: f64, margin_ps: i64, kinds: &[CellKind]) -> Vec<HoldShort> {
        let d = &self.d;
        let route = |a: u32| (self.arc_ps[a as usize] as f64 * route_scale).round() as i64;
        let mut out = Vec::new();
        for k in 0..d.clocks.len() as u32 {
            let arr = self.hold_arrivals(k, route_scale);
            for c in d.cells.iter().filter(|c| kinds.contains(&c.kind)) {
                let pins = if c.seq() { &c.ins } else { &c.caps };
                for pin in pins.iter().filter(|p| p.arc != NONE && c.clock_of(&p.port) == Some(k)) {
                    let a = arr[pin.bit as usize];
                    if a.0 == i64::MAX || self.arc_state[pin.arc as usize] != ArcState::Timed {
                        continue;
                    }
                    let slack = a.0 + route(pin.arc) - delay::hold_pin(c.kind, &pin.port).ps;
                    if slack < margin_ps {
                        let arc = &d.arcs[pin.arc as usize];
                        out.push(HoldShort {
                            net: arc.net as usize,
                            nodes: arc.cands.iter().map(|c| c.0).collect(),
                            add_ps: ((margin_ps - slack) as f64 / route_scale).ceil() as i64,
                            slack_ps: slack,
                        });
                    }
                }
            }
        }
        out
    }

    /// Arrivals forward from every clock's registers, required times back
    /// from the registers of a clock with a period, over the arc delays as
    /// they stand.
    pub fn analyze(&mut self) -> &SlackTable {
        let d = &self.d;
        let bits = d.driver.len();
        let arc_ps = |a: u32| if a == NONE { 0 } else { self.arc_ps[a as usize] };
        let mut net_slack = vec![FREE; d.net_bit.len()];
        let mut net_rel = vec![0.0f32; d.net_bit.len()];
        let mut net_arrival = vec![UNSET; d.net_bit.len()];
        let mut arc_slack = vec![FREE; d.arcs.len()];
        let mut paths: Vec<PathReport> = Vec::new();
        let mut endpoints = 0usize;

        for k in 0..d.clocks.len() as u32 {
            let mut arr = vec![UNSET; bits];
            // The (cell, input pin) each arrival came through.
            let mut pred: Vec<(u32, u32)> = vec![(NONE, 0); bits];
            for c in d.cells.iter().filter(|c| c.seq()) {
                for (p, b) in &c.outs {
                    if c.clock_of(p) == Some(k) {
                        arr[*b as usize] = arr[*b as usize].max(c.launch(p).ps);
                    }
                }
            }
            for &ci in &d.order {
                let c = &d.cells[ci as usize];
                for (op, ob) in &c.outs {
                    let mut best = (UNSET, 0u32);
                    for (pi, pin) in c.ins.iter().enumerate() {
                        let t = arr[pin.bit as usize];
                        if t == UNSET || !delay::has_arc(c.kind, &pin.port, op) {
                            continue;
                        }
                        let v = t + arc_ps(pin.arc) + delay::comb(c.kind, &pin.port, op).ps;
                        if v > best.0 {
                            best = (v, pi as u32);
                        }
                    }
                    if best.0 != UNSET {
                        arr[*ob as usize] = best.0;
                        pred[*ob as usize] = (ci, best.1);
                    }
                }
            }

            // End points: the slowest capture of each clock, and (on this
            // clock, with a period) where required times start.
            let period = self.period[k as usize];
            let mut req = vec![FREE; if period.is_some() { bits } else { 0 }];
            let mut worst: BTreeMap<u32, (i64, usize, bool, usize)> = BTreeMap::new();
            for (ci, c) in d.cells.iter().enumerate() {
                let (pins, caps) = if c.seq() { (&c.ins, false) } else { (&c.caps, true) };
                for (pi, pin) in pins.iter().enumerate() {
                    let Some(k2) = c.clock_of(&pin.port) else { continue };
                    let su = delay::setup(c.kind, &pin.port).ps;
                    let a = arc_ps(pin.arc);
                    if let (Some(t), true) = (period, k2 == k) {
                        let r = &mut req[pin.bit as usize];
                        *r = (*r).min(t - su - a);
                    }
                    let t = arr[pin.bit as usize];
                    if t == UNSET {
                        continue;
                    }
                    endpoints += 1;
                    if let (Some(period), true, true) = (period, k2 == k, pin.arc != NONE) {
                        let s = &mut arc_slack[pin.arc as usize];
                        *s = (*s).min(period - su - a - t);
                    }
                    if worst.get(&k2).is_none_or(|w| t + a + su > w.0) {
                        worst.insert(k2, (t + a + su, ci, caps, pi));
                    }
                }
            }
            if period.is_some() {
                for &ci in d.order.iter().rev() {
                    let c = &d.cells[ci as usize];
                    for pin in &c.ins {
                        let need = c
                            .outs
                            .iter()
                            .filter(|o| req[o.1 as usize] != FREE && delay::has_arc(c.kind, &pin.port, &o.0))
                            .map(|(op, ob)| req[*ob as usize] - delay::comb(c.kind, &pin.port, op).ps)
                            .min();
                        let Some(need) = need else { continue };
                        let a = arc_ps(pin.arc);
                        let r = &mut req[pin.bit as usize];
                        *r = (*r).min(need - a);
                        let t = arr[pin.bit as usize];
                        if pin.arc != NONE && t != UNSET {
                            let s = &mut arc_slack[pin.arc as usize];
                            *s = (*s).min(need - a - t);
                        }
                    }
                }
            }
            // This clock's slowest path, for relative criticality.
            let dmax = worst.get(&k).map(|w| w.0).filter(|&t| t > 0);
            for (ni, &b) in d.net_bit.iter().enumerate().filter(|(_, b)| **b != NONE) {
                let t = arr[b as usize];
                if t == UNSET {
                    continue;
                }
                net_arrival[ni] = net_arrival[ni].max(t);
                // `req` is empty for a clock with no period.
                if let Some(p) = period.filter(|_| req[b as usize] != FREE) {
                    let s = req[b as usize] - t;
                    net_slack[ni] = net_slack[ni].min(s);
                    if let Some(dm) = dmax {
                        // The slowest path through the net: period less slack.
                        net_rel[ni] = net_rel[ni].max(((p - s) as f32 / dm as f32).clamp(0.0, 1.0));
                    }
                }
            }

            for (k2, (total, ci, caps, pi)) in worst {
                let end = &d.cells[ci];
                let pin = if caps { &end.caps[pi] } else { &end.ins[pi] };
                let su = delay::setup(end.kind, &pin.port);
                let (mut logic, mut route, mut levels) = (su.ps, arc_ps(pin.arc), 0usize);
                let mut assumed: Vec<String> = Vec::new();
                if su.assumed {
                    assumed.push(format!("setup {:?}", end.kind));
                }
                let mut steps = vec![json!({"setup": su.ps, "route": arc_ps(pin.arc), "cell": end.name, "site": end.site})];
                let mut bit = pin.bit;
                while pred[bit as usize].0 != NONE {
                    let (ci, pi) = pred[bit as usize];
                    let c = &d.cells[ci as usize];
                    let pin = &c.ins[pi as usize];
                    let out = c.outs.iter().find(|o| o.1 == bit).map(|o| o.0.as_str()).unwrap_or("");
                    let cd = delay::comb(c.kind, &pin.port, out);
                    logic += cd.ps;
                    route += arc_ps(pin.arc);
                    levels += 1;
                    if cd.assumed {
                        assumed.push(format!("through {:?}", c.kind));
                    }
                    steps.push(json!({"logic": cd.ps, "route": arc_ps(pin.arc), "kind": format!("{:?}", c.kind), "cell": c.name, "site": c.site}));
                    bit = pin.bit;
                }
                let launch = &d.cells[d.driver[bit as usize] as usize];
                let port = launch.outs.iter().find(|o| o.1 == bit).map(|o| o.0.as_str()).unwrap_or("");
                let q = launch.launch(port);
                logic += q.ps;
                if q.assumed {
                    assumed.push(format!("clock-to-out {:?}", launch.kind));
                }
                steps.push(json!({"clk_to_q": q.ps, "cell": launch.name, "site": launch.site}));
                steps.reverse();
                paths.push(PathReport {
                    from_clk: d.clocks[k as usize].clone(),
                    to_clk: d.clocks[k2 as usize].clone(),
                    delay_ps: total,
                    logic_ps: logic,
                    route_ps: route,
                    levels,
                    from: launch.name.clone(),
                    to: end.name.clone(),
                    from_kind: launch.kind,
                    to_kind: end.kind,
                    assumed,
                    steps,
                });
            }
        }
        paths.sort_by_key(|p| std::cmp::Reverse(p.delay_ps));

        let mut crit = CriticalityMap { net: net_slack.iter().map(|&s| if s == FREE { 0.0 } else { criticality(s) }).collect(), sink: vec![Vec::new(); net_slack.len()] };
        for (arc, &s) in d.arcs.iter().zip(&arc_slack) {
            if s != FREE && s < 0 {
                let c = criticality(s);
                crit.sink[arc.net as usize].extend(arc.cands.iter().map(|&(n, _)| (n, c)));
            }
        }
        for s in &mut crit.sink {
            // A node of two arcs keeps the larger.
            s.sort_by(|a, b| a.0.cmp(&b.0).then(b.1.total_cmp(&a.1)));
            s.dedup_by_key(|e| e.0);
        }
        self.table = SlackTable {
            net_slack: net_slack.into_iter().map(|s| (s != FREE).then_some(s)).collect(),
            net_arrival: net_arrival.into_iter().map(|t| (t != UNSET).then_some(t)).collect(),
            net_rel,
            paths,
            endpoints,
            crit,
        };
        &self.table
    }
}

/// What the router asks its timing of: the table for `routes` as they now
/// stand, where only the nets in `touched` have changed since the last
/// call.
pub trait TimingSource {
    fn update(&mut self, g: &Graph, nets: &[Net], routes: &[Vec<Hop>], touched: &[usize]) -> &SlackTable;

    /// The block RAM captures short of `margin_ps` of hold slack under
    /// `routes` (`Timer::hold_shorts`). A source with no design has none.
    fn hold_targets(&mut self, _g: &Graph, _nets: &[Net], _routes: &[Vec<Hop>], _touched: &[usize], _margin_ps: i64) -> Vec<HoldShort> {
        Vec::new()
    }
}

/// One capture short of hold slack (`Timer::hold_shorts`).
#[derive(Debug, Clone)]
pub struct HoldShort {
    /// The import net into the pin.
    pub net: usize,
    /// The nodes it may enter the cell's site by.
    pub nodes: Vec<u32>,
    /// Slow-corner routing delay (ps) to add to the arc.
    pub add_ps: i64,
    pub slack_ps: i64,
}

impl TimingSource for Timer {
    fn update(&mut self, g: &Graph, nets: &[Net], routes: &[Vec<Hop>], touched: &[usize]) -> &SlackTable {
        for &i in touched {
            if self.d.net_arcs.get(i).is_none_or(Vec::is_empty) {
                continue;
            }
            let edges: Vec<TreeEdge> = routes[i].iter().map(|h| (h.src, h.dst, h.tile, h.pi)).collect();
            let arr = delay::arrivals_with(g, nets[i].source, &edges, self.model);
            self.annotate_with(i, &mut |n| arr.get(&n).copied());
        }
        self.analyze()
    }

    fn hold_targets(&mut self, g: &Graph, nets: &[Net], routes: &[Vec<Hop>], touched: &[usize], margin_ps: i64) -> Vec<HoldShort> {
        self.update(g, nets, routes, touched);
        self.hold_shorts(delay::HOLD_ROUTE_SCALE, margin_ps, &[CellKind::Bram])
    }
}

/// A source that answers one table whatever the routes: a router under
/// test needs no design and no clock.
pub struct Stub(pub SlackTable);

impl Stub {
    pub fn with_nets(crit: Vec<f32>) -> Self {
        Stub(SlackTable::from_criticality(CriticalityMap::from_nets(crit)))
    }
}

impl TimingSource for Stub {
    fn update(&mut self, _: &Graph, _: &[Net], _: &[Vec<Hop>], _: &[usize]) -> &SlackTable {
        &self.0
    }
}

/// The router's handle on timing (`router::Opts::timing`): the source it
/// re-times through, and how the answer weighs in.
#[derive(Clone)]
pub struct TimingView {
    source: Rc<RefCell<dyn TimingSource>>,
    /// Weight of `criticality * delay / delay_norm` in an edge's cost.
    pub tau: f32,
    /// A sink pays `1 - discount * criticality` of an edge's congestion
    /// cost: 0, every sink gives way alike; towards 1, a critical sink
    /// holds its route and the others move (VPR's trade). Default 0.9:
    /// with span_aware on it removed a destination overshoot on picoBEAM
    /// (1464 to 650 ps on one worst-path connection); the aggregate gain
    /// (0.6 to 1.9 ns) is at the edge of run-to-run spread.
    pub discount: f32,
    /// A sink's A* distance estimate is scaled by `1 - exact *
    /// criticality`: 0, as greedy as any search (`Opts::estimate`);
    /// towards 1, a critical sink's search nears a cheapest-path search.
    pub exact: f32,
    /// At most this many nets are ripped up for timing per iteration, the
    /// most critical first.
    pub hot_max: usize,
    /// A net is ripped up for timing again only once its criticality has
    /// risen by this much over what it was last routed with.
    pub min_rise: f32,
    /// Iterations that may rip nets up for timing; after them only
    /// congestion does.
    pub passes: u32,
}

impl TimingView {
    /// Defaults from picoBEAM `xc7_m5` (pb_clk 50 ns, axi_clk 8 ns), one
    /// run each, not swept for noise: tau 1 with 6 passes and min_rise
    /// 0.05 took pb_clk's slowest path from 74.9 ns to 65.8; these took it
    /// to 62.4. tau 6, and tau 3 with `Opts::astar_delay` 0.1, landed
    /// within 0.8 ns of that. `hot_max` is unswept: those runs ripped up
    /// 5368 to 9506 nets in all, so it bound in few iterations if any.
    pub fn new(source: Rc<RefCell<dyn TimingSource>>) -> Self {
        TimingView { source, tau: 3.0, discount: 0.9, exact: 0.0, hot_max: 2000, min_rise: 0.02, passes: 12 }
    }

    /// Re-time and list the block RAM captures short of hold slack.
    pub fn hold_targets(&self, g: &Graph, nets: &[Net], routes: &[Vec<Hop>], touched: &[usize], margin_ps: i64) -> Vec<HoldShort> {
        self.source.borrow_mut().hold_targets(g, nets, routes, touched, margin_ps)
    }

    /// Re-time and read the table: `(&SlackTable) -> &CriticalityMap` is
    /// the one way a cost gets its numbers.
    pub fn retime<T>(&self, g: &Graph, nets: &[Net], routes: &[Vec<Hop>], touched: &[usize], read: impl FnOnce(&SlackTable) -> T) -> T {
        read(self.source.borrow_mut().update(g, nets, routes, touched))
    }

    /// The nets to rip up for timing: failing (criticality above 0), and
    /// by `min_rise` more critical than when last routed (`routed_with`);
    /// the `hot_max` most critical of those.
    pub fn hot(&self, crit: &CriticalityMap, routed_with: &[f32], eligible: impl Fn(usize) -> bool) -> Vec<usize> {
        let mut hot: Vec<(f32, usize)> = (0..routed_with.len())
            .filter(|&i| eligible(i))
            .map(|i| (crit.of_net(i), i))
            .filter(|&(c, i)| c > 0.0 && c - routed_with[i] >= self.min_rise)
            .collect();
        hot.sort_by(|a, b| b.0.total_cmp(&a.0).then(a.1.cmp(&b.1)));
        hot.truncate(self.hot_max);
        hot.into_iter().map(|h| h.1).collect()
    }
}

impl std::fmt::Debug for TimingView {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TimingView").field("tau", &self.tau).field("discount", &self.discount).field("exact", &self.exact).field("hot_max", &self.hot_max).field("min_rise", &self.min_rise).field("passes", &self.passes).finish()
    }
}

/// A routed design's FASM as routing delays: every pip it turns on, by
/// source node.
pub struct FasmRouting<'g> {
    g: &'g Graph,
    model: Model,
    out: HashMap<u32, Vec<TreeEdge>>,
    trees: Vec<Net>,
    /// Node -> the tree it is on.
    node_tree: HashMap<u32, usize>,
}

/// How `FasmRouting::annotate` timed the arcs.
#[derive(Debug, Clone, Copy, Default)]
pub struct FasmCounts {
    /// From the net's source.
    pub from_source: usize,
    /// From the root of the sink's own tree only (`fixed_pins`).
    pub from_sink_tree: usize,
    pub unreached: usize,
}

impl<'g> FasmRouting<'g> {
    pub fn new(g: &'g Graph, fasm: &str, model: Model) -> Self {
        let (mut trees, _) = nets::from_fasm(g, fasm);
        let mut out: HashMap<u32, Vec<TreeEdge>> = HashMap::new();
        for e in trees.iter().flat_map(|t| &t.edges) {
            out.entry(e.src).or_default().push((e.src, e.dst, e.tile, e.pi));
        }
        // Carry each tree on along the fixed hops a FASM has no lines for,
        // to the site pins.
        for t in &mut trees {
            let mut stack = t.sinks.clone();
            let mut seen: HashSet<u32> = stack.iter().copied().collect();
            while let Some(n) = stack.pop() {
                for &(d, tile, pi) in g.out_edges(n) {
                    if g.edge_kind(tile, pi) == EdgeKind::Fixed && seen.insert(d) {
                        t.edges.push(Edge { src: n, dst: d, tile, pi, kind: EdgeKind::Fixed });
                        stack.push(d);
                    }
                }
            }
        }
        let node_tree = trees.iter().enumerate().flat_map(|(i, t)| t.edges.iter().map(move |e| (e.dst, i))).collect();
        FasmRouting { g, model, out, trees, node_tree }
    }

    /// Arrivals over the whole of a net from its source: every pip the
    /// FASM turns on and every fixed hop (which has no line, at the
    /// driver, at a sink or between two stretches of pips).
    fn net_arrivals(&self, src: u32) -> HashMap<u32, u64> {
        let mut edges: Vec<TreeEdge> = Vec::new();
        let mut seen = HashSet::from([src]);
        let mut stack = vec![src];
        while let Some(n) = stack.pop() {
            let fixed = self.g.out_edges(n).iter().filter(|&&(_, tile, pi)| self.g.edge_kind(tile, pi) == EdgeKind::Fixed).map(|&(d, tile, pi)| (n, d, tile, pi));
            for e in self.out.get(&n).into_iter().flatten().copied().chain(fixed) {
                if seen.insert(e.1) && edges.len() < 200_000 {
                    edges.push(e);
                    stack.push(e.1);
                }
            }
        }
        delay::arrivals_with(self.g, src, &edges, self.model)
    }

    /// The pips from a net's source `src` to the first of `to` its routing
    /// reaches, in order, each with its intrinsic delay (ps). Empty: none
    /// reached from that source.
    pub fn trace(&self, src: u32, to: &[u32]) -> Vec<(String, u32)> {
        let mut pred: HashMap<u32, TreeEdge> = HashMap::new();
        let mut stack = vec![src];
        while let Some(n) = stack.pop() {
            let fixed = self.g.out_edges(n).iter().filter(|&&(_, tile, pi)| self.g.edge_kind(tile, pi) == EdgeKind::Fixed).map(|&(d, tile, pi)| (n, d, tile, pi));
            for e in self.out.get(&n).into_iter().flatten().copied().chain(fixed) {
                if e.1 != src && !pred.contains_key(&e.1) && pred.len() < 200_000 {
                    pred.insert(e.1, e);
                    stack.push(e.1);
                }
            }
        }
        let Some(mut n) = to.iter().copied().find(|n| pred.contains_key(n)) else { return Vec::new() };
        let mut out = Vec::new();
        while let Some(&(s, _, tile, pi)) = pred.get(&n) {
            out.push((self.g.edge_pip(tile, pi).name(), self.g.edge_timing(tile, pi).delay));
            n = s;
        }
        out.reverse();
        out
    }

    /// Annotate every net of `timer` (`nets`: the import's). `fixed_pins`:
    /// the sinks are the pins the FASM's own placer chose (`Perm::None`).
    /// A driver that leaves its site by two pins then has two trees and
    /// the import names one; a sink the named one misses is timed on its
    /// own node's tree, from that tree's root.
    pub fn annotate(&self, timer: &mut Timer, nets: &[Net], fixed_pins: bool) -> FasmCounts {
        let mut counts = FasmCounts::default();
        let mut tree_arr: HashMap<usize, HashMap<u32, u64>> = HashMap::new();
        for (i, net) in nets.iter().enumerate() {
            if timer.d.net_arcs.get(i).is_none_or(Vec::is_empty) {
                continue;
            }
            let arr = self.net_arrivals(net.source);
            timer.annotate_with(i, &mut |n| arr.get(&n).copied());
            let missed = timer.unreached(i);
            counts.from_source += timer.d.net_arcs[i].len() - missed.len();
            for (a, cands) in missed {
                let mut best: Option<i64> = None;
                for (n, p) in cands.into_iter().filter(|_| fixed_pins) {
                    let Some(&t) = self.node_tree.get(&n) else { continue };
                    let tree = &self.trees[t];
                    let ta = tree_arr.entry(t).or_insert_with(|| {
                        let edges: Vec<TreeEdge> = tree.edges.iter().map(|e| (e.src, e.dst, e.tile, e.pi)).collect();
                        delay::arrivals_with(self.g, tree.source, &edges, self.model)
                    });
                    if let Some(&v) = ta.get(&n) {
                        best = best.max(Some(v as i64 + timer.site_pip * p as i64));
                    }
                }
                match best {
                    Some(t) => {
                        timer.arc_ps[a as usize] = t + timer.site_pip * timer.d.arcs[a as usize].src_pips as i64;
                        timer.arc_state[a as usize] = ArcState::Timed;
                        counts.from_sink_tree += 1;
                    }
                    None => counts.unreached += 1,
                }
            }
        }
        counts
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pin(port: &str, bit: u32, arc: u32) -> Pin {
        Pin { port: port.into(), bit, arc }
    }

    fn cell(name: &str, kind: CellKind) -> Cell {
        Cell { name: name.into(), kind, site: None, ins: vec![], outs: vec![], clocks: vec![], caps: vec![], out_reg: [false; 2] }
    }

    /// ff0 -(net 0)-> lut0 -(net 1)-> lut1 -(net 2)-> ff1, one clock, and
    /// ff0 -(net 0)-> ff2 beside it. Arc i enters by node 100 + i; arc 3 is
    /// net 0 into ff2.
    fn chain() -> Design {
        let mut ff0 = cell("ff0", CellKind::Ff);
        ff0.outs.push(("Q".into(), 0));
        ff0.clocks.push(("CK".into(), 0));
        let mut lut0 = cell("lut0", CellKind::Lut);
        lut0.ins.push(pin("A1", 0, 0));
        lut0.outs.push(("O6".into(), 1));
        let mut lut1 = cell("lut1", CellKind::Lut);
        lut1.ins.push(pin("A2", 1, 1));
        lut1.outs.push(("O6".into(), 2));
        let mut ff1 = cell("ff1", CellKind::Ff);
        ff1.ins.push(pin("D", 2, 2));
        ff1.clocks.push(("CK".into(), 0));
        let mut ff2 = cell("ff2", CellKind::Ff);
        ff2.ins.push(pin("D", 0, 3));
        ff2.clocks.push(("CK".into(), 0));
        let arc = |net: u32, sink: u32, node: u32| Arc { net, sink, bit: net, cands: vec![(node, 0)], src_pips: 0 };
        let mut d = Design {
            cells: vec![ff0, lut0, lut1, ff1, ff2],
            clocks: vec!["clk".into()],
            arcs: vec![arc(0, 1, 100), arc(1, 2, 101), arc(2, 3, 102), arc(0, 4, 103)],
            net_arcs: vec![vec![0, 3], vec![1], vec![2]],
            net_bit: vec![0, 1, 2],
            ..Default::default()
        };
        d.finish(3);
        d
    }

    fn timer(period_ps: i64) -> Timer {
        Timer::new(chain(), &Constraints { default_ps: Some(period_ps), clocks: vec![] }, Model::Intrinsic)
    }

    /// Give net `net` the arrivals in `at` (node, ps).
    fn route(t: &mut Timer, net: usize, at: &[(u32, u64)]) {
        t.annotate_with(net, &mut |n| at.iter().find(|a| a.0 == n).map(|a| a.1));
    }

    #[test]
    fn criticality_is_zero_at_slack_and_half_at_one_ns_short() {
        assert_eq!(criticality(0), 0.0);
        assert_eq!(criticality(250), 0.0);
        assert_eq!(criticality(-1000), 0.5);
        assert!((criticality(-9000) - 0.9).abs() < 1e-6);
    }

    #[test]
    fn a_path_is_clock_to_out_cells_routing_and_setup() {
        let mut t = timer(10_000);
        route(&mut t, 0, &[(100, 1000), (103, 200)]);
        route(&mut t, 1, &[(101, 500)]);
        route(&mut t, 2, &[(102, 300)]);
        let table = t.analyze();
        // 350 + 1000 + 120 + 500 + 120 + 300 + 100.
        let p = table.path("clk").expect("one clock, one path");
        assert_eq!((p.delay_ps, p.logic_ps, p.route_ps, p.levels), (2490, 690, 1800, 2));
        assert_eq!((p.from.as_str(), p.to.as_str()), ("ff0", "ff1"));
        assert!(p.assumed.is_empty());
        assert_eq!(table.net_slack, vec![Some(7510), Some(7510), Some(7510)]);
        assert_eq!(table.net_arrival, vec![Some(350), Some(1470), Some(2090)]);
        assert_eq!(table.criticality().failing(), 0);
    }

    #[test]
    fn a_missed_period_makes_the_path_critical_and_the_side_sink_not() {
        let mut t = timer(2_000);
        route(&mut t, 0, &[(100, 1000), (103, 200)]);
        route(&mut t, 1, &[(101, 500)]);
        route(&mut t, 2, &[(102, 300)]);
        let crit = t.analyze().criticality().clone();
        // 490 ps short everywhere along the path.
        let want = criticality(-490);
        for net in 0..3 {
            assert!((crit.of_net(net) - want).abs() < 1e-6, "net {net}");
        }
        // Net 0's sink at ff2 (350 + 200 + 100) meets the clock.
        assert!((crit.of_sink(0, &[100]) - want).abs() < 1e-6);
        assert_eq!(crit.of_sink(0, &[103]), 0.0);
        assert_eq!(crit.hot_sinks(0).collect::<Vec<_>>(), vec![100]);
    }

    #[test]
    fn re_timing_one_net_moves_every_slack_on_its_path() {
        let mut t = timer(2_000);
        route(&mut t, 0, &[(100, 1000), (103, 200)]);
        route(&mut t, 1, &[(101, 500)]);
        route(&mut t, 2, &[(102, 300)]);
        assert_eq!(t.analyze().worst_slack(), Some(-490));
        // Net 0 alone is routed again, 600 ps faster to lut0.
        route(&mut t, 0, &[(100, 400), (103, 200)]);
        let table = t.analyze();
        assert_eq!(table.net_slack, vec![Some(110), Some(110), Some(110)]);
        assert_eq!(table.criticality().failing(), 0);
        assert_eq!(t.arc_counts(), ArcCounts { timed: 4, unknown: 0, unreached: 0 });
    }

    #[test]
    fn an_arc_its_routing_misses_is_counted_not_guessed() {
        let mut t = timer(2_000);
        route(&mut t, 0, &[(100, 1000)]);
        assert_eq!(t.arc_counts(), ArcCounts { timed: 1, unknown: 2, unreached: 1 });
    }

    #[test]
    fn a_clock_without_a_period_has_paths_but_no_slack() {
        let mut t = Timer::new(chain(), &Constraints::default(), Model::Intrinsic);
        route(&mut t, 0, &[(100, 1000), (103, 200)]);
        let table = t.analyze();
        assert_eq!(table.path("clk").map(|p| p.delay_ps), Some(350 + 1000 + 120 + 120 + 100));
        assert_eq!(table.net_slack, vec![None, None, None]);
        assert_eq!(table.criticality().failing(), 0);
    }

    /// The same chain as `chain`, from a netlist with no routing.
    fn chain_from_cells() -> Design {
        let pin = |p: &str, out: bool, bit: u32| (p.to_string(), out, bit);
        let spec = |name: &str, kind: CellKind, pins: Vec<(String, bool, u32)>| CellSpec { name: name.into(), kind, pins, out_reg: [false; 2] };
        // Bit 9 is the clock; bits 0..3 are nets 0..3.
        let cells = vec![
            spec("ff0", CellKind::Ff, vec![pin("Q", true, 0), pin("CK", false, 9)]),
            spec("lut0", CellKind::Lut, vec![pin("A1", false, 0), pin("O6", true, 1)]),
            spec("lut1", CellKind::Lut, vec![pin("A2", false, 1), pin("O6", true, 2)]),
            spec("ff1", CellKind::Ff, vec![pin("D", false, 2), pin("CK", false, 9)]),
            spec("ff2", CellKind::Ff, vec![pin("D", false, 0), pin("CK", false, 9)]),
        ];
        Design::from_cells(cells, 10, 3, &|b| (b < 3).then_some(b))
    }

    #[test]
    fn a_design_from_cells_times_like_the_routed_one() {
        let d = chain_from_cells();
        assert_eq!(d.arcs_len(), 4);
        assert_eq!(d.in_loops, 0);
        // Arc 0 is net 0 into lut0, driven by ff0.
        assert_eq!(d.arc_cells(0), (Some(0), 1));
        assert_eq!(d.cell_kind(1), CellKind::Lut);
        let mut t = Timer::new(d, &Constraints { default_ps: Some(10_000), clocks: vec![] }, Model::Intrinsic);
        // Arcs in cell order: into lut0, lut1, ff1, ff2. The routed
        // chain's delays.
        assert_eq!((0..4).map(|a| t.design().arc_cells(a).1).collect::<Vec<_>>(), vec![1, 2, 3, 4]);
        for (a, ps) in [(0, 1000), (1, 500), (2, 300), (3, 200)] {
            t.set_arc_ps(a, ps);
        }
        let table = t.analyze();
        assert_eq!(table.paths[0].delay_ps, 2490);
        assert_eq!(table.net_slack, vec![Some(7510), Some(7510), Some(7510)]);
    }

    #[test]
    fn the_floor_keeps_logic_and_one_hop_per_routed_arc() {
        // Routed: 690 of logic and 1000 + 500 + 300 of wire to ff1.
        let routed = [(0, 1000), (1, 500), (2, 300), (3, 200)];
        let timed = |level: Option<Floor>| {
            let mut t = Timer::new(chain_from_cells(), &Constraints::default(), Model::Intrinsic);
            for (a, ps) in routed {
                t.set_arc_ps(a, ps);
            }
            let n = level.map(|l| t.floor(l, 301));
            (t.analyze().paths[0].delay_ps, n)
        };
        assert_eq!(timed(None).0, 2490);
        // No sites here, so nothing is packed together: the LUT into ff1
        // keeps its 300 (under a hop), the two longer arcs fall to 301.
        let (packed, n) = timed(Some(Floor::Packed));
        assert_eq!(packed, 690 + 301 + 301 + 300);
        assert_eq!(n, Some(FloorCounts { in_slice: 0, dedicated: 0, routed: 4, lowered: 2 }));
        // The netlist alone: that LUT could share ff1's slice.
        let (netlist, n) = timed(Some(Floor::Netlist));
        assert_eq!(netlist, 690 + 301 + 301);
        assert_eq!(n, Some(FloorCounts { in_slice: 1, dedicated: 0, routed: 3, lowered: 3 }));
    }

    #[test]
    fn hold_takes_the_earliest_arrival_and_names_the_short_path() {
        let mut t = Timer::new(chain_from_cells(), &Constraints::default(), Model::Intrinsic);
        for (a, ps) in [(0, 1000), (1, 500), (2, 300), (3, 200)] {
            t.set_arc_ps(a, ps);
        }
        // Routing at half its slow delay. ff0 -> ff2 direct: 99 + 100,
        // against a 241 hold: 42 short. Through the LUTs to ff1:
        // 99 + 500 + 42 + 250 + 42 + 150 = 1083, 842 to spare.
        let h = t.hold(0.5, 0);
        assert_eq!(h.len(), 1);
        let w = &h[0];
        assert_eq!((w.slack_ps, w.arrival_ps, w.hold_ps), (-42, 199, 241));
        assert_eq!((w.from.as_str(), w.to.as_str(), w.cells), ("ff0", "ff2", 0));
        assert_eq!((w.checked, w.under_margin), (2, 1));
    }

    #[test]
    fn relative_criticality_is_against_the_worst_path_not_the_period() {
        let mut t = Timer::new(chain_from_cells(), &Constraints { default_ps: Some(1_000_000), clocks: vec![] }, Model::Intrinsic);
        for (a, ps) in [(0, 1000), (1, 500), (2, 300), (3, 200)] {
            t.set_arc_ps(a, ps);
        }
        let table = t.analyze();
        // Nothing fails a 1 us period, yet the slowest path's nets are
        // fully critical: every net here is on it.
        assert_eq!(table.criticality().failing(), 0);
        assert_eq!(table.net_rel, vec![1.0, 1.0, 1.0]);
        // Net 0 into ff2 alone would be 350 + 200 + 100 of 2490; its
        // worst path is the long one, so it stays at 1.
        assert!(table.net_rel.iter().all(|&r| (0.0..=1.0).contains(&r)));
    }

    #[test]
    fn constraints_name_clocks_or_default() {
        let c = Constraints::parse("pb_clk=50000, *=8000").unwrap();
        assert_eq!(c.period("pb_clk"), Some(50_000));
        assert_eq!(c.period("axi_clk"), Some(8_000));
        assert_eq!(Constraints::parse("").unwrap().period("x"), None);
        assert!(Constraints::parse("pb_clk").is_err());
        assert!(Constraints::parse("pb_clk=fast").is_err());
    }

    #[test]
    fn a_stubbed_view_picks_hot_nets_without_a_design() {
        let view = TimingView { hot_max: 2, ..TimingView::new(Rc::new(RefCell::new(Stub::with_nets(vec![0.0, 0.9, 0.3, 0.6, 0.02])))) };
        let crit = CriticalityMap::from_nets(vec![0.0, 0.9, 0.3, 0.6, 0.02]);
        // Failing, risen by min_rise, most critical first, two at most.
        assert_eq!(view.hot(&crit, &[0.0; 5], |_| true), vec![1, 3]);
        // Net 1 was last routed knowing it was this critical.
        assert_eq!(view.hot(&crit, &[0.0, 0.9, 0.0, 0.0, 0.0], |_| true), vec![3, 2]);
        // Net 3 is not the router's to move.
        assert_eq!(view.hot(&crit, &[0.0; 5], |i| i != 3), vec![1, 2]);
        // Without per-sink values a sink is as critical as its net.
        assert_eq!(crit.of_sink(1, &[7]), 0.9);
        assert_eq!(crit.of_net(99), 0.0);
    }
}
