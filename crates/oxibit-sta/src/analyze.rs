//! Setup analysis of a routed design: arrival times over a graph of cell
//! pins, per clock domain, and each routed sink's criticality.
//!
//! Nodes are (cell, bel pin). Edges:
//! - net arcs, driver pin to each sink pin: the routed tree's interconnect
//!   delay to the sink's node (`delay::arrivals_with`) plus a site pip at
//!   each end ([`Options::site_pip_ps`]), or one site pip for a sink inside
//!   the driver's site;
//! - cell arcs, from the SDF library by cell type and bel: LUT inputs to
//!   O6/O5, CARRY4 inputs to O/CO, F7/F8 mux inputs to OUT, and the inputs
//!   to the outputs of a DSP48E1 with no clock (every register bypassed),
//!   by bus, the worst over the library's unregistered multiply modes.
//!
//! Startpoints are clocked outputs (flop Q at clock-to-Q; BRAM outputs at
//! the library's clock-to-DO; a clocked DSP's outputs at the worst
//! clock-to-output of its file, a bound) and the data outputs of cells the
//! library does not time (PS7, MMCM), which launch at 0 in the domain of
//! the cell's clock. Endpoints are clocked inputs with their setup (flop
//! D, CE, SR; LUTRAM write pins; BRAM inputs by pin group; a clocked DSP's
//! inputs at the worst setup of its file, a bound) and the data inputs of
//! untimed cells.
//!
//! Clock pins of untimed cells (CLKOUT*, CLKFB*, CLKIN*, FCLKCLK*, *CLK)
//! are not data, and BUFGs carry only clocks: none of them starts or ends
//! a path. Their asynchronous pins (LOCKED, FCLKRESETN*, RST, PWRDWN) are
//! false paths: counted, not timed.
//!
//! Clocks: a cell's clock arrives at its clock pin at the routed clock
//! tree's delay from the buffer (same model, site pips included). A path
//! launches at its start cell's clock arrival and is captured at its end
//! cell's, so its delay is data arrival + setup - capture clock arrival,
//! which carries the skew. Insertion delay ahead of the buffer is not
//! modelled: across two buffers' domains the skew is relative.
//!
//! A domain's Fmax is 1 / its worst same-domain path. Criticality of a net
//! arc is 1 - slack / Dmax, with every endpoint required by its capture
//! domain's worst path (the period it would run at) and Dmax the worst of
//! those; cross-domain paths are held to the capture domain's period.

use std::collections::{BTreeMap, HashMap, VecDeque};

use oxibit_fasm::route::Graph;
use oxibit_pnr::delay::{self, Model, TreeEdge};
use oxibit_pnr::nets::{Net, NetClass};
use oxibit_pnr::placed::{Import, SinkPin};
use oxibit_pnr::router::Hop;
use serde_json::{Value, json};

use crate::sdf::{CheckKind, Library};

#[derive(Debug, Clone, Copy)]
pub struct Options {
    /// Delay of one site pip (bel pin to site pin, or back), ps. The
    /// interconnect model counts none.
    pub site_pip_ps: u32,
    pub model: Model,
    /// Worst paths to report in full.
    pub paths: usize,
}

impl Default for Options {
    fn default() -> Self {
        Options {
            site_pip_ps: 20,
            model: Model::Intrinsic,
            paths: 10,
        }
    }
}

/// The domain of a startpoint or endpoint: a clock net's name, or one of
/// [`ASYNC`], [`UNCLOCKED`].
type Domain = String;

/// Asynchronous pins of untimed cells: false paths.
pub const ASYNC: &str = "async";
/// Cells with no clock pin on a global clock net.
pub const UNCLOCKED: &str = "unclocked";

#[derive(Debug, Default)]
pub struct Stats {
    pub nodes: usize,
    pub net_arcs: usize,
    pub cell_arcs: usize,
    pub startpoints: usize,
    pub endpoints: usize,
    /// Nodes on a combinational loop (never ordered).
    pub in_loops: usize,
    /// Clocked DSPs, timed by a bound.
    pub bounded_cells: usize,
    /// DSPs with every register bypassed, timed through.
    pub dsp_comb_cells: usize,
    pub untimed: BTreeMap<String, usize>,
    /// Sink pins no part of their net reached.
    pub unreached_sinks: usize,
    /// Cells with a clock pin on a global clock net, and those the clock
    /// tree does not reach.
    pub clock_sinks: usize,
    pub clock_unreached: usize,
    /// Clock arrival over clocked cells, ps.
    pub clock_min_ps: i64,
    pub clock_max_ps: i64,
    /// False-path startpoints and endpoints (asynchronous pins).
    pub async_starts: usize,
    pub async_ends: usize,
    /// Net arcs with criticality at least 0.5, 0.8, 0.9, 0.95.
    pub crit_ge: [usize; 4],
}

/// One endpoint's worst path.
#[derive(Debug, Clone)]
pub struct Path {
    pub launch: Domain,
    pub capture: Domain,
    /// Data arrival plus setup less the capture clock's arrival, ps.
    pub delay: i64,
    pub setup: i64,
    /// Launch clock arrival less capture clock arrival, ps (in `delay`).
    pub skew: i64,
    /// (cell, pin, arrival ps) from the startpoint to the endpoint.
    pub pins: Vec<(String, String, i64)>,
}

#[derive(Debug, Default)]
pub struct Report {
    pub stats: Stats,
    /// Worst delay per (launch, capture) domain pair.
    pub worst: BTreeMap<(Domain, Domain), Path>,
    /// The worst same-domain paths, worst first.
    pub paths: Vec<Path>,
    /// Per import net, per sink pin (`Import::sink_pins`): criticality,
    /// 0..=1 (0 for clock nets and arcs on no timed path).
    pub crit: Vec<Vec<f32>>,
}

fn timed(d: &str) -> bool {
    d != ASYNC && d != UNCLOCKED
}

impl Report {
    pub fn json(&self) -> Value {
        let path = |p: &Path, full: bool| {
            let mut v = json!({"launch": p.launch, "capture": p.capture, "delay_ps": p.delay, "setup_ps": p.setup,
                               "skew_ps": p.skew,
                               "start": p.pins.first().map(|x| format!("{}.{}", x.0, x.1)),
                               "end": p.pins.last().map(|x| format!("{}.{}", x.0, x.1)), "depth": p.pins.len()});
            if full {
                v["pins"] = json!(
                    p.pins
                        .iter()
                        .map(|x| format!("{}.{} @{}", x.0, x.1, x.2))
                        .collect::<Vec<_>>()
                );
            }
            v
        };
        let domains: Vec<Value> = self
            .worst
            .iter()
            .map(|((l, c), p)| {
                let mut v = path(p, false);
                if l == c && timed(l) && p.delay > 0 {
                    v["fmax_mhz"] = json!(((1e6 / p.delay as f64) * 100.0).round() / 100.0);
                }
                v
            })
            .collect();
        let s = &self.stats;
        json!({
            "domains": domains,
            "paths": self.paths.iter().map(|p| path(p, true)).collect::<Vec<_>>(),
            "stats": {"nodes": s.nodes, "net_arcs": s.net_arcs, "cell_arcs": s.cell_arcs, "startpoints": s.startpoints,
                      "endpoints": s.endpoints, "in_loops": s.in_loops, "bounded_cells": s.bounded_cells,
                      "dsp_comb_cells": s.dsp_comb_cells, "untimed": s.untimed, "unreached_sinks": s.unreached_sinks,
                      "clock_sinks": s.clock_sinks, "clock_unreached": s.clock_unreached,
                      "clock_min_ps": s.clock_min_ps, "clock_max_ps": s.clock_max_ps,
                      "async_starts": s.async_starts, "async_ends": s.async_ends,
                      "crit_ge_50_80_90_95": s.crit_ge},
        })
    }

    /// Per import net, the criticality of each sink node, for the router
    /// (`router::Opts::criticality`): a pin group's nodes all take the
    /// group's.
    pub fn router_criticality(&self, imp: &Import) -> Vec<HashMap<u32, f32>> {
        imp.sink_pins
            .iter()
            .zip(&self.crit)
            .map(|(pins, cs)| {
                let mut m: HashMap<u32, f32> = HashMap::new();
                for (sp, &c) in pins.iter().zip(cs) {
                    if c <= 0.0 {
                        continue;
                    }
                    for &n in &sp.nodes {
                        let e = m.entry(n).or_insert(0.0);
                        *e = e.max(c);
                    }
                }
                m
            })
            .collect()
    }
}

fn clean(p: &str) -> String {
    p.replace(['[', ']'], "")
}

/// A pin's bus: its name less the trailing bit index.
fn bus(p: &str) -> &str {
    p.trim_end_matches(|c: char| c.is_ascii_digit())
}

/// A clock pin of an untimed cell (MMCM, PS7, BUFG).
fn clock_pin(p: &str) -> bool {
    p.starts_with("CLKOUT")
        || p.starts_with("CLKFB")
        || p.starts_with("CLKIN")
        || p.starts_with("FCLKCLK")
        || p.ends_with("CLK")
}

/// An asynchronous pin of an untimed cell.
fn async_pin(p: &str) -> bool {
    p == "LOCKED" || p.starts_with("FCLKRESETN") || p == "RST" || p == "PWRDWN"
}

const DSP_OUT: [&str; 11] = [
    "P",
    "PCOUT",
    "CARRYOUT",
    "CARRYCASCOUT",
    "MULTSIGNOUT",
    "PATTERNDETECT",
    "PATTERNBDETECT",
    "ACOUT",
    "BCOUT",
    "OVERFLOW",
    "UNDERFLOW",
];

/// BRAM input pin groups, by name prefix.
const BRAM_IN: [&str; 6] = ["DI", "ADDR", "WE", "EN", "REGCE", "RST"];

/// Worst value over a file's arcs from a clock pin, and over its setups.
fn file_bounds(lib: &Library, file: &str) -> (i64, i64) {
    let mut clk2out = 0i64;
    let mut setup = 0i64;
    for ((f, _, _), c) in lib.cells() {
        if f != file {
            continue;
        }
        for a in &c.arcs {
            if a.from.contains("CLK") {
                clk2out = clk2out.max(a.delay.max as i64);
            }
        }
        for k in &c.checks {
            if k.kind == CheckKind::Setup {
                setup = setup.max(k.value.max as i64);
            }
        }
    }
    (clk2out, setup)
}

/// Worst setup of a BRAM input group (`BRAM_IN`) over the file.
fn bram_setups(lib: &Library, fallback: i64) -> Vec<(&'static str, i64)> {
    BRAM_IN
        .iter()
        .map(|&pre| {
            let s = lib
                .cells()
                .filter(|((f, _, _), _)| f == "BRAM_L")
                .flat_map(|(_, c)| c.checks.iter())
                .filter(|k| k.kind == CheckKind::Setup && clean(&k.pin).starts_with(pre))
                .map(|k| k.value.max as i64)
                .max()
                .unwrap_or(fallback);
            (pre, s)
        })
        .collect()
}

/// (input bus, output bus) -> worst delay over the DSP file's modes with
/// every register bypassed and a static multiplier.
fn dsp_comb_arcs(lib: &Library) -> HashMap<(String, String), i64> {
    let skip = [
        "REG_1",
        "REG_2",
        "USE_DPORT_TRUE",
        "USE_MULT_DYNAMIC",
        "USE_MULT_NONE",
    ];
    let mut out: HashMap<(String, String), i64> = HashMap::new();
    for ((f, t, _), c) in lib.cells() {
        if f != "DSP_L" || skip.iter().any(|s| t.contains(s)) {
            continue;
        }
        for a in &c.arcs {
            if a.from.contains("CLK") {
                continue;
            }
            let k = (
                bus(&clean(&a.from)).to_string(),
                bus(&clean(&a.to)).to_string(),
            );
            let e = out.entry(k).or_insert(0);
            *e = (*e).max(a.delay.max as i64);
        }
    }
    out
}

struct Graphs {
    ids: HashMap<(usize, String), u32>,
    names: Vec<(usize, String)>,
    out: Vec<Vec<(u32, i64)>>,
    indeg: Vec<u32>,
}

impl Graphs {
    fn node(&mut self, cell: usize, port: &str) -> u32 {
        let key = (cell, clean(port));
        if let Some(&n) = self.ids.get(&key) {
            return n;
        }
        let n = self.names.len() as u32;
        self.ids.insert(key.clone(), n);
        self.names.push(key);
        self.out.push(Vec::new());
        self.indeg.push(0);
        n
    }
    fn get(&self, cell: usize, port: &str) -> Option<u32> {
        self.ids.get(&(cell, clean(port))).copied()
    }
    fn edge(&mut self, a: u32, b: u32, d: i64) {
        if let Some(e) = self.out[a as usize].iter_mut().find(|e| e.0 == b) {
            e.1 = e.1.max(d);
            return;
        }
        self.out[a as usize].push((b, d));
        self.indeg[b as usize] += 1;
    }
}

/// Setup analysis of a routed design. `nets` and `routes` are the
/// router's (the import's nets first, in its order).
pub fn analyze(
    g: &Graph,
    imp: &Import,
    nets: &[Net],
    routes: &[Vec<Hop>],
    lib: &Library,
    opts: Options,
) -> Report {
    let mut rep = Report::default();
    let mut gr = Graphs {
        ids: HashMap::new(),
        names: Vec::new(),
        out: Vec::new(),
        indeg: Vec::new(),
    };
    let pip = opts.site_pip_ps as i64;
    let tree_at = |i: usize| -> HashMap<u32, u64> {
        if i >= nets.len() || i >= routes.len() {
            return HashMap::new();
        }
        let edges: Vec<TreeEdge> = routes[i]
            .iter()
            .map(|h| (h.src, h.dst, h.tile, h.pi))
            .collect();
        delay::arrivals_with(g, nets[i].driver.anchor(), &edges, opts.model)
    };
    let sink_at = |at: &HashMap<u32, u64>, sp: &SinkPin| -> Option<i64> {
        if sp.internal {
            Some(pip)
        } else {
            sp.nodes
                .iter()
                .filter_map(|n| at.get(n))
                .min()
                .map(|&a| a as i64 + 2 * pip)
        }
    };

    // Clock domain of each cell (the clock net on its clock pin) and the
    // clock's arrival there.
    let mut clock_of: HashMap<usize, Domain> = HashMap::new();
    let mut clk_at: HashMap<usize, i64> = HashMap::new();
    for (i, n) in imp.nets.iter().enumerate() {
        if n.class != NetClass::GlobalClock {
            continue;
        }
        let name = imp.names[i]
            .split('@')
            .next()
            .unwrap_or_default()
            .to_string();
        let at = tree_at(i);
        for sp in &imp.sink_pins[i] {
            if clock_of.contains_key(&sp.cell) {
                continue;
            }
            clock_of.insert(sp.cell, name.clone());
            rep.stats.clock_sinks += 1;
            match sink_at(&at, sp) {
                Some(a) => {
                    clk_at.insert(sp.cell, a);
                }
                None => rep.stats.clock_unreached += 1,
            }
        }
    }
    rep.stats.clock_min_ps = clk_at.values().copied().min().unwrap_or(0);
    rep.stats.clock_max_ps = clk_at.values().copied().max().unwrap_or(0);

    // Routed nets by driver pin: a net split by exit has one part per exit.
    let mut parts: HashMap<&(usize, String), Vec<usize>> = HashMap::new();
    for (i, d) in imp.driver_pins.iter().enumerate() {
        parts.entry(d).or_default().push(i);
    }

    // Net arcs: (net, sink pin index, from, to, delay).
    let mut net_arcs: Vec<(usize, usize, u32, u32, i64)> = Vec::new();
    for i in 0..imp.nets.len().min(nets.len()) {
        if nets[i].class != NetClass::Signal {
            continue;
        }
        let at = tree_at(i);
        let (dc, dp) = &imp.driver_pins[i];
        let from = gr.node(*dc, dp);
        for (k, sp) in imp.sink_pins[i].iter().enumerate() {
            match sink_at(&at, sp) {
                Some(d) => {
                    let to = gr.node(sp.cell, &sp.port);
                    gr.edge(from, to, d);
                    net_arcs.push((i, k, from, to, d));
                    rep.stats.net_arcs += 1;
                }
                None => {
                    // Another exit's part reaches it, or nothing does.
                    let others = parts[&imp.driver_pins[i]].len() > 1;
                    if !others {
                        rep.stats.unreached_sinks += 1;
                    }
                }
            }
        }
    }

    // Cell arcs, startpoints, endpoints.
    let (bram_c2o, bram_su) = file_bounds(lib, "BRAM_L");
    let bram_su_by = bram_setups(lib, bram_su);
    let (dsp_c2o, dsp_su) = file_bounds(lib, "DSP_L");
    let dsp_comb = dsp_comb_arcs(lib);
    let ff = lib.get("slicel", "FF_INIT", "SLICEL");
    let ff_c2q = ff
        .and_then(|c| c.arc("CLK", "Q"))
        .map(|d| d.max as i64)
        .unwrap_or(0);
    let su = |pin: &str| {
        ff.and_then(|c| c.check(CheckKind::Setup, pin))
            .map(|k| k.value.max as i64)
            .unwrap_or(0)
    };
    let (ff_su_d, ff_su_ce) = (su("DIN"), su("CE"));
    let ram_su = lib
        .cells()
        .filter(|((f, t, _), _)| f == "slicem" && t.starts_with("LUT_OR_MEM6LRAM"))
        .flat_map(|(_, c)| c.checks.iter())
        .filter(|k| k.kind == CheckKind::Setup)
        .map(|k| k.value.max as i64)
        .max()
        .unwrap_or(0);
    let carry = lib.get("slicel", "CARRY4", "SLICEL");
    let carry_o5 = lib.get("slicel", "CARRY4_O5", "SLICEL");

    let mut start: Vec<(u32, i64, Domain)> = Vec::new();
    let mut end: Vec<(u32, i64, Domain)> = Vec::new();
    let mut by_cell: HashMap<usize, Vec<u32>> = HashMap::new();
    for (n, (c, _)) in gr.names.iter().enumerate() {
        by_cell.entry(*c).or_default().push(n as u32);
    }
    let mut cells: Vec<usize> = by_cell.keys().copied().collect();
    cells.sort_unstable();
    for c in cells {
        let info = &imp.cells[c];
        let pins: Vec<(u32, String)> = by_cell[&c]
            .iter()
            .map(|&n| (n, gr.names[n as usize].1.clone()))
            .collect();
        let dom = clock_of
            .get(&c)
            .cloned()
            .unwrap_or_else(|| UNCLOCKED.to_string());
        let is_out = |p: &str| -> bool {
            matches!(p, "O5" | "O6" | "Q" | "OUT")
                || p.starts_with("CO")
                || (p.starts_with('O') && p[1..].chars().all(|ch| ch.is_ascii_digit()))
        };
        match info.ty.as_str() {
            "SLICE_LUTX" => {
                let memory = pins.iter().any(|(_, p)| p == "CLK" || p == "WE");
                let six = info.bel.ends_with("6LUT");
                let out = if six { "O6" } else { "O5" };
                let lut = if memory {
                    lib.get(
                        "slicem",
                        if six {
                            "LUT_OR_MEM6LRAM"
                        } else {
                            "LUT_OR_MEM5LRAM"
                        },
                        &format!("SLICEM/{}", info.bel),
                    )
                } else {
                    lib.get(
                        "slicel",
                        if six { "LUT6" } else { "LUT5" },
                        &format!("SLICEL/{}", info.bel),
                    )
                };
                let worst = lut
                    .map(|l| l.arcs.iter().map(|a| a.delay.max as i64).max().unwrap_or(0))
                    .unwrap_or(0);
                if lut.is_none() {
                    *rep.stats
                        .untimed
                        .entry(format!("{} {}", info.ty, info.bel))
                        .or_default() += 1;
                }
                let Some(o) = gr.get(c, out) else { continue };
                for (n, p) in &pins {
                    if p.len() == 2 && p.starts_with('A') {
                        let d = lut
                            .and_then(|l| l.arc(p, out))
                            .map(|d| d.max as i64)
                            .unwrap_or(worst);
                        gr.edge(*n, o, d);
                        rep.stats.cell_arcs += 1;
                    } else if memory && (p.starts_with("WA") || p.starts_with("DI") || p == "WE") {
                        end.push((*n, ram_su, dom.clone()));
                    }
                }
            }
            "SLICE_FFX" => {
                for (n, p) in &pins {
                    match p.as_str() {
                        "Q" => start.push((*n, ff_c2q, dom.clone())),
                        "D" => end.push((*n, ff_su_d, dom.clone())),
                        "CE" | "SR" => end.push((*n, ff_su_ce, dom.clone())),
                        _ => {}
                    }
                }
            }
            "CARRY4" => {
                for (n, p) in &pins {
                    if is_out(p) {
                        continue;
                    }
                    for (m, q) in &pins {
                        if !is_out(q) {
                            continue;
                        }
                        let src = if p.starts_with("DI") { carry_o5 } else { carry };
                        if let Some(d) = src.and_then(|cc| cc.arc(p, q)) {
                            gr.edge(*n, *m, d.max as i64);
                            rep.stats.cell_arcs += 1;
                        }
                    }
                }
            }
            "SELMUX2_1" => {
                let mx = lib.get("slicel", "SELMUX2_1", &format!("SLICEL/{}", info.bel));
                let worst = mx
                    .map(|l| l.arcs.iter().map(|a| a.delay.max as i64).max().unwrap_or(0))
                    .unwrap_or(0);
                if let Some(o) = gr.get(c, "OUT") {
                    for (n, p) in &pins {
                        if p != "OUT" {
                            let d = mx
                                .and_then(|l| {
                                    l.arc(&format!("I{p}"), "OUT").or_else(|| l.arc(p, "OUT"))
                                })
                                .map(|d| d.max as i64)
                                .unwrap_or(worst);
                            gr.edge(*n, o, d);
                            rep.stats.cell_arcs += 1;
                        }
                    }
                }
            }
            t if t.starts_with("DSP48") && !clock_of.contains_key(&c) => {
                // Every register bypassed: a combinational block.
                rep.stats.dsp_comb_cells += 1;
                let outs: Vec<(u32, String)> = pins
                    .iter()
                    .filter(|(_, p)| DSP_OUT.contains(&bus(p)))
                    .cloned()
                    .collect();
                for (n, p) in &pins {
                    let b = bus(p);
                    if DSP_OUT.contains(&b) {
                        continue;
                    }
                    for (m, q) in &outs {
                        if let Some(&d) = dsp_comb.get(&(b.to_string(), bus(q).to_string())) {
                            gr.edge(*n, *m, d);
                            rep.stats.cell_arcs += 1;
                        }
                    }
                }
            }
            t if t.starts_with("RAMB") || t.starts_with("DSP48") => {
                let bram = t.starts_with("RAMB");
                if !bram {
                    rep.stats.bounded_cells += 1;
                }
                let c2o = if bram { bram_c2o } else { dsp_c2o };
                for (n, p) in &pins {
                    if p.contains("CLK") {
                        continue;
                    }
                    let out = if bram {
                        p.starts_with("DO") || p.starts_with("CASCADEOUT")
                    } else {
                        DSP_OUT.contains(&bus(p))
                    };
                    if out {
                        start.push((*n, c2o, dom.clone()));
                    } else {
                        let s = if bram {
                            bram_su_by
                                .iter()
                                .find(|(pre, _)| p.starts_with(pre))
                                .map(|x| x.1)
                                .unwrap_or(bram_su)
                        } else {
                            dsp_su
                        };
                        end.push((*n, s, dom.clone()));
                    }
                }
            }
            t => {
                *rep.stats.untimed.entry(t.to_string()).or_default() += 1;
                if t.starts_with("BUFG") {
                    continue;
                }
                for (n, p) in &pins {
                    if clock_pin(p) {
                        continue;
                    }
                    let d = if async_pin(p) {
                        ASYNC.to_string()
                    } else {
                        dom.clone()
                    };
                    if gr.indeg[*n as usize] == 0 && !gr.out[*n as usize].is_empty() {
                        start.push((*n, 0, d));
                    } else if gr.indeg[*n as usize] > 0 && gr.out[*n as usize].is_empty() {
                        end.push((*n, 0, d));
                    }
                }
            }
        }
    }
    let clk = |x: u32| -> i64 { clk_at.get(&gr.names[x as usize].0).copied().unwrap_or(0) };

    // Arrival times, in topological order, from each startpoint's clock
    // arrival plus its clock-to-output.
    let n = gr.names.len();
    let mut arr: Vec<Option<(i64, u32)>> = vec![None; n]; // (arrival, start node)
    let mut pred: Vec<u32> = vec![u32::MAX; n];
    let mut launch: HashMap<u32, Domain> = HashMap::new();
    for (s, d, dom) in &start {
        if dom == ASYNC {
            rep.stats.async_starts += 1;
            continue;
        }
        let d = d + clk(*s);
        if arr[*s as usize].is_none_or(|a| a.0 < d) {
            arr[*s as usize] = Some((d, *s));
        }
        launch.insert(*s, dom.clone());
    }
    let mut indeg = gr.indeg.clone();
    let mut q: VecDeque<u32> = (0..n as u32).filter(|&v| indeg[v as usize] == 0).collect();
    let mut order: Vec<u32> = Vec::with_capacity(n);
    while let Some(v) = q.pop_front() {
        order.push(v);
        let av = arr[v as usize];
        for &(w, d) in &gr.out[v as usize] {
            if let Some((a, s)) = av
                && arr[w as usize].is_none_or(|x| x.0 < a + d)
            {
                arr[w as usize] = Some((a + d, s));
                pred[w as usize] = v;
            }
            indeg[w as usize] -= 1;
            if indeg[w as usize] == 0 {
                q.push_back(w);
            }
        }
    }
    rep.stats.nodes = n;
    rep.stats.in_loops = n - order.len();
    rep.stats.startpoints = start.len();
    rep.stats.endpoints = end.len();

    // Endpoints: worst per domain pair, and the worst same-domain paths.
    let trace = |e: u32| -> Vec<(String, String, i64)> {
        let mut v = Vec::new();
        let mut x = e;
        loop {
            let (c, p) = &gr.names[x as usize];
            v.push((
                imp.cells[*c].name.clone(),
                p.clone(),
                arr[x as usize].map(|a| a.0).unwrap_or(0),
            ));
            let pr = pred[x as usize];
            if pr == u32::MAX {
                break;
            }
            x = pr;
        }
        v.reverse();
        v
    };
    let mut same: Vec<(i64, u32, i64, Domain, i64)> = Vec::new();
    for (e, s, cap) in &end {
        if cap == ASYNC {
            rep.stats.async_ends += 1;
            continue;
        }
        let Some((a, st)) = arr[*e as usize] else {
            continue;
        };
        let lau = launch.get(&st).cloned().unwrap_or_default();
        let ce = clk(*e);
        let total = a + s - ce;
        let skew = clk(st) - ce;
        let key = (lau.clone(), cap.clone());
        if rep.worst.get(&key).is_none_or(|p| p.delay < total) {
            rep.worst.insert(
                key,
                Path {
                    launch: lau.clone(),
                    capture: cap.clone(),
                    delay: total,
                    setup: *s,
                    skew,
                    pins: trace(*e),
                },
            );
        }
        if &lau == cap {
            same.push((total, *e, *s, cap.clone(), skew));
        }
    }
    same.sort_unstable_by(|a, b| b.0.cmp(&a.0));
    for (total, e, s, dom, skew) in same.into_iter().take(opts.paths) {
        rep.paths.push(Path {
            launch: dom.clone(),
            capture: dom,
            delay: total,
            setup: s,
            skew,
            pins: trace(e),
        });
    }

    // Required times: every endpoint by its capture domain's worst path,
    // then backwards over the graph.
    let mut period: HashMap<Domain, i64> = HashMap::new();
    for ((l, c), p) in &rep.worst {
        if l == c && timed(l) {
            period.insert(l.clone(), p.delay);
        }
    }
    let dmax = period.values().copied().max().unwrap_or(0).max(1);
    const INF: i64 = i64::MAX / 4;
    let mut req = vec![INF; n];
    for (e, s, cap) in &end {
        if cap == ASYNC {
            continue;
        }
        let t = period.get(cap).copied().unwrap_or(dmax);
        let r = t - s + clk(*e);
        req[*e as usize] = req[*e as usize].min(r);
    }
    for &v in order.iter().rev() {
        for &(w, d) in &gr.out[v as usize] {
            if req[w as usize] < INF {
                req[v as usize] = req[v as usize].min(req[w as usize] - d);
            }
        }
    }
    rep.crit = imp
        .sink_pins
        .iter()
        .map(|v| vec![0.0; v.len()])
        .collect();
    for &(i, k, from, to, d) in &net_arcs {
        let (Some((a, _)), r) = (arr[from as usize], req[to as usize]) else {
            continue;
        };
        if r >= INF {
            continue;
        }
        let slack = r - (a + d);
        let c = (1.0 - slack as f64 / dmax as f64).clamp(0.0, 1.0) as f32;
        rep.crit[i][k] = c;
        for (j, t) in [0.5f32, 0.8, 0.9, 0.95].iter().enumerate() {
            if c >= *t {
                rep.stats.crit_ge[j] += 1;
            }
        }
    }
    rep
}
