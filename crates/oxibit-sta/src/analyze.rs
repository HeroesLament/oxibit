//! Setup analysis of a routed design: arrival times over a graph of cell
//! pins, per clock domain.
//!
//! Nodes are (cell, bel pin). Edges:
//! - net arcs, driver pin to each sink pin: the routed tree's interconnect
//!   delay to the sink's node (`delay::arrivals_with`) plus a site pip at
//!   each end ([`Options::site_pip_ps`]), or one site pip for a sink inside
//!   the driver's site;
//! - cell arcs, from the SDF library by cell type and bel: LUT inputs to
//!   O6/O5, CARRY4 inputs to O/CO, F7/F8 mux inputs to OUT.
//!
//! Startpoints are clocked outputs (flop Q at clock-to-Q; BRAM and DSP
//! outputs at the worst clock-to-output of their SDF file, a bound) and the
//! outputs of cells the library does not time (PS7, MMCM), which launch at
//! 0 in their own domain. Endpoints are clocked inputs with their setup
//! (flop D, CE, SR; LUTRAM write pins; BRAM and DSP inputs at the worst
//! setup of their file, a bound). Clocks are ideal: no skew, no
//! clock-tree delay.
//!
//! A path's delay is its arrival at the endpoint plus the endpoint's
//! setup; a domain's Fmax is 1 / its worst same-domain path.

use std::collections::{BTreeMap, HashMap, VecDeque};

use oxibit_fasm::route::Graph;
use oxibit_pnr::delay::{self, Model, TreeEdge};
use oxibit_pnr::nets::{Net, NetClass};
use oxibit_pnr::placed::Import;
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

/// The domain of a startpoint or endpoint: a clock net's name, or the
/// cell type of an untimed cell (`PS7_PS7`).
type Domain = String;

#[derive(Debug, Default)]
pub struct Stats {
    pub nodes: usize,
    pub net_arcs: usize,
    pub cell_arcs: usize,
    pub startpoints: usize,
    pub endpoints: usize,
    /// Nodes on a combinational loop (never ordered).
    pub in_loops: usize,
    /// Cells timed by a bound (BRAM, DSP), and cells with no model.
    pub bounded_cells: usize,
    pub untimed: BTreeMap<String, usize>,
    /// Sink pins no part of their net reached.
    pub unreached_sinks: usize,
}

/// One endpoint's worst path.
#[derive(Debug, Clone)]
pub struct Path {
    pub launch: Domain,
    pub capture: Domain,
    /// Arrival at the endpoint plus its setup, ps.
    pub delay: i64,
    pub setup: i64,
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
}

impl Report {
    pub fn json(&self) -> Value {
        let path = |p: &Path, full: bool| {
            let mut v = json!({"launch": p.launch, "capture": p.capture, "delay_ps": p.delay, "setup_ps": p.setup,
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
                if l == c && p.delay > 0 {
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
                      "untimed": s.untimed, "unreached_sinks": s.unreached_sinks},
        })
    }
}

fn clean(p: &str) -> String {
    p.replace(['[', ']'], "")
}

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

    // Clock domain of each cell: the clock net on its clock pin.
    let mut clock_of: HashMap<usize, Domain> = HashMap::new();
    for (i, n) in imp.nets.iter().enumerate() {
        if n.class != NetClass::GlobalClock {
            continue;
        }
        let name = imp.names[i]
            .split('@')
            .next()
            .unwrap_or_default()
            .to_string();
        for sp in &imp.sink_pins[i] {
            clock_of.entry(sp.cell).or_insert_with(|| name.clone());
        }
    }

    // Routed nets by driver pin: a net split by exit has one part per exit.
    let mut parts: HashMap<&(usize, String), Vec<usize>> = HashMap::new();
    for (i, d) in imp.driver_pins.iter().enumerate() {
        parts.entry(d).or_default().push(i);
    }

    // Net arcs.
    for i in 0..imp.nets.len().min(nets.len()) {
        if nets[i].class != NetClass::Signal {
            continue;
        }
        let edges: Vec<TreeEdge> = routes[i]
            .iter()
            .map(|h| (h.src, h.dst, h.tile, h.pi))
            .collect();
        let at = delay::arrivals_with(g, nets[i].driver.anchor(), &edges, opts.model);
        let (dc, dp) = &imp.driver_pins[i];
        let from = gr.node(*dc, dp);
        for sp in &imp.sink_pins[i] {
            let d = if sp.internal {
                Some(pip)
            } else {
                sp.nodes
                    .iter()
                    .filter_map(|n| at.get(n))
                    .min()
                    .map(|&a| a as i64 + 2 * pip)
            };
            match d {
                Some(d) => {
                    let to = gr.node(sp.cell, &sp.port);
                    gr.edge(from, to, d);
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
    let (dsp_c2o, dsp_su) = file_bounds(lib, "DSP_L");
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
            .unwrap_or_else(|| "unclocked".to_string());
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
            t if t.starts_with("RAMB") || t.starts_with("DSP48") => {
                rep.stats.bounded_cells += 1;
                let (c2o, s) = if t.starts_with("RAMB") {
                    (bram_c2o, bram_su)
                } else {
                    (dsp_c2o, dsp_su)
                };
                for (n, p) in &pins {
                    if p.contains("CLK") {
                        continue;
                    }
                    if gr.indeg[*n as usize] == 0 && !gr.out[*n as usize].is_empty() {
                        start.push((*n, c2o, dom.clone()));
                    } else if gr.indeg[*n as usize] > 0 {
                        end.push((*n, s, dom.clone()));
                    }
                }
            }
            t => {
                *rep.stats.untimed.entry(t.to_string()).or_default() += 1;
                for (n, _) in &pins {
                    if gr.indeg[*n as usize] == 0 && !gr.out[*n as usize].is_empty() {
                        start.push((*n, 0, t.to_string()));
                    } else if gr.indeg[*n as usize] > 0 && gr.out[*n as usize].is_empty() {
                        end.push((*n, 0, t.to_string()));
                    }
                }
            }
        }
    }

    // Arrival times, in topological order.
    let n = gr.names.len();
    let mut arr: Vec<Option<(i64, u32)>> = vec![None; n]; // (arrival, start node)
    let mut pred: Vec<u32> = vec![u32::MAX; n];
    let mut launch: HashMap<u32, Domain> = HashMap::new();
    for (s, d, dom) in &start {
        if arr[*s as usize].is_none_or(|a| a.0 < *d) {
            arr[*s as usize] = Some((*d, *s));
        }
        launch.insert(*s, dom.clone());
    }
    let mut indeg = gr.indeg.clone();
    let mut q: VecDeque<u32> = (0..n as u32).filter(|&v| indeg[v as usize] == 0).collect();
    let mut done = 0usize;
    while let Some(v) = q.pop_front() {
        done += 1;
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
    rep.stats.in_loops = n - done;
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
    let mut same: Vec<(i64, u32, i64, Domain)> = Vec::new();
    for (e, s, cap) in &end {
        let Some((a, st)) = arr[*e as usize] else {
            continue;
        };
        let lau = launch.get(&st).cloned().unwrap_or_default();
        let total = a + s;
        let key = (lau.clone(), cap.clone());
        if rep.worst.get(&key).is_none_or(|p| p.delay < total) {
            rep.worst.insert(
                key,
                Path {
                    launch: lau.clone(),
                    capture: cap.clone(),
                    delay: total,
                    setup: *s,
                    pins: trace(*e),
                },
            );
        }
        if &lau == cap {
            same.push((total, *e, *s, cap.clone()));
        }
    }
    same.sort_unstable_by(|a, b| b.0.cmp(&a.0));
    for (total, e, s, dom) in same.into_iter().take(opts.paths) {
        rep.paths.push(Path {
            launch: dom.clone(),
            capture: dom,
            delay: total,
            setup: s,
            pins: trace(e),
        });
    }
    rep
}
