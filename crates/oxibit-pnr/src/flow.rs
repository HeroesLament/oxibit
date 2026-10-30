//! Standalone place-and-route back end: from a placed design (nextpnr's
//! placement, `placed::import`), route every net and write the FASM.
//!
//! Every line has an owner (`fasm_out`):
//!
//! - `Route`: the pips of the signal nets, the global clock nets (across the
//!   clock tiles' buffers) and the constant nets (from VCC/GND ties);
//! - `Always`: the `always` pseudo-pips on default constant paths;
//! - `LutInit`: every cell LUT's INIT, permuted for the pins the router
//!   chose;
//! - `SiteMux`: the slice routing-bel settings of the chosen site paths;
//! - `SliceCfg`, `DspTie`: slice FF, control set, LUT mode, carry-in, and
//!   DSP48E1 tie-offs, from the cells;
//! - `Bufg`, `Dsp`, `Mmcm`: BUFGCTRL, DSP48E1 and MMCME2_ADV cell settings;
//! - `Clocking`: the clock-network enables (HCLK buffers, HROW lanes,
//!   GCLK rebuffers, CMT buffer use) of the pips the routes use.
//!
//! `Base` is what is still taken from the design's FASM: the configuration
//! of cells no writer here covers yet (IOB, MMCM, PS7, BUFG, BRAM, DSP
//! attributes), and a cell-less LUT that does logic. The report counts it.

use std::collections::{BTreeMap, HashSet};

use oxibit_fasm::db::Database;
use oxibit_fasm::parse::parse_line;
use oxibit_fasm::route::{EdgeKind, Graph, PipLookup};
use oxibit_report::Progress;
use serde_json::{Value, json};

use crate::{cellcfg, clocking, mmcm};
use crate::fasm_out::{FasmOut, Owner};
use crate::lutinit::{self, Locator};
use crate::nets::{Net, NetClass};
use crate::placed::{self, Import};
use crate::reroute::{self, Check};
use crate::router::{Hop, Opts, Router, Stats};

type R<T> = Result<T, String>;

/// What `standalone` made.
pub struct Standalone {
    pub fasm: FasmOut,
    /// Every net: the import's, then the constant and default nets.
    pub nets: Vec<Net>,
    pub routes: Vec<Vec<Hop>>,
    pub stats: Stats,
    /// The routing checked over the written FASM alone.
    pub check: Check,
    /// One JSON object per phase, in order.
    pub report: Vec<Value>,
}

impl Standalone {
    /// Routed with no overuse, every sink reached, no short, no two writers
    /// in conflict.
    pub fn ok(&self) -> bool {
        self.stats.overused == 0
            && self.stats.unroutable_sinks == 0
            && self.check.sinks_missing == 0
            && self.check.shorts.is_empty()
            && self.fasm.conflicts().is_empty()
    }
}

/// Route `imp` on `g` (built with `GraphOpts { defaults: true }`) and write
/// the design: `placed` is the placed design JSON `imp` came from, `base`
/// the design's FASM for the cell configuration not written here.
pub fn standalone(db: &Database, g: &Graph, placed: &Value, imp: &Import, base: &str, progress: &dyn Progress) -> R<Standalone> {
    standalone_with(db, g, placed, imp, base, Opts::default(), progress)
}

/// `standalone` with the router's options.
pub fn standalone_with(db: &Database, g: &Graph, placed: &Value, imp: &Import, base: &str, opts: Opts, progress: &dyn Progress) -> R<Standalone> {
    let mut report = Vec::new();

    // Nets: the import's (signal, global clock), then the constant inputs.
    let (cnets, missed) = placed::constant_nets(g, imp);
    let by_class = |c: NetClass| imp.nets.iter().filter(|n| n.class == c).count();
    let mut no_tie_by_tile: BTreeMap<String, usize> = BTreeMap::new();
    for &n in &missed {
        let name = g.node_name(n);
        let tile = name.split('/').next().unwrap_or("").split("_X").next().unwrap_or("").to_string();
        *no_tie_by_tile.entry(tile).or_default() += 1;
    }
    report.push(json!({"standalone": {
        "signal_nets": by_class(NetClass::Signal), "clock_nets": by_class(NetClass::GlobalClock),
        "constant_nets": cnets.iter().filter(|n| n.class == NetClass::Constant).count(),
        "default_nets": cnets.iter().filter(|n| n.class == NetClass::Default).count(),
        "default_sinks": cnets.iter().filter(|n| n.class == NetClass::Default).map(|n| n.sinks.len()).sum::<usize>(),
        "constant_sinks": imp.const_sinks.len(),
        "constant_sinks_vcc": imp.const_sinks.iter().filter(|s| s.0).count(),
        "constant_sinks_unique": imp.const_sinks.iter().map(|s| s.1).collect::<HashSet<_>>().len(),
        "constant_internal": imp.const_internal,
        "constant_sinks_no_tie": missed.len(), "no_tie_by_tile": no_tie_by_tile,
        "no_tie_examples": missed.iter().take(6).map(|&n| g.node_name(n)).collect::<Vec<_>>(),
    }}));
    let mut nets = imp.nets.clone();
    nets.extend(cnets);

    let mut router = Router::new(g, opts);
    let (routes, stats) = router.run_with(&nets, progress);
    report.push(json!({"route": stats.to_json(), "nets": nets.len()}));

    let mut fasm = FasmOut::new();
    let loc = Locator::new(db)?;

    // Clock-network enables for the pips the routes use: every hop, and
    // the hard edges of default constant paths (nextpnr counts every pip a
    // net binds).
    let mut used: Vec<clocking::UsedPip> = Vec::new();
    for (n, hops) in nets.iter().zip(&routes) {
        let edges = hops.iter().map(|h| (h.tile, h.pi)).chain(n.edges.iter().filter(|_| n.class == NetClass::Default).map(|e| (e.tile, e.pi)));
        for (t, pi) in edges {
            let p = g.edge_pip(t, pi);
            used.push((p.tile.to_string(), p.dst.to_string(), p.src.to_string()));
        }
    }
    let enables = clocking::clock_enables(used, &loc);
    for l in &enables {
        fasm.add(Owner::Clocking, l);
    }

    // Routing: every net's pips, and the `always` pseudo-pips of default
    // constant paths (as nextpnr writes them: no bits, but a BUFGCTRL's
    // relay names its control pins').
    for (n, hops) in nets.iter().zip(&routes) {
        for h in hops {
            for l in g.edge_pip(h.tile, h.pi).fasm_lines() {
                fasm.add(Owner::Route, &l);
            }
        }
        if n.class == NetClass::Default {
            for e in n.edges.iter().filter(|e| e.kind == EdgeKind::Fixed) {
                fasm.add(Owner::Always, &g.edge_pip(e.tile, e.pi).name());
            }
        }
    }

    // Slice routing bels of the chosen site paths. Before the cells' carry
    // inputs: a carry-in a site path sets is the path's.
    let sm = placed::site_mux_lines(imp);
    for l in &sm.lines {
        fasm.add(Owner::SiteMux, l);
    }

    // LUT INITs from the cells, for the pins the router chose.
    let (cells, skipped) = lutinit::cell_inits(placed, &|b| loc.locate(b))?;
    let moves = lutinit::moves(imp, &routes[..imp.nets.len()])?;
    let (inits, moved) = lutinit::final_inits(&cells, &moves)?;
    let mut written = 0usize;
    for (k, &v) in &inits {
        written += 1;
        if let Some(l) = lutinit::init_line(k, v) {
            fasm.add(Owner::LutInit, &l);
        }
    }

    // Slice FF, control set, LUT mode, carry-in; DSP tie-offs.
    let halves = cellcfg::slice_config(placed, &|b| loc.locate(b))?;
    let (mut from_paths, mut disagree) = (0usize, Vec::new());
    for ((tile, site), lines) in &halves {
        let path_pin = imp.site_muxes.get(&(tile.clone(), site.clone())).and_then(|m| m.get("PRECYINIT"));
        let stem = format!("{tile}.{site}.PRECYINIT.");
        for l in lines {
            if let (Some(pin), Some(v)) = (path_pin, l.strip_prefix(&stem)) {
                from_paths += 1;
                let p = match pin.as_str() {
                    "0" => "C0",
                    "1" => "C1",
                    p => p,
                };
                if p != v {
                    disagree.push(format!("{tile}.{site}: cells {v}, paths {p}"));
                }
                continue;
            }
            fasm.add(Owner::SliceCfg, l);
        }
    }
    let ties = cellcfg::dsp_ties(placed, &|b| loc.locate(b))?;
    for l in &ties {
        fasm.add(Owner::DspTie, l);
    }
    for l in &cellcfg::bufg_config(placed, &|b| loc.locate(b))? {
        fasm.add(Owner::Bufg, l);
    }
    for l in &cellcfg::dsp_config(placed, &|b| loc.locate(b))? {
        fasm.add(Owner::Dsp, l);
    }
    for l in &mmcm::mmcm_config(placed, &|b| loc.locate(b))? {
        fasm.add(Owner::Mmcm, l);
    }

    // Base: the design FASM's lines no writer here owns. Dropped: every pip
    // (the routing is ours), the clock tiles' BUFHCE settings (they follow
    // the routing), the clock-network enables (they follow the routing),
    // slice configuration, LUT INITs of cell LUTs and of route-through
    // LUTs, and every slice routing-bel setting (the site paths set the
    // ones the routes use; another tool's, such as an output mux its own
    // routing used, is no setting of this design's). A cell-less LUT doing
    // logic keeps its INIT.
    let theirs = lutinit::fasm_inits(base)?;
    let (mut base_kept, mut base_dropped, mut cellless_kept, mut routing_luts) = (0usize, 0usize, 0usize, 0usize);
    let mut lut_seen: HashSet<lutinit::LutKey> = HashSet::new();
    for l in base.lines() {
        let f = l.split('#').next().unwrap_or("").trim();
        if f.is_empty() {
            continue;
        }
        let pip = matches!(g.lookup_pip(f), PipLookup::Edge { .. } | PipLookup::SameNode(_));
        if pip
            || f.contains(".BUFHCE.")
            || clocking::is_clock_enable(f, &loc)
            || cellcfg::is_slice_config(f)
            || cellcfg::is_dsp_tie(f)
            || cellcfg::is_bufg_config(f)
            || cellcfg::is_dsp_config(f)
            || mmcm::is_mmcm_config(f)
            || placed::is_site_mux_line(f)
        {
            base_dropped += 1;
            continue;
        }
        if let Some(key) = parse_line(l)?.and_then(|p| lutinit::lut_key(&p.feature)) {
            base_dropped += 1;
            if inits.contains_key(&key) || !lut_seen.insert(key.clone()) {
                continue;
            }
            let init = theirs[&key];
            if lutinit::passes(init).is_some() {
                routing_luts += 1;
            } else {
                cellless_kept += 1;
                fasm.add(Owner::Base, &format!("{}.{}.{}LUT.INIT[63:0] = 64'h{init:016X}", key.0, key.1, key.2));
            }
            continue;
        }
        base_kept += 1;
        fasm.add(Owner::Base, l);
    }

    report.push(json!({"lut_init": {"cells": cells.len(), "skipped": skipped, "written": written, "moved": moved,
                                    "routing_dropped": routing_luts, "cellless_kept": cellless_kept}}));
    report.push(json!({"cell_config": {"slice_lines": fasm.of(Owner::SliceCfg).count(), "dsp_ties": ties.len(),
                                       "precyinit_from_paths": from_paths, "precyinit_disagree": disagree.len(),
                                       "disagree_examples": disagree.iter().take(4).collect::<Vec<_>>()}}));
    report.push(json!({"site_muxes": {"set": sm.set, "written": sm.lines.len()}}));
    let counts: BTreeMap<&str, usize> = fasm.counts().into_iter().map(|(o, n)| (o.name(), n)).collect();
    report.push(json!({"fasm": {"lines_by_owner": counts, "base_kept": base_kept, "base_dropped": base_dropped,
                                "conflicts": fasm.conflicts().len(),
                                "conflict_examples": fasm.conflicts().iter().take(6).map(|c| json!({"key": c.key, "kept": [c.kept.0.name(), &c.kept.1], "refused": [c.refused.0.name(), &c.refused.1]})).collect::<Vec<_>>()}}));

    let text = fasm.text();
    let check = reroute::check(g, &nets, &text);
    report.push(json!({"check": check.to_json(),
                       "missing_examples": check.missing.iter().take(12).map(|&(s, k)| format!("{} -> {}", g.node_name(s), g.node_name(k))).collect::<Vec<_>>()}));
    Ok(Standalone { fasm, nets, routes, stats, check, report })
}
