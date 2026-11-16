//! Verilog to Nexus FASM: synthesis (oxibit-synth, 4-input LUTs), netlist,
//! placement, routing, and the FASM for cells, routes and unused
//! resources. Cell features follow nextpnr's Nexus FASM writer.

use std::collections::{BTreeMap, HashMap, HashSet};

use oxibit_hdl::{elab, parser};
use oxibit_synth::{lib_ports, lower, map};

use crate::emit::Emit;
use crate::graph::Graph;
use crate::netlist::{self, CellKind, Netlist};
use crate::place::{self, Placement};
use crate::route::{self, Routes};
use crate::{Chip, Result, err};

/// Parse, elaborate, lower and map `src` (top module last), then build the
/// Nexus netlist.
pub fn synthesize(src: &str) -> Result<Netlist> {
    let mods = parser::parse(src).map_err(|e| err!("parse: {e}"))?;
    let top = mods.last().ok_or_else(|| err!("no module"))?;
    let m = elab::Elab::module(top).map_err(|e| err!("elaborate: {e}"))?;
    let mut dirs = lib_ports::PrimPorts::new();
    dirs.extend(lib_ports::scan(netlist::PRIMITIVES_V));
    let mut lw = lower::lower(&m, &dirs, lower::Config { carry_min: 0, lut_k: 4 }).map_err(|e| err!("lower: {e}"))?;
    lw.sweep();
    let outs = lw.outputs();
    let mp = map::map_k(&lw.aig, &outs, 4, 2, 2, &oxibit_report::Silent);
    netlist::build(&lw, &mp)
}

/// Pin constraints from a Lattice PDC: `ldc_set_location -site {BALL} [get_ports {PORT}]`.
pub fn parse_pdc(text: &str) -> Result<HashMap<String, String>> {
    let mut pins = HashMap::new();
    for (i, l) in text.lines().enumerate() {
        let l = l.trim();
        if !l.starts_with("ldc_set_location") {
            continue;
        }
        // `{value}` or a bare word after `after`.
        let braced = |s: &str, after: &str| -> Option<String> {
            let p = s.find(after)? + after.len();
            let rest = s[p..].trim_start();
            if let Some(r) = rest.strip_prefix('{') {
                let b = r.find('}')?;
                return Some(r[..b].trim().to_string());
            }
            let w: String = rest.chars().take_while(|c| !c.is_whitespace() && *c != ']').collect();
            (!w.is_empty()).then_some(w)
        };
        let ball = braced(l, "-site").ok_or_else(|| err!("pdc line {}: no -site {{..}}", i + 1))?;
        let port = braced(l, "get_ports").ok_or_else(|| err!("pdc line {}: no get_ports {{..}}", i + 1))?;
        pins.insert(port, ball);
    }
    Ok(pins)
}

fn slice_of(bel_name: &str) -> (&str, char) {
    // SLICEA_LUT0 / SLICEA_FF1 -> ("SLICEA", '0' / '1')
    (&bel_name[..6], bel_name.chars().last().unwrap_or('0'))
}

/// FASM for a placed and routed design.
pub fn write_fasm(chip: &Chip, g: &Graph, nl: &Netlist, pl: &Placement, routes: &Routes, package: &str) -> Result<String> {
    let mut e = Emit::new(chip, g);
    for (ni, arcs) in routes.arcs.iter().enumerate() {
        if arcs.is_empty() {
            continue;
        }
        e.comment(format!("net {}", nl.nets[ni].name));
        e.route(arcs);
        e.out.push('\n');
    }
    // Pins every routed net reaches are used too (sources and sinks).
    for ni in 0..nl.nets.len() {
        for &(c, p) in nl.nets[ni].sinks.iter().chain(nl.nets[ni].driver.iter()) {
            if let Ok(n) = route::pin_node(g, nl, pl, c, p) {
                e.used.insert(n);
            }
        }
    }

    let mut used_cells = HashSet::new();
    let mut vcco: BTreeMap<i32, u32> = BTreeMap::new();
    for (ci, c) in nl.cells.iter().enumerate() {
        let pb = &g.bels[pl.bel[ci]];
        let group = pb.group();
        used_cells.insert(group.clone());
        let tile = chip.tiles[pb.tile as usize].name.replace(':', "__");
        e.comment(format!("cell {} at {group}", c.name));
        match &c.kind {
            CellKind::Lut { init } => {
                let (sl, k) = slice_of(&pb.bel.name);
                e.line(format!("{tile}.{sl}.K{k}.INIT[15:0] = 16'b{init:016b}"));
            }
            CellKind::Ff(cfg) => {
                let (sl, k) = slice_of(&pb.bel.name);
                let r = format!("{tile}.{sl}.REG{k}");
                e.line(format!("{r}.USED.YES"));
                e.line(format!("{r}.REGSET.{}", if cfg.set { "SET" } else { "RESET" }));
                e.line(format!("{r}.LSRMODE.LSR"));
                e.line(format!("{r}.SEL.{}", if pl.ff_on_di.contains(&ci) { "DL" } else { "DF" }));
                let s = format!("{tile}.{sl}");
                e.line(format!("{s}.SRMODE.{}", if cfg.async_sr { "ASYNC" } else { "LSR_OVER_CE" }));
                e.line(format!("{s}.CLKMUX.{}", if cfg.clk_inv { "INV" } else { "CLK" }));
                match (c.input("CE"), cfg.ce_inv) {
                    (Some(_), false) => e.line(format!("{s}.CEMUX.CE")),
                    (Some(_), true) | (None, true) => e.line(format!("{s}.CEMUX.INV")),
                    (None, false) => {}
                }
                match (c.input("LSR"), cfg.lsr_inv) {
                    (Some(_), false) => e.line(format!("{s}.LSRMUX.LSR")),
                    // Unused LSR floats high: invert it to hold reset off.
                    (Some(_), true) | (None, _) => e.line(format!("{s}.LSRMUX.INV")),
                }
            }
            CellKind::Ibuf { .. } | CellKind::Obuf { .. } => {
                let pre = format!("{group}.{}", pb.bel.name);
                let out = matches!(c.kind, CellKind::Obuf { .. });
                e.line(format!("{pre}.BASE_TYPE.{}_LVCMOS33", if out { "OUTPUT" } else { "INPUT" }));
                e.line(format!("{pre}.PULLMODE.{}", if out { "NONE" } else { "UP" }));
                e.line(format!("{pre}.GLITCHFILTER.OFF"));
                e.line(format!("{pre}.SLEWRATE.MED"));
                if out {
                    e.line(format!("{pre}.TMUX.INV"));
                    e.float_pins(pb, &["T", "I3CRESEN", "I3CWKPU"]);
                    let pad = io_bank(chip, pb, package)?;
                    vcco.insert(pad, 3300);
                } else {
                    e.float_pins(pb, &["T", "I", "I3CRESEN", "I3CWKPU"]);
                }
                e.used_io.insert(group.clone());
            }
            CellKind::Osc { hf_div } => {
                let pre = format!("{tile}.{}", pb.bel.name);
                e.line(format!("{pre}.HF_OSC_EN.ENABLED"));
                e.line(format!("{pre}.HFDIV_FABRIC_EN.ENABLED"));
                e.line(format!("{pre}.DTR_EN.ENABLED"));
                e.line(format!("{pre}.DEBUG_N.DISABLED"));
                e.line(format!("{pre}.HF_CLK_DIV[7:0] = 8'b{hf_div:08b}"));
                e.line(format!("{pre}.HF_SED_SEC_DIV[7:0] = 8'b00000001"));
                e.float_pins(pb, &["HFOUTEN"]);
            }
            CellKind::Dcc => {
                e.line(format!("{group}.{}.DCCEN.1", pb.bel.name));
                e.float_pins(pb, &["CE"]);
            }
        }
        e.out.push('\n');
    }
    e.bank_cfg(&vcco);
    e.unused(&used_cells);
    Ok(e.out)
}

/// Bank of the pad a PIO bel bonds to.
fn io_bank(chip: &Chip, pb: &crate::graph::PlacedBel, package: &str) -> Result<i32> {
    let db = chip.db;
    let pio = match pb.bel.name.as_str() {
        "PIOA" => 0,
        "PIOB" => 1,
        n => return Err(err!("{n} is not a PIO")),
    };
    let (w, h) = (db.data.max_col as i32, db.data.max_row as i32);
    let pkg = db.iodb.packages.iter().position(|p| p == package);
    db.iodb
        .pads
        .iter()
        .find(|p| {
            p.pio == pio
                && match p.side.as_str() {
                    "L" => pb.x == 0 && pb.y == p.offset,
                    "R" => pb.x == w && pb.y == p.offset,
                    "T" => pb.y == 0 && pb.x == p.offset,
                    "B" => pb.y == h && pb.x == p.offset,
                    _ => false,
                }
                && pkg.is_none_or(|k| p.pins.get(k).is_some_and(|b| b != "-"))
        })
        .map(|p| p.bank)
        .ok_or_else(|| err!("no pad for {}", pb.group()))
}

/// Everything from Verilog to FASM, with progress lines on stderr.
pub fn run(chip: &Chip, g: &Graph, src: &str, package: &str, pins: &HashMap<String, String>) -> Result<String> {
    let t0 = std::time::Instant::now();
    let nl = synthesize(src)?;
    let luts = nl.count(|k| matches!(k, CellKind::Lut { .. }));
    let ffs = nl.count(|k| matches!(k, CellKind::Ff(_)));
    eprintln!("synth: {luts} LUT4, {ffs} FF, {} cells, {} nets ({:.2}s)", nl.cells.len(), nl.nets.len(), t0.elapsed().as_secs_f64());
    let t1 = std::time::Instant::now();
    let pl = place::place(chip, g, &nl, package, pins)?;
    let plcs: HashSet<u32> = nl
        .cells
        .iter()
        .enumerate()
        .filter(|(_, c)| matches!(c.kind, CellKind::Lut { .. } | CellKind::Ff(_)))
        .map(|(i, _)| g.bels[pl.bel[i]].tile)
        .collect();
    eprintln!("place: {} PLC tiles, {} FFs on DI ({:.2}s)", plcs.len(), pl.ff_on_di.len(), t1.elapsed().as_secs_f64());
    let t2 = std::time::Instant::now();
    let routes = route::route(g, &nl, &pl)?;
    let pips: usize = routes.arcs.iter().map(|a| a.iter().filter(|x| x.kind == crate::graph::ArcKind::Pip).count()).sum();
    eprintln!("route: {pips} pips, {} iterations ({:.2}s)", routes.iterations, t2.elapsed().as_secs_f64());
    write_fasm(chip, g, &nl, &pl, &routes, package)
}
