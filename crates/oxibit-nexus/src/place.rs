//! Placement for small Nexus designs.
//!
//! - IO buffers go to the PIO of their package ball; OSC_CORE to the
//!   device's oscillator.
//! - Each DCC goes to the DCC whose CLKI is nearest (in routing hops) to
//!   its clock source; that hop path is kept as the source net's route.
//! - LUTs and FFs are packed into slices (2 LUT4 + 2 FF; the two FFs share
//!   CLK, CE, LSR and their muxes). An FF whose D comes from a LUT sits on
//!   that LUT's output (DI, `SEL.DL`); others take D from fabric (M).
//! - Slices fill PLC tiles in order of distance from the mean of the
//!   fixed IO locations.
//!
//! No optimisation: this is enough for designs of a few hundred cells.

use std::collections::{BTreeMap, HashMap, HashSet, VecDeque};

use crate::graph::{Arc, Graph, NodeId};
use crate::netlist::{CellId, CellKind, FfCfg, NetId, Netlist};
use crate::{Chip, Result, err};

/// FF control set: what the two FFs of a slice must share.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct CtrlSet {
    pub clk: NetId,
    pub ce: Option<NetId>,
    pub lsr: Option<NetId>,
    pub clk_inv: bool,
    pub ce_inv: bool,
    pub lsr_inv: bool,
    pub async_sr: bool,
}

pub struct Placement {
    /// Graph bel index of each cell.
    pub bel: Vec<usize>,
    /// FFs fed from their slice's LUT (DI, SEL.DL) rather than fabric (M).
    pub ff_on_di: HashSet<CellId>,
    /// Routes fixed during placement (clock source to DCC), by net.
    pub fixed_routes: BTreeMap<NetId, Vec<Arc>>,
}

fn ctrl_of(nl: &Netlist, c: CellId, cfg: &FfCfg) -> Result<CtrlSet> {
    let cell = &nl.cells[c];
    Ok(CtrlSet {
        clk: cell.input("CLK").ok_or_else(|| err!("flop {} has no clock", cell.name))?,
        ce: cell.input("CE"),
        lsr: cell.input("LSR"),
        clk_inv: cfg.clk_inv,
        ce_inv: cfg.ce_inv,
        lsr_inv: cfg.lsr_inv,
        async_sr: cfg.async_sr,
    })
}

#[derive(Default, Clone)]
struct Slice {
    lut: [Option<CellId>; 2],
    ff: [Option<CellId>; 2],
}

/// `pins`: port bit name (`led[3]`) -> package ball (`F13`).
pub fn place(chip: &Chip, g: &Graph, nl: &Netlist, package: &str, pins: &HashMap<String, String>) -> Result<Placement> {
    let db = chip.db;
    let mut bel = vec![usize::MAX; nl.cells.len()];
    let bel_index = |group: &str| g.bel_index(group).ok_or_else(|| err!("no bel {group}"));
    let mut taken: HashSet<usize> = HashSet::new();

    // IO.
    let (w, h) = (db.data.max_col as i32, db.data.max_row as i32);
    let mut anchor = (0i64, 0i64, 0i64);
    for (i, c) in nl.cells.iter().enumerate() {
        let port = match &c.kind {
            CellKind::Ibuf { port } | CellKind::Obuf { port } => port,
            _ => continue,
        };
        let ball = pins.get(port).ok_or_else(|| err!("port {port} has no pin constraint"))?;
        let pad = db.iodb.pad(package, ball).ok_or_else(|| err!("no pad {ball} in {package} (port {port})"))?;
        let (x, y) = match pad.side.as_str() {
            "L" => (0, pad.offset),
            "R" => (w, pad.offset),
            "T" => (pad.offset, 0),
            "B" => (pad.offset, h),
            s => return Err(err!("pad {ball}: side {s:?}")),
        };
        let group = format!("R{y}C{x}_PIO{}", ['A', 'B'][pad.pio as usize & 1]);
        let b = bel_index(&group)?;
        if g.bels[b].bel.beltype != "SEIO33_CORE" {
            return Err(err!("{port} on {ball}: {group} is {}; only 3.3 V IO is supported yet", g.bels[b].bel.beltype));
        }
        if !taken.insert(b) {
            return Err(err!("{port}: pin {ball} used twice"));
        }
        bel[i] = b;
        anchor = (anchor.0 + x as i64, anchor.1 + y as i64, anchor.2 + 1);
    }
    // Oscillator.
    for (i, c) in nl.cells.iter().enumerate() {
        if matches!(c.kind, CellKind::Osc { .. }) {
            let b = (0..g.bels.len())
                .find(|&b| g.bels[b].bel.beltype == "OSC_CORE" && !taken.contains(&b))
                .ok_or_else(|| err!("no free OSC_CORE for {}", c.name))?;
            taken.insert(b);
            bel[i] = b;
        }
    }
    // DCCs: nearest to their source pin, forward through routing.
    let mut fixed_routes = BTreeMap::new();
    for (i, c) in nl.cells.iter().enumerate() {
        if !matches!(c.kind, CellKind::Dcc) {
            continue;
        }
        let src_net = c.input("CLKI").ok_or_else(|| err!("{}: no CLKI", c.name))?;
        let (dc, dp) = nl.nets[src_net].driver.ok_or_else(|| err!("{}: clock source undriven", c.name))?;
        if bel[dc] == usize::MAX {
            return Err(err!("{}: clock source {} must be placed first", c.name, nl.cells[dc].name));
        }
        let start = g.bels[bel[dc]].pin(dp).ok_or_else(|| err!("{}: no pin {dp}", nl.cells[dc].name))?;
        let mut dcc_at: HashMap<NodeId, usize> = HashMap::new();
        for (b, pb) in g.bels.iter().enumerate() {
            if pb.bel.beltype == "DCC" && !taken.contains(&b) {
                if let Some(n) = pb.pin("CLKI") {
                    dcc_at.insert(n, b);
                }
            }
        }
        let mut prev: HashMap<NodeId, Arc> = HashMap::new();
        let mut q = VecDeque::from([start]);
        let mut seen = HashSet::from([start]);
        let mut hit = None;
        while let Some(n) = q.pop_front() {
            if let Some(&b) = dcc_at.get(&n) {
                hit = Some((n, b));
                break;
            }
            for a in g.downhill(n) {
                if seen.insert(a.to) {
                    prev.insert(a.to, *a);
                    q.push_back(a.to);
                }
            }
            if seen.len() > 200_000 {
                break;
            }
        }
        let (n, b) = hit.ok_or_else(|| err!("{}: no DCC reachable from the clock source", c.name))?;
        let mut path = Vec::new();
        let mut cur = n;
        while cur != start {
            let a = prev[&cur];
            path.push(a);
            cur = a.from;
        }
        path.reverse();
        taken.insert(b);
        bel[i] = b;
        fixed_routes.insert(src_net, path);
    }

    // Pack LUTs and FFs into slices.
    let mut ff_on_di = HashSet::new();
    let mut lut_paired: HashSet<CellId> = HashSet::new();
    let mut groups: BTreeMap<CtrlSet, Vec<(Option<CellId>, CellId)>> = BTreeMap::new();
    for (i, c) in nl.cells.iter().enumerate() {
        let CellKind::Ff(cfg) = &c.kind else { continue };
        let ctrl = ctrl_of(nl, i, cfg)?;
        let d = c.input("M").ok_or_else(|| err!("flop {} has no D", c.name))?;
        let lut = nl.nets[d]
            .driver
            .filter(|(dc, _)| matches!(nl.cells[*dc].kind, CellKind::Lut { .. }) && !lut_paired.contains(dc))
            .map(|(dc, _)| dc);
        if let Some(l) = lut {
            lut_paired.insert(l);
            ff_on_di.insert(i);
        }
        groups.entry(ctrl).or_default().push((lut, i));
    }
    let mut slices: Vec<Slice> = Vec::new();
    for (_, members) in groups {
        for pair in members.chunks(2) {
            let mut s = Slice::default();
            for (k, (lut, ff)) in pair.iter().enumerate() {
                s.lut[k] = *lut;
                s.ff[k] = Some(*ff);
            }
            slices.push(s);
        }
    }
    let lone: Vec<CellId> = nl
        .cells
        .iter()
        .enumerate()
        .filter(|(i, c)| matches!(c.kind, CellKind::Lut { .. }) && !lut_paired.contains(i))
        .map(|(i, _)| i)
        .collect();
    let mut lone = lone.into_iter();
    for s in slices.iter_mut() {
        for k in 0..2 {
            if s.lut[k].is_none() {
                s.lut[k] = lone.next();
            }
        }
    }
    let rest: Vec<CellId> = lone.collect();
    for pair in rest.chunks(2) {
        let mut s = Slice::default();
        for (k, l) in pair.iter().enumerate() {
            s.lut[k] = Some(*l);
        }
        slices.push(s);
    }

    // PLC tiles nearest the anchor.
    let (ax, ay) = if anchor.2 > 0 { (anchor.0 / anchor.2, anchor.1 / anchor.2) } else { (w as i64 / 2, h as i64 / 2) };
    let mut plcs: Vec<(i64, u32, u32)> = chip
        .tiles
        .iter()
        .filter(|t| t.tiletype == "PLC")
        .map(|t| ((t.x as i64 - ax).abs() + (t.y as i64 - ay).abs(), t.x, t.y))
        .collect();
    plcs.sort();
    let need = slices.len().div_ceil(4);
    if need > plcs.len() {
        return Err(err!("{} slices do not fit in {} PLCs", slices.len(), plcs.len()));
    }
    for (si, s) in slices.iter().enumerate() {
        let (_, x, y) = plcs[si / 4];
        let sl = ['A', 'B', 'C', 'D'][si % 4];
        for k in 0..2 {
            if let Some(l) = s.lut[k] {
                bel[l] = bel_index(&format!("R{y}C{x}_SLICE{sl}_LUT{k}"))?;
            }
            if let Some(f) = s.ff[k] {
                bel[f] = bel_index(&format!("R{y}C{x}_SLICE{sl}_FF{k}"))?;
            }
        }
    }
    if let Some(i) = bel.iter().position(|&b| b == usize::MAX) {
        return Err(err!("cell {} was not placed", nl.cells[i].name));
    }
    Ok(Placement { bel, ff_on_di, fixed_routes })
}
