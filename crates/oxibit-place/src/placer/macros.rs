//! Hard macros onto real sites: block RAM, DSP, clock buffers, the PS.
//!
//! These do not go through spreading. A macro whose site type has a
//! single site (the PS) goes there before anything is solved
//! (`place_forced`). The rest each take the free site of their own site
//! type nearest to where a first placement, with macros free to follow
//! the wirelength, put them (`legalise`), and get their bel there and then. A cell with a bel is a fixed pin to everything after
//! (`Problem::build`), so the slices are then placed around the macros
//! rather than the other way round.
//!
//! A macro with no wirelength nets (a clock buffer: its output is a clock
//! net, its input often a single wire) is aimed at the cells driving its
//! inputs instead.
//!
//! Slice-shaped macros (wide-mux trees, carry chains) are not handled
//! here: they need slice sites, which are decided after the slices
//! (`bels::assign`).

use std::collections::{BTreeMap, BTreeSet, HashMap};

use oxibit_pnr::packed::Loc;
use serde_json::{Value, json};

use super::{InstKind, PinMap, PlacementState, Problem, loc_xy};
use crate::device::Device;
use crate::netlist::{CellId, Dir};
use crate::pack::Packer;

/// Packed cell type -> (site type in the device database, bel).
fn home(ty: &str) -> Option<(&'static [&'static str], &'static str)> {
    Some(match ty {
        "RAMB36E1_RAMB36E1" => (&["RAMBFIFO36E1"], "RAMB36E1"),
        // Either half of a block RAM tile takes an 18K RAM.
        "RAMB18E1_RAMB18E1" => (&["RAMB18E1", "FIFO18E1"], "RAMB18E1"),
        "BUFGCTRL" => (&["BUFGCTRL"], "BUFGCTRL"),
        "MMCME2_ADV" => (&["MMCME2_ADV"], "MMCME2_ADV"),
        "PS7_PS7" => (&["PS7"], "PS7"),
        // Not on the validation design: the bel is named by analogy with
        // the others and has not been through the router.
        "DSP48E1_DSP48E1" => (&["DSP48E1"], "DSP48E1"),
        _ => return None,
    })
}

/// Cell types that live in slices.
pub fn slice_shaped(ty: &str) -> bool {
    ty.starts_with("SLICE_") || ty.starts_with("SELMUX") || ty.starts_with("CARRY")
}

#[derive(Debug, Clone, Default)]
pub struct Stats {
    /// Placed without a choice (`place_forced`).
    pub forced: usize,
    /// DSP cascade consumers moved onto their producer's tile-above
    /// (`fix_dsp_cascades`).
    pub dsp_cascades_fixed: usize,
    /// Cell type -> how many were placed.
    pub placed: BTreeMap<String, usize>,
    /// Movable cells this pass has no site type for.
    pub unhandled: Vec<String>,
    /// Tiles between where the first placement had them and their sites.
    pub mean_disp: f64,
    pub max_disp: f64,
    /// Macro moves made afterwards by `refine`, over all its rounds.
    pub refined: usize,
    /// The same per cell type: (cells, sum of tiles, most tiles).
    pub disp_by_type: BTreeMap<String, (usize, f64, f64)>,
}

impl Stats {
    pub fn to_json(&self) -> Value {
        let by_type: BTreeMap<&str, Value> =
            self.disp_by_type.iter().map(|(ty, (n, sum, max))| (ty.as_str(), json!({"mean": sum / *n as f64, "max": max}))).collect();
        json!({"forced": self.forced, "refined": self.refined, "disp_by_type": by_type, "placed": self.placed, "unhandled": self.unhandled, "mean_disp": self.mean_disp, "max_disp": self.max_disp, "dsp_cascades_fixed": self.dsp_cascades_fixed})
    }
}

/// A site as bels name it: tile grid coordinates and the site relative to
/// its tile.
type SiteKey = (i32, i32, String);

fn site_key(dev: &Device, site: &str) -> Option<SiteKey> {
    match dev.grid_loc(site, "")? {
        Loc::Grid { x, y, site, .. } => Some((x, y, site)),
        Loc::Site { .. } => None,
    }
}

/// What putting a cell on `site` takes: the site, and for block RAM the
/// sites of the same tile it shares its memory with (a 36K RAM is the
/// tile's two 18K halves).
fn claims(dev: &Device, site: &str) -> Vec<SiteKey> {
    let mut out: Vec<SiteKey> = site_key(dev, site).into_iter().collect();
    let Some(tile) = dev.tile_of(site) else { return out };
    let whole = |ty: &str| ty == "RAMBFIFO36E1";
    let half = |ty: &str| matches!(ty, "RAMB18E1" | "FIFO18E1");
    let mine = tile.sites.get(site).map(String::as_str).unwrap_or("");
    for (other, ty) in &tile.sites {
        if (whole(mine) && half(ty)) || (half(mine) && whole(ty)) {
            out.extend(site_key(dev, other));
        }
    }
    out
}

/// The sites cells already sit on, with what those take along (`claims`).
fn used_sites(p: &Packer, dev: &Device) -> BTreeSet<SiteKey> {
    let mut used: BTreeSet<SiteKey> = BTreeSet::new();
    for c in p.nl.cell_ids() {
        match &p.nl.cell(c).bel {
            Some(Loc::Grid { x, y, site, .. }) => {
                used.insert((*x, *y, site.clone()));
            }
            Some(Loc::Site { site, .. }) => used.extend(site_key(dev, site)),
            None => {}
        }
    }
    for tile in dev.tiles.values().filter(|t| t.ty.starts_with("BRAM")) {
        for site in tile.sites.keys() {
            if site_key(dev, site).is_some_and(|k| used.contains(&k)) {
                used.extend(claims(dev, site));
            }
        }
    }
    used
}

/// Site type -> its sites (name, x, y), by name.
fn sites_by_type(dev: &Device) -> BTreeMap<&str, Vec<(&str, f64, f64)>> {
    let mut sites: BTreeMap<&str, Vec<(&str, f64, f64)>> = BTreeMap::new();
    for t in dev.tiles.values() {
        for (name, ty) in &t.sites {
            sites.entry(ty.as_str()).or_default().push((name.as_str(), f64::from(t.grid_x), f64::from(t.grid_y)));
        }
    }
    for v in sites.values_mut() {
        v.sort_by(|a, b| a.0.cmp(b.0));
    }
    sites
}

/// One guarded pass over the placed hard macros, for when the slices have
/// been placed around them: each moves to the free site of its type that
/// most shortens its own nets (cell-accurate, pins where `pins` has them),
/// and stays if none does. How many moved. The slices then want placing
/// again.
pub fn refine(p: &mut Packer, dev: &Device, pins: &PinMap, max_fanout: usize) -> usize {
    let mut used = used_sites(p, dev);
    let sites = sites_by_type(dev);
    let mut moved = 0;
    for c in p.nl.cell_ids() {
        let cell = p.nl.cell(c);
        let Some((site_tys, bel)) = home(&cell.ty) else { continue };
        let Some(here) = cell.bel.as_ref().and_then(|l| loc_xy(dev, l)) else { continue };
        // A type with one site has nowhere else to go.
        let candidates: Vec<(&str, f64, f64)> = site_tys.iter().filter_map(|t| sites.get(t)).flatten().copied().collect();
        if candidates.len() < 2 {
            continue;
        }
        // Per net, the box of its other pins: (x0, x1, y0, y1).
        let mut boxes: Vec<(i32, i32, i32, i32)> = Vec::new();
        for (name, port) in &cell.ports {
            let Some(n) = port.net else { continue };
            let clock = name.starts_with("CLK") || p.nl.driver(n).is_some_and(|(d, _)| p.nl.cell(d).ty.starts_with("BUFG"));
            if clock || p.nl.is_const(n) {
                continue;
            }
            let net = p.nl.net(n);
            let mut others: Vec<(i32, i32)> = Vec::new();
            let mut placed = true;
            for (u, uport) in net.users.iter().chain(net.driver.iter()) {
                let other = p.nl.cell(*u);
                if *u == c || other.ty.starts_with("PSEUDO_") {
                    continue;
                }
                match other.bel.as_ref().and_then(|l| loc_xy(dev, l)) {
                    Some(t) => others.push(pins.at(&other.ty, uport).unwrap_or(t)),
                    None => placed = false,
                }
            }
            if placed && !others.is_empty() && others.len() < max_fanout {
                let xs = others.iter().map(|t| t.0);
                let ys = others.iter().map(|t| t.1);
                boxes.push((xs.clone().min().unwrap_or(0), xs.max().unwrap_or(0), ys.clone().min().unwrap_or(0), ys.max().unwrap_or(0)));
            }
        }
        let cost = |x: i32, y: i32| -> i64 { boxes.iter().map(|&(x0, x1, y0, y1)| i64::from((x1.max(x) - x0.min(x)) + (y1.max(y) - y0.min(y)))).sum() };
        let best = candidates
            .iter()
            .filter_map(|&(name, x, y)| site_key(dev, name).map(|k| (cost(x as i32, y as i32), name, k)))
            .filter(|(_, _, k)| !used.contains(k))
            .min_by(|a, b| a.0.cmp(&b.0).then(a.1.cmp(b.1)));
        let Some((_, name, _)) = best.filter(|b| b.0 < cost(here.0, here.1)) else { continue };
        p.nl.cell_mut(c).bel = dev.grid_loc(name, bel);
        used = used_sites(p, dev);
        moved += 1;
    }
    moved
}

/// Fix every unplaced macro whose site type has exactly one site in the
/// device; how many that was.
pub fn place_forced(p: &mut Packer, dev: &Device) -> usize {
    let mut n = 0;
    for c in p.nl.cell_ids() {
        if p.nl.cell(c).bel.is_some() {
            continue;
        }
        let Some((site_tys, bel)) = home(&p.nl.cell(c).ty) else { continue };
        let mut sites = dev.tiles.values().flat_map(|t| t.sites.iter()).filter(|(_, ty)| site_tys.contains(&ty.as_str())).map(|(s, _)| s.as_str());
        if let (Some(only), None) = (sites.next(), sites.next()) {
            p.nl.cell_mut(c).bel = dev.grid_loc(only, bel);
            n += 1;
        }
    }
    n
}

/// Give every hard macro of `prob` a bel on a free site of its type, the
/// nearest to its coordinate in `at`.
pub fn legalise(p: &mut Packer, dev: &Device, prob: &Problem, at: &PlacementState) -> Result<Stats, String> {
    let mut used = used_sites(p, dev);
    let sites = sites_by_type(dev);

    let inst_of: HashMap<CellId, usize> = prob.insts.iter().enumerate().flat_map(|(i, inst)| inst.cells.iter().map(move |&c| (c, i))).collect();
    // Where a cell is now: its bel if it has one, else its instance.
    let whereabouts = |p: &Packer, c: CellId| -> Option<(f64, f64)> {
        match &p.nl.cell(c).bel {
            Some(l) => loc_xy(dev, l).map(|(x, y)| (f64::from(x), f64::from(y))),
            None => inst_of.get(&c).map(|&i| (at.x[i], at.y[i])),
        }
    };

    let mut st = Stats::default();
    let mut disp: Vec<f64> = Vec::new();
    for (i, inst) in prob.insts.iter().enumerate() {
        if inst.kind != InstKind::Macro {
            continue;
        }
        for &c in &inst.cells {
            let ty = p.nl.cell(c).ty.clone();
            let Some((site_tys, bel)) = home(&ty) else {
                if !slice_shaped(&ty) {
                    st.unhandled.push(p.nl.cell(c).name.clone());
                }
                continue;
            };
            let mut want = (at.x[i], at.y[i]);
            if prob.inst_nets[i].is_empty() {
                let drivers: Vec<(f64, f64)> = p
                    .nl
                    .cell(c)
                    .ports
                    .values()
                    .filter(|port| port.dir == Dir::Input)
                    .filter_map(|port| port.net)
                    .filter(|&n| !p.nl.is_const(n))
                    .filter_map(|n| p.nl.driver(n))
                    .filter_map(|(d, _)| whereabouts(p, d))
                    .collect();
                if !drivers.is_empty() {
                    let k = drivers.len() as f64;
                    want = (drivers.iter().map(|d| d.0).sum::<f64>() / k, drivers.iter().map(|d| d.1).sum::<f64>() / k);
                }
            }
            let free = site_tys
                .iter()
                .filter_map(|t| sites.get(t))
                .flatten()
                .filter_map(|&(name, x, y)| site_key(dev, name).map(|k| (name, x, y, k)))
                .filter(|(_, _, _, k)| !used.contains(k))
                .min_by(|a, b| {
                    let d = |s: &(&str, f64, f64, SiteKey)| (s.1 - want.0).abs() + (s.2 - want.1).abs();
                    d(a).total_cmp(&d(b)).then(a.0.cmp(b.0))
                });
            let Some((name, x, y, _)) = free else {
                return Err(format!("no free {} site for {}", site_tys[0], p.nl.cell(c).name));
            };
            used.extend(claims(dev, name));
            p.nl.cell_mut(c).bel = dev.grid_loc(name, bel);
            let d = (x - want.0).abs() + (y - want.1).abs();
            let e = st.disp_by_type.entry(ty.clone()).or_default();
            *e = (e.0 + 1, e.1 + d, e.2.max(d));
            *st.placed.entry(ty).or_default() += 1;
            disp.push(d);
        }
    }
    st.max_disp = disp.iter().copied().fold(0.0, f64::max);
    st.mean_disp = if disp.is_empty() { 0.0 } else { disp.iter().sum::<f64>() / disp.len() as f64 };
    Ok(st)
}


/// After `legalise`, enforce DSP cascade adjacency. The cascade nets are
/// `always` pseudo-pips plus a tileconn union, upward only, with no pips
/// to steer them, so each producer->consumer chain must sit as a rigid
/// snake: cell 0 on DSP_0 of its root tile, cell 1 on DSP_1 of the same
/// tile, cell 2 on DSP_0 of the tile above, cell 3 on its DSP_1, and so
/// on. Chains are re-sited near where they currently sit; non-cascade
/// DSP cells in the way are displaced to free sites. Returns how many
/// cells moved.
pub fn fix_dsp_cascades(p: &mut Packer, dev: &Device) -> usize {
    let pairs: Vec<(CellId, CellId)> = std::mem::take(&mut p.dsp_cascades);
    if pairs.is_empty() { return 0; }
    
    let mut succ: BTreeMap<CellId, CellId> = BTreeMap::new();
    let mut pred: BTreeMap<CellId, CellId> = BTreeMap::new();
    for &(a, b) in &pairs {
        succ.entry(a).or_insert(b);
        pred.insert(b, a);
    }
    let cascade_cells: BTreeSet<CellId> = pairs.iter().flat_map(|(a, b)| [a, b]).copied().collect();
    // DSP tile columns: grid x -> [(grid y, DSP_0 site, DSP_1 site)],
    // sorted ascending y (index 0 = top of the die).
    let mut cols: BTreeMap<i32, Vec<(i32, String, String)>> = BTreeMap::new();
    for t in dev.tiles.values() {
        // A DSP tile has exactly two DSP48E1 sites with absolute names
        // like DSP48_X1Y10 / DSP48_X1Y11; the lower-numbered one is
        // the DSP_0 site (its ACOUT feeds the DSP_1 site's ACIN).
        let mut dsp_sites: Vec<&String> = t
            .sites
            .iter()
            .filter(|(_, ty)| ty.as_str() == "DSP48E1")
            .map(|(n, _)| n)
            .collect();
        if dsp_sites.len() != 2 { continue; }
        dsp_sites.sort_by_key(|n| parse_site_y(n).unwrap_or(0));
        let (s0, s1) = (dsp_sites[0].clone(), dsp_sites[1].clone());
        cols.entry(t.grid_x).or_default().push((t.grid_y, s0, s1));
    }
    for v in cols.values_mut() { v.sort_by_key(|(y, _, _)| *y); }
    // site name -> occupying cell, for every cell on a DSP site now.
    let mut occupied: BTreeMap<String, CellId> = BTreeMap::new();
    for c in p.nl.cell_ids() {
        if let Some(bel) = p.nl.cell(c).bel.clone() {
            let s = site_name(&bel);
            if s.starts_with("DSP48_") { occupied.insert(s, c); }
        }
    }
    let mut claimed: BTreeSet<String> = BTreeSet::new();
    let mut displaced: Vec<CellId> = Vec::new();
    let mut moved = 0;
    let mut roots: Vec<CellId> = succ.keys().copied().filter(|c| !pred.contains_key(c)).collect();
    roots.sort();
    for root in roots {
        let mut chain = vec![root];
        while let Some(&nxt) = succ.get(chain.last().unwrap()) { chain.push(nxt); }
        if chain.len() < 2 { continue; }
        // Column from the root's current site (fallback: centroid).
        let (cx, cy): (i32, i64) = match p.nl.cell(root).bel.clone() {
            Some(Loc::Grid { x, y, .. }) => (x, y as i64),
            _ => {
                let (mut sx, mut sy, mut n) = (0i64, 0i64, 0i64);
                for &c in &chain {
                    if let Some(Loc::Grid { x, y, .. }) = p.nl.cell(c).bel.clone() {
                        sx += x as i64; sy += y as i64; n += 1;
                    }
                }
                if n == 0 { continue; }
                ((sx / n) as i32, sy / n)
            }
        };
        let Some(col) = cols.get(&cx) else { continue };
        let need = chain.len().div_ceil(2);
        if col.len() < need { continue; }
        // The snake's bottom tile index si: col[si] is the root tile, the
        // window si-(need-1)..=si runs upward (grid y decreasing). Pick
        // the free window closest to where the chain sits now.
        let mut best: Option<(i64, usize)> = None;
        for si in (need - 1)..col.len() {
            if col[si].0 as i64 > cy + 40 { break; }
            let mut blocked = false;
            for k in 0..need {
                let (_, s0, s1) = &col[si - k];
                for s in [s0, s1] {
                    if claimed.contains(s) { blocked = true; }
                    if let Some(&oc) = occupied.get(s) {
                        if cascade_cells.contains(&oc) { blocked = true; }
                    }
                }
            }
            if blocked { continue; }
            let dy = (col[si].0 as i64 - cy).abs();
            if best.is_none_or(|(bd, _)| dy < bd) { best = Some((dy, si)); }
        }
        let Some((_, si)) = best else { continue };
        let bel_str = p
            .nl
            .cell(root)
            .bel
            .as_ref()
            .map(bel_bel)
            .unwrap_or_else(|| "DSP48E1".to_string());
        for (i, &c) in chain.iter().enumerate() {
            let (_, s0, s1) = &col[si - i / 2];
            let site = if i % 2 == 0 { s0 } else { s1 };
            if let Some(&oc) = occupied.get(site) {
                if !cascade_cells.contains(&oc) {
                    displaced.push(oc);
                    occupied.remove(site);
                }
            }
            claimed.insert(site.clone());
            let Some(new_bel) = dev.grid_loc(site, &bel_str) else { continue };
            let old = p.nl.cell(c).bel.clone();
            if let Some(o) = &old { occupied.remove(&site_name(o)); }
            if old.as_ref() != Some(&new_bel) { moved += 1; }
            p.nl.cell_mut(c).bel = Some(new_bel);
            occupied.insert(site.clone(), c);
        }
    }
    // Re-home displaced non-cascade cells on any free DSP site.
    for c in displaced {
        let bel_str = p
            .nl
            .cell(c)
            .bel
            .as_ref()
            .map(bel_bel)
            .unwrap_or_else(|| "DSP48E1".to_string());
        'outer: for col in cols.values() {
            for (_, s0, s1) in col {
                for s in [s0, s1] {
                    if !occupied.contains_key(s) && !claimed.contains(s) {
                        if let Some(nb) = dev.grid_loc(s, &bel_str) {
                            p.nl.cell_mut(c).bel = Some(nb);
                            occupied.insert(s.clone(), c);
                        }
                        break 'outer;
                    }
                }
            }
        }
    }
    moved
}

fn site_name(l: &Loc) -> String {
    match l {
        Loc::Grid { site, .. } | Loc::Site { site, .. } => site.clone(),
    }
}
fn bel_bel(l: &Loc) -> String {
    match l {
        Loc::Grid { bel, .. } | Loc::Site { bel, .. } => bel.clone(),
    }
}

fn parse_site_y(s: &str) -> Option<i64> {
    let i = s.rfind("_X")?;
    let y = s[i + 2..].split_once('Y')?.1;
    y.parse().ok()
}
