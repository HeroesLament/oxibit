//! Global placement: coordinates for the packed design, computed instead
//! of imported.
//!
//! The engine is the published analytic one, from the papers:
//!
//! 1. `b2b`: quadratic wirelength in the bound-to-bound net model
//!    (Spindler, Schlichtmann, Johannes, "Kraftwerk2", TCAD 2008), solved
//!    per axis by conjugate gradient (`qp`). Gives the starting point.
//! 2. `nesterov`: weighted-average wirelength plus an electrostatic density
//!    penalty (`density`), minimised by Nesterov's method with the
//!    Lipschitz-predicted step (ePlace: Lu et al., TODAES 2015; RePlAce:
//!    Cheng, Kahng, Kang, Wang, TCAD 2019).
//! 3. `legalise_spread`: every slice instance onto a slice site of its own.
//!
//! What moves is an instance: a slice group from `pack::slices`, or a
//! macro (a cluster or lone cell that is not slice logic: block RAM, clock
//! buffers, carry chains). Cells that packing fixed (I/O) are fixed pins.
//! Coordinates are prjxray tile-grid coordinates, the ones `Loc::Grid`
//! carries.
//!
//! Skeleton limits: only slice instances and memory macros (distributed
//! RAM, shift registers: `Problem::needs_slicem`) take up room and get
//! legalised; other macros follow the wirelength and are left where it
//! puts them. Pins sit
//! at their instance's centre.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use oxibit_pnr::packed::{Design, Loc};
use serde_json::{Value, json};

use crate::device::Device;
use crate::netlist::{CellId, NetId};
use crate::pack::Packer;

pub mod b2b;
pub mod density;
pub mod legalise_spread;
pub mod nesterov;
pub mod qp;
pub mod timing;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rect {
    pub x0: f64,
    pub y0: f64,
    pub x1: f64,
    pub y1: f64,
}

impl Rect {
    pub fn centre(&self) -> (f64, f64) {
        ((self.x0 + self.x1) / 2.0, (self.y0 + self.y1) / 2.0)
    }
}

/// A slice site, at its tile's grid coordinates.
#[derive(Debug, Clone)]
pub struct Site {
    pub x: f64,
    pub y: f64,
    pub name: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InstKind {
    /// One slice's worth of logic: takes a slice site.
    Slice,
    /// Anything else that moves; no room modelled yet.
    Macro,
}

#[derive(Debug, Clone)]
pub struct Inst {
    pub cells: Vec<CellId>,
    pub kind: InstKind,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Pin {
    Inst(u32),
    Fixed(f64, f64),
}

#[derive(Debug, Clone)]
pub struct PNet {
    /// Each instance once; at least two pins, at least one of them movable.
    pub pins: Vec<Pin>,
    pub weight: f64,
}

/// What the placer works on. Built once; placement never changes it.
#[derive(Debug, Clone)]
pub struct Problem {
    pub insts: Vec<Inst>,
    pub nets: Vec<PNet>,
    pub sites: Vec<Site>,
    pub region: Rect,
    /// Per instance, the nets it is on.
    pub inst_nets: Vec<Vec<u32>>,
    /// Per net, the netlist net it stands for (`build`); empty for a
    /// problem made from parts.
    pub net_ids: Vec<NetId>,
    /// Per site, whether it is a SLICEM; per instance, whether it is a
    /// macro that must have one (distributed RAM, shift register), which
    /// legalisation sites before any slice (`legalise_spread`). Both
    /// `build`'s; empty from parts.
    pub site_slicem: Vec<bool>,
    pub needs_slicem: Vec<bool>,
    /// Per instance, the slices a carry chain stacks up one column (its
    /// CARRY4 rows); 0 for anything else. `build`'s; empty from parts.
    pub chain_rows: Vec<usize>,
    /// Per site, the slice a carry chain continues into: the same slice
    /// column one row up (`SLICE_XaYb` -> `SLICE_XaY(b+1)`) in the same
    /// grid column, at most two grid rows up (an HCLK or BRKH row between
    /// them passes COUT to CIN). `None` where the column ends.
    pub site_up: Vec<Option<u32>>,
}

/// `SLICE_X12Y34` -> (12, 34).
fn slice_xy(name: &str) -> Option<(i64, i64)> {
    let (x, y) = name.strip_prefix("SLICE_X")?.split_once('Y')?;
    Some((x.parse().ok()?, y.parse().ok()?))
}

/// `Problem::site_up` for `sites`.
fn site_up(sites: &[Site]) -> Vec<Option<u32>> {
    let index: HashMap<(i64, i64), u32> = sites.iter().enumerate().filter_map(|(i, s)| slice_xy(&s.name).map(|xy| (xy, i as u32))).collect();
    sites
        .iter()
        .map(|s| {
            let (x, y) = slice_xy(&s.name)?;
            let &t = index.get(&(x, y + 1))?;
            let up = &sites[t as usize];
            let rows = s.y - up.y;
            (up.x == s.x && (1.0..=2.0).contains(&rows)).then_some(t)
        })
        .collect()
}

/// A coordinate per instance.
#[derive(Debug, Clone, PartialEq)]
pub struct PlacementState {
    pub x: Vec<f64>,
    pub y: Vec<f64>,
}

impl Problem {
    /// From its parts, deriving the region and the instance-to-net index.
    pub fn from_parts(insts: Vec<Inst>, nets: Vec<PNet>, sites: Vec<Site>) -> Problem {
        let fold = |f: fn(f64, f64) -> f64, init: f64, get: fn(&Site) -> f64| sites.iter().map(get).fold(init, f);
        let region = Rect {
            x0: fold(f64::min, f64::INFINITY, |s| s.x) - 0.5,
            y0: fold(f64::min, f64::INFINITY, |s| s.y) - 0.5,
            x1: fold(f64::max, f64::NEG_INFINITY, |s| s.x) + 0.5,
            y1: fold(f64::max, f64::NEG_INFINITY, |s| s.y) + 0.5,
        };
        let mut inst_nets = vec![Vec::new(); insts.len()];
        for (e, net) in nets.iter().enumerate() {
            for pin in &net.pins {
                if let Pin::Inst(i) = pin {
                    inst_nets[*i as usize].push(e as u32);
                }
            }
        }
        Problem {
            insts,
            nets,
            region,
            inst_nets,
            net_ids: Vec::new(),
            site_slicem: Vec::new(),
            needs_slicem: Vec::new(),
            chain_rows: Vec::new(),
            site_up: site_up(&sites),
            sites,
        }
    }

    /// The `n` sites a chain rooted at `root` takes, bottom first, if the
    /// column holds that many from there.
    pub fn chain_run(&self, root: u32, n: usize) -> Option<Vec<u32>> {
        let mut run = Vec::with_capacity(n);
        let mut s = root;
        run.push(s);
        while run.len() < n {
            s = self.site_up.get(s as usize).copied().flatten()?;
            run.push(s);
        }
        Some(run)
    }

    /// From a packed design whose slices are packed (`pack_slices`). Nets
    /// with more than `max_fanout` pins, clock nets and constants carry no
    /// wirelength. A pin of a fixed cell is at the cell's tile unless
    /// `pins` places it elsewhere.
    pub fn build(p: &Packer, dev: &Device, max_fanout: usize, pins: &PinMap) -> Result<Problem, String> {
        let mut sites: Vec<Site> = Vec::new();
        for t in dev.tiles.values() {
            for (name, ty) in &t.sites {
                if ty.starts_with("SLICE") {
                    sites.push(Site {
                        x: f64::from(t.grid_x),
                        y: f64::from(t.grid_y),
                        name: name.clone(),
                    });
                }
            }
        }
        if sites.is_empty() {
            return Err("device has no slice sites".into());
        }
        sites.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)).then(a.name.cmp(&b.name)));

        let nl = &p.nl;
        let mut insts: Vec<Inst> = Vec::new();
        let mut inst_of: HashMap<CellId, u32> = HashMap::new();
        for g in &p.slices {
            for &c in &g.cells {
                inst_of.insert(c, insts.len() as u32);
            }
            insts.push(Inst {
                cells: g.cells.clone(),
                kind: InstKind::Slice,
            });
        }
        // The rest that is not fixed: one macro per cluster or lone cell.
        let mut macro_of: BTreeMap<CellId, u32> = BTreeMap::new();
        for c in nl.cell_ids() {
            let cell = nl.cell(c);
            if inst_of.contains_key(&c) || cell.bel.is_some() || cell.ty.starts_with("PSEUDO_") {
                continue;
            }
            let root = p.cluster_of.get(&c).copied().unwrap_or(c);
            let i = *macro_of.entry(root).or_insert_with(|| {
                insts.push(Inst {
                    cells: Vec::new(),
                    kind: InstKind::Macro,
                });
                insts.len() as u32 - 1
            });
            insts[i as usize].cells.push(c);
            inst_of.insert(c, i);
        }

        // Net -> movable pins and fixed pins (grid coordinates).
        type NetPins = (Vec<u32>, Vec<(i32, i32)>);
        let mut on: BTreeMap<NetId, NetPins> = BTreeMap::new();
        let mut skip: BTreeSet<NetId> = BTreeSet::new();
        for c in nl.cell_ids() {
            let cell = nl.cell(c);
            let fixed = match &cell.bel {
                Some(Loc::Grid { x, y, .. }) => Some((*x, *y)),
                Some(Loc::Site { site, .. }) => dev.tile_of(site).map(|t| (t.grid_x, t.grid_y)),
                None => None,
            };
            for (name, port) in &cell.ports {
                let Some(n) = port.net else { continue };
                if nl.is_const(n) {
                    continue;
                }
                if name == "CK" || name == "CLK" || cell.ty.starts_with("BUFG") {
                    skip.insert(n);
                }
                let e = on.entry(n).or_default();
                match (inst_of.get(&c), fixed.map(|f| pins.at(&cell.ty, name).unwrap_or(f))) {
                    (Some(&i), _) => e.0.push(i),
                    (None, Some(f)) => e.1.push(f),
                    (None, None) => {}
                }
            }
        }
        let mut nets: Vec<PNet> = Vec::new();
        let mut net_ids: Vec<NetId> = Vec::new();
        for (n, (mut mov, mut fixed)) in on {
            mov.sort_unstable();
            mov.dedup();
            fixed.sort_unstable();
            fixed.dedup();
            let pins = mov.len() + fixed.len();
            if skip.contains(&n) || mov.is_empty() || pins < 2 || pins > max_fanout {
                continue;
            }
            nets.push(PNet {
                pins: mov
                    .into_iter()
                    .map(Pin::Inst)
                    .chain(fixed.into_iter().map(|(x, y)| Pin::Fixed(f64::from(x), f64::from(y))))
                    .collect(),
                weight: 1.0,
            });
            net_ids.push(n);
        }
        let site_slicem: Vec<bool> = sites.iter().map(|s| dev.site_type(&s.name) == Some("SLICEM")).collect();
        let needs_slicem: Vec<bool> = insts
            .iter()
            .map(|inst| {
                let cell = |c: &CellId| nl.cell(*c);
                inst.kind == InstKind::Macro
                    && inst.cells.iter().all(|c| macros::slice_shaped(&cell(c).ty))
                    && inst.cells.iter().any(|c| cell(c).attrs.contains_key("X_LUT_AS_DRAM") || cell(c).attrs.contains_key("X_LUT_AS_SRL"))
            })
            .collect();
        // A carry chain's root is its first CARRY4; its rows, from the
        // members' row offsets.
        let chain_rows: Vec<usize> = insts
            .iter()
            .map(|inst| {
                if inst.kind != InstKind::Macro {
                    return 0;
                }
                let Some(&root) = inst.cells.first().and_then(|c| p.cluster_of.get(c)) else { return 0 };
                if nl.cell(root).ty != "CARRY4" {
                    return 0;
                }
                1 + p.clusters[&root].members.iter().map(|(_, o)| crate::pack::carry::chain_row(o.dy)).max().unwrap_or(0)
            })
            .collect();
        let mut prob = Problem::from_parts(insts, nets, sites);
        prob.net_ids = net_ids;
        prob.site_slicem = site_slicem;
        prob.needs_slicem = needs_slicem;
        prob.chain_rows = chain_rows;
        Ok(prob)
    }

    /// Slice sites left for slice logic once carry chains and memory
    /// macros have theirs. Meaningful before slices are packed too: loose
    /// cells then count as neither.
    pub fn slice_room(&self) -> usize {
        let chains: usize = self.chain_rows.iter().sum();
        let memories = self.needs_slicem.iter().filter(|&&m| m).count();
        self.sites.len().saturating_sub(chains + memories)
    }

    /// Whether instance `i` needs a slice site of its own, so takes room in
    /// global placement: a slice, a memory macro (`needs_slicem`), or a
    /// carry chain (`chain_rows`).
    pub fn takes_site(&self, i: usize) -> bool {
        self.site_area(i) > 0.0
    }

    /// Where instance `i`'s charge sits relative to its position, one unit
    /// each: a carry chain is a bar of its rows, one grid row apart,
    /// centred on it (a chain's rows are one slice up the column each);
    /// anything else that takes a site is a point.
    pub fn footprint(&self, i: usize) -> impl Iterator<Item = f64> + Clone {
        let n = match self.chain_rows.get(i) {
            Some(&n) if n > 1 => n,
            _ => 1,
        };
        let mid = (n as f64 - 1.0) / 2.0;
        (0..n).map(move |k| k as f64 - mid)
    }

    /// Slice sites instance `i` takes: 1 for a slice or memory macro, a
    /// carry chain's rows, 0 for anything else. Its charge in global
    /// placement's density (`nesterov`, `density::overflow`).
    pub fn site_area(&self, i: usize) -> f64 {
        match self.chain_rows.get(i) {
            Some(&n) if n > 0 => n as f64,
            _ if self.insts[i].kind == InstKind::Slice || self.needs_slicem.get(i) == Some(&true) => 1.0,
            _ => 0.0,
        }
    }

    pub fn pin_pos(&self, pin: Pin, st: &PlacementState) -> (f64, f64) {
        match pin {
            Pin::Inst(i) => (st.x[i as usize], st.y[i as usize]),
            Pin::Fixed(x, y) => (x, y),
        }
    }

    /// Half-perimeter wirelength of one net.
    pub fn net_hpwl(&self, e: u32, st: &PlacementState) -> f64 {
        let net = &self.nets[e as usize];
        let (mut x0, mut x1, mut y0, mut y1) = (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
        for &pin in &net.pins {
            let (x, y) = self.pin_pos(pin, st);
            (x0, x1, y0, y1) = (x0.min(x), x1.max(x), y0.min(y), y1.max(y));
        }
        net.weight * ((x1 - x0) + (y1 - y0))
    }

    pub fn hpwl(&self, st: &PlacementState) -> f64 {
        (0..self.nets.len() as u32).map(|e| self.net_hpwl(e, st)).sum()
    }

    /// Wirelength over the nets whose instances are all `known`, and how
    /// many nets that is: for comparing against a placement that does not
    /// cover every instance.
    pub fn hpwl_known(&self, st: &PlacementState, known: &[bool]) -> (f64, usize) {
        let covered = |e: &u32| {
            self.nets[*e as usize].pins.iter().all(|p| match p {
                Pin::Inst(i) => known[*i as usize],
                Pin::Fixed(..) => true,
            })
        };
        let nets: Vec<u32> = (0..self.nets.len() as u32).filter(covered).collect();
        (nets.iter().map(|&e| self.net_hpwl(e, st)).sum(), nets.len())
    }

    /// Where another placement of the same netlist put each instance (the
    /// mean of its cells found there by name), and which instances it
    /// covers.
    pub fn reference_state(&self, p: &Packer, theirs: &Design, dev: &Device) -> (PlacementState, Vec<bool>) {
        let n = self.insts.len();
        let (cx, cy) = self.region.centre();
        let mut st = PlacementState {
            x: vec![cx; n],
            y: vec![cy; n],
        };
        let mut known = vec![false; n];
        for (i, inst) in self.insts.iter().enumerate() {
            let (mut sx, mut sy, mut k) = (0.0, 0.0, 0.0);
            for &c in &inst.cells {
                let at = match theirs.cells.get(&p.nl.cell(c).name).and_then(|t| t.loc.as_ref()) {
                    Some(Loc::Grid { x, y, .. }) => Some((*x, *y)),
                    Some(Loc::Site { site, .. }) => dev.tile_of(site).map(|t| (t.grid_x, t.grid_y)),
                    None => None,
                };
                if let Some((x, y)) = at {
                    (sx, sy, k) = (sx + f64::from(x), sy + f64::from(y), k + 1.0);
                }
            }
            if k > 0.0 {
                (st.x[i], st.y[i], known[i]) = (sx / k, sy / k, true);
            }
        }
        (st, known)
    }
}

#[derive(Debug, Clone, Default)]
pub struct GpOpts {
    pub qp: b2b::QpOpts,
    pub nesterov: nesterov::Opts,
    /// `place_design`: rounds of macro refinement (`macros::refine`) each
    /// followed by placing the slices again; 0 for none. Ours, not swept.
    pub macro_rounds: usize,
    /// `place_design`: timing-weighted nets (`timing`); off by default.
    pub timing: timing::TimingOpts,
    /// `place_design`: the timing weights' wire delays from this measured
    /// grid rather than `timing::wire_ps`; `None` for the built-in table.
    pub delay_grid: Option<std::sync::Arc<timing::DelayGrid>>,
    /// Sites withheld where an earlier route was congested; `None` for none.
    pub congest: Option<std::sync::Arc<density::Congest>>,
}

/// The run, stage by stage.
#[derive(Debug, Clone)]
pub struct GpReport {
    pub slice_insts: usize,
    pub macro_insts: usize,
    pub nets: usize,
    pub sites: usize,
    /// After the quadratic start (overlapping).
    pub hpwl_quadratic: f64,
    /// After Nesterov (spread, not yet on sites).
    pub hpwl_global: f64,
    /// On sites, before and after the guarded refinement.
    pub hpwl_spread: f64,
    pub hpwl_legal: f64,
    pub nesterov: nesterov::Report,
    pub legal: legalise_spread::Stats,
}

impl GpReport {
    pub fn to_json(&self) -> Value {
        json!({"slice_insts": self.slice_insts, "macro_insts": self.macro_insts, "nets": self.nets, "sites": self.sites,
               "hpwl": {"quadratic": self.hpwl_quadratic, "global": self.hpwl_global, "spread": self.hpwl_spread, "legal": self.hpwl_legal},
               "nesterov": self.nesterov.to_json(), "legal": self.legal.to_json()})
    }
}

/// A finished placement: coordinates, and the slice site of each slice
/// instance (`None` for macros).
#[derive(Debug, Clone)]
pub struct Placement {
    pub state: PlacementState,
    pub site_of: Vec<Option<u32>>,
    pub report: GpReport,
}

/// Quadratic start, Nesterov global placement, spreading legalisation.
pub fn place(prob: &Problem, opts: &GpOpts) -> Result<Placement, String> {
    place_timed(prob, opts, None)
}

/// `place`, the Nesterov stage's nets weighted by `timing`.
pub fn place_timed(prob: &Problem, opts: &GpOpts, timing: Option<&mut timing::PlaceTiming>) -> Result<Placement, String> {
    let start = b2b::initial_place(prob, &opts.qp, &qp::Cg::default());
    let hpwl_quadratic = prob.hpwl(&start);
    let grid = density::BinGrid::new(prob, opts.nesterov.bins);
    let withheld = opts.congest.as_ref().map(|c| grid.withheld(c));
    let (global, nesterov) = nesterov::run_withheld(prob, &grid, start, &opts.nesterov, timing, withheld.as_deref());
    let hpwl_global = prob.hpwl(&global);
    let legal = legalise_spread::legalise_spread(prob, &grid, &global)?;
    let slice_insts = prob.insts.iter().filter(|i| i.kind == InstKind::Slice).count();
    Ok(Placement {
        report: GpReport {
            slice_insts,
            macro_insts: prob.insts.len() - slice_insts,
            nets: prob.nets.len(),
            sites: prob.sites.len(),
            hpwl_quadratic,
            hpwl_global,
            hpwl_spread: legal.stats.hpwl_spread,
            hpwl_legal: legal.stats.hpwl,
            nesterov,
            legal: legal.stats.clone(),
        },
        state: legal.state,
        site_of: legal.site_of,
    })
}

pub mod bels;
pub mod macros;

/// Where the pins of large fixed cells really are: cell type -> port ->
/// grid tile. Without it every pin of a cell is at the cell's one tile,
/// which for the PS is tens of tiles from where its nets enter and leave
/// the interconnect (measured on `libresdr_radio`: 66 tiles for the pins
/// its block RAMs drive).
#[derive(Debug, Clone, Default)]
pub struct PinMap(HashMap<String, HashMap<String, (i32, i32)>>);

impl PinMap {
    pub fn at(&self, ty: &str, port: &str) -> Option<(i32, i32)> {
        self.0.get(ty)?.get(port).copied()
    }

    /// Pins known.
    pub fn len(&self) -> usize {
        self.0.values().map(HashMap::len).sum()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// The PS7's pins from what `oxibit-pnr`'s `ps_entries` example writes
    /// (routing node -> where it meets the interconnect, `in` or `out`): a
    /// PS7 port `P` is the node `<PSS tile>/PS7_P`.
    pub fn from_ps_entries(v: &Value) -> PinMap {
        let mut ps: HashMap<String, (i32, i32)> = HashMap::new();
        for (node, e) in v.as_object().into_iter().flatten() {
            let Some((_, port)) = node.split_once("/PS7_") else { continue };
            let xy = [&e["in"], &e["out"]].into_iter().find_map(|a| Some((a.get(0)?.as_i64()? as i32, a.get(1)?.as_i64()? as i32)));
            if let Some(xy) = xy {
                ps.insert(port.to_string(), xy);
            }
        }
        PinMap(HashMap::from([("PS7_PS7".to_string(), ps)]))
    }
}

/// The tile-grid coordinates of a bel location.
pub fn loc_xy(dev: &Device, loc: &Loc) -> Option<(i32, i32)> {
    match loc {
        Loc::Grid { x, y, .. } => Some((*x, *y)),
        Loc::Site { site, .. } => dev.tile_of(site).map(|t| (t.grid_x, t.grid_y)),
    }
}

/// Cell-accurate half-perimeter wirelength, net by net, of a placement of
/// `p`'s netlist: every cell at the tile `at` gives for it. Constants,
/// clock nets, and nets of more than `max_fanout` cells are left out, and
/// so is any net with a cell `at` has no tile for. A pin `pins` knows is
/// counted where `pins` has it, not at its cell. Two placements of the
/// same netlist are compared over the nets both maps hold
/// (`common_hpwl`).
pub fn cell_net_hpwl(p: &Packer, max_fanout: usize, pins: &PinMap, at: &dyn Fn(CellId) -> Option<(i32, i32)>) -> BTreeMap<NetId, f64> {
    let nl = &p.nl;
    // Net -> its cells' tiles; `None` once a cell is unplaced or the net
    // is a clock.
    let mut on: BTreeMap<NetId, Option<Vec<(i32, i32)>>> = BTreeMap::new();
    for c in nl.cell_ids() {
        let cell = nl.cell(c);
        if cell.ty.starts_with("PSEUDO_") {
            continue;
        }
        let xy = at(c);
        for (name, port) in &cell.ports {
            let Some(n) = port.net else { continue };
            if nl.is_const(n) {
                continue;
            }
            let e = on.entry(n).or_insert_with(|| Some(Vec::new()));
            let clock = name == "CK" || name == "CLK" || cell.ty.starts_with("BUFG");
            match (xy.map(|t| pins.at(&cell.ty, name).unwrap_or(t)), e.as_mut()) {
                (Some(xy), Some(v)) if !clock => v.push(xy),
                _ => *e = None,
            }
        }
    }
    on.into_iter()
        .filter_map(|(n, v)| {
            let v = v?;
            if v.len() < 2 || v.len() > max_fanout {
                return None;
            }
            let span = |get: fn(&(i32, i32)) -> i32| v.iter().map(get).max().unwrap_or(0) - v.iter().map(get).min().unwrap_or(0);
            Some((n, f64::from(span(|t| t.0) + span(|t| t.1))))
        })
        .collect()
}

/// Both wirelengths over the nets both maps hold, and how many that is.
pub fn common_hpwl(a: &BTreeMap<NetId, f64>, b: &BTreeMap<NetId, f64>) -> (f64, f64, usize) {
    let both: Vec<(f64, f64)> = a.iter().filter_map(|(n, x)| b.get(n).map(|y| (*x, *y))).collect();
    (both.iter().map(|v| v.0).sum(), both.iter().map(|v| v.1).sum(), both.len())
}

#[derive(Debug, Clone)]
pub struct DesignReport {
    pub macros: macros::Stats,
    pub gp: GpReport,
    pub bels: bels::Stats,
}

impl DesignReport {
    pub fn to_json(&self) -> Value {
        let mut v = self.gp.to_json();
        v["macros"] = self.macros.to_json();
        v["bels"] = self.bels.to_json();
        v
    }
}

/// Place a packed design (`pack_slices` done) and write every bel:
///
/// 1. single-site macros (the PS) to their site; a first full placement
///    with the other macros free, then those to the nearest free sites
///    of their type (`macros`);
/// 2. with the macros now fixed, the full placement again for the slices
///    (`place`);
/// 3. bels within the slices (`bels`);
/// 4. `GpOpts::macro_rounds` rounds of moving each macro to a better free
///    site if there is one (`macros::refine`) and repeating 2 and 3.
///
/// `place::legalise` still has to run afterwards (PS7 input ties).
pub fn place_design(p: &mut Packer, dev: &Device, opts: &GpOpts, max_fanout: usize, pins: &PinMap) -> Result<DesignReport, String> {
    // Timing, when on, weighs each problem's nets from its own graph.
    let place_with = |p: &Packer, prob: &Problem| -> Result<Placement, String> {
        if opts.timing.every == 0 {
            return place(prob, opts);
        }
        let mut t = timing::PlaceTiming::build(p, prob, opts.timing).with_grid(opts.delay_grid.clone());
        place_timed(prob, opts, Some(&mut t))
    };
    let forced = macros::place_forced(p, dev);
    let first = Problem::build(p, dev, max_fanout, pins)?;
    let rough = place_with(p, &first)?;
    let mut macros = macros::legalise(p, dev, &first, &rough.state)?;
    macros.forced = forced;
    let prob = Problem::build(p, dev, max_fanout, pins)?;
    let pl = place_with(p, &prob)?;
    let mut bels = bels::assign(p, dev, &prob, &pl.site_of)?;
    bels::assign_macros(p, dev, &prob, &pl.site_of, &pl.state, &mut bels);
    let mut gp = pl.report;
    // 4. The macros were sited before the slices settled around them: let
    //    each move where its nets are now shorter, and place the slices
    //    again, until none moves.
    for _ in 0..opts.macro_rounds {
        let moved = macros::refine(p, dev, pins, max_fanout);
        if moved == 0 {
            break;
        }
        macros.refined += moved;
        bels::clear(p);
        let prob = Problem::build(p, dev, max_fanout, pins)?;
        let pl = place_with(p, &prob)?;
        bels = bels::assign(p, dev, &prob, &pl.site_of)?;
        bels::assign_macros(p, dev, &prob, &pl.site_of, &pl.state, &mut bels);
        gp = pl.report;
    }
    // 5. DSP cascade adjacency last: refinement may have moved the DSPs,
    //    and the cascade nets (always pseudo-pips + tileconn, upward
    //    only) leave no wiggle room, so re-site consumers once more.
    macros.dsp_cascades_fixed = macros::fix_dsp_cascades(p, dev);
    // It moved cells after the last look: check for collisions again.
    bels::audit(p, &mut bels);
    Ok(DesignReport { macros, gp, bels })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::Tile;
    use crate::netlist::{Dir, Netlist};
    use crate::pack::slices::SliceGroup;

    #[test]
    fn pin_positions_reach_the_problem_and_the_reported_wirelength() {
        // A slice LUT at (50, 57) talking both ways to a PS whose site is at
        // (32, 51) and whose two pins meet the interconnect elsewhere.
        let mut dev = Device::default();
        dev.site_tile.insert("SLICE_X0Y0".into(), "CLBLL_L_X2Y1".into());
        dev.tiles.insert(
            "CLBLL_L_X2Y1".into(),
            Tile {
                name: "CLBLL_L_X2Y1".into(),
                ty: "CLBLL_L".into(),
                grid_x: 50,
                grid_y: 57,
                clock_region: None,
                sites: [("SLICE_X0Y0".to_string(), "SLICEL".to_string())].into_iter().collect(),
            },
        );
        let mut p = Packer::new(Netlist::new("t"));
        let (from_ps, to_ps) = (p.nl.add_net("from_ps"), p.nl.add_net("to_ps"));
        let ps = p.nl.add_cell("ps", "PS7_PS7");
        p.nl.add_port(ps, "OUTP", Dir::Output);
        p.nl.connect(ps, "OUTP", from_ps);
        p.nl.connect(ps, "INP", to_ps);
        p.nl.cell_mut(ps).bel = Some(Loc::Grid {
            x: 32,
            y: 51,
            site: "PS7_X0Y0".into(),
            bel: "PS7".into(),
        });
        let lut = p.nl.add_cell("lut", "SLICE_LUTX");
        p.nl.connect(lut, "A1", from_ps);
        p.nl.add_port(lut, "O6", Dir::Output);
        p.nl.connect(lut, "O6", to_ps);
        p.slices = vec![SliceGroup {
            cells: vec![lut],
            luts: 1,
            ffs: 0,
        }];

        // As `ps_entries` writes them: PS7 pins among other PS wires.
        let pins = PinMap::from_ps_entries(&json!({
            "PSS2_X32Y105/PS7_OUTP": {"in": null, "out": [51, 57]},
            "PSS2_X32Y105/PS7_INP": {"in": [52, 61], "out": null},
            "PSS2_X32Y105/PSS2_LOGIC_OUTS0": {"in": null, "out": [51, 60]},
            "PSS2_X32Y105/PS7_NOWHERE": {"in": null, "out": null},
        }));
        assert_eq!(pins.len(), 2);
        assert_eq!((pins.at("PS7_PS7", "OUTP"), pins.at("PS7_PS7", "INP")), (Some((51, 57)), Some((52, 61))));
        assert_eq!(pins.at("PS7_PS7", "NOWHERE"), None);
        assert_eq!(pins.at("RAMB36E1_RAMB36E1", "OUTP"), None);

        let fixed = |prob: &Problem| -> Vec<Pin> { prob.nets.iter().map(|n| n.pins[1]).collect() };
        let with = Problem::build(&p, &dev, 200, &pins).unwrap();
        assert_eq!(with.nets.iter().map(|n| n.pins[0]).collect::<Vec<_>>(), vec![Pin::Inst(0); 2]);
        assert_eq!(fixed(&with), vec![Pin::Fixed(51.0, 57.0), Pin::Fixed(52.0, 61.0)]);
        // Without a map every PS pin is at the PS site's tile.
        let without = Problem::build(&p, &dev, 200, &PinMap::default()).unwrap();
        assert_eq!(fixed(&without), vec![Pin::Fixed(32.0, 51.0); 2]);

        // The reported wirelength reads the same positions.
        let at = |c: CellId| Some(if c == lut { (50, 57) } else { (32, 51) });
        let total = |pins: &PinMap| cell_net_hpwl(&p, 200, pins, &at).values().sum::<f64>();
        assert_eq!(total(&pins), 1.0 + 6.0);
        assert_eq!(total(&PinMap::default()), 2.0 * 24.0);
    }
}
