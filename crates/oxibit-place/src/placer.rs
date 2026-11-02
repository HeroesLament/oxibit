//! oxibit's placer, v0.
//!
//! 1. Slice fill: the packed cells are grouped into slice-sized contents
//!    (`Fill`), legal by construction: carry chains as vertical stacks of
//!    slices, LUTRAM and SRL in SLICEM contents, and logic grouped by flop
//!    control set (one clock, CE, SR per slice), each letter holding a 6LUT
//!    and its flop, a second flop on the 5FF only when the first is fed from
//!    its own LUT (so the letter's bypass pin is free for it). Units join a
//!    slice by shared nets, so connected logic shares slices.
//!    At most `FillOptions::bypass_ffs_per_slice` flops per slice take a
//!    bypass pin.
//! 2. Placement: chains into free column runs first, then every single-slice
//!    content and hard block (DSP, block RAM) by simulated annealing on
//!    half-perimeter wirelength, moves within a shrinking window, plus a
//!    cost on each CLB tile's input pins over `Options::pins` and on its
//!    bypass pins over `Options::bypass`, and on each region's used
//!    slices over `Options::density`.
//! 3. Bels written to the netlist (`Cell::bel`), ready for `place::legalise`.
//!
//!    A lone LUT (no flop of its own) shares a letter with another LUT when
//!    the two read at most five distinct nets: first the free 5LUT beside a
//!    carry chain's 6LUT (mostly route-through feeds), then another lone LUT.
//!
//! Not yet: fracturing beside a LUT that feeds a flop, replicating
//! wide-fanout drivers near their loads, timing.

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};

use oxibit_pnr::packed::Loc;

use crate::device::Device;
use crate::netlist::{CellId, NetId};
use crate::policy::Lut5Fill;
use crate::pack::{Packer, slot};

// ---------------------------------------------------------------- sites ---

#[derive(Debug, Clone)]
pub struct SliceSite {
    pub x: i32,
    pub y: i32,
    /// Tile-relative site name (`SLICE_X0Y0`).
    pub rel: String,
    pub m: bool,
}

#[derive(Debug, Clone)]
pub struct HardSite {
    pub x: i32,
    pub y: i32,
    pub rel: String,
    /// The block RAM tile index for RAMB18/RAMB36 sites (a RAMB36 excludes
    /// both RAMB18s of its tile).
    pub tile: usize,
}

#[derive(Debug, Default)]
pub struct Sites {
    pub slices: Vec<SliceSite>,
    /// Slice site -> the site a carry continues into (the same slice
    /// position in the CLB above, across an HCLK / break row).
    pub up: HashMap<usize, usize>,
    pub dsp: Vec<HardSite>,
    pub ramb36: Vec<HardSite>,
    pub ramb18: Vec<HardSite>,
    pub bufg: Vec<HardSite>,
    pub ps7: Vec<HardSite>,
    pub mmcm: Vec<HardSite>,
}

impl Sites {
    pub fn from_device(dev: &Device) -> Sites {
        let mut s = Sites::default();
        let mut tiles: Vec<&crate::device::Tile> = dev.tiles.values().collect();
        tiles.sort_by_key(|t| (t.grid_x, t.grid_y, t.name.clone()));
        let mut by_pos: HashMap<(i32, i32, String), usize> = HashMap::new();
        let mut bram_tile = 0usize;
        let rel = |site: &str| match dev.grid_loc(site, "") {
            Some(Loc::Grid { site, .. }) => site,
            _ => String::new(),
        };
        for t in &tiles {
            if t.ty.starts_with("CLBL") {
                for (site, ty) in &t.sites {
                    if ty == "SLICEL" || ty == "SLICEM" {
                        let r = rel(site);
                        by_pos.insert((t.grid_x, t.grid_y, r.clone()), s.slices.len());
                        s.slices.push(SliceSite { x: t.grid_x, y: t.grid_y, rel: r, m: ty == "SLICEM" });
                    }
                }
            } else if t.ty.starts_with("BRAM_") && !t.ty.contains("INT") {
                for (site, ty) in &t.sites {
                    let h = HardSite { x: t.grid_x, y: t.grid_y, rel: rel(site), tile: bram_tile };
                    match ty.as_str() {
                        "RAMBFIFO36E1" => s.ramb36.push(h),
                        "RAMB18E1" | "FIFO18E1" => s.ramb18.push(h),
                        _ => {}
                    }
                }
                bram_tile += 1;
            } else if t.ty.starts_with("DSP_") {
                for (site, ty) in &t.sites {
                    if ty == "DSP48E1" {
                        s.dsp.push(HardSite { x: t.grid_x, y: t.grid_y, rel: rel(site), tile: 0 });
                    }
                }
            } else if t.ty.starts_with("CLK_BUFG_") && !t.ty.contains("REBUF") {
                for (site, ty) in &t.sites {
                    if ty == "BUFGCTRL" {
                        s.bufg.push(HardSite { x: t.grid_x, y: t.grid_y, rel: rel(site), tile: 0 });
                    }
                }
            } else if t.sites.values().any(|ty| ty == "MMCME2_ADV") {
                for (site, ty) in &t.sites {
                    if ty == "MMCME2_ADV" {
                        s.mmcm.push(HardSite { x: t.grid_x, y: t.grid_y, rel: rel(site), tile: 0 });
                    }
                }
            } else if t.sites.values().any(|ty| ty == "PS7") {
                for (site, ty) in &t.sites {
                    if ty == "PS7" {
                        s.ps7.push(HardSite { x: t.grid_x, y: t.grid_y, rel: rel(site), tile: 0 });
                    }
                }
            }
        }
        // Carry continuation: the same slice position one CLB row up
        // (grid y - 1), or two when the row between is an HCLK / break row.
        let row_ty: HashMap<(i32, i32), &str> = dev.tiles.values().map(|t| ((t.grid_x, t.grid_y), t.ty.as_str())).collect();
        for (i, sl) in s.slices.iter().enumerate() {
            if let Some(&j) = by_pos.get(&(sl.x, sl.y - 1, sl.rel.clone())) {
                s.up.insert(i, j);
            } else if row_ty.get(&(sl.x, sl.y - 1)).is_some_and(|t| t.starts_with("HCLK") || t.starts_with("BRKH"))
                && let Some(&j) = by_pos.get(&(sl.x, sl.y - 2, sl.rel.clone()))
            {
                s.up.insert(i, j);
            }
        }
        s
    }
}

// ----------------------------------------------------------- slice fill ---

/// What one slice holds: (cell, bel name) pairs.
#[derive(Debug, Clone, Default)]
pub struct Fill {
    pub cells: Vec<(CellId, String)>,
    pub need_m: bool,
    /// Plain logic (LUTs and flops by letter): its letters may trade units
    /// with other logic slices during annealing.
    pub logic: bool,
}

/// A unit of placement: one slice, or a carry chain of stacked slices
/// (index 0 at the bottom, continuing up).
#[derive(Debug, Clone)]
pub struct Macro {
    pub slices: Vec<Fill>,
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord)]
struct Ctrl {
    clk: Option<NetId>,
    ce: Option<NetId>,
    sr: Option<NetId>,
    clk_inv: bool,
    sr_inv: bool,
    sync: bool,
}

fn ctrl_of(p: &Packer, ff: CellId) -> Ctrl {
    let c = p.nl.cell(ff);
    let inv = |k: &str| c.int_param(k).unwrap_or(0) != 0;
    Ctrl {
        clk: c.net("CK"),
        ce: c.net("CE"),
        sr: c.net("SR"),
        clk_inv: inv("IS_C_INVERTED"),
        sr_inv: inv("IS_R_INVERTED") || inv("IS_S_INVERTED") || inv("IS_CLR_INVERTED") || inv("IS_PRE_INVERTED"),
        sync: c.attrs.contains_key("X_FFSYNC"),
    }
}

fn letter(l: i32) -> char {
    (b'A' + l as u8) as char
}

fn bel_name(z: i32) -> Option<String> {
    let l = letter(z >> 4);
    Some(match z & 0xF {
        slot::LUT6 => format!("{l}6LUT"),
        slot::LUT5 => format!("{l}5LUT"),
        slot::FF => format!("{l}FF"),
        slot::FF2 => format!("{l}5FF"),
        slot::CARRY4 => "CARRY4".into(),
        slot::F7MUX => if z >> 4 < 2 { "F7AMUX".into() } else { "F7BMUX".into() },
        slot::F8MUX => "F8MUX".into(),
        _ => return None,
    })
}

fn is_memory(p: &Packer, c: CellId) -> bool {
    let a = &p.nl.cell(c).attrs;
    a.contains_key("X_LUT_AS_DRAM") || a.contains_key("X_LUT_AS_SRL")
}

#[derive(Debug, Default)]
pub struct FillStats {
    pub chains: usize,
    pub chain_slices: usize,
    pub memory_slices: usize,
    pub srl_slices: usize,
    pub logic_slices: usize,
    pub lone_lut_letters: usize,
    /// Lone LUTs on the 5LUT of a carry-chain letter.
    pub chain_5luts: usize,
    /// Lone LUTs on the 5LUT beside another lone LUT.
    pub paired_luts: usize,
    /// Lone LUTs on the 5LUT of a letter whose 6LUT feeds its flop.
    pub ff_letter_5luts: usize,
    pub ff2_used: usize,
    pub control_sets: usize,
    pub leftovers: Vec<String>,
}

/// Slice fill limits.
#[derive(Debug, Clone)]
pub struct FillOptions {
    /// Flops fed through a bypass pin (D from outside their letter's LUT)
    /// per slice. Each takes the letter's BYP_ALT, which hard pips also
    /// join to BYP_BOUNCE: a tile with all eight taken has no bounce left
    /// for its neighbours' routes (the LDPC hard-decision registers on the
    /// PHY, four per slice, left eight tiles at 56 of 56 input pins). 4 (no
    /// limit) by default: 2 cost the PHY 342 slices and 2.6% wirelength for
    /// no routing gain.
    pub bypass_ffs_per_slice: usize,
    /// Where a lone LUT may take a 5LUT (gated by the pack policy).
    pub lut5: Lut5Fill,
}

impl Default for FillOptions {
    fn default() -> Self {
        FillOptions {
            bypass_ffs_per_slice: 4,
            lut5: Lut5Fill::ALL,
        }
    }
}

/// One logic letter's worth: a LUT (6LUT) and the flop it drives directly,
/// or a lone flop.
#[derive(Debug, Clone, Copy)]
struct Unit {
    lut: Option<CellId>,
    /// A second LUT on the same letter's 5LUT (inputs shared, at most five).
    lut5: Option<CellId>,
    ff: Option<CellId>,
}

pub fn fill(p: &Packer, opts: &FillOptions) -> (Vec<Macro>, FillStats) {
    let max_lone = opts.bypass_ffs_per_slice.clamp(1, 4);
    let mut st = FillStats::default();
    let mut macros = Vec::new();
    let mut done: HashSet<CellId> = HashSet::new();

    // Carry chains and LUTRAM: clusters with absolute slots.
    for (&root, cl) in &p.clusters {
        let rty = p.nl.cell(root).ty.as_str();
        let Some(rz) = cl.root_abs_z else { continue };
        if rty == "CARRY4" {
            // Slice index by dy (0, -1, ... with the skip rows folded out).
            let mut dys: BTreeSet<i32> = cl.members.iter().map(|(_, o)| o.dy).collect();
            dys.insert(0);
            let order: Vec<i32> = dys.into_iter().rev().collect();
            let mut slices = vec![Fill::default(); order.len()];
            let idx = |dy: i32| order.iter().position(|&d| d == dy).expect("dy listed");
            slices[0].cells.push((root, "CARRY4".into()));
            done.insert(root);
            for (m, o) in &cl.members {
                let z = if o.abs { o.z } else { rz + o.z };
                if let Some(b) = bel_name(z) {
                    slices[idx(o.dy)].cells.push((*m, b));
                    done.insert(*m);
                }
            }
            st.chains += 1;
            st.chain_slices += slices.len();
            macros.push(Macro { slices });
        } else if rty == "SLICE_LUTX" && is_memory(p, root) && p.nl.cell(root).attrs.contains_key("X_LUT_AS_DRAM") {
            let mut f = Fill { need_m: true, ..Fill::default() };
            f.cells.push((root, bel_name(rz).expect("lut slot")));
            done.insert(root);
            for (m, o) in &cl.members {
                let z = if o.abs { o.z } else { rz + o.z };
                if let Some(b) = bel_name(z) {
                    f.cells.push((*m, b));
                    done.insert(*m);
                }
            }
            st.memory_slices += 1;
            macros.push(Macro { slices: vec![f] });
        }
    }

    // Logic units: LUT + its directly fed flop, SRL + flop, lone LUTs, lone
    // flops.
    let mut units: Vec<Unit> = Vec::new();
    let mut srl_units: Vec<Unit> = Vec::new();
    for c in p.nl.cell_ids() {
        if done.contains(&c) {
            continue;
        }
        let ty = p.nl.cell(c).ty.as_str();
        if ty != "SLICE_LUTX" {
            continue;
        }
        // The flop this LUT drives directly, if a cluster paired them; a
        // memory LUT keeps it only on the same clock (a SLICEM's flops run on
        // its write clock).
        let ff = p
            .clusters
            .get(&c)
            .and_then(|cl| cl.members.iter().find(|(m, _)| p.nl.cell(*m).ty == "SLICE_FFX" && !done.contains(m)).map(|(m, _)| *m))
            .filter(|&f| !is_memory(p, c) || p.nl.cell(f).net("CK") == p.nl.cell(c).net("CLK"));
        let u = Unit { lut: Some(c), lut5: None, ff };
        done.insert(c);
        if let Some(f) = ff {
            done.insert(f);
        }
        if is_memory(p, c) {
            srl_units.push(u);
        } else {
            units.push(u);
        }
    }
    for c in p.nl.cell_ids() {
        if !done.contains(&c) && p.nl.cell(c).ty == "SLICE_FFX" {
            units.push(Unit { lut: None, lut5: None, ff: Some(c) });
            done.insert(c);
        }
    }
    for c in p.nl.cell_ids() {
        let ty = p.nl.cell(c).ty.as_str();
        if !done.contains(&c) && matches!(ty, "SLICE_LUTX" | "SLICE_FFX" | "CARRY4" | "SELMUX2_1") {
            st.leftovers.push(p.nl.cell(c).name.clone());
        }
    }

    // SRLs: SLICEM, one write clock and write enable per slice, flops on the
    // same clock.
    let mut srl_groups: BTreeMap<(Option<NetId>, Option<NetId>, Option<Ctrl>), Vec<Unit>> = BTreeMap::new();
    for u in srl_units {
        let l = u.lut.expect("srl");
        let key = (p.nl.cell(l).net("CLK"), p.nl.cell(l).net("WE"), u.ff.map(|f| ctrl_of(p, f)));
        srl_groups.entry(key).or_default().push(u);
    }
    for (_, us) in srl_groups {
        for chunk in us.chunks(4) {
            let mut f = Fill { need_m: true, ..Fill::default() };
            for (i, u) in chunk.iter().enumerate() {
                let l = letter(i as i32);
                f.cells.push((u.lut.expect("srl"), format!("{l}6LUT")));
                if let Some(ff) = u.ff {
                    f.cells.push((ff, format!("{l}FF")));
                }
            }
            st.srl_slices += 1;
            macros.push(Macro { slices: vec![f] });
        }
    }

    // Fractured letters (see the module doc).
    let units = fracture(p, &mut macros, units, opts.lut5, &mut st);

    // Logic: group flop-carrying units by control set; LUT-only units fill
    // whatever letters remain.
    let mut by_ctrl: BTreeMap<Ctrl, Vec<usize>> = BTreeMap::new();
    let mut lone_luts: Vec<usize> = Vec::new();
    for (i, u) in units.iter().enumerate() {
        match u.ff {
            Some(f) => by_ctrl.entry(ctrl_of(p, f)).or_default().push(i),
            None => lone_luts.push(i),
        }
    }
    st.control_sets = by_ctrl.len();

    // Net adjacency between units for locality.
    let mut unit_of: HashMap<CellId, usize> = HashMap::new();
    for (i, u) in units.iter().enumerate() {
        for c in u.lut.iter().chain(u.lut5.iter()).chain(u.ff.iter()) {
            unit_of.insert(*c, i);
        }
    }
    let nets_of = |u: &Unit| -> Vec<NetId> {
        let mut v = Vec::new();
        for c in u.lut.iter().chain(u.lut5.iter()).chain(u.ff.iter()) {
            for port in p.nl.cell(*c).ports.values() {
                if let Some(n) = port.net
                    && !p.nl.is_const(n)
                    && p.nl.net(n).users.len() < 64
                {
                    v.push(n);
                }
            }
        }
        v
    };
    let neighbours = |i: usize| -> Vec<usize> {
        let mut v = Vec::new();
        for n in nets_of(&units[i]) {
            let net = p.nl.net(n);
            for (c, _) in net.driver.iter().chain(net.users.iter()) {
                if let Some(&j) = unit_of.get(c)
                    && j != i
                {
                    v.push(j);
                }
            }
        }
        v
    };

    let mut used: HashSet<usize> = HashSet::new();
    let mut logic: Vec<Vec<usize>> = Vec::new(); // letters per slice: unit indices
    for (_, list) in &by_ctrl {
        let in_set: HashSet<usize> = list.iter().copied().collect();
        for &seed in list {
            if used.contains(&seed) {
                continue;
            }
            // A slice: up to four LUT-carrying units, plus up to four lone
            // flops on the 5FF slots of letters whose flop is LUT-fed.
            let mut slice = vec![seed];
            used.insert(seed);
            let mut gain: HashMap<usize, usize> = HashMap::new();
            let grow = |members: &Vec<usize>, gain: &mut HashMap<usize, usize>, used: &HashSet<usize>| {
                for j in neighbours(*members.last().expect("non-empty")) {
                    if in_set.contains(&j) && !used.contains(&j) {
                        *gain.entry(j).or_default() += 1;
                    }
                }
            };
            grow(&slice, &mut gain, &used);
            while slice.len() < 8 {
                let luts = slice.iter().filter(|&&i| units[i].lut.is_some()).count();
                let lones = slice.len() - luts;
                let fits = |j: usize| if units[j].lut.is_some() { luts < 4 } else { lones < max_lone };
                let best = gain.iter().filter(|(j, _)| !used.contains(*j) && fits(**j)).max_by_key(|(j, g)| (**g, std::cmp::Reverse(**j))).map(|(j, _)| *j);
                let next = match best {
                    Some(j) => j,
                    None => match list.iter().copied().find(|&j| !used.contains(&j) && fits(j)) {
                        Some(j) => j,
                        None => break,
                    },
                };
                used.insert(next);
                slice.push(next);
                grow(&slice, &mut gain, &used);
            }
            logic.push(slice);
        }
    }

    // Lone LUTs: into a free letter of the slice holding most of their
    // neighbours, else grouped with each other by shared nets.
    let mut slice_of: HashMap<usize, usize> = HashMap::new();
    let mut free: Vec<usize> = Vec::new();
    for (si, slice) in logic.iter().enumerate() {
        for &i in slice {
            slice_of.insert(i, si);
        }
        free.push(4 - slice.iter().filter(|&&i| units[i].lut.is_some()).count());
    }
    let mut extra: Vec<Vec<usize>> = vec![Vec::new(); logic.len()];
    let mut lone_sorted = lone_luts.clone();
    lone_sorted.sort_by_key(|&i| std::cmp::Reverse(neighbours(i).len()));
    let mut unplaced_lone: Vec<usize> = Vec::new();
    for &i in &lone_sorted {
        let mut score: HashMap<usize, usize> = HashMap::new();
        for j in neighbours(i) {
            if let Some(&si) = slice_of.get(&j)
                && free[si] > 0
            {
                *score.entry(si).or_default() += 1;
            }
        }
        match score.into_iter().max_by_key(|&(si, sc)| (sc, std::cmp::Reverse(si))) {
            Some((si, _)) => {
                free[si] -= 1;
                extra[si].push(i);
                slice_of.insert(i, si);
            }
            None => unplaced_lone.push(i),
        }
    }
    // Lone LUTs with no free letter near them: the 5LUT of a LUT + flop
    // letter in the neighbours' slices, else in any slice, when the two
    // LUTs read at most five nets. `ff5`: host unit -> the LUT on its 5LUT.
    let mut ff5: HashMap<usize, CellId> = HashMap::new();
    let mut hosts_of: Vec<Vec<usize>> = vec![Vec::new(); logic.len()];
    for (si, slice) in logic.iter().enumerate() {
        for &h in slice {
            let u = &units[h];
            if let (Some(l), None, Some(_)) = (u.lut, u.lut5, u.ff)
                && opts.lut5.ff_letter
                && lut_nets(p, l).len() <= 5
            {
                hosts_of[si].push(h);
            }
        }
    }
    let mut still: Vec<usize> = Vec::new();
    let mut any_slice = 0usize;
    for &i in &unplaced_lone {
        let Some(l) = units[i].lut.filter(|_| units[i].lut5.is_none()) else {
            still.push(i);
            continue;
        };
        let mine = lut_nets(p, l);
        if mine.len() > 5 {
            still.push(i);
            continue;
        }
        let mut score: HashMap<usize, usize> = HashMap::new();
        for j in neighbours(i) {
            if let Some(&si) = slice_of.get(&j) {
                *score.entry(si).or_default() += 1;
            }
        }
        let mut ranked: Vec<(usize, usize)> = score.into_iter().collect();
        ranked.sort_by_key(|&(si, sc)| (std::cmp::Reverse(sc), si));
        let fits = |h: usize, ff5: &HashMap<usize, CellId>| {
            !ff5.contains_key(&h) && lut_nets(p, units[h].lut.expect("lut")).union(&mine).count() <= 5
        };
        let mut pick = ranked.iter().find_map(|&(si, _)| hosts_of[si].iter().copied().find(|&h| fits(h, &ff5)));
        // no neighbour slice has room: scan on from where the last scan stopped
        if pick.is_none() {
            for k in 0..logic.len() {
                let si = (any_slice + k) % logic.len();
                if let Some(h) = hosts_of[si].iter().copied().find(|&h| fits(h, &ff5)) {
                    pick = Some(h);
                    any_slice = si;
                    break;
                }
            }
        }
        match pick {
            Some(h) => {
                ff5.insert(h, l);
                st.ff_letter_5luts += 1;
            }
            None => still.push(i),
        }
    }
    let unplaced_lone = still;
    let rest_set: HashSet<usize> = unplaced_lone.iter().copied().collect();
    let mut taken: HashSet<usize> = HashSet::new();
    let mut lone_groups: Vec<Vec<usize>> = Vec::new();
    for &seed in &unplaced_lone {
        if taken.contains(&seed) {
            continue;
        }
        taken.insert(seed);
        let mut g = vec![seed];
        let mut gain: HashMap<usize, usize> = HashMap::new();
        while g.len() < 4 {
            for j in neighbours(*g.last().expect("seed")) {
                if rest_set.contains(&j) && !taken.contains(&j) {
                    *gain.entry(j).or_default() += 1;
                }
            }
            let best = gain.iter().filter(|(j, _)| !taken.contains(*j)).max_by_key(|(j, g)| (**g, std::cmp::Reverse(**j))).map(|(j, _)| *j);
            let next = match best {
                Some(j) => j,
                None => match unplaced_lone.iter().copied().find(|j| !taken.contains(j)) {
                    Some(j) => j,
                    None => break,
                },
            };
            taken.insert(next);
            g.push(next);
        }
        lone_groups.push(g);
    }

    let mut fills: Vec<Fill> = Vec::new();
    for (si, slice) in logic.iter().enumerate() {
        let mut f = Fill { logic: true, ..Fill::default() };
        let lut_units: Vec<usize> = slice.iter().copied().filter(|&i| units[i].lut.is_some()).collect();
        let lone_ffs: Vec<usize> = slice.iter().copied().filter(|&i| units[i].lut.is_none()).collect();
        let mut letter_i = 0;
        for &i in &lut_units {
            let l = letter(letter_i);
            f.cells.push((units[i].lut.expect("lut"), format!("{l}6LUT")));
            if let Some(l5) = units[i].lut5.or(ff5.get(&i).copied()) {
                f.cells.push((l5, format!("{l}5LUT")));
            }
            f.cells.push((units[i].ff.expect("ff"), format!("{l}FF")));
            letter_i += 1;
        }
        // Lone flops: on the 5FF of a LUT-fed letter (bypass pin free), else
        // on the main FF of an empty letter (bypass pin carries its D).
        let mut ff2_letter = 0;
        let mut empty_letter = letter_i;
        for &i in &lone_ffs {
            let ff = units[i].ff.expect("ff");
            if ff2_letter < letter_i {
                f.cells.push((ff, format!("{}5FF", letter(ff2_letter))));
                ff2_letter += 1;
                st.ff2_used += 1;
            } else if empty_letter < 4 {
                f.cells.push((ff, format!("{}FF", letter(empty_letter))));
                empty_letter += 1;
            }
        }
        // Lone LUTs on the letters with no LUT.
        let mut lone = extra[si].iter();
        for li in letter_i..4 {
            if let Some(&i) = lone.next() {
                f.cells.push((units[i].lut.expect("lut"), format!("{}6LUT", letter(li))));
                if let Some(l5) = units[i].lut5 {
                    f.cells.push((l5, format!("{}5LUT", letter(li))));
                }
                st.lone_lut_letters += 1;
            }
        }
        fills.push(f);
    }
    for g in &lone_groups {
        let mut f = Fill { logic: true, ..Fill::default() };
        for (k, &i) in g.iter().enumerate() {
            f.cells.push((units[i].lut.expect("lut"), format!("{}6LUT", letter(k as i32))));
            if let Some(l5) = units[i].lut5 {
                f.cells.push((l5, format!("{}5LUT", letter(k as i32))));
            }
            st.lone_lut_letters += 1;
        }
        fills.push(f);
    }
    st.logic_slices = fills.len();
    macros.extend(fills.into_iter().map(|f| Macro { slices: vec![f] }));
    (macros, st)
}

/// Whether letter `l` (index `k`) of chain slice `f` sends a carry output
/// (`O`/`CO` of position `k`) anywhere but its own flops or the next
/// CARRY4: that takes the letter's MUX output pin.
fn mux_busy(p: &Packer, f: &Fill, k: usize, l: char) -> bool {
    let Some(carry) = f.cells.iter().find(|(_, b)| b == "CARRY4").map(|(c, _)| *c) else {
        return false;
    };
    let own: HashSet<CellId> = f
        .cells
        .iter()
        .filter(|(_, b)| *b == format!("{l}FF") || *b == format!("{l}5FF"))
        .map(|(c, _)| *c)
        .collect();
    let names = [format!("O{k}"), format!("O[{k}]"), format!("CO{k}"), format!("CO[{k}]")];
    names.iter().any(|port| {
        let Some(n) = p.nl.cell(carry).net(port) else { return false };
        p.nl.net(n).users.iter().any(|(u, _)| !own.contains(u) && p.nl.cell(*u).ty != "CARRY4")
    })
}

/// Distinct nets on a LUT's inputs (as `legalise` counts them).
fn lut_nets(p: &Packer, c: CellId) -> BTreeSet<NetId> {
    crate::place::lut_inputs(p, c).into_iter().map(|(_, n)| n).collect()
}

/// Nets on any port of `c`, skipping constants and very wide nets.
fn local_nets(p: &Packer, c: CellId) -> Vec<NetId> {
    p.nl.cell(c)
        .ports
        .values()
        .filter_map(|port| port.net)
        .filter(|&n| !p.nl.is_const(n) && p.nl.net(n).users.len() < 64)
        .collect()
}

/// Lone LUTs into 5LUT positions: beside carry-chain 6LUTs, then beside
/// each other. Returns the units left (a LUT placed in a chain slice
/// leaves; a paired LUT rides on its partner's unit).
fn fracture(p: &Packer, macros: &mut [Macro], units: Vec<Unit>, lut5: Lut5Fill, st: &mut FillStats) -> Vec<Unit> {
    let lone = |u: &Unit| -> Option<CellId> {
        match (u.lut, u.lut5, u.ff) {
            (Some(c), None, None) if !is_memory(p, c) => Some(c),
            _ => None,
        }
    };
    let mut nets: HashMap<CellId, BTreeSet<NetId>> = HashMap::new();
    for u in &units {
        if let Some(c) = lone(u) {
            let n = lut_nets(p, c);
            if n.len() <= 5 {
                nets.insert(c, n);
            }
        }
    }

    // Chain letters with a 6LUT of at most five inputs and a free 5LUT.
    struct Host {
        mi: usize,
        si: usize,
        letter: char,
        nets: BTreeSet<NetId>,
        free: bool,
    }
    let mut hosts: Vec<Host> = Vec::new();
    let mut slice_of: HashMap<CellId, (usize, usize)> = HashMap::new();
    for (mi, m) in macros.iter().enumerate() {
        let chain = m.slices.iter().any(|f| f.cells.iter().any(|(_, b)| b == "CARRY4"));
        if !chain {
            continue;
        }
        for (si, f) in m.slices.iter().enumerate() {
            for (c, _) in &f.cells {
                slice_of.insert(*c, (mi, si));
            }
            for (k, l) in ['A', 'B', 'C', 'D'].into_iter().enumerate() {
                let six = f.cells.iter().find(|(_, b)| *b == format!("{l}6LUT")).map(|(c, _)| *c);
                let five = f.cells.iter().any(|(_, b)| *b == format!("{l}5LUT"));
                let Some(six) = six else { continue };
                // The 5LUT's output leaves on the letter's MUX pin, which a
                // chain output used outside the slice already takes.
                if !lut5.chain || five || is_memory(p, six) || mux_busy(p, f, k, l) {
                    continue;
                }
                let n = lut_nets(p, six);
                if n.len() <= 5 {
                    hosts.push(Host { mi, si, letter: l, nets: n, free: true });
                }
            }
        }
    }
    if std::env::var_os("OXIBIT_FILL_DEBUG").is_some() {
        // why chain letters are not hosts: (no 6LUT, 5LUT used, memory, mux busy, > 5 inputs)
        let mut why = [0usize; 5];
        for m in macros.iter() {
            if !m.slices.iter().any(|f| f.cells.iter().any(|(_, b)| b == "CARRY4")) {
                continue;
            }
            for f in &m.slices {
                for (k, l) in ['A', 'B', 'C', 'D'].into_iter().enumerate() {
                    let six = f.cells.iter().find(|(_, b)| *b == format!("{l}6LUT")).map(|(c, _)| *c);
                    let five = f.cells.iter().any(|(_, b)| *b == format!("{l}5LUT"));
                    match six {
                        None => why[0] += 1,
                        Some(_) if five => why[1] += 1,
                        Some(c) if is_memory(p, c) => why[2] += 1,
                        Some(_) if mux_busy(p, f, k, l) => why[3] += 1,
                        Some(c) if lut_nets(p, c).len() > 5 => why[4] += 1,
                        _ => {}
                    }
                }
            }
        }
        eprintln!("fill debug: chain hosts {} lone candidates {} not-host {:?}", hosts.len(), nets.len(), why);
    }
    let mut hosts_in: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
    for (h, host) in hosts.iter().enumerate() {
        hosts_in.entry((host.mi, host.si)).or_default().push(h);
    }

    let mut gone: HashSet<usize> = HashSet::new();
    for (i, u) in units.iter().enumerate() {
        let Some(c) = lone(u) else { continue };
        let Some(mine) = nets.get(&c) else { continue };
        let mut score: HashMap<(usize, usize), usize> = HashMap::new();
        for n in local_nets(p, c) {
            let net = p.nl.net(n);
            for (d, _) in net.driver.iter().chain(net.users.iter()) {
                if let Some(&s) = slice_of.get(d) {
                    *score.entry(s).or_default() += 1;
                }
            }
        }
        let mut ranked: Vec<((usize, usize), usize)> = score.into_iter().collect();
        ranked.sort_by_key(|&(s, sc)| (std::cmp::Reverse(sc), s));
        let pick = ranked.iter().find_map(|(s, _)| {
            hosts_in.get(s)?.iter().copied().find(|&h| hosts[h].free && hosts[h].nets.union(mine).count() <= 5)
        });
        if let Some(h) = pick {
            let host = &mut hosts[h];
            host.free = false;
            macros[host.mi].slices[host.si].cells.push((c, format!("{}5LUT", host.letter)));
            st.chain_5luts += 1;
            gone.insert(i);
        }
    }

    // Lone LUTs left: pair by shared inputs, then any two that fit.
    let mut by_net: HashMap<NetId, Vec<usize>> = HashMap::new();
    for (i, u) in units.iter().enumerate() {
        if gone.contains(&i) {
            continue;
        }
        if let Some(c) = lone(u)
            && let Some(n) = nets.get(&c)
        {
            for net in n {
                if !p.nl.is_const(*net) {
                    by_net.entry(*net).or_default().push(i);
                }
            }
        }
    }
    let mut partner: HashMap<usize, usize> = HashMap::new(); // 6LUT unit -> 5LUT unit
    let mut taken: HashSet<usize> = HashSet::new();
    let mut order: Vec<usize> = (0..units.len())
        .filter(|i| lut5.pairs && !gone.contains(i) && lone(&units[*i]).is_some_and(|c| nets.contains_key(&c)))
        .collect();
    order.sort_by_key(|&i| std::cmp::Reverse(nets[&units[i].lut.expect("lut")].len()));
    for &i in &order {
        if taken.contains(&i) {
            continue;
        }
        let ni = &nets[&units[i].lut.expect("lut")];
        let mut shared: HashMap<usize, usize> = HashMap::new();
        for net in ni {
            for &j in by_net.get(net).map(|v| v.as_slice()).unwrap_or(&[]) {
                if j != i && !taken.contains(&j) {
                    *shared.entry(j).or_default() += 1;
                }
            }
        }
        let best = shared
            .into_iter()
            .filter(|&(j, _)| nets[&units[j].lut.expect("lut")].union(ni).count() <= 5)
            .max_by_key(|&(j, sc)| (sc, std::cmp::Reverse(j)));
        if let Some((j, _)) = best {
            taken.insert(i);
            taken.insert(j);
            partner.insert(i, j);
        }
    }
    // Unshared leftovers: smallest with largest, while the sum fits.
    let mut rest: Vec<usize> = order.iter().copied().filter(|i| !taken.contains(i)).collect();
    rest.sort_by_key(|&i| nets[&units[i].lut.expect("lut")].len());
    let (mut lo, mut hi) = (0usize, rest.len());
    while lo + 1 < hi {
        let (a, b) = (rest[lo], rest[hi - 1]);
        let fit = nets[&units[a].lut.expect("lut")].union(&nets[&units[b].lut.expect("lut")]).count() <= 5;
        if fit {
            partner.insert(b, a);
            lo += 1;
            hi -= 1;
        } else {
            hi -= 1;
        }
    }
    let mut riders: HashSet<usize> = partner.values().copied().collect();
    riders.extend(gone.iter().copied());
    let mut out = Vec::with_capacity(units.len());
    for (i, u) in units.iter().enumerate() {
        if riders.contains(&i) {
            continue;
        }
        let mut u = *u;
        if let Some(&j) = partner.get(&i) {
            u.lut5 = units[j].lut;
            st.paired_luts += 1;
        }
        out.push(u);
    }
    out
}

// ------------------------------------------------------------ placement ---

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum Kind {
    Slice,
    Dsp,
    Ramb36,
    Ramb18,
}

/// A movable object: a single-slice fill, or one hard cell.
struct Obj {
    kind: Kind,
    need_m: bool,
    cells: Vec<CellId>,
    /// Sites held: one, or a chain's run bottom to top.
    sites: Vec<usize>,
}

#[derive(Debug, Default)]
pub struct PlaceStats {
    pub objects: usize,
    pub chains_placed: usize,
    pub chain_failures: usize,
    pub hpwl_start: f64,
    pub hpwl_end: f64,
    pub moves: usize,
    pub accepted: usize,
    pub seconds: f64,
    /// (temperature, window, acceptance rate, cost) per outer step.
    pub trace: Vec<(f64, f64, f64, f64)>,
    /// Input pins over `Options::pins`, summed over CLB tiles, before and
    /// after; tiles over the cap after; the fullest tile after.
    pub pin_excess_start: u64,
    pub pin_excess_end: u64,
    pub tiles_over_cap: usize,
    pub tile_pins_max: u32,
    /// The same for bypass pins (`Options::bypass`).
    pub bypass_excess_start: u64,
    pub bypass_excess_end: u64,
    pub bypass_tiles_over_cap: usize,
    pub bypass_tile_max: u32,
    /// Fills that ask two nets of one bypass pin (a packing bug if not 0).
    pub bypass_clashes: usize,
    /// Used slices over the density target, summed over bins, before and
    /// after; bins over the target after.
    pub density_excess_start: f64,
    pub density_excess_end: f64,
    pub density_bins_over: usize,
}

/// A small, fast, seeded PRNG (xorshift64*).
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    fn below(&mut self, n: usize) -> usize {
        (self.next() % n.max(1) as u64) as usize
    }
    fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
}

pub struct Options {
    pub seed: u64,
    /// Annealing effort: moves per object per temperature.
    pub inner: usize,
    /// Nets with more pins than this are left out of the cost.
    pub max_fanout: usize,
    /// Stop after this many moves per object.
    pub max_moves_per_obj: usize,
    /// Start temperature, in average net costs.
    pub t0_scale: f64,
    /// Share of moves that move a LUT (or LUT and flop) between logic slices.
    pub unit_frac: f64,
    /// Unit moves start once the move window has shrunk to this many tiles.
    pub unit_window: f64,
    /// Temperature steps of greedy unit moves after the anneal.
    pub polish: usize,
    /// Input pins (LUT inputs and bypass pins, of 56) per CLB tile. Off by
    /// default: on the PHY (cap 48, weight 4) it cleared every tile over 48
    /// but moved the pins into the neighbours, the 3x5-tile peak rose (695
    /// to 704 of 840) and the route did worse.
    pub pins: PinCost,
    /// Bypass pins (AX..DX of both slices, 8) per CLB tile: the pin class
    /// the router's plateau is made of (BYP, BYP_ALT, BYP_BOUNCE).
    pub bypass: PinCost,
    /// Used slices per region over a target share of its slice sites.
    pub density: DensityCost,
}

/// Slice density per region: the die cut into `bin` x `bin` tile bins,
/// each used slice above `target` of a bin's slice sites costs `weight`
/// tiles of wirelength; `weight` 0 is off. Wirelength alone pulls the
/// design into one block at the centre of the die; this spreads it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DensityCost {
    pub bin: i32,
    pub target: f64,
    pub weight: f64,
}

impl DensityCost {
    pub const OFF: DensityCost = DensityCost { bin: 8, target: 1.0, weight: 0.0 };

    pub fn on(&self) -> bool {
        self.weight != 0.0
    }
}

/// One class of a tile's input pins: each pin over `cap` costs `weight`
/// tiles of wirelength; `weight` 0 is off.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PinCost {
    pub cap: u32,
    pub weight: f64,
}

impl PinCost {
    pub const OFF_PINS: PinCost = PinCost { cap: 48, weight: 0.0 };
    pub const OFF_BYPASS: PinCost = PinCost { cap: 8, weight: 0.0 };

    pub fn excess(&self, load: u32) -> u32 {
        load.saturating_sub(self.cap)
    }

    pub fn on(&self) -> bool {
        self.weight != 0.0
    }
}

/// A slice's or tile's input pins: all of them, and the bypass pins among them.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct Load {
    all: u32,
    x: u32,
}

#[derive(Clone, Copy, Debug)]
struct Delta {
    all: i64,
    x: i64,
}

impl std::ops::Add for Load {
    type Output = Load;
    fn add(self, o: Load) -> Load {
        Load { all: self.all + o.all, x: self.x + o.x }
    }
}

impl Load {
    fn delta(new: Load, old: Load) -> Delta {
        Delta { all: i64::from(new.all) - i64::from(old.all), x: i64::from(new.x) - i64::from(old.x) }
    }

    fn apply(self, d: Delta) -> Load {
        Load { all: (i64::from(self.all) + d.all).max(0) as u32, x: (i64::from(self.x) + d.x).max(0) as u32 }
    }
}

impl From<Load> for Delta {
    fn from(l: Load) -> Delta {
        Delta { all: l.all.into(), x: l.x.into() }
    }
}

impl std::ops::Neg for Delta {
    type Output = Delta;
    fn neg(self) -> Delta {
        Delta { all: -self.all, x: -self.x }
    }
}

impl Default for Options {
    fn default() -> Self {
        Options { seed: 1, inner: 20, max_fanout: 200, max_moves_per_obj: 3000, t0_scale: 24.0, unit_frac: 0.5, unit_window: 4.0, polish: 0, pins: PinCost::OFF_PINS, bypass: PinCost::OFF_BYPASS, density: DensityCost::OFF }
    }
}

pub fn place(p: &mut Packer, dev: &Device, sites: &Sites, macros: Vec<Macro>, opts: &Options) -> Result<PlaceStats, String> {
    let t0 = std::time::Instant::now();
    let mut st = PlaceStats::default();
    let mut rng = Rng(opts.seed.max(1) ^ 0x9E37_79B9_7F4A_7C15);
    let _ = dev;

    // Objects: slice macros (one slice, or a chain of slices going up) and
    // hard cells. `fills[o]` are the slice contents of a slice object.
    let mut objs: Vec<Obj> = Vec::new();
    let mut fills: Vec<Vec<Fill>> = Vec::new();
    for m in macros {
        let need_m = m.slices.iter().any(|f| f.need_m);
        let cells = m.slices.iter().flat_map(|f| f.cells.iter().map(|(c, _)| *c)).collect();
        objs.push(Obj { kind: Kind::Slice, need_m, cells, sites: Vec::new() });
        fills.push(m.slices);
    }
    for c in p.nl.cell_ids() {
        if p.nl.cell(c).bel.is_some() {
            continue;
        }
        let k = match p.nl.cell(c).ty.as_str() {
            "DSP48E1_DSP48E1" => Kind::Dsp,
            "RAMB36E1_RAMB36E1" => Kind::Ramb36,
            "RAMB18E1_RAMB18E1" => Kind::Ramb18,
            _ => continue,
        };
        objs.push(Obj { kind: k, need_m: false, cells: vec![c], sites: Vec::new() });
        fills.push(Vec::new());
    }
    st.objects = objs.len();
    let n = objs.len().max(1);

    let pool_len = |k: Kind| match k {
        Kind::Slice => sites.slices.len(),
        Kind::Dsp => sites.dsp.len(),
        Kind::Ramb36 => sites.ramb36.len(),
        Kind::Ramb18 => sites.ramb18.len(),
    };
    let site_xy = |k: Kind, s: usize| -> (i32, i32) {
        match k {
            Kind::Slice => (sites.slices[s].x, sites.slices[s].y),
            Kind::Dsp => (sites.dsp[s].x, sites.dsp[s].y),
            Kind::Ramb36 => (sites.ramb36[s].x, sites.ramb36[s].y),
            Kind::Ramb18 => (sites.ramb18[s].x, sites.ramb18[s].y),
        }
    };
    let kidx = |k: Kind| match k {
        Kind::Slice => 0usize,
        Kind::Dsp => 1,
        Kind::Ramb36 => 2,
        Kind::Ramb18 => 3,
    };
    let kinds = [Kind::Slice, Kind::Dsp, Kind::Ramb36, Kind::Ramb18];
    let mut at: HashMap<(usize, i32, i32), Vec<usize>> = HashMap::new();
    for k in kinds {
        for s in 0..pool_len(k) {
            let (x, y) = site_xy(k, s);
            at.entry((kidx(k), x, y)).or_default().push(s);
        }
    }
    let mut owner: Vec<Vec<Option<usize>>> = kinds.iter().map(|&k| vec![None; pool_len(k)]).collect();
    let mut b36: HashMap<usize, bool> = HashMap::new();
    let mut b18: HashMap<usize, usize> = HashMap::new();

    // The run of slice sites a chain of `len` would occupy from `base`.
    let run_from = |base: usize, len: usize| -> Option<Vec<usize>> {
        let mut v = vec![base];
        let mut cur = base;
        for _ in 1..len {
            cur = *sites.up.get(&cur)?;
            v.push(cur);
        }
        Some(v)
    };
    let fits = |o: &Obj, f: &Vec<Fill>, run: &[usize], owner: &Vec<Vec<Option<usize>>>, me: usize| -> bool {
        run.iter().zip(f).all(|(&s, fl)| owner[0][s].is_none_or(|w| w == me) && (!fl.need_m || sites.slices[s].m)) && o.kind == Kind::Slice
    };

    // Initial placement: chains longest first, then the rest, each on the
    // first free legal spot in a shuffled scan.
    let mut order: Vec<usize> = (0..n).collect();
    order.sort_by_key(|&o| (objs[o].kind != Kind::Slice, std::cmp::Reverse(fills[o].len())));
    let mut scans: Vec<Vec<usize>> = kinds
        .iter()
        .map(|&k| {
            let mut v: Vec<usize> = (0..pool_len(k)).collect();
            for i in (1..v.len()).rev() {
                let j = rng.below(i + 1);
                v.swap(i, j);
            }
            v
        })
        .collect();
    let mut cursor = [0usize; 4];
    for &o in &order {
        let k = objs[o].kind;
        let ki = kidx(k);
        let list = std::mem::take(&mut scans[ki]);
        let mut placed: Option<Vec<usize>> = None;
        for step in 0..list.len() {
            let s = list[(cursor[ki] + step) % list.len()];
            let cand = match k {
                Kind::Slice => run_from(s, fills[o].len()).filter(|r| fits(&objs[o], &fills[o], r, &owner, usize::MAX)),
                Kind::Dsp => owner[ki][s].is_none().then(|| vec![s]),
                Kind::Ramb36 => (owner[ki][s].is_none() && b18.get(&sites.ramb36[s].tile).copied().unwrap_or(0) == 0).then(|| vec![s]),
                Kind::Ramb18 => (owner[ki][s].is_none() && !b36.get(&sites.ramb18[s].tile).copied().unwrap_or(false)).then(|| vec![s]),
            };
            if let Some(r) = cand {
                cursor[ki] = (cursor[ki] + step + 1) % list.len();
                placed = Some(r);
                break;
            }
        }
        scans[ki] = list;
        let Some(run) = placed else {
            return Err(format!("no free site for a {k:?} object of {} slice(s)", fills[o].len().max(1)));
        };
        for &s in &run {
            owner[ki][s] = Some(o);
        }
        match k {
            Kind::Ramb36 => {
                b36.insert(sites.ramb36[run[0]].tile, true);
            }
            Kind::Ramb18 => *b18.entry(sites.ramb18[run[0]].tile).or_default() += 1,
            _ => {}
        }
        if fills[o].len() > 1 {
            st.chains_placed += 1;
        }
        objs[o].sites = run;
    }

    // Placeable cells, densely indexed; where each sits: (object, slice).
    let mut cidx: HashMap<CellId, u32> = HashMap::new();
    let mut cells: Vec<CellId> = Vec::new();
    let mut cell_at: Vec<(u32, u32)> = Vec::new();
    let mut obj_cells: Vec<Vec<u32>> = vec![Vec::new(); n];
    for (o, f) in fills.iter().enumerate() {
        let list: Vec<(usize, CellId)> = if f.is_empty() {
            vec![(0, objs[o].cells[0])]
        } else {
            f.iter().enumerate().flat_map(|(k, fl)| fl.cells.iter().map(move |(c, _)| (k, *c))).collect()
        };
        for (k, c) in list {
            let ci = cells.len() as u32;
            cidx.insert(c, ci);
            cells.push(c);
            cell_at.push((o as u32, k as u32));
            obj_cells[o].push(ci);
        }
    }

    // Logic slices as letters: [6LUT, FF, 5FF, 5LUT] per letter. A letter
    // holding a 5LUT stays where the fill put it (its two LUTs share the
    // A pins); unit moves leave it alone.
    const NONE: u32 = u32::MAX;
    let mut letters: Vec<[[u32; 4]; 4]> = vec![[[NONE; 4]; 4]; n];
    let mut is_logic: Vec<bool> = vec![false; n];
    for (o, f) in fills.iter().enumerate() {
        if f.len() != 1 || !f[0].logic {
            continue;
        }
        is_logic[o] = true;
        for (c, b) in &f[0].cells {
            let l = (b.as_bytes()[0] - b'A') as usize;
            let slot = match &b[1..] {
                "6LUT" => 0,
                "FF" => 1,
                "5FF" => 2,
                "5LUT" => 3,
                _ => continue,
            };
            letters[o][l][slot] = cidx[c];
        }
    }
    // Control set id per flop; per logic slice the id and flop count.
    let mut ctrl_ids: HashMap<Ctrl, u32> = HashMap::new();
    let mut ctrl_of_cell: HashMap<u32, u32> = HashMap::new();
    for (ci, &c) in cells.iter().enumerate() {
        if p.nl.cell(c).ty == "SLICE_FFX" {
            let k = ctrl_of(p, c);
            let next = ctrl_ids.len() as u32;
            let id = *ctrl_ids.entry(k).or_insert(next);
            ctrl_of_cell.insert(ci as u32, id);
        }
    }
    let mut slice_ffs: Vec<(u32, u32)> = vec![(NONE, 0); n]; // (ctrl, count)
    for o in 0..n {
        if !is_logic[o] {
            continue;
        }
        for l in 0..4 {
            for slot in [1, 2] {
                let ci = letters[o][l][slot];
                if ci != NONE {
                    slice_ffs[o].0 = ctrl_of_cell[&ci];
                    slice_ffs[o].1 += 1;
                }
            }
        }
    }
    // A letter's flop fed by its own LUT (the pair moves together).
    let fed_by = |lut: u32, ff: u32| -> bool {
        if lut == NONE || ff == NONE {
            return false;
        }
        let d = p.nl.cell(cells[ff as usize]).net("D");
        d.is_some() && p.nl.cell(cells[lut as usize]).net("O6") == d
    };

    // Input pins per CLB tile: LUT inputs on live nets, plus a bypass pin
    // for each flop not fed from inside its slice, each live carry CYINIT
    // or outside-fed carry DI, and each DRAM DI or WA7/WA8. Logic slices
    // count from their letters as moves change them; the rest once, from
    // their fills.
    let live = |c: CellId, port: &str| p.nl.cell(c).net(port).is_some_and(|n| !p.nl.is_const(n));
    let lut_pins: Vec<u32> = cells
        .iter()
        .map(|&c| if p.nl.cell(c).ty == "SLICE_LUTX" { (1..=6).filter(|i| live(c, &format!("A{i}"))).count() as u32 } else { 0 })
        .collect();
    let logic_pins = |lt: &[[u32; 4]; 4]| -> Load {
        lt.iter()
            .map(|&[lut, ff, ff2, l5]| {
                // two LUTs on one letter share A1..A5
                let luts = match (lut != NONE, l5 != NONE) {
                    (true, true) => (lut_pins[lut as usize] + lut_pins[l5 as usize]).min(5),
                    (true, false) => lut_pins[lut as usize],
                    (false, true) => lut_pins[l5 as usize],
                    (false, false) => 0,
                };
                // One bypass pin per letter: the flop's D from outside, or the
                // 5FF's. The fill puts a 5FF only beside a LUT-fed flop and
                // unit moves keep that, so the two never meet.
                let x = u32::from(ff != NONE && !fed_by(lut, ff)) + u32::from(ff2 != NONE);
                debug_assert!(x <= 1, "two bypass users on one letter");
                Load { all: luts + x, x }
            })
            .fold(Load::default(), |a, b| a + b)
    };
    let fill_pins = |f: &Fill| -> Load {
        let mut pins: BTreeSet<(u8, &str)> = BTreeSet::new();
        for (c, b) in &f.cells {
            if p.nl.cell(*c).ty == "SLICE_LUTX" {
                for port in ["A1", "A2", "A3", "A4", "A5", "A6", "DI2", "WA7", "WA8"] {
                    if live(*c, port) {
                        pins.insert((b.as_bytes()[0], port));
                    }
                }
            }
        }
        let x = crate::slice::bypass_claims(p, &f.cells).0.used();
        Load { all: pins.len() as u32 + x, x }
    };
    let mut tile_ids: HashMap<(i32, i32), u32> = HashMap::new();
    let site_tile: Vec<u32> = sites
        .slices
        .iter()
        .map(|s| {
            let next = tile_ids.len() as u32;
            *tile_ids.entry((s.x, s.y)).or_insert(next)
        })
        .collect();
    let mut slice_pins: Vec<Vec<Load>> = (0..n)
        .map(|o| match objs[o].kind {
            Kind::Slice if is_logic[o] => vec![logic_pins(&letters[o])],
            Kind::Slice => fills[o].iter().map(fill_pins).collect(),
            _ => Vec::new(),
        })
        .collect();
    let mut tile_load: Vec<Load> = vec![Load::default(); tile_ids.len()];
    for o in 0..n {
        for (&s, &pins) in objs[o].sites.iter().zip(&slice_pins[o]) {
            let t = site_tile[s] as usize;
            tile_load[t] = tile_load[t] + pins;
        }
    }
    st.bypass_clashes = (0..n)
        .filter(|&o| objs[o].kind == Kind::Slice && !is_logic[o])
        .flat_map(|o| fills[o].iter())
        .map(|f| crate::slice::bypass_claims(p, &f.cells).1.len())
        .sum();
    let (pin_cost, byp_cost) = (opts.pins, opts.bypass);
    // Slice density per bin.
    let dens = opts.density;
    let bsz = dens.bin.max(1);
    let mut bin_ids: HashMap<(i32, i32), u32> = HashMap::new();
    let site_bin: Vec<u32> = sites
        .slices
        .iter()
        .map(|s| {
            let next = bin_ids.len() as u32;
            *bin_ids.entry((s.x / bsz, s.y / bsz)).or_insert(next)
        })
        .collect();
    let mut bin_cap = vec![0u32; bin_ids.len()];
    for &b in &site_bin {
        bin_cap[b as usize] += 1;
    }
    let mut bin_load = vec![0u32; bin_ids.len()];
    for o in 0..n {
        if objs[o].kind == Kind::Slice {
            for &s in &objs[o].sites {
                bin_load[site_bin[s] as usize] += 1;
            }
        }
    }
    let bin_excess = |b: usize, load: u32| -> f64 { (f64::from(load) - dens.target * f64::from(bin_cap[b])).max(0.0) };
    let total_dens = |bl: &[u32]| -> f64 { bl.iter().enumerate().map(|(b, &l)| bin_excess(b, l)).sum() };
    st.density_excess_start = total_dens(&bin_load);
    let total_excess = |tl: &[Load]| -> (u64, u64) {
        tl.iter().fold((0, 0), |(a, b), l| (a + u64::from(pin_cost.excess(l.all)), b + u64::from(byp_cost.excess(l.x))))
    };
    (st.pin_excess_start, st.bypass_excess_start) = total_excess(&tile_load);
    // The cost change of `changes` (tile, pins added) to the tile loads.
    let pin_delta = |tile_load: &[Load], changes: &[(u32, Delta)]| -> f64 {
        if !pin_cost.on() && !byp_cost.on() {
            return 0.0;
        }
        let mut merged: Vec<(u32, Delta)> = Vec::with_capacity(changes.len());
        for &(t, d) in changes {
            match merged.iter_mut().find(|m| m.0 == t) {
                Some(m) => {
                    m.1.all += d.all;
                    m.1.x += d.x;
                }
                None => merged.push((t, d)),
            }
        }
        merged
            .iter()
            .map(|&(t, d)| {
                let old = tile_load[t as usize];
                let new = old.apply(d);
                (f64::from(pin_cost.excess(new.all)) - f64::from(pin_cost.excess(old.all))) * pin_cost.weight
                    + (f64::from(byp_cost.excess(new.x)) - f64::from(byp_cost.excess(old.x))) * byp_cost.weight
            })
            .sum::<f64>()
    };
    let apply = |tile_load: &mut [Load], changes: &[(u32, Delta)]| {
        for &(t, d) in changes {
            tile_load[t as usize] = tile_load[t as usize].apply(d);
        }
    };

    // Nets: pins are placeable cells or fixed points.
    #[derive(Clone, Copy)]
    enum Pin {
        Cell(u32),
        Fixed(i32, i32),
    }
    let mut net_pins: Vec<Vec<Pin>> = Vec::new();
    let mut cell_nets: Vec<Vec<u32>> = vec![Vec::new(); cells.len()];
    {
        let mut seen: HashSet<NetId> = HashSet::new();
        for c in p.nl.cell_ids() {
            for port in p.nl.cell(c).ports.values() {
                let Some(net_id) = port.net else { continue };
                if p.nl.is_const(net_id) || !seen.insert(net_id) {
                    continue;
                }
                let net = p.nl.net(net_id);
                if net.users.iter().any(|(_, port)| port == "CK" || port == "CLK" || port.starts_with("CLKARDCLK") || port.starts_with("CLKBWRCLK")) {
                    continue;
                }
                let mut pins = Vec::new();
                for (pc, _) in net.driver.iter().chain(net.users.iter()) {
                    if let Some(&ci) = cidx.get(pc) {
                        pins.push(Pin::Cell(ci));
                    } else if let Some(Loc::Grid { x, y, .. }) = &p.nl.cell(*pc).bel {
                        pins.push(Pin::Fixed(*x, *y));
                    }
                }
                if pins.len() < 2 || pins.len() > opts.max_fanout {
                    continue;
                }
                let ni = net_pins.len() as u32;
                let mut cs: Vec<u32> = pins.iter().filter_map(|p| if let Pin::Cell(c) = p { Some(*c) } else { None }).collect();
                cs.sort_unstable();
                cs.dedup();
                for c in cs {
                    cell_nets[c as usize].push(ni);
                }
                net_pins.push(pins);
            }
        }
    }
    let mut pos: Vec<Vec<(i32, i32)>> = objs.iter().map(|o| o.sites.iter().map(|&s| site_xy(o.kind, s)).collect()).collect();
    let cost_of = |pins: &[Pin], pos: &Vec<Vec<(i32, i32)>>, cell_at: &Vec<(u32, u32)>| -> f64 {
        let (mut x0, mut x1, mut y0, mut y1) = (i32::MAX, i32::MIN, i32::MAX, i32::MIN);
        for p in pins {
            let (x, y) = match *p {
                Pin::Cell(c) => {
                    let (o, k) = cell_at[c as usize];
                    pos[o as usize][k as usize]
                }
                Pin::Fixed(x, y) => (x, y),
            };
            x0 = x0.min(x);
            x1 = x1.max(x);
            y0 = y0.min(y);
            y1 = y1.max(y);
        }
        ((x1 - x0) + (y1 - y0)) as f64
    };
    let mut net_cost: Vec<f64> = net_pins.iter().map(|ps| cost_of(ps, &pos, &cell_at)).collect();
    let mut total: f64 = net_cost.iter().sum();
    st.hpwl_start = total;

    // Annealing: object moves (a slice, a chain, a hard block) and unit
    // moves (a LUT, or a LUT with its flop, between logic slices).
    let (w, h) = sites.slices.iter().fold((0, 0), |(w, h), s| (w.max(s.x), h.max(s.y)));
    let maxwin = w.max(h) as f64;
    let mut window = maxwin;
    let avg0 = total / net_pins.len().max(1) as f64;
    let mut temp = (opts.t0_scale * avg0).max(1.0);
    let mut stamp: Vec<u32> = vec![0; net_pins.len()];
    let mut epoch: u32 = 0;
    let mut touched: Vec<u32> = Vec::new();
    let mut old_costs: Vec<(u32, f64)> = Vec::new();
    let logic_objs: Vec<usize> = (0..n).filter(|&o| is_logic[o]).collect();
    let mut polish = 0usize;
    loop {
        let moves = n * opts.inner;
        let mut accepted = 0usize;
        let mut evaluated = 0usize;
        for _ in 0..moves {
            let unit_move = polish > 0 || (!logic_objs.is_empty() && window <= opts.unit_window && rng.unit() < opts.unit_frac);
            let oi = if unit_move { logic_objs[rng.below(logic_objs.len())] } else { rng.below(n) };
            let k = objs[oi].kind;
            let ki = kidx(k);
            let (ox, oy) = pos[oi][0];
            let win = window.max(1.0) as i32;
            let mut target = None;
            for _ in 0..16 {
                let tx = ox + rng.below((2 * win + 1) as usize) as i32 - win;
                let ty = oy + rng.below((2 * win + 1) as usize) as i32 - win;
                if let Some(list) = at.get(&(ki, tx, ty)) {
                    let s = list[rng.below(list.len())];
                    if s != objs[oi].sites[0] {
                        target = Some(s);
                        break;
                    }
                }
            }
            let Some(ts) = target else { continue };

            if unit_move {
                // Source letter and what moves.
                let la = rng.below(4);
                let [lut_a, ff_a, _, l5_a] = letters[oi][la];
                if lut_a == NONE || l5_a != NONE {
                    continue;
                }
                let pair_a = fed_by(lut_a, ff_a);
                let Some(oj) = owner[0][ts] else { continue };
                if !is_logic[oj] || oj == oi {
                    continue;
                }
                let lb = rng.below(4);
                let [lut_b, ff_b, ff2_b, l5_b] = letters[oj][lb];
                if l5_b != NONE {
                    continue;
                }
                let pair_b = fed_by(lut_b, ff_b);
                // Legal shapes: lone LUT <-> lone LUT or empty LUT slot; pair
                // <-> pair of the same control set, or pair -> empty letter
                // of a slice on that control set (or without flops).
                let ok = if !pair_a {
                    lut_b == NONE || !pair_b
                } else if pair_b {
                    ctrl_of_cell[&ff_a] == ctrl_of_cell[&ff_b]
                } else {
                    lut_b == NONE && ff_b == NONE && (slice_ffs[oj].1 == 0 || slice_ffs[oj].0 == ctrl_of_cell[&ff_a]) && (ff2_b == NONE || true)
                };
                if !ok {
                    continue;
                }
                let moving_a: Vec<u32> = if pair_a { vec![lut_a, ff_a] } else { vec![lut_a] };
                let moving_b: Vec<u32> = if pair_a {
                    if pair_b { vec![lut_b, ff_b] } else { vec![] }
                } else if lut_b != NONE {
                    vec![lut_b]
                } else {
                    vec![]
                };
                // A pair leaving its letter: its 5FF (X-fed lone flop) may
                // stay, legal with no main flop. A pair arriving on a letter
                // holding a 5FF: legal (main flop LUT-fed).
                for &c in &moving_a {
                    cell_at[c as usize] = (oj as u32, 0);
                }
                for &c in &moving_b {
                    cell_at[c as usize] = (oi as u32, 0);
                }
                epoch = epoch.wrapping_add(1);
                touched.clear();
                for &c in moving_a.iter().chain(&moving_b) {
                    for &ni in &cell_nets[c as usize] {
                        if stamp[ni as usize] != epoch {
                            stamp[ni as usize] = epoch;
                            touched.push(ni);
                        }
                    }
                }
                old_costs.clear();
                evaluated += 1;
                let mut delta = 0.0;
                for &ni in &touched {
                    let c = cost_of(&net_pins[ni as usize], &pos, &cell_at);
                    old_costs.push((ni, net_cost[ni as usize]));
                    delta += c - net_cost[ni as usize];
                    net_cost[ni as usize] = c;
                }
                // The two slices' letters after the move, and their pins.
                let (mut na, mut nb) = (letters[oi], letters[oj]);
                na[la][0] = if pair_a { if pair_b { lut_b } else { NONE } } else { if lut_b != NONE { lut_b } else { NONE } };
                nb[lb][0] = lut_a;
                if pair_a {
                    na[la][1] = if pair_b { ff_b } else { NONE };
                    nb[lb][1] = ff_a;
                }
                let (pins_a, pins_b) = (logic_pins(&na), logic_pins(&nb));
                let changes = [
                    (site_tile[objs[oi].sites[0]], Load::delta(pins_a, slice_pins[oi][0])),
                    (site_tile[objs[oj].sites[0]], Load::delta(pins_b, slice_pins[oj][0])),
                ];
                let dpin = pin_delta(&tile_load, &changes);
                if delta + dpin <= 0.0 || rng.unit() < (-(delta + dpin) / temp).exp() {
                    accepted += 1;
                    total += delta;
                    apply(&mut tile_load, &changes);
                    slice_pins[oi][0] = pins_a;
                    slice_pins[oj][0] = pins_b;
                    // Letters, object cell lists, flop counts.
                    letters[oi] = na;
                    letters[oj] = nb;
                    if pair_a {
                        if !pair_b {
                            slice_ffs[oi].1 -= 1;
                            slice_ffs[oj].1 += 1;
                            slice_ffs[oj].0 = ctrl_of_cell[&ff_a];
                        }
                    }
                    for &c in &moving_a {
                        obj_cells[oi].retain(|&x| x != c);
                        obj_cells[oj].push(c);
                    }
                    for &c in &moving_b {
                        obj_cells[oj].retain(|&x| x != c);
                        obj_cells[oi].push(c);
                    }
                } else {
                    for &(ni, c) in &old_costs {
                        net_cost[ni as usize] = c;
                    }
                    for &c in &moving_a {
                        cell_at[c as usize] = (oi as u32, 0);
                    }
                    for &c in &moving_b {
                        cell_at[c as usize] = (oj as u32, 0);
                    }
                }
                continue;
            }

            // Object move.
            let len = objs[oi].sites.len();
            let (new_run, other): (Vec<usize>, Option<usize>) = if len > 1 {
                match run_from(ts, len) {
                    Some(r) if fits(&objs[oi], &fills[oi], &r, &owner, oi) => (r, None),
                    _ => continue,
                }
            } else {
                let other = owner[ki][ts];
                if let Some(oj) = other {
                    if objs[oj].sites.len() > 1 {
                        continue;
                    }
                    if k == Kind::Slice && ((objs[oi].need_m && !sites.slices[ts].m) || (objs[oj].need_m && !sites.slices[objs[oi].sites[0]].m)) {
                        continue;
                    }
                } else {
                    let ok = match k {
                        Kind::Slice => !objs[oi].need_m || sites.slices[ts].m,
                        Kind::Dsp => true,
                        Kind::Ramb36 => {
                            let t = sites.ramb36[ts].tile;
                            b18.get(&t).copied().unwrap_or(0) == 0 && !b36.get(&t).copied().unwrap_or(false)
                        }
                        Kind::Ramb18 => !b36.get(&sites.ramb18[ts].tile).copied().unwrap_or(false),
                    };
                    if !ok {
                        continue;
                    }
                }
                (vec![ts], other)
            };
            let old_run = objs[oi].sites.clone();
            let old_pos_i = pos[oi].clone();
            pos[oi] = new_run.iter().map(|&s| site_xy(k, s)).collect();
            if let Some(oj) = other {
                pos[oj] = vec![site_xy(k, old_run[0])];
            }
            epoch = epoch.wrapping_add(1);
            touched.clear();
            for &c in obj_cells[oi].iter().chain(other.map(|oj| &obj_cells[oj]).into_iter().flatten()) {
                for &ni in &cell_nets[c as usize] {
                    if stamp[ni as usize] != epoch {
                        stamp[ni as usize] = epoch;
                        touched.push(ni);
                    }
                }
            }
            old_costs.clear();
            evaluated += 1;
            let mut delta = 0.0;
            for &ni in &touched {
                let c = cost_of(&net_pins[ni as usize], &pos, &cell_at);
                old_costs.push((ni, net_cost[ni as usize]));
                delta += c - net_cost[ni as usize];
                net_cost[ni as usize] = c;
            }
            // Tile pins: this object's slices leave their tiles for the new
            // run's; a swapped slice goes the other way.
            let mut changes: Vec<(u32, Delta)> = Vec::new();
            if k == Kind::Slice {
                for (&s, &pins) in old_run.iter().zip(&slice_pins[oi]) {
                    changes.push((site_tile[s], -Delta::from(pins)));
                }
                for (&s, &pins) in new_run.iter().zip(&slice_pins[oi]) {
                    changes.push((site_tile[s], Delta::from(pins)));
                }
                if let Some(oj) = other {
                    let pins = Delta::from(slice_pins[oj][0]);
                    changes.push((site_tile[new_run[0]], -pins));
                    changes.push((site_tile[old_run[0]], pins));
                }
            }
            // Bin loads: this object's slices leave their bins for the new
            // run's; a swapped slice goes the other way.
            let mut dchg: Vec<(u32, i32)> = Vec::new();
            if k == Kind::Slice && dens.on() {
                for &s in &old_run {
                    dchg.push((site_bin[s], -1));
                }
                for &s in &new_run {
                    dchg.push((site_bin[s], 1));
                }
                if other.is_some() {
                    dchg.push((site_bin[new_run[0]], -1));
                    dchg.push((site_bin[old_run[0]], 1));
                }
            }
            let ddens = if dchg.is_empty() {
                0.0
            } else {
                let mut merged: Vec<(u32, i32)> = Vec::new();
                for &(b, d) in &dchg {
                    match merged.iter_mut().find(|m| m.0 == b) {
                        Some(m) => m.1 += d,
                        None => merged.push((b, d)),
                    }
                }
                merged
                    .iter()
                    .map(|&(b, d)| {
                        let l = bin_load[b as usize];
                        bin_excess(b as usize, (i64::from(l) + i64::from(d)).max(0) as u32) - bin_excess(b as usize, l)
                    })
                    .sum::<f64>()
                    * dens.weight
            };
            let dpin = pin_delta(&tile_load, &changes);
            if delta + dpin + ddens <= 0.0 || rng.unit() < (-(delta + dpin + ddens) / temp).exp() {
                accepted += 1;
                total += delta;
                apply(&mut tile_load, &changes);
                for &(b, d) in &dchg {
                    bin_load[b as usize] = (i64::from(bin_load[b as usize]) + i64::from(d)).max(0) as u32;
                }
                for &s in &old_run {
                    owner[ki][s] = None;
                }
                if let Some(oj) = other {
                    owner[ki][old_run[0]] = Some(oj);
                    objs[oj].sites = vec![old_run[0]];
                } else {
                    match k {
                        Kind::Ramb36 => {
                            b36.insert(sites.ramb36[old_run[0]].tile, false);
                            b36.insert(sites.ramb36[new_run[0]].tile, true);
                        }
                        Kind::Ramb18 => {
                            *b18.entry(sites.ramb18[old_run[0]].tile).or_default() -= 1;
                            *b18.entry(sites.ramb18[new_run[0]].tile).or_default() += 1;
                        }
                        _ => {}
                    }
                }
                for &s in &new_run {
                    owner[ki][s] = Some(oi);
                }
                objs[oi].sites = new_run;
            } else {
                for &(ni, c) in &old_costs {
                    net_cost[ni as usize] = c;
                }
                pos[oi] = old_pos_i;
                if let Some(oj) = other {
                    pos[oj] = vec![site_xy(k, objs[oj].sites[0])];
                }
            }
        }
        st.moves += moves;
        st.accepted += accepted;
        let rate = accepted as f64 / evaluated.max(1) as f64;
        st.trace.push((temp, window, rate, total));
        let alpha = if rate > 0.96 {
            0.5
        } else if rate > 0.8 {
            0.9
        } else if rate > 0.15 {
            0.95
        } else {
            0.8
        };
        if polish > 0 {
            polish -= 1;
            if polish == 0 {
                break;
            }
            continue;
        }
        temp *= alpha;
        window = (window * (1.0 - 0.44 + rate)).clamp(1.0, maxwin);
        if temp < 0.005 * total / net_pins.len().max(1) as f64 || st.moves > opts.max_moves_per_obj * n {
            if opts.polish == 0 || logic_objs.is_empty() {
                break;
            }
            // Greedy unit-move polish on the converged placement.
            polish = opts.polish;
            temp = 1e-6;
            window = opts.unit_window;
        }
    }
    st.hpwl_end = total;
    (st.pin_excess_end, st.bypass_excess_end) = total_excess(&tile_load);
    st.density_excess_end = total_dens(&bin_load);
    st.density_bins_over = bin_load.iter().enumerate().filter(|&(b, &l)| bin_excess(b, l) > 0.0).count();
    st.tiles_over_cap = tile_load.iter().filter(|l| l.all > pin_cost.cap).count();
    st.tile_pins_max = tile_load.iter().map(|l| l.all).max().unwrap_or(0);
    st.bypass_tiles_over_cap = tile_load.iter().filter(|l| l.x > byp_cost.cap).count();
    st.bypass_tile_max = tile_load.iter().map(|l| l.x).max().unwrap_or(0);

    // Bels: logic slices from their letters, the rest from their fills.
    for (o, ob) in objs.iter().enumerate() {
        match ob.kind {
            Kind::Slice if is_logic[o] => {
                let s = &sites.slices[ob.sites[0]];
                for (l, lt) in letters[o].iter().enumerate() {
                    for (slot, &ci) in lt.iter().enumerate() {
                        if ci == NONE {
                            continue;
                        }
                        let bel = format!("{}{}", letter(l as i32), ["6LUT", "FF", "5FF", "5LUT"][slot]);
                        p.nl.cell_mut(cells[ci as usize]).bel = Some(Loc::Grid { x: s.x, y: s.y, site: s.rel.clone(), bel });
                    }
                }
            }
            Kind::Slice => {
                for (fl, &si) in fills[o].iter().zip(&ob.sites) {
                    let s = &sites.slices[si];
                    for (c, b) in &fl.cells {
                        p.nl.cell_mut(*c).bel = Some(Loc::Grid { x: s.x, y: s.y, site: s.rel.clone(), bel: b.clone() });
                    }
                }
            }
            Kind::Dsp => {
                let s = &sites.dsp[ob.sites[0]];
                p.nl.cell_mut(ob.cells[0]).bel = Some(Loc::Grid { x: s.x, y: s.y, site: s.rel.clone(), bel: "DSP48E1".into() });
            }
            Kind::Ramb36 => {
                let s = &sites.ramb36[ob.sites[0]];
                p.nl.cell_mut(ob.cells[0]).bel = Some(Loc::Grid { x: s.x, y: s.y, site: s.rel.clone(), bel: "RAMB36E1~RAMB36E1".into() });
            }
            Kind::Ramb18 => {
                let s = &sites.ramb18[ob.sites[0]];
                let bel = if s.rel.ends_with("Y0") { "RAMB18E1~RAMB18E1" } else { "RAMB18E1" };
                p.nl.cell_mut(ob.cells[0]).bel = Some(Loc::Grid { x: s.x, y: s.y, site: s.rel.clone(), bel: bel.into() });
            }
        }
    }
    let mut bufg = sites.bufg.iter();
    for c in p.nl.cells_of("BUFGCTRL") {
        if p.nl.cell(c).bel.is_none()
            && let Some(s) = bufg.next()
        {
            p.nl.cell_mut(c).bel = Some(Loc::Grid { x: s.x, y: s.y, site: s.rel.clone(), bel: "BUFGCTRL".into() });
        }
    }
    // MMCMs: MMCME2_ADV_X0Y0 first (nextpnr's choice for a clock it feeds
    // to the BUFGs above it), then the rest in device order.
    let mut mmcm: Vec<&HardSite> = sites.mmcm.iter().collect();
    mmcm.sort_by_key(|s| (s.rel != "MMCME2_ADV_X0Y0", s.rel.clone()));
    let mut mmcm = mmcm.into_iter();
    for c in p.nl.cells_of("MMCME2_ADV_MMCME2_ADV") {
        if p.nl.cell(c).bel.is_none()
            && let Some(s) = mmcm.next()
        {
            p.nl.cell_mut(c).bel = Some(Loc::Grid { x: s.x, y: s.y, site: s.rel.clone(), bel: "MMCME2_ADV".into() });
        }
    }
    for c in p.nl.cells_of("PS7_PS7") {
        if let Some(s) = sites.ps7.first() {
            p.nl.cell_mut(c).bel = Some(Loc::Grid { x: s.x, y: s.y, site: s.rel.clone(), bel: "PS7".into() });
        }
    }
    for (c, b) in [("$PACKER_GND_DRV", "PSEUDO_GND"), ("$PACKER_VCC_DRV", "PSEUDO_VCC")] {
        if let Some(id) = p.nl.cell_id(c) {
            p.nl.cell_mut(id).bel = Some(Loc::Site { site: "X0Y0".into(), bel: b.into() });
        }
    }
    st.seconds = t0.elapsed().as_secs_f64();
    Ok(st)
}

/// Half-perimeter wirelength of the current bels, over the nets the
/// annealer counts (no constants, clocks, or nets above `max_fanout`).
pub fn hpwl(p: &Packer, max_fanout: usize) -> f64 {
    let pos = |c: CellId| match &p.nl.cell(c).bel {
        Some(Loc::Grid { x, y, .. }) => Some((*x, *y)),
        _ => None,
    };
    let mut seen: HashSet<NetId> = HashSet::new();
    let mut total = 0.0;
    for c in p.nl.cell_ids() {
        for port in p.nl.cell(c).ports.values() {
            let Some(n) = port.net else { continue };
            if p.nl.is_const(n) || !seen.insert(n) {
                continue;
            }
            let net = p.nl.net(n);
            if net.users.iter().any(|(_, port)| port == "CK" || port == "CLK" || port.starts_with("CLKARDCLK") || port.starts_with("CLKBWRCLK")) {
                continue;
            }
            let pts: Vec<(i32, i32)> = net.driver.iter().chain(net.users.iter()).filter_map(|(c, _)| pos(*c)).collect();
            if pts.len() < 2 || pts.len() > max_fanout {
                continue;
            }
            let (x0, x1) = pts.iter().fold((i32::MAX, i32::MIN), |(a, b), &(x, _)| (a.min(x), b.max(x)));
            let (y0, y1) = pts.iter().fold((i32::MAX, i32::MIN), |(a, b), &(_, y)| (a.min(y), b.max(y)));
            total += ((x1 - x0) + (y1 - y0)) as f64;
        }
    }
    total
}
