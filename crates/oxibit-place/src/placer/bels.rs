//! Bels for slice logic, once every slice instance has a site.
//!
//! Not an optimiser: the slice model is filled in a fixed order. In a
//! slice:
//!
//! 1. the leaf LUTs of an F7 mux go where the mux needs them (the `1`
//!    input's LUT on letter A, the `0` input's on B, as `pack::muxf` lays
//!    a tree out) and the mux takes F7AMUX; a second mux in the slice
//!    takes C, D and F7BMUX;
//! 2. each other LUT takes the 6LUT of the next free letter, and the flop
//!    packing paired with it takes that letter's FF, fed from O6;
//! 3. each remaining flop takes the FF of an empty letter if one is left,
//!    else of a letter whose FF is free. Such a flop is fed through the
//!    letter's bypass pin (AX..DX), which is the router's to reach. In a
//!    slice with an F7 mux, letter A's bypass is the mux select and is
//!    left alone.
//!
//! A letter packing fractured (`Packer::fractured`) also gets its second
//! LUT on the 5LUT and that LUT's flop on the 5FF. The pins the two LUTs
//! share are laid out afterwards by `place::legalise`.
//! Only single F7 muxes are laid out here. F8 trees, carry chains and other
//! slice-shaped macros are `assign_macros`'s; what that cannot lay out is
//! reported as unsupported, not guessed at.

use std::collections::{BTreeMap, HashMap, HashSet};

use serde_json::{Value, json};

use super::macros::slice_shaped;
use super::{InstKind, Problem};
use crate::device::Device;
use crate::netlist::CellId;
use crate::pack::{Packer, slot};

#[derive(Debug, Clone, Default)]
pub struct Stats {
    pub slices: usize,
    pub luts: usize,
    pub ffs: usize,
    /// Flops on the FF of the letter their own LUT is in.
    pub paired_ffs: usize,
    /// Flops fed through a bypass pin, and how many of those share the
    /// letter with a LUT that is not theirs.
    pub bypass_ffs: usize,
    pub shared_letters: usize,
    /// Letters with a LUT on the 5LUT as well.
    pub fractured: usize,
    /// F7 muxes placed with their leaves.
    pub f7_muxes: usize,
    /// F8 muxes placed with their two F7 muxes.
    pub f8_muxes: usize,
    /// Slice-shaped macros given a slice of their own (`assign_macros`).
    pub slice_macros: usize,
    /// Carry chains laid out on their run of slices (`assign_macros`).
    pub chain_macros: usize,
    /// Of those, the ones needing a SLICEM (distributed RAM, shift
    /// registers), and how far (tiles, Manhattan) each had to go from `at`
    /// (`assign_macros`).
    pub memory_macros: usize,
    pub memory_disp: Vec<f64>,
    /// SLICEM sites in the region, and how many slice logic holds.
    pub slicem_sites: usize,
    pub slicem_by_logic: usize,
    /// Macros this pass does not know how to lay out.
    pub unsupported: Vec<String>,
    /// After everything: cells without a bel, and bels given twice.
    pub unplaced: Vec<String>,
    pub collisions: Vec<String>,
}

impl Stats {
    pub fn to_json(&self) -> Value {
        json!({"slices": self.slices, "luts": self.luts, "ffs": self.ffs, "paired_ffs": self.paired_ffs,
               "bypass_ffs": self.bypass_ffs, "shared_letters": self.shared_letters, "fractured": self.fractured, "f7_muxes": self.f7_muxes, "f8_muxes": self.f8_muxes, "slice_macros": self.slice_macros, "chain_macros": self.chain_macros,
               "memory_macros": self.memory_macros, "memory_disp": disp_summary(&self.memory_disp),
               "slicem_sites": self.slicem_sites, "slicem_by_logic": self.slicem_by_logic,
               "unsupported": self.unsupported, "unplaced": self.unplaced.len(),
               "unplaced_examples": self.unplaced.iter().take(8).collect::<Vec<_>>(), "collisions": self.collisions})
    }

    pub fn ok(&self) -> bool {
        self.unsupported.is_empty() && self.unplaced.is_empty() && self.collisions.is_empty()
    }
}

/// Mean, max, and how many over 5 and 15 tiles.
fn disp_summary(d: &[f64]) -> Value {
    let mean = if d.is_empty() { 0.0 } else { d.iter().sum::<f64>() / d.len() as f64 };
    let max = d.iter().copied().fold(0.0, f64::max);
    json!({"mean": mean, "max": max, "over_5": d.iter().filter(|&&x| x > 5.0).count(), "over_15": d.iter().filter(|&&x| x > 15.0).count()})
}

/// One letter being filled: the LUT on its 6LUT and that LUT's flop on
/// its FF (or a bypass-fed flop there), and for a fractured letter the
/// LUT on its 5LUT and that LUT's flop on its 5FF.
#[derive(Debug, Clone, Copy, Default)]
struct Letter {
    lut: Option<CellId>,
    ff: Option<CellId>,
    lut5: Option<CellId>,
    ff5: Option<CellId>,
    /// Its 6LUT must stay empty: an F7 mux reads it as its grounded input
    /// (an unused LUT reads 0).
    reserved: bool,
}

/// The flop packing paired with `lut`.
fn paired(p: &Packer, lut: CellId) -> Option<CellId> {
    p.clusters.get(&lut).and_then(|cl| cl.members.first()).map(|&(ff, _)| ff)
}

/// The bel at slot `z` of a slice (`letter << 4 | function`, as clusters
/// record their members).
pub fn bel_at(z: i32) -> Option<String> {
    let letter = z >> 4;
    if !(0..4).contains(&letter) {
        return None;
    }
    let l = (b'A' + letter as u8) as char;
    Some(match z & 0xF {
        slot::LUT6 => format!("{l}6LUT"),
        slot::LUT5 => format!("{l}5LUT"),
        slot::FF => format!("{l}FF"),
        slot::FF2 => format!("{l}5FF"),
        // One F7 mux per letter pair, one F8 per slice.
        slot::F7MUX if letter == 0 => "F7AMUX".into(),
        slot::F7MUX if letter == 2 => "F7BMUX".into(),
        slot::F8MUX if letter == 0 => "F8MUX".into(),
        slot::CARRY4 if letter == 0 => "CARRY4".into(),
        _ => return None,
    })
}

/// Slice-shaped macros (wide-mux trees packing kept whole, distributed
/// RAM, shift registers): each cluster takes a whole free slice, the
/// nearest to where placement left it, with its cells on the slots its
/// offsets name. Memory needs a SLICEM, and takes the one legalisation
/// reserved for it (`site_of`) when there is one. `at`: where the
/// placement left each instance (a reserved macro: its site already, so
/// its displacement here is 0; `legalise_spread::Stats` has the one from
/// global placement). A cluster that spans tiles (a
/// carry chain), or whose slots this cannot name or that collide, stays
/// unplaced and is reported. Then `st`'s unplaced, collision and
/// unsupported lists are brought up to date.
///
/// First cut, to see a larger design through: nothing here has been
/// checked against the router beyond the single-F7 case in `assign`.
pub fn assign_macros(p: &mut Packer, dev: &Device, prob: &Problem, site_of: &[Option<u32>], at: &super::PlacementState, st: &mut Stats) {
    let mut taken = vec![false; prob.sites.len()];
    for (i, s) in site_of.iter().enumerate() {
        let Some(s) = *s else { continue };
        let rows = prob.chain_rows.get(i).copied().unwrap_or(0).max(1);
        for t in prob.chain_run(s, rows).unwrap_or_else(|| vec![s]) {
            taken[t as usize] = true;
        }
    }
    let slicem: Vec<bool> = prob.sites.iter().map(|s| dev.site_type(&s.name) == Some("SLICEM")).collect();
    st.slicem_sites = slicem.iter().filter(|&&m| m).count();
    st.slicem_by_logic = (0..prob.insts.len())
        .filter(|&i| prob.insts[i].kind == InstKind::Slice)
        .filter_map(|i| site_of[i])
        .filter(|&s| slicem[s as usize])
        .count();
    st.memory_macros = 0;
    st.memory_disp.clear();
    for (i, inst) in prob.insts.iter().enumerate() {
        let cells = &inst.cells;
        if inst.kind != InstKind::Macro || !cells.iter().all(|&c| slice_shaped(&p.nl.cell(c).ty)) || cells.iter().all(|&c| p.nl.cell(c).bel.is_some()) {
            continue;
        }
        // A carry chain: each member on the slot its offset names, in the
        // slice of its row of the run legalisation gave it.
        if let Some(&rows) = prob.chain_rows.get(i).filter(|&&n| n > 0) {
            let Some(run) = site_of[i].and_then(|s| prob.chain_run(s, rows)) else { continue };
            let Some(&root) = cells.first().and_then(|c| p.cluster_of.get(c)) else { continue };
            let cl = p.clusters.get(&root).cloned().unwrap_or_default();
            let root_z = cl.root_abs_z.unwrap_or(slot::CARRY4);
            let slots = std::iter::once((root, root_z, 0)).chain(cl.members.iter().map(|(m, o)| (*m, if o.abs { o.z } else { root_z + o.z }, o.dy)));
            let plan: Option<Vec<(CellId, u32, String)>> = slots
                .map(|(c, z, dy)| Some((c, *run.get(crate::pack::carry::chain_row(dy))?, bel_at(z)?)))
                .collect();
            let Some(plan) = plan else { continue };
            let mut distinct: Vec<(u32, &str)> = plan.iter().map(|(_, s, b)| (*s, b.as_str())).collect();
            distinct.sort_unstable();
            distinct.dedup();
            if plan.len() != cells.len() || distinct.len() != plan.len() {
                continue;
            }
            for (c, s, bel) in plan {
                p.nl.cell_mut(c).bel = dev.grid_loc(&prob.sites[s as usize].name, &bel);
            }
            st.chain_macros += 1;
            continue;
        }
        // An F7 mux with both inputs grounded: any slice whose A and B LUTs
        // stay empty, which a slice of its own does.
        let lone_mux = matches!(cells[..], [m] if p.nl.cell(m).ty == "SELMUX2_1" && ["0", "1"].iter().all(|port| p.nl.cell(m).net(port) == Some(p.nl.gnd)));
        // A shift register or LUT RAM packing left on its own (no cluster):
        // letter D of a SLICEM, whose data input needs no DI1MUX line.
        let lone_memory = matches!(cells[..], [c] if !p.cluster_of.contains_key(&c) && {
            let a = &p.nl.cell(c).attrs;
            a.contains_key("X_LUT_AS_SRL") || a.contains_key("X_LUT_AS_DRAM")
        });
        let root = cells.iter().copied().find(|c| p.cluster_of.get(c) == Some(c));
        let Some(root) = root.or((lone_mux || lone_memory).then(|| cells[0])) else { continue };
        let cl = p.clusters.get(&root).cloned().unwrap_or_default();
        // A mux root sits on the F8 slot if muxes hang off it, else on F7.
        let f8 = cl.members.iter().any(|(m, _)| p.nl.cell(*m).ty == "SELMUX2_1");
        let root_z = cl.root_abs_z.unwrap_or(match p.nl.cell(root).ty.as_str() {
            "SELMUX2_1" if f8 => slot::F8MUX,
            "SELMUX2_1" => slot::F7MUX,
            _ if lone_memory => (3 << 4) | slot::LUT6,
            _ => slot::LUT6,
        });
        let slots = std::iter::once((root, root_z, true)).chain(cl.members.iter().map(|(m, off)| (*m, if off.abs { off.z } else { root_z + off.z }, (off.dx, off.dy) == (0, 0))));
        let plan: Vec<(CellId, String)> = slots.filter_map(|(c, z, here)| bel_at(z).filter(|_| here).map(|b| (c, b))).collect();
        let mut distinct: Vec<&str> = plan.iter().map(|(_, b)| b.as_str()).collect();
        distinct.sort_unstable();
        distinct.dedup();
        if plan.len() != cells.len() || distinct.len() != plan.len() {
            continue;
        }
        let memory = cells.iter().any(|&c| {
            let a = &p.nl.cell(c).attrs;
            a.contains_key("X_LUT_AS_DRAM") || a.contains_key("X_LUT_AS_SRL")
        });
        let here = (at.x[i], at.y[i]);
        let reserved = site_of[i].map(|s| s as usize).filter(|&s| !memory || slicem[s]);
        let free = reserved.or_else(|| {
            (0..prob.sites.len()).filter(|&s| !taken[s] && (!memory || slicem[s])).min_by(|&a, &b| {
                let d = |s: usize| (prob.sites[s].x - here.0).abs() + (prob.sites[s].y - here.1).abs();
                d(a).total_cmp(&d(b)).then(a.cmp(&b))
            })
        });
        let Some(s) = free else { continue };
        taken[s] = true;
        if memory {
            st.memory_macros += 1;
            st.memory_disp.push((prob.sites[s].x - here.0).abs() + (prob.sites[s].y - here.1).abs());
        }
        for (c, bel) in plan {
            p.nl.cell_mut(c).bel = dev.grid_loc(&prob.sites[s].name, &bel);
        }
        st.slice_macros += 1;
    }
    audit(p, st);
}

/// Bring `st`'s lists up to date with the design as it stands: cells
/// without a bel, bels given twice, and of the macros reported as
/// unsupported those still unplaced.
pub fn audit(p: &Packer, st: &mut Stats) {
    st.unplaced.clear();
    st.collisions.clear();
    let mut seen: HashMap<String, String> = HashMap::new();
    for c in p.nl.cell_ids() {
        let cell = p.nl.cell(c);
        match &cell.bel {
            None if !cell.ty.starts_with("PSEUDO_") => st.unplaced.push(cell.name.clone()),
            Some(l) => {
                if let Some(prev) = seen.insert(l.text(), cell.name.clone())
                    && st.collisions.len() < 20
                {
                    st.collisions.push(format!("{} and {} on {}", prev, cell.name, l.text()));
                }
            }
            None => {}
        }
    }
    let unplaced: HashSet<&str> = st.unplaced.iter().map(String::as_str).collect();
    st.unsupported.retain(|n| unplaced.contains(n.as_str()));
}

/// Take back the bels `assign` wrote, so the slices can be placed again.
/// Cells the design pins to a bel (`BEL` attribute) keep theirs.
pub fn clear(p: &mut Packer) {
    for c in p.nl.cell_ids() {
        let cell = p.nl.cell(c);
        if slice_shaped(&cell.ty) && cell.bel.is_some() && !cell.attrs.contains_key("BEL") {
            p.nl.cell_mut(c).bel = None;
        }
    }
}

/// Write bels for the slice instances of `prob`, each on the site
/// `site_of` gives it (`Placement::site_of`).
pub fn assign(p: &mut Packer, dev: &Device, prob: &Problem, site_of: &[Option<u32>]) -> Result<Stats, String> {
    let mut st = Stats::default();
    // Leaf LUT -> (its mux, the letter it must take).
    let mut leaf: HashMap<CellId, (CellId, usize)> = HashMap::new();
    for m in p.nl.cells_of("SELMUX2_1") {
        for (port, letter) in [("1", 0), ("0", 1)] {
            if let Some((d, _)) = p.nl.cell(m).net(port).and_then(|n| p.nl.driver(n)) {
                leaf.insert(d, (m, letter));
            }
        }
    }
    // Mux -> which of its inputs are tied to ground, by letter (`1`, `0`).
    let grounded: HashMap<CellId, [bool; 2]> = p
        .nl
        .cells_of("SELMUX2_1")
        .into_iter()
        .map(|m| (m, ["1", "0"].map(|port| p.nl.cell(m).net(port) == Some(p.nl.gnd))))
        .collect();
    // The LUT leaves a mux has to find in one slice before it can join them.
    let wants = |m: &CellId| 2 - grounded[m].iter().filter(|g| **g).count();
    // F7 mux -> the F8 mux it feeds, and its half of the slice there: the
    // F8's `1` input is the F7 on letters A and B, its `0` input the one
    // on C and D.
    let mut under: HashMap<CellId, (CellId, usize)> = HashMap::new();
    // F8 mux -> its F7 mux on each half.
    let mut kids: HashMap<CellId, [Option<CellId>; 2]> = HashMap::new();
    for m in p.nl.cells_of("SELMUX2_1") {
        for (port, half) in [("1", 0), ("0", 1)] {
            if let Some((d, _)) = p.nl.cell(m).net(port).and_then(|n| p.nl.driver(n))
                && p.nl.cell(d).ty == "SELMUX2_1"
            {
                under.insert(d, (m, half));
                kids.entry(m).or_default()[half] = Some(d);
            }
        }
    }
    // LUTs that ride on another LUT's letter.
    let low: HashSet<CellId> = p.fractured.values().copied().collect();
    // Mux -> the site holding its leaves so far, and how many.
    let mut mux_at: BTreeMap<CellId, (String, usize, &str)> = BTreeMap::new();

    for (i, inst) in prob.insts.iter().enumerate() {
        // Memory macros have sites too; `assign_macros` lays those out.
        if inst.kind != InstKind::Slice {
            continue;
        }
        let Some(s) = site_of[i] else { continue };
        let site = &prob.sites[s as usize].name;
        let mut letters = [Letter::default(); 4];
        let (mut units, mut lone_ffs): (Vec<Letter>, Vec<CellId>) = (Vec::new(), Vec::new());
        for &c in &inst.cells {
            match p.nl.cell(c).ty.as_str() {
                "SLICE_LUTX" if !low.contains(&c) => {
                    let lut5 = p.fractured.get(&c).copied();
                    let u = Letter {
                        lut: Some(c),
                        ff: paired(p, c),
                        lut5,
                        ff5: lut5.and_then(|l| paired(p, l)),
                        reserved: false,
                    };
                    st.paired_ffs += usize::from(u.ff.is_some()) + usize::from(u.ff5.is_some());
                    st.fractured += usize::from(lut5.is_some());
                    units.push(u);
                }
                "SLICE_FFX" if !p.cluster_of.contains_key(&c) => lone_ffs.push(c),
                _ => {}
            }
        }
        let full = || format!("slice at {site}: more logic than letters");
        // 1. Mux leaves, at the letters the mux dictates.
        let (fixed, rest): (Vec<Letter>, Vec<Letter>) = units.into_iter().partition(|u| u.lut.is_some_and(|l| leaf.contains_key(&l)));
        // A slice has two F7 muxes: the first here takes letters A and B
        // (F7AMUX), the second C and D (F7BMUX). Each takes its select on
        // its lower letter's bypass pin (AX, CX), so no flop may be fed
        // through that.
        let mut by_mux: BTreeMap<CellId, Vec<(usize, Letter)>> = BTreeMap::new();
        for u in fixed {
            let (m, k) = leaf[&u.lut.expect("a LUT")];
            by_mux.entry(m).or_default().push((k, u));
        }
        if by_mux.len() > 2 {
            return Err(format!("slice at {site}: leaves of {} F7 muxes", by_mux.len()));
        }
        let mut select = [false; 4];
        let mut half_used = [false; 2];
        // F7 muxes under an F8 first: their half is given.
        let mut muxes: Vec<(CellId, Vec<(usize, Letter)>)> = by_mux.into_iter().collect();
        muxes.sort_by_key(|(m, _)| !under.contains_key(m));
        for (m, units) in muxes {
            let n = match under.get(&m) {
                Some(&(_, half)) => half,
                None => usize::from(half_used[0]),
            };
            if half_used[n] {
                return Err(format!("slice at {site}: two F7 muxes for one half"));
            }
            half_used[n] = true;
            select[2 * n] = true;
            // The F8 mux's select is BX.
            select[1] |= under.contains_key(&m);
            for k in 0..2 {
                letters[2 * n + k].reserved |= grounded[&m][k];
            }
            // The other F7 of its F8, if that one has no LUT leaves at all,
            // has nowhere to be but here: its half of the slice stays empty.
            if let Some(&(f8, half)) = under.get(&m)
                && let Some(sibling) = kids[&f8][1 - half]
                && wants(&sibling) == 0
                && !half_used[1 - half]
            {
                let other = 1 - half;
                half_used[other] = true;
                select[2 * other] = true;
                letters[2 * other].reserved = true;
                letters[2 * other + 1].reserved = true;
                mux_at.insert(sibling, (site.clone(), 0, if other == 0 { "F7AMUX" } else { "F7BMUX" }));
            }
            for (k, u) in units {
                if letters[2 * n + k].lut.is_some() {
                    return Err(format!("slice at {site}: two LUTs for letter {}", 2 * n + k));
                }
                letters[2 * n + k] = u;
                let e = mux_at.entry(m).or_insert_with(|| (site.clone(), 0, if n == 0 { "F7AMUX" } else { "F7BMUX" }));
                if e.0 == *site {
                    e.1 += 1;
                }
            }
        }
        // 2. Other LUTs, with their flops.
        for u in rest {
            let k = letters.iter().position(|l| !l.reserved && l.lut.is_none() && l.ff.is_none()).ok_or_else(full)?;
            letters[k] = u;
        }
        // 3. Flops with no LUT of their own.
        for ff in lone_ffs {
            st.bypass_ffs += 1;
            let open = |k: &usize| !select[*k];
            let empty = (0..4).filter(open).find(|&k| letters[k].lut.is_none() && letters[k].ff.is_none());
            let k = match empty {
                Some(k) => k,
                None => {
                    st.shared_letters += 1;
                    (0..4).filter(open).find(|&k| letters[k].ff.is_none()).ok_or_else(full)?
                }
            };
            letters[k].ff = Some(ff);
        }
        st.slices += 1;
        for (k, u) in letters.into_iter().enumerate() {
            let l = (b'A' + k as u8) as char;
            for (c, bel) in [(u.lut, "6LUT"), (u.lut5, "5LUT")] {
                if let Some(c) = c {
                    p.nl.cell_mut(c).bel = dev.grid_loc(site, &format!("{l}{bel}"));
                    st.luts += 1;
                }
            }
            for (c, bel) in [(u.ff, "FF"), (u.ff5, "5FF")] {
                if let Some(c) = c {
                    p.nl.cell_mut(c).bel = dev.grid_loc(site, &format!("{l}{bel}"));
                    st.ffs += 1;
                }
            }
        }
    }

    // A mux whose two leaves landed in one slice joins them there.
    for m in p.nl.cells_of("SELMUX2_1") {
        if let Some((site, found, bel)) = mux_at.get(&m)
            && *found == wants(&m)
        {
            p.nl.cell_mut(m).bel = dev.grid_loc(site, bel);
            st.f7_muxes += 1;
        }
    }
    // An F8 mux whose two F7 muxes landed in one slice joins them.
    let mut f8_at: BTreeMap<CellId, Vec<&str>> = BTreeMap::new();
    for (f7, (f8, _)) in &under {
        if let Some((site, found, _)) = mux_at.get(f7)
            && *found == wants(f7)
        {
            f8_at.entry(*f8).or_default().push(site.as_str());
        }
    }
    for (f8, sites) in f8_at {
        if let [a, b] = sites[..]
            && a == b
        {
            p.nl.cell_mut(f8).bel = dev.grid_loc(a, "F8MUX");
            st.f8_muxes += 1;
        }
    }
    for m in p.nl.cells_of("SELMUX2_1") {
        if p.nl.cell(m).bel.is_none() {
            st.unsupported.push(p.nl.cell(m).name.clone());
        }
    }
    // Any other slice-shaped macro is beyond this pass.
    for inst in &prob.insts {
        if inst.kind == InstKind::Macro {
            for &c in &inst.cells {
                let cell = p.nl.cell(c);
                if cell.bel.is_none() && slice_shaped(&cell.ty) && cell.ty != "SELMUX2_1" {
                    st.unsupported.push(cell.name.clone());
                }
            }
        }
    }

    let mut seen: HashMap<String, String> = HashMap::new();
    for c in p.nl.cell_ids() {
        let cell = p.nl.cell(c);
        match &cell.bel {
            None if !cell.ty.starts_with("PSEUDO_") => st.unplaced.push(cell.name.clone()),
            Some(l) => {
                if let Some(prev) = seen.insert(l.text(), cell.name.clone())
                    && st.collisions.len() < 20
                {
                    st.collisions.push(format!("{} and {} on {}", prev, cell.name, l.text()));
                }
            }
            None => {}
        }
    }
    Ok(st)
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use oxibit_pnr::packed::Loc;

    use super::*;
    use crate::device::Tile;
    use crate::netlist::{Dir, NetId, Netlist};
    use crate::pack::ctx::{PackCtx, ScoreParams};
    use crate::pack::slices::{PackerKind, SliceOpts, pack_slices};
    use crate::placer::{Inst, Site};

    /// One CLB tile with its two slices.
    fn device() -> Device {
        let sites: BTreeMap<String, String> = ["SLICE_X0Y0", "SLICE_X1Y0"].iter().map(|s| (s.to_string(), "SLICEL".to_string())).collect();
        let mut dev = Device::default();
        for s in sites.keys() {
            dev.site_tile.insert(s.clone(), "CLBLL_L_X2Y1".into());
        }
        dev.tiles.insert(
            "CLBLL_L_X2Y1".into(),
            Tile {
                name: "CLBLL_L_X2Y1".into(),
                ty: "CLBLL_L".into(),
                grid_x: 5,
                grid_y: 7,
                clock_region: None,
                sites,
            },
        );
        dev
    }

    fn lut(p: &mut Packer, name: &str, out: NetId) -> CellId {
        let c = p.nl.add_cell(name, "SLICE_LUTX");
        p.nl.add_port(c, "O6", Dir::Output);
        p.nl.connect(c, "O6", out);
        c
    }

    fn flop(p: &mut Packer, name: &str, d: NetId, clk: NetId) -> CellId {
        let c = p.nl.add_cell(name, "SLICE_FFX");
        p.nl.connect(c, "D", d);
        p.nl.connect(c, "CK", clk);
        c
    }

    /// The cells of a slice built around an F7 mux.
    struct MuxSlice {
        leaf1: CellId,
        leaf0: CellId,
        mux: CellId,
        lx: CellId,
        ly: CellId,
    }

    /// An F7 mux, its two leaf LUTs (no flops), and two more LUTs: four
    /// letters' worth, every letter with a LUT and a free FF.
    fn mux_slice() -> (Packer, MuxSlice) {
        let mut p = Packer::new(Netlist::new("t"));
        let nets: Vec<NetId> = ["i1", "i0", "x", "y", "o"].iter().map(|n| p.nl.add_net(n)).collect();
        let (leaf1, leaf0) = (lut(&mut p, "leaf1", nets[0]), lut(&mut p, "leaf0", nets[1]));
        let (lx, ly) = (lut(&mut p, "lx", nets[2]), lut(&mut p, "ly", nets[3]));
        let mux = p.nl.add_cell("mux", "SELMUX2_1");
        p.nl.connect(mux, "1", nets[0]);
        p.nl.connect(mux, "0", nets[1]);
        p.nl.add_port(mux, "OUT", Dir::Output);
        p.nl.connect(mux, "OUT", nets[4]);
        (p, MuxSlice { leaf1, leaf0, mux, lx, ly })
    }

    /// The site (relative to its tile) a cell's bel is on.
    fn site(p: &Packer, c: CellId) -> String {
        match &p.nl.cell(c).bel {
            Some(Loc::Grid { site, .. }) => site.clone(),
            other => panic!("{}: {other:?}", p.nl.cell(c).name),
        }
    }

    fn bel(p: &Packer, c: CellId) -> String {
        match &p.nl.cell(c).bel {
            Some(Loc::Grid { bel, .. }) => bel.clone(),
            other => panic!("{}: {other:?}", p.nl.cell(c).name),
        }
    }

    /// `cells` as one slice instance on the tile's first slice.
    fn one_slice(cells: Vec<CellId>) -> (Problem, Vec<Option<u32>>) {
        let inst = Inst {
            cells,
            kind: InstKind::Slice,
        };
        let site = Site {
            x: 5.0,
            y: 7.0,
            name: "SLICE_X0Y0".into(),
        };
        (Problem::from_parts(vec![inst], Vec::new(), vec![site]), vec![Some(0)])
    }

    #[test]
    fn a_bypass_flop_stays_off_letter_a_beside_an_f7_mux() {
        // Found by routing: AX is the F7A mux's select, so a flop fed
        // through AX in the mux's slice put two nets on one pin.
        let (mut p, MuxSlice { leaf1, leaf0, mux, lx, ly }) = mux_slice();
        let (d, clk) = (p.nl.add_net("d"), p.nl.add_net("clk"));
        let ff = flop(&mut p, "ff", d, clk);
        // Flop first, leaves last: the order of the instance's cells must
        // not decide anything.
        let (prob, site_of) = one_slice(vec![ff, lx, ly, leaf0, leaf1]);
        let st = assign(&mut p, &device(), &prob, &site_of).unwrap();
        assert!(st.ok(), "{st:?}");
        assert_eq!([bel(&p, leaf1), bel(&p, leaf0), bel(&p, mux)], ["A6LUT", "B6LUT", "F7AMUX"]);
        // All four letters hold a LUT, so the flop shares one. Letter A's
        // FF is free, and it must still go to B.
        assert_eq!(bel(&p, ff), "BFF");
        assert_eq!((st.bypass_ffs, st.shared_letters, st.f7_muxes), (1, 1, 1));
    }

    #[test]
    fn packing_seats_no_more_bypass_flops_beside_an_f7_mux_than_it_has_ffs_for() {
        // Found by routing the order-of-appearance arm: the packer filled a
        // mux slice to four flops, and bel assignment had three FFs to
        // give (letter A's is behind the mux select).
        for kind in [PackerKind::Rules, PackerKind::RulesScore] {
            let (mut p, MuxSlice { leaf1, mux, .. }) = mux_slice();
            let clk = p.nl.add_net("clk");
            let ffs: Vec<CellId> = (0..4)
                .map(|i| {
                    let d = p.nl.add_net(&format!("d{i}"));
                    flop(&mut p, &format!("ff{i}"), d, clk)
                })
                .collect();
            {
                let ctx = PackCtx::new(&p, ScoreParams::default());
                assert_eq!(ctx.atoms[ctx.atom_of[&leaf1] as usize].blocks_ff, 1);
            }
            let report = pack_slices(
                &mut p,
                &SliceOpts {
                    kind,
                    max_slices: Some(2),
                    ..SliceOpts::default()
                },
            );
            assert!(report.illegal.is_empty(), "{kind:?}: {:?}", report.illegal);
            // Four LUTs and four flops would fit one slice by count alone.
            assert_eq!(p.slices.len(), 2, "{kind:?}");

            // Each slice on a site of the tile; bel assignment must seat it.
            let insts: Vec<Inst> = p
                .slices
                .iter()
                .map(|g| Inst {
                    cells: g.cells.clone(),
                    kind: InstKind::Slice,
                })
                .collect();
            let sites = ["SLICE_X0Y0", "SLICE_X1Y0"]
                .iter()
                .map(|s| Site {
                    x: 5.0,
                    y: 7.0,
                    name: s.to_string(),
                })
                .collect();
            let prob = Problem::from_parts(insts, Vec::new(), sites);
            let st = assign(&mut p, &device(), &prob, &[Some(0), Some(1)]).unwrap();
            assert!(st.ok(), "{kind:?}: {st:?}");
            assert_eq!(bel(&p, mux), "F7AMUX");
            let beside: Vec<&CellId> = ffs.iter().filter(|&&f| site(&p, f) == site(&p, mux)).collect();
            assert!(beside.len() <= 3, "{kind:?}: {} flops beside the mux", beside.len());
            assert!(beside.iter().all(|&&f| bel(&p, f) != "AFF"), "{kind:?}");
            if kind == PackerKind::Rules {
                // Order of appearance fills the mux slice as far as it goes.
                assert_eq!(beside.len(), 3);
            }
        }
    }

    #[test]
    fn without_a_mux_a_bypass_flop_may_take_letter_a() {
        let mut p = Packer::new(Netlist::new("t"));
        let nets: Vec<NetId> = ["a", "b", "c", "e", "d", "clk"].iter().map(|n| p.nl.add_net(n)).collect();
        let luts: Vec<CellId> = (0..4).map(|i| lut(&mut p, &format!("l{i}"), nets[i])).collect();
        let ff = flop(&mut p, "ff", nets[4], nets[5]);
        let (prob, site_of) = one_slice(luts.iter().copied().chain([ff]).collect());
        let st = assign(&mut p, &device(), &prob, &site_of).unwrap();
        assert!(st.ok(), "{st:?}");
        assert_eq!(bel(&p, ff), "AFF");
        assert_eq!(bel(&p, luts[0]), "A6LUT");
    }
}
