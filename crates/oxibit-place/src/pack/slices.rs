//! Slice packing: atoms (`ctx::Atom`) into slice-sized groups.
//!
//! Two packers behind one legality check (`fits`): `rules` fills slices in
//! order of appearance; `rules+score` is best-choice clustering
//! (`bestchoice`), which picks among the same legal merges by
//! `merging_score`. Either way the result is `Packer::slices`, for the
//! placer to move as units. Cell ports and types are not touched.

use std::collections::{BTreeMap, BTreeSet};
use std::str::FromStr;

use serde_json::{Value, json};

use super::ctx::{AtomId, PackCtx, ScoreParams, SliceCap};
use super::ffs::Ctrl;
use super::{Packer, bestchoice};
use crate::netlist::{CellId, Dir, NetId};

/// The cells sharing one slice.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SliceGroup {
    pub cells: Vec<CellId>,
    pub luts: u8,
    pub ffs: u8,
}

/// A slice being filled.
#[derive(Debug, Clone)]
pub struct Group {
    pub atoms: Vec<AtomId>,
    pub luts: u8,
    pub ffs: u8,
    /// FF slots nothing in it can use (`Atom::blocks_ff`).
    pub blocked: u8,
    pub ctrl: Option<Ctrl>,
    /// Scored nets it touches (indices into `PackCtx::nets`), ascending.
    pub nets: Vec<u32>,
    /// Sum of its atoms' positions.
    sum: Option<(f64, f64)>,
}

impl Group {
    pub fn of(a: AtomId, ctx: &PackCtx) -> Group {
        let atom = &ctx.atoms[a as usize];
        Group {
            atoms: vec![a],
            luts: atom.luts,
            ffs: atom.ffs,
            blocked: atom.blocks_ff,
            ctrl: atom.ctrl.clone(),
            nets: ctx.atom_nets[a as usize].clone(),
            sum: ctx.pos.as_ref().map(|p| p[a as usize]),
        }
    }

    /// Letters it occupies.
    pub fn area(&self) -> u32 {
        u32::from(self.luts.max(self.ffs)).max(1)
    }

    /// Centroid of its atoms.
    pub fn pos(&self) -> Option<(f64, f64)> {
        let n = self.atoms.len() as f64;
        self.sum.map(|(x, y)| (x / n, y / n))
    }

    pub fn full(&self, cap: SliceCap) -> bool {
        self.luts >= cap.luts && self.ffs + self.blocked >= cap.ffs
    }

    /// Take `o` in. The caller checked `fits`.
    pub fn absorb(&mut self, o: Group) {
        self.atoms.extend(o.atoms);
        self.luts += o.luts;
        self.ffs += o.ffs;
        self.blocked += o.blocked;
        if self.ctrl.is_none() {
            self.ctrl = o.ctrl;
        }
        let mut nets = Vec::with_capacity(self.nets.len() + o.nets.len());
        let (mut i, mut j) = (0, 0);
        while i < self.nets.len() || j < o.nets.len() {
            let a = self.nets.get(i).copied().unwrap_or(u32::MAX);
            let b = o.nets.get(j).copied().unwrap_or(u32::MAX);
            nets.push(a.min(b));
            i += usize::from(a <= b);
            j += usize::from(b <= a);
        }
        self.nets = nets;
        self.sum = match (self.sum, o.sum) {
            (Some(a), Some(b)) => Some((a.0 + b.0, a.1 + b.1)),
            _ => None,
        };
    }
}

/// The legality of one slice holding both: the capacity table (FF slots an
/// F7 mux puts out of reach count as taken), and one control set for all
/// its flops.
pub fn fits(a: &Group, b: &Group, cap: SliceCap) -> bool {
    if a.luts + b.luts > cap.luts || a.ffs + b.ffs + a.blocked + b.blocked > cap.ffs {
        return false;
    }
    match (&a.ctrl, &b.ctrl) {
        (Some(x), Some(y)) => x == y,
        _ => true,
    }
}

/// How far back first-fit looks among the slices still open.
const WINDOW: usize = 64;

/// Fill in the order given: each group joins the oldest open slice (of the
/// last `WINDOW`) it fits, else opens one. Merging stops once no more than
/// `max` groups remain.
pub fn first_fit(groups: Vec<Group>, cap: SliceCap, max: usize) -> Vec<Group> {
    let mut alive = groups.len();
    let mut out: Vec<Group> = Vec::with_capacity(groups.len());
    let mut open: Vec<usize> = Vec::new();
    for g in groups {
        if alive > max {
            let lo = open.len().saturating_sub(WINDOW);
            if let Some(k) = (lo..open.len()).find(|&k| fits(&out[open[k]], &g, cap)) {
                let h = open[k];
                out[h].absorb(g);
                alive -= 1;
                if out[h].full(cap) {
                    open.remove(k);
                }
                continue;
            }
        }
        if !g.full(cap) {
            open.push(out.len());
        }
        out.push(g);
    }
    out
}

/// Groups in a stable order (by their first atom), atoms ascending.
pub fn normalise(mut groups: Vec<Group>) -> Vec<Group> {
    for g in &mut groups {
        g.atoms.sort_unstable();
    }
    groups.sort_by_key(|g| g.atoms[0]);
    groups
}

/// The starting groups: one per atom, except that atoms tied to one slice
/// (`PackCtx::ties`) start merged. A group's first atom is its lowest.
pub fn seeds(ctx: &PackCtx) -> Vec<Group> {
    let mut groups: Vec<Option<Group>> = (0..ctx.atoms.len() as AtomId).map(|a| Some(Group::of(a, ctx))).collect();
    let mut at: Vec<usize> = (0..ctx.atoms.len()).collect();
    for &(a, b) in &ctx.ties {
        let (lo, hi) = (at[a as usize].min(at[b as usize]), at[a as usize].max(at[b as usize]));
        if lo == hi {
            continue;
        }
        let moved = groups[hi].take().expect("live");
        if fits(groups[lo].as_ref().expect("live"), &moved, ctx.cap) {
            for &m in &moved.atoms {
                at[m as usize] = lo;
            }
            groups[lo].as_mut().expect("live").absorb(moved);
        } else {
            // Two control sets: no slice holds both. Left apart for
            // bel assignment to report.
            groups[hi] = Some(moved);
        }
    }
    groups.into_iter().flatten().collect()
}

/// Order-of-appearance packing.
pub fn pack_in_order(ctx: &PackCtx) -> Vec<Group> {
    normalise(first_fit(seeds(ctx), ctx.cap, 0))
}

/// Merge the smallest groups, connected or not, until at most `max` remain
/// (or nothing more fits): density when the device needs it.
pub fn compact(mut groups: Vec<Group>, cap: SliceCap, max: usize) -> Vec<Group> {
    if groups.len() <= max {
        return groups;
    }
    for g in &mut groups {
        g.atoms.sort_unstable();
    }
    groups.sort_by_key(|g| (g.area(), g.atoms[0]));
    normalise(first_fit(groups, cap, max))
}

/// Break up the smallest slices while more than `max` remain: each atom of
/// the smallest goes to the slice it fits that shares the most net weight
/// with it. A slice whose atoms do not all find a home is put back whole
/// and not tried again. This is what gets best-choice down to the density
/// of plain filling, since merging whole groups strands 3 + 2 where 4 + 1
/// fits.
pub fn dissolve(mut groups: Vec<Group>, ctx: &PackCtx, max: usize) -> Vec<Group> {
    // Slices that would not dissolve, by first atom (stable under absorb).
    let mut stuck: BTreeSet<AtomId> = BTreeSet::new();
    // Tied atoms stay together, so their slices are not broken up.
    let tied: BTreeSet<AtomId> = ctx.ties.iter().flat_map(|&(a, b)| [a, b]).collect();
    while groups.len() > max {
        let smallest = (0..groups.len())
            .filter(|&i| !stuck.contains(&groups[i].atoms[0]) && !groups[i].atoms.iter().any(|a| tied.contains(a)))
            .min_by_key(|&i| (groups[i].area(), groups[i].atoms.len(), std::cmp::Reverse(groups[i].atoms[0])));
        let Some(i) = smallest else { break };
        let victim = groups.swap_remove(i);
        // Hosts as they were, for putting things back.
        let mut undo: Vec<(usize, Group)> = Vec::new();
        let mut homed = true;
        for &a in &victim.atoms {
            let single = Group::of(a, ctx);
            let host = (0..groups.len())
                .filter(|&h| fits(&groups[h], &single, ctx.cap))
                .map(|h| (ctx.shared_weight(&groups[h].nets, &single.nets), h))
                .max_by(|x, y| x.0.total_cmp(&y.0).then(y.1.cmp(&x.1)));
            let Some((_, h)) = host else {
                homed = false;
                break;
            };
            undo.push((h, groups[h].clone()));
            groups[h].absorb(single);
        }
        if !homed {
            for (h, g) in undo.into_iter().rev() {
                groups[h] = g;
            }
            stuck.insert(victim.atoms[0]);
            groups.push(victim);
        }
    }
    normalise(groups)
}

/// Quality of a packing, placement-free: what the slices cut.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SliceStats {
    pub slices: usize,
    pub atoms: usize,
    /// 6LUT and FF slots taken (not cells: a fractured letter's second
    /// LUT and its flop ride along).
    pub luts: usize,
    pub ffs: usize,
    /// Letters holding two LUTs.
    pub fractured: usize,
    /// Scored nets entirely inside one slice.
    pub absorbed: usize,
    /// Sum over scored nets of (slices touched - 1).
    pub cut: usize,
    /// The same, each net weighted `1 / pins`.
    pub cut_weight: f64,
    /// Scored nets (connecting two atoms or more, below the fanout cap).
    pub nets: usize,
    /// Distinct nets a slice reads from outside itself (clocks and
    /// constants aside): the most, the mean, and slices by count (up to 8,
    /// 9 to 12, 13 to 16, 17 to 20, more).
    pub ext_in_max: usize,
    pub ext_in_mean: f64,
    pub ext_in_hist: [usize; 5],
    /// Distinct control sets among the flops, as written and with
    /// equivalent ones made one (`Ctrl::normalised`).
    pub ctrl_sets: usize,
    pub ctrl_sets_merged: usize,
}

impl SliceStats {
    pub fn to_json(&self) -> Value {
        json!({"slices": self.slices, "atoms": self.atoms, "luts": self.luts, "ffs": self.ffs, "fractured": self.fractured, "nets": self.nets,
               "absorbed": self.absorbed, "cut": self.cut, "cut_weight": self.cut_weight,
               "ext_in": {"max": self.ext_in_max, "mean": self.ext_in_mean, "hist_8_12_16_20_more": self.ext_in_hist},
               "ctrl_sets": self.ctrl_sets, "ctrl_sets_merged": self.ctrl_sets_merged})
    }
}

/// The distinct nets `g`'s cells read from outside it, clocks and
/// constants aside: what the slice asks of the interconnect around it.
pub fn external_inputs(g: &Group, ctx: &PackCtx) -> usize {
    let nl = ctx.nl;
    let cells: BTreeSet<CellId> = g.atoms.iter().flat_map(|&a| ctx.atoms[a as usize].cells.iter().copied()).collect();
    let mut nets: BTreeSet<NetId> = BTreeSet::new();
    for &c in &cells {
        for (name, port) in &nl.cell(c).ports {
            if port.dir != Dir::Input || name == "CK" || name == "CLK" {
                continue;
            }
            if let Some(n) = port.net
                && !nl.is_const(n)
                && nl.driver(n).is_none_or(|(d, _)| !cells.contains(&d))
            {
                nets.insert(n);
            }
        }
    }
    nets.len()
}

pub fn stats(groups: &[Group], ctx: &PackCtx) -> SliceStats {
    let mut of = vec![u32::MAX; ctx.atoms.len()];
    let mut st = SliceStats {
        slices: groups.len(),
        nets: ctx.nets.len(),
        ..SliceStats::default()
    };
    for (i, g) in groups.iter().enumerate() {
        st.atoms += g.atoms.len();
        st.luts += usize::from(g.luts);
        st.ffs += usize::from(g.ffs);
        st.fractured += g.atoms.iter().filter(|&&a| ctx.atoms[a as usize].five.is_some()).count();
        for &a in &g.atoms {
            of[a as usize] = i as u32;
        }
    }
    for g in groups {
        let n = external_inputs(g, ctx);
        st.ext_in_max = st.ext_in_max.max(n);
        st.ext_in_mean += n as f64 / groups.len() as f64;
        st.ext_in_hist[match n {
            0..=8 => 0,
            9..=12 => 1,
            13..=16 => 2,
            17..=20 => 3,
            _ => 4,
        }] += 1;
    }
    let sets: BTreeSet<String> = ctx.atoms.iter().filter_map(|a| a.ctrl.as_ref()).map(|c| format!("{c:?}")).collect();
    let merged: BTreeSet<String> =
        ctx.atoms.iter().filter_map(|a| a.ctrl.as_ref()).map(|c| format!("{:?}", c.normalised(ctx.nl.gnd, ctx.nl.vcc))).collect();
    (st.ctrl_sets, st.ctrl_sets_merged) = (sets.len(), merged.len());
    for net in &ctx.nets {
        let touched: BTreeSet<u32> = net.atoms.iter().map(|&a| of[a as usize]).collect();
        let k = touched.len() - 1;
        st.cut += k;
        st.cut_weight += k as f64 * net.weight;
        if k == 0 && !net.open {
            st.absorbed += 1;
        }
    }
    st
}

/// Every way `groups` breaks the rules, recomputed from the atoms: each
/// atom in exactly one slice, capacity, one control set per slice.
pub fn verify(groups: &[Group], ctx: &PackCtx) -> Vec<String> {
    let mut bad = Vec::new();
    let mut seen = vec![0u32; ctx.atoms.len()];
    for (i, g) in groups.iter().enumerate() {
        let (mut luts, mut ffs) = (0u32, 0u32);
        let mut set: Option<&Ctrl> = None;
        for &a in &g.atoms {
            let atom = &ctx.atoms[a as usize];
            seen[a as usize] += 1;
            luts += u32::from(atom.luts);
            ffs += u32::from(atom.ffs) + u32::from(atom.blocks_ff);
            match (set, &atom.ctrl) {
                (Some(s), Some(c)) if s != c => bad.push(format!("slice {i}: control sets differ")),
                (None, Some(c)) => set = Some(c),
                _ => {}
            }
        }
        if luts > u32::from(ctx.cap.luts) || ffs > u32::from(ctx.cap.ffs) {
            bad.push(format!("slice {i}: {luts} LUTs, {ffs} flops"));
        }
    }
    for (a, n) in seen.iter().enumerate() {
        if *n != 1 {
            bad.push(format!("atom {a} in {n} slices"));
        }
    }
    bad
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PackerKind {
    /// Rules only: slices filled in order of appearance.
    #[default]
    Rules,
    /// Rules for legality, `merging_score` for choice.
    RulesScore,
}

impl FromStr for PackerKind {
    type Err = String;

    fn from_str(s: &str) -> Result<PackerKind, String> {
        match s {
            "rules" => Ok(PackerKind::Rules),
            "rules+score" => Ok(PackerKind::RulesScore),
            _ => Err(format!("packer {s:?}: rules or rules+score")),
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct SliceOpts {
    pub kind: PackerKind,
    pub params: ScoreParams,
    /// Compact to at most this many slices (`rules+score` only; the rules
    /// packer is already as dense as it gets).
    pub max_slices: Option<usize>,
    /// A position per atom, in `PackCtx::atoms` order.
    pub positions: Option<Vec<(f64, f64)>>,
}

#[derive(Debug, Clone, Default)]
pub struct SliceReport {
    pub stats: SliceStats,
    /// Merges the score chose (0 for `rules`).
    pub scored_merges: usize,
    /// Rule violations; empty unless there is a bug.
    pub illegal: Vec<String>,
}

impl SliceReport {
    pub fn to_json(&self) -> Value {
        let mut v = self.stats.to_json();
        v["scored_merges"] = json!(self.scored_merges);
        v["illegal"] = json!(self.illegal.len());
        v
    }
}

/// Pack `p`'s atoms into slices, leaving them in `p.slices`.
pub fn pack_slices(p: &mut Packer, opts: &SliceOpts) -> SliceReport {
    let (slices, fractured, report) = {
        let mut ctx = PackCtx::new(p, opts.params);
        if let Some(pos) = &opts.positions {
            ctx = ctx.with_positions(pos.clone());
        }
        let (groups, scored_merges) = match opts.kind {
            PackerKind::Rules => (pack_in_order(&ctx), 0),
            PackerKind::RulesScore => {
                let (groups, merges) = bestchoice::best_choice(&ctx);
                let max = opts.max_slices.unwrap_or(usize::MAX);
                (dissolve(compact(groups, ctx.cap, max), &ctx, max), merges)
            }
        };
        let report = SliceReport {
            stats: stats(&groups, &ctx),
            scored_merges,
            illegal: verify(&groups, &ctx),
        };
        let slices: Vec<SliceGroup> = groups
            .iter()
            .map(|g| SliceGroup {
                cells: g.atoms.iter().flat_map(|&a| ctx.atoms[a as usize].cells.iter().copied()).collect(),
                luts: g.luts,
                ffs: g.ffs,
            })
            .collect();
        let fractured: BTreeMap<CellId, CellId> = groups
            .iter()
            .flat_map(|g| &g.atoms)
            .filter_map(|&a| ctx.atoms[a as usize].five.map(|five| (ctx.atoms[a as usize].cells[0], five)))
            .collect();
        (slices, fractured, report)
    };
    p.fractured = fractured;
    p.stats.insert("slices".into(), slices.len());
    p.slices = slices;
    report
}
