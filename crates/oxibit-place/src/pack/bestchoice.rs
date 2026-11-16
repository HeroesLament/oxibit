//! Best-choice slice packing: the globally best legal merge first.
//!
//! The loop is best-choice clustering with lazy updates (Alpert et al.,
//! ISPD 2005; Nam et al., TCAD 2006). Every group's best legal partner goes
//! into a priority queue keyed by score. The top pair merges; an entry
//! whose groups changed since it was scored is rescored and requeued
//! rather than trusted. It ends when no pair has positive score, so groups
//! with nothing in common stay apart (`slices::compact` packs those when
//! density calls for it).
//!
//! Candidates are logical neighbors (groups sharing a scored net) and, when
//! positions are known, physical ones (groups within the neighbor-distance
//! threshold, found through a bin grid). `slices::fits` filters both before
//! the score sees them.
//!
//! Phases: the initial scoring of every group is read-only over `&State`
//! and runs on scoped threads; pops and merges are serial.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

use super::ctx::{PackCtx, manhattan, score_of};
use super::slices::{Group, fits, normalise, seeds};
use crate::par::par_map;

struct State<'c, 'a> {
    ctx: &'c PackCtx<'a>,
    /// A group is indexed by its lowest starting atom; a merged-away
    /// group is `None`.
    groups: Vec<Option<Group>>,
    /// Bumped whenever a group changes.
    ver: Vec<u32>,
    /// Atom -> the group holding it.
    rep: Vec<u32>,
    /// Bin -> groups whose centroid was there when entered. Entries go
    /// stale (the group moved or died) and are checked on read.
    grid: Option<HashMap<(i64, i64), Vec<u32>>>,
}

fn bin(pos: (f64, f64), size: f64) -> (i64, i64) {
    ((pos.0 / size).floor() as i64, (pos.1 / size).floor() as i64)
}

impl State<'_, '_> {
    fn bin_of(&self, g: u32) -> Option<(i64, i64)> {
        let pos = self.groups[g as usize].as_ref()?.pos()?;
        Some(bin(pos, self.ctx.params.neighbor_dist))
    }

    /// The best legal partner of `u` and the score, if any scores above zero.
    fn best(&self, u: u32) -> Option<(f64, u32)> {
        let ctx = self.ctx;
        let g = self.groups[u as usize].as_ref()?;
        if g.full(ctx.cap) {
            return None;
        }
        // Shared net weight per neighboring group, each net once.
        let mut shared: HashMap<u32, f64> = HashMap::new();
        let mut reps: Vec<u32> = Vec::new();
        for &e in &g.nets {
            let net = &ctx.nets[e as usize];
            reps.clear();
            reps.extend(net.atoms.iter().map(|&a| self.rep[a as usize]).filter(|&r| r != u));
            reps.sort_unstable();
            reps.dedup();
            for &r in &reps {
                *shared.entry(r).or_insert(0.0) += net.weight;
            }
        }
        if let (Some(grid), Some((bx, by))) = (&self.grid, self.bin_of(u)) {
            for key in (-1..=1).flat_map(|dx| (-1..=1).map(move |dy| (bx + dx, by + dy))) {
                for &r in grid.get(&key).into_iter().flatten() {
                    if r != u && self.bin_of(r) == Some(key) {
                        shared.entry(r).or_insert(0.0);
                    }
                }
            }
        }
        let mut best: Option<(f64, u32)> = None;
        for (&v, &w) in &shared {
            let h = self.groups[v as usize].as_ref().expect("a rep is live");
            if !fits(g, h, ctx.cap) {
                continue;
            }
            let dist = g.pos().zip(h.pos()).map(|(a, b)| manhattan(a, b));
            let s = score_of(w, g.area() + h.area(), dist, &ctx.params);
            if s > 0.0 && best.is_none_or(|(bs, bv)| s > bs || (s == bs && v < bv)) {
                best = Some((s, v));
            }
        }
        best
    }

    /// `v` into `u`.
    fn merge(&mut self, u: u32, v: u32) {
        let gone = self.groups[v as usize].take().expect("live");
        for &a in &gone.atoms {
            self.rep[a as usize] = u;
        }
        self.groups[u as usize].as_mut().expect("live").absorb(gone);
        self.ver[u as usize] += 1;
        self.ver[v as usize] += 1;
        if let Some(key) = self.bin_of(u)
            && let Some(grid) = &mut self.grid
        {
            let cell = grid.entry(key).or_default();
            if !cell.contains(&u) {
                cell.push(u);
            }
        }
    }
}

/// A scored pair, valid while both versions hold.
struct Entry {
    score: f64,
    u: u32,
    v: u32,
    ver_u: u32,
    ver_v: u32,
}

impl Ord for Entry {
    fn cmp(&self, o: &Entry) -> Ordering {
        // Highest score first; ties to the lowest ids, for determinism.
        self.score.total_cmp(&o.score).then_with(|| (o.u, o.v).cmp(&(self.u, self.v)))
    }
}

impl PartialOrd for Entry {
    fn partial_cmp(&self, o: &Entry) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}

impl PartialEq for Entry {
    fn eq(&self, o: &Entry) -> bool {
        self.cmp(o) == Ordering::Equal
    }
}

impl Eq for Entry {}

/// Best-choice packing of `ctx`'s atoms: the groups, and how many merges
/// the score chose.
pub fn best_choice(ctx: &PackCtx) -> (Vec<Group>, usize) {
    let n = ctx.atoms.len();
    let mut st = State {
        ctx,
        groups: (0..n).map(|_| None).collect(),
        ver: vec![0; n],
        rep: (0..n as u32).collect(),
        grid: None,
    };
    for g in seeds(ctx) {
        let id = g.atoms[0];
        for &a in &g.atoms {
            st.rep[a as usize] = id;
        }
        st.groups[id as usize] = Some(g);
    }
    if let Some(pos) = &ctx.pos {
        let mut grid: HashMap<(i64, i64), Vec<u32>> = HashMap::new();
        for (a, &p) in pos.iter().enumerate() {
            grid.entry(bin(p, ctx.params.neighbor_dist)).or_default().push(a as u32);
        }
        st.grid = Some(grid);
    }

    let entry = |st: &State, u: u32, (score, v): (f64, u32)| Entry {
        score,
        u,
        v,
        ver_u: st.ver[u as usize],
        ver_v: st.ver[v as usize],
    };
    let first = par_map(n, |u| st.best(u));
    let mut heap: BinaryHeap<Entry> = first
        .into_iter()
        .enumerate()
        .filter_map(|(u, b)| b.map(|b| entry(&st, u as u32, b)))
        .collect();

    let mut merges = 0;
    while let Some(e) = heap.pop() {
        if st.groups[e.u as usize].is_none() {
            continue;
        }
        let fresh = st.ver[e.u as usize] == e.ver_u && st.ver[e.v as usize] == e.ver_v && st.groups[e.v as usize].is_some();
        if fresh {
            st.merge(e.u, e.v);
            merges += 1;
        }
        // Either way `u` needs its best partner as things stand now.
        if let Some(b) = st.best(e.u) {
            heap.push(entry(&st, e.u, b));
        }
    }
    (normalise(st.groups.into_iter().flatten().collect()), merges)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::netlist::{CellId, Dir, NetId, Netlist};
    use crate::pack::ctx::{ScoreParams, merging_score};
    use crate::pack::slices::{PackerKind, SliceOpts, pack_in_order, pack_slices, stats, verify};
    use crate::pack::{Offset, Packer, slot};

    fn lut(p: &mut Packer, name: &str, ins: &[NetId], out: NetId) -> CellId {
        let c = p.nl.add_cell(name, "SLICE_LUTX");
        for (i, &n) in ins.iter().enumerate() {
            p.nl.connect(c, &format!("A{}", i + 1), n);
        }
        p.nl.add_port(c, "O6", Dir::Output);
        p.nl.connect(c, "O6", out);
        c
    }

    fn ff(p: &mut Packer, name: &str, d: NetId, clk: NetId, ce: NetId) -> CellId {
        let c = p.nl.add_cell(name, "SLICE_FFX");
        p.nl.connect(c, "D", d);
        p.nl.connect(c, "CK", clk);
        p.nl.connect(c, "CE", ce);
        c
    }

    /// Two chains of four LUTs, `a0 -> a1 -> a2 -> a3` and `b0 .. b3`,
    /// created interleaved.
    fn two_chains() -> Packer {
        let mut p = Packer::new(Netlist::new("t"));
        for s in ["a", "b"] {
            for i in 0..5 {
                p.nl.add_net(&format!("{s}{i}"));
            }
        }
        for i in 0..4 {
            for s in ["a", "b"] {
                let (i_net, o_net) = (p.nl.net_id(&format!("{s}{i}")).unwrap(), p.nl.net_id(&format!("{s}{}", i + 1)).unwrap());
                lut(&mut p, &format!("{s}_lut{i}"), &[i_net], o_net);
            }
        }
        p
    }

    #[test]
    fn score_follows_shared_nets_and_distance() {
        let mut p = Packer::new(Netlist::new("t"));
        let nets: Vec<NetId> = ["x", "y", "z", "q0", "q1", "q2"].iter().map(|n| p.nl.add_net(n)).collect();
        // l0 drives l1 alone (two pins); l0 and l2 only share the wider `x`.
        let l0 = lut(&mut p, "l0", &[nets[0]], nets[1]);
        let l1 = lut(&mut p, "l1", &[nets[1]], nets[3]);
        let l2 = lut(&mut p, "l2", &[nets[0]], nets[4]);
        let l3 = lut(&mut p, "l3", &[nets[0]], nets[5]);
        let ctx = PackCtx::new(&p, ScoreParams::default());
        let [a0, a1, a2, a3] = [l0, l1, l2, l3].map(|c| ctx.atom_of[&c]);
        let (s01, s02) = (merging_score(a0, a1, &ctx), merging_score(a0, a2, &ctx));
        // A two-pin net (1/2), then a three-pin one (1/3), over two letters.
        assert!((s01 - 0.5 / 2.0).abs() < 1e-12, "{s01}");
        assert!((s02 - (1.0 / 3.0) / 2.0).abs() < 1e-12, "{s02}");
        assert_eq!(s01, merging_score(a1, a0, &ctx));
        assert_eq!(merging_score(a1, a3, &ctx), 0.0);

        let ctx = ctx.with_positions(vec![(0.0, 0.0), (1.0, 1.0), (10.0, 0.0), (0.0, 0.0)]);
        assert!(merging_score(a0, a1, &ctx) > s01);
        assert_eq!(merging_score(a0, a2, &ctx), f64::NEG_INFINITY);
    }

    #[test]
    fn best_choice_keeps_chains_together() {
        let p = two_chains();
        let ctx = PackCtx::new(&p, ScoreParams::default());
        assert_eq!(ctx.atoms.len(), 8);
        assert_eq!(ctx.nets.len(), 6);

        let ordered = pack_in_order(&ctx);
        let (scored, merges) = best_choice(&ctx);
        assert!(verify(&ordered, &ctx).is_empty());
        assert!(verify(&scored, &ctx).is_empty());
        let (so, ss) = (stats(&ordered, &ctx), stats(&scored, &ctx));
        assert_eq!((so.slices, ss.slices), (2, 2));
        assert_eq!(so.cut, 2);
        assert_eq!((ss.cut, ss.absorbed, merges), (0, 6, 6));

        // Same answer every run.
        let again = best_choice(&ctx).0;
        assert_eq!(scored.iter().map(|g| &g.atoms).collect::<Vec<_>>(), again.iter().map(|g| &g.atoms).collect::<Vec<_>>());
    }

    #[test]
    fn control_sets_never_mix() {
        let mut p = Packer::new(Netlist::new("t"));
        let clk = p.nl.add_net("clk");
        let (ce0, ce1) = (p.nl.add_net("ce0"), p.nl.add_net("ce1"));
        let d = p.nl.add_net("d");
        // Six flops on one data net, alternating clock enables.
        for i in 0..6 {
            ff(&mut p, &format!("f{i}"), d, clk, if i % 2 == 0 { ce0 } else { ce1 });
        }
        for kind in [PackerKind::Rules, PackerKind::RulesScore] {
            let r = pack_slices(
                &mut p,
                &SliceOpts {
                    kind,
                    ..SliceOpts::default()
                },
            );
            assert!(r.illegal.is_empty(), "{:?}", r.illegal);
            assert_eq!(r.stats.slices, 2, "{kind:?}");
            for s in &p.slices {
                let ces: std::collections::BTreeSet<_> = s.cells.iter().map(|&c| p.nl.cell(c).net("CE")).collect();
                assert_eq!(ces.len(), 1);
            }
        }
    }

    #[test]
    fn lut_flop_pairs_are_one_atom_and_capacity_holds() {
        let mut p = Packer::new(Netlist::new("t"));
        let clk = p.nl.add_net("clk");
        let ce = p.nl.add_net("ce");
        let x = p.nl.add_net("x");
        // Five LUT+flop pairs all reading `x`: more than one slice holds.
        for i in 0..5 {
            let o = p.nl.add_net(&format!("o{i}"));
            let l = lut(&mut p, &format!("l{i}"), &[x], o);
            let f = ff(&mut p, &format!("f{i}"), o, clk, ce);
            p.add_member(
                l,
                f,
                Offset {
                    dx: 0,
                    dy: 0,
                    z: slot::FF - slot::LUT6,
                    abs: false,
                },
            );
        }
        let r = pack_slices(
            &mut p,
            &SliceOpts {
                kind: PackerKind::RulesScore,
                ..SliceOpts::default()
            },
        );
        assert!(r.illegal.is_empty());
        assert_eq!((r.stats.atoms, r.stats.luts, r.stats.ffs), (5, 5, 5));
        // Two slices, a pair never split, none over four letters. Which
        // split (4+1 or 3+2) is the merge order's business.
        assert_eq!(p.slices.len(), 2);
        assert_eq!(p.slices.iter().map(|s| s.cells.len()).sum::<usize>(), 10);
        for s in &p.slices {
            assert!(s.cells.len() % 2 == 0 && s.luts <= 4 && s.ffs <= 4, "{s:?}");
            for pair in s.cells.chunks(2) {
                assert_eq!(p.cluster_of[&pair[1]], pair[0]);
            }
        }
    }

    #[test]
    fn physical_neighbors_merge_without_shared_nets() {
        let mut p = Packer::new(Netlist::new("t"));
        for i in 0..4 {
            let (a, b) = (p.nl.add_net(&format!("i{i}")), p.nl.add_net(&format!("o{i}")));
            lut(&mut p, &format!("l{i}"), &[a], b);
        }
        let mut opts = SliceOpts {
            kind: PackerKind::RulesScore,
            ..SliceOpts::default()
        };
        assert_eq!(pack_slices(&mut p, &opts).stats.slices, 4);
        opts.positions = Some(vec![(0.0, 0.0), (50.0, 50.0), (1.0, 0.0), (51.0, 50.0)]);
        assert_eq!(pack_slices(&mut p, &opts).stats.slices, 2);
        let got: Vec<Vec<&str>> = p.slices.iter().map(|s| s.cells.iter().map(|&c| p.nl.cell(c).name.as_str()).collect()).collect();
        assert_eq!(got, vec![vec!["l0", "l2"], vec!["l1", "l3"]]);
        // Density on request: unrelated slices merge down to the budget.
        opts.positions = None;
        opts.max_slices = Some(1);
        assert_eq!(pack_slices(&mut p, &opts).stats.slices, 1);
    }
}
