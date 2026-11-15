//! Spreading legalisation: every slice instance onto a slice site of its
//! own, staying close to where global placement left it.
//!
//! Not `place::legalise` (pin fix-ups after any placement); this decides
//! sites. First the macros that need a SLICEM (`Problem::needs_slicem`)
//! each take the nearest free one, in instance order: SLICEMs are a third
//! of the sites, and slice logic, which can use either kind, would
//! otherwise take them first and push the memories tens of tiles away.
//! Then each carry chain, longest first, takes the free run of slices up
//! one column (`Problem::chain_run`) whose middle is nearest where global
//! placement left it: a chain cannot bend, so it goes before the slices
//! that could take any site. Then three steps over the placer's bin grid,
//! on the sites left:
//!
//! 1. Spread. While a bin holds more instances than sites, the most
//!    overflowed bin sends one instance to the nearest bin with a site to
//!    spare: the instance already closest to that bin, moved the least
//!    that puts it inside. Each move removes one unit of overflow and
//!    creates none, so it ends.
//! 2. Assign. Within a bin, each instance takes the nearest free site.
//! 3. Refine, guarded. Each instance may move to a free site nearer the
//!    median of the pins it connects to, and the move is kept only if the
//!    wirelength of its nets goes down. Wirelength after this step is
//!    never above wirelength before it.
//!
//! The moving-along-rows cost model of Abacus (Spindler, Schlichtmann,
//! Johannes, ISPD 2008) does not apply as is: sites here are a sparse
//! grid of columns, not rows of abutting cells. What is kept is its aim,
//! least displacement from the global placement.

use serde_json::{Value, json};

use super::density::BinGrid;
use super::{InstKind, Pin, PlacementState, Problem};

#[derive(Debug, Clone, Default)]
pub struct Stats {
    /// Bin-to-bin moves in step 1.
    pub spread_moves: usize,
    /// Moves kept in step 3, and the passes it took.
    pub refine_moves: usize,
    pub refine_passes: usize,
    /// Wirelength on sites before and after step 3.
    pub hpwl_spread: f64,
    pub hpwl: f64,
    /// Displacement of slice instances from global placement (Manhattan).
    pub max_disp: f64,
    pub mean_disp: f64,
    /// Macros sited on a SLICEM first, and their displacement.
    pub memory_macros: usize,
    pub memory_max_disp: f64,
    pub memory_mean_disp: f64,
    /// Carry chains sited on a run of their own, the slices they took, the
    /// ones no free run was found for, and the displacement of a sited
    /// chain's middle.
    pub chains: usize,
    pub chain_slices: usize,
    pub chains_unsited: usize,
    pub chain_max_disp: f64,
    pub chain_mean_disp: f64,
}

impl Stats {
    pub fn to_json(&self) -> Value {
        json!({"spread_moves": self.spread_moves, "refine_moves": self.refine_moves, "refine_passes": self.refine_passes,
               "max_disp": self.max_disp, "mean_disp": self.mean_disp,
               "memory_macros": self.memory_macros, "memory_max_disp": self.memory_max_disp, "memory_mean_disp": self.memory_mean_disp,
               "chains": self.chains, "chain_slices": self.chain_slices, "chains_unsited": self.chains_unsited,
               "chain_max_disp": self.chain_max_disp, "chain_mean_disp": self.chain_mean_disp})
    }
}

#[derive(Debug, Clone)]
pub struct Legal {
    /// Slice instances and memory macros at their sites; other macros
    /// where `gp` had them.
    pub state: PlacementState,
    /// Per instance, its site (index into `Problem::sites`): every slice,
    /// every memory macro a SLICEM was found for, and the bottom site of
    /// every carry chain a run was found for (`Problem::chain_run` from
    /// there gives the rest).
    pub site_of: Vec<Option<u32>>,
    pub stats: Stats,
}

/// Passes of step 3 at most.
const REFINE_PASSES: usize = 4;
/// Bins either side of the target bin that step 3 looks for a free site in.
const REFINE_REACH: isize = 2;

fn manhattan(a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - b.0).abs() + (a.1 - b.1).abs()
}

fn median(v: &mut [f64]) -> f64 {
    v.sort_unstable_by(f64::total_cmp);
    let m = v.len() / 2;
    if v.len() % 2 == 1 { v[m] } else { (v[m - 1] + v[m]) / 2.0 }
}

pub fn legalise_spread(prob: &Problem, grid: &BinGrid, gp: &PlacementState) -> Result<Legal, String> {
    let slices: Vec<usize> = (0..prob.insts.len()).filter(|&i| prob.insts[i].kind == InstKind::Slice).collect();
    let mut st = gp.clone();
    let mut stats = Stats::default();
    let mut taken = vec![false; prob.sites.len()];
    let mut site_of: Vec<Option<u32>> = vec![None; prob.insts.len()];
    let site_pos = |s: u32| (prob.sites[s as usize].x, prob.sites[s as usize].y);

    // 0. Memory macros onto SLICEMs, before any slice.
    let mut mem_disp: Vec<f64> = Vec::new();
    for i in (0..prob.insts.len()).filter(|&i| prob.needs_slicem.get(i) == Some(&true)) {
        let here = (st.x[i], st.y[i]);
        let s = (0..prob.sites.len() as u32)
            .filter(|&s| !taken[s as usize] && prob.site_slicem.get(s as usize) == Some(&true))
            .min_by(|&p, &q| manhattan(site_pos(p), here).total_cmp(&manhattan(site_pos(q), here)).then(p.cmp(&q)));
        let Some(s) = s else { continue };
        taken[s as usize] = true;
        site_of[i] = Some(s);
        (st.x[i], st.y[i]) = site_pos(s);
        mem_disp.push(manhattan(site_pos(s), here));
    }
    stats.memory_macros = mem_disp.len();
    stats.memory_max_disp = mem_disp.iter().copied().fold(0.0, f64::max);
    stats.memory_mean_disp = if mem_disp.is_empty() { 0.0 } else { mem_disp.iter().sum::<f64>() / mem_disp.len() as f64 };

    // 0b. Carry chains onto runs up one column, longest first.
    let mut chains: Vec<usize> = (0..prob.insts.len()).filter(|&i| prob.chain_rows.get(i).is_some_and(|&n| n > 0)).collect();
    chains.sort_by_key(|&i| (std::cmp::Reverse(prob.chain_rows[i]), i));
    let mut chain_disp: Vec<f64> = Vec::new();
    for i in chains {
        let n = prob.chain_rows[i];
        let here = (st.x[i], st.y[i]);
        let mut best: Option<(f64, u32, (f64, f64))> = None;
        for s in 0..prob.sites.len() as u32 {
            if taken[s as usize] {
                continue;
            }
            // A cheap bound first: the run's middle is no nearer than its
            // column is.
            if best.is_some_and(|(d, _, _)| (prob.sites[s as usize].x - here.0).abs() >= d) {
                continue;
            }
            let Some(run) = prob.chain_run(s, n) else { continue };
            if run.iter().any(|&t| taken[t as usize]) {
                continue;
            }
            let top = site_pos(*run.last().expect("n > 0"));
            let mid = (top.0, (site_pos(s).1 + top.1) / 2.0);
            let d = manhattan(mid, here);
            if best.is_none_or(|(b, bs, _)| d < b || (d == b && s < bs)) {
                best = Some((d, s, mid));
            }
        }
        let Some((d, s, mid)) = best else {
            stats.chains_unsited += 1;
            continue;
        };
        for t in prob.chain_run(s, n).expect("found") {
            taken[t as usize] = true;
        }
        site_of[i] = Some(s);
        (st.x[i], st.y[i]) = mid;
        chain_disp.push(d);
        stats.chains += 1;
        stats.chain_slices += n;
    }
    stats.chain_max_disp = chain_disp.iter().copied().fold(0.0, f64::max);
    stats.chain_mean_disp = if chain_disp.is_empty() { 0.0 } else { chain_disp.iter().sum::<f64>() / chain_disp.len() as f64 };
    if stats.chains_unsited > 0 {
        return Err(format!("{} carry chains found no free run of slices", stats.chains_unsited));
    }

    let free_sites = taken.iter().filter(|&&t| !t).count();
    if slices.len() > free_sites {
        return Err(format!("{} slices, {} free slice sites", slices.len(), free_sites));
    }
    // Room per bin: its sites the memories left.
    let room: Vec<usize> = (0..grid.len()).map(|b| grid.sites[b].iter().filter(|&&s| !taken[s as usize]).count()).collect();
    let cap = |b: usize| room[b];

    // 1. Spread.
    let mut members: Vec<Vec<usize>> = vec![Vec::new(); grid.len()];
    for &i in &slices {
        members[grid.bin_at(st.x[i], st.y[i])].push(i);
    }
    loop {
        let worst = (0..grid.len())
            .filter(|&b| members[b].len() > cap(b))
            .max_by_key(|&b| (members[b].len() - cap(b), std::cmp::Reverse(b)));
        let Some(b) = worst else { break };
        let from = grid.centre(b);
        let to = (0..grid.len())
            .filter(|&t| members[t].len() < cap(t))
            .min_by(|&p, &q| manhattan(grid.centre(p), from).total_cmp(&manhattan(grid.centre(q), from)).then(p.cmp(&q)))
            .expect("more sites than slices");
        let (tx, ty) = grid.centre(to);
        let k = (0..members[b].len())
            .min_by(|&p, &q| {
                let d = |k: usize| manhattan((st.x[members[b][k]], st.y[members[b][k]]), (tx, ty));
                d(p).total_cmp(&d(q)).then(members[b][p].cmp(&members[b][q]))
            })
            .expect("an overflowed bin has members");
        let i = members[b].swap_remove(k);
        st.x[i] = st.x[i].clamp(tx - grid.bw / 2.0, tx + grid.bw / 2.0);
        st.y[i] = st.y[i].clamp(ty - grid.bh / 2.0, ty + grid.bh / 2.0);
        members[to].push(i);
        stats.spread_moves += 1;
    }

    // 2. Assign.
    for (b, m) in members.iter_mut().enumerate() {
        m.sort_unstable();
        for &i in m.iter() {
            let s = grid.sites[b]
                .iter()
                .copied()
                .filter(|&s| !taken[s as usize])
                .min_by(|&p, &q| manhattan(site_pos(p), (st.x[i], st.y[i])).total_cmp(&manhattan(site_pos(q), (st.x[i], st.y[i]))).then(p.cmp(&q)))
                .expect("the bin has room");
            taken[s as usize] = true;
            site_of[i] = Some(s);
            (st.x[i], st.y[i]) = site_pos(s);
        }
    }
    stats.hpwl_spread = prob.hpwl(&st);

    // 3. Refine, keeping only moves that shorten the instance's nets.
    let local = |st: &PlacementState, i: usize| prob.inst_nets[i].iter().map(|&e| prob.net_hpwl(e, st)).sum::<f64>();
    for _ in 0..REFINE_PASSES {
        stats.refine_passes += 1;
        let mut moved = 0;
        for &i in &slices {
            let (mut xs, mut ys): (Vec<f64>, Vec<f64>) = prob.inst_nets[i]
                .iter()
                .flat_map(|&e| prob.nets[e as usize].pins.iter().copied())
                .filter(|&p| p != Pin::Inst(i as u32))
                .map(|p| prob.pin_pos(p, &st))
                .unzip();
            if xs.is_empty() {
                continue;
            }
            let want = (median(&mut xs), median(&mut ys));
            let here = (st.x[i], st.y[i]);
            let tb = grid.bin_at(want.0, want.1);
            let (ti, tj) = ((tb % grid.nx) as isize, (tb / grid.nx) as isize);
            let near = (-REFINE_REACH..=REFINE_REACH)
                .flat_map(|dj| (-REFINE_REACH..=REFINE_REACH).map(move |di| (ti + di, tj + dj)))
                .filter(|&(bi, bj)| bi >= 0 && bj >= 0 && (bi as usize) < grid.nx && (bj as usize) < grid.ny)
                .flat_map(|(bi, bj)| grid.sites[bj as usize * grid.nx + bi as usize].iter().copied())
                .filter(|&s| !taken[s as usize])
                .min_by(|&p, &q| manhattan(site_pos(p), want).total_cmp(&manhattan(site_pos(q), want)).then(p.cmp(&q)));
            let Some(s) = near else { continue };
            if manhattan(site_pos(s), want) >= manhattan(here, want) {
                continue;
            }
            let before = local(&st, i);
            (st.x[i], st.y[i]) = site_pos(s);
            if local(&st, i) < before - 1e-9 {
                let old = site_of[i].replace(s).expect("assigned");
                taken[old as usize] = false;
                taken[s as usize] = true;
                moved += 1;
            } else {
                (st.x[i], st.y[i]) = here;
            }
        }
        stats.refine_moves += moved;
        if moved == 0 {
            break;
        }
    }
    stats.hpwl = prob.hpwl(&st);

    let disp: Vec<f64> = slices.iter().map(|&i| manhattan((st.x[i], st.y[i]), (gp.x[i], gp.y[i]))).collect();
    stats.max_disp = disp.iter().copied().fold(0.0, f64::max);
    stats.mean_disp = if disp.is_empty() { 0.0 } else { disp.iter().sum::<f64>() / disp.len() as f64 };
    Ok(Legal { state: st, site_of, stats })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::placer::{Inst, PNet, Site};

    fn field(side: usize, n: u32) -> Problem {
        let sites = (0..side * side)
            .map(|k| Site {
                x: (k % side) as f64,
                y: (k / side) as f64,
                name: format!("S{k}"),
            })
            .collect();
        let insts = vec![
            Inst {
                cells: Vec::new(),
                kind: InstKind::Slice
            };
            n as usize
        ];
        // A chain, tied to a pad at the origin.
        let mut nets: Vec<PNet> = (1..n)
            .map(|i| PNet {
                pins: vec![Pin::Inst(i - 1), Pin::Inst(i)],
                weight: 1.0,
            })
            .collect();
        nets.push(PNet {
            pins: vec![Pin::Fixed(0.0, 0.0), Pin::Inst(0)],
            weight: 1.0,
        });
        Problem::from_parts(insts, nets, sites)
    }

    #[test]
    fn a_stack_ends_up_on_distinct_sites_and_refinement_only_helps() {
        let prob = field(6, 10);
        let grid = BinGrid::new(&prob, 3);
        let gp = PlacementState {
            x: vec![4.2; 10],
            y: vec![3.9; 10],
        };
        let l = legalise_spread(&prob, &grid, &gp).unwrap();
        let mut used: Vec<u32> = l.site_of.iter().map(|s| s.expect("every slice has a site")).collect();
        used.sort_unstable();
        used.dedup();
        assert_eq!(used.len(), 10);
        for (i, s) in l.site_of.iter().enumerate() {
            let site = &prob.sites[s.unwrap() as usize];
            assert_eq!((l.state.x[i], l.state.y[i]), (site.x, site.y));
        }
        // Four sites a bin, ten instances in one: six had to leave.
        assert_eq!(l.stats.spread_moves, 6);
        assert!(l.stats.hpwl <= l.stats.hpwl_spread, "{:?}", l.stats);
        assert_eq!(l.stats.hpwl, prob.hpwl(&l.state));
    }

    #[test]
    fn a_memory_macro_gets_the_nearest_slicem_before_any_slice() {
        // Ten slices and one memory macro piled on (4.2, 3.9); only column
        // x = 4 is SLICEM.
        let mut prob = field(6, 10);
        prob.insts.push(Inst { cells: Vec::new(), kind: InstKind::Macro });
        prob.inst_nets.push(Vec::new());
        prob.site_slicem = prob.sites.iter().map(|s| s.x == 4.0).collect();
        prob.needs_slicem = (0..prob.insts.len()).map(|i| i == 10).collect();
        let grid = BinGrid::new(&prob, 3);
        let gp = PlacementState { x: vec![4.2; 11], y: vec![3.9; 11] };
        let l = legalise_spread(&prob, &grid, &gp).unwrap();
        let m = l.site_of[10].expect("the macro has a site") as usize;
        assert_eq!((prob.sites[m].x, prob.sites[m].y), (4.0, 4.0));
        assert_eq!((l.state.x[10], l.state.y[10]), (4.0, 4.0));
        let mut used: Vec<u32> = l.site_of.iter().map(|s| s.expect("every instance has a site")).collect();
        used.sort_unstable();
        used.dedup();
        assert_eq!(used.len(), 11, "no slice shares the macro's site");
        assert_eq!(l.stats.memory_macros, 1);
    }

    /// Slice sites named as the device names them: `columns` slice columns
    /// (grid x 0, 3, 6, ...), `rows` slices each, one grid row apart except
    /// an HCLK-style gap of two between rows 1 and 2.
    fn named_columns(columns: usize, rows: i64) -> Vec<Site> {
        let mut sites = Vec::new();
        for c in 0..columns {
            for r in 0..rows {
                let gap = if r >= 2 { 1.0 } else { 0.0 };
                sites.push(Site {
                    x: 3.0 * c as f64,
                    y: 20.0 - r as f64 - gap,
                    name: format!("SLICE_X{c}Y{r}"),
                });
            }
        }
        sites
    }

    #[test]
    fn a_carry_chain_takes_a_run_up_one_column_before_any_slice() {
        // Column 0 has 5 slices, column 1 has 5. A 4-row chain aimed at
        // column 1's middle, and 6 slices: the chain's run must be four
        // slices up one column (across the gap) that no slice shares.
        let sites = named_columns(2, 5);
        let mut insts = vec![Inst { cells: Vec::new(), kind: InstKind::Slice }; 6];
        insts.push(Inst { cells: Vec::new(), kind: InstKind::Macro });
        let mut prob = Problem::from_parts(insts, Vec::new(), sites);
        prob.chain_rows = (0..7).map(|i| if i == 6 { 4 } else { 0 }).collect();
        assert_eq!(prob.site_area(6), 4.0);
        // The gap of two grid rows still chains; the column's top does not.
        assert_eq!(prob.site_up[1], Some(2));
        assert_eq!(prob.site_up[4], None);
        assert!(prob.chain_run(2, 4).is_none(), "only three slices above row 1");

        let grid = BinGrid::new(&prob, 2);
        let gp = PlacementState { x: vec![3.0; 7], y: vec![18.0; 7] };
        let l = legalise_spread(&prob, &grid, &gp).unwrap();
        let root = l.site_of[6].expect("the chain has a run");
        let run = prob.chain_run(root, 4).expect("a full run");
        assert!(run.iter().all(|&s| prob.sites[s as usize].x == 3.0), "one column: {run:?}");
        let names: Vec<&str> = run.iter().map(|&s| prob.sites[s as usize].name.as_str()).collect();
        assert_eq!(names[0].strip_prefix("SLICE_X1Y").and_then(|y| y.parse::<i64>().ok()).map(|y| y + 3), names[3].strip_prefix("SLICE_X1Y").and_then(|y| y.parse().ok()));
        for i in 0..6 {
            let s = l.site_of[i].expect("every slice has a site");
            assert!(!run.contains(&s), "slice {i} on the chain's run");
        }
        assert_eq!((l.stats.chains, l.stats.chain_slices, l.stats.chains_unsited), (1, 4, 0));
    }

    #[test]
    fn a_chain_no_column_holds_is_an_error() {
        let mut prob = Problem::from_parts(vec![Inst { cells: Vec::new(), kind: InstKind::Macro }], Vec::new(), named_columns(2, 5));
        prob.chain_rows = vec![6];
        let grid = BinGrid::new(&prob, 2);
        let gp = PlacementState { x: vec![0.0], y: vec![18.0] };
        assert!(legalise_spread(&prob, &grid, &gp).is_err());
    }

    #[test]
    fn too_many_slices_is_an_error() {
        let prob = field(2, 5);
        let grid = BinGrid::new(&prob, 2);
        let gp = PlacementState {
            x: vec![0.0; 5],
            y: vec![0.0; 5],
        };
        assert!(legalise_spread(&prob, &grid, &gp).is_err());
    }
}
