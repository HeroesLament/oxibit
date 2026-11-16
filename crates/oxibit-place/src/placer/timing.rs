//! Timing for global placement: the packed netlist's timing graph
//! (`oxibit_pnr::sta::Design::from_cells`, the same analysis the router
//! and the final report use), wire delays estimated from where the
//! instances are, and from it a weight per placer net.
//!
//! Each update sets every arc's delay from the Manhattan distance between
//! its two instances (`wire_ps`), analyses, and weighs net `e` by
//! `1 + beta * rel^exp`, `rel` its relative criticality
//! (`SlackTable::net_rel`: the slowest path through the net over its
//! clock's slowest path). Relative, not slack: estimated delays need not
//! fail any period for the deep paths to matter, and the weights then do
//! not depend on the constraints at all.
//!
//! The weights multiply a net's pull in the wirelength gradient
//! (`nesterov::run`), so a critical net is drawn shorter at the cost of
//! others. They are refreshed as the instances move, every
//! `TimingOpts::every` iterations.

use std::time::Instant;

use oxibit_pnr::delay::{CellKind, Model};
use oxibit_pnr::sta::{CellSpec, Constraints, Design, Timer};

use super::{PlacementState, Problem};
use crate::netlist::Dir;
use crate::pack::Packer;

/// Mean routed delay (ps) of a connection by the tile distance between its
/// two cells: picoBEAM `xc7_m5` on nextpnr's placement, routed by oxibit
/// (span-aware, timing on; `docs/clock-improvement.md`, run `nts`),
/// 207754 arcs. Site pips included, as the analysis counts them.
const DELAY_TABLE: [(f64, f64); 12] = [
    (0.0, 266.0),
    (1.0, 367.0),
    (2.0, 431.0),
    (5.0, 605.0),
    (10.0, 854.0),
    (20.0, 1338.0),
    (30.0, 1732.0),
    (40.0, 2147.0),
    (60.0, 2803.0),
    (80.0, 3483.0),
    (100.0, 4622.0),
    (150.0, 8570.0),
];

/// Estimated delay (ps) of a connection spanning `dist` tiles: the table,
/// linear between its points and past its last two.
pub fn wire_ps(dist: f64) -> f64 {
    let t = &DELAY_TABLE;
    let k = t.iter().position(|&(d, _)| d >= dist).unwrap_or(t.len() - 1).max(1);
    let ((d0, p0), (d1, p1)) = (t[k - 1], t[k]);
    (p0 + (dist - d0) * (p1 - p0) / (d1 - d0)).max(t[0].1)
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TimingOpts {
    /// Refresh the weights every this many Nesterov iterations; 0: no
    /// timing.
    pub every: usize,
    /// Weight `1 + beta * rel^exp`.
    pub beta: f64,
    pub exp: f64,
}

impl Default for TimingOpts {
    /// On, every 10 iterations: on picoBEAM 6.7 ns off the worst path for
    /// 1.27x the placement time, where every iteration was no better at
    /// 4.6x (`docs/clock-improvement.md`, Phase 3). `beta` and `exp`
    /// ours, not swept.
    fn default() -> TimingOpts {
        TimingOpts { every: 10, beta: 4.0, exp: 4.0 }
    }
}

/// The timing graph of one `Problem`, ready to be weighed.
pub struct PlaceTiming {
    timer: Timer,
    /// Per arc: driver and sink instance, and whether it stays inside one
    /// slice (a LUT or mux into a flop or mux beside it: no routing).
    ends: Vec<(Option<u32>, Option<u32>, bool)>,
    /// Per placer net: its net in the timing graph.
    net_of: Vec<Option<u32>>,
    pub opts: TimingOpts,
}

/// What one update found.
#[derive(Debug, Clone, Copy, Default)]
pub struct Update {
    /// The slowest estimated path of any clock, ns.
    pub worst_ns: f64,
    /// Nets with a weight above 1.5.
    pub heavy: usize,
}

impl PlaceTiming {
    /// The timing graph of `prob`, built from `p`'s netlist.
    pub fn build(p: &Packer, prob: &Problem, opts: TimingOpts) -> PlaceTiming {
        let nl = &p.nl;
        let mut inst_of = std::collections::HashMap::new();
        for (i, inst) in prob.insts.iter().enumerate() {
            for &c in &inst.cells {
                inst_of.insert(c, i as u32);
            }
        }
        let mut specs = Vec::new();
        let mut inst = Vec::new();
        let mut bits = 0usize;
        for c in nl.cell_ids() {
            let cell = nl.cell(c);
            let Some(kind) = CellSpec::kind_of(&cell.ty) else { continue };
            let mut pins = Vec::new();
            for (name, port) in &cell.ports {
                let Some(n) = port.net.filter(|&n| !nl.is_const(n)) else { continue };
                bits = bits.max(n.0 as usize + 1);
                pins.push((name.clone(), port.dir == Dir::Output, n.0));
            }
            let reg = |k: &str| cell.int_param(k) == Some(1);
            specs.push(CellSpec { name: cell.name.clone(), kind, pins, out_reg: [reg("DOA_REG"), reg("DOB_REG")] });
            inst.push(inst_of.get(&c).copied());
        }
        let d = Design::from_cells(specs, bits, bits, &|b| Some(b));
        let ends = (0..d.arcs_len())
            .map(|a| {
                let (drv, sink) = d.arc_cells(a);
                let di = drv.and_then(|c| inst[c]);
                let si = inst[sink];
                let local = drv.is_some_and(|c| matches!(d.cell_kind(c), CellKind::Lut | CellKind::Mux)) && matches!(d.cell_kind(sink), CellKind::Ff | CellKind::Mux);
                (di, si, local && di.is_some() && di == si)
            })
            .collect();
        let net_of = prob.net_ids.iter().map(|n| Some(n.0).filter(|&b| (b as usize) < bits)).collect();
        // Any period: relative criticality does not depend on it.
        let timer = Timer::new(d, &Constraints { default_ps: Some(1_000_000_000), clocks: vec![] }, Model::Intrinsic);
        PlaceTiming { timer, ends, net_of, opts }
    }

    /// Arcs in the graph.
    pub fn arcs(&self) -> usize {
        self.ends.len()
    }

    /// Delays from `st`, an analysis, and a weight per net of the problem.
    pub fn weights(&mut self, st: &PlacementState, nets: usize) -> (Vec<f64>, Update) {
        for (a, &(di, si, local)) in self.ends.iter().enumerate() {
            let ps = match (local, di, si) {
                (true, ..) => 0.0,
                (false, Some(i), Some(j)) => wire_ps((st.x[i as usize] - st.x[j as usize]).abs() + (st.y[i as usize] - st.y[j as usize]).abs()),
                _ => wire_ps(0.0),
            };
            self.timer.set_arc_ps(a, ps.round() as i64);
        }
        let table = self.timer.analyze();
        let worst = table.paths.iter().filter(|p| p.from_clk == p.to_clk).map(|p| p.delay_ps).max().unwrap_or(0);
        let TimingOpts { beta, exp, .. } = self.opts;
        let w: Vec<f64> = (0..nets)
            .map(|e| {
                let rel = self.net_of.get(e).copied().flatten().and_then(|n| table.net_rel.get(n as usize)).copied().unwrap_or(0.0);
                1.0 + beta * f64::from(rel).powf(exp)
            })
            .collect();
        let heavy = w.iter().filter(|&&v| v > 1.5).count();
        (w, Update { worst_ns: worst as f64 / 1000.0, heavy })
    }
}

/// Timing updates over one global placement.
#[derive(Debug, Clone, Default)]
pub struct Stats {
    pub updates: usize,
    pub seconds: f64,
    /// (iteration, estimated worst path ns, heavy nets) at each update.
    pub trace: Vec<(usize, f64, usize)>,
}

impl Stats {
    pub fn time<T>(&mut self, f: impl FnOnce() -> T) -> T {
        let t = Instant::now();
        let out = f();
        self.seconds += t.elapsed().as_secs_f64();
        self.updates += 1;
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_delay_estimate_follows_the_table_and_extends_past_it() {
        assert_eq!(wire_ps(0.0), 266.0);
        assert_eq!(wire_ps(10.0), 854.0);
        assert!((wire_ps(15.0) - (854.0 + 1338.0) / 2.0).abs() < 1e-9);
        // Past the last point, along the last segment.
        assert!((wire_ps(200.0) - (8570.0 + 50.0 * (8570.0 - 4622.0) / 50.0)).abs() < 1e-9);
        // Never under a same-tile connection.
        assert_eq!(wire_ps(-1.0), 266.0);
    }
}
