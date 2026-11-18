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

/// Routed delay (ps) of a connection by its horizontal and vertical tile
/// offsets, measured from a routed design: VPR's delay lookup matrix.
/// `examples/delay_grid.rs` builds one from `sta`'s `STA_ARCS`. On 7-series
/// a step across columns and a step along rows are not the same wire, which
/// `wire_ps`, by distance alone, cannot see.
#[derive(Debug, Clone, PartialEq)]
pub struct DelayGrid {
    /// Offsets of the columns, ascending from 0.
    pub dx: Vec<f64>,
    /// Offsets of the rows, ascending from 0.
    pub dy: Vec<f64>,
    /// Row by row: the delay at (`dx[i]`, `dy[j]`) is `ps[j * dx.len() + i]`.
    pub ps: Vec<f64>,
}

impl DelayGrid {
    /// Text form: a `dx` line and a `dy` line of offsets, then one line of
    /// delays per `dy`, smallest first. `#` starts a comment.
    pub fn parse(text: &str) -> Result<DelayGrid, String> {
        let mut lines = text.lines().map(|l| l.split('#').next().unwrap_or("").trim()).filter(|l| !l.is_empty());
        let nums = |l: &str| -> Result<Vec<f64>, String> {
            l.split_whitespace().map(|v| v.parse::<f64>().map_err(|_| format!("delay grid: {v} is not a number"))).collect()
        };
        let mut axis = |name: &str| -> Result<Vec<f64>, String> {
            let l = lines.next().ok_or(format!("delay grid: no {name} line"))?;
            let rest = l.strip_prefix(name).and_then(|r| r.trim_start().strip_prefix(':')).ok_or(format!("delay grid: expected `{name}:`"))?;
            let v = nums(rest)?;
            if v.len() < 2 || v[0] != 0.0 || v.windows(2).any(|w| w[1] <= w[0]) {
                return Err(format!("delay grid: {name} must rise from 0 over at least two points"));
            }
            Ok(v)
        };
        let dx = axis("dx")?;
        let dy = axis("dy")?;
        let mut ps = Vec::with_capacity(dx.len() * dy.len());
        for row in lines {
            let r = nums(row)?;
            if r.len() != dx.len() {
                return Err(format!("delay grid: a row of {} values for {} columns", r.len(), dx.len()));
            }
            ps.extend(r);
        }
        if ps.len() != dx.len() * dy.len() || ps.iter().any(|v| !v.is_finite() || *v < 0.0) {
            return Err(format!("delay grid: {} rows for {} offsets, or a bad delay", ps.len() / dx.len().max(1), dy.len()));
        }
        Ok(DelayGrid { dx, dy, ps })
    }

    /// The text `parse` reads.
    pub fn to_text(&self) -> String {
        let join = |v: &[f64], prec: usize| v.iter().map(|x| format!("{x:.prec$}")).collect::<Vec<_>>().join(" ");
        let mut out = format!("dx: {}\ndy: {}\n", join(&self.dx, 0), join(&self.dy, 0));
        for row in self.ps.chunks(self.dx.len()) {
            out.push_str(&join(row, 1));
            out.push('\n');
        }
        out
    }

    /// The delay at offsets (`dx`, `dy`), either sign: bilinear between the
    /// grid's points and along its last segment past them, never under the
    /// delay at (0, 0).
    pub fn ps(&self, dx: f64, dy: f64) -> f64 {
        let (i, tx) = segment(&self.dx, dx.abs());
        let (j, ty) = segment(&self.dy, dy.abs());
        let n = self.dx.len();
        let at = |a: usize, b: usize| self.ps[b * n + a];
        let lo = at(i - 1, j - 1) + tx * (at(i, j - 1) - at(i - 1, j - 1));
        let hi = at(i - 1, j) + tx * (at(i, j) - at(i - 1, j));
        (lo + ty * (hi - lo)).max(self.ps[0])
    }
}

/// The segment of `points` (rising from 0, two or more) that `v` falls in,
/// as the index of its upper end and the fraction along it; past the last
/// point, the last segment and a fraction over 1.
fn segment(points: &[f64], v: f64) -> (usize, f64) {
    let k = points.iter().position(|&p| p >= v).unwrap_or(points.len() - 1).max(1);
    (k, (v - points[k - 1]) / (points[k] - points[k - 1]))
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TimingOpts {
    /// Refresh the weights every this many Nesterov iterations; 0: no
    /// timing.
    pub every: usize,
    /// Weight `1 + beta * rel^exp`.
    pub beta: f64,
    pub exp: f64,
    /// The exponent at the first update. It rises to `exp` as overflow
    /// falls to the target: weight spread across the failing nets early,
    /// drawn onto the worst late (VPR's adaptive criticality exponent, from
    /// 1 to 8). Equal to `exp`: constant, as before.
    pub exp_first: f64,
    /// The share of the previous weights kept by each update, blended in
    /// log space; criticality swings while instances are still moving, and
    /// this damps it (DREAMPlace 4.0, decay 0.5). 0: each update replaces
    /// the weights, as before.
    pub momentum: f64,
    /// No update until overflow is at or below this: early placements are
    /// too overlapped for an analysis to mean much (DREAMPlace 4.0 waits
    /// 500 iterations). Infinite: from the first iteration, as before.
    pub start_overflow: f64,
}

impl Default for TimingOpts {
    /// On, every 10 iterations: on picoBEAM 6.7 ns off the worst path for
    /// 1.27x the placement time, where every iteration was no better at
    /// 4.6x (`docs/clock-improvement.md`, Phase 3). `beta` and `exp`
    /// ours, not swept.
    fn default() -> TimingOpts {
        TimingOpts { every: 10, beta: 4.0, exp: 4.0, exp_first: 4.0, momentum: 0.0, start_overflow: f64::INFINITY }
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
    /// Wire delays from this instead of `wire_ps`.
    grid: Option<std::sync::Arc<DelayGrid>>,
    /// Per net, the log of its weight carried between updates (`momentum`).
    log_w: Vec<f64>,
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
        PlaceTiming { timer, ends, net_of, opts, grid: None, log_w: Vec::new() }
    }

    /// Estimate wire delays from `grid` rather than the built-in table.
    pub fn with_grid(mut self, grid: Option<std::sync::Arc<DelayGrid>>) -> PlaceTiming {
        self.grid = grid;
        self
    }

    /// Arcs in the graph.
    pub fn arcs(&self) -> usize {
        self.ends.len()
    }

    /// Delays from `st`, an analysis, and a weight per net of the problem.
    /// `progress` runs from 0 at the first update to 1 at the target
    /// overflow, for `exp_first`.
    pub fn weights(&mut self, st: &PlacementState, nets: usize, progress: f64) -> (Vec<f64>, Update) {
        for (a, &(di, si, local)) in self.ends.iter().enumerate() {
            let ps = match (local, di, si) {
                (true, ..) => 0.0,
                (false, Some(i), Some(j)) => {
                    let dx = (st.x[i as usize] - st.x[j as usize]).abs();
                    let dy = (st.y[i as usize] - st.y[j as usize]).abs();
                    match &self.grid {
                        Some(g) => g.ps(dx, dy),
                        None => wire_ps(dx + dy),
                    }
                }
                _ => match &self.grid {
                    Some(g) => g.ps(0.0, 0.0),
                    None => wire_ps(0.0),
                },
            };
            self.timer.set_arc_ps(a, ps.round() as i64);
        }
        let table = self.timer.analyze();
        let worst = table.paths.iter().filter(|p| p.from_clk == p.to_clk).map(|p| p.delay_ps).max().unwrap_or(0);
        let TimingOpts { beta, exp, exp_first, momentum, .. } = self.opts;
        // This update's exponent. Kept exactly `exp` when constant: the
        // defaults must reproduce the placements made before the schedule.
        let e = if exp_first == exp { exp } else { exp_first + (exp - exp_first) * progress.clamp(0.0, 1.0) };
        let mut w: Vec<f64> = (0..nets)
            .map(|n| {
                let rel = self.net_of.get(n).copied().flatten().and_then(|b| table.net_rel.get(b as usize)).copied().unwrap_or(0.0);
                1.0 + beta * f64::from(rel).powf(e)
            })
            .collect();
        if momentum > 0.0 {
            // Weights start at 1 (log 0) and move towards each update's.
            if self.log_w.len() != nets {
                self.log_w = vec![0.0; nets];
            }
            for (lw, v) in self.log_w.iter_mut().zip(w.iter_mut()) {
                *lw = momentum * *lw + (1.0 - momentum) * v.ln();
                *v = lw.exp();
            }
        }
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

    fn grid() -> DelayGrid {
        // Columns cost more than rows here, as on 7-series.
        DelayGrid { dx: vec![0.0, 2.0, 4.0], dy: vec![0.0, 4.0], ps: vec![300.0, 500.0, 700.0, 400.0, 600.0, 800.0] }
    }

    #[test]
    fn the_grid_hits_its_points_and_interpolates_between_them() {
        let g = grid();
        assert_eq!(g.ps(0.0, 0.0), 300.0);
        assert_eq!(g.ps(4.0, 0.0), 700.0);
        assert_eq!(g.ps(4.0, 4.0), 800.0);
        assert!((g.ps(1.0, 0.0) - 400.0).abs() < 1e-9);
        assert!((g.ps(1.0, 2.0) - 450.0).abs() < 1e-9);
        // Sign does not matter, and a step in x is not a step in y.
        assert_eq!(g.ps(-2.0, -4.0), g.ps(2.0, 4.0));
        assert!(g.ps(2.0, 0.0) > g.ps(0.0, 2.0));
    }

    #[test]
    fn the_grid_extends_along_its_last_segment_and_never_dips_under_zero_offset() {
        let g = grid();
        assert!((g.ps(6.0, 0.0) - 900.0).abs() < 1e-9);
        assert!((g.ps(0.0, 8.0) - 500.0).abs() < 1e-9);
        let flat = DelayGrid { dx: vec![0.0, 1.0], dy: vec![0.0, 1.0], ps: vec![300.0, 100.0, 300.0, 100.0] };
        assert_eq!(flat.ps(5.0, 0.0), 300.0);
    }

    #[test]
    fn the_grid_text_round_trips_and_bad_text_is_refused() {
        let g = grid();
        assert_eq!(DelayGrid::parse(&g.to_text()), Ok(g.clone()));
        let commented = format!("# from somewhere\n{}", g.to_text());
        assert_eq!(DelayGrid::parse(&commented), Ok(g));
        assert!(DelayGrid::parse("dx: 0 1\ndy: 0 1\n1 2\n").is_err(), "a row short");
        assert!(DelayGrid::parse("dx: 1 2\ndy: 0 1\n1 2\n3 4\n").is_err(), "dx not from 0");
        assert!(DelayGrid::parse("dx: 0 0\ndy: 0 1\n1 2\n3 4\n").is_err(), "dx not rising");
        assert!(DelayGrid::parse("dy: 0 1\ndx: 0 1\n1 2\n3 4\n").is_err(), "axes swapped");
    }

    #[test]
    fn the_defaults_keep_the_schedule_off() {
        let t = TimingOpts::default();
        assert_eq!(t.exp_first, t.exp, "constant exponent");
        assert_eq!(t.momentum, 0.0, "each update replaces the weights");
        assert!(t.start_overflow.is_infinite(), "updates from the first iteration");
    }
}
