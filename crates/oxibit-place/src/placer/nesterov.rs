//! Nesterov global placement: weighted-average wirelength plus
//! electrostatic density, as in ePlace (Lu et al., TODAES 2015) and
//! RePlAce (Cheng, Kahng, Kang, Wang, TCAD 2019).
//!
//! Minimised is `W(v) + lambda * N(v)`: `W` the weighted-average (WA)
//! smooth wirelength, `N` the potential energy of the charges
//! (`density`). The pieces, each from the papers:
//!
//! - Fillers: unconnected unit charges that take up the capacity the
//!   design does not, so the field spreads real instances only as far as
//!   they need rather than across the whole device.
//! - Preconditioner: an instance's gradient is divided by its pin count
//!   plus `lambda` times its charge.
//! - Step: no line search on the objective. The step is the inverse of a
//!   local Lipschitz estimate, `|v_k - v_{k-1}| / |g_k - g_{k-1}|`,
//!   re-estimated at the new point and shrunk until the two agree
//!   (ePlace's backtracking).
//! - `lambda` starts at the ratio of the wirelength and density gradient
//!   norms and is multiplied each iteration by a factor that grows it
//!   while wirelength is not getting worse quickly.
//! - The WA smoothing `gamma` tightens as overflow falls.
//!
//! Phases per gradient: per-net sums and per-instance gradients are
//! read-only maps (`par_map`); the step-length norms are serial reduces.
//!
//! Every default in `Opts` says where it is from. "RePlAce release" means
//! the defaults documented for the authors' implementation (the
//! `global_placement` options: `-overflow`, `-min_phi_coef`,
//! `-max_phi_coef`, `-initial_place_max_iter`); "ePlace" means the
//! paper's constants as recalled here, to be checked against the text
//! before they are quoted anywhere; "ours" means chosen for this device
//! and not tuned.

use serde_json::{Value, json};

use super::density::{BinGrid, Overflow, Spectral, overflow};
use super::timing::{self, PlaceTiming};
use super::{Pin, PlacementState, Problem};
use crate::par::par_map;

#[derive(Debug, Clone, Copy)]
pub struct Opts {
    /// Bins a side. Ours: about a dozen slice sites a bin on a 7z020.
    pub bins: usize,
    /// Ours.
    pub max_iter: usize,
    /// Stop when overflow is at or below this. RePlAce release: 0.1.
    pub target_overflow: f64,
    /// Share of each bin's sites that may be used. Ours: 1.0, sites being
    /// discrete (the RePlAce release default of 0.7 is row utilisation).
    pub target_density: f64,
    /// Bounds of the per-iteration `lambda` multiplier. RePlAce release:
    /// 0.95 and 1.05 (ePlace used 0.75 and 1.1).
    pub mu_min: f64,
    pub mu_max: f64,
    /// The wirelength growth per iteration, as a fraction of the current
    /// wirelength, at which `lambda` holds still. Ours.
    ///
    /// Units, for reproducing a run: the papers' multiplier is
    /// `mu = mu_max ^ (1 - dHPWL / dHPWL_REF)` with `dHPWL_REF` an
    /// absolute length in the benchmark's database units. Lengths here
    /// are in prjxray tile-grid units (one unit is one tile column or
    /// row, the coordinates `Loc::Grid` carries), and the reference is
    /// relative: `dHPWL_REF = hpwl_ref * HPWL` at the current iterate.
    /// To restate it as an absolute length, multiply by the wirelength a
    /// report prints: 0.02 at an HPWL of 7818 is 156 tile units an
    /// iteration. Reports carry `hpwl_ref` and `units`.
    pub hpwl_ref: f64,
    /// Accept a step when the re-estimated one is at least this share of
    /// it. ePlace: 0.95.
    pub backtrack_eps: f64,
    /// Ours.
    pub backtrack_max: usize,
    /// Also stop when overflow has not improved by `stall_eps` in
    /// `stall_window` iterations. Ours.
    pub stall_window: usize,
    pub stall_eps: f64,
    /// Also stop when wirelength has grown past this multiple of what it
    /// was when overflow first came within 1.5 times the target: from
    /// there `lambda` keeps growing, and if the last of the overflow will
    /// not go (instances over ground with no sites), it trades wirelength
    /// away for nothing. Ours, from picoBEAM, where without it wirelength
    /// went from 0.42 M to 2.4 M while overflow moved from 0.145 to 0.097.
    pub runaway: f64,
}

impl Default for Opts {
    fn default() -> Opts {
        Opts {
            bins: 32,
            max_iter: 1000,
            target_overflow: 0.1,
            target_density: 1.0,
            mu_min: 0.95,
            mu_max: 1.05,
            hpwl_ref: 0.02,
            backtrack_eps: 0.95,
            backtrack_max: 10,
            stall_window: 200,
            stall_eps: 1e-3,
            runaway: 1.2,
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct Report {
    pub iters: usize,
    /// Why it stopped: `overflow`, `stall`, `runaway`, `diverged` or
    /// `max_iter`.
    pub stop: &'static str,
    pub overflow: Overflow,
    /// Most-used bin, as instances over sites.
    pub max_util: f64,
    pub lambda: f64,
    /// `Opts::hpwl_ref` of the run.
    pub hpwl_ref: f64,
    pub fillers: usize,
    /// (iteration, wirelength, overflow) every 25 iterations.
    pub trace: Vec<(usize, f64, f64)>,
    /// Timing-weight updates (`run`'s `timing`).
    pub timing: timing::Stats,
}

impl Report {
    pub fn to_json(&self) -> Value {
        json!({"iters": self.iters, "stop": self.stop, "lambda": self.lambda, "hpwl_ref": self.hpwl_ref, "units": "prjxray tile grid", "fillers": self.fillers, "max_util": self.max_util,
               "overflow": {"bins": self.overflow.bins, "max": self.overflow.max, "sum": self.overflow.sum, "ratio": self.overflow.ratio},
               "trace": self.trace,
               "timing": {"updates": self.timing.updates, "seconds": self.timing.seconds, "trace": self.timing.trace}})
    }
}

/// One axis of one net under the WA model: the extremes (for stable
/// exponents) and the sums `S = sum e`, `T = sum c e` at both ends.
#[derive(Debug, Clone, Copy)]
struct Wa {
    hi: f64,
    lo: f64,
    s_hi: f64,
    t_hi: f64,
    s_lo: f64,
    t_lo: f64,
}

fn wa(cs: impl Iterator<Item = f64> + Clone, gamma: f64) -> Wa {
    let (lo, hi) = cs.clone().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), c| (lo.min(c), hi.max(c)));
    let mut w = Wa {
        hi,
        lo,
        s_hi: 0.0,
        t_hi: 0.0,
        s_lo: 0.0,
        t_lo: 0.0,
    };
    for c in cs {
        let (eh, el) = (((c - hi) / gamma).exp(), ((lo - c) / gamma).exp());
        w.s_hi += eh;
        w.t_hi += c * eh;
        w.s_lo += el;
        w.t_lo += c * el;
    }
    w
}

/// d(WA length)/dc for a pin at `c` of a net with sums `w`.
fn wa_grad(c: f64, w: &Wa, gamma: f64) -> f64 {
    let (eh, el) = (((c - w.hi) / gamma).exp(), ((w.lo - c) / gamma).exp());
    eh * ((1.0 + c / gamma) * w.s_hi - w.t_hi / gamma) / (w.s_hi * w.s_hi) - el * ((1.0 - c / gamma) * w.s_lo + w.t_lo / gamma) / (w.s_lo * w.s_lo)
}

/// The two gradients at a point, before `lambda` joins them.
struct Raw {
    wx: Vec<f64>,
    wy: Vec<f64>,
    dx: Vec<f64>,
    dy: Vec<f64>,
}

struct Model<'a> {
    prob: &'a Problem,
    grid: &'a BinGrid,
    spec: Spectral,
    /// Real instances; variables beyond are fillers.
    n: usize,
    /// Charge per variable: the sites it takes (`Problem::site_area`: a
    /// carry chain its rows), 1 for fillers, 0 for other macros.
    q: Vec<f64>,
    /// Pins per variable.
    deg: Vec<f64>,
    /// The charge that never moves: each bin's shortfall of sites.
    fixed: Vec<f64>,
    /// Per net, a multiplier on its weight: the timing weight, 1 without.
    tw: Vec<f64>,
}

impl Model<'_> {
    fn raw(&self, x: &[f64], y: &[f64], gamma: (f64, f64)) -> Raw {
        let prob = self.prob;
        let at = |pin: Pin| match pin {
            Pin::Inst(i) => (x[i as usize], y[i as usize]),
            Pin::Fixed(fx, fy) => (fx, fy),
        };
        // Per net, read-only.
        let stats: Vec<(Wa, Wa)> = par_map(prob.nets.len(), |e| {
            let pins = &prob.nets[e as usize].pins;
            (wa(pins.iter().map(|&p| at(p).0), gamma.0), wa(pins.iter().map(|&p| at(p).1), gamma.1))
        });
        // Per instance, read-only.
        let wl: Vec<(f64, f64)> = par_map(self.n, |i| {
            let (mut gx, mut gy) = (0.0, 0.0);
            for &e in &prob.inst_nets[i as usize] {
                let (sx, sy) = &stats[e as usize];
                let w = prob.nets[e as usize].weight * self.tw[e as usize];
                gx += w * wa_grad(x[i as usize], sx, gamma.0);
                gy += w * wa_grad(y[i as usize], sy, gamma.1);
            }
            (gx, gy)
        });
        let total = x.len();
        // A real instance's charge sits on its footprint (a carry chain is
        // a bar of unit charges, `Problem::footprint`); a filler's is a
        // point.
        let unit = |i: usize| -> Vec<f64> { if i < self.n { self.prob.footprint(i).collect() } else { vec![0.0] } };
        let mut rho = self.fixed.clone();
        for i in 0..total {
            if self.q[i] != 0.0 {
                let offs = unit(i);
                let q = self.q[i] / offs.len() as f64;
                for dy in offs {
                    self.grid.deposit(x[i], y[i] + dy, q, &mut rho);
                }
            }
        }
        let (ex, ey) = self.spec.field(&rho);
        // Energy falls along the field, so its gradient is against it.
        let dens: Vec<(f64, f64)> = par_map(total, |i| {
            let (i, q) = (i as usize, self.q[i as usize]);
            if q == 0.0 {
                return (0.0, 0.0);
            }
            let offs = unit(i);
            let q = q / offs.len() as f64;
            offs.iter().fold((0.0, 0.0), |(gx, gy), dy| {
                (gx - q * self.grid.sample(x[i], y[i] + dy, &ex), gy - q * self.grid.sample(x[i], y[i] + dy, &ey))
            })
        });
        let pad = |v: Vec<f64>| -> Vec<f64> { v.into_iter().chain(std::iter::repeat(0.0)).take(total).collect() };
        Raw {
            wx: pad(wl.iter().map(|g| g.0).collect()),
            wy: pad(wl.iter().map(|g| g.1).collect()),
            dx: dens.iter().map(|g| g.0).collect(),
            dy: dens.iter().map(|g| g.1).collect(),
        }
    }

    /// The preconditioned gradient of `W + lambda N`.
    fn gradient(&self, raw: &Raw, lambda: f64) -> (Vec<f64>, Vec<f64>) {
        let pre = |i: usize| (self.deg[i] + lambda * self.q[i]).max(1.0);
        let gx = (0..raw.wx.len()).map(|i| (raw.wx[i] + lambda * raw.dx[i]) / pre(i)).collect();
        let gy = (0..raw.wy.len()).map(|i| (raw.wy[i] + lambda * raw.dy[i]) / pre(i)).collect();
        (gx, gy)
    }

    fn clamp(&self, x: &mut [f64], y: &mut [f64]) {
        let r = self.grid.region;
        for v in x.iter_mut() {
            *v = v.clamp(r.x0, r.x1);
        }
        for v in y.iter_mut() {
            *v = v.clamp(r.y0, r.y1);
        }
    }

    /// The real instances' part of a point.
    fn state(&self, x: &[f64], y: &[f64]) -> PlacementState {
        PlacementState {
            x: x[..self.n].to_vec(),
            y: y[..self.n].to_vec(),
        }
    }
}

/// Distance between two points, both axes together.
fn dist(ax: &[f64], ay: &[f64], bx: &[f64], by: &[f64]) -> f64 {
    let sq = |a: &[f64], b: &[f64]| a.iter().zip(b).map(|(p, q)| (p - q) * (p - q)).sum::<f64>();
    (sq(ax, bx) + sq(ay, by)).sqrt()
}

fn axpy(a: &[f64], k: f64, b: &[f64]) -> Vec<f64> {
    a.iter().zip(b).map(|(p, q)| p + k * q).collect()
}

/// Global placement from `start`: the spread coordinates of the real
/// instances (the iterate with the least overflow), and the report.
///
/// With `timing`, each net's pull is multiplied by its timing weight,
/// refreshed from the current iterate every `TimingOpts::every`
/// iterations from the first (`PlaceTiming::weights`). The wirelength
/// the schedule and the stops read stays unweighted.
pub fn run(prob: &Problem, grid: &BinGrid, start: PlacementState, opts: &Opts, mut timing: Option<&mut PlaceTiming>) -> (PlacementState, Report) {
    let n = prob.insts.len();
    let slices = (0..n).map(|i| prob.site_area(i)).sum::<f64>().round() as usize;
    let cap: f64 = (0..grid.len()).map(|b| grid.cap(b)).sum();
    let fillers = ((cap * opts.target_density).round() as usize).saturating_sub(slices);
    let total = n + fillers;

    let mut q: Vec<f64> = (0..n).map(|i| prob.site_area(i)).collect();
    q.resize(total, 1.0);
    let mut deg: Vec<f64> = prob.inst_nets.iter().map(|v| v.len() as f64).collect();
    deg.resize(total, 0.0);
    let mut model = Model {
        prob,
        grid,
        spec: Spectral::new(grid),
        n,
        q,
        deg,
        fixed: (0..grid.len()).map(|b| grid.cap_max - grid.cap(b)).collect(),
        tw: vec![1.0; prob.nets.len()],
    };
    let mut tstats = timing::Stats::default();
    // Overflow at the first timing update: where the exponent schedule starts.
    let mut first_ratio: Option<f64> = None;
    // Refresh the timing weights from the instances at `x`, `y`, at
    // overflow `ratio`.
    let mut reweigh = |k: usize, ratio: f64, model: &mut Model, x: &[f64], y: &[f64]| -> bool {
        let Some(t) = timing.as_deref_mut() else { return false };
        if t.opts.every == 0 || k % t.opts.every != 0 || ratio > t.opts.start_overflow {
            return false;
        }
        // 0 at the first update, 1 once overflow reaches the target.
        let o = *first_ratio.get_or_insert(ratio);
        let target = opts.target_overflow;
        let progress = if o > target { ((o - ratio) / (o - target)).clamp(0.0, 1.0) } else { 1.0 };
        let st = model.state(x, y);
        let (w, up) = tstats.time(|| t.weights(&st, prob.nets.len(), progress));
        model.tw = w;
        tstats.trace.push((k, up.worst_ns, up.heavy));
        true
    };

    // Fillers start evenly over the sites: capacity-shaped from the start.
    let (mut ux, mut uy) = (start.x, start.y);
    for k in 0..fillers {
        let s = &prob.sites[k * prob.sites.len() / fillers];
        ux.push(s.x);
        uy.push(s.y);
    }
    model.clamp(&mut ux, &mut uy);
    let (mut vx, mut vy) = (ux.clone(), uy.clone());

    // ePlace's smoothing schedule: gamma = 8 * bin * 10^(20/9 tau - 11/9),
    // so 80 bins wide at overflow 1 and 0.8 at overflow 0.1.
    let gamma_of = |tau: f64| {
        let k = 8.0 * 10f64.powf(20.0 / 9.0 * tau.clamp(0.0, 1.0) - 11.0 / 9.0);
        (k * grid.bw, k * grid.bh)
    };
    let of = |x: &[f64], y: &[f64]| overflow(prob, grid, &model.state(x, y), opts.target_density);
    let mut over = of(&ux, &uy);
    let mut gamma = gamma_of(over.ratio);
    // The first weights, before the wirelength gradient's scale is taken.
    reweigh(0, over.ratio, &mut model, &ux, &uy);
    let of = |x: &[f64], y: &[f64]| overflow(prob, grid, &model_state(n, x, y), opts.target_density);

    // lambda_0: the gradient norms balance (ePlace).
    let mut raw = model.raw(&vx, &vy, gamma);
    let norm1 = |a: &[f64], b: &[f64]| a.iter().chain(b).map(|v| v.abs()).sum::<f64>();
    let (wn, dn) = (norm1(&raw.wx, &raw.wy), norm1(&raw.dx, &raw.dy));
    let mut lambda = if wn > 0.0 && dn > 0.0 { wn / dn } else { 1.0 };
    let (mut gx, mut gy) = model.gradient(&raw, lambda);

    // A previous point to estimate the first step from (ours): a tenth of
    // a bin back along the gradient.
    let gmax = gx.iter().chain(&gy).fold(0.0f64, |m, v| m.max(v.abs()));
    let eta = if gmax > 0.0 { 0.1 * grid.bw.min(grid.bh) / gmax } else { 0.0 };
    let (mut px, mut py) = (axpy(&vx, -eta, &gx), axpy(&vy, -eta, &gy));
    model.clamp(&mut px, &mut py);
    let (pgx, pgy) = model.gradient(&model.raw(&px, &py, gamma), lambda);
    let ratio = |num: f64, den: f64, or: f64| if num > 0.0 && den > 0.0 && (num / den).is_finite() { num / den } else { or };
    let mut alpha = ratio(dist(&vx, &vy, &px, &py), dist(&gx, &gy, &pgx, &pgy), eta.max(1e-3));

    let mut a = 1.0f64;
    let mut hpwl = prob.hpwl(&model.state(&ux, &uy));
    let mut best = (over.ratio, hpwl, model.state(&ux, &uy));
    let mut rep = Report {
        stop: "max_iter",
        fillers,
        hpwl_ref: opts.hpwl_ref,
        ..Report::default()
    };
    let mut stall = (over.ratio, 0usize);
    // Wirelength when overflow first came within 1.5 times the target.
    let mut near: Option<f64> = None;
    for k in 0..opts.max_iter {
        if over.ratio <= opts.target_overflow {
            rep.stop = "overflow";
            break;
        }
        // One Nesterov step, the length backtracked.
        let a_new = (1.0 + (4.0 * a * a + 1.0).sqrt()) / 2.0;
        let (nux, nuy, nvx, nvy, nraw, next) = loop_step(&model, &ux, &uy, &vx, &vy, &gx, &gy, a, a_new, alpha, gamma, lambda, opts);
        (ux, uy, vx, vy, raw, alpha, a) = (nux, nuy, nvx, nvy, nraw, next, a_new);

        over = of(&ux, &uy);
        let st = model.state(&ux, &uy);
        let now = prob.hpwl(&st);
        if !now.is_finite() {
            rep.stop = "diverged";
            break;
        }
        if near.is_none() && over.ratio <= 1.5 * opts.target_overflow {
            near = Some(now);
        }
        if near.is_some_and(|h| now > opts.runaway * h) {
            rep.stop = "runaway";
            break;
        }
        if (over.ratio, now) < (best.0, best.1) {
            best = (over.ratio, now, st);
        }
        // lambda grows unless wirelength is rising faster than the
        // reference rate (the papers' mu, with a relative reference).
        let rise = (now - hpwl) / (opts.hpwl_ref * hpwl.max(f64::MIN_POSITIVE));
        lambda *= opts.mu_max.powf(1.0 - rise).clamp(opts.mu_min, opts.mu_max);
        hpwl = now;
        gamma = gamma_of(over.ratio);
        // New weights change the gradient at `v`: take it again.
        if reweigh(k + 1, over.ratio, &mut model, &ux, &uy) {
            raw = model.raw(&vx, &vy, gamma);
        }
        (gx, gy) = model.gradient(&raw, lambda);

        rep.iters = k + 1;
        if (k + 1) % 25 == 0 {
            rep.trace.push((k + 1, now, over.ratio));
        }
        if over.ratio < stall.0 - opts.stall_eps {
            stall = (over.ratio, k);
        } else if k - stall.1 >= opts.stall_window {
            rep.stop = "stall";
            break;
        }
    }
    let fin = best.2;
    rep.timing = tstats;
    rep.overflow = overflow(prob, grid, &fin, opts.target_density);
    rep.lambda = lambda;
    let mut count = vec![0.0; grid.len()];
    for i in (0..n).filter(|&i| prob.takes_site(i)) {
        for dy in prob.footprint(i) {
            count[grid.bin_at(fin.x[i], fin.y[i] + dy)] += 1.0;
        }
    }
    rep.max_util = (0..grid.len()).filter(|&b| count[b] > 0.0).map(|b| count[b] / grid.cap(b).max(1.0)).fold(0.0, f64::max);
    (fin, rep)
}

/// The real instances' part of a point (`Model::state` without a model).
fn model_state(n: usize, x: &[f64], y: &[f64]) -> PlacementState {
    PlacementState { x: x[..n].to_vec(), y: y[..n].to_vec() }
}

type Step = (Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>, Raw, f64);

/// The next `u`, `v`, the raw gradients there, and the step estimate for
/// the iteration after: the step shrinks until the Lipschitz estimate at
/// the new point supports it.
#[allow(clippy::too_many_arguments)]
fn loop_step(
    model: &Model,
    ux: &[f64],
    uy: &[f64],
    vx: &[f64],
    vy: &[f64],
    gx: &[f64],
    gy: &[f64],
    a: f64,
    a_new: f64,
    mut step: f64,
    gamma: (f64, f64),
    lambda: f64,
    opts: &Opts,
) -> Step {
    let momentum = (a - 1.0) / a_new;
    let mut tries = 0;
    loop {
        let (mut nux, mut nuy) = (axpy(vx, -step, gx), axpy(vy, -step, gy));
        model.clamp(&mut nux, &mut nuy);
        let mut nvx: Vec<f64> = nux.iter().zip(ux).map(|(n, o)| n + momentum * (n - o)).collect();
        let mut nvy: Vec<f64> = nuy.iter().zip(uy).map(|(n, o)| n + momentum * (n - o)).collect();
        model.clamp(&mut nvx, &mut nvy);
        let raw = model.raw(&nvx, &nvy, gamma);
        let (ngx, ngy) = model.gradient(&raw, lambda);
        let (moved, turned) = (dist(&nvx, &nvy, vx, vy), dist(&ngx, &ngy, gx, gy));
        let est = if moved > 0.0 && turned > 0.0 && (moved / turned).is_finite() { moved / turned } else { step };
        if est >= opts.backtrack_eps * step || tries >= opts.backtrack_max {
            return (nux, nuy, nvx, nvy, raw, est);
        }
        step = est;
        tries += 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::placer::{Inst, InstKind, PNet, Site};

    #[test]
    fn wa_gradient_matches_a_finite_difference() {
        let cs = [1.0, 4.0, 2.5, 9.0];
        let gamma = 1.5;
        let len = |cs: &[f64]| {
            let w = wa(cs.iter().copied(), gamma);
            w.t_hi / w.s_hi - w.t_lo / w.s_lo
        };
        let w = wa(cs.iter().copied(), gamma);
        for k in 0..cs.len() {
            let mut up = cs;
            up[k] += 1e-6;
            let fd = (len(&up) - len(&cs)) / 1e-6;
            assert!((fd - wa_grad(cs[k], &w, gamma)).abs() < 1e-5, "pin {k}: {fd} vs {}", wa_grad(cs[k], &w, gamma));
        }
        // WA length is below the true span and close to it.
        assert!(len(&cs) < 8.0 && len(&cs) > 7.0);
    }

    #[test]
    fn stacked_instances_spread_to_the_overflow_target() {
        // A 12 x 12 field of sites, 40 instances in a ring of two-pin
        // nets, all starting within a twentieth of a site of one point
        // (exactly coincident charges feel the same field and never part).
        let sites: Vec<Site> = (0..144)
            .map(|k| Site {
                x: (k % 12) as f64,
                y: (k / 12) as f64,
                name: String::new(),
            })
            .collect();
        let n = 40u32;
        let insts = vec![
            Inst {
                cells: Vec::new(),
                kind: InstKind::Slice
            };
            n as usize
        ];
        let nets = (0..n)
            .map(|i| PNet {
                pins: vec![Pin::Inst(i), Pin::Inst((i + 1) % n)],
                weight: 1.0,
            })
            .collect();
        let prob = Problem::from_parts(insts, nets, sites);
        let grid = BinGrid::new(&prob, 6);
        let start = PlacementState {
            x: (0..n).map(|i| 5.5 + 0.01 * f64::from(i % 7)).collect(),
            y: (0..n).map(|i| 5.5 + 0.01 * f64::from(i % 5)).collect(),
        };
        let before = overflow(&prob, &grid, &start, 1.0);
        assert!(before.ratio > 0.8, "{before:?}");
        let (st, rep) = run(&prob, &grid, start, &Opts::default(), None);
        assert_eq!(rep.stop, "overflow", "{rep:?}");
        assert!(rep.overflow.ratio <= 0.1, "{rep:?}");
        assert!(st.x.iter().chain(&st.y).all(|v| (-0.5..=11.5).contains(v)));
        // The ring stays a ring: neighbours end up near each other.
        assert!(prob.hpwl(&st) < 40.0 * 6.0, "{}", prob.hpwl(&st));
    }
}
