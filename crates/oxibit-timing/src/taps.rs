//! MMCM phase-tap spacing.
//!
//! The capture phase moves in 1/8-VCO steps through the MMCM's phase mux, so
//! step `k` uses tap `k mod 8`. If the taps are unevenly spaced, every edge
//! position carries an error that depends on the tap it was sampled on. To
//! see it, the same design is swept with the launch clock moved `t` taps
//! (runner `TOBS_L_TAP`): a point at capture step `k` then sits at nominal
//! relative phase `u = (k - t) * step` and true offset
//! `(a1[k mod 8] - a0[t]) * step`, `a1` the capture output's tap errors and
//! `a0` the launch output's (`a0[0] = 0`, `a1` summing to 0).
//!
//! The fit alternates: every edge fitted on the current tap errors
//! (`D`, `sigma`, by grid), then each tap error line-searched with the edges
//! held. All points take part, including the 0/1 points that only bracket
//! an edge; sharp edges (sigma of a few ps) have almost no partial points.

use std::fs;
use std::path::Path;
use std::thread;

use crate::edge::{NO_TAPS, coarse};
use crate::phi;
use crate::sweep::{Point, Sweep, points};

/// A capture-fraction sample with its launch tap.
#[derive(Debug, Clone)]
pub struct TapPoint {
    /// Nominal phase relative to the launch, `(step - t) * step_ps` mod period.
    pub u: f64,
    /// Capture tap, `step mod 8`.
    pub j: usize,
    /// Launch tap.
    pub t: usize,
    pub step_ps: f64,
    pub frac: [f64; 8],
}

/// Whether a file name is a tap sweep of `tag` on `unit`: `tap_<tag>_<unit>_lt*_*.json`.
pub fn is_tap_sweep_of(name: &str, tag: &str, unit: &str) -> bool {
    let prefix = format!("tap_{tag}_{unit}_lt");
    match name.strip_prefix(&prefix) {
        Some(rest) => rest.ends_with(".json") && rest.contains('_'),
        None => false,
    }
}

/// Every tap sweep of `tag` on `unit`, sorted by relative phase. Sweeps
/// whose design ID did not match are skipped.
pub fn load(dir: &Path, tag: &str, unit: &str, split: bool) -> Result<Vec<TapPoint>, String> {
    let entries = fs::read_dir(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    let mut names: Vec<String> = entries
        .filter_map(|e| e.ok())
        .filter_map(|e| e.file_name().into_string().ok())
        .filter(|n| is_tap_sweep_of(n, tag, unit))
        .collect();
    names.sort();
    let mut out = Vec::new();
    for name in &names {
        let path = dir.join(name);
        let text = fs::read_to_string(&path).map_err(|e| format!("{}: {e}", path.display()))?;
        let sweep: Sweep = serde_json::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))?;
        if !sweep.id_ok {
            continue;
        }
        let t = sweep.l_tap;
        for (p, row) in points(&sweep, split).into_iter().zip(sweep.rows.iter().filter(|r| r.locked && r.ncyc > 0)) {
            let u = ((row.step as f64 - t as f64) * sweep.step_ps).rem_euclid(crate::absolute::PERIOD_PS);
            out.push(TapPoint { u, j: p.tap, t, step_ps: p.step_ps, frac: p.frac });
        }
    }
    out.sort_by(|a, b| {
        a.u.total_cmp(&b.u)
            .then(a.j.cmp(&b.j))
            .then(a.step_ps.total_cmp(&b.step_ps))
            .then_with(|| a.frac.iter().zip(&b.frac).map(|(x, y)| x.total_cmp(y)).find(|o| o.is_ne()).unwrap_or(std::cmp::Ordering::Equal))
            .then(a.t.cmp(&b.t))
    });
    Ok(out)
}

/// The points around one window edge, with its levels.
#[derive(Debug, Clone)]
pub struct EdgeData {
    pub name: String,
    /// Grid position of the edge.
    pub x: f64,
    /// +1 lower edge, -1 upper.
    pub sgn: f64,
    pub l0: f64,
    pub l1: f64,
    pub u: Vec<f64>,
    pub j: Vec<usize>,
    pub t: Vec<usize>,
    pub s: Vec<f64>,
    pub f: Vec<f64>,
}

/// The edges a tap fit can use: windows at least 150 ps wide, edges whose
/// two sides differ by at least 0.3 in capture fraction; points within 120
/// ps of the grid edge.
pub fn edges(pts: &[TapPoint], name: &str) -> Vec<EdgeData> {
    let grid: Vec<Point> = pts.iter().map(|p| Point { ps: p.u, rev_shift: 0.0, tap: p.j, step_ps: p.step_ps, frac: p.frac }).collect();
    let mut out = Vec::new();
    for slot in 0..8 {
        let (lo, hi) = if slot < 4 { (0.0, 5000.0) } else { (5000.0, 10000.0) };
        let Some(w) = coarse(&grid, slot, lo, hi, &NO_TAPS) else { continue };
        if w.1 - w.0 < 150.0 {
            continue;
        }
        for (x, sgn) in [(w.0, 1.0), (w.1, -1.0)] {
            let near: Vec<&TapPoint> = pts.iter().filter(|p| (p.u - x).abs() <= 200.0).collect();
            let mut off: Vec<f64> = near.iter().filter(|p| sgn * (p.u - x) < -60.0).map(|p| p.frac[slot]).collect();
            let mut on: Vec<f64> =
                near.iter().filter(|p| 60.0 < sgn * (p.u - x) && sgn * (p.u - x) < 140.0).map(|p| p.frac[slot]).collect();
            if off.is_empty() || on.is_empty() {
                continue;
            }
            off.sort_by(f64::total_cmp);
            on.sort_by(f64::total_cmp);
            let (l0, l1) = (off[off.len() / 2], on[on.len() / 2]);
            if (l1 - l0).abs() < 0.3 {
                continue;
            }
            let p: Vec<&&TapPoint> = near.iter().filter(|p| (p.u - x).abs() <= 120.0).collect();
            out.push(EdgeData {
                name: format!("{name} c{slot} {}", if sgn > 0.0 { "lo" } else { "hi" }),
                x,
                sgn,
                l0,
                l1,
                u: p.iter().map(|p| p.u).collect(),
                j: p.iter().map(|p| p.j).collect(),
                t: p.iter().map(|p| p.t).collect(),
                s: p.iter().map(|p| p.step_ps).collect(),
                f: p.iter().map(|p| p.frac[slot]).collect(),
            });
        }
    }
    out
}

/// Edge widths tried, ps.
pub const SIGMAS: [f64; 11] = [1.0, 2.0, 3.0, 4.0, 6.0, 8.0, 11.0, 15.0, 20.0, 27.0, 36.0];

fn edge_sse(e: &EdgeData, a1: &[f64; 8], a0: &[f64; 8], d: f64, sigma: f64) -> f64 {
    let mut c = 0.0;
    for k in 0..e.u.len() {
        let xq = e.u[k] + (a1[e.j[k]] - a0[e.t[k]]) * e.s[k];
        let m = e.l0 + (e.l1 - e.l0) * phi(e.sgn * (xq - d) / sigma);
        c += (m - e.f[k]).powi(2);
    }
    c
}

/// Best `(D, sigma)` of one edge: `D` on a 1 ps grid within 45 ps of the
/// grid edge, sigma from [`SIGMAS`].
pub fn fit_edge(e: &EdgeData, a1: &[f64; 8], a0: &[f64; 8]) -> (f64, f64) {
    let mut best = (f64::INFINITY, e.x, SIGMAS[0]);
    for k in 0..=90 {
        let d = e.x - 45.0 + k as f64;
        for &sigma in &SIGMAS {
            let c = edge_sse(e, a1, a0, d, sigma);
            if c < best.0 {
                best = (c, d, sigma);
            }
        }
    }
    (best.1, best.2)
}

fn fit_all(edges: &[EdgeData], a1: &[f64; 8], a0: &[f64; 8]) -> Vec<(f64, f64)> {
    let n = thread::available_parallelism().map_or(1, |n| n.get()).min(8);
    let chunk = edges.len().div_ceil(n).max(1);
    thread::scope(|s| {
        let hs: Vec<_> = edges.chunks(chunk).map(|c| s.spawn(move || c.iter().map(|e| fit_edge(e, a1, a0)).collect::<Vec<_>>())).collect();
        hs.into_iter().flat_map(|h| h.join().expect("edge fit thread")).collect()
    })
}

fn total(edges: &[EdgeData], a1: &[f64; 8], a0: &[f64; 8], fits: &[(f64, f64)]) -> f64 {
    edges.iter().zip(fits).map(|(e, &(d, s))| edge_sse(e, a1, a0, d, s)).sum()
}

/// Result of a tap fit.
#[derive(Debug, Clone, serde::Serialize)]
pub struct TapFit {
    /// Capture-output tap errors, fraction of one step, summing to 0.
    pub a1: [f64; 8],
    /// Launch-output tap errors relative to tap 0 (`a0[0] = 0`).
    pub a0: [f64; 8],
    pub sse_start: f64,
    pub sse: f64,
    pub edges: usize,
    pub points: usize,
    /// Fitted edge widths, ps, one per edge.
    pub sigmas: Vec<f64>,
    /// Each edge: name, grid position, points used.
    pub edge_points: Vec<(String, f64, usize)>,
}

/// Fit the tap errors. `iters` rounds of: line-search every tap error over
/// -0.3..0.3 step in 0.01 steps with the edges held, centre `a1`, refit the
/// edges.
pub fn fit(edges: &[EdgeData], iters: usize) -> TapFit {
    let mut a1 = [0.0; 8];
    let mut a0 = [0.0; 8];
    let mut fits = fit_all(edges, &a1, &a0);
    let sse_start = total(edges, &a1, &a0, &fits);
    for _ in 0..iters {
        for which in 0..15 {
            let mut best: Option<(f64, [f64; 8], [f64; 8])> = None;
            for k in 0..=60 {
                let g = -0.3 + 0.01 * k as f64;
                let (mut b1, mut b0) = (a1, a0);
                if which < 8 {
                    b1[which] = g;
                } else {
                    b0[which - 7] = g;
                }
                let c = total(edges, &b1, &b0, &fits);
                if best.as_ref().is_none_or(|b| c < b.0) {
                    best = Some((c, b1, b0));
                }
            }
            if let Some((_, b1, b0)) = best {
                (a1, a0) = (b1, b0);
            }
        }
        let mean = a1.iter().sum::<f64>() / 8.0;
        a1.iter_mut().for_each(|x| *x -= mean);
        fits = fit_all(edges, &a1, &a0);
    }
    TapFit {
        a1,
        a0,
        sse_start,
        sse: total(edges, &a1, &a0, &fits),
        edges: edges.len(),
        points: edges.iter().map(|e| e.u.len()).sum(),
        sigmas: fits.iter().map(|f| f.1).collect(),
        edge_points: edges.iter().map(|e| (e.name.clone(), e.x, e.u.len())).collect(),
    }
}

/// Pearson correlation.
pub fn corr(x: &[f64; 8], y: &[f64; 8]) -> f64 {
    let mx = x.iter().sum::<f64>() / 8.0;
    let my = y.iter().sum::<f64>() / 8.0;
    let (mut sxy, mut sxx, mut syy) = (0.0, 0.0, 0.0);
    for i in 0..8 {
        sxy += (x[i] - mx) * (y[i] - my);
        sxx += (x[i] - mx).powi(2);
        syy += (y[i] - my).powi(2);
    }
    sxy / (sxx * syy).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tap_sweep_names() {
        assert!(is_tap_sweep_of("tap_t2p_dr-b_lt3_v11.json", "t2p", "dr-b"));
        assert!(!is_tap_sweep_of("tobs_t2p_dr-b_r1.json", "t2p", "dr-b"));
        assert!(!is_tap_sweep_of("tap_t2p_dr-b_s2_lt3_r1.json", "t2p", "dr-b"));
    }

    const A1: [f64; 8] = [0.06, -0.04, 0.0, -0.10, 0.02, 0.07, -0.05, 0.04];
    const A0: [f64; 8] = [0.0, -0.03, 0.02, -0.06, 0.05, 0.01, -0.02, 0.04];

    /// Three windows sampled through known tap errors on all five grids and
    /// all eight launch taps, edges of width `sigma`.
    fn synthetic(sigma: f64) -> Vec<EdgeData> {
        synthetic_with(sigma, &A1, &A0)
    }

    fn synthetic_with(sigma: f64, a1: &[f64; 8], a0: &[f64; 8]) -> Vec<EdgeData> {
        synthetic_drift(sigma, a1, a0, 0.0, 0.0, 1)
    }

    /// Like `synthetic_with`, each sweep (multiplier, launch tap) shifted by
    /// `drift_ps` per sweep in run order plus Gaussian `jitter_ps`.
    fn synthetic_drift(sigma: f64, a1: &[f64; 8], a0: &[f64; 8], drift_ps: f64, jitter_ps: f64, seed: u64) -> Vec<EdgeData> {
        let mut rng = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1;
        let mut gauss = move || {
            let mut u = || {
                rng ^= rng << 13;
                rng ^= rng >> 7;
                rng ^= rng << 17;
                (rng >> 11) as f64 / (1u64 << 53) as f64
            };
            let (x, y) = (u().max(1e-300), u());
            (-2.0 * x.ln()).sqrt() * (2.0 * std::f64::consts::PI * y).cos()
        };
        let mut order = 0.0;
        let mut pts = Vec::new();
        for (step_ps, mult) in [(125.0, 10u32), (156.25, 8), (138.888_888_888_888_9, 9), (113.636_363_636_363_64, 11), (104.166_666_666_666_67, 12)] {
            for t in 0..8usize {
                let shift = drift_ps * order + jitter_ps * gauss();
                order += 1.0;
                for k in 0..(8 * mult) {
                    let u = ((k as f64 - t as f64) * step_ps).rem_euclid(10_000.0);
                    let j = (k % 8) as usize;
                    let true_pos = u + (a1[j] - a0[t]) * step_ps + shift;
                    let mut frac = [0.0; 8];
                    for (slot, (lo, hi)) in [(0, (400.0, 900.0)), (1, (430.0, 1210.0)), (2, (380.0, 1030.0))] {
                        frac[slot] = phi((true_pos - lo) / sigma) * phi((hi - true_pos) / sigma);
                    }
                    pts.push(TapPoint { u, j, t, step_ps, frac });
                }
            }
        }
        pts.sort_by(|a, b| a.u.total_cmp(&b.u).then(a.j.cmp(&b.j)).then(a.t.cmp(&b.t)));
        edges(&pts, "synthetic")
    }

    fn worst_error(r: &TapFit) -> f64 {
        let centre = |a: [f64; 8]| a.map(|x| x - a.iter().sum::<f64>() / 8.0);
        let (a1, a0, t1, t0) = (centre(r.a1), centre(r.a0), centre(A1), centre(A0));
        (0..8).map(|i| (a1[i] - t1[i]).abs().max((a0[i] - t0[i]).abs())).fold(0.0, f64::max)
    }

    /// Diagnostic: recovery error by edge width and rounds. `cargo test -- --ignored --nocapture`
    #[test]
    #[ignore]
    fn recovery_table() {
        for sigma in [1.0, 3.0, 6.0, 10.0, 20.0] {
            let e = synthetic(sigma);
            for iters in [10, 40] {
                let r = fit(&e, iters);
                println!("sigma {sigma:4} iters {iters:3} edges {} worst |err| {:.4} step  sse {:.5} -> {:.5}", r.edges, worst_error(&r), r.sse_start, r.sse);
            }
        }
        // No tap errors, sweeps shifted by drift and jitter: whatever the fit
        // reports is the method's own artifact.
        for (sigma, drift, jitter) in [(2.0, 0.0, 2.0), (2.0, 0.1, 2.0), (2.0, 0.0, 4.0), (10.0, 0.0, 2.0)] {
            for seed in 1..=3 {
                let r = fit(&synthetic_drift(sigma, &[0.0; 8], &[0.0; 8], drift, jitter, seed), 40);
                let a0c = r.a0.map(|x| x - r.a0.iter().sum::<f64>() / 8.0);
                let max = r.a1.iter().chain(&a0c).fold(0.0f64, |m, x| m.max(x.abs()));
                println!("null sigma {sigma:4} drift {drift} ps/sweep jitter {jitter} ps seed {seed}: max |a| {:.1} ps  corr(a1, a0) {:.3}", max * 125.0, corr(&r.a1, &a0c));
            }
        }
        // No tap errors at all: whatever the fit reports is the method's own artifact.
        for sigma in [1.0, 2.0, 3.0, 10.0] {
            let r = fit(&synthetic_with(sigma, &[0.0; 8], &[0.0; 8]), 40);
            let a0c = r.a0.map(|x| x - r.a0.iter().sum::<f64>() / 8.0);
            let max = r.a1.iter().chain(&a0c).fold(0.0f64, |m, x| m.max(x.abs()));
            println!("zero truth sigma {sigma:4}: max |a| {:.4} step ({:.1} ps)  corr(a1, a0) {:.3}  a1 {:?}", max, max * 125.0, corr(&r.a1, &a0c), r.a1.map(|x| (x * 1000.0).round() / 1000.0));
        }
    }

    /// Diagnostic: the sorted points of a real tap sweep set near one edge.
    /// `OXT_DIR=... cargo test dump_real_points -- --ignored --nocapture`
    #[test]
    #[ignore]
    fn dump_real_points() {
        let dir = std::env::var("OXT_DIR").expect("OXT_DIR");
        let pts = load(Path::new(&dir), "t2p", "dr-a", true).unwrap();
        for p in pts.iter().filter(|p| (200.0..=340.0).contains(&p.u)) {
            println!("{:?} {} {:.2} {} {:.3}", p.u, p.j, p.step_ps, p.t, p.frac[0]);
        }
    }

    /// Edges sampled through known tap errors come back out of the fit.
    #[test]
    fn recovers_injected_taps() {
        let e = synthetic(10.0);
        assert_eq!(e.len(), 6);
        let r = fit(&e, 40);
        assert!(worst_error(&r) <= 0.02, "worst {}: {r:?}", worst_error(&r));
    }
}
