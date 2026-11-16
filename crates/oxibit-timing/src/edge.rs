//! Window location and sub-step edge fits.
//!
//! A slot's window is the phase range where it disagrees with the reference.
//! [`coarse`] finds it at grid resolution: the longest run of points at or
//! above [`ON`], a run ending only at two off points in a row, so an isolated
//! partial capture outside the window does not extend it. Each window edge is
//! then fitted with an error-function step,
//!
//! ```text
//! f(p) = l0 + (l1 - l0) * Phi(s * (p - E) / sigma)
//! ```
//!
//! `s` = +1 on the lower edge, -1 on the upper; `l0`, `l1` the medians of the
//! points on either side. `E` and `sigma` by a coarse grid then a fine one.
//! Windows narrower than [`PULSE_BELOW`] have too few points on their inside
//! for two separate fits, and get one pulse fit for both edges instead.
//!
//! Phase positions may carry a per-tap correction `e[tap] * step_ps` (MMCM
//! phase-tap spacing); [`NO_TAPS`] leaves them nominal.

use crate::phi;
use crate::sweep::Point;

/// Capture fraction at which a point counts as inside a window.
pub const ON: f64 = 0.25;
/// Windows narrower than this (ps) get a pulse fit.
pub const PULSE_BELOW: f64 = 150.0;
/// Nominal phase taps.
pub const NO_TAPS: [f64; 8] = [0.0; 8];

/// Phase of a point for a slot: reverse slots on the 10 ns frame (see
/// [`Point::rev_shift`]), with the per-tap correction.
pub fn pos(p: &Point, taps: &[f64; 8], slot: usize) -> f64 {
    nominal(p, slot) + taps[p.tap] * p.step_ps
}

/// Phase of a point for a slot without tap correction.
pub fn nominal(p: &Point, slot: usize) -> f64 {
    if slot < 4 { p.ps } else { p.ps + p.rev_shift }
}

/// A window at grid resolution: midpoints between its outermost on points
/// and the nearest off points beyond them.
pub fn coarse(pts: &[Point], slot: usize, lo: f64, hi: f64, taps: &[f64; 8]) -> Option<(f64, f64)> {
    // Scan in corrected phase order: tap corrections reorder points that
    // are closer together than the correction (Vernier grids interleave at a
    // few ps). Stable, so with nominal taps the order is the input's.
    let mut s: Vec<(f64, f64)> =
        pts.iter().filter(|p| lo <= nominal(p, slot) && nominal(p, slot) <= hi).map(|p| (pos(p, taps, slot), p.frac[slot])).collect();
    s.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut runs: Vec<Vec<f64>> = Vec::new();
    let mut cur: Vec<f64> = Vec::new();
    let mut off = 0;
    for &(q, f) in &s {
        if f >= ON {
            cur.push(q);
            off = 0;
        } else {
            off += 1;
            if off >= 2 && !cur.is_empty() {
                runs.push(std::mem::take(&mut cur));
            }
        }
    }
    if !cur.is_empty() {
        runs.push(cur);
    }
    // Longest by span; the first of equal spans, as a stable max would.
    let mut on: Option<&Vec<f64>> = None;
    for r in &runs {
        let span = r[r.len() - 1] - r[0];
        if on.is_none_or(|o| span > o[o.len() - 1] - o[0]) {
            on = Some(r);
        }
    }
    let on = on?;
    let (a, b) = (on[0], on[on.len() - 1]);
    let below = s.iter().filter(|&&(q, f)| q < a && f < ON).map(|&(q, _)| q).fold(None, |m: Option<f64>, q| Some(m.map_or(q, |m| m.max(q)))).unwrap_or(a);
    let above = s.iter().filter(|&&(q, f)| q > b && f < ON).map(|&(q, _)| q).fold(None, |m: Option<f64>, q| Some(m.map_or(q, |m| m.min(q)))).unwrap_or(b);
    Some(((a + below) / 2.0, (b + above) / 2.0))
}

/// One fitted edge (or, for a pulse, both).
#[derive(Debug, Clone, PartialEq)]
pub enum Fit {
    Edge { e: f64, sigma: f64, sse: f64, n: usize, l0: f64, l1: f64 },
    Pulse { e0: f64, e1: f64, sigma: f64, sse: f64, n: usize, l0: f64, l1: f64 },
}

fn median_upper(mut v: Vec<f64>) -> f64 {
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

/// Python's `range(int(a), int(b), step)`: bounds truncated toward zero, end exclusive.
fn int_range(a: f64, b: f64, step: usize) -> impl Iterator<Item = f64> {
    ((a.trunc() as i64)..(b.trunc() as i64)).step_by(step).map(|x| x as f64)
}

fn step_sse(obs: &[(f64, f64)], e: f64, sigma: f64, sgn: f64, l0: f64, l1: f64) -> f64 {
    obs.iter().map(|&(q, f)| (f - (l0 + (l1 - l0) * phi(sgn * (q - e) / sigma))).powi(2)).sum()
}

/// Fit the edge near `x`; `sgn` +1 for a lower (rising) edge, -1 for an upper.
/// `span`: how far inside the window the on level is taken.
pub fn fit_edge(pts: &[Point], slot: usize, x: f64, sgn: f64, taps: &[f64; 8], span: f64) -> Option<Fit> {
    const HALF: f64 = 220.0;
    let obs: Vec<(f64, f64)> = pts
        .iter()
        .map(|p| (pos(p, taps, slot), p.frac[slot]))
        .filter(|&(q, _)| -HALF <= sgn * (q - x) && sgn * (q - x) <= span + 30.0)
        .collect();
    let off: Vec<f64> = obs.iter().filter(|&&(q, _)| sgn * (q - x) < -60.0).map(|&(_, f)| f).collect();
    let on: Vec<f64> = obs.iter().filter(|&&(q, _)| 30.0 < sgn * (q - x) && sgn * (q - x) < span).map(|&(_, f)| f).collect();
    if off.is_empty() || on.is_empty() {
        return None;
    }
    let (l0, l1) = (median_upper(off), median_upper(on));
    if (l1 - l0).abs() < 0.2 {
        return None;
    }
    let mut best: Option<(f64, f64, f64)> = None;
    for e in int_range(x - 120.0, x + 121.0, 4) {
        for sigma in int_range(3.0, 80.0, 6) {
            let c = step_sse(&obs, e, sigma, sgn, l0, l1);
            if best.is_none_or(|b| c < b.0) {
                best = Some((c, e, sigma));
            }
        }
    }
    let (_, e0, s0) = best?;
    for de in -10..=10 {
        let e = e0 + de as f64 / 2.0;
        for ds in -6..=6 {
            let sigma = (s0 + ds as f64).max(1.0);
            let c = step_sse(&obs, e, sigma, sgn, l0, l1);
            if c < best.map_or(f64::INFINITY, |b| b.0) {
                best = Some((c, e, sigma));
            }
        }
    }
    let (sse, e, sigma) = best?;
    Some(Fit::Edge { e, sigma, sse, n: obs.len(), l0, l1 })
}

/// Fit a narrow window as one pulse,
/// `f = l0 + A (Phi((p - E0)/sigma) + Phi((E1 - p)/sigma) - 1)`, `A` by least squares.
pub fn fit_pulse(pts: &[Point], slot: usize, w: (f64, f64), taps: &[f64; 8]) -> Option<Fit> {
    let obs: Vec<(f64, f64)> = pts
        .iter()
        .map(|p| (pos(p, taps, slot), p.frac[slot]))
        .filter(|&(q, _)| w.0 - 220.0 <= q && q <= w.1 + 220.0)
        .collect();
    let off: Vec<f64> = obs.iter().filter(|&&(q, _)| q < w.0 - 60.0 || q > w.1 + 60.0).map(|&(_, f)| f).collect();
    if off.is_empty() {
        return None;
    }
    let l0 = median_upper(off);
    let mut best: Option<(f64, f64, f64, f64, f64)> = None;
    let mut g = vec![0.0; obs.len()];
    for e0 in int_range(w.0 - 50.0, w.0 + 51.0, 3) {
        for e1 in int_range(w.1 - 50.0, w.1 + 51.0, 3) {
            if e1 <= e0 {
                continue;
            }
            for sigma in int_range(3.0, 45.0, 4) {
                for (gi, &(q, _)) in g.iter_mut().zip(&obs) {
                    *gi = phi((q - e0) / sigma) + phi((e1 - q) / sigma) - 1.0;
                }
                let den: f64 = g.iter().map(|x| x * x).sum();
                if den < 1e-9 {
                    continue;
                }
                let a = g.iter().zip(&obs).map(|(x, &(_, f))| x * (f - l0)).sum::<f64>() / den;
                let c: f64 = g.iter().zip(&obs).map(|(x, &(_, f))| (f - l0 - a * x).powi(2)).sum();
                if best.is_none_or(|b| c < b.0) {
                    best = Some((c, e0, e1, sigma, a));
                }
            }
        }
    }
    let (sse, e0, e1, sigma, a) = best?;
    Some(Fit::Pulse { e0, e1, sigma, sse, n: obs.len(), l0, l1: l0 + a })
}

/// A slot's window: refined edges, the fits behind them, and the grid window.
#[derive(Debug, Clone, PartialEq)]
pub struct Window {
    pub lo: f64,
    pub hi: f64,
    pub lo_fit: Option<Fit>,
    pub hi_fit: Option<Fit>,
    pub grid: (f64, f64),
}

/// The eight slot windows of one design on one unit: forward slots 0..4 in
/// the first half period, reverse slots 4..8 in the second. `None` where a
/// slot never disagrees. An edge whose fit fails keeps its grid position.
pub fn windows(pts: &[Point], taps: &[f64; 8]) -> [Option<Window>; 8] {
    std::array::from_fn(|slot| {
        let (lo, hi) = if slot < 4 { (0.0, 5000.0) } else { (5000.0, 10000.0) };
        let w = coarse(pts, slot, lo, hi, taps)?;
        let width = w.1 - w.0;
        if width < PULSE_BELOW {
            let fit = fit_pulse(pts, slot, w, taps);
            return Some(match fit {
                Some(Fit::Pulse { e0, e1, .. }) => Window { lo: e0, hi: e1, lo_fit: fit.clone(), hi_fit: fit, grid: w },
                _ => Window { lo: w.0, hi: w.1, lo_fit: None, hi_fit: None, grid: w },
            });
        }
        let span = (width - 40.0).min(160.0);
        let a = fit_edge(pts, slot, w.0, 1.0, taps, span);
        let b = fit_edge(pts, slot, w.1, -1.0, taps, span);
        let at = |f: &Option<Fit>, grid: f64| match f {
            Some(Fit::Edge { e, .. }) => *e,
            _ => grid,
        };
        Some(Window { lo: at(&a, w.0), hi: at(&b, w.1), lo_fit: a, hi_fit: b, grid: w })
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A window from 400 to 900 ps with Gaussian edges, sampled on three grids.
    fn synthetic(sigma: f64) -> Vec<Point> {
        let mut pts = Vec::new();
        for step_ps in [125.0, 113.636_363_636_363_64, 156.25] {
            let mut k = 0u32;
            while (k as f64) * step_ps < 5000.0 {
                let ps = (k as f64 * step_ps * 10.0).round() / 10.0;
                let f = phi((ps - 400.0) / sigma) * phi((900.0 - ps) / sigma);
                let mut frac = [0.0; 8];
                frac[0] = f;
                pts.push(Point { ps, rev_shift: 0.0, tap: (k % 8) as usize, step_ps, frac });
                k += 1;
            }
        }
        crate::sweep::sort_points(&mut pts);
        pts
    }

    #[test]
    fn finds_window_and_edges() {
        let w = windows(&synthetic(12.0), &NO_TAPS)[0].clone().unwrap();
        assert!((w.lo - 400.0).abs() <= 2.0, "lo {}", w.lo);
        assert!((w.hi - 900.0).abs() <= 2.0, "hi {}", w.hi);
        assert!(matches!(w.lo_fit, Some(Fit::Edge { .. })));
    }

    #[test]
    fn isolated_blip_does_not_extend_window() {
        let mut pts = synthetic(5.0);
        // one partial capture 350 ps above the window, off points around it
        for p in pts.iter_mut().filter(|p| (p.ps - 1250.0).abs() < 1.0) {
            p.frac[0] = 0.4;
        }
        let w = coarse(&pts, 0, 0.0, 5000.0, &NO_TAPS).unwrap();
        assert!(w.1 < 1000.0, "{w:?}");
    }

    /// A tap correction that reorders points must not split the window.
    #[test]
    fn corrected_order() {
        let mut taps = NO_TAPS;
        taps[3] = 0.12;
        let shifted: Vec<Point> = synthetic(12.0)
            .into_iter()
            .map(|mut p| {
                // sample the window at the corrected position, so the correction is exact
                let q = pos(&p, &taps, 0);
                p.frac[0] = phi((q - 400.0) / 12.0) * phi((900.0 - q) / 12.0);
                p
            })
            .collect();
        let w = windows(&shifted, &taps)[0].clone().unwrap();
        assert!((w.lo - 400.0).abs() <= 2.0 && (w.hi - 900.0).abs() <= 2.0, "{w:?}");
    }

    #[test]
    fn empty_slot() {
        assert!(windows(&synthetic(10.0), &NO_TAPS)[1].is_none());
    }
}
