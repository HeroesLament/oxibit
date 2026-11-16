//! Sweep files, as written by the fabfarm runner (`tobs_run.exs`).
//!
//! One file is one sweep: a design at one VCO multiplier, the capture phase
//! stepped over a period. Files of one design on one unit are named
//! `tobs_<tag>_<unit>_<run>.json`, `<run>` being `r<n>` (the build's own
//! multiplier) or `v<m>` (a Vernier multiplier).

use std::fs;
use std::path::Path;

use serde::Deserialize;

/// One sweep file. Fields the analysis does not use are ignored.
#[derive(Debug, Clone, Deserialize)]
pub struct Sweep {
    pub id_ok: bool,
    pub mult: u32,
    /// One phase step, ps.
    pub step_ps: f64,
    /// Launch clock moved this many phase taps for the sweep (runner
    /// `TOBS_L_TAP`); 0 in ordinary sweeps.
    #[serde(default)]
    pub l_tap: usize,
    /// Launch and capture period, ps (runner `TOBS_FCLK`); 10 ns when absent.
    #[serde(default = "default_period")]
    pub period_ps: f64,
    pub rows: Vec<Row>,
}

fn default_period() -> f64 {
    crate::absolute::PERIOD_PS
}

/// One phase step.
#[derive(Debug, Clone, Deserialize)]
pub struct Row {
    pub step: u32,
    /// Capture phase, ps.
    pub ps: f64,
    pub locked: bool,
    /// Capture-clock cycles counted (forward slots 0..4).
    pub ncyc: u64,
    /// Launch-clock cycles counted (reverse slots 4..8); 0 on designs without.
    #[serde(default)]
    pub ncyc_l: u64,
    /// Mismatch counts of the eight slots.
    pub n: [u64; 8],
}

/// A capture-fraction sample: one step of one sweep.
#[derive(Debug, Clone, PartialEq)]
pub struct Point {
    pub ps: f64,
    /// Added to the phase of reverse slots (4..8): `PERIOD_PS - period`. A
    /// reverse edge sits at `T - d' - K`, so sweeps at other periods are
    /// brought onto the 10 ns frame before they are merged; forward edges
    /// (`d - K`) do not depend on the period.
    pub rev_shift: f64,
    /// MMCM phase tap, `step mod 8`.
    pub tap: usize,
    pub step_ps: f64,
    /// Fraction of cycles in which each slot disagreed, clamped to 1.
    pub frac: [f64; 8],
}

/// Whether a file name is a sweep of `tag` on `unit`: `tobs_<tag>_<unit>_[rv]*.json`.
pub fn is_sweep_of(name: &str, tag: &str, unit: &str) -> bool {
    let prefix = format!("tobs_{tag}_{unit}_");
    match name.strip_prefix(&prefix) {
        Some(rest) => rest.ends_with(".json") && matches!(rest.as_bytes().first(), Some(b'r' | b'v')),
        None => false,
    }
}

/// The capture-fraction points of a sweep. `split` designs count each
/// transition direction in a counter of its own, so their fractions are
/// doubled (each counter sees at most half the cycles). Unlocked steps and
/// steps with no cycles are dropped.
pub fn points(sweep: &Sweep, split: bool) -> Vec<Point> {
    let k = if split { 2.0 } else { 1.0 };
    let mut out = Vec::with_capacity(sweep.rows.len());
    for row in &sweep.rows {
        if !row.locked || row.ncyc == 0 {
            continue;
        }
        let fwd = row.ncyc as f64;
        let rev = if row.ncyc_l == 0 { 1.0 } else { row.ncyc_l as f64 };
        let mut frac = [0.0; 8];
        for (i, f) in frac.iter_mut().enumerate() {
            let cycles = if i < 4 { fwd } else { rev };
            *f = (k * row.n[i] as f64 / cycles).min(1.0);
        }
        out.push(Point {
            ps: row.ps,
            rev_shift: crate::absolute::PERIOD_PS - sweep.period_ps,
            tap: (row.step % 8) as usize,
            step_ps: sweep.step_ps,
            frac,
        });
    }
    out
}

/// Every sweep of `tag` on `unit` in `dir`, merged and sorted by phase.
pub fn load(dir: &Path, tag: &str, unit: &str, split: bool) -> Result<Vec<Point>, String> {
    let entries = fs::read_dir(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    let mut names: Vec<String> = entries
        .filter_map(|e| e.ok())
        .filter_map(|e| e.file_name().into_string().ok())
        .filter(|n| is_sweep_of(n, tag, unit))
        .collect();
    names.sort();
    let mut pts = Vec::new();
    for name in &names {
        let path = dir.join(name);
        let text = fs::read_to_string(&path).map_err(|e| format!("{}: {e}", path.display()))?;
        let sweep: Sweep = serde_json::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))?;
        pts.extend(points(&sweep, split));
    }
    sort_points(&mut pts);
    Ok(pts)
}

/// Phase order; ties by tap, step, then fractions, so the order (and with it
/// the window scan) does not depend on file order.
pub fn sort_points(pts: &mut [Point]) {
    pts.sort_by(|a, b| {
        a.ps.total_cmp(&b.ps)
            .then(a.rev_shift.total_cmp(&b.rev_shift))
            .then(a.tap.cmp(&b.tap))
            .then(a.step_ps.total_cmp(&b.step_ps))
            .then_with(|| {
                a.frac.iter().zip(&b.frac).map(|(x, y)| x.total_cmp(y)).find(|o| o.is_ne()).unwrap_or(std::cmp::Ordering::Equal)
            })
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Coinciding grid points must coincide exactly (see Cargo.toml).
    #[test]
    fn parses_exactly() {
        let s: Sweep = serde_json::from_str(r#"{"id_ok":true,"mult":12,"step_ps":104.16666666666667,"rows":[]}"#).unwrap();
        assert_eq!(3.0 * s.step_ps, 312.5);
    }

    /// Reverse slots of a sweep at another period land on the 10 ns frame.
    #[test]
    fn period_frame() {
        let s: Sweep = serde_json::from_str(r#"{"id_ok":true,"mult":10,"step_ps":137.5,"period_ps":11000.0,"rows":[{"step":76,"ps":10450.0,"locked":true,"ncyc":100,"ncyc_l":100,"n":[0,0,0,0,100,0,0,0]}]}"#).unwrap();
        let p = &points(&s, false)[0];
        assert_eq!(crate::edge::nominal(p, 0), 10450.0);
        assert_eq!(crate::edge::nominal(p, 4), 9450.0);
        let old: Sweep = serde_json::from_str(r#"{"id_ok":true,"mult":10,"step_ps":125.0,"rows":[]}"#).unwrap();
        assert_eq!(old.period_ps, 10_000.0);
    }

    #[test]
    fn sweep_names() {
        assert!(is_sweep_of("tobs_t2c_dr-b_r1.json", "t2c", "dr-b"));
        assert!(is_sweep_of("tobs_t2c_dr-b_v11.json", "t2c", "dr-b"));
        assert!(!is_sweep_of("tobs_t2c_dr-b_s2_r1.json", "t2c", "dr-b"));
        assert!(is_sweep_of("tobs_t2c_dr-b_s2_r1.json", "t2c", "dr-b_s2"));
        assert!(!is_sweep_of("tobs_t2c_dr-b_r1.log", "t2c", "dr-b"));
        assert!(!is_sweep_of("tap_t2c_dr-b_lt0_r1.json", "t2c", "dr-b"));
    }

    #[test]
    fn split_fractions_double_and_clamp() {
        let row = |n: [u64; 8]| Row { step: 9, ps: 125.0, locked: true, ncyc: 100, ncyc_l: 50, n };
        let sweep = Sweep { id_ok: true, mult: 10, step_ps: 125.0, l_tap: 0, period_ps: 10_000.0, rows: vec![row([30, 60, 0, 0, 10, 30, 0, 0])] };
        let p = &points(&sweep, true)[0];
        assert_eq!(p.tap, 1);
        assert_eq!(&p.frac[..2], &[0.6, 1.0]);
        assert_eq!(&p.frac[4..6], &[0.4, 1.0]);
        assert_eq!(points(&sweep, false)[0].frac[0], 0.3);
    }
}
