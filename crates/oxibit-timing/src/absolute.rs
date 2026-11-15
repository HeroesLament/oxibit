//! Model-free path delays from mirrored windows, and unit comparison.
//!
//! A design pairs forward slot `k` (launch on lclk, capture on cclk) with
//! reverse slot `k + 4` (the same structure mirrored). The forward window
//! edge sits at `d - K`, the reverse at `T - d' - K`, with `K` the unknown
//! offset between the clocks, so `(T + f - r) / 2` is the mean of `d` and
//! `d'` with `K` gone.
//!
//! Which window edge is the transition of interest depends on the design
//! (one edge per window moves with the slot's path, the other is shared with
//! the reference flop). [`side`] votes for the edge that coincides across
//! slots; it votes over every unit together, so all units use the same edge.

use std::collections::BTreeMap;

use crate::edge::Window;

/// One period, ps (100 MHz).
pub const PERIOD_PS: f64 = 10_000.0;
/// Edges this close (ps) count as the same edge when voting.
pub const COINCIDE_PS: f64 = 45.0;

/// The window edges of one design on one unit: forward slots, reverse slots.
pub type Edges = ([Option<(f64, f64)>; 4], [Option<(f64, f64)>; 4]);

pub fn edges(w: &[Option<Window>; 8]) -> Edges {
    let at = |i: usize| w[i].as_ref().map(|w| (w.lo, w.hi));
    (std::array::from_fn(at), std::array::from_fn(|i| at(i + 4)))
}

fn pick(e: (f64, f64), side: usize) -> f64 {
    if side == 0 { e.0 } else { e.1 }
}

/// The shared (reference) edge side, 0 lower or 1 upper: the side whose edges
/// coincide most across slots, summed over units. Ties go to the lower.
pub fn side(units: &[&[Option<(f64, f64)>; 4]]) -> usize {
    let score = |s: usize| -> i64 {
        units
            .iter()
            .map(|ws| {
                let e: Vec<f64> = ws.iter().flatten().map(|&w| pick(w, s)).collect();
                let close = e.iter().flat_map(|x| e.iter().map(move |y| (x - y).abs() <= COINCIDE_PS)).filter(|&c| c).count();
                close as i64 - e.len() as i64
            })
            .sum()
    };
    if score(1) > score(0) { 1 } else { 0 }
}

/// The reference edge (upper median of the shared side) and each slot's own
/// edge (the other side; the reference where the slot has no window).
pub fn split(ws: &[Option<(f64, f64)>; 4], side: usize) -> Option<(f64, [f64; 4])> {
    let mut e: Vec<f64> = ws.iter().flatten().map(|&w| pick(w, side)).collect();
    if e.is_empty() {
        return None;
    }
    e.sort_by(f64::total_cmp);
    let r = e[e.len() / 2];
    Some((r, std::array::from_fn(|i| ws[i].map_or(r, |w| pick(w, 1 - side)))))
}

/// Path delays of one design on `unit`: `[reference, slot 0..4]`, ps.
/// `all` holds the same design on every unit that votes for the edge side.
pub fn absolute(all: &BTreeMap<String, Edges>, unit: &str) -> Result<[f64; 5], String> {
    let fwd: Vec<_> = all.values().map(|e| &e.0).collect();
    let rev: Vec<_> = all.values().map(|e| &e.1).collect();
    let (jf, jr) = (side(&fwd), side(&rev));
    let mine = all.get(unit).ok_or_else(|| format!("no windows for {unit}"))?;
    let (rf, f) = split(&mine.0, jf).ok_or_else(|| format!("{unit}: no forward window"))?;
    let (rr, r) = split(&mine.1, jr).ok_or_else(|| format!("{unit}: no reverse window"))?;
    let d = |a: f64, b: f64| (PERIOD_PS + a - b) / 2.0;
    Ok([d(rf, rr), d(f[0], r[0]), d(f[1], r[1]), d(f[2], r[2]), d(f[3], r[3])])
}

/// Summary of unit-to-unit differences, ps.
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct Stats {
    pub n: usize,
    pub mean: f64,
    pub median: f64,
    /// Population standard deviation.
    pub sd: f64,
    pub min: f64,
    pub max: f64,
}

pub fn stats(d: &[f64]) -> Option<Stats> {
    if d.is_empty() {
        return None;
    }
    let n = d.len();
    let mean = d.iter().sum::<f64>() / n as f64;
    let mut s = d.to_vec();
    s.sort_by(f64::total_cmp);
    let median = if n % 2 == 1 { s[n / 2] } else { (s[n / 2 - 1] + s[n / 2]) / 2.0 };
    let sd = (d.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / n as f64).sqrt();
    Some(Stats { n, mean, median, sd, min: s[0], max: s[n - 1] })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn side_votes_for_coinciding_edge() {
        // lower edges shared (reference), upper edges per slot
        let ws = [Some((400.0, 900.0)), Some((410.0, 700.0)), Some((395.0, 1200.0)), None];
        assert_eq!(side(&[&ws]), 0);
        let flipped = [Some((100.0, 900.0)), Some((300.0, 905.0)), Some((600.0, 890.0)), None];
        assert_eq!(side(&[&flipped]), 1);
    }

    #[test]
    fn split_and_absolute() {
        let ws = [Some((400.0, 900.0)), Some((410.0, 700.0)), None, Some((395.0, 1200.0))];
        let (r, own) = split(&ws, 0).unwrap();
        assert_eq!(r, 400.0);
        assert_eq!(own, [900.0, 700.0, 400.0, 1200.0]);

        let fwd = [Some((400.0, 900.0)), Some((400.0, 700.0)), Some((400.0, 800.0)), Some((400.0, 1000.0))];
        let rev = [Some((9000.0, 9600.0)), Some((8800.0, 9600.0)), Some((8900.0, 9600.0)), Some((8700.0, 9600.0))];
        let all = BTreeMap::from([("u".to_string(), (fwd, rev))]);
        let d = absolute(&all, "u").unwrap();
        assert_eq!(d, [400.0, 950.0, 950.0, 950.0, 1150.0]);
    }

    #[test]
    fn stats_match_python() {
        let s = stats(&[1.0, 2.0, 4.0, 7.0]).unwrap();
        assert_eq!((s.n, s.mean, s.median, s.min, s.max), (4, 3.5, 3.0, 1.0, 7.0));
        assert!((s.sd - 2.291_287_847_477_92).abs() < 1e-12);
    }
}
