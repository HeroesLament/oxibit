//! The bound-to-bound quadratic wirelength model and the starting
//! placement it gives.
//!
//! B2B (Spindler, Schlichtmann, Johannes, "Kraftwerk2", TCAD 2008): per
//! axis, a net of `p` pins becomes two-pin edges between each pin and the
//! net's two boundary pins (lowest and highest coordinate), each weighted
//! `2 / ((p - 1) * length)`. At the placement it was built from, the
//! quadratic cost equals the net's half-perimeter length, so re-building
//! and re-solving walks towards a linear-wirelength optimum. The case
//! split is by net shape: two pins (one edge), the boundary pair, and
//! every inner pin to both bounds.
//!
//! The solve leaves instances on top of each other wherever wirelength
//! wants them; spreading is `nesterov`'s job.

use super::qp::{Csr, SolveQp};
use super::{Pin, PlacementState, Problem};

#[derive(Debug, Clone, Copy)]
pub struct QpOpts {
    /// Rebuild-and-solve rounds.
    pub iters: usize,
    /// Edge lengths below this count as this, so coincident pins do not
    /// get unbounded weight. In grid units.
    pub min_dist: f64,
    /// Pull of every instance towards the region's centre: keeps the
    /// system positive definite for logic with no path to a fixed pin.
    pub anchor: f64,
}

impl Default for QpOpts {
    fn default() -> QpOpts {
        QpOpts {
            // RePlAce release default, `-initial_place_max_iter 20`.
            iters: 20,
            // Ours: a tenth of a tile.
            min_dist: 0.1,
            // Ours: small against any real edge weight.
            anchor: 1e-6,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Axis {
    X,
    Y,
}

/// The B2B system `A c = b` for one axis at placement `st`.
pub fn build(prob: &Problem, st: &PlacementState, axis: Axis, opts: &QpOpts) -> (Csr, Vec<f64>) {
    let n = prob.insts.len();
    let coord = |pin: Pin| {
        let (x, y) = prob.pin_pos(pin, st);
        if axis == Axis::X { x } else { y }
    };
    let centre = if axis == Axis::X { prob.region.centre().0 } else { prob.region.centre().1 };
    let mut t: Vec<(u32, u32, f64)> = (0..n as u32).map(|i| (i, i, opts.anchor)).collect();
    let mut b = vec![opts.anchor * centre; n];
    let mut edge = |p: Pin, q: Pin, w: f64| match (p, q) {
        (Pin::Inst(i), Pin::Inst(j)) if i != j => {
            t.extend([(i, i, w), (j, j, w), (i, j, -w), (j, i, -w)]);
        }
        (Pin::Inst(i), f @ Pin::Fixed(..)) | (f @ Pin::Fixed(..), Pin::Inst(i)) => {
            t.push((i, i, w));
            b[i as usize] += w * coord(f);
        }
        _ => {}
    };
    for net in &prob.nets {
        let pins = &net.pins;
        let cs: Vec<f64> = pins.iter().map(|&p| coord(p)).collect();
        let (mut lo, mut hi) = (0, 0);
        for k in 1..pins.len() {
            if cs[k] < cs[lo] {
                lo = k;
            }
            if cs[k] >= cs[hi] {
                hi = k;
            }
        }
        if lo == hi {
            // Every pin at one coordinate: any two distinct pins serve.
            hi = if lo == 0 { pins.len() - 1 } else { 0 };
        }
        let scale = net.weight * 2.0 / (pins.len() - 1) as f64;
        let w = |a: usize, c: usize| scale / (cs[a] - cs[c]).abs().max(opts.min_dist);
        edge(pins[lo], pins[hi], w(lo, hi));
        for k in (0..pins.len()).filter(|&k| k != lo && k != hi) {
            edge(pins[k], pins[lo], w(k, lo));
            edge(pins[k], pins[hi], w(k, hi));
        }
    }
    (Csr::from_triplets(n, t), b)
}

/// The quadratic start: everything at the region's centre, then
/// `opts.iters` rounds of build and solve on both axes.
pub fn initial_place(prob: &Problem, opts: &QpOpts, solver: &dyn SolveQp) -> PlacementState {
    let n = prob.insts.len();
    let (cx, cy) = prob.region.centre();
    let mut st = PlacementState {
        x: vec![cx; n],
        y: vec![cy; n],
    };
    for _ in 0..opts.iters {
        let (ax, bx) = build(prob, &st, Axis::X, opts);
        let (ay, by) = build(prob, &st, Axis::Y, opts);
        solver.solve(&ax, &bx, &mut st.x);
        solver.solve(&ay, &by, &mut st.y);
    }
    st
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::placer::qp::Cg;
    use crate::placer::{Inst, InstKind, PNet, Site};

    #[test]
    fn a_chain_between_two_pads_spreads_evenly() {
        // pad(0,0) - i0 - i1 - i2 - pad(8,4), all two-pin nets.
        let insts = vec![
            Inst {
                cells: Vec::new(),
                kind: InstKind::Slice
            };
            3
        ];
        let net = |a: Pin, b: Pin| PNet {
            pins: vec![a, b],
            weight: 1.0,
        };
        let nets = vec![
            net(Pin::Fixed(0.0, 0.0), Pin::Inst(0)),
            net(Pin::Inst(0), Pin::Inst(1)),
            net(Pin::Inst(1), Pin::Inst(2)),
            net(Pin::Inst(2), Pin::Fixed(8.0, 4.0)),
        ];
        let sites = [(0.0, 0.0), (8.0, 4.0)]
            .iter()
            .map(|&(x, y)| Site { x, y, name: String::new() })
            .collect();
        let prob = Problem::from_parts(insts, nets, sites);
        let st = initial_place(&prob, &QpOpts::default(), &Cg::default());
        // Linear wirelength is flat along the chain; what is asserted is
        // order, bounds and the minimum HPWL (12).
        assert!(st.x.windows(2).all(|w| w[0] <= w[1] + 1e-6), "{st:?}");
        assert!(st.x.iter().all(|&x| (-1e-6..=8.0 + 1e-6).contains(&x)));
        assert!(st.y.iter().all(|&y| (-1e-6..=4.0 + 1e-6).contains(&y)));
        assert!((prob.hpwl(&st) - 12.0).abs() < 1e-3, "{}", prob.hpwl(&st));
    }
}
