//! The placer's wire delay grid (`placer::timing::DelayGrid`) from a routed
//! design's timed arcs, as `sta` writes them with STA_ARCS.
//!
//! cargo run --release -p oxibit-place --example delay_grid -- \
//!     ARCS.tsv [MIN] > grid.txt
//!
//! Each cell is the mean routed delay of the arcs whose |dx| and |dy| fall
//! nearest its offsets: VPR's delay matrix, which is the routed delay at
//! each offset. Left out: arcs with an unplaced end, and carry chain to
//! carry chain (dedicated wires the placer never times). A cell with fewer
//! than MIN arcs (default 20) takes the mean at its Manhattan distance over
//! all the arcs, and the grid is then made to rise along both axes, so no
//! offset is cheaper than a nearer one.
//!
//! stderr: one JSON line of counts. Same file in, same grid out.

use oxibit_place::placer::timing::DelayGrid;
use serde_json::json;

/// Offsets the grid is measured at, in tiles. Dense near, where most arcs
/// are; sparse far, where few are and delay is close to linear.
const POINTS: [f64; 15] = [0.0, 1.0, 2.0, 3.0, 4.0, 6.0, 8.0, 12.0, 16.0, 24.0, 32.0, 48.0, 64.0, 96.0, 128.0];

/// The point nearest `v`, ties to the lower; past the last, the last.
fn nearest(v: f64) -> usize {
    let mut best = 0;
    for (i, &p) in POINTS.iter().enumerate() {
        if (v - p).abs() < (v - POINTS[best]).abs() {
            best = i;
        }
    }
    best
}

/// `ys` over ascending `xs` at `v`: linear between points, along the end
/// segments past them.
fn lerp(xs: &[f64], ys: &[f64], v: f64) -> f64 {
    let k = xs.iter().position(|&x| x >= v).unwrap_or(xs.len() - 1).max(1);
    ys[k - 1] + (v - xs[k - 1]) * (ys[k] - ys[k - 1]) / (xs[k] - xs[k - 1])
}

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).ok_or("usage: delay_grid ARCS.tsv [MIN]")?;
    let min: usize = match args.get(2) {
        Some(m) => m.parse().map_err(|_| format!("{m} is not a count"))?,
        None => 20,
    };
    let text = std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?;

    // 1. Sums by (dx, dy) point, and by Manhattan distance for the fallback.
    let n = POINTS.len();
    let (mut sum, mut cnt) = (vec![0f64; n * n], vec![0usize; n * n]);
    let (mut sum1, mut cnt1) = (vec![0f64; n], vec![0usize; n]);
    let (mut used, mut unplaced, mut carry) = (0usize, 0usize, 0usize);
    let (mut max_dx, mut max_dy) = (0i64, 0i64);
    for (ln, line) in text.lines().enumerate() {
        let f: Vec<&str> = line.split('\t').collect();
        if f.len() < 8 {
            return Err(format!("{path}:{}: {} columns; this needs sta's dx, dy and cell kinds (8)", ln + 1, f.len()));
        }
        let num = |s: &str| s.trim().parse::<i64>().map_err(|_| format!("{path}:{}: {s} is not a number", ln + 1));
        let (ps, dist, dx, dy) = (num(f[2])?, num(f[3])?, num(f[4])?.abs(), num(f[5])?.abs());
        if dist < 0 {
            unplaced += 1;
            continue;
        }
        if f[6].trim() == "Carry" && f[7].trim() == "Carry" {
            carry += 1;
            continue;
        }
        used += 1;
        max_dx = max_dx.max(dx);
        max_dy = max_dy.max(dy);
        let (i, j) = (nearest(dx as f64), nearest(dy as f64));
        sum[j * n + i] += ps as f64;
        cnt[j * n + i] += 1;
        let k = nearest((dx + dy) as f64);
        sum1[k] += ps as f64;
        cnt1[k] += 1;
    }
    if used == 0 {
        return Err(format!("{path}: no arcs to measure"));
    }

    // 2. The one-dimensional means, at the distances with enough arcs.
    let known: Vec<(f64, f64)> = (0..n).filter(|&k| cnt1[k] >= min).map(|k| (POINTS[k], sum1[k] / cnt1[k] as f64)).collect();
    let fallback = |d: f64| -> f64 {
        match known.len() {
            0 => f64::NAN,
            1 => known[0].1,
            _ => {
                let (xs, ys): (Vec<f64>, Vec<f64>) = known.iter().copied().unzip();
                lerp(&xs, &ys, d).max(ys[0])
            }
        }
    };
    if known.is_empty() {
        return Err(format!("{path}: no distance has {min} arcs; lower MIN"));
    }

    // 3. The grid over the offsets seen, one point past the largest.
    let upto = |m: i64| POINTS.iter().position(|&p| p >= m as f64).map_or(n, |k| k + 1).clamp(2, n);
    let (nx, ny) = (upto(max_dx), upto(max_dy));
    let (mut measured, mut filled) = (0usize, 0usize);
    let mut ps = vec![0f64; nx * ny];
    for j in 0..ny {
        for i in 0..nx {
            let c = j * n + i;
            ps[j * nx + i] = if cnt[c] >= min {
                measured += 1;
                sum[c] / cnt[c] as f64
            } else {
                filled += 1;
                fallback(POINTS[i] + POINTS[j])
            };
        }
    }

    // 4. Rising along both axes: a farther offset never costs less.
    let mut raised = 0usize;
    for j in 0..ny {
        for i in 0..nx {
            let mut v = ps[j * nx + i];
            if i > 0 {
                v = v.max(ps[j * nx + i - 1]);
            }
            if j > 0 {
                v = v.max(ps[(j - 1) * nx + i]);
            }
            if v > ps[j * nx + i] {
                raised += 1;
            }
            ps[j * nx + i] = v;
        }
    }

    let grid = DelayGrid { dx: POINTS[..nx].to_vec(), dy: POINTS[..ny].to_vec(), ps };
    // Round-trip before writing: what the placer reads is what this made.
    let out = grid.to_text();
    DelayGrid::parse(&out)?;
    print!("# delay_grid from {path}: {used} arcs, cells under {min} arcs filled by distance\n{out}");
    eprintln!(
        "{}",
        json!({"delay_grid": {"arcs": used, "unplaced": unplaced, "carry_to_carry": carry, "min": min,
                              "max_dx": max_dx, "max_dy": max_dy, "cells": nx * ny,
                              "measured": measured, "filled": filled, "raised": raised}})
    );
    Ok(())
}
