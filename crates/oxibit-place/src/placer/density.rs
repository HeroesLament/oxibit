//! The bin grid, overflow, and the electrostatic density field.
//!
//! Density is ePlace's (Lu et al., TODAES 2015): instances are charges,
//! the charge density `rho` over the bins has a potential `psi` with
//! `-laplace(psi) = rho` under zero-gradient boundaries, and each
//! instance is pushed along the field `E = -grad(psi)`. With `rho` in a
//! cosine series, `psi` and `E` come out term by term:
//!
//!   rho = sum a_uv cos(w_u x) cos(w_v y)
//!   E_x = sum a_uv w_u / (w_u^2 + w_v^2) sin(w_u x) cos(w_v y)
//!   E_y = sum a_uv w_v / (w_u^2 + w_v^2) cos(w_u x) sin(w_v y)
//!
//! The transforms here are plain separable sums over cosine tables,
//! O(n^3) for an n-by-n grid. At the grid sizes a Zynq-7000 needs that is
//! microseconds, so there is no FFT.
//!
//! A bin's capacity is the slice sites in it. Bins short of the fullest
//! one carry the difference as fixed charge, so columns without slices
//! (block RAM, DSP, the PS corner) repel like occupied ground.

use super::{PlacementState, Problem, Rect};

#[derive(Debug, Clone)]
pub struct BinGrid {
    pub nx: usize,
    pub ny: usize,
    pub region: Rect,
    pub bw: f64,
    pub bh: f64,
    /// Per bin (`y * nx + x`), the slice sites in it (indices into
    /// `Problem::sites`).
    pub sites: Vec<Vec<u32>>,
    /// The most sites any bin holds.
    pub cap_max: f64,
}

/// Router congestion fed back to the placer: history weight at grid
/// points, and the share of its sites the most congested bins give up.
#[derive(Debug, Clone, Default)]
pub struct Congest {
    pub points: Vec<(f64, f64, f64)>,
    pub strength: f64,
}

impl BinGrid {
    /// Per bin, the capacity withheld for `c`: `strength` of the bin's sites
    /// at or above the 90th percentile of weight among bins that have any,
    /// in proportion below it. Points outside the region count for nothing.
    pub fn withheld(&self, c: &Congest) -> Vec<f64> {
        let mut w = vec![0.0f64; self.len()];
        let r = &self.region;
        for &(x, y, v) in &c.points {
            if x >= r.x0 && x <= r.x1 && y >= r.y0 && y <= r.y1 {
                w[self.bin_at(x, y)] += v;
            }
        }
        let mut some: Vec<f64> = w.iter().copied().filter(|&v| v > 0.0).collect();
        if some.is_empty() {
            return w;
        }
        some.sort_by(|a, b| a.total_cmp(b));
        let top = some[(some.len() * 9 / 10).min(some.len() - 1)];
        (0..self.len()).map(|b| c.strength * self.cap(b) * (w[b] / top).min(1.0)).collect()
    }

    /// `n` bins a side over the problem's region.
    pub fn new(prob: &Problem, n: usize) -> BinGrid {
        let n = n.max(1);
        let region = prob.region;
        let mut g = BinGrid {
            nx: n,
            ny: n,
            region,
            bw: (region.x1 - region.x0) / n as f64,
            bh: (region.y1 - region.y0) / n as f64,
            sites: vec![Vec::new(); n * n],
            cap_max: 0.0,
        };
        for (i, s) in prob.sites.iter().enumerate() {
            let b = g.bin_at(s.x, s.y);
            g.sites[b].push(i as u32);
        }
        g.cap_max = g.sites.iter().map(Vec::len).max().unwrap_or(0) as f64;
        g
    }

    pub fn len(&self) -> usize {
        self.nx * self.ny
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn cap(&self, b: usize) -> f64 {
        self.sites[b].len() as f64
    }

    /// The bin holding a point (clamped into the region).
    pub fn bin_at(&self, x: f64, y: f64) -> usize {
        let i = (((x - self.region.x0) / self.bw).floor().max(0.0) as usize).min(self.nx - 1);
        let j = (((y - self.region.y0) / self.bh).floor().max(0.0) as usize).min(self.ny - 1);
        j * self.nx + i
    }

    pub fn centre(&self, b: usize) -> (f64, f64) {
        let (i, j) = (b % self.nx, b / self.nx);
        (self.region.x0 + (i as f64 + 0.5) * self.bw, self.region.y0 + (j as f64 + 0.5) * self.bh)
    }

    /// The four bins around a point and their bilinear weights.
    fn corners(&self, x: f64, y: f64) -> [(usize, f64); 4] {
        let fx = (x - self.region.x0) / self.bw - 0.5;
        let fy = (y - self.region.y0) / self.bh - 0.5;
        let (i0, j0) = (fx.floor(), fy.floor());
        let (tx, ty) = (fx - i0, fy - j0);
        let ci = |i: f64| (i.max(0.0) as usize).min(self.nx - 1);
        let cj = |j: f64| (j.max(0.0) as usize).min(self.ny - 1);
        let (ia, ib, ja, jb) = (ci(i0), ci(i0 + 1.0), cj(j0), cj(j0 + 1.0));
        [
            (ja * self.nx + ia, (1.0 - tx) * (1.0 - ty)),
            (ja * self.nx + ib, tx * (1.0 - ty)),
            (jb * self.nx + ia, (1.0 - tx) * ty),
            (jb * self.nx + ib, tx * ty),
        ]
    }

    /// Add charge `q` at a point to `rho`, shared bilinearly.
    pub fn deposit(&self, x: f64, y: f64, q: f64, rho: &mut [f64]) {
        for (b, w) in self.corners(x, y) {
            rho[b] += q * w;
        }
    }

    /// A per-bin quantity at a point, interpolated the way `deposit` shares.
    pub fn sample(&self, x: f64, y: f64, f: &[f64]) -> f64 {
        self.corners(x, y).iter().map(|&(b, w)| f[b] * w).sum()
    }
}

/// How far over capacity the bins are: the convergence signal.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Overflow {
    /// Bins over capacity.
    pub bins: usize,
    /// The worst bin's excess, in slices.
    pub max: f64,
    /// Total excess, in slices.
    pub sum: f64,
    /// `sum` over the slice instances placed: ePlace's overflow `tau`.
    pub ratio: f64,
}

/// Overflow of the instances that take a site (`Problem::takes_site`) at
/// `st`, each counted in the bin its centre is in, against `target` times
/// each bin's sites.
pub fn overflow(prob: &Problem, grid: &BinGrid, st: &PlacementState, target: f64) -> Overflow {
    let mut count = vec![0.0; grid.len()];
    let mut n = 0.0;
    for i in (0..prob.insts.len()).filter(|&i| prob.takes_site(i)) {
        for dy in prob.footprint(i) {
            count[grid.bin_at(st.x[i], st.y[i] + dy)] += 1.0;
            n += 1.0;
        }
    }
    let mut o = Overflow::default();
    for (b, &c) in count.iter().enumerate() {
        let over = c - target * grid.cap(b);
        if over > 0.0 {
            o.bins += 1;
            o.max = o.max.max(over);
            o.sum += over;
        }
    }
    o.ratio = if n > 0.0 { o.sum / n } else { 0.0 };
    o
}

/// The field solver for one grid: cosine and sine tables, built once.
#[derive(Debug, Clone)]
pub struct Spectral {
    nx: usize,
    ny: usize,
    /// `cos(w_u (i + 1/2))` at `[u * nx + i]`, and the sines.
    cx: Vec<f64>,
    sx: Vec<f64>,
    cy: Vec<f64>,
    sy: Vec<f64>,
    /// Physical frequencies `w_u = pi u / width`.
    wx: Vec<f64>,
    wy: Vec<f64>,
}

impl Spectral {
    pub fn new(grid: &BinGrid) -> Spectral {
        let table = |n: usize, f: fn(f64) -> f64| -> Vec<f64> {
            (0..n * n)
                .map(|k| f(std::f64::consts::PI * (k / n) as f64 * ((k % n) as f64 + 0.5) / n as f64))
                .collect()
        };
        let freq = |n: usize, width: f64| -> Vec<f64> { (0..n).map(|u| std::f64::consts::PI * u as f64 / width).collect() };
        Spectral {
            nx: grid.nx,
            ny: grid.ny,
            cx: table(grid.nx, f64::cos),
            sx: table(grid.nx, f64::sin),
            cy: table(grid.ny, f64::cos),
            sy: table(grid.ny, f64::sin),
            wx: freq(grid.nx, grid.region.x1 - grid.region.x0),
            wy: freq(grid.ny, grid.region.y1 - grid.region.y0),
        }
    }

    /// The field `(E_x, E_y)` per bin for charge density `rho` per bin.
    /// Only `rho`'s variation matters: its mean has no field.
    pub fn field(&self, rho: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let (nx, ny) = (self.nx, self.ny);
        // Forward: a[u][v] = norm * sum_ij rho[j][i] cx[u][i] cy[v][j].
        let mut half = vec![0.0; nx * ny]; // [u][j]
        for u in 0..nx {
            for j in 0..ny {
                half[u * ny + j] = (0..nx).map(|i| rho[j * nx + i] * self.cx[u * nx + i]).sum();
            }
        }
        let mut ex_c = vec![0.0; nx * ny]; // [u][v]
        let mut ey_c = vec![0.0; nx * ny];
        for u in 0..nx {
            for v in 0..ny {
                if u == 0 && v == 0 {
                    continue;
                }
                let s: f64 = (0..ny).map(|j| half[u * ny + j] * self.cy[v * ny + j]).sum();
                let norm = (if u == 0 { 1.0 } else { 2.0 }) * (if v == 0 { 1.0 } else { 2.0 }) / (nx * ny) as f64;
                let k2 = self.wx[u] * self.wx[u] + self.wy[v] * self.wy[v];
                ex_c[u * ny + v] = norm * s * self.wx[u] / k2;
                ey_c[u * ny + v] = norm * s * self.wy[v] / k2;
            }
        }
        // Inverse, y first: [u][j], then x.
        let mut ex_h = vec![0.0; nx * ny];
        let mut ey_h = vec![0.0; nx * ny];
        for u in 0..nx {
            for j in 0..ny {
                ex_h[u * ny + j] = (0..ny).map(|v| ex_c[u * ny + v] * self.cy[v * ny + j]).sum();
                ey_h[u * ny + j] = (0..ny).map(|v| ey_c[u * ny + v] * self.sy[v * ny + j]).sum();
            }
        }
        let mut ex = vec![0.0; nx * ny];
        let mut ey = vec![0.0; nx * ny];
        for j in 0..ny {
            for i in 0..nx {
                ex[j * nx + i] = (0..nx).map(|u| ex_h[u * ny + j] * self.sx[u * nx + i]).sum();
                ey[j * nx + i] = (0..nx).map(|u| ey_h[u * ny + j] * self.cx[u * nx + i]).sum();
            }
        }
        (ex, ey)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::placer::Site;

    fn grid(n: usize, side: f64) -> BinGrid {
        // One site in every bin.
        let step = side / n as f64;
        let sites = (0..n * n)
            .map(|k| Site {
                x: ((k % n) as f64 + 0.5) * step,
                y: ((k / n) as f64 + 0.5) * step,
                name: String::new(),
            })
            .collect();
        let mut prob = Problem::from_parts(Vec::new(), Vec::new(), sites);
        prob.region = Rect {
            x0: 0.0,
            y0: 0.0,
            x1: side,
            y1: side,
        };
        BinGrid::new(&prob, n)
    }

    #[test]
    fn the_field_points_away_from_charge() {
        let g = grid(16, 32.0);
        assert!(g.sites.iter().all(|s| s.len() == 1));
        let mut rho = vec![0.0; g.len()];
        g.deposit(16.0, 16.0, 1.0, &mut rho);
        assert!((rho.iter().sum::<f64>() - 1.0).abs() < 1e-12);
        let (ex, ey) = Spectral::new(&g).field(&rho);
        let at = |x: f64, y: f64| (g.sample(x, y, &ex), g.sample(x, y, &ey));
        let (right, left, up, down) = (at(22.0, 16.0), at(10.0, 16.0), at(16.0, 22.0), at(16.0, 10.0));
        assert!(right.0 > 0.0 && left.0 < 0.0, "{right:?} {left:?}");
        assert!(up.1 > 0.0 && down.1 < 0.0, "{up:?} {down:?}");
        // Symmetric charge, symmetric field; nothing across the axis.
        assert!((right.0 + left.0).abs() < 1e-9 && right.1.abs() < 1e-9);
        // Uniform charge has no field.
        let (ex, ey) = Spectral::new(&g).field(&vec![3.0; g.len()]);
        assert!(ex.iter().chain(&ey).all(|v| v.abs() < 1e-9));
    }
}
