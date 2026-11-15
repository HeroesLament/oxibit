//! The linear solve behind the quadratic placement: a sparse symmetric
//! positive-definite system `A x = b` per axis.
//!
//! `SolveQp` is the seam. `Cg` is the only implementation: compressed
//! sparse rows and Jacobi-preconditioned conjugate gradient, no
//! dependencies. A library backend goes behind the same trait if a profile
//! or convergence ever asks for one.

/// A square sparse matrix, compressed sparse rows.
#[derive(Debug, Clone)]
pub struct Csr {
    pub n: usize,
    row: Vec<usize>,
    col: Vec<u32>,
    val: Vec<f64>,
}

impl Csr {
    /// From `(row, col, value)` entries; entries at the same place add up.
    pub fn from_triplets(n: usize, mut t: Vec<(u32, u32, f64)>) -> Csr {
        t.sort_unstable_by_key(|&(r, c, _)| (r, c));
        let mut m = Csr {
            n,
            row: vec![0; n + 1],
            col: Vec::with_capacity(t.len()),
            val: Vec::with_capacity(t.len()),
        };
        let mut last: Option<(u32, u32)> = None;
        for (r, c, v) in t {
            if last == Some((r, c)) {
                *m.val.last_mut().expect("an entry") += v;
            } else {
                m.col.push(c);
                m.val.push(v);
                m.row[r as usize + 1] += 1;
                last = Some((r, c));
            }
        }
        for i in 0..n {
            m.row[i + 1] += m.row[i];
        }
        m
    }

    /// `out = A x`.
    pub fn mul(&self, x: &[f64], out: &mut [f64]) {
        for (i, o) in out.iter_mut().enumerate() {
            let (lo, hi) = (self.row[i], self.row[i + 1]);
            *o = self.col[lo..hi].iter().zip(&self.val[lo..hi]).map(|(&c, &v)| v * x[c as usize]).sum();
        }
    }

    pub fn diag(&self) -> Vec<f64> {
        (0..self.n)
            .map(|i| {
                let (lo, hi) = (self.row[i], self.row[i + 1]);
                self.col[lo..hi].iter().zip(&self.val[lo..hi]).filter(|(c, _)| **c as usize == i).map(|(_, &v)| v).sum()
            })
            .collect()
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct SolveStats {
    pub iters: usize,
    /// `|b - A x| / |b|` at exit.
    pub residual: f64,
}

/// Solves `A x = b` for symmetric positive-definite `A`, starting from the
/// `x` given.
pub trait SolveQp {
    fn solve(&self, a: &Csr, b: &[f64], x: &mut [f64]) -> SolveStats;
}

/// Jacobi-preconditioned conjugate gradient.
#[derive(Debug, Clone, Copy)]
pub struct Cg {
    pub max_iter: usize,
    /// Stop at this relative residual.
    pub tol: f64,
}

impl Default for Cg {
    fn default() -> Cg {
        // Ours: the B2B system is rebuilt every outer iteration, so a
        // loose inner solve is enough.
        Cg { max_iter: 100, tol: 1e-6 }
    }
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

impl SolveQp for Cg {
    fn solve(&self, a: &Csr, b: &[f64], x: &mut [f64]) -> SolveStats {
        let n = a.n;
        let inv: Vec<f64> = a.diag().iter().map(|&d| if d > 0.0 { 1.0 / d } else { 1.0 }).collect();
        let bnorm = dot(b, b).sqrt().max(f64::MIN_POSITIVE);
        let mut ap = vec![0.0; n];
        a.mul(x, &mut ap);
        let mut r: Vec<f64> = (0..n).map(|i| b[i] - ap[i]).collect();
        let mut z: Vec<f64> = (0..n).map(|i| r[i] * inv[i]).collect();
        let mut p = z.clone();
        let mut rz = dot(&r, &z);
        let mut st = SolveStats {
            iters: 0,
            residual: dot(&r, &r).sqrt() / bnorm,
        };
        while st.iters < self.max_iter && st.residual > self.tol {
            a.mul(&p, &mut ap);
            let pap = dot(&p, &ap);
            if pap <= 0.0 {
                break;
            }
            let alpha = rz / pap;
            for i in 0..n {
                x[i] += alpha * p[i];
                r[i] -= alpha * ap[i];
                z[i] = r[i] * inv[i];
            }
            let rz_new = dot(&r, &z);
            let beta = rz_new / rz;
            rz = rz_new;
            for i in 0..n {
                p[i] = z[i] + beta * p[i];
            }
            st.iters += 1;
            st.residual = dot(&r, &r).sqrt() / bnorm;
        }
        st
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cg_solves_a_laplacian_chain() {
        // Five nodes in a chain, the ends tied to 0 and 6: x = 1..5.
        let n = 5;
        let mut t = Vec::new();
        for i in 0..n as u32 {
            t.push((i, i, 1.0));
            t.push((i, i, 1.0));
            if i > 0 {
                t.push((i, i - 1, -1.0));
                t.push((i - 1, i, -1.0));
            }
        }
        let a = Csr::from_triplets(n, t);
        assert_eq!(a.diag(), vec![2.0; 5]);
        let b = [0.0, 0.0, 0.0, 0.0, 6.0];
        let mut x = vec![0.0; n];
        let st = Cg::default().solve(&a, &b, &mut x);
        assert!(st.residual <= 1e-6, "{st:?}");
        for (i, v) in x.iter().enumerate() {
            assert!((v - (i as f64 + 1.0)).abs() < 1e-5, "{x:?}");
        }
    }
}
