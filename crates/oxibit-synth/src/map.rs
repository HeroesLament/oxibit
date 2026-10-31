//! K-input LUT mapping of an AIG with priority cuts.
//!
//! Per node, keep the `C` best K-feasible cuts built from the fanins'
//! cuts. Pass 1 picks for depth; later passes pick for area flow under the
//! required times of the previous cover, so depth never grows. The cover
//! is extracted from the outputs, and each chosen cut's function is
//! computed by simulating its cone over the leaves.

use oxibit_prof::{hot, hot_block};

use crate::aig::{Aig, Lit, Node};

pub const K: usize = 6;
const C: usize = 8;

#[derive(Debug, Clone, Copy)]
struct Cut {
    leaves: [u32; K],
    n: u8,
    sig: u64,
    arr: u32,
    af: f32,
}

impl Cut {
    fn trivial(node: u32) -> Self {
        let mut leaves = [0; K];
        leaves[0] = node;
        Cut {
            leaves,
            n: 1,
            sig: 1u64 << (node % 64),
            arr: 0,
            af: 0.0,
        }
    }
    fn leaves(&self) -> &[u32] {
        &self.leaves[..self.n as usize]
    }
    /// Union of two sorted leaf sets, if it has at most K leaves.
    fn merge(a: &Cut, b: &Cut) -> Option<Cut> {
        let sig = a.sig | b.sig;
        if sig.count_ones() as usize > K {
            return None;
        }
        let (x, y) = (a.leaves(), b.leaves());
        let mut out = [0u32; K];
        let (mut i, mut j, mut n) = (0, 0, 0);
        while i < x.len() || j < y.len() {
            let v = if j == y.len() || (i < x.len() && x[i] < y[j]) {
                i += 1;
                x[i - 1]
            } else if i == x.len() || y[j] < x[i] {
                j += 1;
                y[j - 1]
            } else {
                i += 1;
                j += 1;
                x[i - 1]
            };
            if n == K {
                return None;
            }
            out[n] = v;
            n += 1;
        }
        Some(Cut {
            leaves: out,
            n: n as u8,
            sig,
            arr: 0,
            af: 0.0,
        })
    }
    fn dominates(&self, o: &Cut) -> bool {
        // Every leaf of self is in o.
        self.n <= o.n && (self.sig & o.sig) == self.sig && self.leaves().iter().all(|l| o.leaves().contains(l))
    }
}

/// One LUT of the cover: `root` computes `tt` over `leaves` (leaf i is
/// truth-table variable i).
#[derive(Debug, Clone)]
pub struct Lut {
    pub root: u32,
    pub leaves: Vec<u32>,
    pub tt: u64,
}

#[derive(Debug, Default)]
pub struct Mapping {
    pub luts: Vec<Lut>,
    /// `lut_of[node]`: index into `luts` when the node is a LUT root.
    pub lut_of: Vec<Option<u32>>,
    pub depth: u32,
}

#[derive(Clone, Copy, PartialEq)]
enum Goal {
    Depth,
    Area,
}

struct Mapper<'a> {
    g: &'a Aig,
    cuts: Vec<Vec<Cut>>,
    best: Vec<Option<Cut>>,
    arr: Vec<u32>,
    af: Vec<f32>,
    refs: Vec<f32>,
    req: Vec<u32>,
}

impl<'a> Mapper<'a> {
    fn pass(&mut self, goal: Goal) {
        let g = self.g;
        for (i, node) in g.nodes.iter().enumerate() {
            let Node::And(a, b) = *node else {
                self.cuts[i].clear();
                self.best[i] = None;
                self.arr[i] = 0;
                self.af[i] = 0.0;
                continue;
            };
            let (na, nb) = (a.node() as usize, b.node() as usize);
            let mut cand: Vec<Cut> = Vec::with_capacity(2 * C * C);
            let ta = Cut::trivial(na as u32);
            let tb = Cut::trivial(nb as u32);
            let ca: Vec<&Cut> = std::iter::once(&ta).chain(self.cuts[na].iter()).collect();
            let cb: Vec<&Cut> = std::iter::once(&tb).chain(self.cuts[nb].iter()).collect();
            for x in &ca {
                for y in &cb {
                    let Some(mut c) = Cut::merge(x, y) else {
                        continue;
                    };
                    if cand.iter().any(|o| o.dominates(&c)) {
                        continue;
                    }
                    cand.retain(|o| !c.dominates(o));
                    let mut arr = 0;
                    let mut af = 1.0f32;
                    for &l in c.leaves() {
                        arr = arr.max(self.arr[l as usize]);
                        af += self.af[l as usize];
                    }
                    c.arr = arr + 1;
                    c.af = af;
                    cand.push(c);
                }
            }
            let req = self.req[i];
            match goal {
                Goal::Depth => cand.sort_by(|p, q| p.arr.cmp(&q.arr).then(p.af.total_cmp(&q.af)).then(p.n.cmp(&q.n))),
                Goal::Area => cand.sort_by(|p, q| {
                    (p.arr > req)
                        .cmp(&(q.arr > req))
                        .then(p.af.total_cmp(&q.af))
                        .then(p.arr.cmp(&q.arr))
                        .then(p.n.cmp(&q.n))
                }),
            }
            cand.truncate(C);
            let b = cand[0];
            self.best[i] = Some(b);
            self.arr[i] = b.arr;
            self.af[i] = b.af / self.refs[i].max(1.0);
            self.cuts[i] = cand;
        }
    }

    /// Mark the cover reachable from the outputs; set required times and
    /// reference counts from it. Returns the cover's depth.
    fn cover(&mut self, outs: &[Lit]) -> (Vec<bool>, u32) {
        let n = self.g.nodes.len();
        let mut used = vec![false; n];
        let mut stack: Vec<u32> = Vec::new();
        let mut depth = 0;
        for o in outs {
            let k = o.node();
            depth = depth.max(self.arr[k as usize]);
            if !used[k as usize] {
                used[k as usize] = true;
                stack.push(k);
            }
        }
        let mut nrefs = vec![0u32; n];
        while let Some(k) = stack.pop() {
            if let Some(c) = self.best[k as usize] {
                for &l in c.leaves() {
                    nrefs[l as usize] += 1;
                    if !used[l as usize] {
                        used[l as usize] = true;
                        stack.push(l);
                    }
                }
            }
        }
        // Required times, in reverse topological order.
        self.req.iter_mut().for_each(|r| *r = u32::MAX);
        for o in outs {
            self.req[o.node() as usize] = depth;
        }
        for k in (0..n).rev() {
            if !used[k] {
                continue;
            }
            if let Some(c) = self.best[k] {
                let r = self.req[k].saturating_sub(1);
                for &l in c.leaves() {
                    let x = &mut self.req[l as usize];
                    *x = (*x).min(r);
                }
            }
        }
        // Blend the cover's references into the estimate (as in ABC's if).
        for k in 0..n {
            self.refs[k] = (self.refs[k] + 2.0 * nrefs[k] as f32) / 3.0;
        }
        (used, depth)
    }
}

/// Truth table of `root` over `leaves` (at most 6).
pub fn cut_tt(g: &Aig, root: u32, leaves: &[u32]) -> u64 {
    const VAR: [u64; 6] = [
        0xAAAA_AAAA_AAAA_AAAA,
        0xCCCC_CCCC_CCCC_CCCC,
        0xF0F0_F0F0_F0F0_F0F0,
        0xFF00_FF00_FF00_FF00,
        0xFFFF_0000_FFFF_0000,
        0xFFFF_FFFF_0000_0000,
    ];
    let mut memo: Vec<(u32, u64)> = leaves.iter().enumerate().map(|(i, &l)| (l, VAR[i])).collect();
    fn go(g: &Aig, n: u32, memo: &mut Vec<(u32, u64)>) -> u64 {
        if let Some(&(_, v)) = memo.iter().find(|(k, _)| *k == n) {
            return v;
        }
        let v = match g.nodes[n as usize] {
            Node::Const => 0,
            Node::And(a, b) => {
                let x = go(g, a.node(), memo) ^ if a.compl() { !0 } else { 0 };
                let y = go(g, b.node(), memo) ^ if b.compl() { !0 } else { 0 };
                x & y
            }
            Node::Input(_) => unreachable!("input {n} inside a cut cone"),
        };
        memo.push((n, v));
        v
    }
    go(g, root, &mut memo)
}

#[hot]
pub fn map(g: &Aig, outs: &[Lit], area_passes: usize, rep: &dyn oxibit_report::Progress) -> Mapping {
    let n = g.nodes.len();
    let mut refs = vec![0f32; n];
    for node in &g.nodes {
        if let Node::And(a, b) = node {
            refs[a.node() as usize] += 1.0;
            refs[b.node() as usize] += 1.0;
        }
    }
    for o in outs {
        refs[o.node() as usize] += 1.0;
    }
    let mut m = Mapper {
        g,
        cuts: vec![Vec::new(); n],
        best: vec![None; n],
        arr: vec![0; n],
        af: vec![0.0; n],
        refs,
        req: vec![u32::MAX; n],
    };
    hot_block!("map.depth", { m.pass(Goal::Depth) });
    let (mut used, mut depth) = m.cover(outs);
    let report = |pass: &str, best: &[Option<Cut>], used: &[bool], depth: u32| {
        let luts = used.iter().enumerate().filter(|(k, u)| **u && best[*k].is_some()).count();
        rep.emit(oxibit_report::Event::progress(
            "map",
            serde_json::json!({"pass": pass, "luts": luts, "depth": depth}),
        ));
    };
    report("depth", &m.best, &used, depth);
    for k in 0..area_passes {
        hot_block!("map.area", { m.pass(Goal::Area) });
        (used, depth) = m.cover(outs);
        report(&format!("area{}", k + 1), &m.best, &used, depth);
    }
    m.cuts = Vec::new();
    hot_block!("map.tt", {
        let mut out = Mapping {
            luts: Vec::new(),
            lut_of: vec![None; n],
            depth,
        };
        for k in 0..n {
            if !used[k] {
                continue;
            }
            if let Some(c) = m.best[k] {
                let leaves = c.leaves().to_vec();
                let tt = cut_tt(g, k as u32, &leaves);
                out.lut_of[k] = Some(out.luts.len() as u32);
                out.luts.push(Lut { root: k as u32, leaves, tt });
            }
        }
        out
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aig::lit_val;

    /// Evaluate the cover for 64 patterns and compare with the AIG.
    fn check(g: &Aig, outs: &[Lit], m: &Mapping) {
        let mut seed = 0x1234_5678_9abc_def0u64;
        let pats: Vec<u64> = (0..g.inputs.len())
            .map(|_| {
                seed ^= seed << 13;
                seed ^= seed >> 7;
                seed ^= seed << 17;
                seed
            })
            .collect();
        let want = g.simulate(&pats);
        let mut got = vec![0u64; g.nodes.len()];
        for (i, node) in g.nodes.iter().enumerate() {
            if let Node::Input(k) = node {
                got[i] = pats[*k as usize];
            }
        }
        for l in &m.luts {
            let mut v = 0u64;
            for bit in 0..64 {
                let mut idx = 0;
                for (j, leaf) in l.leaves.iter().enumerate() {
                    idx |= (((got[*leaf as usize] >> bit) & 1) as usize) << j;
                }
                v |= ((l.tt >> idx) & 1) << bit;
            }
            got[l.root as usize] = v;
        }
        for o in outs {
            assert_eq!(lit_val(&got, *o), lit_val(&want, *o));
        }
    }

    #[test]
    fn maps_adder() {
        let mut g = Aig::new();
        let a: Vec<Lit> = (0..12).map(|_| g.input()).collect();
        let b: Vec<Lit> = (0..12).map(|_| g.input()).collect();
        let mut c = Lit::FALSE;
        let mut outs = Vec::new();
        for i in 0..12 {
            let s = g.xor(a[i], b[i]);
            outs.push(g.xor(s, c));
            let t = g.and(a[i], b[i]);
            let u = g.and(s, c);
            c = g.or(t, u);
        }
        outs.push(c);
        let m = map(&g, &outs, 2, &oxibit_report::Silent);
        check(&g, &outs, &m);
        assert!(m.luts.iter().all(|l| l.leaves.len() <= K));
    }
}
