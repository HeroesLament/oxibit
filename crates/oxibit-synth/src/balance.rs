//! AIG balancing (ABC `balance -x`): rebuild AND and XOR supergates as
//! trees ordered by arrival level.
//!
//! Lowering builds each word-level cell balanced on its own, but chains of
//! cells (an LFSR advanced many steps in one cycle, an accumulation spelled
//! as a sequence of `^` or `&`) stay as deep as the source spelled them. A
//! supergate is a maximal tree of same-kind gates whose internal nodes feed
//! nothing else: AND nodes through uncomplemented edges, or XOR nodes (the
//! three-AND pattern `aig::xor` builds, whose operands each feed two of
//! the parent's ANDs) through any edge, the complements folded into a
//! parity. Its leaves are rebuilt first, then combined two
//! shallowest at a time, which gives the minimum depth for those leaves.
//! An XOR cone whose leaves cancel down to a few (linear logic: an LFSR or
//! scrambler stepped many times in one cycle) is collapsed through shared
//! XOR nodes as well, duplicating them, since the collapsed form is both
//! shallower and usually smaller. The result is a new AIG; `Lowered`
//! literals are mapped onto it, inputs keeping their order.

use std::cmp::Reverse;
use std::collections::BinaryHeap;

use crate::aig::{Aig, Lit, Node};
use crate::lower::Lowered;

struct Bal<'a> {
    old: &'a Aig,
    refs: Vec<u32>,
    new: Aig,
    map: Vec<Option<Lit>>,
    level: Vec<u32>,
}

impl Bal<'_> {
    fn fanins(&self, n: u32) -> Option<(Lit, Lit)> {
        match self.old.nodes[n as usize] {
            Node::And(a, b) => Some((a, b)),
            _ => None,
        }
    }

    /// `n` is XOR(u, v) as `aig::xor` builds it: n = AND(~x, ~y) with
    /// x = AND(u, v) and y = AND(~u, ~v), x and y feeding only n.
    fn xor_of(&self, n: u32) -> Option<(Lit, Lit)> {
        let (p, q) = self.fanins(n)?;
        if !p.compl() || !q.compl() || self.refs[p.node() as usize] != 1 || self.refs[q.node() as usize] != 1 {
            return None;
        }
        let (a, b) = self.fanins(p.node())?;
        let (c, d) = self.fanins(q.node())?;
        if (c == a.not() && d == b.not()) || (c == b.not() && d == a.not()) {
            Some((a, b))
        } else {
            None
        }
    }

    fn lvl(&self, l: Lit) -> u32 {
        self.level.get(l.node() as usize).copied().unwrap_or(0)
    }

    fn and(&mut self, a: Lit, b: Lit) -> Lit {
        let r = self.new.and(a, b);
        self.set_level(r, self.lvl(a).max(self.lvl(b)) + 1);
        r
    }

    fn xor(&mut self, a: Lit, b: Lit) -> Lit {
        let r = self.new.xor(a, b);
        self.set_level(r, self.lvl(a).max(self.lvl(b)) + 2);
        r
    }

    fn set_level(&mut self, r: Lit, l: u32) {
        let n = r.node() as usize;
        if n >= self.level.len() {
            self.level.resize(n + 1, 0);
            self.level[n] = l;
        }
    }

    /// The new literal for old literal `l`.
    fn build(&mut self, l: Lit) -> Lit {
        let n = l.node();
        if let Some(m) = self.map[n as usize] {
            return m.not_if(l.compl());
        }
        // Iterative post-order over supergate leaves, so deep chains do not
        // recurse once per node.
        let mut stack = vec![(n, false)];
        while let Some((k, expanded)) = stack.pop() {
            if self.map[k as usize].is_some() {
                continue;
            }
            let leaves = self.leaves(k);
            if !expanded {
                stack.push((k, true));
                for leaf in &leaves.1 {
                    if self.map[leaf.node() as usize].is_none() {
                        stack.push((leaf.node(), false));
                    }
                }
                continue;
            }
            let r = self.combine(&leaves);
            self.map[k as usize] = Some(r);
        }
        self.map[n as usize].unwrap().not_if(l.compl())
    }

    /// (is XOR, leaf literals, parity) of the supergate rooted at `k`.
    /// AND: the root's value is the AND of the leaves. XOR: the XOR of the
    /// leaves, complemented when parity is set.
    /// Expand the XOR cone at `k` through shared XOR nodes too, cancelling
    /// repeated leaves (linear logic such as an LFSR stepped many times in
    /// one cycle collapses to a few state bits). `None` when the expansion
    /// visits more than `COLLAPSE_VISITS` nodes or leaves more than
    /// `COLLAPSE_LEAVES` leaves.
    fn xor_collapse(&self, k: u32) -> Option<(Vec<Lit>, bool)> {
        const COLLAPSE_VISITS: usize = 512;
        const COLLAPSE_LEAVES: usize = 12;
        let (u, v) = self.xor_of(k)?;
        let mut odd: std::collections::BTreeMap<Lit, bool> = std::collections::BTreeMap::new();
        let mut parity = false;
        let mut work = vec![u, v];
        let mut visits = 0;
        while let Some(l) = work.pop() {
            visits += 1;
            if visits > COLLAPSE_VISITS {
                return None;
            }
            parity ^= l.compl();
            if let Some((a, b)) = self.xor_of(l.node()) {
                work.push(a);
                work.push(b);
            } else {
                let e = odd.entry(l.positive()).or_insert(false);
                *e = !*e;
            }
        }
        let out: Vec<Lit> = odd.into_iter().filter(|(_, o)| *o).map(|(l, _)| l).collect();
        if out.len() > COLLAPSE_LEAVES {
            return None;
        }
        Some((out, parity))
    }

    fn leaves(&self, k: u32) -> (bool, Vec<Lit>, bool) {
        if let Some((out, parity)) = self.xor_collapse(k) {
            return (true, out, parity);
        }
        if let Some((u, v)) = self.xor_of(k) {
            let mut out = Vec::new();
            let mut parity = false;
            let mut work = vec![u, v];
            while let Some(l) = work.pop() {
                // An XOR operand feeds both of the parent's ANDs: two
                // references when nothing else reads it.
                let m = l.node();
                if self.refs[m as usize] == 2 {
                    if let Some((a, b)) = self.xor_of(m) {
                        parity ^= l.compl();
                        work.push(a);
                        work.push(b);
                        continue;
                    }
                }
                parity ^= l.compl();
                out.push(l.positive());
            }
            // x ^ x = 0.
            out.sort();
            let mut dedup = Vec::with_capacity(out.len());
            for l in out {
                if dedup.last() == Some(&l) {
                    dedup.pop();
                } else {
                    dedup.push(l);
                }
            }
            return (true, dedup, parity);
        }
        match self.old.nodes[k as usize] {
            Node::And(a, b) => {
                let mut out = Vec::new();
                let mut work = vec![a, b];
                while let Some(l) = work.pop() {
                    let m = l.node();
                    if !l.compl() && self.refs[m as usize] == 1 && self.xor_of(m).is_none() {
                        if let Some((x, y)) = self.fanins(m) {
                            work.push(x);
                            work.push(y);
                            continue;
                        }
                    }
                    out.push(l);
                }
                out.sort();
                out.dedup();
                (false, out, false)
            }
            _ => (false, Vec::new(), false),
        }
    }

    fn combine(&mut self, (is_xor, leaves, parity): &(bool, Vec<Lit>, bool)) -> Lit {
        if leaves.is_empty() {
            // An input or the constant.
            return Lit::FALSE.not_if(*is_xor && *parity);
        }
        let mut heap: BinaryHeap<Reverse<(u32, Lit)>> = BinaryHeap::new();
        for l in leaves {
            let m = self.map[l.node() as usize].expect("leaf built first").not_if(l.compl());
            heap.push(Reverse((self.lvl(m), m)));
        }
        while heap.len() > 1 {
            let Reverse((_, a)) = heap.pop().unwrap();
            let Reverse((_, b)) = heap.pop().unwrap();
            let r = if *is_xor { self.xor(a, b) } else { self.and(a, b) };
            heap.push(Reverse((self.lvl(r), r)));
        }
        let Reverse((_, r)) = heap.pop().unwrap();
        r.not_if(*is_xor && *parity)
    }
}

/// Balance `lw.aig` and map every literal of `lw` onto the result.
/// Returns (AND nodes before, after).
pub fn balance(lw: &mut Lowered) -> (usize, usize) {
    let before = lw.aig.ands();
    let outs = lw.outputs();
    let old = std::mem::take(&mut lw.aig);
    let n = old.nodes.len();
    let mut refs = vec![0u32; n];
    for node in &old.nodes {
        if let Node::And(a, b) = node {
            refs[a.node() as usize] += 1;
            refs[b.node() as usize] += 1;
        }
    }
    for l in &outs {
        refs[l.node() as usize] += 1;
    }
    let mut new = Aig::new();
    new.carry_min = old.carry_min;
    new.lut_k = old.lut_k;
    let mut b = Bal {
        old: &old,
        refs,
        new,
        map: vec![None; n],
        level: vec![0],
    };
    b.map[0] = Some(Lit::FALSE);
    for &i in &old.inputs {
        let l = b.new.input();
        b.map[i as usize] = Some(l);
        b.set_level(l, 0);
    }
    for l in &outs {
        b.build(*l);
    }
    let map = b.map;
    let mut new = b.new;
    let f = |l: Lit| -> Lit { map[l.node() as usize].expect("literal outside the cover").not_if(l.compl()) };
    new.chains = old
        .chains
        .iter()
        .map(|c| crate::aig::Chain {
            s: c.s.iter().map(|l| f(*l)).collect(),
            di: c.di.iter().map(|l| f(*l)).collect(),
            cin: f(c.cin),
            o: c.o.iter().map(|l| f(*l)).collect(),
            co: f(c.co),
        })
        .collect();
    lw.aig = new;
    for p in &mut lw.ports {
        // Input port bits are AIG inputs; output bits are roots.
        p.3 = p.3.iter().map(|l| f(*l)).collect();
    }
    for ff in &mut lw.ffs {
        ff.q = f(ff.q);
        ff.d = f(ff.d);
        ff.ce = f(ff.ce);
        ff.clk = f(ff.clk);
        ff.sr = f(ff.sr);
    }
    for i in &mut lw.insts {
        for (_, bits) in &mut i.conns {
            *bits = bits.iter().map(|l| f(*l)).collect();
        }
    }
    for m in &mut lw.mems {
        for (c, a, d) in &mut m.rd {
            if let Some((cl, _)) = c {
                *cl = f(*cl);
            }
            *a = a.iter().map(|l| f(*l)).collect();
            *d = d.iter().map(|l| f(*l)).collect();
        }
        for ((c, _), a, d, e) in &mut m.wr {
            *c = f(*c);
            *a = a.iter().map(|l| f(*l)).collect();
            *d = d.iter().map(|l| f(*l)).collect();
            *e = e.iter().map(|l| f(*l)).collect();
        }
    }
    // Names of nodes that are now inside a supergate are dropped.
    for (_, _, bits) in &mut lw.named {
        for b in bits.iter_mut() {
            *b = b.and_then(|l| map[l.node() as usize].map(|m| m.not_if(l.compl())));
        }
    }
    lw.dead = lw.dead.iter().filter_map(|n| map[*n as usize].map(|l| l.node())).collect();
    (before, lw.aig.ands())
}

pub struct Balance;

#[derive(Debug, Default, Clone, Copy)]
pub struct BalanceStats {
    pub ands_before: usize,
    pub ands_after: usize,
}

impl crate::stage::Pass for Balance {
    const NAME: &'static str = "balance";
    const EMITS: &'static [crate::caps::Mode] = &[];
    type Stats = BalanceStats;
    fn enabled(p: &crate::policy::PassSet) -> bool {
        p.balance
    }
    fn json(s: &BalanceStats) -> serde_json::Value {
        serde_json::json!({ "ands_before": s.ands_before, "ands_after": s.ands_after })
    }
}

impl crate::stage::AigPass for Balance {
    fn run(&self, lw: &mut Lowered) -> BalanceStats {
        let (ands_before, ands_after) = balance(lw);
        BalanceStats { ands_before, ands_after }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::aig::lit_val;

    /// A deep XOR chain and a deep AND chain come back log-depth and
    /// equal on random inputs.
    #[test]
    fn chains() {
        let mut lw = Lowered::default();
        lw.aig = Aig::new();
        let ins: Vec<Lit> = (0..32).map(|_| lw.aig.input()).collect();
        let mut x = ins[0];
        let mut a = ins[0];
        for (k, i) in ins[1..].iter().enumerate() {
            x = lw.aig.xor(x, if k % 3 == 0 { i.not() } else { *i });
            a = lw.aig.and(a, *i);
        }
        let m = lw.aig.mux(ins[3], x, a.not());
        lw.ports.push(("y".into(), oxibit_hdl::netlist::Dir::Output, false, vec![x, a, m]));
        let pats: Vec<u64> = (0..32u64)
            .map(|k| k.wrapping_mul(0x9e37_79b9_7f4a_7c15).rotate_left(k as u32 * 7))
            .collect();
        let v0 = lw.aig.simulate(&pats);
        let want: Vec<u64> = lw.ports[0].3.iter().map(|l| lit_val(&v0, *l)).collect();
        balance(&mut lw);
        let v1 = lw.aig.simulate(&pats);
        let got: Vec<u64> = lw.ports[0].3.iter().map(|l| lit_val(&v1, *l)).collect();
        assert_eq!(want, got);
        // Depth: levels of the new AIG.
        let mut lv = vec![0u32; lw.aig.nodes.len()];
        for (i, n) in lw.aig.nodes.iter().enumerate() {
            if let Node::And(p, q) = n {
                lv[i] = 1 + lv[p.node() as usize].max(lv[q.node() as usize]);
            }
        }
        let d = |l: Lit| lv[l.node() as usize];
        assert!(d(lw.ports[0].3[0]) <= 10, "xor depth {}", d(lw.ports[0].3[0]));
        assert!(d(lw.ports[0].3[1]) <= 5, "and depth {}", d(lw.ports[0].3[1]));
    }
}
