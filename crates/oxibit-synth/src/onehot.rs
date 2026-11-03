//! One-hot selection sums to ORs.
//!
//! EHDL writes a selection as `(s == 0) * a + (s == 1) * b + ...`. Each
//! term is zero unless its compare holds, and compares of one word against
//! distinct constants never hold together, so at most one term is nonzero
//! and every `+` in the sum adds a value to zero. Then `x + y == x | y` bit
//! for bit (no carry is ever generated), at any width, and the `$add`
//! becomes an `$or`: LUT logic instead of a carry chain.
//!
//! A cell is one-hot over the word `s` with key set `S` when it is
//! - an unsigned `$mul` of a one-bit `s == k` and a value, or an `$and` of
//!   a value and that bit replicated: `S = {k}` (a `k` that can never equal
//!   `s` adds nothing: the term is always zero);
//! - an unsigned `$add` or `$or` whose operands are each, bit for bit,
//!   constant zeros or output bits of one-hot cells over `s`, with the two
//!   operands' key sets disjoint: `S` is their union.
//!
//! An operand is zero unless one of its keys equals `s`; with disjoint key
//! sets the two operands are never nonzero together, which is all the
//! rewrite needs. Only `$add` cells change, and the miter checks the result.
//!
//! A second rule covers sums whose selects are not compares of one word:
//! `b * x + bnot(b) * y`, or priority-derived enables that are exclusive by
//! construction. A term `b * v` (a one-bit `b`, zero-extended, or replicated
//! under `$and`) is zero unless its GUARD `b` is 1, and a sum of guarded
//! values is zero unless one of their guards is. An `$add` the first rule
//! leaves is an `$or` when its two operands' guard sets are proven never 1
//! together (`exclusive::Exclusive`, SAT over their cones; unproven stays an
//! `$add`).

use std::collections::{BTreeSet, HashMap, HashSet};

use oxibit_hdl::netlist::{Bit, Cell, Module, Op, SigSpec};
use oxibit_prof::hot;

use crate::dsp::{Canon, K};

#[derive(Debug, Default, Clone)]
pub struct OneHotStats {
    /// `$add` cells turned into `$or`, and their output bits.
    pub adds: usize,
    pub bits: usize,
    /// Selection terms under the rewritten sums.
    pub terms: usize,
    /// Of `adds`, those rewritten by the guard rule, and its SAT checks.
    pub guarded: usize,
    pub sat_calls: usize,
    pub sat_proven: usize,
}

/// Guard sets past this many bits are not tracked (the add stays).
const GUARD_MAX: usize = 64;

/// Values that are zero unless one of their guard bits is 1. A guard is
/// kept as (canonical net, a bit of it) so sets dedupe by net.
struct Guards<'a> {
    m: &'a Module,
    cn: Canon,
    driver: HashMap<usize, usize>,
    memo: HashMap<usize, Option<Vec<(usize, Bit)>>>,
}

impl Guards<'_> {
    /// A selection term's guard: the one-bit side of an unsigned `$mul`
    /// (zero-extended) or `$and` (replicated).
    fn term(&mut self, ci: usize) -> Option<Vec<(usize, Bit)>> {
        let m = self.m;
        let Cell::Binary { op: op @ (Op::Mul | Op::And), a, b, signed: false, .. } = &m.cells[ci] else {
            return None;
        };
        for side in [a, b] {
            let first = *side.0.first()?;
            let Kb::Net(n) = kb(self.cn.k(first)) else { continue };
            let ok = match op {
                Op::Mul => side.0[1..].iter().all(|x| kb(self.cn.k(*x)) == Kb::Zero),
                _ => side.0.iter().all(|x| kb(self.cn.k(*x)) == Kb::Net(n)),
            };
            if ok {
                return Some(vec![(n, first)]);
            }
        }
        None
    }

    /// An operand's guards: constant zeros add none, every other bit must
    /// come from a guarded cell.
    fn operand(&mut self, s: &SigSpec) -> Option<Vec<(usize, Bit)>> {
        let mut acc: Vec<(usize, Bit)> = Vec::new();
        let mut seen: Vec<usize> = Vec::new();
        for b in &s.0 {
            match kb(self.cn.k(*b)) {
                Kb::Zero => {}
                Kb::Net(n) => {
                    let ci = *self.driver.get(&n)?;
                    if seen.contains(&ci) {
                        continue;
                    }
                    seen.push(ci);
                    for g in self.node(ci)? {
                        if !acc.iter().any(|h| h.0 == g.0) {
                            acc.push(g);
                        }
                    }
                    if acc.len() > GUARD_MAX {
                        return None;
                    }
                }
                _ => return None,
            }
        }
        Some(acc)
    }

    fn node(&mut self, ci: usize) -> Option<Vec<(usize, Bit)>> {
        if let Some(r) = self.memo.get(&ci) {
            return r.clone();
        }
        self.memo.insert(ci, None);
        let m = self.m;
        let r = match &m.cells[ci] {
            // zero unless one of either side's guards is 1, exclusive or not
            Cell::Binary { op: Op::Add | Op::Or, a, b, signed: false, .. } => match (self.operand(a), self.operand(b)) {
                (Some(mut x), Some(y)) => {
                    for g in y {
                        if !x.iter().any(|h| h.0 == g.0) {
                            x.push(g);
                        }
                    }
                    (x.len() <= GUARD_MAX).then_some(x)
                }
                _ => None,
            },
            Cell::Binary { op: Op::Mul | Op::And, .. } => self.term(ci),
            _ => None,
        };
        self.memo.insert(ci, r.clone());
        r
    }
}

/// A canonical bit, hashable.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
enum Kb {
    Net(usize),
    Zero,
    One,
    Other,
}

fn kb(k: K) -> Kb {
    match k {
        K::Net(n) => Kb::Net(n),
        K::Const(Bit::Zero) => Kb::Zero,
        K::Const(Bit::One) => Kb::One,
        K::Const(_) => Kb::Other,
    }
}

/// What is known of a one-hot value: the compared word (`None` while only
/// zeros are known) and the constants it is compared against.
#[derive(Clone, Debug, PartialEq)]
struct OneHot {
    sel: Option<Vec<Kb>>,
    keys: BTreeSet<Vec<bool>>,
    terms: usize,
}

impl OneHot {
    fn zero() -> Self {
        OneHot { sel: None, keys: BTreeSet::new(), terms: 0 }
    }

    /// Both together, if over the same word; `disjoint` also asks that no
    /// key is shared.
    fn join(mut self, o: &OneHot, disjoint: bool) -> Option<OneHot> {
        match (&self.sel, &o.sel) {
            (Some(a), Some(b)) if a != b => return None,
            (None, Some(b)) => self.sel = Some(b.clone()),
            _ => {}
        }
        if disjoint && !self.keys.is_disjoint(&o.keys) {
            return None;
        }
        self.keys.extend(o.keys.iter().cloned());
        self.terms += o.terms;
        Some(self)
    }
}

struct Ctx<'a> {
    m: &'a Module,
    cn: Canon,
    /// Canonical net -> the cell whose output it is (mul, and, add, or, eq).
    driver: HashMap<usize, usize>,
    memo: HashMap<usize, Option<OneHot>>,
}

impl Ctx<'_> {
    fn k(&mut self, b: Bit) -> Kb {
        kb(self.cn.k(b))
    }

    /// `s == k` with one constant side: the word and the key (`None` when
    /// the constant has bits beyond the word, so it never matches).
    fn compare(&mut self, net: usize) -> Option<(Vec<Kb>, Option<Vec<bool>>)> {
        let ci = *self.driver.get(&net)?;
        let m = self.m;
        let Cell::Binary { op: Op::Eq, a, b, signed: false, y } = &m.cells[ci] else {
            return None;
        };
        if y.0.first().map(|b| self.k(*b)) != Some(Kb::Net(net)) {
            return None;
        }
        let konst = |me: &mut Self, s: &SigSpec| -> Option<Vec<bool>> {
            s.0.iter()
                .map(|b| match me.k(*b) {
                    Kb::Zero => Some(false),
                    Kb::One => Some(true),
                    _ => None,
                })
                .collect()
        };
        let (x, key) = match (konst(self, a), konst(self, b)) {
            (None, Some(k)) => (a, k),
            (Some(k), None) => (b, k),
            _ => return None,
        };
        let word: Vec<Kb> = x.0.iter().map(|b| self.k(*b)).collect();
        let w = word.len();
        let key = if key.iter().skip(w).any(|v| *v) {
            None
        } else {
            let mut v: Vec<bool> = key.into_iter().take(w).collect();
            v.resize(w, false);
            Some(v)
        };
        Some((word, key))
    }

    /// The selection term `ci` is, if it is one.
    fn term(&mut self, ci: usize) -> Option<OneHot> {
        let m = self.m;
        let (a, b, signed, op) = match &m.cells[ci] {
            Cell::Binary { op: op @ (Op::Mul | Op::And), a, b, signed, .. } => (a.clone(), b.clone(), *signed, *op),
            _ => return None,
        };
        if signed {
            return None;
        }
        // the select side: one bit, zero-extended (mul), or one bit
        // replicated (and)
        let select = |me: &mut Self, s: &SigSpec| -> Option<usize> {
            let first = me.k(*s.0.first()?);
            let Kb::Net(n) = first else { return None };
            let ok = match op {
                Op::Mul => s.0[1..].iter().all(|b| me.k(*b) == Kb::Zero),
                _ => s.0.iter().all(|b| me.k(*b) == first),
            };
            ok.then_some(n)
        };
        for side in [&a, &b] {
            if let Some(n) = select(self, side)
                && let Some((word, key)) = self.compare(n)
            {
                let mut keys = BTreeSet::new();
                keys.extend(key);
                return Some(OneHot { sel: Some(word), keys, terms: 1 });
            }
        }
        None
    }

    /// What an operand is: constant zeros and bits of one-hot cells.
    fn operand(&mut self, s: &SigSpec) -> Option<OneHot> {
        let mut acc = OneHot::zero();
        let mut seen: Vec<usize> = Vec::new();
        for b in &s.0 {
            match self.k(*b) {
                Kb::Zero => {}
                Kb::Net(n) => {
                    let ci = *self.driver.get(&n)?;
                    if seen.contains(&ci) {
                        continue;
                    }
                    seen.push(ci);
                    let h = self.node(ci)?;
                    acc = acc.join(&h, false)?;
                }
                _ => return None,
            }
        }
        Some(acc)
    }

    fn node(&mut self, ci: usize) -> Option<OneHot> {
        if let Some(r) = self.memo.get(&ci) {
            return r.clone();
        }
        // in progress: a loop through these cells is not one-hot
        self.memo.insert(ci, None);
        let m = self.m;
        let r = match &m.cells[ci] {
            Cell::Binary { op: Op::Add | Op::Or, a, b, signed: false, .. } => {
                let (a, b) = (a.clone(), b.clone());
                match (self.operand(&a), self.operand(&b)) {
                    (Some(x), Some(y)) => x.join(&y, true),
                    _ => None,
                }
            }
            Cell::Binary { op: Op::Mul | Op::And, .. } => self.term(ci),
            _ => None,
        };
        self.memo.insert(ci, r.clone());
        r
    }
}

/// Turn every one-hot `$add` into `$or`.
#[hot]
pub fn rewrite(m: &mut Module) -> OneHotStats {
    let mut st = OneHotStats::default();
    let mr: &Module = m;
    let mut cx = Ctx {
        m: mr,
        cn: Canon::new(mr),
        driver: HashMap::new(),
        memo: HashMap::new(),
    };
    for (ci, c) in mr.cells.iter().enumerate() {
        if let Cell::Binary { op: Op::Mul | Op::And | Op::Add | Op::Or | Op::Eq, y, .. } = c {
            for b in &y.0 {
                if let K::Net(n) = cx.cn.k(*b) {
                    cx.driver.entry(n).or_insert(ci);
                }
            }
        }
    }
    let mut change: Vec<usize> = Vec::new();
    for (ci, c) in mr.cells.iter().enumerate() {
        if let Cell::Binary { op: Op::Add, y, .. } = c
            && let Some(h) = cx.node(ci)
            && h.sel.is_some()
        {
            change.push(ci);
            st.bits += y.0.len();
            st.terms += h.terms;
        }
    }
    drop(cx);

    // The guard rule, for the adds the key rule left.
    let done: HashSet<usize> = change.iter().copied().collect();
    let mut gx = Guards {
        m: mr,
        cn: Canon::new(mr),
        driver: HashMap::new(),
        memo: HashMap::new(),
    };
    for (ci, c) in mr.cells.iter().enumerate() {
        if let Cell::Binary { op: Op::Mul | Op::And | Op::Add | Op::Or, y, .. } = c {
            for b in &y.0 {
                if let K::Net(n) = gx.cn.k(*b) {
                    gx.driver.entry(n).or_insert(ci);
                }
            }
        }
    }
    let mut ex = crate::exclusive::Exclusive::new(mr);
    for (ci, c) in mr.cells.iter().enumerate() {
        if done.contains(&ci) {
            continue;
        }
        let Cell::Binary { op: Op::Add, a, b, signed: false, y } = c else { continue };
        let (Some(ga), Some(gb)) = (gx.operand(a), gx.operand(b)) else { continue };
        if ga.is_empty() || gb.is_empty() {
            continue;
        }
        let xa: Vec<Bit> = ga.iter().map(|g| g.1).collect();
        let xb: Vec<Bit> = gb.iter().map(|g| g.1).collect();
        if ex.exclusive(&xa, &xb) {
            change.push(ci);
            st.guarded += 1;
            st.bits += y.0.len();
            st.terms += ga.len() + gb.len();
        }
    }
    st.sat_calls = ex.stats.calls;
    st.sat_proven = ex.stats.proven;
    drop(ex);
    drop(gx);
    for ci in change {
        if let Cell::Binary { op, .. } = &mut m.cells[ci] {
            *op = Op::Or;
            st.adds += 1;
        }
    }
    st
}

/// One-hot selection sums (`(s == k) * v + ...`) to ORs.
pub struct OneHotSums;

impl crate::stage::Pass for OneHotSums {
    const NAME: &'static str = "onehot";
    const EMITS: &'static [crate::caps::Mode] = &[];
    type Stats = OneHotStats;
    fn enabled(p: &crate::policy::PassSet) -> bool {
        p.onehot
    }
    fn json(s: &OneHotStats) -> serde_json::Value {
        serde_json::json!({ "adds": s.adds, "bits": s.bits, "terms": s.terms, "guarded": s.guarded,
                            "sat_calls": s.sat_calls, "sat_proven": s.sat_proven })
    }
}

impl crate::stage::RtlPass for OneHotSums {
    fn run(&self, m: &mut crate::stage::Rtl) -> OneHotStats {
        rewrite(&mut m.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxibit_hdl::netlist::Signal;

    fn sig(m: &mut Module, name: &str, w: u32) -> SigSpec {
        let id = m.add_signal(Signal { name: name.into(), width: w, signed: false, port: None, init: None });
        SigSpec::of(id, w)
    }

    fn bin(m: &mut Module, op: Op, a: SigSpec, b: SigSpec, w: u32) -> SigSpec {
        let y = m.wire(w);
        m.cells.push(Cell::Binary { op, a, b, signed: false, y: y.clone() });
        y
    }

    /// `(s == k0) * v0 + (s == k1) * v1 + ...`, left to right.
    fn selection(m: &mut Module, s: &SigSpec, keys: &[u64], w: u32) -> SigSpec {
        let mut acc: Option<SigSpec> = None;
        for (i, &k) in keys.iter().enumerate() {
            let v = sig(m, &format!("v{i}"), w);
            let e = bin(m, Op::Eq, s.clone(), SigSpec::constant(k, s.width()), 1);
            let t = bin(m, Op::Mul, e, v, w);
            acc = Some(match acc {
                None => t,
                Some(a) => bin(m, Op::Add, a, t, w),
            });
        }
        acc.expect("terms")
    }

    fn ops(m: &Module) -> Vec<Op> {
        m.cells
            .iter()
            .filter_map(|c| match c {
                Cell::Binary { op: op @ (Op::Add | Op::Or), .. } => Some(*op),
                _ => None,
            })
            .collect()
    }

    #[test]
    fn distinct_keys_become_or() {
        let mut m = Module::new("t");
        let s = sig(&mut m, "s", 2);
        selection(&mut m, &s, &[0, 1, 2, 3], 8);
        let st = rewrite(&mut m);
        assert_eq!(st.adds, 3);
        assert_eq!(st.terms, 4 + 3 + 2);
        assert!(ops(&m).iter().all(|o| *o == Op::Or));
    }

    #[test]
    fn a_repeated_key_stays_an_add() {
        let mut m = Module::new("t");
        let s = sig(&mut m, "s", 2);
        selection(&mut m, &s, &[0, 1, 1], 8);
        rewrite(&mut m);
        // the first add (keys 0 and 1) is safe, the second (1 again) is not
        assert_eq!(ops(&m), vec![Op::Or, Op::Add]);
    }

    #[test]
    fn different_words_stay_adds() {
        let mut m = Module::new("t");
        let s = sig(&mut m, "s", 2);
        let r = sig(&mut m, "r", 2);
        let a = selection(&mut m, &s, &[0], 8);
        let b = selection(&mut m, &r, &[1], 8);
        bin(&mut m, Op::Add, a, b, 8);
        rewrite(&mut m);
        assert_eq!(ops(&m), vec![Op::Add]);
    }

    #[test]
    fn zero_extended_select() {
        // `(s == k) * v` with the compare widened to the product's width
        let mut m = Module::new("t");
        let s = sig(&mut m, "s", 1);
        let mut terms = Vec::new();
        for k in 0..2 {
            let v = sig(&mut m, &format!("v{k}"), 8);
            let e = bin(&mut m, Op::Eq, s.clone(), SigSpec::constant(k, 1), 1);
            let wide = e.extend(8, false);
            terms.push(bin(&mut m, Op::Mul, wide, v, 8));
        }
        bin(&mut m, Op::Add, terms[0].clone(), terms[1].clone(), 9);
        assert_eq!(rewrite(&mut m).adds, 1);
    }

    /// `b * x + bnot(b) * y`: complementary guards.
    #[test]
    fn complementary_guards_become_or() {
        let mut m = Module::new("t");
        let b = sig(&mut m, "b", 1);
        let nb = m.wire(1);
        m.cells.push(Cell::Unary { op: Op::Not, a: b.clone(), a_signed: false, y: nb.clone() });
        let x = sig(&mut m, "x", 8);
        let y = sig(&mut m, "y", 8);
        let t0 = bin(&mut m, Op::Mul, b.extend(8, false), x, 8);
        let t1 = bin(&mut m, Op::Mul, nb.extend(8, false), y, 8);
        bin(&mut m, Op::Add, t0, t1, 8);
        let st = rewrite(&mut m);
        assert_eq!((st.adds, st.guarded), (1, 1));
        assert_eq!(ops(&m), vec![Op::Or]);
    }

    /// Two independent selects can both be 1: the sum stays an add.
    #[test]
    fn independent_guards_stay_add() {
        let mut m = Module::new("t");
        let p = sig(&mut m, "p", 1);
        let q = sig(&mut m, "q", 1);
        let x = sig(&mut m, "x", 8);
        let y = sig(&mut m, "y", 8);
        let t0 = bin(&mut m, Op::Mul, p.extend(8, false), x, 8);
        let t1 = bin(&mut m, Op::Mul, q.extend(8, false), y, 8);
        bin(&mut m, Op::Add, t0, t1, 8);
        let st = rewrite(&mut m);
        assert_eq!(st.adds, 0);
        assert_eq!(ops(&m), vec![Op::Add]);
    }

    /// Priority-derived selects: `p` and `bnot(p) & q` never hold together.
    #[test]
    fn priority_guards_become_or() {
        let mut m = Module::new("t");
        let p = sig(&mut m, "p", 1);
        let q = sig(&mut m, "q", 1);
        let np = m.wire(1);
        m.cells.push(Cell::Unary { op: Op::Not, a: p.clone(), a_signed: false, y: np.clone() });
        let g1 = bin(&mut m, Op::And, np, q, 1);
        let x = sig(&mut m, "x", 8);
        let y = sig(&mut m, "y", 8);
        let t0 = bin(&mut m, Op::Mul, p.extend(8, false), x, 8);
        let t1 = bin(&mut m, Op::Mul, g1.extend(8, false), y, 8);
        bin(&mut m, Op::Add, t0, t1, 8);
        assert_eq!(rewrite(&mut m).guarded, 1);
    }

    #[test]
    fn plain_adds_are_left() {
        let mut m = Module::new("t");
        let a = sig(&mut m, "a", 8);
        let b = sig(&mut m, "b", 8);
        bin(&mut m, Op::Add, a, b, 8);
        assert_eq!(rewrite(&mut m).adds, 0);
    }
}
