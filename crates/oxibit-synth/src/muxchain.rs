//! Priority mux chains over one selector to parallel (one-hot) form
//! (yosys `muxpack` / `$pmux`).
//!
//! `if (x == 0) v = A; else if (x == 1) v = B; else if ...` elaborates to
//! a chain of `$mux` cells, one level per arm, whose selects are `$eq`
//! cells comparing the same word `x` against constants. Distinct constants
//! make the selects mutually exclusive, so priority does not matter and
//! the chain equals `OR_k (eq_k & v_k) | (none ? default : 0)`: log depth
//! instead of one level per arm, and with constant arms (a lookup table)
//! each output bit is an OR of decoded selector values the mapper packs
//! into a few LUT levels. Arms whose constant repeats an earlier one are
//! unreachable and dropped; chains shorter than `MIN_ARMS` are left alone.

use std::collections::{HashMap, HashSet};

use oxibit_hdl::netlist::{Bit, Cell, Module, Op, SigSpec};
use oxibit_prof::hot;

use crate::dsp::{Canon, K};

pub const MIN_ARMS: usize = 4;

#[derive(Debug, Default, Clone)]
pub struct MuxChainStats {
    pub chains: usize,
    pub arms: usize,
    pub dropped_arms: usize,
}

/// A mux whose select is `x == const`.
struct Arm {
    cell: usize,
    /// Canonical bits of the compared word.
    x: Vec<K>,
    /// The constant, extended or truncated to `x`'s width; `None` when it
    /// can never equal `x` (bits beyond `x` that are not zero).
    key: Option<Vec<bool>>,
    sel: Bit,
}

fn konst(cn: &mut Canon, s: &SigSpec) -> Option<Vec<bool>> {
    s.0.iter()
        .map(|b| match cn.k(*b) {
            K::Const(Bit::Zero) => Some(false),
            K::Const(Bit::One) => Some(true),
            _ => None,
        })
        .collect()
}

#[hot]
pub fn pack(m: &mut Module) -> MuxChainStats {
    let mut st = MuxChainStats::default();
    let mut cn = Canon::new(m);
    // Drivers of canonical nets: `$eq` outputs and `$mux` outputs.
    let mut eq_of: HashMap<usize, usize> = HashMap::new();
    let mut mux_of: HashMap<usize, usize> = HashMap::new();
    for (ci, c) in m.cells.iter().enumerate() {
        match c {
            Cell::Binary { op: Op::Eq, y, .. } => {
                if let K::Net(n) = cn.k(y.0[0]) {
                    eq_of.insert(n, ci);
                }
            }
            Cell::Mux { y, .. } => {
                if let Some(K::Net(n)) = y.0.first().map(|b| cn.k(*b)) {
                    mux_of.insert(n, ci);
                }
            }
            _ => {}
        }
    }
    // Uses of every canonical net as a cell input or an output port.
    let mut uses: HashMap<usize, u32> = HashMap::new();
    {
        let mut count = |s: &SigSpec, cn: &mut Canon| {
            for b in &s.0 {
                if let K::Net(n) = cn.k(*b) {
                    *uses.entry(n).or_default() += 1;
                }
            }
        };
        for c in &m.cells {
            match c {
                Cell::Unary { a, .. } => count(a, &mut cn),
                Cell::Binary { a, b, .. } => {
                    count(a, &mut cn);
                    count(b, &mut cn);
                }
                Cell::Mux { a, b, s, .. } => {
                    count(a, &mut cn);
                    count(b, &mut cn);
                    count(s, &mut cn);
                }
                Cell::Dff { clk, d, .. } => {
                    count(clk, &mut cn);
                    count(d, &mut cn);
                }
                Cell::Adff { clk, arst, d, .. } => {
                    count(clk, &mut cn);
                    count(arst, &mut cn);
                    count(d, &mut cn);
                }
                Cell::Mem(mem) => {
                    for r in &mem.reads {
                        count(&r.addr, &mut cn);
                        if let Some((c, _)) = &r.clk {
                            count(c, &mut cn);
                        }
                    }
                    for w in &mem.writes {
                        count(&w.clk.0, &mut cn);
                        count(&w.addr, &mut cn);
                        count(&w.data, &mut cn);
                        count(&w.en, &mut cn);
                    }
                }
                Cell::Instance { conns, .. } => conns.values().for_each(|s| count(s, &mut cn)),
            }
        }
        for (i, s) in m.signals.iter().enumerate() {
            if s.port.is_some() {
                count(&SigSpec::of(oxibit_hdl::netlist::SigId(i as u32), s.width), &mut cn);
            }
        }
    }
    // Arms.
    let mut arm_of: HashMap<usize, Arm> = HashMap::new();
    for (ci, c) in m.cells.iter().enumerate() {
        let Cell::Mux { s, .. } = c else { continue };
        if s.0.len() != 1 {
            continue;
        }
        let K::Net(sn) = cn.k(s.0[0]) else { continue };
        let Some(&ei) = eq_of.get(&sn) else { continue };
        let Cell::Binary { a, b, signed: false, .. } = &m.cells[ei] else {
            continue;
        };
        let (x, k) = match (konst(&mut cn, a), konst(&mut cn, b)) {
            (None, Some(k)) => (a, k),
            (Some(k), None) => (b, k),
            _ => continue,
        };
        let xk: Vec<K> = x.0.iter().map(|b| cn.k(*b)).collect();
        let w = xk.len();
        let key = if k.iter().skip(w).any(|v| *v) {
            None
        } else {
            let mut v: Vec<bool> = k.iter().take(w).copied().collect();
            v.resize(w, false);
            Some(v)
        };
        arm_of.insert(
            ci,
            Arm {
                cell: ci,
                x: xk,
                key,
                sel: s.0[0],
            },
        );
    }
    // next[c]: the arm in c's else input, when that mux feeds only c.
    let mut next: HashMap<usize, usize> = HashMap::new();
    let mut has_prev: HashSet<usize> = HashSet::new();
    for (&ci, _) in &arm_of {
        let Cell::Mux { a, .. } = &m.cells[ci] else { unreachable!() };
        let Some(K::Net(n0)) = a.0.first().map(|b| cn.k(*b)) else { continue };
        let Some(&mi) = mux_of.get(&n0) else { continue };
        if mi == ci || !arm_of.contains_key(&mi) {
            continue;
        }
        let Cell::Mux { y: y2, .. } = &m.cells[mi] else { unreachable!() };
        if y2.width() != a.width() {
            continue;
        }
        let same = a.0.iter().zip(&y2.0).all(|(p, q)| {
            let (kp, kq) = (cn.k(*p), cn.k(*q));
            kp == kq && matches!(kq, K::Net(n) if uses.get(&n) == Some(&1))
        });
        if same {
            next.insert(ci, mi);
            has_prev.insert(mi);
        }
    }
    let mut drop = vec![false; m.cells.len()];
    let mut add: Vec<Cell> = Vec::new();
    let heads: Vec<usize> = arm_of.keys().copied().filter(|c| !has_prev.contains(c)).collect();
    for head in heads {
        // The prefix of the chain comparing the same word.
        let mut chain = vec![head];
        let mut seen = HashSet::from([head]);
        while let Some(&n) = next.get(chain.last().unwrap()) {
            if !seen.insert(n) || arm_of[&n].x != arm_of[&head].x {
                break;
            }
            chain.push(n);
        }
        if chain.len() < MIN_ARMS {
            continue;
        }
        let (Cell::Mux { y, .. }, Cell::Mux { a: default, .. }) = (&m.cells[head], &m.cells[*chain.last().unwrap()]) else {
            unreachable!()
        };
        let (y, default) = (y.clone(), default.clone());
        let w = y.width();
        let mut keys: HashSet<Vec<bool>> = HashSet::new();
        let mut terms: Vec<SigSpec> = Vec::new();
        let mut sels: Vec<Bit> = Vec::new();
        for &ci in &chain {
            drop[ci] = true;
            let arm = &arm_of[&ci];
            let Some(key) = &arm.key else {
                st.dropped_arms += 1;
                continue;
            };
            if !keys.insert(key.clone()) {
                st.dropped_arms += 1;
                continue;
            }
            let Cell::Mux { b, .. } = &m.cells[arm.cell] else { unreachable!() };
            let b = b.clone();
            let sel = arm.sel;
            let t = m.wire(w);
            add.push(Cell::Binary {
                op: Op::And,
                a: b,
                b: SigSpec(vec![sel; w as usize]),
                signed: false,
                y: t.clone(),
            });
            terms.push(t);
            sels.push(sel);
            st.arms += 1;
        }
        // Default when no select is set.
        let any = m.wire(1);
        add.push(Cell::Unary {
            op: Op::ReduceOr,
            a: SigSpec(sels),
            a_signed: false,
            y: any.clone(),
        });
        let d = m.wire(w);
        add.push(Cell::Mux {
            a: default,
            b: SigSpec::constant(0, w),
            s: any,
            y: d.clone(),
        });
        terms.push(d);
        // Balanced OR tree; the root drives the chain's output.
        while terms.len() > 1 {
            let mut nxt = Vec::with_capacity(terms.len().div_ceil(2));
            for pair in terms.chunks(2) {
                if pair.len() == 1 {
                    nxt.push(pair[0].clone());
                    continue;
                }
                let o = m.wire(w);
                add.push(Cell::Binary {
                    op: Op::Or,
                    a: pair[0].clone(),
                    b: pair[1].clone(),
                    signed: false,
                    y: o.clone(),
                });
                nxt.push(o);
            }
            terms = nxt;
        }
        m.connects.push((y, terms.pop().unwrap()));
        st.chains += 1;
    }
    let mut k = 0;
    m.cells.retain(|_| {
        let keep = !drop[k];
        k += 1;
        keep
    });
    m.cells.extend(add);
    st
}

pub struct MuxPack;

impl crate::stage::Pass for MuxPack {
    const NAME: &'static str = "muxpack";
    const EMITS: &'static [crate::caps::Mode] = &[];
    type Stats = MuxChainStats;
    fn enabled(p: &crate::policy::PassSet) -> bool {
        p.muxpack
    }
    fn json(s: &MuxChainStats) -> serde_json::Value {
        serde_json::json!({ "chains": s.chains, "arms": s.arms, "dropped_arms": s.dropped_arms })
    }
}

impl crate::stage::RtlPass for MuxPack {
    fn run(&self, m: &mut crate::stage::Rtl) -> MuxChainStats {
        pack(&mut m.0)
    }
}
