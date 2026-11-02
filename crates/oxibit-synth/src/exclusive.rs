//! Proving netlist bits are never 1 together, by SAT over their
//! combinational cones: oxibit's counterpart of the check yosys's
//! `memory_share` makes before it merges write ports.
//!
//! A cone is encoded bit by bit (Tseitin) through the cells whose bit
//! function is local: Not, LogicNot, the reductions, And / Or / Xor /
//! Xnor, LogicAnd / LogicOr, Eq / Ne and Mux. Every other driver (flops,
//! ports, memory reads, arithmetic, ordered compares, shifts) and any X is
//! a free variable. Free variables only add behaviours, so UNSAT means the
//! bits are exclusive in every state, reachable or not: the answer is
//! sound, and incomplete where exclusivity rests on arithmetic or on
//! reachability. A cone past `node_limit` bits, or a call past the
//! conflict budget, answers "not proven".

use std::cell::Cell as StdCell;
use std::collections::HashMap;

use batsat::{Callbacks, Lit as SLit, SolverInterface, lbool};
use oxibit_hdl::netlist::{Bit, Cell, Module, Op, SigSpec};

use crate::dsp::{Canon, K};

/// Conflict budget through the callbacks: each learnt clause is a conflict.
struct Budget {
    conflicts: StdCell<u64>,
    limit: StdCell<u64>,
}

impl Callbacks for Budget {
    fn on_new_clause(&mut self, _c: &[SLit], src: batsat::ClauseKind) {
        if matches!(src, batsat::ClauseKind::Learnt) {
            self.conflicts.set(self.conflicts.get() + 1);
        }
    }
    fn stop(&self) -> bool {
        self.conflicts.get() >= self.limit.get()
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct ExclusiveStats {
    pub calls: usize,
    pub proven: usize,
    pub refuted: usize,
    pub unknown: usize,
    pub cone_bits: usize,
}

pub struct Exclusive<'m> {
    m: &'m Module,
    cn: Canon,
    /// Canonical bit -> (cell index, output bit) for cells encoded here.
    drv: HashMap<usize, (usize, usize)>,
    solver: batsat::Solver<Budget>,
    lit: HashMap<usize, SLit>,
    t: SLit,
    nodes: usize,
    depth: usize,
    pub node_limit: usize,
    pub conflict_limit: u64,
    pub stats: ExclusiveStats,
}

impl<'m> Exclusive<'m> {
    pub fn new(m: &'m Module) -> Self {
        let mut cn = Canon::new(m);
        let mut drv = HashMap::new();
        for (ci, c) in m.cells.iter().enumerate() {
            let y = match c {
                Cell::Unary { y, .. } | Cell::Binary { y, .. } | Cell::Mux { y, .. } => y,
                _ => continue,
            };
            for (i, b) in y.0.iter().enumerate() {
                if let K::Net(r) = cn.k(*b) {
                    drv.insert(r, (ci, i));
                }
            }
        }
        let mut solver = batsat::Solver::new(
            Default::default(),
            Budget {
                conflicts: StdCell::new(0),
                limit: StdCell::new(u64::MAX),
            },
        );
        let t = SLit::new(solver.new_var_default(), true);
        solver.add_clause_reuse(&mut vec![t]);
        Exclusive {
            m,
            cn,
            drv,
            solver,
            lit: HashMap::new(),
            t,
            nodes: 0,
            depth: 0,
            node_limit: 200_000,
            conflict_limit: 20_000,
            stats: ExclusiveStats::default(),
        }
    }

    fn fresh(&mut self) -> SLit {
        SLit::new(self.solver.new_var_default(), true)
    }

    fn and2(&mut self, a: SLit, b: SLit) -> SLit {
        let f = !self.t;
        if a == f || b == f || a == !b {
            return f;
        }
        if a == self.t || a == b {
            return b;
        }
        if b == self.t {
            return a;
        }
        let o = self.fresh();
        self.solver.add_clause_reuse(&mut vec![!o, a]);
        self.solver.add_clause_reuse(&mut vec![!o, b]);
        self.solver.add_clause_reuse(&mut vec![o, !a, !b]);
        o
    }

    fn or2(&mut self, a: SLit, b: SLit) -> SLit {
        !self.and2(!a, !b)
    }

    fn xor2(&mut self, a: SLit, b: SLit) -> SLit {
        let f = !self.t;
        if a == f {
            return b;
        }
        if b == f {
            return a;
        }
        if a == self.t {
            return !b;
        }
        if b == self.t {
            return !a;
        }
        if a == b {
            return f;
        }
        if a == !b {
            return self.t;
        }
        let o = self.fresh();
        self.solver.add_clause_reuse(&mut vec![!o, a, b]);
        self.solver.add_clause_reuse(&mut vec![!o, !a, !b]);
        self.solver.add_clause_reuse(&mut vec![o, !a, b]);
        self.solver.add_clause_reuse(&mut vec![o, a, !b]);
        o
    }

    fn mux2(&mut self, s: SLit, a: SLit, b: SLit) -> SLit {
        // s ? b : a
        let x = self.and2(s, b);
        let y = self.and2(!s, a);
        self.or2(x, y)
    }

    fn any(&mut self, v: &[SLit]) -> SLit {
        let mut acc = !self.t;
        for x in v {
            acc = self.or2(acc, *x);
        }
        acc
    }

    /// Bit `i` of a cell operand extended to any width.
    fn ext(v: &SigSpec, i: usize, signed: bool) -> Bit {
        match v.0.get(i) {
            Some(b) => *b,
            None if signed => v.0.last().copied().unwrap_or(Bit::Zero),
            None => Bit::Zero,
        }
    }

    fn word(&mut self, v: &SigSpec) -> Option<Vec<SLit>> {
        v.0.iter().map(|b| self.bit(*b)).collect()
    }

    /// The literal for one netlist bit; `None` past the node limit.
    pub fn bit(&mut self, b: Bit) -> Option<SLit> {
        let r = match self.cn.k(b) {
            K::Const(Bit::Zero) => return Some(!self.t),
            K::Const(Bit::One) => return Some(self.t),
            K::Const(_) => return Some(self.fresh()),
            K::Net(r) => r,
        };
        if let Some(l) = self.lit.get(&r) {
            return Some(*l);
        }
        self.nodes += 1;
        if self.nodes > self.node_limit {
            return None;
        }
        // deep cones give up rather than exhaust the stack
        if self.depth > 20_000 {
            return None;
        }
        self.depth += 1;
        let l = match self.drv.get(&r).copied() {
            Some((ci, i)) => self.cell_bit(ci, i),
            None => Some(self.fresh()),
        };
        self.depth -= 1;
        let l = l?;
        self.lit.insert(r, l);
        self.stats.cone_bits += 1;
        Some(l)
    }

    fn cell_bit(&mut self, ci: usize, i: usize) -> Option<SLit> {
        let m = self.m;
        let f = !self.t;
        Some(match &m.cells[ci] {
            Cell::Unary { op, a, a_signed, .. } => match op {
                Op::Not => !self.bit(Self::ext(a, i, *a_signed))?,
                Op::LogicNot | Op::ReduceAnd | Op::ReduceOr | Op::ReduceXor if i > 0 => f,
                Op::LogicNot => {
                    let v = self.word(a)?;
                    !self.any(&v)
                }
                Op::ReduceOr => {
                    let v = self.word(a)?;
                    self.any(&v)
                }
                Op::ReduceAnd => {
                    let v = self.word(a)?;
                    let n: Vec<SLit> = v.iter().map(|x| !*x).collect();
                    !self.any(&n)
                }
                Op::ReduceXor => {
                    let v = self.word(a)?;
                    let mut acc = f;
                    for x in v {
                        acc = self.xor2(acc, x);
                    }
                    acc
                }
                _ => self.fresh(),
            },
            Cell::Binary { op, a, b, signed, .. } => match op {
                Op::And | Op::Or | Op::Xor | Op::Xnor => {
                    let x = self.bit(Self::ext(a, i, *signed))?;
                    let y = self.bit(Self::ext(b, i, *signed))?;
                    match op {
                        Op::And => self.and2(x, y),
                        Op::Or => self.or2(x, y),
                        Op::Xor => self.xor2(x, y),
                        _ => !self.xor2(x, y),
                    }
                }
                Op::LogicAnd | Op::LogicOr | Op::Eq | Op::Ne if i > 0 => f,
                Op::LogicAnd | Op::LogicOr => {
                    let va = self.word(a)?;
                    let vb = self.word(b)?;
                    let x = self.any(&va);
                    let y = self.any(&vb);
                    if *op == Op::LogicAnd { self.and2(x, y) } else { self.or2(x, y) }
                }
                Op::Eq | Op::Ne => {
                    let w = a.0.len().max(b.0.len());
                    let mut diff = Vec::with_capacity(w);
                    for k in 0..w {
                        let x = self.bit(Self::ext(a, k, *signed))?;
                        let y = self.bit(Self::ext(b, k, *signed))?;
                        diff.push(self.xor2(x, y));
                    }
                    let ne = self.any(&diff);
                    if *op == Op::Eq { !ne } else { ne }
                }
                _ => self.fresh(),
            },
            Cell::Mux { a, b, s, .. } => {
                let sv = self.bit(s.0[0])?;
                let x = self.bit(a.0.get(i).copied().unwrap_or(Bit::Zero))?;
                let y = self.bit(b.0.get(i).copied().unwrap_or(Bit::Zero))?;
                self.mux2(sv, x, y)
            }
            _ => self.fresh(),
        })
    }

    /// `true` when no assignment makes some bit of `x` and some bit of `y`
    /// 1 at once (proven within the limits).
    pub fn exclusive(&mut self, x: &[Bit], y: &[Bit]) -> bool {
        self.stats.calls += 1;
        let (Some(vx), Some(vy)) = (self.word(&SigSpec(x.to_vec())), self.word(&SigSpec(y.to_vec()))) else {
            self.stats.unknown += 1;
            return false;
        };
        let ax = self.any(&vx);
        let ay = self.any(&vy);
        self.solver.cb_mut().conflicts.set(0);
        self.solver.cb_mut().limit.set(self.conflict_limit);
        let r = self.solver.solve_limited(&[ax, ay]);
        if r == lbool::FALSE {
            self.stats.proven += 1;
            true
        } else {
            if r == lbool::TRUE {
                self.stats.refuted += 1;
            } else {
                self.stats.unknown += 1;
            }
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn eqk(m: &mut Module, s: &SigSpec, k: u64) -> Bit {
        let y = m.wire(1);
        m.cells.push(Cell::Binary {
            op: Op::Eq,
            a: s.clone(),
            b: SigSpec::constant(k, s.width()),
            signed: false,
            y: y.clone(),
        });
        y.0[0]
    }

    #[test]
    fn different_states_are_exclusive() {
        let mut m = Module::new("t");
        let state = m.wire(3);
        let a = eqk(&mut m, &state, 1);
        let b = eqk(&mut m, &state, 2);
        let mut ex = Exclusive::new(&m);
        assert!(ex.exclusive(&[a], &[b]));
    }

    #[test]
    fn same_state_is_not() {
        let mut m = Module::new("t");
        let state = m.wire(3);
        let a = eqk(&mut m, &state, 5);
        let b = eqk(&mut m, &state, 5);
        let mut ex = Exclusive::new(&m);
        assert!(!ex.exclusive(&[a], &[b]));
    }

    #[test]
    fn independent_flags_are_not() {
        let mut m = Module::new("t");
        let a = m.wire(1).0[0];
        let b = m.wire(1).0[0];
        let mut ex = Exclusive::new(&m);
        assert!(!ex.exclusive(&[a], &[b]));
    }

    #[test]
    fn masked_by_a_shared_condition() {
        // a = go & (s == 0), b = go & (s == 3): exclusive through the compare
        let mut m = Module::new("t");
        let s = m.wire(2);
        let go = m.wire(1);
        let e0 = eqk(&mut m, &s, 0);
        let e3 = eqk(&mut m, &s, 3);
        let a = m.wire(1);
        let b = m.wire(1);
        m.cells.push(Cell::Binary { op: Op::And, a: go.clone(), b: SigSpec(vec![e0]), signed: false, y: a.clone() });
        m.cells.push(Cell::Binary { op: Op::And, a: go, b: SigSpec(vec![e3]), signed: false, y: b.clone() });
        let mut ex = Exclusive::new(&m);
        assert!(ex.exclusive(&a.0, &b.0));
    }
}
