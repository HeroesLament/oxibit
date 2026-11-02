//! Prove AIG outputs constant: random simulation, then SAT sweeping
//! (fraiging) of each output's cone, then a SAT call per output.
//!
//! Simulation gives every variable a signature (W x 64 random patterns);
//! an output whose signature is not all-ones (or all-zeros, for the
//! constant asked) already has a counterexample. Sweeping walks a cone in
//! topological order, rebuilding it with structural hashing; a new node
//! whose signature matches an earlier node's (or its complement) is
//! checked equal by SAT under a conflict budget and merged when proven.
//! Equal nodes collapse, so an output that is constant collapses to a
//! constant or is proven by one cheap final call.
//!
//! Outputs go out in small groups to a pool of threads; each thread has its
//! own solver and keeps what it has swept across groups.
//!
//! "Failed" means refuted for this miter: with registers cut to inputs, the
//! counterexample may use register values the circuit never reaches, so a
//! sequential check decides those.

use std::cell::Cell;
use std::collections::HashMap;

use batsat::{Callbacks, Lit as SLit, SolverInterface, lbool};

use std::sync::Mutex;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

use oxibit_prof::{hot, hot_block};
use oxibit_report::{Event, Progress};
use serde_json::json;

use crate::aiger::Aig;

pub const W: usize = 16;

/// SAT calls per solver before it is recycled.
const RECYCLE: u64 = 64;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Verdict {
    Proven,
    Failed,
    Open,
}

/// Signatures of every variable: `sig[v * W .. (v + 1) * W]`.
#[hot]
pub fn simulate(g: &Aig, seed: u64) -> Vec<u64> {
    let n = g.num_vars();
    let mut sig = vec![0u64; n * W];
    let mut s = seed | 1;
    for v in 1..(g.first_and() as usize) {
        for w in 0..W {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            sig[v * W + w] = s;
        }
    }
    let first = g.first_and() as usize;
    for (k, &(a, b)) in g.ands.iter().enumerate() {
        let v = first + k;
        let (va, vb) = ((a >> 1) as usize, (b >> 1) as usize);
        let (ma, mb) = (if a & 1 == 1 { !0 } else { 0 }, if b & 1 == 1 { !0 } else { 0 });
        for w in 0..W {
            sig[v * W + w] = (sig[va * W + w] ^ ma) & (sig[vb * W + w] ^ mb);
        }
    }
    sig
}

fn lit_sig(sig: &[u64], l: u32, w: usize) -> u64 {
    let s = sig[(l >> 1) as usize * W + w];
    if l & 1 == 1 { !s } else { s }
}

/// Conflict budget through the callbacks: each learnt clause is a conflict.
struct Budget {
    conflicts: Cell<u64>,
    limit: Cell<u64>,
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

type Solver = batsat::Solver<Budget>;

struct Sweeper<'a> {
    g: &'a Aig,
    sig: &'a [u64],
    /// Representative literal of each original variable after sweeping
    /// (an original literal), when visited.
    repr: Vec<u32>,
    visited: Vec<bool>,
    strash: HashMap<(u32, u32), u32>,
    /// Signature class -> representative variables (normalised so the
    /// first pattern bit is 0).
    classes: HashMap<u64, Vec<u32>>,
    sat_var: Vec<Option<SLit>>,
    solver: Solver,
    /// SAT calls since the solver was last recycled.
    since_reset: u64,
    budget: u64,
    pub sat_calls: u64,
    pub merges: u64,
}

impl<'a> Sweeper<'a> {
    fn new(g: &'a Aig, sig: &'a [u64], budget: u64) -> Self {
        let n = g.num_vars();
        let mut s = Sweeper {
            g,
            sig,
            repr: (0..n as u32).map(|v| v << 1).collect(),
            visited: vec![false; n],
            strash: HashMap::new(),
            classes: HashMap::new(),
            sat_var: vec![None; n],
            solver: Self::fresh_solver(),
            since_reset: 0,
            budget,
            sat_calls: 0,
            merges: 0,
        };
        s.reset_solver();
        s
    }

    fn fresh_solver() -> Solver {
        Solver::new(
            Default::default(),
            Budget {
                conflicts: Cell::new(0),
                limit: Cell::new(u64::MAX),
            },
        )
    }

    /// Start over with an empty solver; cones are re-encoded on demand.
    /// Recycling keeps each call's clause database to the logic recent
    /// calls touched (retired activation literals and old cones otherwise
    /// pile up and every propagation pays for them).
    fn reset_solver(&mut self) {
        self.solver = Self::fresh_solver();
        self.sat_var.iter_mut().for_each(|x| *x = None);
        // Constant: a variable forced false.
        let f = self.solver.new_var_default();
        let fl = SLit::new(f, true);
        self.solver.add_clause_reuse(&mut vec![!fl]);
        self.sat_var[0] = Some(fl);
        self.since_reset = 0;
    }

    fn norm(&self, v: u32) -> (u64, bool) {
        let first = self.sig[v as usize * W] & 1 == 1;
        let mut h: u64 = 0xcbf2_9ce4_8422_2325;
        for w in 0..W {
            let s = self.sig[v as usize * W + w] ^ if first { !0 } else { 0 };
            h = (h ^ s).wrapping_mul(0x0100_0000_01b3);
        }
        (h, first)
    }

    fn same_sig(&self, a: u32, b: u32) -> Option<bool> {
        let (sa, sb) = (
            &self.sig[a as usize * W..(a as usize + 1) * W],
            &self.sig[b as usize * W..(b as usize + 1) * W],
        );
        if sa == sb {
            Some(false)
        } else if sa.iter().zip(sb).all(|(x, y)| *x == !*y) {
            Some(true)
        } else {
            None
        }
    }

    /// SAT literal of original literal `l`, encoding its cone (in the
    /// reduced graph) on demand.
    fn slit(&mut self, l: u32) -> SLit {
        let v = l >> 1;
        let mut stack = vec![v];
        while let Some(&x) = stack.last() {
            if self.sat_var[x as usize].is_some() {
                stack.pop();
                continue;
            }
            match self.reduced_fanins(x) {
                None => {
                    let nv = self.solver.new_var_default();
                    self.sat_var[x as usize] = Some(SLit::new(nv, true));
                    stack.pop();
                }
                Some((a, b)) => {
                    let (va, vb) = (a >> 1, b >> 1);
                    if self.sat_var[va as usize].is_none() {
                        stack.push(va);
                        continue;
                    }
                    if self.sat_var[vb as usize].is_none() {
                        stack.push(vb);
                        continue;
                    }
                    let la = self.sat_var[va as usize].unwrap() ^ (a & 1 == 1);
                    let lb = self.sat_var[vb as usize].unwrap() ^ (b & 1 == 1);
                    let nv = self.solver.new_var_default();
                    let o = SLit::new(nv, true);
                    self.solver.add_clause_reuse(&mut vec![!o, la]);
                    self.solver.add_clause_reuse(&mut vec![!o, lb]);
                    self.solver.add_clause_reuse(&mut vec![o, !la, !lb]);
                    self.sat_var[x as usize] = Some(o);
                    stack.pop();
                }
            }
        }
        self.sat_var[v as usize].unwrap() ^ (l & 1 == 1)
    }

    /// Fanins of a reduced-graph node (an original AND variable that is its
    /// own representative), already mapped through `repr`.
    fn reduced_fanins(&self, v: u32) -> Option<(u32, u32)> {
        let (a, b) = self.g.fanins(v)?;
        Some((self.map(a), self.map(b)))
    }

    fn map(&self, l: u32) -> u32 {
        self.repr[(l >> 1) as usize] ^ (l & 1)
    }

    /// Is `p == q` for all inputs? Some(true) proven, Some(false) refuted,
    /// None undecided within the budget.
    fn equal(&mut self, p: u32, q: u32, budget: u64) -> Option<bool> {
        if self.since_reset >= RECYCLE {
            self.reset_solver();
        }
        self.since_reset += 1;
        let (sp, sq) = (self.slit(p), self.slit(q));
        let t = SLit::new(self.solver.new_var_default(), true);
        // t -> p != q
        self.solver.add_clause_reuse(&mut vec![!t, sp, sq]);
        self.solver.add_clause_reuse(&mut vec![!t, !sp, !sq]);
        self.solver.cb_mut().conflicts.set(0);
        self.solver.cb_mut().limit.set(budget);
        self.sat_calls += 1;
        let r = self.solver.solve_limited(&[t]);
        // Retire the activation literal either way.
        self.solver.add_clause_reuse(&mut vec![!t]);
        if r == lbool::FALSE {
            Some(true)
        } else if r == lbool::TRUE {
            Some(false)
        } else {
            None
        }
    }

    /// Sweep original variable `v` (fanins already swept).
    fn visit(&mut self, v: u32) {
        let Some((a, b)) = self.g.fanins(v) else {
            self.visited[v as usize] = true;
            return;
        };
        let (a, b) = (self.map(a), self.map(b));
        let (a, b) = if a <= b { (a, b) } else { (b, a) };
        let r = if a == 0 || a == (b ^ 1) {
            Some(0)
        } else if a == 1 || a == b {
            Some(b)
        } else {
            self.strash.get(&(a, b)).copied()
        };
        self.visited[v as usize] = true;
        if let Some(r) = r {
            self.repr[v as usize] = r;
            return;
        }
        self.strash.insert((a, b), v << 1);
        // Constant by simulation: try to prove it.
        let all0 = (0..W).all(|w| self.sig[v as usize * W + w] == 0);
        let all1 = (0..W).all(|w| self.sig[v as usize * W + w] == !0);
        if all0 || all1 {
            let c = u32::from(all1);
            if self.equal(v << 1, c, self.budget) == Some(true) {
                self.repr[v as usize] = c;
                self.merges += 1;
                return;
            }
        }
        let (h, _) = self.norm(v);
        let cands = self.classes.get(&h).cloned().unwrap_or_default();
        for c in cands {
            if let Some(neg) = self.same_sig(v, c) {
                let cl = (c << 1) | u32::from(neg);
                if self.equal(v << 1, cl, self.budget) == Some(true) {
                    self.repr[v as usize] = cl;
                    self.merges += 1;
                    return;
                }
            }
        }
        self.classes.entry(h).or_default().push(v);
    }
}

pub struct Stats {
    pub sat_calls: u64,
    pub merges: u64,
}

/// Counters shared by the worker threads, reported as `progress` events
/// (phase "sweep") at most once a second.
struct Counters {
    groups: AtomicU64,
    groups_total: u64,
    outputs: AtomicU64,
    outputs_total: u64,
    proven: AtomicU64,
    failed: AtomicU64,
    open: AtomicU64,
    sat_calls: AtomicU64,
    merges: AtomicU64,
    last: Mutex<Instant>,
}

impl Counters {
    fn report(&self, rep: &dyn Progress, force: bool) {
        {
            let mut last = self.last.lock().unwrap();
            if !force && last.elapsed() < Duration::from_secs(1) {
                return;
            }
            *last = Instant::now();
        }
        let l = |a: &AtomicU64| a.load(Ordering::Relaxed);
        rep.emit(Event::progress(
            "sweep",
            json!({
                "groups": l(&self.groups), "groups_total": self.groups_total,
                "outputs": l(&self.outputs), "outputs_total": self.outputs_total,
                "proven": l(&self.proven), "failed": l(&self.failed), "open": l(&self.open),
                "sat_calls": l(&self.sat_calls), "merges": l(&self.merges),
            }),
        ));
    }
}

/// Prove each output equal to constant `want` (true: all ones). Returns a
/// verdict per output and summed statistics. Reports phases "simulate" and
/// "sweep" through `rep`, and stops early when it is cancelled (outputs not
/// reached stay open).
#[hot]
pub fn prove(g: &Aig, want: bool, threads: usize, sweep_budget: u64, final_budget: u64, rep: &dyn Progress) -> (Vec<Verdict>, Stats) {
    rep.emit(Event::Phase {
        phase: "simulate".into(),
        state: oxibit_report::PhaseState::Begin,
        ms: None,
    });
    let sig = simulate(g, 0x2545_f491_4f6c_dd1d);
    rep.emit(Event::Phase {
        phase: "simulate".into(),
        state: oxibit_report::PhaseState::End,
        ms: None,
    });
    let n_out = g.outputs.len();
    let mut verdict = vec![Verdict::Open; n_out];
    // Simulation verdicts first.
    let mut todo = Vec::new();
    for (i, &o) in g.outputs.iter().enumerate() {
        let ok = (0..W).all(|w| lit_sig(&sig, o, w) == if want { !0 } else { 0 });
        if ok { todo.push(i) } else { verdict[i] = Verdict::Failed }
    }
    // Small groups of neighbouring outputs (they tend to share logic),
    // handed out to threads as they free up. Each thread keeps its sweeper
    // across groups, so logic it already swept stays merged.
    let threads = threads.max(1).min(todo.len().max(1));
    let gsize = todo.len().div_ceil(threads * 16).clamp(1, 256);
    let groups: Vec<Vec<usize>> = todo.chunks(gsize).map(|c| c.to_vec()).collect();
    let next = std::sync::atomic::AtomicUsize::new(0);
    let refuted = (n_out - todo.len()) as u64;
    let ctr = Counters {
        groups: AtomicU64::new(0),
        groups_total: groups.len() as u64,
        outputs: AtomicU64::new(refuted),
        outputs_total: n_out as u64,
        proven: AtomicU64::new(0),
        failed: AtomicU64::new(refuted),
        open: AtomicU64::new(0),
        sat_calls: AtomicU64::new(0),
        merges: AtomicU64::new(0),
        last: Mutex::new(Instant::now()),
    };
    rep.emit(Event::Phase {
        phase: "sweep".into(),
        state: oxibit_report::PhaseState::Begin,
        ms: None,
    });
    let results: Vec<(Vec<(usize, Verdict)>, u64, u64)> = std::thread::scope(|sc| {
        let hs: Vec<_> = (0..threads)
            .map(|_| {
                let (sig, groups, next, ctr) = (&sig, &groups, &next, &ctr);
                sc.spawn(move || {
                    let mut sw = Sweeper::new(g, sig, sweep_budget);
                    let mut out = Vec::new();
                    let mut mark = vec![false; g.num_vars()];
                    loop {
                        if rep.cancelled() {
                            break;
                        }
                        let k = next.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                        let Some(grp) = groups.get(k) else { break };
                        // The group's cone not yet swept, in topological
                        // (index) order.
                        let mut cone = Vec::new();
                        let mut stack: Vec<u32> = grp.iter().map(|&i| g.outputs[i] >> 1).collect();
                        while let Some(v) = stack.pop() {
                            if sw.visited[v as usize] || std::mem::replace(&mut mark[v as usize], true) {
                                continue;
                            }
                            cone.push(v);
                            if let Some((a, b)) = g.fanins(v) {
                                stack.push(a >> 1);
                                stack.push(b >> 1);
                            }
                        }
                        cone.sort_unstable();
                        let (c0, m0) = (sw.sat_calls, sw.merges);
                        hot_block!("cec.sweep_group", {
                            for &v in &cone {
                                sw.visit(v);
                                mark[v as usize] = false;
                            }
                        });
                        for &i in grp {
                            let o = sw.map(g.outputs[i]);
                            let c = u32::from(want);
                            let v = if o == c {
                                Verdict::Proven
                            } else if o == (c ^ 1) {
                                Verdict::Failed
                            } else {
                                match sw.equal(o, c, final_budget) {
                                    Some(true) => Verdict::Proven,
                                    Some(false) => Verdict::Failed,
                                    None => Verdict::Open,
                                }
                            };
                            match v {
                                Verdict::Proven => &ctr.proven,
                                Verdict::Failed => &ctr.failed,
                                Verdict::Open => &ctr.open,
                            }
                            .fetch_add(1, Ordering::Relaxed);
                            ctr.outputs.fetch_add(1, Ordering::Relaxed);
                            out.push((i, v));
                            ctr.report(rep, false);
                        }
                        ctr.groups.fetch_add(1, Ordering::Relaxed);
                        ctr.sat_calls.fetch_add(sw.sat_calls - c0, Ordering::Relaxed);
                        ctr.merges.fetch_add(sw.merges - m0, Ordering::Relaxed);
                        ctr.report(rep, false);
                    }
                    (out, sw.sat_calls, sw.merges)
                })
            })
            .collect();
        hs.into_iter().map(|h| h.join().unwrap()).collect()
    });
    ctr.report(rep, true);
    rep.emit(Event::Phase {
        phase: "sweep".into(),
        state: oxibit_report::PhaseState::End,
        ms: None,
    });
    let mut st = Stats { sat_calls: 0, merges: 0 };
    for (out, c, m) in results {
        st.sat_calls += c;
        st.merges += m;
        for (i, v) in out {
            verdict[i] = v;
        }
    }
    (verdict, st)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// a & b vs !(!a | !b) built separately: equal; and a & b vs a: not.
    #[test]
    fn small_miter() {
        // vars: 1 a, 2 b; 3 = a&b; 4 = !a & !b; 5 = !4 & ... build xnor(3, !4') etc.
        // Output 0: x = (a&b) XNOR (a&b) rebuilt via 5 = a & b (duplicate).
        // Simpler: outputs are literals checked == 1.
        // 3 = a & b, 4 = b & a (strash-equal), 5 = 3 & !4 (always 0), out0 = !5 (always 1)
        // 6 = a & !b, out1 = !6 (not always 1)
        let g = Aig {
            inputs: 2,
            latches: 0,
            ands: vec![(2, 4), (4, 2), (6, 9), (2, 5)],
            outputs: vec![11, 13],
        };
        let (v, _) = prove(&g, true, 2, 1000, 10000, &oxibit_report::Silent);
        assert_eq!(v, vec![Verdict::Proven, Verdict::Failed]);
    }
}
