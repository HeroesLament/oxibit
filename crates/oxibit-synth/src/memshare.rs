//! Write ports that never write together, merged (yosys `memory_share`'s
//! write-port case, with a structural proof in place of its SAT one).
//!
//! RAM primitives have one write port, so a memory with two becomes flops
//! (`memflop`). Often the ports are exclusive by construction: picoBEAM's
//! tables have a working write under `!rst` and a clearing sweep under
//! `rst`. Two ports are exclusive when no assignment of the cut points of
//! the lowered AIG (flops, memory reads, ports) turns both enables on, so
//! the proof holds whatever state the design is in: first structurally
//! (one enable's AND tree holds a literal whose complement is in the
//! other's), else by oxibit-cec proving `!(a & b)` on the two enables'
//! cones. A pair the solver cannot settle within its budget counts as not
//! exclusive. A memory whose write ports are pairwise exclusive, on one clock
//! edge, each with one enable for all its bits, gets one write port: the
//! enables ORed, address and data multiplexed by which port is enabled.
//! Where none is, the merged port is disabled, as every original one was.
//!
//! The merged memory carries the attribute `oxibit_merged_writes` (the
//! port count it had), so the equivalence check can compare it in the form
//! both sides share (`scripts/cec_synth.sh` maps such memories to
//! registers on both sides).

use std::collections::{HashMap, HashSet};

use oxibit_hdl::netlist::{Bit, Cell, MemWrite, Module, Op, SigSpec};
use oxibit_prof::hot;

use crate::aig::{Aig, Lit, Node};
use crate::lib_ports::PrimPorts;
use crate::lower;

#[derive(Debug, Default, Clone)]
pub struct MemShareStats {
    /// Memories whose write ports were merged into one.
    pub merged: usize,
    /// Write ports removed.
    pub ports: usize,
    /// Multi-write memories left as they are, with the reason.
    pub left: Vec<(String, &'static str)>,
}

/// The leaves of the AND tree of `l` (at most `cap`; `None` past it).
fn conjuncts(g: &Aig, l: Lit, out: &mut HashSet<Lit>, cap: usize) -> Option<()> {
    if out.len() > cap {
        return None;
    }
    if !l.compl() {
        if let Node::And(a, b) = g.nodes[l.node() as usize] {
            conjuncts(g, a, out, cap)?;
            return conjuncts(g, b, out, cap);
        }
    }
    out.insert(l);
    Some(())
}

/// `a` and `b` are never both true, by a literal one's AND tree requires
/// and the other's forbids.
fn exclusive(g: &Aig, a: Lit, b: Lit) -> bool {
    const CAP: usize = 256;
    let (mut sa, mut sb) = (HashSet::new(), HashSet::new());
    if conjuncts(g, a, &mut sa, CAP).is_none() || conjuncts(g, b, &mut sb, CAP).is_none() {
        return false;
    }
    sa.contains(&Lit::FALSE) || sb.contains(&Lit::FALSE) || sa.iter().any(|l| sb.contains(&l.not())) || never_both(g, a, b)
}

/// oxibit-cec's proof that `a & b` is constant false over the cones of
/// `a` and `b`, their cut points free.
fn never_both(g: &Aig, a: Lit, b: Lit) -> bool {
    // The cone, in node order (topological: fanins come first).
    let mut seen = HashSet::new();
    let mut stack = vec![a.node(), b.node()];
    while let Some(n) = stack.pop() {
        if n == 0 || !seen.insert(n) {
            continue;
        }
        if let Node::And(x, y) = g.nodes[n as usize] {
            stack.push(x.node());
            stack.push(y.node());
        }
    }
    let mut nodes: Vec<u32> = seen.into_iter().collect();
    nodes.sort_unstable();
    let (inputs, ands): (Vec<u32>, Vec<u32>) = nodes.into_iter().partition(|n| !matches!(g.nodes[*n as usize], Node::And(..)));
    let mut var: HashMap<u32, u32> = HashMap::new();
    for (i, n) in inputs.iter().enumerate() {
        var.insert(*n, 1 + i as u32);
    }
    for (i, n) in ands.iter().enumerate() {
        var.insert(*n, 1 + inputs.len() as u32 + i as u32);
    }
    let lit = |l: Lit| -> u32 { if l.node() == 0 { u32::from(l.compl()) } else { 2 * var[&l.node()] + u32::from(l.compl()) } };
    let mut cone = oxibit_cec::aiger::Aig {
        inputs: inputs.len() as u32,
        latches: 0,
        ands: ands
            .iter()
            .map(|n| match g.nodes[*n as usize] {
                Node::And(x, y) => (lit(x), lit(y)),
                _ => unreachable!(),
            })
            .collect(),
        outputs: vec![],
    };
    cone.ands.push((lit(a), lit(b)));
    let both = 2 * (inputs.len() + ands.len() + 1) as u32;
    cone.outputs.push(both ^ 1);
    let (verdict, _) = oxibit_cec::sweep::prove(&cone, true, 1, 1000, 10_000, &oxibit_report::Silent);
    verdict.first() == Some(&oxibit_cec::sweep::Verdict::Proven)
}

fn uniform(v: &[Lit]) -> Option<Lit> {
    let first = *v.first()?;
    v.iter().all(|l| *l == first).then_some(first)
}

fn binary(m: &mut Module, op: Op, a: SigSpec, b: SigSpec) -> SigSpec {
    let y = m.wire(a.width());
    m.cells.push(Cell::Binary {
        op,
        a,
        b,
        signed: false,
        y: y.clone(),
    });
    y
}

fn mux(m: &mut Module, a: SigSpec, b: SigSpec, s: Bit) -> SigSpec {
    let y = m.wire(a.width());
    m.cells.push(Cell::Mux {
        a,
        b,
        s: SigSpec(vec![s]),
        y: y.clone(),
    });
    y
}

/// One write port doing what the exclusive `ports` did.
fn merge_ports(m: &mut Module, ports: &[MemWrite]) -> MemWrite {
    let aw = ports.iter().map(|w| w.addr.width()).max().unwrap_or(0);
    let mut en = ports[0].en.clone();
    let mut addr = ports[0].addr.extend(aw, false);
    let mut data = ports[0].data.clone();
    for w in &ports[1..] {
        let s = w.en.0[0];
        en = binary(m, Op::Or, en, w.en.clone());
        addr = mux(m, addr, w.addr.extend(aw, false), s);
        data = mux(m, data, w.data.clone(), s);
    }
    MemWrite {
        clk: ports[0].clk.clone(),
        addr,
        data,
        en,
    }
}

#[hot]
pub fn share(m: &mut Module, dirs: &PrimPorts, cfg: lower::Config) -> Result<MemShareStats, String> {
    let mut st = MemShareStats::default();
    if !m.cells.iter().any(|c| matches!(c, Cell::Mem(mem) if mem.writes.len() > 1)) {
        return Ok(st);
    }
    let lw = lower::lower(m, dirs, cfg)?;
    let by_name: HashMap<&str, &lower::MemL> = lw.mems.iter().map(|ml| (ml.mem.name.as_str(), ml)).collect();
    let mut plans: Vec<usize> = Vec::new();
    for (ci, c) in m.cells.iter().enumerate() {
        let Cell::Mem(mem) = c else { continue };
        if mem.writes.len() < 2 {
            continue;
        }
        let Some(ml) = by_name.get(mem.name.as_str()) else {
            st.left.push((mem.name.clone(), "not lowered"));
            continue;
        };
        if mem.writes.iter().any(|w| w.clk != mem.writes[0].clk) {
            st.left.push((mem.name.clone(), "write ports on different clocks"));
            continue;
        }
        let Some(ens) = ml.wr.iter().map(|(_, _, _, e)| uniform(e)).collect::<Option<Vec<Lit>>>() else {
            st.left.push((mem.name.clone(), "per-bit write enables"));
            continue;
        };
        let pairwise = (0..ens.len()).all(|i| (i + 1..ens.len()).all(|j| exclusive(&lw.aig, ens[i], ens[j])));
        if !pairwise {
            st.left.push((mem.name.clone(), "write enables not provably exclusive"));
            continue;
        }
        plans.push(ci);
    }
    for ci in plans {
        let Cell::Mem(mem) = &m.cells[ci] else { unreachable!() };
        let ports = mem.writes.clone();
        let merged = merge_ports(m, &ports);
        if let Cell::Mem(mem) = &mut m.cells[ci] {
            mem.attrs.insert("oxibit_merged_writes".into(), ports.len().to_string());
            mem.writes = vec![merged];
        }
        st.merged += 1;
        st.ports += ports.len() - 1;
    }
    Ok(st)
}

/// Exclusive write ports merged.
pub struct MemShare<'a> {
    pub dirs: &'a PrimPorts,
    pub carry_min: usize,
    pub lut_k: usize,
}

impl crate::stage::Pass for MemShare<'_> {
    const NAME: &'static str = "memshare";
    const EMITS: &'static [crate::caps::Mode] = &[];
    type Stats = MemShareStats;
    fn enabled(p: &crate::policy::PassSet) -> bool {
        p.memshare
    }
    fn json(s: &MemShareStats) -> serde_json::Value {
        serde_json::json!({ "merged": s.merged, "ports": s.ports, "left": s.left })
    }
}

impl crate::stage::RtlPass for MemShare<'_> {
    fn run(&self, m: &mut crate::stage::Rtl) -> MemShareStats {
        // A lowering failure here fails the real lowering right after,
        // which reports it; merge nothing.
        let cfg = lower::Config {
            carry_min: self.carry_min,
            lut_k: self.lut_k,
        };
        share(&mut m.0, self.dirs, cfg).unwrap_or_default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxibit_hdl::netlist::{Dir, MemRead, Memory, Signal};
    use std::collections::BTreeMap;

    fn input(m: &mut Module, name: &str, w: u32) -> SigSpec {
        let id = m.add_signal(Signal {
            name: name.into(),
            width: w,
            signed: false,
            port: Some(Dir::Input),
            init: None,
        });
        SigSpec::of(id, w)
    }

    #[derive(Clone, Copy, PartialEq)]
    enum Second {
        /// `rst`: excluded by `!rst` in the first enable.
        Rst,
        /// `rst ^ we`: excluded too, but only a solver sees it.
        Xor,
        /// `we2`: free to write with the first.
        Clash,
    }

    /// A 4 x 2 memory written under `!rst & we` and under a second
    /// enable, read at `ra`.
    fn design(second: Second) -> Module {
        let mut m = Module::new("t");
        let clk = input(&mut m, "clk", 1);
        let rst = input(&mut m, "rst", 1);
        let we = input(&mut m, "we", 1);
        let we2 = input(&mut m, "we2", 1);
        let (a0, a1, ra, d0) = (input(&mut m, "a0", 2), input(&mut m, "a1", 2), input(&mut m, "ra", 2), input(&mut m, "d0", 2));
        let out = m.add_signal(Signal {
            name: "q".into(),
            width: 2,
            signed: false,
            port: Some(Dir::Output),
            init: None,
        });
        let nrst = m.wire(1);
        m.cells.push(Cell::Unary {
            op: Op::Not,
            a: rst.clone(),
            a_signed: false,
            y: nrst.clone(),
        });
        let e0 = binary(&mut m, Op::And, nrst, we.clone());
        let e1 = match second {
            Second::Rst => rst,
            Second::Xor => {
                // !rst & we on the first port: both on needs we = 1, rst = 0,
                // so the xor is 1. Use its complement: never on with port 0.
                let x = binary(&mut m, Op::Xor, rst, we);
                let y = m.wire(1);
                m.cells.push(Cell::Unary {
                    op: Op::Not,
                    a: x,
                    a_signed: false,
                    y: y.clone(),
                });
                y
            }
            Second::Clash => we2,
        };
        m.cells.push(Cell::Mem(Memory {
            name: "mm".into(),
            width: 2,
            size: 4,
            offset: 0,
            init: BTreeMap::new(),
            attrs: BTreeMap::new(),
            reads: vec![MemRead {
                clk: None,
                addr: ra,
                data: SigSpec::of(out, 2),
            }],
            writes: vec![
                MemWrite {
                    clk: (clk.clone(), true),
                    addr: a0,
                    data: d0,
                    en: SigSpec(vec![e0.0[0]; 2]),
                },
                MemWrite {
                    clk: (clk, true),
                    addr: a1,
                    data: SigSpec::constant(0, 2),
                    en: SigSpec(vec![e1.0[0]; 2]),
                },
            ],
        }));
        m
    }

    fn writes(m: &Module) -> usize {
        m.cells.iter().find_map(|c| if let Cell::Mem(mem) = c { Some(mem.writes.len()) } else { None }).unwrap()
    }

    #[test]
    fn reset_sweep_merges() {
        let mut m = design(Second::Rst);
        let st = share(&mut m, &PrimPorts::new(), lower::Config { carry_min: 0, lut_k: 6 }).unwrap();
        assert_eq!((st.merged, st.ports), (1, 1));
        assert_eq!(writes(&m), 1);
    }

    #[test]
    fn solver_proves_what_structure_misses() {
        let mut m = design(Second::Xor);
        let st = share(&mut m, &PrimPorts::new(), lower::Config { carry_min: 0, lut_k: 6 }).unwrap();
        assert_eq!(st.merged, 1);
        assert_eq!(writes(&m), 1);
    }

    #[test]
    fn independent_enables_stay() {
        let mut m = design(Second::Clash);
        let st = share(&mut m, &PrimPorts::new(), lower::Config { carry_min: 0, lut_k: 6 }).unwrap();
        assert_eq!(st.merged, 0);
        assert_eq!(writes(&m), 2);
        assert_eq!(st.left[0].1, "write enables not provably exclusive");
    }
}
