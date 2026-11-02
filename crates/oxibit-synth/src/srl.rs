//! Shift registers to SRLC32E (yosys `shregmap -tech xilinx`).
//!
//! A chain of synthesis flops (`oxsyn_*` FDRE / FDSE) where each stage's D
//! is the previous stage's Q, every stage shares C, CE and clock polarity,
//! no stage has an active synchronous reset or set, and every Q but the
//! last feeds only the next stage, is a shift register. Chains of at least
//! `MIN_LEN` stages become SRLC32E cells, 32 stages each, with the address
//! tied to the length minus one; a remainder shorter than `MIN_LEN` stays
//! as flops at the tail. Runs after the checking view is taken, like the
//! RAM mapping: the check sees the flops, and the flop-chain-to-SRL step is
//! proven per shape (models/srl_shapes.sh).

use std::collections::HashMap;

use oxibit_hdl::netlist::{Bit, Cell, Module, Param, SigSpec};
use oxibit_prof::hot;

pub const MIN_LEN: usize = 3;
const SEG: usize = 32;

#[derive(Debug, Default, Clone)]
pub struct SrlStats {
    pub srlc32e: usize,
    /// Flops replaced, by type.
    pub ffs: usize,
    pub by_type: std::collections::BTreeMap<&'static str, usize>,
    pub chains: usize,
}

struct Stage {
    cell: usize,
    fdse: bool,
    d: Bit,
    q: Bit,
    clk: Bit,
    ce: Bit,
    clk_inv: bool,
    init: bool,
}

fn bit1(conns: &std::collections::BTreeMap<String, SigSpec>, p: &str) -> Option<Bit> {
    match conns.get(p) {
        Some(s) if s.0.len() == 1 => Some(s.0[0]),
        _ => None,
    }
}

fn flag(params: &std::collections::BTreeMap<String, Param>, p: &str) -> Option<bool> {
    match params.get(p) {
        None => Some(false),
        Some(Param::Bits(b)) if b.len() == 1 => b[0],
        Some(Param::Int(v)) => Some(*v != 0),
        _ => None,
    }
}

/// A synthesis flop with no active synchronous reset or set.
fn stage(ci: usize, c: &Cell) -> Option<Stage> {
    let Cell::Instance { ty, name, params, conns } = c else { return None };
    if !name.starts_with("oxsyn_") {
        return None;
    }
    let (srp, inv) = match ty.as_str() {
        "FDRE" => ("R", "IS_R_INVERTED"),
        "FDSE" => ("S", "IS_S_INVERTED"),
        _ => return None,
    };
    let sr = bit1(conns, srp)?;
    let sr_inv = flag(params, inv)?;
    let idle = matches!((sr, sr_inv), (Bit::Zero, false) | (Bit::One, true));
    if !idle || flag(params, "IS_D_INVERTED")? || flag(params, "IS_CE_INVERTED")? {
        return None;
    }
    let init = match params.get("INIT") {
        Some(Param::Bits(b)) if b.len() == 1 => b[0].unwrap_or(ty == "FDSE"),
        Some(Param::Int(v)) => *v != 0,
        _ => ty == "FDSE",
    };
    Some(Stage {
        cell: ci,
        fdse: ty == "FDSE",
        d: bit1(conns, "D")?,
        q: bit1(conns, "Q")?,
        clk: bit1(conns, "C")?,
        ce: bit1(conns, "CE")?,
        clk_inv: flag(params, "IS_C_INVERTED")?,
        init,
    })
}

/// Is the left side of a connection a name only (a non-port signal emit
/// added so a design wire keeps its name)? Nothing reads those.
fn name_only(m: &Module, l: &SigSpec) -> bool {
    l.0.iter().all(|b| matches!(b, Bit::Sig(id, _) if m.signal(*id).port.is_none()))
}

/// Every appearance of a signal bit anywhere in the module: cell ports,
/// memory ports, connections to ports. Directions are not needed: a flop's
/// own Q is one appearance, so a Q with exactly two feeds exactly one other
/// place.
fn appearances(m: &Module) -> HashMap<Bit, u32> {
    let mut n: HashMap<Bit, u32> = HashMap::new();
    let mut add = |s: &SigSpec| {
        for b in &s.0 {
            if matches!(b, Bit::Sig(..)) {
                *n.entry(*b).or_default() += 1;
            }
        }
    };
    for c in &m.cells {
        match c {
            Cell::Unary { a, y, .. } => {
                add(a);
                add(y);
            }
            Cell::Binary { a, b, y, .. } => {
                add(a);
                add(b);
                add(y);
            }
            Cell::Mux { a, b, s, y } => {
                add(a);
                add(b);
                add(s);
                add(y);
            }
            Cell::Dff { clk, d, q, .. } => {
                add(clk);
                add(d);
                add(q);
            }
            Cell::Adff { clk, arst, d, q, .. } => {
                add(clk);
                add(arst);
                add(d);
                add(q);
            }
            Cell::Mem(mem) => {
                for r in &mem.reads {
                    add(&r.addr);
                    add(&r.data);
                    if let Some((c, _)) = &r.clk {
                        add(c);
                    }
                }
                for w in &mem.writes {
                    add(&w.clk.0);
                    add(&w.addr);
                    add(&w.data);
                    add(&w.en);
                }
            }
            Cell::Instance { conns, .. } => conns.values().for_each(&mut add),
        }
    }
    for (l, r) in &m.connects {
        if !name_only(m, l) {
            add(l);
            add(r);
        }
    }
    for (i, s) in m.signals.iter().enumerate() {
        if s.port.is_some() {
            add(&SigSpec::of(oxibit_hdl::netlist::SigId(i as u32), s.width));
        }
    }
    n
}

#[hot]
pub fn map(m: &mut Module) -> SrlStats {
    let mut st = SrlStats::default();
    let stages: Vec<Stage> = m.cells.iter().enumerate().filter_map(|(i, c)| stage(i, c)).collect();
    let uses = appearances(m);
    let by_q: HashMap<Bit, usize> = stages.iter().enumerate().map(|(k, s)| (s.q, k)).collect();
    // prev[k]: the stage feeding stage k, when the pair can shift together.
    let mut prev: Vec<Option<usize>> = vec![None; stages.len()];
    let mut next: Vec<Option<usize>> = vec![None; stages.len()];
    for (k, s) in stages.iter().enumerate() {
        let Some(&p) = by_q.get(&s.d) else { continue };
        let ps = &stages[p];
        if p != k && uses.get(&ps.q) == Some(&2) && ps.clk == s.clk && ps.ce == s.ce && ps.clk_inv == s.clk_inv {
            prev[k] = Some(p);
            next[p] = Some(k);
        }
    }
    let mut drop = vec![false; m.cells.len()];
    let mut dead: std::collections::HashSet<Bit> = std::collections::HashSet::new();
    let mut new_cells = Vec::new();
    for head in 0..stages.len() {
        if prev[head].is_some() {
            continue;
        }
        let mut chain = vec![head];
        while let Some(n) = next[*chain.last().unwrap()] {
            if n == head {
                break; // a ring: leave it
            }
            chain.push(n);
        }
        if chain.len() < MIN_LEN {
            continue;
        }
        st.chains += 1;
        let mut d = stages[chain[0]].d;
        let mut at = 0;
        while chain.len() - at >= MIN_LEN {
            let len = (chain.len() - at).min(SEG);
            let seg = &chain[at..at + len];
            let last = &stages[*seg.last().unwrap()];
            let first = &stages[seg[0]];
            let mut init = vec![Some(false); SEG];
            for (i, k) in seg.iter().enumerate() {
                init[i] = Some(stages[*k].init);
                *st.by_type.entry(if stages[*k].fdse { "FDSE" } else { "FDRE" }).or_default() += 1;
                drop[stages[*k].cell] = true;
                if i + 1 < len {
                    dead.insert(stages[*k].q);
                }
            }
            let a = SigSpec::constant((len - 1) as u64, 5);
            let mut conns = std::collections::BTreeMap::new();
            conns.insert("A".to_string(), a);
            conns.insert("CE".to_string(), SigSpec(vec![first.ce]));
            conns.insert("CLK".to_string(), SigSpec(vec![first.clk]));
            conns.insert("D".to_string(), SigSpec(vec![d]));
            conns.insert("Q".to_string(), SigSpec(vec![last.q]));
            let mut params = std::collections::BTreeMap::from([("INIT".to_string(), Param::Bits(init))]);
            if first.clk_inv {
                params.insert("IS_CLK_INVERTED".into(), Param::Bits(vec![Some(true)]));
            }
            let name = match &m.cells[first.cell] {
                Cell::Instance { name, .. } => format!("oxsyn_srl{}${}", st.srlc32e, name.trim_start_matches("oxsyn_")),
                _ => unreachable!(),
            };
            new_cells.push(Cell::Instance {
                ty: "SRLC32E".into(),
                name,
                params,
                conns,
            });
            st.srlc32e += 1;
            st.ffs += len;
            d = last.q;
            at += len;
        }
        // A remainder shorter than MIN_LEN stays as flops; its first D is
        // already the last SRL's Q.
    }
    let mut k = 0;
    m.cells.retain(|_| {
        let keep = !drop[k];
        k += 1;
        keep
    });
    m.cells.extend(new_cells);
    // Names of stages that no longer exist.
    let conns = std::mem::take(&mut m.connects);
    m.connects = conns
        .into_iter()
        .filter(|(l, r)| !(name_only(m, l) && r.0.iter().any(|b| dead.contains(b))))
        .collect();
    st
}

pub struct Srl;

impl crate::stage::Pass for Srl {
    const NAME: &'static str = "srl";
    const EMITS: &'static [crate::caps::Mode] = &[crate::caps::Mode::Srlc32e];
    type Stats = SrlStats;
    fn enabled(p: &crate::policy::PassSet) -> bool {
        p.srl
    }
    fn json(s: &SrlStats) -> serde_json::Value {
        serde_json::json!({ "srlc32e": s.srlc32e, "ffs": s.ffs, "chains": s.chains })
    }
}

impl crate::stage::PhysicalPass for Srl {
    const EVIDENCE: crate::policy::PostCheckEvidence = crate::policy::PostCheckEvidence::ShapeProof {
        harness: "models/srl_shapes.sh over examples/srlcases (yosys k-induction)",
    };
    fn run(&self, m: &mut crate::stage::Physical) -> SrlStats {
        map(&mut m.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxibit_hdl::netlist::{Dir, Signal};
    use std::collections::BTreeMap;

    fn ff(m: &mut Module, k: usize, clk: Bit, d: Bit, q: Bit) {
        m.cells.push(Cell::Instance {
            ty: "FDRE".into(),
            name: format!("oxsyn_ff{k}$r"),
            params: BTreeMap::from([("INIT".to_string(), Param::Bits(vec![Some(k % 2 == 1)]))]),
            conns: BTreeMap::from([
                ("C".to_string(), SigSpec(vec![clk])),
                ("CE".to_string(), SigSpec(vec![Bit::One])),
                ("R".to_string(), SigSpec(vec![Bit::Zero])),
                ("D".to_string(), SigSpec(vec![d])),
                ("Q".to_string(), SigSpec(vec![q])),
            ]),
        });
    }

    fn port(m: &mut Module, name: &str, w: u32, dir: Dir) -> SigSpec {
        let id = m.add_signal(Signal {
            name: name.into(),
            width: w,
            signed: false,
            port: Some(dir),
            init: None,
        });
        SigSpec::of(id, w)
    }

    /// A chain of `n` stages with a tap on stage `tap` (0: none).
    fn chain(n: usize, tap: usize) -> Module {
        let mut m = Module::new("t");
        let clk = port(&mut m, "clk", 1, Dir::Input).0[0];
        let din = port(&mut m, "d", 1, Dir::Input).0[0];
        let q = port(&mut m, "q", 1, Dir::Output).0[0];
        let t = port(&mut m, "t", 1, Dir::Output).0[0];
        let w = m.wire(n as u32);
        let mut d = din;
        for k in 0..n {
            let o = if k == n - 1 { q } else { w.0[k] };
            ff(&mut m, k, clk, d, o);
            d = o;
        }
        let tb = if tap > 0 { w.0[tap - 1] } else { Bit::Zero };
        m.connects.push((SigSpec(vec![t]), SigSpec(vec![tb])));
        m
    }

    fn count(m: &Module, ty: &str) -> usize {
        m.cells.iter().filter(|c| matches!(c, Cell::Instance { ty: t, .. } if t == ty)).count()
    }

    #[test]
    fn lengths() {
        for (n, srl, ff) in [
            (2, 0, 2),
            (3, 1, 0),
            (32, 1, 0),
            (33, 1, 1),
            (34, 1, 2),
            (35, 2, 0),
            (64, 2, 0),
            (70, 3, 0),
        ] {
            let mut m = chain(n, 0);
            let st = map(&mut m);
            assert_eq!((count(&m, "SRLC32E"), count(&m, "FDRE")), (srl, ff), "n={n} {st:?}");
        }
    }

    #[test]
    fn tap_splits() {
        // Stage 5's Q also drives a port: stages 1..=5 and 6..=10 are
        // separate chains.
        let mut m = chain(10, 5);
        map(&mut m);
        assert_eq!((count(&m, "SRLC32E"), count(&m, "FDRE")), (2, 0));
    }

    #[test]
    fn init_order() {
        let mut m = chain(4, 0);
        map(&mut m);
        let Some(Cell::Instance { params, conns, .. }) = m.cells.iter().find(|c| matches!(c, Cell::Instance { ty, .. } if ty == "SRLC32E")) else {
            panic!()
        };
        let Param::Bits(b) = &params["INIT"] else { panic!() };
        assert_eq!(&b[..4], &[Some(false), Some(true), Some(false), Some(true)]);
        assert_eq!(conns["A"], SigSpec::constant(3, 5));
    }
}
