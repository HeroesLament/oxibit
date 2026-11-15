//! Multipliers to DSP48E1.
//!
//! A `$mul`'s operands are reduced modulo 2^Y_WIDTH (yosys's `$mul` is the
//! product truncated to `Y_WIDTH`) and stripped of redundant sign or zero
//! extension. Operands that fit the 25 x 18 signed multiplier give one
//! combinational DSP48E1 (no pipeline registers yet), `Y` = low bits of
//! `P`; that is exact because a 25 x 18 signed product fits P's 48 bits.
//! Wider operands are split into chunks (top chunk signed, the rest
//! unsigned) and the shifted partial products summed with `$add` cells,
//! which lowering puts on carry chains. Multipliers needing more DSPs than
//! the limit stay as logic. A partial product with a chunk of at most
//! `SMALL` bits (the slivers left over when an operand is a bit or two
//! wider than the multiplier input) is a `$mul` in fabric instead of a
//! DSP: a few AND rows into the adder that sums the partials anyway.
//!
//! A multiplier with a constant operand takes no DSP: it stays a `$mul`,
//! whose partial-product rows lowering folds against the constant (a power
//! of two, the usual case, is a shift and costs nothing; picoBEAM writes
//! shifts as `x * 134_217_728`).
//!
//! The device has a fixed number of DSP48E1 sites (`budget`, less any the
//! design instantiates itself). When the multipliers want more, the largest
//! (by operand area, the fabric they would otherwise cost) keep their DSPs
//! and the rest stay `$mul` for lowering, size-first: a multiplier that
//! does not fit what is left is skipped and smaller ones may still take it.

use std::collections::{BTreeMap, BTreeSet};

use oxibit_hdl::netlist::{Bit, Cell, Module, Op, Param, SigSpec};
use oxibit_prof::hot;

/// Bit identity through the module's continuous connections.
pub(crate) struct Canon {
    offs: Vec<usize>,
    parent: Vec<u32>,
    konst: Vec<Option<Bit>>,
}

#[derive(Clone, Copy, PartialEq)]
pub(crate) enum K {
    Net(usize),
    Const(Bit),
}

impl Canon {
    pub(crate) fn new(m: &Module) -> Self {
        let mut offs = Vec::with_capacity(m.signals.len() + 1);
        let mut n = 0;
        for s in &m.signals {
            offs.push(n);
            n += s.width as usize;
        }
        offs.push(n);
        let mut c = Canon {
            offs,
            parent: (0..n as u32).collect(),
            konst: vec![None; n],
        };
        for (l, r) in &m.connects {
            for (a, b) in l.0.iter().zip(&r.0) {
                match (c.idx(*a), c.idx(*b)) {
                    (Some(x), Some(y)) => {
                        let (rx, ry) = (c.find(x), c.find(y));
                        if rx != ry {
                            let k = c.konst[rx].or(c.konst[ry]);
                            c.parent[ry] = rx as u32;
                            c.konst[rx] = k;
                        }
                    }
                    (Some(x), None) => {
                        let r = c.find(x);
                        c.konst[r].get_or_insert(*b);
                    }
                    (None, Some(y)) => {
                        let r = c.find(y);
                        c.konst[r].get_or_insert(*a);
                    }
                    _ => {}
                }
            }
        }
        c
    }
    fn idx(&self, b: Bit) -> Option<usize> {
        match b {
            Bit::Sig(id, i) => Some(self.offs[id.0 as usize] + i as usize),
            _ => None,
        }
    }
    fn find(&mut self, mut x: usize) -> usize {
        while self.parent[x] as usize != x {
            let g = self.parent[self.parent[x] as usize];
            self.parent[x] = g;
            x = g as usize;
        }
        x
    }
    pub(crate) fn k(&mut self, b: Bit) -> K {
        match self.idx(b) {
            Some(x) => {
                let r = self.find(x);
                match self.konst[r] {
                    Some(c) => K::Const(c),
                    None => K::Net(r),
                }
            }
            None => K::Const(b),
        }
    }
}

/// The shortest signed bit vector equal to `v` modulo 2^yw: `v` is first
/// extended (by its signedness) or truncated to `yw` bits, as `$mul` does,
/// then redundant sign or zero bits are stripped, whichever is shorter.
/// Products of such representatives agree with `$mul` in the low `yw`
/// bits.
fn signed_width(cn: &mut Canon, v: &[Bit], signed: bool, yw: usize) -> (usize, Vec<Bit>) {
    let mut b: Vec<Bit> = v.iter().take(yw).copied().collect();
    let fill = if signed { b.last().copied().unwrap_or(Bit::Zero) } else { Bit::Zero };
    b.resize(yw, fill);
    let mut s = b.clone();
    while s.len() > 1 && cn.k(s[s.len() - 1]) == cn.k(s[s.len() - 2]) {
        s.pop();
    }
    let mut z = b;
    while !z.is_empty() && cn.k(z[z.len() - 1]) == K::Const(Bit::Zero) {
        z.pop();
    }
    z.push(Bit::Zero);
    if z.len() < s.len() { (z.len(), z) } else { (s.len(), s) }
}

/// For an operand that is a constant, its set bits (1: a power of two);
/// `None` for one that depends on a net.
fn const_ones(cn: &mut Canon, v: &[Bit]) -> Option<usize> {
    v.iter().try_fold(0, |n, &b| match cn.k(b) {
        K::Const(Bit::One) => Some(n + 1),
        K::Const(_) => Some(n),
        K::Net(_) => None,
    })
}

fn sext(v: &[Bit], w: usize) -> SigSpec {
    let mut o = v.to_vec();
    let fill = *o.last().unwrap_or(&Bit::Zero);
    o.resize(w, fill);
    SigSpec(o)
}

fn c(v: u64, w: u32) -> SigSpec {
    SigSpec::constant(v, w)
}

/// Chunks of a signed operand for a `w`-bit signed multiplier input: (bit
/// offset, bits as signed). Lower chunks are w-1 bits unsigned with a zero
/// sign appended; the top chunk keeps the operand's sign.
fn chunks(v: &[Bit], w: usize) -> Vec<(usize, Vec<Bit>)> {
    let mut out = Vec::new();
    let mut lo = 0;
    while v.len() - lo > w {
        let mut c = v[lo..lo + w - 1].to_vec();
        c.push(Bit::Zero);
        out.push((lo, c));
        lo += w - 1;
    }
    out.push((lo, v[lo..].to_vec()));
    out
}

/// A combinational DSP48E1 computing `P = A * B` (A 30 bits, of which the
/// multiplier uses the low 25; B 18 bits).
fn dsp_cell(name: String, a: SigSpec, b: SigSpec, p: SigSpec) -> Cell {
    let mut params = BTreeMap::new();
    for k in [
        "ACASCREG",
        "ADREG",
        "ALUMODEREG",
        "AREG",
        "BCASCREG",
        "BREG",
        "CARRYINREG",
        "CARRYINSELREG",
        "CREG",
        "DREG",
        "INMODEREG",
        "MREG",
        "OPMODEREG",
        "PREG",
    ] {
        params.insert(k.to_string(), Param::Int(0));
    }
    for (k, v) in [
        ("A_INPUT", "DIRECT"),
        ("B_INPUT", "DIRECT"),
        ("USE_DPORT", "FALSE"),
        ("USE_MULT", "MULTIPLY"),
        ("USE_SIMD", "ONE48"),
    ] {
        params.insert(k.to_string(), Param::Str(v.to_string()));
    }
    let mut conns = BTreeMap::new();
    conns.insert("A".to_string(), a);
    conns.insert("B".to_string(), b);
    conns.insert("C".to_string(), c(0, 48));
    conns.insert("D".to_string(), c(0, 25));
    conns.insert("ACIN".to_string(), c(0, 30));
    conns.insert("BCIN".to_string(), c(0, 18));
    conns.insert("PCIN".to_string(), c(0, 48));
    conns.insert("ALUMODE".to_string(), c(0, 4));
    conns.insert("INMODE".to_string(), c(0, 5));
    // X = M, Y = M, Z = 0.
    conns.insert("OPMODE".to_string(), c(0b000_0101, 7));
    conns.insert("CARRYINSEL".to_string(), c(0, 3));
    for k in ["CARRYCASCIN", "CARRYIN", "MULTSIGNIN", "CLK"] {
        conns.insert(k.to_string(), c(0, 1));
    }
    for k in [
        "CEA1",
        "CEA2",
        "CEAD",
        "CEALUMODE",
        "CEB1",
        "CEB2",
        "CEC",
        "CECARRYIN",
        "CECTRL",
        "CED",
        "CEINMODE",
        "CEM",
        "CEP",
    ] {
        conns.insert(k.to_string(), c(1, 1));
    }
    for k in [
        "RSTA",
        "RSTALLCARRYIN",
        "RSTALUMODE",
        "RSTB",
        "RSTC",
        "RSTCTRL",
        "RSTD",
        "RSTINMODE",
        "RSTM",
        "RSTP",
    ] {
        conns.insert(k.to_string(), c(0, 1));
    }
    conns.insert("P".to_string(), p);
    Cell::Instance {
        ty: "DSP48E1".into(),
        name,
        params,
        conns,
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct DspStats {
    pub dsp: usize,
    /// DSP48E1 sites available to inference (the device's, less the
    /// design's own instances).
    pub budget: usize,
    /// DSP48E1s that inference wanted (every multiplier within `opt_max`).
    pub wanted: usize,
    /// Multipliers left to fabric because the budget was spent.
    pub mul_budget: usize,
    pub mul_left: usize,
    /// Multipliers with a constant operand, left to lowering.
    pub mul_const: usize,
    /// Partial products left to fabric (a chunk of at most `SMALL` bits).
    pub fabric_partials: usize,
    /// Multipliers whose every partial is narrow, left whole to fabric.
    pub mul_fabric: usize,
}

/// Chunks this narrow (sign position included) multiply in fabric.
pub const SMALL: usize = 4;

fn small(x: &[Bit], y: &[Bit]) -> bool {
    x.len() <= SMALL || y.len() <= SMALL
}

/// What inference makes of one `$mul`.
enum Plan {
    Const,
    /// Every partial narrow: the multiplier is cheaper in fabric whole.
    Fabric,
    /// More DSPs than `opt_max`.
    Left,
    /// Operand chunks (A side, B side), the DSPs they take, the operand
    /// area (the size the budget ranks by).
    Dsp {
        ca: Vec<(usize, Vec<Bit>)>,
        cb: Vec<(usize, Vec<Bit>)>,
        need: usize,
        area: usize,
    },
}

fn plan(cn: &mut Canon, ci: usize, a: &SigSpec, b: &SigSpec, signed: bool, yw: usize, opt_max: usize) -> Plan {
    let (wa, ea) = signed_width(cn, &a.0, signed, yw);
    let (wb, eb) = signed_width(cn, &b.0, signed, yw);
    let (ka, kb) = (const_ones(cn, &ea), const_ones(cn, &eb));
    if std::env::var_os("OXIBIT_DSP_DEBUG").is_some() {
        eprintln!("mul {ci}: a {} b {} y {yw} signed {signed} -> {wa} x {wb} const_ones {ka:?} {kb:?}", a.width(), b.width());
    }
    if ka.is_some() || kb.is_some() {
        return Plan::Const;
    }
    // Split each operand into chunks the multiplier takes: the top chunk
    // signed (A <= 25, B <= 18 bits), the rest unsigned (24 / 17 bits, fed
    // with a zero sign bit). Put the longer operand on A.
    let (pa, pb) = if wa >= wb { (ea, eb) } else { (eb, ea) };
    let ca = chunks(&pa, 25);
    let cb = chunks(&pb, 18);
    let need = ca
        .iter()
        .flat_map(|(_, xa)| cb.iter().map(move |(_, xb)| (xa, xb)))
        .filter(|(xa, xb)| !small(xa, xb))
        .count();
    if need == 0 {
        Plan::Fabric
    } else if need > opt_max {
        Plan::Left
    } else {
        Plan::Dsp {
            ca,
            cb,
            need,
            area: wa * wb,
        }
    }
}

#[hot]
/// Map multipliers needing at most `opt_max` DSPs, at most `budget` DSPs
/// in all counting the design's own DSP48E1 instances, largest first;
/// leave the rest as logic.
pub fn infer(m: &mut Module, opt_max: usize, budget: usize) -> DspStats {
    let mut st = DspStats::default();
    let mut cn = Canon::new(m);
    let cells = std::mem::take(&mut m.cells);
    let own = cells.iter().filter(|c| matches!(c, Cell::Instance { ty, .. } if ty == "DSP48E1")).count();
    st.budget = budget.saturating_sub(own);
    let mut plans: Vec<Option<Plan>> = cells
        .iter()
        .enumerate()
        .map(|(ci, cell)| match cell {
            Cell::Binary {
                op: Op::Mul,
                a,
                b,
                signed,
                y,
            } => Some(plan(&mut cn, ci, a, b, *signed, y.width() as usize, opt_max)),
            _ => None,
        })
        .collect();
    // Largest operand area first, then fewer DSPs, then netlist order.
    let mut want: Vec<(usize, usize, usize)> = plans
        .iter()
        .enumerate()
        .filter_map(|(ci, p)| match p {
            Some(Plan::Dsp { need, area, .. }) => Some((*area, *need, ci)),
            _ => None,
        })
        .collect();
    want.sort_by(|x, y| y.0.cmp(&x.0).then(x.1.cmp(&y.1)).then(x.2.cmp(&y.2)));
    let mut left = st.budget;
    let mut keep = BTreeSet::new();
    for &(_, need, ci) in &want {
        st.wanted += need;
        if need <= left {
            left -= need;
            keep.insert(ci);
        }
    }
    let mut out = Vec::with_capacity(cells.len());
    for (ci, cell) in cells.into_iter().enumerate() {
        let Cell::Binary { op: Op::Mul, y, .. } = &cell else {
            out.push(cell);
            continue;
        };
        let (ca, cb) = match plans[ci].take() {
            Some(Plan::Dsp { ca, cb, .. }) if keep.contains(&ci) => (ca, cb),
            p => {
                match p {
                    Some(Plan::Const) => st.mul_const += 1,
                    Some(Plan::Fabric) => st.mul_fabric += 1,
                    Some(Plan::Left) => st.mul_left += 1,
                    _ => st.mul_budget += 1,
                }
                out.push(cell);
                continue;
            }
        };
        let yw = y.width() as usize;
        let mut terms: Vec<SigSpec> = Vec::new();
        for (sa, xa) in &ca {
            for (sb, xb) in &cb {
                let shift = sa + sb;
                if shift >= yw {
                    continue;
                }
                if small(xa, xb) {
                    // Signed chunks, product modulo 2^(yw - shift).
                    let p = m.wire((yw - shift) as u32);
                    out.push(Cell::Binary {
                        op: Op::Mul,
                        a: SigSpec(xa.clone()),
                        b: SigSpec(xb.clone()),
                        signed: true,
                        y: p.clone(),
                    });
                    st.fabric_partials += 1;
                    let mut t = vec![Bit::Zero; shift];
                    t.extend(p.0.iter().copied());
                    terms.push(SigSpec(t));
                    continue;
                }
                let p = m.wire(48);
                out.push(dsp_cell(format!("oxsyn_dsp{ci}_{sa}_{sb}"), sext(xa, 30), sext(xb, 18), p.clone()));
                st.dsp += 1;
                let mut t = vec![Bit::Zero; shift];
                t.extend(p.0.iter().copied());
                let sign = p.0[47];
                t.resize(yw.max(t.len()), sign);
                t.truncate(yw);
                terms.push(SigSpec(t));
            }
        }
        let mut acc = terms[0].clone();
        for t in &terms[1..] {
            let s = m.wire(yw as u32);
            out.push(Cell::Binary {
                op: Op::Add,
                a: acc,
                b: t.clone(),
                signed: false,
                y: s.clone(),
            });
            acc = s;
        }
        m.connects.push((y.clone(), acc));
    }
    m.cells = out;
    st
}

pub struct Dsp {
    pub max_per_mul: usize,
    /// DSP48E1 sites on the device.
    pub budget: usize,
}

impl crate::stage::Pass for Dsp {
    const NAME: &'static str = "dsp";
    const EMITS: &'static [crate::caps::Mode] = &[crate::caps::Mode::Dsp48Comb];
    type Stats = DspStats;
    fn enabled(p: &crate::policy::PassSet) -> bool {
        p.dsp
    }
    fn json(s: &DspStats) -> serde_json::Value {
        serde_json::json!({ "dsp": s.dsp, "budget": s.budget, "wanted": s.wanted, "mul_budget": s.mul_budget, "mul_left": s.mul_left, "mul_const": s.mul_const, "mul_fabric": s.mul_fabric, "fabric_partials": s.fabric_partials })
    }
}

impl crate::stage::RtlPass for Dsp {
    fn run(&self, m: &mut crate::stage::Rtl) -> DspStats {
        infer(&mut m.0, self.max_per_mul, self.budget)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `y = a * b`, signed, on fresh nets of the given widths.
    fn mul(m: &mut Module, wa: u32, wb: u32) {
        let (a, b, y) = (m.wire(wa), m.wire(wb), m.wire(wa + wb));
        m.cells.push(Cell::Binary {
            op: Op::Mul,
            a,
            b,
            signed: true,
            y,
        });
    }

    fn dsps(m: &Module) -> usize {
        m.cells.iter().filter(|c| matches!(c, Cell::Instance { ty, .. } if ty == "DSP48E1")).count()
    }

    #[test]
    fn constant_operand_takes_no_dsp() {
        let mut m = Module::new("t");
        let (a, y) = (m.wire(40), m.wire(64));
        m.cells.push(Cell::Binary {
            op: Op::Mul,
            a,
            b: c(134_217_728, 32),
            signed: false,
            y,
        });
        let st = infer(&mut m, 4, 220);
        assert_eq!((st.dsp, st.mul_const, st.wanted), (0, 1, 0));
        assert_eq!(dsps(&m), 0);
    }

    #[test]
    fn budget_keeps_the_largest() {
        // 40 x 30: 4 DSPs, area 1200. 24 x 17 and 20 x 15: 1 DSP each.
        let shapes = [(20, 15), (40, 30), (24, 17)];
        let mut m = Module::new("t");
        for (wa, wb) in shapes {
            mul(&mut m, wa, wb);
        }
        let st = infer(&mut m, 4, 5);
        assert_eq!((st.wanted, st.dsp, st.mul_budget), (6, 5, 1));
        // The 20 x 15 is the one left.
        assert!(m.cells.iter().any(|c| matches!(c, Cell::Binary { op: Op::Mul, a, .. } if a.width() == 20)));

        // Room for 3 once the design's own DSP48E1 is counted: the 40 x 30
        // does not fit, the two smaller ones still do.
        let mut m = Module::new("t");
        for (wa, wb) in shapes {
            mul(&mut m, wa, wb);
        }
        let (a, b, p) = (m.wire(30), m.wire(18), m.wire(48));
        m.cells.push(dsp_cell("own".into(), a, b, p));
        let st = infer(&mut m, 4, 4);
        assert_eq!((st.budget, st.dsp, st.mul_budget), (3, 2, 1));
        assert_eq!(dsps(&m), 3);
    }

    #[test]
    fn unbounded_budget_maps_everything() {
        let mut m = Module::new("t");
        for (wa, wb) in [(20, 15), (40, 30), (24, 17)] {
            mul(&mut m, wa, wb);
        }
        let st = infer(&mut m, 4, usize::MAX);
        assert_eq!((st.dsp, st.mul_budget), (6, 0));
    }
}
