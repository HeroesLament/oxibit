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

use std::collections::BTreeMap;

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
    pub mul_left: usize,
    /// Partial products left to fabric (a chunk of at most `SMALL` bits).
    pub fabric_partials: usize,
    /// Multipliers whose every partial is narrow, left whole to fabric.
    pub mul_fabric: usize,
}

/// Chunks this narrow (sign position included) multiply in fabric.
pub const SMALL: usize = 4;

#[hot]
/// Map multipliers needing at most `opt_max` DSPs; leave the rest as logic.
pub fn infer(m: &mut Module, opt_max: usize) -> DspStats {
    let mut st = DspStats::default();
    let mut cn = Canon::new(m);
    let cells = std::mem::take(&mut m.cells);
    let mut out = Vec::with_capacity(cells.len());
    for (ci, cell) in cells.into_iter().enumerate() {
        let Cell::Binary {
            op: Op::Mul,
            a,
            b,
            signed,
            y,
        } = &cell
        else {
            out.push(cell);
            continue;
        };
        let yw = y.width() as usize;
        let (wa, ea) = signed_width(&mut cn, &a.0, *signed, yw);
        let (wb, eb) = signed_width(&mut cn, &b.0, *signed, yw);
        if std::env::var_os("OXIBIT_DSP_DEBUG").is_some() {
            eprintln!("mul {ci}: a {} b {} y {yw} signed {signed} -> {wa} x {wb}", a.width(), b.width());
        }
        // Split each operand into chunks the multiplier takes: the top
        // chunk signed (A <= 25, B <= 18 bits), the rest unsigned (24 / 17
        // bits, fed with a zero sign bit). Put the longer operand on A.
        let (pa, pb) = if wa >= wb { (ea, eb) } else { (eb, ea) };
        let ca = chunks(&pa, 25);
        let cb = chunks(&pb, 18);
        let small = |x: &Vec<Bit>, y: &Vec<Bit>| x.len() <= SMALL || y.len() <= SMALL;
        let need = ca
            .iter()
            .flat_map(|(_, xa)| cb.iter().map(move |(_, xb)| (xa, xb)))
            .filter(|(xa, xb)| !small(xa, xb))
            .count();
        // Every partial narrow: the multiplier is cheaper in fabric whole.
        if need == 0 {
            st.mul_fabric += 1;
            out.push(cell);
            continue;
        }
        if need > opt_max {
            st.mul_left += 1;
            out.push(cell);
            continue;
        }
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
}

impl crate::stage::Pass for Dsp {
    const NAME: &'static str = "dsp";
    const EMITS: &'static [crate::caps::Mode] = &[crate::caps::Mode::Dsp48Comb];
    type Stats = DspStats;
    fn enabled(p: &crate::policy::PassSet) -> bool {
        p.dsp
    }
    fn json(s: &DspStats) -> serde_json::Value {
        serde_json::json!({ "dsp": s.dsp, "mul_left": s.mul_left, "mul_fabric": s.mul_fabric, "fabric_partials": s.fabric_partials })
    }
}

impl crate::stage::RtlPass for Dsp {
    fn run(&self, m: &mut crate::stage::Rtl) -> DspStats {
        infer(&mut m.0, self.max_per_mul)
    }
}
