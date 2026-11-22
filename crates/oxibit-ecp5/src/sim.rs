//! Cycle simulation, 64 patterns at a time, of the ECP5 cell netlist and
//! of the lowered design it came from, to check synthesis against its own
//! input (`check_synthesis`).
//!
//! Both sides use the same cycle model: all flops share one clock and
//! update together; a flop with an asynchronous set/reset shows the set or
//! reset value while it is active; outputs are sampled after the
//! combinational logic settles, before the edge. A memory read port is a
//! register on the read data: at the edge it samples the word at the
//! address before that edge's write lands (read-first), as the DP16KD
//! port pair does for different addresses. Both sides hold 2^abits words,
//! so a write the source would drop past the end of a memory whose size
//! is not a power of two lands in both (only out-of-range reads see it,
//! and the source leaves those undefined).
//!
//! Kept primitives (PLL, BB) are opaque on both sides: each output pin is
//! a free input of the check (`probes`), each input pin an observed value.
//! The pass-through primitives (IB, OB, OBZ, DCCA) copy input to output in
//! the lowered design, as the netlist's pads and clock network do.

use std::collections::HashMap;

use oxibit_synth::aig::{Aig, Lit, Node};
use oxibit_synth::lower::{FfKind, Lowered};

use crate::netlist::{Cell, EbrWidth, Netlist, Polarity, PrimPin, RegSet, SrMode};

/// A kept primitive's pin the check drives (`out`: its value is a free
/// input) or observes (an input pin).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Probe {
    /// The instance's index in the lowered design.
    pub inst: usize,
    pub pin: String,
    pub out: bool,
    /// The netlist net on the pin.
    pub net: u32,
}

/// The netlist's primitive pins, in instance order then pin order.
pub fn probes(nl: &Netlist) -> Vec<Probe> {
    let mut prims: Vec<&crate::netlist::Prim> = nl
        .cells
        .iter()
        .filter_map(|c| match c {
            Cell::Prim(p) => Some(&**p),
            _ => None,
        })
        .collect();
    prims.sort_by_key(|p| p.inst);
    let mut v = Vec::new();
    for p in prims {
        for (pin, conn) in &p.pins {
            let (out, net) = match *conn {
                PrimPin::In(Some(n)) => (false, n),
                PrimPin::Out(Some(n)) => (true, n),
                _ => continue,
            };
            v.push(Probe {
                inst: p.inst,
                pin: pin.clone(),
                out,
                net,
            });
        }
    }
    v
}

/// Ports the check drives: inputs (inout pads read through their BB).
fn is_input(p: &crate::netlist::Port) -> bool {
    !p.output && !p.bidir
}
use crate::{Result, err};

/// xorshift64*, deterministic.
#[derive(Clone, Copy, Debug)]
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Rng {
        Rng(seed | 1)
    }
    pub fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
}

fn lut4(init: u16, ins: [u64; 4]) -> u64 {
    let mut z = 0u64;
    for j in 0..16u16 {
        if (init >> j) & 1 == 0 {
            continue;
        }
        let mut m = !0u64;
        for (i, &v) in ins.iter().enumerate() {
            m &= if (j >> i) & 1 == 1 { v } else { !v };
        }
        z |= m;
    }
    z
}

/// A memory, 64 lanes: `bits[word * width + j]` holds bit `j` of the word
/// in every lane.
struct LaneMem {
    width: usize,
    words: usize,
    bits: Vec<u64>,
}

impl LaneMem {
    fn new(width: usize, words: usize) -> LaneMem {
        LaneMem {
            width,
            words,
            bits: vec![0; width * words],
        }
    }
    fn addr(&self, addr: &[u64], lane: usize) -> usize {
        let mut a = 0usize;
        for (k, &v) in addr.iter().enumerate() {
            a |= (((v >> lane) & 1) as usize) << k;
        }
        a % self.words
    }
    fn read(&self, addr: &[u64]) -> Vec<u64> {
        let mut out = vec![0u64; self.width];
        for lane in 0..64 {
            let a = self.addr(addr, lane);
            for (j, o) in out.iter_mut().enumerate() {
                *o |= ((self.bits[a * self.width + j] >> lane) & 1) << lane;
            }
        }
        out
    }
    fn write(&mut self, addr: &[u64], data: &[u64], en: &[u64]) {
        let n = self.width.min(data.len()).min(en.len());
        for lane in 0..64 {
            let a = self.addr(addr, lane);
            for j in 0..n {
                if (en[j] >> lane) & 1 == 1 {
                    let w = &mut self.bits[a * self.width + j];
                    *w = (*w & !(1 << lane)) | (((data[j] >> lane) & 1) << lane);
                }
            }
        }
    }
}

/// Simulate the netlist: per cycle, the input port values (one u64 of
/// lanes per input port bit, in port order) in, the output port values out.
pub fn simulate_netlist(nl: &Netlist, inputs: &[Vec<u64>]) -> Result<Vec<Vec<u64>>> {
    let n = nl.nets as usize;
    let mut val = vec![0u64; n];
    // Flop state, in cell order.
    let mut state: HashMap<usize, u64> = HashMap::new();
    for (i, c) in nl.cells.iter().enumerate() {
        if let Cell::Ff { regset, .. } = c {
            state.insert(i, if *regset == RegSet::Set { !0 } else { 0 });
        }
    }
    // Block RAM contents and read registers, in cell order.
    let mut rams: HashMap<usize, (LaneMem, Vec<u64>)> = HashMap::new();
    for (i, c) in nl.cells.iter().enumerate() {
        if let Cell::Ebr(e) = c {
            let cw = e.width.bits();
            let mut m = LaneMem::new(cw, e.width.depth());
            for w in 0..e.width.depth() {
                for j in 0..cw {
                    let (word, bit) = match e.width {
                        EbrWidth::X9 => (w, j),
                        EbrWidth::X18 => (2 * w + j / 9, j % 9),
                    };
                    if (e.init[word] >> bit) & 1 == 1 {
                        m.bits[w * cw + j] = !0;
                    }
                }
            }
            rams.insert(i, (m, vec![0; e.read_width.bits()]));
        }
    }
    let input_ports: Vec<usize> = (0..nl.ports.len()).filter(|&p| is_input(&nl.ports[p])).collect();
    let output_ports: Vec<usize> = (0..nl.ports.len()).filter(|&p| nl.ports[p].output).collect();
    let probes = probes(nl);
    // CCU2C chains: heads (cin None) and the cell after each.
    let mut next_of: HashMap<usize, usize> = HashMap::new();
    for (i, c) in nl.cells.iter().enumerate() {
        if let Cell::Ccu2 { cin: Some(p), .. } = c {
            next_of.insert(*p as usize, i);
        }
    }
    let mut out = Vec::new();
    for ins in inputs {
        let mut pv: HashMap<usize, u64> = HashMap::new();
        for (k, &p) in input_ports.iter().enumerate() {
            pv.insert(p, ins[k]);
        }
        // Sources.
        let mut free = input_ports.len();
        for pr in probes.iter().filter(|p| p.out) {
            val[pr.net as usize] = ins[free];
            free += 1;
        }
        for (i, c) in nl.cells.iter().enumerate() {
            match c {
                Cell::Ib { port, o } => val[*o as usize] = pv[port],
                Cell::Ff { q, .. } => val[*q as usize] = state[&i],
                Cell::Ebr(e) => {
                    let reg = &rams[&i].1;
                    for (j, d) in e.read.data.iter().enumerate() {
                        if let Some(d) = d {
                            val[*d as usize] = reg[j];
                        }
                    }
                }
                _ => {}
            }
        }
        // Async set/reset visible now.
        let apply_async = |val: &mut Vec<u64>| {
            for (i, c) in nl.cells.iter().enumerate() {
                if let Cell::Ff { q, ctrl, regset, .. } = c {
                    if let Some(l) = ctrl.lsr.filter(|l| l.mode == SrMode::Async) {
                        let a = val[l.net as usize] ^ if l.polarity == Polarity::Neg { !0 } else { 0 };
                        let v = if *regset == RegSet::Set { !0 } else { 0 };
                        let s = state[&i];
                        val[*q as usize] = (s & !a) | (v & a);
                    }
                }
            }
        };
        // Settle combinational logic: iterate to a fixed point.
        let mut settled = false;
        for _ in 0..(nl.cells.len() + 2) {
            apply_async(&mut val);
            let before = val.clone();
            for (i, c) in nl.cells.iter().enumerate() {
                match c {
                    Cell::Lut { inputs, init, z } => {
                        let ins = inputs.map(|x| x.map_or(0, |n| val[n as usize]));
                        val[*z as usize] = lut4(*init, ins);
                    }
                    Cell::Ccu2 { cin: None, .. } => {
                        // Walk the chain from its head.
                        let mut carry = 0u64;
                        let mut k = i;
                        loop {
                            let Cell::Ccu2 { halves, .. } = &nl.cells[k] else {
                                return Err(err!("chain link {k} is not a CCU2C"));
                            };
                            for h in halves {
                                let ins = h.inputs.map(|x| x.map_or(0, |n| val[n as usize]));
                                let l4 = lut4(h.init, ins);
                                let l2 = lut4(h.init & 0xF, [ins[0], ins[1], 0, 0]);
                                if let Some(s) = h.s {
                                    val[s as usize] = l4 ^ carry;
                                }
                                carry = (!l4 & l2) | (l4 & carry);
                            }
                            match next_of.get(&k) {
                                Some(&nx) => k = nx,
                                None => break,
                            }
                        }
                    }
                    _ => {}
                }
            }
            if val == before {
                settled = true;
                break;
            }
        }
        if !settled {
            return Err(err!("combinational logic does not settle"));
        }
        let mut o = nl
            .cells
            .iter()
            .filter_map(|c| match c {
                Cell::Ob { port, i } => Some((*port, val[*i as usize])),
                Cell::Prim(p) if p.ty == "OBZ" => {
                    let Some(PrimPin::In(Some(i))) = p.pin("I") else { return None };
                    p.pad.map(|port| (port, val[i as usize]))
                }
                _ => None,
            })
            .fold(vec![0u64; output_ports.len()], |mut acc, (p, v)| {
                let k = output_ports.iter().position(|&x| x == p).expect("output port");
                acc[k] = v;
                acc
            });
        o.extend(probes.iter().filter(|p| !p.out).map(|p| val[p.net as usize]));
        out.push(o);
        // Clock edge.
        for (i, c) in nl.cells.iter().enumerate() {
            if let Cell::Ff { d, ctrl, regset, .. } = c {
                let ce = ctrl.ce.map_or(!0, |(n, p)| {
                    val[n as usize] ^ if p == Polarity::Neg { !0 } else { 0 }
                });
                let v = if *regset == RegSet::Set { !0 } else { 0 };
                let mut next = (state[&i] & !ce) | (val[*d as usize] & ce);
                if let Some(l) = ctrl.lsr {
                    let a = val[l.net as usize] ^ if l.polarity == Polarity::Neg { !0 } else { 0 };
                    next = (next & !a) | (v & a);
                }
                state.insert(i, next);
            }
        }
        for (i, c) in nl.cells.iter().enumerate() {
            if let Cell::Ebr(e) = c {
                let (m, reg) = rams.get_mut(&i).expect("ram state");
                let addr: Vec<u64> = e.read.addr.iter().map(|&n| val[n as usize]).collect();
                let next = if e.read_width == e.width {
                    m.read(&addr)
                } else {
                    // x18 read of x9 contents: words 2 addr, 2 addr + 1.
                    let mut v = Vec::with_capacity(18);
                    for lsb in [0u64, !0] {
                        let mut a = vec![lsb];
                        a.extend(&addr);
                        v.extend(m.read(&a));
                    }
                    v
                };
                if let Some(w) = &e.write {
                    let addr: Vec<u64> = w.addr.iter().map(|&n| val[n as usize]).collect();
                    let data: Vec<u64> = w
                        .data
                        .iter()
                        .map(|d| d.map_or(0, |n| val[n as usize]))
                        .collect();
                    let en: Vec<u64> = (0..data.len())
                        .map(|j| w.we[j / 9].map_or(0, |n| val[n as usize]))
                        .collect();
                    m.write(&addr, &data, &en);
                }
                *reg = next;
            }
        }
    }
    Ok(out)
}

/// Values of every AIG node, given the input node values.
fn eval_aig(g: &Aig, input_vals: &HashMap<u32, u64>) -> Vec<u64> {
    let mut v = vec![0u64; g.nodes.len()];
    let lit = |v: &Vec<u64>, l: Lit| v[l.node() as usize] ^ if l.compl() { !0 } else { 0 };
    for (k, n) in g.nodes.iter().enumerate() {
        v[k] = match *n {
            Node::Const => 0,
            Node::Input(_) => input_vals.get(&(k as u32)).copied().unwrap_or(0),
            Node::And(a, b) => lit(&v, a) & lit(&v, b),
        };
    }
    v
}

/// Simulate the lowered design (the synthesis input) the same way.
pub fn simulate_lowered(lw: &Lowered, inputs: &[Vec<u64>], probes: &[Probe]) -> Vec<Vec<u64>> {
    use oxibit_hdl::netlist::Dir;
    let g = &lw.aig;
    let pin_lit = |inst: usize, pin: &str| -> Lit {
        lw.insts[inst]
            .conns
            .iter()
            .find(|(p, _)| p == pin)
            .and_then(|(_, b)| b.first().copied())
            .unwrap_or(Lit::FALSE)
    };
    // Pass-through primitives: (from, to).
    let through: Vec<(Lit, Lit)> = lw
        .insts
        .iter()
        .enumerate()
        .filter_map(|(k, i)| {
            let (f, t) = match i.ty.as_str() {
                "IB" | "OB" | "OBZ" => ("I", "O"),
                "DCCA" => ("CLKI", "CLKO"),
                _ => return None,
            };
            Some((pin_lit(k, f), pin_lit(k, t)))
        })
        .filter(|(_, t)| !t.is_const())
        .collect();
    let lit = |v: &Vec<u64>, l: Lit| v[l.node() as usize] ^ if l.compl() { !0 } else { 0 };
    let mut state: Vec<u64> = lw.ffs.iter().map(|f| if f.init { !0 } else { 0 }).collect();
    // Memories (2^abits words) and their read registers.
    let mut mems: Vec<(LaneMem, usize, Vec<Vec<u64>>)> = lw
        .mems
        .iter()
        .map(|ml| {
            let width = ml.mem.width as usize;
            let abits = crate::synth::ceil_log2(ml.mem.size).max(1);
            let mut m = LaneMem::new(width, 1 << abits);
            for (&a, bits) in &ml.mem.init {
                if (a as usize) < m.words {
                    for (j, b) in bits.iter().enumerate().take(width) {
                        if *b == Some(true) {
                            m.bits[a as usize * width + j] = !0;
                        }
                    }
                }
            }
            let regs = ml.rd.iter().map(|(_, _, d)| vec![0u64; d.len()]).collect();
            (m, abits, regs)
        })
        .collect();
    let mut out = Vec::new();
    for ins in inputs {
        let mut iv: HashMap<u32, u64> = HashMap::new();
        let mut k = 0;
        for (_, dir, _, bits) in &lw.ports {
            if *dir == Dir::Input {
                for &b in bits {
                    iv.insert(b.node(), ins[k]);
                    k += 1;
                }
            }
        }
        for pr in probes.iter().filter(|p| p.out) {
            let l = pin_lit(pr.inst, &pr.pin);
            if !l.is_const() {
                iv.insert(l.node(), ins[k]);
            }
            k += 1;
        }
        for (f, &s) in lw.ffs.iter().zip(&state) {
            iv.insert(f.q.node(), s);
        }
        for (ml, (_, _, regs)) in lw.mems.iter().zip(&mems) {
            for ((_, _, data), reg) in ml.rd.iter().zip(regs) {
                for (d, &r) in data.iter().zip(reg) {
                    iv.insert(d.node(), r);
                }
            }
        }
        let mut v = eval_aig(g, &iv);
        // Async set/reset and carry chains until stable.
        for _ in 0..(g.chains.len() + lw.ffs.len() + through.len() + 2) {
            let before = iv.clone();
            for &(f, t) in &through {
                iv.insert(t.node(), lit(&v, f) ^ if t.compl() { !0 } else { 0 });
            }
            for (f, &s) in lw.ffs.iter().zip(&state) {
                if matches!(f.kind, FfKind::Fdce | FfKind::Fdpe) && f.sr != Lit::FALSE {
                    let a = lit(&v, f.sr) ^ if f.sr_inv { !0 } else { 0 };
                    let val = if f.kind == FfKind::Fdpe { !0 } else { 0 };
                    iv.insert(f.q.node(), (s & !a) | (val & a));
                }
            }
            for c in &g.chains {
                let mut carry = lit(&v, c.cin);
                for i in 0..c.s.len() {
                    let s = lit(&v, c.s[i]);
                    let di = lit(&v, c.di[i]);
                    iv.insert(c.o[i].node(), s ^ carry);
                    carry = (s & carry) | (!s & di);
                }
                iv.insert(c.co.node(), carry);
            }
            v = eval_aig(g, &iv);
            if iv == before {
                break;
            }
        }
        let mut o = Vec::new();
        for (_, dir, _, bits) in &lw.ports {
            if *dir == Dir::Output {
                for &b in bits {
                    o.push(lit(&v, b));
                }
            }
        }
        o.extend(probes.iter().filter(|p| !p.out).map(|p| lit(&v, pin_lit(p.inst, &p.pin))));
        out.push(o);
        for (f, s) in lw.ffs.iter().zip(state.iter_mut()) {
            let ce = lit(&v, f.ce);
            let mut next = (*s & !ce) | (lit(&v, f.d) & ce);
            if f.sr != Lit::FALSE {
                let a = lit(&v, f.sr) ^ if f.sr_inv { !0 } else { 0 };
                let val = if matches!(f.kind, FfKind::Fdse | FfKind::Fdpe) { !0 } else { 0 };
                next = (next & !a) | (val & a);
            }
            *s = next;
        }
        for (ml, (m, abits, regs)) in lw.mems.iter().zip(mems.iter_mut()) {
            let addr = |a: &Vec<Lit>| -> Vec<u64> {
                (0..*abits).map(|k| a.get(k).map_or(0, |&l| lit(&v, l))).collect()
            };
            let next: Vec<Vec<u64>> = ml
                .rd
                .iter()
                .map(|(_, a, d)| {
                    let mut r = m.read(&addr(a));
                    r.truncate(d.len());
                    r
                })
                .collect();
            for (_, a, d, e) in &ml.wr {
                let data: Vec<u64> = d.iter().map(|&l| lit(&v, l)).collect();
                let en: Vec<u64> = e.iter().map(|&l| lit(&v, l)).collect();
                m.write(&addr(a), &data, &en);
            }
            *regs = next;
        }
    }
    out
}

/// Simulate the synthesized netlist against the lowered design for
/// `cycles` cycles of random inputs (64 lanes each). Returns the first
/// mismatch as (cycle, output port name) if any.
pub fn check_synthesis(
    lw: &Lowered,
    nl: &Netlist,
    cycles: usize,
    seed: u64,
) -> Result<Option<(usize, String)>> {
    let pr = probes(nl);
    let n_in = nl.ports.iter().filter(|p| is_input(p)).count() + pr.iter().filter(|p| p.out).count();
    let mut rng = Rng::new(seed);
    let inputs: Vec<Vec<u64>> = (0..cycles)
        .map(|_| (0..n_in).map(|_| rng.next()).collect())
        .collect();
    let a = simulate_lowered(lw, &inputs, &pr);
    let b = simulate_netlist(nl, &inputs)?;
    let outs: Vec<String> = nl
        .ports
        .iter()
        .filter(|p| p.output)
        .map(|p| p.name.clone())
        .chain(
            pr.iter()
                .filter(|p| !p.out)
                .map(|p| format!("{}.{}", lw.insts[p.inst].name, p.pin)),
        )
        .collect();
    for (c, (x, y)) in a.iter().zip(&b).enumerate() {
        if let Some(k) = (0..x.len()).find(|&k| x[k] != y[k]) {
            return Ok(Some((c, outs[k].clone())));
        }
    }
    Ok(None)
}
