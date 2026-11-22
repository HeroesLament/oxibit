//! Synthesis for ECP5: Verilog through oxibit-hdl and oxibit-synth's
//! target-neutral lowering, then LUT4 covering and ECP5 cells.
//!
//! `oxibit_synth::lower` bit-blasts the design into an AIG between cut
//! points (ports, flops, carry chains), with flop enables and resets found;
//! `map::map_k` covers it with 4-input LUTs. This module turns the result
//! into the ECP5 cell netlist (`netlist`):
//!
//! - Each LUT of the cover is a `Lut`, INIT from its truth table
//!   replicated over unused inputs. A complemented use of a LUT output gets
//!   a copy with the table inverted; of a cut point, a LUT1 inverter.
//! - A flop is a slice flop: kind and polarity onto `ControlSet`, `RegSet`.
//! - A carry chain (`s`, `di`, `cin` -> `o`, `co`, CARRY4 semantics:
//!   `o[i] = s[i] ^ c[i]`, `c[i+1] = s[i] ? c[i] : di[i]`) becomes CCU2C
//!   halves with INJECT1 off. A half's LUT4 is the propagate `s` and its
//!   LUT2 (`INIT[3:0]` over A, B) the generate `di`, so `D` is tied high
//!   (a VCC net) and the propagate table sits in the D = 1 half of INIT.
//!   When `s` is a LUT of at most three inputs that include `di` (an
//!   adder's `a ^ b` with `di = a`) the half takes that LUT's inputs
//!   directly and the LUT goes. The chain starts with a feed-in half that
//!   puts `cin` on the carry and ends, when `co` is used, with a feed-out
//!   half whose sum is the carry.
//!
//! - A memory (after `memdff` folds read-data registers into its read
//!   ports and `memports` splits it per read port) with one synchronous
//!   read port and at most one write port becomes DP16KD columns
//!   (`map_mems`): x9 (2048 deep) or, for wider memories at most 1024
//!   deep, x18 with one write enable per 9-bit lane. Writes through
//!   address bits above the memory's range are gated off (the source
//!   writes nothing there; the RAM would alias).
//!
//! - Instances of the hard primitives in `PRIMITIVES` are kept: an
//!   `EHXPLLL` becomes a `Prim` cell (its outputs are cut points, a
//!   feedback from one of its own outputs becomes the internal path);
//!   `BB` (on an inout port) and `OBZ` (on an output port) become pad
//!   `Prim`s; `IB`, `OB` and `DCCA` dissolve into the pad cells and the
//!   clock network the flow builds anyway.
//!
//! Not yet: asynchronous reads (distributed RAM), memories deeper than
//! 2048 words, other primitives.

use std::collections::{HashMap, HashSet};

use oxibit_hdl::netlist::{Dir, Module};
use oxibit_synth::aig::{Lit, Node};
use oxibit_synth::lib_ports::PrimPorts;
use oxibit_synth::lower::{self, FfKind, Lowered};
use oxibit_synth::map::{self, Mapping};

use crate::netlist::{
    CarryHalf, Cell, CellId, ControlSet, Ebr, EbrRead, EbrWidth, EbrWrite, Lsr, NetId, Netlist,
    Polarity, Port, Prim, PrimPin, RegSet, SrMode,
};

/// The primitives synthesis keeps, as port declarations (directions only;
/// every pin is one bit).
pub const PRIMITIVES: &str = "
module EHXPLLL(input CLKI, CLKFB, PHASESEL1, PHASESEL0, PHASEDIR, PHASESTEP,
  PHASELOADREG, STDBY, PLLWAKESYNC, RST, ENCLKOP, ENCLKOS, ENCLKOS2, ENCLKOS3,
  output CLKOP, CLKOS, CLKOS2, CLKOS3, LOCK, INTLOCK, REFCLK, CLKINTFB);
endmodule
module DCCA(input CLKI, CE, output CLKO);
endmodule
module BB(input I, T, output O, inout B);
endmodule
module OBZ(input I, T, output O);
endmodule
module IB(input I, output O);
endmodule
module OB(input I, output O);
endmodule
module JTAGG(input TCK, TMS, TDI, JTDO2, JTDO1,
  output TDO, JTDI, JTCK, JRTI2, JRTI1, JSHIFT, JUPDATE, JRSTN, JCE2, JCE1);
endmodule
";

/// The primitive port table synthesis lowers instances against.
pub fn primitive_ports() -> PrimPorts {
    oxibit_synth::lib_ports::scan(PRIMITIVES)
}
use crate::{Result, err};

/// What synthesis is asked to do.
#[derive(Clone, Copy, Debug)]
pub struct SynthOptions {
    /// Adders and comparators at least this wide use carry chains.
    pub carry_min: usize,
    pub area_passes: usize,
    pub exact_passes: usize,
}

impl Default for SynthOptions {
    fn default() -> Self {
        SynthOptions {
            carry_min: 4,
            area_passes: 2,
            exact_passes: 1,
        }
    }
}

/// What synthesis reports.
#[derive(Clone, Debug, Default)]
pub struct SynthReport {
    pub luts: usize,
    pub inverters: usize,
    pub ccu2: usize,
    pub carry_halves_absorbed: usize,
    pub ffs: usize,
    /// DP16KD blocks, and the memories they implement.
    pub ebrs: usize,
    pub memories: usize,
    /// Kept hard primitives (PLLs, pad buffers).
    pub prims: usize,
    pub depth: u32,
    pub warnings: Vec<String>,
}

/// Parse `src` and elaborate its last module (or `top`).
pub fn elaborate(src: &str, top: Option<&str>) -> Result<Module> {
    let mods = oxibit_hdl::parser::parse(src).map_err(|e| err!("parse: {e}"))?;
    let m = match top {
        Some(t) => mods
            .iter()
            .find(|m| m.name == t)
            .ok_or_else(|| err!("no module {t}"))?,
        None => mods.last().ok_or(err!("no module"))?,
    };
    oxibit_hdl::elab::Elab::module(m).map_err(|e| err!("elaborate: {e}"))
}

/// 16-bit INIT of a cut's truth table over `k` leaves (leaf i is LUT input
/// i), replicated so the unused inputs do not matter.
fn lut_init(tt: u64, k: usize) -> u16 {
    let mask = (1usize << k) - 1;
    let mut init = 0u16;
    for j in 0..16 {
        if (tt >> (j & mask)) & 1 == 1 {
            init |= 1 << j;
        }
    }
    init
}

/// INIT of a carry half: `prop` (a 3-input table over A, B, C) in the
/// D = 1 rows, `gen` (a 2-input table over A, B) in `INIT[3:0]`. The
/// D = 0, C = 1 rows are never read (D is tied high) and repeat `gen`.
fn carry_init(prop: u8, gen_ab: u8) -> u16 {
    let g = u16::from(gen_ab & 0xF);
    let p = u16::from(prop);
    g | (g << 4) | (p << 8)
}

struct Builder<'a> {
    lw: &'a Lowered,
    mp: &'a Mapping,
    nl: Netlist,
    /// Net of each AIG node's positive literal, where it has one.
    net: HashMap<u32, NetId>,
    /// Net of a node's complement.
    inv: HashMap<u32, NetId>,
    /// Per lowered memory, how it maps.
    plans: &'a [MemPlan],
    vcc: Option<NetId>,
    gnd: Option<NetId>,
    report: SynthReport,
}

/// How one lowered memory (`lw.mems[i]`) maps onto DP16KDs.
#[derive(Clone, Copy, Debug)]
pub struct MemPlan {
    pub width: EbrWidth,
    /// Word address bits the memory uses.
    pub abits: usize,
    /// DP16KD columns (data bits per column: `width.bits()`).
    pub columns: usize,
}

pub(crate) fn ceil_log2(n: u32) -> usize {
    if n <= 1 {
        0
    } else {
        (32 - (n - 1).leading_zeros()) as usize
    }
}

/// Check each memory fits the DP16KD mapping, choose its shape, and gate
/// writes through address bits above its range.
fn plan_mems(lw: &mut Lowered) -> Result<Vec<MemPlan>> {
    let mut plans = Vec::new();
    for k in 0..lw.mems.len() {
        let ml = &lw.mems[k];
        let name = ml.mem.name.clone();
        if ml.mem.offset != 0 {
            return Err(err!("memory {name}: nonzero address offset is not supported yet"));
        }
        if ml.rd.len() != 1 {
            return Err(err!("memory {name}: {} read ports after splitting", ml.rd.len()));
        }
        if ml.wr.len() > 1 {
            return Err(err!("memory {name}: {} write ports", ml.wr.len()));
        }
        let (rclk, _, rdata) = &ml.rd[0];
        if rclk.is_none() {
            return Err(err!(
                "memory {name}: asynchronous read; register the read data \
                 (dout <= mem[addr] in a clocked block) to use block RAM"
            ));
        }
        for d in rdata {
            if d.is_const() || d.compl() {
                return Err(err!("memory {name}: read data bit is not a cut point"));
            }
        }
        let width = ml.mem.width as usize;
        let abits = ceil_log2(ml.mem.size).max(1);
        let shape = if abits <= 10 && width > 9 {
            EbrWidth::X18
        } else if abits <= 11 {
            EbrWidth::X9
        } else {
            return Err(err!(
                "memory {name}: {} words; deeper than 2048 is not supported yet",
                ml.mem.size
            ));
        };
        let cols = width.div_ceil(shape.bits());
        // Lane enables must be one signal per 9-bit lane.
        if let Some((_, _, _, en)) = ml.wr.first() {
            for lane in 0..width.div_ceil(9) {
                let bits = &en[lane * 9..((lane + 1) * 9).min(en.len())];
                if bits.iter().any(|&e| e != bits[0]) {
                    return Err(err!(
                        "memory {name}: per-bit write enables inside 9-bit lane {lane} are not supported"
                    ));
                }
            }
        }
        // Gate writes above the range.
        let has_wr = !lw.mems[k].wr.is_empty();
        if has_wr {
            let upper: Vec<Lit> = lw.mems[k].wr[0].1.iter().skip(abits).copied().collect();
            if !upper.is_empty() {
                let any = lw.aig.or_all(&upper);
                let ok = any.not();
                let en: Vec<Lit> = lw.mems[k].wr[0].3.clone();
                let mut memo: HashMap<Lit, Lit> = HashMap::new();
                let gated: Vec<Lit> = en
                    .iter()
                    .map(|&e| *memo.entry(e).or_insert_with(|| lw.aig.and(e, ok)))
                    .collect();
                lw.mems[k].wr[0].3 = gated;
            }
        }
        plans.push(MemPlan {
            width: shape,
            abits,
            columns: cols,
        });
    }
    Ok(plans)
}

impl Builder<'_> {
    fn constant(&mut self, one: bool) -> NetId {
        let slot = if one { self.vcc } else { self.gnd };
        if let Some(n) = slot {
            return n;
        }
        let z = self.nl.new_net();
        self.nl.add(Cell::Lut {
            inputs: [None; 4],
            init: if one { 0xFFFF } else { 0 },
            z,
        });
        self.nl.net_names.insert(z, if one { "$vcc" } else { "$gnd" }.into());
        if one {
            self.vcc = Some(z);
        } else {
            self.gnd = Some(z);
        }
        z
    }

    /// The net of a literal; its node's net must exist (LUT roots are
    /// emitted before their users).
    fn lit(&mut self, l: Lit) -> Result<NetId> {
        if l.is_const() {
            return Ok(self.constant(l == Lit::TRUE));
        }
        let node = l.node();
        let pos = *self
            .net
            .get(&node)
            .ok_or_else(|| err!("AIG node {node} has no net"))?;
        if !l.compl() {
            return Ok(pos);
        }
        if let Some(&n) = self.inv.get(&node) {
            return Ok(n);
        }
        let z = self.nl.new_net();
        // A LUT root: the same LUT with the table inverted. A cut point:
        // a LUT1 inverter on A.
        if let Some(li) = self.mp.lut_of.get(node as usize).copied().flatten() {
            let lut = &self.mp.luts[li as usize];
            let mut inputs = [None; 4];
            for (i, &leaf) in lut.leaves.iter().enumerate() {
                inputs[i] = Some(self.net[&leaf]);
            }
            let init = !lut_init(lut.tt, lut.leaves.len());
            self.nl.add(Cell::Lut { inputs, init, z });
            self.report.luts += 1;
        } else {
            self.nl.add(Cell::Lut {
                inputs: [Some(pos), None, None, None],
                init: 0x5555,
                z,
            });
            self.report.inverters += 1;
        }
        self.inv.insert(node, z);
        Ok(z)
    }
}

/// The AIG nodes a literal's consumers need, by node.
/// The single bit an instance connects to `pin`.
fn inst_bit(inst: &lower::InstL, pin: &str) -> Option<Lit> {
    inst.conns
        .iter()
        .find(|(p, _)| p == pin)
        .and_then(|(_, b)| b.first().copied())
}

/// A PLL whose CLKFB is one of its own outputs, with `FEEDBK_PATH` naming
/// that output, uses the matching internal path (`INT_OP` for `CLKOP`...)
/// and leaves CLKFB unrouted: the loop divides the same way and needs no
/// clock routed back into the PLL. Diamond writes the same `INT_*` setting
/// for internal feedback.
fn internal_feedback(p: &mut Prim, report: &mut SynthReport) {
    use oxibit_hdl::netlist::Param;
    let path = p.param_str("FEEDBK_PATH").unwrap_or_else(|| "CLKOP".to_owned());
    let int = match path.as_str() {
        "CLKOP" => "INT_OP",
        "CLKOS" => "INT_OS",
        "CLKOS2" => "INT_OS2",
        "CLKOS3" => "INT_OS3",
        _ => return,
    };
    let (Some(PrimPin::In(Some(fb))), Some(PrimPin::Out(Some(out)))) = (p.pin("CLKFB"), p.pin(&path)) else {
        return;
    };
    if fb != out {
        return;
    }
    p.params.insert("FEEDBK_PATH".to_owned(), Param::Str(int.to_owned()));
    for (n, pin) in &mut p.pins {
        if n == "CLKFB" {
            *pin = PrimPin::In(None);
        }
    }
    report.warnings.push(format!(
        "PLL {}: feedback from {path} through the internal path ({int})",
        p.name
    ));
}

fn need(set: &mut HashSet<u32>, l: Lit) {
    if !l.is_const() {
        set.insert(l.node());
    }
}

/// Synthesize an elaborated module.
pub fn synth_module(m: &Module, opts: SynthOptions) -> Result<(Netlist, SynthReport)> {
    let (nl, rep, _) = synth_lowered(m, opts)?;
    Ok((nl, rep))
}

/// `synth_module`, also returning the lowered design (for checking).
pub fn synth_lowered(m: &Module, opts: SynthOptions) -> Result<(Netlist, SynthReport, Lowered)> {
    let dirs = primitive_ports();
    for c in &m.cells {
        if let oxibit_hdl::netlist::Cell::Instance { ty, name, .. } = c {
            if !dirs.contains_key(ty) {
                return Err(err!(
                    "instance {name}: primitive {ty} is not supported (supported: EHXPLLL, DCCA, BB, OBZ, IB, OB)"
                ));
            }
        }
    }
    // Read-data registers into the read ports, one copy per read port,
    // multi-write memories to flops.
    let mut m = m.clone();
    oxibit_synth::memdff::merge(&mut m);
    let mp_stats = oxibit_synth::memports::reshape(&mut m);
    if let Some((name, why)) = mp_stats.left.first() {
        return Err(err!("memory {name}: {why}"));
    }
    let mut lw = lower::lower(
        &m,
        &dirs,
        lower::Config {
            carry_min: opts.carry_min,
            lut_k: 4,
        },
    )
    .map_err(|e| err!("lower: {e}"))?;
    let plans = plan_mems(&mut lw)?;
    fold_init_conflicts(&mut lw);
    lw.sweep();
    let outs = lw.outputs();
    let mp = map::map_k(
        &lw.aig,
        &outs,
        4,
        opts.area_passes,
        opts.exact_passes,
        &oxibit_report::Silent,
    );
    let mut b = Builder {
        lw: &lw,
        mp: &mp,
        nl: Netlist {
            top: lw.name.clone(),
            ..Netlist::default()
        },
        net: HashMap::new(),
        inv: HashMap::new(),
        plans: &plans,
        vcc: None,
        gnd: None,
        report: SynthReport {
            depth: mp.depth,
            ..SynthReport::default()
        },
    };
    build(&mut b)?;
    let rep = b.report;
    let nl = b.nl;
    Ok((nl, rep, lw))
}

/// A carry half before it becomes part of a CCU2C.
#[derive(Clone, Copy, Debug)]
struct Half {
    inputs: [Option<Lit>; 3],
    /// Over (A, B, C).
    prop: u8,
    /// Over (A, B).
    gen_ab: u8,
    /// The node whose net the sum drives (None for a feed-in or padding).
    s_node: Option<u32>,
}

fn build(b: &mut Builder<'_>) -> Result<()> {
    let lw = b.lw;
    let mp = b.mp;
    let g = &lw.aig;

    // Pad primitives by the port bit they sit on: BB.B, OB.O / OBZ.O, IB.I.
    let mut pad_inst: HashMap<u32, usize> = HashMap::new();
    for (k, inst) in lw.insts.iter().enumerate() {
        let pad_pin = match inst.ty.as_str() {
            "BB" => "B",
            "OB" | "OBZ" => "O",
            "IB" => "I",
            _ => continue,
        };
        let l = inst_bit(inst, pad_pin)
            .ok_or_else(|| err!("{} {}: {pad_pin} not connected", inst.ty, inst.name))?;
        if l.is_const() || l.compl() {
            return Err(err!("{} {}: {pad_pin} must be a port bit", inst.ty, inst.name));
        }
        if pad_inst.insert(l.node(), k).is_some() {
            return Err(err!("{} {}: its pad has two buffers", inst.ty, inst.name));
        }
    }
    // Port index of each pad primitive.
    let mut pad_port: HashMap<usize, usize> = HashMap::new();

    // Cut points: ports, flop outputs, chain outputs.
    for (name, dir, _, bits) in &lw.ports {
        for (i, &l) in bits.iter().enumerate() {
            let pname = if bits.len() == 1 {
                name.clone()
            } else {
                format!("{name}[{i}]")
            };
            let port = b.nl.ports.len();
            let on = (!l.is_const()).then(|| pad_inst.get(&l.node()).copied()).flatten();
            if *dir == Dir::Inout {
                match on {
                    Some(k) if lw.insts[k].ty == "BB" => {
                        pad_port.insert(k, port);
                    }
                    _ => {
                        return Err(err!(
                            "inout port bit {pname} is not on a BB (tristate pads go through BB)"
                        ));
                    }
                }
            } else if let Some(k) = on {
                let ok = matches!(
                    (dir, lw.insts[k].ty.as_str()),
                    (Dir::Input, "IB") | (Dir::Output, "OB" | "OBZ")
                );
                if !ok {
                    return Err(err!("{} {} on port bit {pname}: wrong direction", lw.insts[k].ty, lw.insts[k].name));
                }
                pad_port.insert(k, port);
            }
            b.nl.ports.push(Port {
                name: pname.clone(),
                output: *dir == Dir::Output,
                bidir: *dir == Dir::Inout,
            });
            if *dir == Dir::Input {
                if l.is_const() || l.compl() {
                    return Err(err!("input port bit {pname} is not a cut point"));
                }
                let o = b.nl.new_net();
                b.nl.add(Cell::Ib { port, o });
                b.nl.net_names.insert(o, pname);
                b.net.insert(l.node(), o);
            }
        }
    }
    for f in &lw.ffs {
        let q = b.nl.new_net();
        b.nl.net_names.insert(q, f.name.clone());
        b.net.insert(f.q.node(), q);
    }
    // Primitive outputs: a PLL's and a BB's are cut points; IB and DCCA
    // pass their input through (the pad and the flow's own DCC).
    let dirs = primitive_ports();
    for inst in &lw.insts {
        if matches!(inst.ty.as_str(), "EHXPLLL" | "BB" | "JTAGG") {
            for (pin, bits) in &inst.conns {
                if dirs[&inst.ty][pin].dir != Dir::Output {
                    continue;
                }
                for &l in bits {
                    if l.is_const() || l.compl() {
                        return Err(err!("{} {}: output {pin} is not a cut point", inst.ty, inst.name));
                    }
                    let n = b.nl.new_net();
                    b.nl.net_names.insert(n, format!("{}.{pin}", inst.name));
                    b.net.insert(l.node(), n);
                }
            }
        }
    }
    for _ in 0..=lw.insts.len() {
        let mut changed = false;
        for inst in &lw.insts {
            let (from, to) = match inst.ty.as_str() {
                "IB" => ("I", "O"),
                "DCCA" => ("CLKI", "CLKO"),
                _ => continue,
            };
            let (Some(f), Some(t)) = (inst_bit(inst, from), inst_bit(inst, to)) else {
                continue;
            };
            if b.net.contains_key(&t.node()) || t.is_const() {
                continue;
            }
            if f.compl() {
                return Err(err!("{} {}: {from} is inverted", inst.ty, inst.name));
            }
            if let Some(&n) = b.net.get(&f.node()) {
                b.net.insert(t.node(), n);
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }
    for inst in &lw.insts {
        if inst.ty == "DCCA" {
            if let Some(ce) = inst_bit(inst, "CE") {
                if ce != Lit::TRUE {
                    return Err(err!(
                        "DCCA {}: a gated clock buffer (CE not tied high) is not supported",
                        inst.name
                    ));
                }
            }
            let ok = inst_bit(inst, "CLKO").is_some_and(|l| b.net.contains_key(&l.node()));
            if !ok {
                return Err(err!(
                    "DCCA {}: CLKI must come from a pad, a PLL or another DCCA",
                    inst.name
                ));
            }
        }
    }
    // Memory read data: cut points the DP16KDs drive.
    for ml in &lw.mems {
        for (_, _, data) in &ml.rd {
            for (j, d) in data.iter().enumerate() {
                let n = b.nl.new_net();
                b.nl.net_names.insert(n, format!("{}.rd[{j}]", ml.mem.name));
                b.net.insert(d.node(), n);
            }
        }
    }

    // Which carry halves absorb their propagate LUT.
    let mut chain_halves: Vec<Vec<Half>> = Vec::new();
    for c in &g.chains {
        let mut halves = Vec::new();
        // Feed-in: LUT4 = 0 (the carry passes the generate), LUT2 = cin.
        halves.push(if c.cin.is_const() {
            Half {
                inputs: [None; 3],
                prop: 0,
                gen_ab: if c.cin == Lit::TRUE { 0xF } else { 0 },
                s_node: None,
            }
        } else {
            Half {
                inputs: [Some(c.cin.positive()), None, None],
                prop: 0,
                gen_ab: if c.cin.compl() { 0x5 } else { 0xA },
                s_node: None,
            }
        });
        for (i, (&s, &di)) in c.s.iter().zip(&c.di).enumerate() {
            let o = c.o[i].node();
            // Absorb: s a LUT root of <= 3 leaves, di constant or a leaf.
            let lut = (!s.is_const())
                .then(|| mp.lut_of.get(s.node() as usize).copied().flatten())
                .flatten()
                .map(|li| &mp.luts[li as usize]);
            let fits = lut.filter(|l| {
                l.leaves.len() <= 3
                    && (di.is_const() || l.leaves.contains(&di.node()))
            });
            if let Some(l) = fits {
                // Put di's leaf on A.
                let mut order: Vec<u32> = l.leaves.clone();
                if !di.is_const() {
                    let p = order.iter().position(|&x| x == di.node()).expect("contains");
                    order.swap(0, p);
                }
                let mut prop = 0u8;
                for m in 0..8u32 {
                    // Value of s with order[k] = bit k of m.
                    let mut idx = 0usize;
                    for (orig, &leaf) in l.leaves.iter().enumerate() {
                        let k = order.iter().position(|&x| x == leaf).expect("permutation");
                        if (m >> k) & 1 == 1 {
                            idx |= 1 << orig;
                        }
                    }
                    let v = ((l.tt >> idx) & 1 == 1) != s.compl();
                    if v {
                        prop |= 1 << m;
                    }
                }
                let gen_ab = if di.is_const() {
                    if di == Lit::TRUE { 0xF } else { 0 }
                } else if di.compl() {
                    0x5
                } else {
                    0xA
                };
                let mut inputs = [None; 3];
                for (k, &leaf) in order.iter().enumerate() {
                    inputs[k] = Some(Lit(leaf << 1));
                }
                halves.push(Half {
                    inputs,
                    prop,
                    gen_ab,
                    s_node: Some(o),
                });
                b.report.carry_halves_absorbed += 1;
            } else {
                // A = di, B = s.
                halves.push(Half {
                    inputs: [Some(di), Some(s), None],
                    prop: 0b1100_1100,
                    gen_ab: 0xA,
                    s_node: Some(o),
                });
            }
        }
        chain_halves.push(halves);
    }
    for (c, halves) in g.chains.iter().zip(&mut chain_halves) {
        // Chain outputs are cut points; their nets exist up front.
        for h in halves.iter() {
            if let Some(o) = h.s_node {
                let n = b.nl.new_net();
                b.net.insert(o, n);
            }
        }
        let co = b.nl.new_net();
        b.net.insert(c.co.node(), co);
        // Feed-out (always present when co has a net; dropped below if
        // nothing reads it).
        halves.push(Half {
            inputs: [None; 3],
            prop: 0,
            gen_ab: 0,
            s_node: Some(c.co.node()),
        });
    }

    // Which LUT roots are needed: from every consumer, through the cover.
    let mut needed: HashSet<u32> = HashSet::new();
    let mut seeds: Vec<Lit> = Vec::new();
    for (_, dir, _, bits) in &lw.ports {
        if *dir == Dir::Output {
            seeds.extend(bits);
        }
    }
    for f in &lw.ffs {
        seeds.extend([f.d, f.ce, f.clk, f.sr]);
    }
    for ml in &lw.mems {
        for (c, a, _) in &ml.rd {
            seeds.extend(c.map(|c| c.0));
            seeds.extend(a);
        }
        for ((c, _), a, d, e) in &ml.wr {
            seeds.push(*c);
            seeds.extend(a.iter().chain(d).chain(e));
        }
    }
    for halves in &chain_halves {
        for h in halves {
            seeds.extend(h.inputs.iter().flatten());
        }
    }
    for inst in &lw.insts {
        for (pin, bits) in &inst.conns {
            if dirs[&inst.ty][pin].dir == Dir::Input {
                seeds.extend(bits);
            }
        }
    }
    let mut stack: Vec<u32> = Vec::new();
    for l in seeds {
        need(&mut needed, l);
    }
    stack.extend(needed.iter().copied());
    while let Some(n) = stack.pop() {
        if let Some(li) = mp.lut_of.get(n as usize).copied().flatten() {
            for &leaf in &mp.luts[li as usize].leaves {
                if needed.insert(leaf) {
                    stack.push(leaf);
                }
            }
        }
    }
    // Emit needed LUT roots in node order (topological: an AND's fanins
    // have lower indices; leaves are inputs or earlier roots).
    let mut roots: Vec<u32> = needed
        .iter()
        .copied()
        .filter(|&n| matches!(g.nodes[n as usize], Node::And(..)))
        .collect();
    roots.sort_unstable();
    for n in roots {
        let li = mp
            .lut_of
            .get(n as usize)
            .copied()
            .flatten()
            .ok_or_else(|| err!("AIG node {n} is used but not a LUT root"))?;
        let lut = &mp.luts[li as usize];
        let mut inputs = [None; 4];
        for (i, &leaf) in lut.leaves.iter().enumerate() {
            inputs[i] = Some(
                *b.net
                    .get(&leaf)
                    .ok_or_else(|| err!("leaf {leaf} of LUT {n} has no net"))?,
            );
        }
        let z = b.nl.new_net();
        b.nl.add(Cell::Lut {
            inputs,
            init: lut_init(lut.tt, lut.leaves.len()),
            z,
        });
        b.net.insert(n, z);
        b.report.luts += 1;
    }

    // What anything reads: every needed node (consumers' literals and
    // the cover's leaves under them). A chain's feed-out stays only when
    // its co is among them.
    let read = needed.clone();

    // Flops.
    for f in &lw.ffs {
        let d = b.lit(f.d)?;
        if f.clk.is_const() {
            return Err(err!("flop {} has a constant clock", f.name));
        }
        let clk = b.net[&f.clk.node()];
        let clk_polarity = if f.clk.compl() != f.clk_inv {
            Polarity::Neg
        } else {
            Polarity::Pos
        };
        let ce = if f.ce == Lit::TRUE {
            None
        } else if f.ce.is_const() {
            Some((b.constant(false), Polarity::Pos))
        } else {
            Some((
                b.net[&f.ce.node()],
                if f.ce.compl() { Polarity::Neg } else { Polarity::Pos },
            ))
        };
        let (mode, value) = match f.kind {
            FfKind::Fdre => (SrMode::Sync, RegSet::Reset),
            FfKind::Fdse => (SrMode::Sync, RegSet::Set),
            FfKind::Fdce => (SrMode::Async, RegSet::Reset),
            FfKind::Fdpe => (SrMode::Async, RegSet::Set),
        };
        let lsr = if f.sr == Lit::FALSE {
            None
        } else if f.sr.is_const() {
            return Err(err!("flop {} has its set/reset tied active", f.name));
        } else {
            Some(Lsr {
                net: b.net[&f.sr.node()],
                polarity: if f.sr.compl() != f.sr_inv {
                    Polarity::Neg
                } else {
                    Polarity::Pos
                },
                mode,
            })
        };
        let regset = if lsr.is_none() {
            if f.init { RegSet::Set } else { RegSet::Reset }
        } else {
            let init = if f.init { RegSet::Set } else { RegSet::Reset };
            if init != value {
                b.report.warnings.push(format!(
                    "flop {}: initial value {} differs from its {} value; powers up at the latter",
                    f.name,
                    u8::from(f.init),
                    if value == RegSet::Set { "set" } else { "reset" }
                ));
            }
            value
        };
        let q = b.net[&f.q.node()];
        b.nl.add(Cell::Ff {
            d,
            q,
            ctrl: ControlSet {
                clk,
                clk_polarity,
                lsr,
                ce,
            },
            regset,
        });
        b.report.ffs += 1;
    }
    // Block RAM.
    for (ml, &plan) in lw.mems.iter().zip(b.plans) {
        emit_mem(b, ml, plan)?;
    }
    // Output pads (an OB is the pad itself; an OBZ is a pad primitive).
    let mut port = 0usize;
    for (_, dir, _, bits) in &lw.ports {
        for &l in bits {
            if *dir == Dir::Output {
                let on = (!l.is_const()).then(|| pad_inst.get(&l.node()).copied()).flatten();
                match on {
                    Some(k) if lw.insts[k].ty == "OB" => {
                        let src = inst_bit(&lw.insts[k], "I").unwrap_or(Lit::FALSE);
                        let i = b.lit(src)?;
                        b.nl.add(Cell::Ob { port, i });
                    }
                    Some(_) => {}
                    None => {
                        let i = b.lit(l)?;
                        b.nl.add(Cell::Ob { port, i });
                    }
                }
            }
            port += 1;
        }
    }
    // Kept primitives, in instance order.
    for (k, inst) in lw.insts.iter().enumerate() {
        if !matches!(inst.ty.as_str(), "EHXPLLL" | "BB" | "OBZ" | "JTAGG") {
            continue;
        }
        let mut pins = Vec::new();
        for (pin, bits) in &inst.conns {
            let d = dirs[&inst.ty][pin].dir;
            let Some(&l) = bits.first() else { continue };
            match d {
                Dir::Input => pins.push((pin.clone(), PrimPin::In(Some(b.lit(l)?)))),
                Dir::Output => {
                    let n = if inst.ty == "OBZ" { None } else { b.net.get(&l.node()).copied() };
                    pins.push((pin.clone(), PrimPin::Out(n)));
                }
                Dir::Inout => {}
            }
        }
        let mut prim = Prim {
            ty: inst.ty.clone(),
            name: inst.name.clone(),
            params: inst.params.clone(),
            attrs: inst.attrs.clone(),
            pins,
            pad: pad_port.get(&k).copied(),
            inst: k,
        };
        if prim.ty == "EHXPLLL" {
            internal_feedback(&mut prim, &mut b.report);
        }
        b.nl.add(Cell::Prim(Box::new(prim)));
        b.report.prims += 1;
    }

    // Carry chains: halves into CCU2Cs, the feed-out dropped when nothing
    // reads co.
    for (c, mut halves) in g.chains.iter().zip(chain_halves) {
        if !read.contains(&c.co.node()) {
            halves.pop();
        }
        if halves.len() % 2 == 1 {
            halves.push(Half {
                inputs: [None; 3],
                prop: 0,
                gen_ab: 0,
                s_node: None,
            });
        }
        let vcc = b.constant(true);
        let mut prev: Option<CellId> = None;
        let n = halves.len() / 2;
        for (k, pair) in halves.chunks(2).enumerate() {
            let mut hs = [CarryHalf {
                inputs: [None; 4],
                init: 0,
                s: None,
            }; 2];
            for (j, h) in pair.iter().enumerate() {
                let mut inputs = [None; 4];
                for (i, l) in h.inputs.iter().enumerate() {
                    if let Some(l) = l {
                        inputs[i] = Some(b.lit(*l)?);
                    }
                }
                inputs[3] = Some(vcc);
                hs[j] = CarryHalf {
                    inputs,
                    init: carry_init(h.prop, h.gen_ab),
                    s: h.s_node.map(|o| b.net[&o]),
                };
            }
            let id = b.nl.add(Cell::Ccu2 {
                halves: hs,
                cin: prev,
                cout_used: k + 1 < n,
            });
            prev = Some(id);
            b.report.ccu2 += 1;
        }
    }
    Ok(())
}

/// A memory port clock: its net and edge.
fn port_clock(b: &mut Builder<'_>, l: Lit, posedge: bool, what: &str) -> Result<(NetId, Polarity)> {
    if l.is_const() {
        return Err(err!("{what} has a constant clock"));
    }
    let n = match b.net.get(&l.node()) {
        Some(&n) => n,
        None => b.lit(l.positive())?,
    };
    let pol = if l.compl() == posedge {
        Polarity::Neg
    } else {
        Polarity::Pos
    };
    Ok((n, pol))
}

/// The DP16KD columns of one memory.
fn emit_mem(b: &mut Builder<'_>, ml: &lower::MemL, plan: MemPlan) -> Result<()> {
    let name = ml.mem.name.clone();
    let width = ml.mem.width as usize;
    let cw = plan.width.bits();
    let (rclk, raddr, rdata) = &ml.rd[0];
    let (rclk_l, rpos) = rclk.ok_or_else(|| err!("memory {name}: asynchronous read"))?;
    let (rclk, rpol) = port_clock(b, rclk_l, rpos, &format!("memory {name} read port"))?;
    let mut addr_nets = |b: &mut Builder<'_>, a: &[Lit]| -> Result<Vec<NetId>> {
        let mut v = Vec::with_capacity(plan.abits);
        for k in 0..plan.abits {
            v.push(match a.get(k) {
                Some(&l) => b.lit(l)?,
                None => b.constant(false),
            });
        }
        Ok(v)
    };
    let raddr = addr_nets(b, raddr)?;
    let write = match ml.wr.first() {
        Some(((c, pos), a, d, e)) => {
            let (clk, pol) = port_clock(b, *c, *pos, &format!("memory {name} write port"))?;
            Some((clk, pol, addr_nets(b, a)?, d.clone(), e.clone()))
        }
        None => None,
    };
    // Port B does not work x9 on silicon (see `ebr`): an x9 memory reads
    // port B at x18, row `addr >> 1`, and a flop of the address LSB picks
    // the word.
    let wide = plan.width == EbrWidth::X9;
    let (read_width, row_addr, sel) = if wide {
        let sel = match raddr.first() {
            Some(&a0) => {
                let q = b.nl.new_net();
                b.nl.net_names.insert(q, format!("{name}.rd_sel"));
                b.nl.add(Cell::Ff {
                    d: a0,
                    q,
                    ctrl: ControlSet {
                        clk: rclk,
                        clk_polarity: rpol,
                        lsr: None,
                        ce: None,
                    },
                    regset: RegSet::Reset,
                });
                b.report.ffs += 1;
                Some(q)
            }
            None => None,
        };
        (EbrWidth::X18, raddr.get(1..).unwrap_or_default().to_vec(), sel)
    } else {
        (plan.width, raddr.clone(), None)
    };
    for col in 0..plan.columns {
        let base = col * cw;
        let mut rd = Vec::with_capacity(read_width.bits());
        for j in 0..cw {
            let i = base + j;
            rd.push(if i < width && i < rdata.len() {
                Some(b.net[&rdata[i].node()])
            } else {
                None
            });
        }
        if wide {
            // DOB0..8: the even word, DOB9..17: the odd one; the word
            // picked drives the read data net.
            let out = std::mem::take(&mut rd);
            let mut hi = Vec::with_capacity(cw);
            for (j, o) in out.into_iter().enumerate() {
                let Some(z) = o else {
                    rd.push(None);
                    hi.push(None);
                    continue;
                };
                let lo_n = b.nl.new_net();
                let hi_n = b.nl.new_net();
                b.nl.net_names.insert(lo_n, format!("{name}.rd_lo[{}]", base + j));
                b.nl.net_names.insert(hi_n, format!("{name}.rd_hi[{}]", base + j));
                rd.push(Some(lo_n));
                hi.push(Some(hi_n));
                let (inputs, init) = match sel {
                    // Z = C ? B : A.
                    Some(s) => ([Some(lo_n), Some(hi_n), Some(s), None], 0xCACA),
                    None => ([Some(lo_n), None, None, None], 0xAAAA),
                };
                b.nl.add(Cell::Lut { inputs, init, z });
                b.report.luts += 1;
            }
            rd.extend(hi);
        }
        let wr = match &write {
            Some((clk, pol, addr, d, e)) => {
                let mut data = Vec::with_capacity(cw);
                for j in 0..cw {
                    let i = base + j;
                    data.push(if i < width && i < d.len() {
                        Some(b.lit(d[i])?)
                    } else {
                        None
                    });
                }
                let mut we = [None, None];
                for (lane, slot) in we.iter_mut().enumerate().take(cw / 9) {
                    let i = base + lane * 9;
                    if i < width && i < e.len() {
                        *slot = Some(b.lit(e[i])?);
                    }
                }
                Some(EbrWrite {
                    clk: *clk,
                    clk_polarity: *pol,
                    addr: addr.clone(),
                    data,
                    we,
                })
            }
            None => None,
        };
        // Initial contents in nine-bit configuration words.
        let mut init = vec![0u16; 2048];
        for (&a, bits) in &ml.mem.init {
            if (a as usize) >= (1usize << plan.abits) {
                continue;
            }
            for j in 0..cw {
                let i = base + j;
                if bits.get(i).copied().flatten() == Some(true) {
                    let (word, bit) = match plan.width {
                        EbrWidth::X9 => (a as usize, j),
                        EbrWidth::X18 => (2 * a as usize + j / 9, j % 9),
                    };
                    init[word] |= 1 << bit;
                }
            }
        }
        b.nl.add(Cell::Ebr(Box::new(Ebr {
            width: plan.width,
            read_width,
            write: wr,
            read: EbrRead {
                clk: rclk,
                clk_polarity: rpol,
                addr: row_addr.clone(),
                data: rd,
            },
            init,
            mem: name.clone(),
        })));
        b.report.ebrs += 1;
    }
    b.report.memories += 1;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn init_replicates_unused_inputs() {
        // A LUT1 identity on A: 0xAAAA.
        assert_eq!(lut_init(0b10, 1), 0xAAAA);
        // AND of A, B: 0x8888.
        assert_eq!(lut_init(0b1000, 2), 0x8888);
    }

    #[test]
    fn carry_half_tables() {
        // Propagate B, generate A: 0xCCAA, as derived for A = di, B = s.
        assert_eq!(carry_init(0b1100_1100, 0xA), 0xCCAA);
        // Adder bit a ^ b with di = a: 0x66AA.
        assert_eq!(carry_init(0b0110_0110, 0xA), 0x66AA);
    }
}

/// An ECP5 flop powers up at its REGSET value, which is also the value its
/// LSR forces. A synchronous set / reset whose value differs from the
/// flop's initial value therefore cannot use LSR: fold it into D and CE
/// (`Q <= sr ? v : (ce ? d : Q)`), keeping the initial value.
fn fold_init_conflicts(lw: &mut Lowered) {
    for k in 0..lw.ffs.len() {
        let f = &lw.ffs[k];
        let v = match f.kind {
            FfKind::Fdre => false,
            FfKind::Fdse => true,
            FfKind::Fdce | FfKind::Fdpe => continue,
        };
        if f.sr == Lit::FALSE || f.init == v {
            continue;
        }
        let active = if f.sr_inv { f.sr.not() } else { f.sr };
        let (d, ce) = (f.d, f.ce);
        let val = if v { Lit::TRUE } else { Lit::FALSE };
        let d2 = lw.aig.mux(active, d, val);
        let ce2 = lw.aig.or(ce, active);
        let f = &mut lw.ffs[k];
        f.d = d2;
        f.ce = ce2;
        f.sr = Lit::FALSE;
        f.sr_inv = false;
        f.kind = FfKind::Fdre;
    }
}
