//! Nexus cell netlist, built from oxibit-synth's lowered AIG and its 4-input
//! LUT cover.
//!
//! Cells are what the fabric has: `Lut` (OXIDE_COMB in LOGIC mode), `Ff`
//! (OXIDE_FF), `Ibuf` / `Obuf` (SEIO33_CORE), `Osc` (OSC_CORE) and `Dcc`
//! (global clock buffer). Complemented signals cost nothing where the
//! fabric can absorb them: inside a LUT's truth table, or on an FF's
//! invertible CLK / CE / LSR mux. Elsewhere a second LUT computes the
//! complement.
//!
//! Every net driving an FF clock goes through a DCC onto the global
//! network, as nextpnr's packer does. Unused OSC_CORE trim inputs are tied
//! to a logic 0 net: left floating they read as ones and change the trim.

use std::collections::HashMap;

use oxibit_hdl::netlist::{Dir, Param};
use oxibit_synth::aig::Lit;
use oxibit_synth::lower::{FfKind, Lowered};
use oxibit_synth::map::Mapping;

use crate::{Result, err};

pub type NetId = usize;
pub type CellId = usize;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CellKind {
    /// 4-input LUT; INIT bit i is the output for A + 2B + 4C + 8D == i.
    Lut { init: u16 },
    Ff(FfCfg),
    Ibuf { port: String },
    Obuf { port: String },
    Osc { hf_div: u32 },
    Dcc,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct FfCfg {
    /// REGSET: the value LSR forces, and the power-up value.
    pub set: bool,
    /// SRMODE ASYNC (else LSR_OVER_CE).
    pub async_sr: bool,
    pub clk_inv: bool,
    pub ce_inv: bool,
    pub lsr_inv: bool,
}

#[derive(Clone, Debug)]
pub struct Cell {
    pub name: String,
    pub kind: CellKind,
    /// Input pin -> net. Pins absent here are unconnected.
    pub ins: Vec<(&'static str, NetId)>,
    pub out: Option<(&'static str, NetId)>,
}

impl Cell {
    pub fn input(&self, pin: &str) -> Option<NetId> {
        self.ins.iter().find(|(p, _)| *p == pin).map(|(_, n)| *n)
    }
}

#[derive(Clone, Debug, Default)]
pub struct Net {
    pub name: String,
    pub driver: Option<(CellId, &'static str)>,
    pub sinks: Vec<(CellId, &'static str)>,
    /// Drives FF clocks (through the global network after DCC insertion).
    pub clock: bool,
}

#[derive(Default, Debug)]
pub struct Netlist {
    pub cells: Vec<Cell>,
    pub nets: Vec<Net>,
}

/// OSC_CORE inputs tied to 0 when the design leaves them unconnected.
pub const OSC_TIE_LOW: [&str; 19] = [
    "HFTRMFAB0", "HFTRMFAB1", "HFTRMFAB2", "HFTRMFAB3", "HFTRMFAB4", "HFTRMFAB5", "HFTRMFAB6", "HFTRMFAB7",
    "HFTRMFAB8", "LFTRMFAB0", "LFTRMFAB1", "LFTRMFAB2", "LFTRMFAB3", "LFTRMFAB4", "LFTRMFAB5", "LFTRMFAB6",
    "LFTRMFAB7", "LFTRMFAB8", "HFSDSCEN",
];

/// Port directions of the Nexus primitives designs may instantiate, for
/// `oxibit_synth::lower` (via `lib_ports::scan`).
pub const PRIMITIVES_V: &str = "
module OSC_CORE(input HFOUTEN, input HFSDSCEN, input [8:0] HFTRMFAB, input [8:0] LFTRMFAB,
                output HFCLKOUT, output LFCLKOUT, output HFCLKCFG, output HFSDCOUT);
endmodule
";

/// A LUT input or an FF/IO pin's source after constant and complement folding.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Src {
    Const(bool),
    Net(NetId, bool),
}

struct Builder<'a> {
    mp: &'a Mapping,
    nl: Netlist,
    /// Net driven by each AIG input node (port, flop, primitive output).
    input_net: HashMap<u32, NetId>,
    /// Net of each LUT root (true function).
    root_net: HashMap<u32, NetId>,
    /// Nets of complemented signals, built on demand.
    compl_net: HashMap<u32, NetId>,
    const_net: [Option<NetId>; 2],
}

fn expand_tt(tt: u64, k: usize) -> u16 {
    let mask = (1usize << k) - 1;
    let mut init = 0u16;
    for x in 0..16usize {
        if (tt >> (x & mask)) & 1 == 1 {
            init |= 1 << x;
        }
    }
    init
}

impl<'a> Builder<'a> {
    fn net(&mut self, name: String) -> NetId {
        self.nl.nets.push(Net { name, ..Default::default() });
        self.nl.nets.len() - 1
    }

    fn cell(&mut self, name: String, kind: CellKind, ins: Vec<(&'static str, NetId)>, out: Option<(&'static str, NetId)>) -> CellId {
        let id = self.nl.cells.len();
        for (p, n) in &ins {
            self.nl.nets[*n].sinks.push((id, p));
        }
        if let Some((p, n)) = out {
            self.nl.nets[n].driver = Some((id, p));
        }
        self.nl.cells.push(Cell { name, kind, ins, out });
        id
    }

    fn src(&self, l: Lit) -> Result<Src> {
        if l.is_const() {
            return Ok(Src::Const(l == Lit::TRUE));
        }
        let n = l.node();
        let net = self
            .root_net
            .get(&n)
            .or_else(|| self.input_net.get(&n))
            .copied()
            .ok_or_else(|| err!("AIG node {n} is neither a LUT root nor a driven input"))?;
        Ok(Src::Net(net, l.compl()))
    }

    /// A LUT driving a constant.
    fn const_net(&mut self, v: bool) -> NetId {
        if let Some(n) = self.const_net[v as usize] {
            return n;
        }
        let n = self.net(format!("$const{}", v as u8));
        self.cell(format!("$const{}_lut", v as u8), CellKind::Lut { init: if v { 0xFFFF } else { 0 } }, vec![], Some(("F", n)));
        self.const_net[v as usize] = Some(n);
        n
    }

    /// A net carrying `l` exactly (complement realised by a LUT).
    fn plain(&mut self, l: Lit) -> Result<NetId> {
        Ok(match self.src(l)? {
            Src::Const(v) => self.const_net(v),
            Src::Net(n, false) => n,
            Src::Net(n, true) => {
                let node = l.node();
                if let Some(&c) = self.compl_net.get(&node) {
                    return Ok(c);
                }
                let c = self.net(format!("{}$n", self.nl.nets[n].name));
                if let Some(&li) = self.mp.lut_of.get(node as usize).and_then(|x| x.as_ref()) {
                    // Duplicate the LUT with the complemented function.
                    let lut = &self.mp.luts[li as usize];
                    let init = !expand_tt(lut.tt, lut.leaves.len());
                    let leaves = lut.leaves.clone();
                    let ins = self.lut_inputs(&leaves, init)?;
                    self.cell(format!("lut{node}$n"), CellKind::Lut { init: ins.1 }, ins.0, Some(("F", c)));
                } else {
                    self.cell(format!("inv{node}"), CellKind::Lut { init: 0x5555 }, vec![("A", n)], Some(("F", c)));
                }
                self.compl_net.insert(node, c);
                c
            }
        })
    }

    /// Pins for a LUT over `leaves` with expanded `init`, folding constant
    /// leaves and complemented leaf sources into the table.
    fn lut_inputs(&mut self, leaves: &[u32], mut init: u16) -> Result<(Vec<(&'static str, NetId)>, u16)> {
        const PINS: [&str; 4] = ["A", "B", "C", "D"];
        let mut ins = Vec::new();
        for (i, &leaf) in leaves.iter().enumerate() {
            match self.src(Lit(leaf << 1))? {
                Src::Const(v) => {
                    // Fix input i to v: copy the half where it is v.
                    let mut o = 0u16;
                    for x in 0..16 {
                        let y = if v { x | (1 << i) } else { x & !(1 << i) };
                        if (init >> y) & 1 == 1 {
                            o |= 1 << x;
                        }
                    }
                    init = o;
                }
                Src::Net(n, c) => {
                    if c {
                        let mut o = 0u16;
                        for x in 0..16 {
                            if (init >> (x ^ (1 << i))) & 1 == 1 {
                                o |= 1 << x;
                            }
                        }
                        init = o;
                    }
                    ins.push((PINS[i], n));
                }
            }
        }
        Ok((ins, init))
    }
}

/// Build the Nexus netlist. `lw` and `mp` come from `oxibit_synth::lower`
/// and `oxibit_synth::map::map_k(.., 4, ..)`.
pub fn build(lw: &Lowered, mp: &Mapping) -> Result<Netlist> {
    let mut b = Builder {
        mp,
        nl: Netlist::default(),
        input_net: HashMap::new(),
        root_net: HashMap::new(),
        compl_net: HashMap::new(),
        const_net: [None, None],
    };
    if mp.luts.iter().any(|l| l.leaves.len() > 4) {
        return Err(err!("LUT cover has cuts wider than 4"));
    }

    // Drivers of AIG inputs: input ports, flops, primitive outputs.
    let mut ibufs = Vec::new();
    for (name, dir, _, bits) in &lw.ports {
        match dir {
            Dir::Input => {
                for (i, l) in bits.iter().enumerate() {
                    let pname = if bits.len() == 1 { name.clone() } else { format!("{name}[{i}]") };
                    if l.is_const() || l.compl() {
                        return Err(err!("input port bit {pname} is not a plain AIG input"));
                    }
                    let n = b.net(pname.clone());
                    b.input_net.insert(l.node(), n);
                    ibufs.push((pname, n));
                }
            }
            Dir::Output => {}
            Dir::Inout => return Err(err!("inout port {name}: not supported on Nexus yet")),
        }
    }
    for (pname, n) in ibufs {
        b.cell(format!("{pname}$ib"), CellKind::Ibuf { port: pname }, vec![], Some(("O", n)));
    }
    let mut ff_q = Vec::new();
    for f in &lw.ffs {
        let n = b.net(f.name.clone());
        b.input_net.insert(f.q.node(), n);
        ff_q.push(n);
    }
    let mut oscs = Vec::new();
    for inst in &lw.insts {
        match inst.ty.as_str() {
            "OSC_CORE" => {
                let hf_div = match inst.params.get("HF_CLK_DIV") {
                    None => 128,
                    Some(Param::Int(v)) => *v as u32,
                    Some(Param::Str(s)) => s.trim().parse().map_err(|_| err!("{}: HF_CLK_DIV {s:?}", inst.name))?,
                    Some(Param::Bits(bits)) => bits.iter().enumerate().map(|(i, x)| (x.unwrap_or(false) as u32) << i).sum(),
                };
                if hf_div > 255 {
                    return Err(err!("{}: HF_CLK_DIV {hf_div} > 255", inst.name));
                }
                let mut clk = None;
                for (port, bits) in &inst.conns {
                    match port.as_str() {
                        "HFCLKOUT" => {
                            let l = bits.first().ok_or_else(|| err!("{}: HFCLKOUT unconnected", inst.name))?;
                            let n = b.net(format!("{}.HFCLKOUT", inst.name));
                            b.input_net.insert(l.node(), n);
                            clk = Some(n);
                        }
                        "HFOUTEN" => {
                            if bits.first() != Some(&Lit::TRUE) {
                                return Err(err!("{}: HFOUTEN must be tied to 1", inst.name));
                            }
                        }
                        p => return Err(err!("{}: port {p} not supported yet", inst.name)),
                    }
                }
                oscs.push((inst.name.clone(), hf_div, clk));
            }
            t => return Err(err!("primitive {t} ({}) not supported on Nexus yet", inst.name)),
        }
    }
    if !lw.mems.is_empty() || !lw.aig.chains.is_empty() {
        return Err(err!("memories and carry chains are not supported on Nexus yet"));
    }
    for (name, hf_div, clk) in oscs {
        let id = b.cell(name, CellKind::Osc { hf_div }, vec![], None);
        if let Some(n) = clk {
            b.nl.cells[id].out = Some(("HFCLKOUT", n));
            b.nl.nets[n].driver = Some((id, "HFCLKOUT"));
        }
        let gnd = b.const_net(false);
        for p in OSC_TIE_LOW {
            b.nl.cells[id].ins.push((p, gnd));
            b.nl.nets[gnd].sinks.push((id, p));
        }
    }

    // LUTs of the cover.
    for l in &mp.luts {
        let n = b.net(format!("n{}", l.root));
        b.root_net.insert(l.root, n);
    }
    for l in &mp.luts {
        let init = expand_tt(l.tt, l.leaves.len());
        let (ins, init) = b.lut_inputs(&l.leaves, init)?;
        let n = b.root_net[&l.root];
        b.cell(format!("lut{}", l.root), CellKind::Lut { init }, ins, Some(("F", n)));
    }

    // Flops.
    for (f, &q) in lw.ffs.iter().zip(&ff_q) {
        let clk = match b.src(f.clk)? {
            Src::Net(n, c) => (n, c ^ f.clk_inv),
            Src::Const(_) => return Err(err!("flop {}: constant clock", f.name)),
        };
        let mut ins = vec![("CLK", clk.0)];
        let d = b.plain(f.d)?;
        ins.push(("M", d));
        let (ce_inv, ce) = match b.src(f.ce)? {
            Src::Const(true) => (false, None),
            Src::Const(false) => (true, None),
            Src::Net(n, c) => (c, Some(n)),
        };
        if let Some(n) = ce {
            ins.push(("CE", n));
        }
        let async_sr = matches!(f.kind, FfKind::Fdce | FfKind::Fdpe);
        let (lsr_inv, lsr) = match b.src(f.sr)? {
            Src::Const(false) => (true, None),
            Src::Const(true) => return Err(err!("flop {}: reset tied active", f.name)),
            Src::Net(n, c) => (c ^ (async_sr && f.sr_inv), Some(n)),
        };
        if let Some(n) = lsr {
            ins.push(("LSR", n));
        }
        let set = match (f.kind, lsr.is_some()) {
            (FfKind::Fdse | FfKind::Fdpe, true) => true,
            (FfKind::Fdre | FfKind::Fdce, true) => false,
            (_, false) => f.init,
        };
        if set != f.init {
            return Err(err!(
                "flop {}: initial value {} differs from its reset value; Nexus flops power up to their reset value",
                f.name,
                f.init as u8
            ));
        }
        let cfg = FfCfg { set, async_sr, clk_inv: clk.1, ce_inv, lsr_inv };
        b.cell(f.name.clone(), CellKind::Ff(cfg), ins, Some(("Q", q)));
    }

    // Output ports.
    let outs: Vec<(String, Lit)> = lw
        .ports
        .iter()
        .filter(|(_, d, _, _)| *d == Dir::Output)
        .flat_map(|(name, _, _, bits)| {
            let w = bits.len();
            bits.iter().enumerate().map(move |(i, l)| (if w == 1 { name.clone() } else { format!("{name}[{i}]") }, *l))
        })
        .collect();
    for (pname, l) in outs {
        let n = b.plain(l)?;
        b.cell(format!("{pname}$ob"), CellKind::Obuf { port: pname }, vec![("I", n)], None);
    }

    // Clock nets onto the global network through a DCC each.
    let clock_nets: Vec<NetId> = {
        let mut v: Vec<NetId> = b
            .nl
            .cells
            .iter()
            .filter(|c| matches!(c.kind, CellKind::Ff(_)))
            .filter_map(|c| c.input("CLK"))
            .collect();
        v.sort();
        v.dedup();
        v
    };
    for n in clock_nets {
        let g = b.net(format!("{}$glb", b.nl.nets[n].name));
        b.nl.nets[g].clock = true;
        // Move the FF clock sinks to the DCC output.
        let sinks = std::mem::take(&mut b.nl.nets[n].sinks);
        let (moved, kept): (Vec<_>, Vec<_>) = sinks.into_iter().partition(|(c, p)| *p == "CLK" && matches!(b.nl.cells[*c].kind, CellKind::Ff(_)));
        b.nl.nets[n].sinks = kept;
        for (c, p) in &moved {
            for (pin, net) in b.nl.cells[*c].ins.iter_mut() {
                if pin == p {
                    *net = g;
                }
            }
        }
        b.nl.nets[g].sinks = moved;
        let name = format!("{}$dcc", b.nl.nets[n].name);
        b.cell(name, CellKind::Dcc, vec![("CLKI", n)], Some(("CLKO", g)));
    }

    // Drop nets nothing reads (and LUTs that only fed them).
    loop {
        let dead: Vec<CellId> = b
            .nl
            .cells
            .iter()
            .enumerate()
            .filter(|(_, c)| matches!(c.kind, CellKind::Lut { .. }))
            .filter(|(_, c)| c.out.is_some_and(|(_, n)| b.nl.nets[n].sinks.is_empty()))
            .map(|(i, _)| i)
            .collect();
        if dead.is_empty() {
            break;
        }
        for i in dead.into_iter().rev() {
            remove_cell(&mut b.nl, i);
        }
    }
    Ok(b.nl)
}

/// Remove a cell, keeping ids dense.
fn remove_cell(nl: &mut Netlist, id: CellId) {
    let c = nl.cells.remove(id);
    for (p, n) in &c.ins {
        nl.nets[*n].sinks.retain(|(cc, pp)| !(*cc == id && pp == p));
    }
    if let Some((_, n)) = c.out {
        nl.nets[n].driver = None;
    }
    let fix = |x: &mut CellId| {
        if *x > id {
            *x -= 1;
        }
    };
    for net in nl.nets.iter_mut() {
        if let Some((d, _)) = net.driver.as_mut() {
            fix(d);
        }
        for (s, _) in net.sinks.iter_mut() {
            fix(s);
        }
    }
}

impl Netlist {
    pub fn count(&self, f: impl Fn(&CellKind) -> bool) -> usize {
        self.cells.iter().filter(|c| f(&c.kind)).count()
    }
}

#[cfg(test)]
mod tests {
    use super::expand_tt;

    #[test]
    fn narrow_tables_repeat_over_unused_inputs() {
        assert_eq!(expand_tt(0b10, 1), 0xAAAA); // F = A
        assert_eq!(expand_tt(0b1000, 2), 0x8888); // F = A & B
        assert_eq!(expand_tt(0xFFFF, 4), 0xFFFF);
    }
}
