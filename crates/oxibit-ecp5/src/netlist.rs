//! The ECP5 cell netlist synthesis produces and packing consumes.
//!
//! Cells are the fabric's own primitives as Trellis configures them: a
//! LUT4, one CCU2C (two LUT4s and their carry), a slice flop, the pad
//! buffers, a DP16KD block RAM, and instantiated hard primitives (`Prim`:
//! EHXPLLL, and the BB / OBZ pads). Nets are indices; each has exactly one
//! driver.

use std::collections::BTreeMap;

pub type NetId = u32;
pub type CellId = u32;

/// Clock or set/reset polarity at the flop.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Polarity {
    /// Active high / rising edge.
    Pos,
    /// Active low / falling edge.
    Neg,
}

/// Whether set/reset acts on the clock edge or at once.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum SrMode {
    /// `LSR_OVER_CE`: synchronous, and over the clock enable.
    Sync,
    /// `ASYNC`.
    Async,
}

/// The value set/reset (and the initial value through GSR) gives.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum RegSet {
    Reset,
    Set,
}

/// A flop's set/reset input.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Lsr {
    pub net: NetId,
    pub polarity: Polarity,
    pub mode: SrMode,
}

/// The signals a slice's flops share: one clock, one set/reset, one enable.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ControlSet {
    pub clk: NetId,
    pub clk_polarity: Polarity,
    pub lsr: Option<Lsr>,
    pub ce: Option<(NetId, Polarity)>,
}

/// One half of a CCU2C: its LUT4 inputs and INIT, and its sum output.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CarryHalf {
    /// A, B, C, D.
    pub inputs: [Option<NetId>; 4],
    pub init: u16,
    pub s: Option<NetId>,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Cell {
    /// `Z = INIT[D C B A]`; unconnected inputs are don't-cares (INIT does
    /// not depend on them).
    Lut {
        inputs: [Option<NetId>; 4],
        init: u16,
        z: NetId,
    },
    /// Two carry halves, INJECT1 off; `cin` from the previous CCU2C of
    /// its chain (`None`: the first of the chain, whose first half is a
    /// feed-in that does not read it).
    Ccu2 {
        halves: [CarryHalf; 2],
        cin: Option<CellId>,
        /// Whether the next CCU2C of the chain reads this one's COUT.
        cout_used: bool,
    },
    Ff {
        d: NetId,
        q: NetId,
        ctrl: ControlSet,
        regset: RegSet,
    },
    /// Input pad: `o` is the pad's value.
    Ib { port: usize, o: NetId },
    /// Output pad driven by `i`.
    Ob { port: usize, i: NetId },
    /// One DP16KD: port A writes, port B reads (synchronous, read-first).
    Ebr(Box<Ebr>),
    /// An instantiated hard primitive, kept as written.
    Prim(Box<Prim>),
}

/// A primitive pin: an input reads a net, an output drives one. `None`:
/// not connected.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PrimPin {
    In(Option<NetId>),
    Out(Option<NetId>),
}

/// An instantiated primitive (`EHXPLLL`; `BB` and `OBZ` on a pad).
#[derive(Clone, Debug, PartialEq)]
pub struct Prim {
    /// Primitive type, as instantiated.
    pub ty: String,
    pub name: String,
    pub params: BTreeMap<String, oxibit_hdl::netlist::Param>,
    pub attrs: BTreeMap<String, String>,
    /// One-bit pins in the order the instance connects them.
    pub pins: Vec<(String, PrimPin)>,
    /// A pad buffer's port (an index into `Netlist::ports`).
    pub pad: Option<usize>,
    /// The instance's index in the lowered design, for checking.
    pub inst: usize,
}

impl Prim {
    pub fn pin(&self, name: &str) -> Option<PrimPin> {
        self.pins.iter().find(|(n, _)| n == name).map(|&(_, p)| p)
    }
    pub fn param_str(&self, name: &str) -> Option<String> {
        use oxibit_hdl::netlist::Param;
        Some(match self.params.get(name)? {
            Param::Str(s) => s.clone(),
            Param::Int(i) => i.to_string(),
            Param::Bits(b) => {
                let v = b.iter().rev().fold(0i64, |a, x| a * 2 + i64::from(*x == Some(true)));
                v.to_string()
            }
        })
    }
    pub fn param_int(&self, name: &str) -> Option<i64> {
        use oxibit_hdl::netlist::Param;
        match self.params.get(name)? {
            Param::Int(i) => Some(*i),
            Param::Str(s) => s.trim().parse().ok(),
            Param::Bits(b) => Some(b.iter().rev().fold(0i64, |a, x| a * 2 + i64::from(*x == Some(true)))),
        }
    }
}

/// A DP16KD port's data width.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum EbrWidth {
    /// 2048 x 9: word address on `AD<x>3..13`.
    X9,
    /// 1024 x 18: word address on `AD<x>4..13`, byte (9-bit lane) enables
    /// on `AD<x>0..1`.
    X18,
}

impl EbrWidth {
    pub fn bits(self) -> usize {
        match self {
            EbrWidth::X9 => 9,
            EbrWidth::X18 => 18,
        }
    }
    /// The `AD<x>` pin of word address bit 0.
    pub fn addr_lsb(self) -> usize {
        match self {
            EbrWidth::X9 => 3,
            EbrWidth::X18 => 4,
        }
    }
    pub fn depth(self) -> usize {
        match self {
            EbrWidth::X9 => 2048,
            EbrWidth::X18 => 1024,
        }
    }
}

/// The write port (A).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EbrWrite {
    pub clk: NetId,
    pub clk_polarity: Polarity,
    /// Word address, LSB first.
    pub addr: Vec<NetId>,
    /// `DIA<j>`; `None`: unused bit.
    pub data: Vec<Option<NetId>>,
    /// Write enable of each 9-bit lane (X9: lane 0 only).
    pub we: [Option<NetId>; 2],
}

/// The read port (B).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EbrRead {
    pub clk: NetId,
    pub clk_polarity: Polarity,
    /// Word address, LSB first.
    pub addr: Vec<NetId>,
    /// `DOB<j>`; `None`: not read.
    pub data: Vec<Option<NetId>>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Ebr {
    /// Port A's width, and the width the contents are laid out in.
    pub width: EbrWidth,
    /// Port B's width. X18 under an X9 `width`: port B reads row
    /// `addr` (words `2 addr` on `DOB0..8` and `2 addr + 1` on
    /// `DOB9..17`) and the fabric picks the word.
    pub read_width: EbrWidth,
    pub write: Option<EbrWrite>,
    pub read: EbrRead,
    /// Initial contents as the configuration holds them: 2048 nine-bit
    /// words (X18 word `a` is words `2a` (bits 0..8) and `2a + 1`).
    pub init: Vec<u16>,
    /// The memory it implements, for names and reports.
    pub mem: String,
}

/// A top-level port bit: its name (`name` or `name[i]`) and direction.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Port {
    pub name: String,
    pub output: bool,
    /// An inout pad (through a BB); `output` is false.
    pub bidir: bool,
}

#[derive(Clone, Debug, Default)]
pub struct Netlist {
    pub top: String,
    pub cells: Vec<Cell>,
    pub ports: Vec<Port>,
    /// Net names, where synthesis knows one.
    pub net_names: BTreeMap<NetId, String>,
    pub nets: u32,
    /// The pad buffer attributes of each port (PULLMODE from IBPU, BBPD, ...),
    /// under the LPF settings.
    pub pad_attrs: BTreeMap<usize, BTreeMap<String, String>>,
}

impl Netlist {
    pub fn new_net(&mut self) -> NetId {
        let n = self.nets;
        self.nets += 1;
        n
    }

    pub fn add(&mut self, c: Cell) -> CellId {
        let id = CellId::try_from(self.cells.len()).expect("cell count fits u32");
        self.cells.push(c);
        id
    }

    /// The cell driving each net.
    pub fn drivers(&self) -> Vec<Option<CellId>> {
        let mut d = vec![None; self.nets as usize];
        for (i, c) in self.cells.iter().enumerate() {
            let id = CellId::try_from(i).expect("fits");
            for n in outputs(c) {
                d[n as usize] = Some(id);
            }
        }
        d
    }

    /// Cells by kind, for reports.
    pub fn counts(&self) -> BTreeMap<&'static str, usize> {
        let mut m = BTreeMap::new();
        for c in &self.cells {
            let k = match c {
                Cell::Lut { .. } => "LUT4",
                Cell::Ccu2 { .. } => "CCU2C",
                Cell::Ff { .. } => "FF",
                Cell::Ib { .. } => "IB",
                Cell::Ob { .. } => "OB",
                Cell::Ebr(_) => "DP16KD",
                Cell::Prim(p) => match p.ty.as_str() {
                    "EHXPLLL" => "EHXPLLL",
                    "BB" => "BB",
                    "OBZ" => "OBZ",
                    "JTAGG" => "JTAGG",
                    t if crate::sys::is_sys(t) => "config",
                    t if crate::iol::is_iol(t) => "IOLOGIC",
                    t if crate::dsp::is_dsp(t) => "DSP",
                    t if crate::ebr::is_bram(t) => "EBR",
                    t if crate::dqs::is_dqs(t) => "DDR",
                    "TRELLIS_DPR16X4S" => "LUTRAM",
                    _ => "primitive",
                },
            };
            *m.entry(k).or_default() += 1;
        }
        m
    }
}

/// The nets a cell drives.
pub fn outputs(c: &Cell) -> Vec<NetId> {
    match c {
        Cell::Lut { z, .. } => vec![*z],
        Cell::Ccu2 { halves, .. } => halves.iter().filter_map(|h| h.s).collect(),
        Cell::Ff { q, .. } => vec![*q],
        Cell::Ib { o, .. } => vec![*o],
        Cell::Ob { .. } => vec![],
        Cell::Ebr(e) => e.read.data.iter().flatten().copied().collect(),
        Cell::Prim(p) => p
            .pins
            .iter()
            .filter_map(|(_, pin)| match pin {
                PrimPin::Out(n) => *n,
                PrimPin::In(_) => None,
            })
            .collect(),
    }
}

/// The nets a cell reads.
pub fn inputs(c: &Cell) -> Vec<NetId> {
    match c {
        Cell::Lut { inputs, .. } => inputs.iter().flatten().copied().collect(),
        Cell::Ccu2 { halves, .. } => halves
            .iter()
            .flat_map(|h| h.inputs.iter().flatten().copied())
            .collect(),
        Cell::Ff { d, ctrl, .. } => {
            let mut v = vec![*d, ctrl.clk];
            v.extend(ctrl.lsr.map(|l| l.net));
            v.extend(ctrl.ce.map(|c| c.0));
            v
        }
        Cell::Ib { .. } => vec![],
        Cell::Ob { i, .. } => vec![*i],
        Cell::Ebr(e) => {
            let mut v = vec![e.read.clk];
            v.extend(&e.read.addr);
            if let Some(w) = &e.write {
                v.push(w.clk);
                v.extend(&w.addr);
                v.extend(w.data.iter().flatten());
                v.extend(w.we.iter().flatten());
            }
            v
        }
        Cell::Prim(p) => p
            .pins
            .iter()
            .filter_map(|(_, pin)| match pin {
                PrimPin::In(n) => *n,
                PrimPin::Out(_) => None,
            })
            .collect(),
    }
}

/// The clock inputs of a primitive: they route on the global network.
pub const PRIM_CLOCK_INPUTS: &[(&str, &str)] = &[
    ("EHXPLLL", "CLKI"),
    ("EHXPLLL", "CLKFB"),
    ("TRELLIS_DPR16X4S", "WCK"),
];

/// Whether a primitive pin is a clock input.
pub fn is_clock_input(ty: &str, pin: &str) -> bool {
    PRIM_CLOCK_INPUTS
        .iter()
        .chain(crate::iol::CLOCK_INPUTS)
        .chain(crate::clk::CLOCK_INPUTS)
        .any(|&(t, q)| t == ty && q == pin)
        || crate::dsp::is_clock_input(ty, pin)
        || crate::ebr::is_clock_input(ty, pin)
        || crate::dqs::is_clock_input(ty, pin)
}

/// The clock nets: flop clocks, block RAM port clocks, PLL references.
pub fn clock_nets(nl: &Netlist) -> std::collections::BTreeSet<NetId> {
    let mut s = std::collections::BTreeSet::new();
    for c in &nl.cells {
        match c {
            Cell::Prim(p) => {
                for (pin, conn) in &p.pins {
                    if let PrimPin::In(Some(n)) = *conn {
                        if is_clock_input(&p.ty, pin) {
                            s.insert(n);
                        }
                    }
                }
            }
            Cell::Ff { ctrl, .. } => {
                s.insert(ctrl.clk);
            }
            Cell::Ebr(e) => {
                s.insert(e.read.clk);
                if let Some(w) = &e.write {
                    s.insert(w.clk);
                }
            }
            _ => {}
        }
    }
    s
}
