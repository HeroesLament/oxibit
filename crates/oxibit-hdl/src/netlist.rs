//! A word-level netlist: what elaboration produces and synthesis consumes.
//!
//! Shaped like yosys's RTLIL after `proc` on purpose: signals are bit
//! vectors of fixed width, cells are word-level operators, registers and
//! memories, and every cell port is a `SigSpec` (a concatenation of signal
//! slices and constant bits). That makes the netlist writable as yosys
//! JSON, so the front end can be checked equivalent to `read_verilog;
//! proc` cell for cell before synthesis exists.

use std::collections::BTreeMap;

/// A signal: a named or anonymous wire of `width` bits.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Signal {
    pub name: String,
    pub width: u32,
    pub signed: bool,
    /// A module port, and its direction.
    pub port: Option<Dir>,
    /// `initial` value of a register, LSB first; `None` bits are x.
    pub init: Option<Vec<Option<bool>>>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Dir {
    Input,
    Output,
    Inout,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SigId(pub u32);

/// One bit of a `SigSpec`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Bit {
    Sig(SigId, u32),
    Zero,
    One,
    X,
    Z,
}

/// A concatenation of bits, LSB first.
#[derive(Debug, Clone, Default, PartialEq, Eq, Hash)]
pub struct SigSpec(pub Vec<Bit>);

impl SigSpec {
    pub fn width(&self) -> u32 {
        self.0.len() as u32
    }
    pub fn constant(value: u64, width: u32) -> Self {
        SigSpec((0..width).map(|i| if i < 64 && (value >> i) & 1 == 1 { Bit::One } else { Bit::Zero }).collect())
    }
    pub fn of(id: SigId, width: u32) -> Self {
        SigSpec((0..width).map(|i| Bit::Sig(id, i)).collect())
    }
    /// Bits `lo..lo+width` of this spec.
    pub fn slice(&self, lo: u32, width: u32) -> Self {
        SigSpec(self.0[lo as usize..(lo + width) as usize].to_vec())
    }
    /// Zero- or sign-extended (or truncated) to `width`.
    pub fn extend(&self, width: u32, signed: bool) -> Self {
        let mut v = self.0.clone();
        v.truncate(width as usize);
        let fill = if signed { v.last().copied().unwrap_or(Bit::Zero) } else { Bit::Zero };
        v.resize(width as usize, fill);
        SigSpec(v)
    }
    /// `self` in the low bits, `hi` above.
    pub fn concat(&self, hi: &SigSpec) -> Self {
        let mut v = self.0.clone();
        v.extend_from_slice(&hi.0);
        SigSpec(v)
    }
}

/// Word-level operators, named as yosys's internal cells.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Op {
    // unary
    Not,
    Neg,
    ReduceAnd,
    ReduceOr,
    ReduceXor,
    LogicNot,
    // binary
    And,
    Or,
    Xor,
    Xnor,
    Add,
    Sub,
    Mul,
    Shl,
    Shr,
    Sshr,
    Eq,
    Ne,
    Lt,
    Le,
    Gt,
    Ge,
    LogicAnd,
    LogicOr,
    /// `$shiftx`: A shifted right by B (signed B shifts left), x shifted in.
    Shiftx,
}

impl Op {
    /// The yosys cell type (`$add`, `$eq`, ...).
    pub fn yosys(self) -> &'static str {
        use Op::*;
        match self {
            Not => "$not",
            Neg => "$neg",
            ReduceAnd => "$reduce_and",
            ReduceOr => "$reduce_or",
            ReduceXor => "$reduce_xor",
            LogicNot => "$logic_not",
            And => "$and",
            Or => "$or",
            Xor => "$xor",
            Xnor => "$xnor",
            Add => "$add",
            Sub => "$sub",
            Mul => "$mul",
            Shl => "$shl",
            Shr => "$shr",
            Sshr => "$sshr",
            Eq => "$eq",
            Ne => "$ne",
            Lt => "$lt",
            Le => "$le",
            Gt => "$gt",
            Ge => "$ge",
            LogicAnd => "$logic_and",
            LogicOr => "$logic_or",
            Shiftx => "$shiftx",
        }
    }
    pub fn is_unary(self) -> bool {
        use Op::*;
        matches!(self, Not | Neg | ReduceAnd | ReduceOr | ReduceXor | LogicNot)
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum Cell {
    Unary { op: Op, a: SigSpec, a_signed: bool, y: SigSpec },
    Binary { op: Op, a: SigSpec, b: SigSpec, signed: bool, y: SigSpec },
    /// `y = s ? b : a`.
    Mux { a: SigSpec, b: SigSpec, s: SigSpec, y: SigSpec },
    /// A positive- or negative-edge D flip-flop.
    Dff { clk: SigSpec, posedge: bool, d: SigSpec, q: SigSpec },
    /// A flop with an asynchronous reset to the constant `value`.
    Adff { clk: SigSpec, posedge: bool, arst: SigSpec, arst_high: bool, value: Vec<bool>, d: SigSpec, q: SigSpec },
    /// A memory: `width` x `size` words, initial contents, ports.
    Mem(Memory),
    /// A primitive or user module instantiated by name.
    Instance { ty: String, name: String, params: BTreeMap<String, Param>, conns: BTreeMap<String, SigSpec> },
}

#[derive(Debug, Clone, PartialEq)]
pub enum Param {
    Int(i64),
    Bits(Vec<Option<bool>>),
    Str(String),
}

#[derive(Debug, Clone, PartialEq)]
pub struct Memory {
    pub name: String,
    pub width: u32,
    pub size: u32,
    pub offset: i64,
    /// Word `i` of the initial contents, LSB first.
    pub init: BTreeMap<u32, Vec<Option<bool>>>,
    pub attrs: BTreeMap<String, String>,
    pub reads: Vec<MemRead>,
    pub writes: Vec<MemWrite>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct MemRead {
    /// `None`: asynchronous.
    pub clk: Option<(SigSpec, bool)>,
    pub addr: SigSpec,
    pub data: SigSpec,
}

#[derive(Debug, Clone, PartialEq)]
pub struct MemWrite {
    pub clk: (SigSpec, bool),
    pub addr: SigSpec,
    pub data: SigSpec,
    /// Per-bit enable.
    pub en: SigSpec,
}

/// A flattened module.
#[derive(Debug, Clone, Default)]
pub struct Module {
    pub name: String,
    pub signals: Vec<Signal>,
    pub cells: Vec<Cell>,
    /// Continuous connections `lhs = rhs`, after elaboration.
    pub connects: Vec<(SigSpec, SigSpec)>,
    by_name: BTreeMap<String, SigId>,
}

impl Module {
    pub fn new(name: &str) -> Self {
        Module { name: name.to_string(), ..Default::default() }
    }

    pub fn add_signal(&mut self, s: Signal) -> SigId {
        let id = SigId(self.signals.len() as u32);
        if !s.name.is_empty() {
            self.by_name.insert(s.name.clone(), id);
        }
        self.signals.push(s);
        id
    }

    /// A fresh anonymous wire of `width` bits.
    pub fn wire(&mut self, width: u32) -> SigSpec {
        let n = self.signals.len();
        let id = self.add_signal(Signal { name: format!("$w{n}"), width, signed: false, port: None, init: None });
        SigSpec::of(id, width)
    }

    pub fn lookup(&self, name: &str) -> Option<SigId> {
        self.by_name.get(name).copied()
    }

    pub fn signal(&self, id: SigId) -> &Signal {
        &self.signals[id.0 as usize]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sigspec_ops() {
        let c = SigSpec::constant(0b101, 3);
        assert_eq!(c.0, vec![Bit::One, Bit::Zero, Bit::One]);
        assert_eq!(c.extend(5, true).0, vec![Bit::One, Bit::Zero, Bit::One, Bit::One, Bit::One]);
        assert_eq!(c.extend(5, false).0[4], Bit::Zero);
        assert_eq!(c.slice(1, 2).0, vec![Bit::Zero, Bit::One]);
        assert_eq!(c.concat(&SigSpec::constant(1, 1)).width(), 4);
    }
}
