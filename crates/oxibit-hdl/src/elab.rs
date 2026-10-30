//! Elaboration: a parsed module to a word-level netlist.
//!
//! Expression sizing follows IEEE 1364-2005 5.4 and 5.5: each expression
//! has a self-determined width and signedness, computed bottom up; a
//! context-determined operand (arithmetic, bitwise, `?:` branches) is
//! evaluated at the width of its context, and is signed only when every
//! operand of that context is. Comparison operands are sized to each
//! other; shift amounts, concatenation items, conditions and reduction
//! operands are self-determined. Extension happens at the leaves.
//!
//! Processes are executed symbolically: an `always @(*)` block becomes
//! the multiplexers its `if`/`case` structure implies; an edge-triggered
//! block becomes the same multiplexers in front of a `$dff` per register
//! it assigns, holding its value on paths that do not assign it.

use std::collections::{BTreeMap, HashMap};

use crate::ast::{CaseKind, Item, ModuleAst, Sens, Stmt};
use crate::expr::Expr;
use crate::netlist::{Bit, Cell, Dir, MemRead, MemWrite, Memory, Module, Op, Param, SigId, SigSpec, Signal};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ElabError(pub String);

impl std::fmt::Display for ElabError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

type R<T> = Result<T, ElabError>;

fn err<T>(m: impl Into<String>) -> R<T> {
    Err(ElabError(m.into()))
}

/// A declared net or variable.
#[derive(Debug, Clone)]
struct Net {
    id: SigId,
    width: u32,
    signed: bool,
    /// Declared `[msb:lsb]`.
    msb: i64,
    lsb: i64,
}

impl Net {
    /// Position (0 = LSB) of declared index `k`, if in range.
    fn pos(&self, k: i64) -> Option<u32> {
        let p = if self.msb >= self.lsb { k - self.lsb } else { self.lsb - k };
        (p >= 0 && (p as u32) < self.width).then_some(p as u32)
    }
}

#[derive(Debug, Clone)]
struct Mem {
    index: usize,
    width: u32,
    lo: i64,
    hi: i64,
}

/// A constant value: bits LSB first, and whether it is signed.
#[derive(Debug, Clone, PartialEq)]
pub struct Const {
    pub bits: Vec<Option<bool>>,
    pub signed: bool,
}

impl Const {
    pub fn to_i64(&self) -> Option<i64> {
        let mut v: i64 = 0;
        for (i, b) in self.bits.iter().enumerate().take(64) {
            if (*b)? {
                v |= 1 << i;
            }
        }
        if self.signed && self.bits.len() < 64 && self.bits.last().copied().flatten() == Some(true) {
            v |= -1i64 << self.bits.len();
        }
        Some(v)
    }
}

pub struct Elab {
    pub m: Module,
    params: HashMap<String, Const>,
    nets: HashMap<String, Net>,
    mems: HashMap<String, Mem>,
}

/// The symbolic state of a process: each assigned variable's current
/// value, one bit at a time (`None`: not assigned on this path).
type Env = BTreeMap<SigId, Vec<Option<Bit>>>;

impl Elab {
    pub fn module(ast: &ModuleAst) -> R<Module> {
        let mut e = Elab { m: Module::new(&ast.name), params: HashMap::new(), nets: HashMap::new(), mems: HashMap::new() };
        // Parameters, in order (later ones may use earlier ones).
        for it in &ast.items {
            if let Item::Param { name, value, .. } = it {
                let c = e.const_eval(value)?;
                e.params.insert(name.clone(), c);
            }
        }
        // Declarations.
        for it in &ast.items {
            if let Item::Decl { name, dir, signed, range, array, attrs, .. } = it {
                let (msb, lsb) = match range {
                    Some(r) => (e.const_int(&r.msb)?, e.const_int(&r.lsb)?),
                    None => (0, 0),
                };
                let width = (msb - lsb).unsigned_abs() as u32 + 1;
                if let Some(a) = array {
                    let (x, y) = (e.const_int(&a.msb)?, e.const_int(&a.lsb)?);
                    let (lo, hi) = (x.min(y), x.max(y));
                    let index = e.m.cells.len();
                    let mut mattrs = BTreeMap::new();
                    for (k, v) in attrs {
                        let val = match v {
                            Some(Expr::Ident(s)) => s.clone(),
                            Some(other) => format!("{other:?}"),
                            None => String::new(),
                        };
                        mattrs.insert(k.clone(), val);
                    }
                    e.m.cells.push(Cell::Mem(Memory {
                        name: name.clone(),
                        width,
                        size: (hi - lo + 1) as u32,
                        offset: lo,
                        init: BTreeMap::new(),
                        attrs: mattrs,
                        reads: vec![],
                        writes: vec![],
                    }));
                    e.mems.insert(name.clone(), Mem { index, width, lo, hi });
                    continue;
                }
                match e.nets.get(name) {
                    // `output x;` then `reg [3:0] x;`: the later, fuller
                    // declaration sets the range.
                    Some(n) => {
                        let id = n.id;
                        let s = &mut e.m.signals[id.0 as usize];
                        if range.is_some() {
                            s.width = width;
                        }
                        s.signed |= *signed;
                        if dir.is_some() {
                            s.port = *dir;
                        }
                        let n = e.nets.get_mut(name).unwrap();
                        if range.is_some() {
                            n.width = width;
                            n.msb = msb;
                            n.lsb = lsb;
                        }
                        n.signed |= *signed;
                    }
                    None => {
                        let id = e.m.add_signal(Signal { name: name.clone(), width, signed: *signed, port: *dir, init: None });
                        e.nets.insert(name.clone(), Net { id, width, signed: *signed, msb, lsb });
                    }
                }
            }
        }
        // Declaration initialisers: a register's initial value, or a net's
        // continuous assignment.
        for it in &ast.items {
            if let Item::Decl { name, is_reg, init: Some(v), array: None, .. } = it {
                if *is_reg {
                    e.set_init(name, v)?;
                } else {
                    e.assign(&Expr::Ident(name.clone()), v)?;
                }
            }
        }
        for it in &ast.items {
            match it {
                Item::Assign { lhs, rhs } => e.assign(lhs, rhs)?,
                Item::Always { sens, body } => e.process(sens, body)?,
                Item::Initial(body) => e.initial(body)?,
                Item::Instance { ty, name, params, conns } => e.instance(ty, name, params, conns)?,
                _ => {}
            }
        }
        Ok(e.m)
    }

    // ---- constants -------------------------------------------------------

    fn const_int(&self, e: &Expr) -> R<i64> {
        self.const_eval(e)?.to_i64().ok_or_else(|| ElabError(format!("not a defined integer: {e:?}")))
    }

    /// Constant folding for parameters, ranges, indices.
    pub fn const_eval(&self, e: &Expr) -> R<Const> {
        let int = |v: i64| Const { bits: (0..32).map(|i| Some((v >> i) & 1 == 1)).collect(), signed: true };
        Ok(match e {
            Expr::Number { width, signed, bits } => {
                let w = width.unwrap_or(32) as usize;
                let mut b = bits.clone();
                b.resize(w.max(b.len()), Some(false));
                b.truncate(w);
                Const { bits: b, signed: *signed || width.is_none() && *signed }
            }
            Expr::Ident(n) => match self.params.get(n) {
                Some(c) => c.clone(),
                None => return err(format!("{n} is not a constant")),
            },
            Expr::Unary(op, a) => {
                let v = self.const_int(a)?;
                int(match *op {
                    "-" => -v,
                    "+" => v,
                    "~" => !v,
                    "!" => i64::from(v == 0),
                    o => return err(format!("constant unary {o}")),
                })
            }
            Expr::Binary(op, a, b) => {
                let (x, y) = (self.const_int(a)?, self.const_int(b)?);
                int(match *op {
                    "+" => x + y,
                    "-" => x - y,
                    "*" => x * y,
                    "/" if y != 0 => x / y,
                    "%" if y != 0 => x % y,
                    "**" => x.pow(y as u32),
                    "<<" | "<<<" => x << y,
                    ">>" | ">>>" => x >> y,
                    "&" => x & y,
                    "|" => x | y,
                    "^" => x ^ y,
                    "==" => i64::from(x == y),
                    "!=" => i64::from(x != y),
                    "<" => i64::from(x < y),
                    "<=" => i64::from(x <= y),
                    ">" => i64::from(x > y),
                    ">=" => i64::from(x >= y),
                    "&&" => i64::from(x != 0 && y != 0),
                    "||" => i64::from(x != 0 || y != 0),
                    o => return err(format!("constant binary {o}")),
                })
            }
            Expr::Ternary(c, t, f) => {
                if self.const_int(c)? != 0 {
                    self.const_eval(t)?
                } else {
                    self.const_eval(f)?
                }
            }
            Expr::Call(n, args) if n == "$clog2" && args.len() == 1 => {
                let v = self.const_int(&args[0])?;
                int(if v <= 1 { 0 } else { 64 - ((v - 1) as u64).leading_zeros() as i64 })
            }
            other => return err(format!("not a constant expression: {other:?}")),
        })
    }

    // ---- expression typing -------------------------------------------------

    /// Self-determined width and signedness.
    fn self_type(&self, e: &Expr) -> R<(u32, bool)> {
        Ok(match e {
            Expr::Number { width, signed, .. } => (width.unwrap_or(32), *signed),
            Expr::Ident(n) => {
                if let Some(net) = self.nets.get(n) {
                    (net.width, net.signed)
                } else if let Some(c) = self.params.get(n) {
                    (c.bits.len() as u32, c.signed)
                } else {
                    return err(format!("undeclared {n}"));
                }
            }
            Expr::Unary(op, a) => match *op {
                "+" | "-" | "~" => self.self_type(a)?,
                _ => (1, false),
            },
            Expr::Binary(op, a, b) => {
                let (wa, sa) = self.self_type(a)?;
                let (wb, sb) = self.self_type(b)?;
                match *op {
                    "+" | "-" | "*" | "/" | "%" | "&" | "|" | "^" | "~^" | "^~" => (wa.max(wb), sa && sb),
                    "<<" | ">>" | "<<<" | ">>>" | "**" => (wa, sa),
                    _ => (1, false),
                }
            }
            Expr::Ternary(_, t, f) => {
                let (wt, st) = self.self_type(t)?;
                let (wf, sf) = self.self_type(f)?;
                (wt.max(wf), st && sf)
            }
            Expr::Concat(items) => {
                let mut w = 0;
                for i in items {
                    w += self.self_type(i)?.0;
                }
                (w, false)
            }
            Expr::Repeat(n, items) => {
                let k = self.const_int(n)? as u32;
                let mut w = 0;
                for i in items {
                    w += self.self_type(i)?.0;
                }
                (k * w, false)
            }
            Expr::Index(base, _) => match base.as_ref() {
                Expr::Ident(n) if self.mems.contains_key(n) => (self.mems[n].width, false),
                _ => (1, false),
            },
            Expr::Range(_, a, b) => ((self.const_int(a)? - self.const_int(b)?).unsigned_abs() as u32 + 1, false),
            Expr::IndexedRange { width, .. } => (self.const_int(width)? as u32, false),
            Expr::Str(_) | Expr::Real(_) => return err(format!("string or real in a value context: {e:?}")),
            Expr::Call(n, args) => match (n.as_str(), args.as_slice()) {
                ("$signed", [a]) => (self.self_type(a)?.0, true),
                ("$unsigned", [a]) => (self.self_type(a)?.0, false),
                ("$clog2", _) => (32, true),
                _ => return err(format!("unsupported call {n}")),
            },
        })
    }

    // ---- cells ---------------------------------------------------------------

    fn unary(&mut self, op: Op, a: SigSpec, a_signed: bool, w: u32) -> SigSpec {
        let y = self.m.wire(w);
        self.m.cells.push(Cell::Unary { op, a, a_signed, y: y.clone() });
        y
    }

    fn binary(&mut self, op: Op, a: SigSpec, b: SigSpec, signed: bool, w: u32) -> SigSpec {
        let y = self.m.wire(w);
        self.m.cells.push(Cell::Binary { op, a, b, signed, y: y.clone() });
        y
    }

    fn mux(&mut self, s: SigSpec, a: SigSpec, b: SigSpec) -> SigSpec {
        if a == b {
            return a;
        }
        let y = self.m.wire(a.width());
        self.m.cells.push(Cell::Mux { a, b, s, y: y.clone() });
        y
    }

    /// A one-bit truth value of `v` (`$reduce_bool` when wider).
    fn truth(&mut self, v: SigSpec) -> SigSpec {
        if v.width() == 1 {
            v
        } else {
            self.unary(Op::ReduceOr, v, false, 1)
        }
    }

    // ---- expression evaluation -----------------------------------------------

    /// `e` evaluated in a context of `w` bits, signedness `s`.
    fn eval(&mut self, e: &Expr, w: u32, s: bool) -> R<SigSpec> {
        Ok(match e {
            Expr::Number { .. } => {
                let c = self.const_eval(e)?;
                const_spec(&c.bits).extend(w, s)
            }
            Expr::Ident(n) => {
                if let Some(net) = self.nets.get(n) {
                    SigSpec::of(net.id, net.width).extend(w, s)
                } else if let Some(c) = self.params.get(n) {
                    const_spec(&c.bits).extend(w, s)
                } else {
                    return err(format!("undeclared {n}"));
                }
            }
            Expr::Unary(op, a) => match *op {
                "+" => self.eval(a, w, s)?,
                "-" => {
                    let v = self.eval(a, w, s)?;
                    self.unary(Op::Neg, v, s, w)
                }
                "~" => {
                    let v = self.eval(a, w, s)?;
                    self.unary(Op::Not, v, s, w)
                }
                "!" | "&" | "|" | "^" | "~&" | "~|" | "~^" | "^~" => {
                    let (wa, sa) = self.self_type(a)?;
                    let v = self.eval(a, wa, sa)?;
                    let r = match *op {
                        "!" => self.unary(Op::LogicNot, v, sa, 1),
                        "&" => self.unary(Op::ReduceAnd, v, sa, 1),
                        "|" => self.unary(Op::ReduceOr, v, sa, 1),
                        "^" => self.unary(Op::ReduceXor, v, sa, 1),
                        "~&" => {
                            let t = self.unary(Op::ReduceAnd, v, sa, 1);
                            self.unary(Op::Not, t, false, 1)
                        }
                        "~|" => {
                            let t = self.unary(Op::ReduceOr, v, sa, 1);
                            self.unary(Op::Not, t, false, 1)
                        }
                        _ => {
                            let t = self.unary(Op::ReduceXor, v, sa, 1);
                            self.unary(Op::Not, t, false, 1)
                        }
                    };
                    r.extend(w, false)
                }
                o => return err(format!("unary {o}")),
            },
            Expr::Binary(op, a, b) => match *op {
                "+" | "-" | "*" | "&" | "|" | "^" | "~^" | "^~" => {
                    let va = self.eval(a, w, s)?;
                    let vb = self.eval(b, w, s)?;
                    let o = match *op {
                        "+" => Op::Add,
                        "-" => Op::Sub,
                        "*" => Op::Mul,
                        "&" => Op::And,
                        "|" => Op::Or,
                        "^" => Op::Xor,
                        _ => Op::Xnor,
                    };
                    self.binary(o, va, vb, s, w)
                }
                "<<" | ">>" | "<<<" | ">>>" => {
                    let va = self.eval(a, w, s)?;
                    let (wb, _) = self.self_type(b)?;
                    let vb = self.eval(b, wb, false)?;
                    let o = match *op {
                        "<<" | "<<<" => Op::Shl,
                        ">>>" if s => Op::Sshr,
                        _ => Op::Shr,
                    };
                    // The shift amount is unsigned whatever its type.
                    let y = self.m.wire(w);
                    self.m.cells.push(Cell::Binary { op: o, a: va, b: vb, signed: s && o == Op::Sshr, y: y.clone() });
                    y
                }
                "==" | "!=" | "<" | "<=" | ">" | ">=" | "===" | "!==" => {
                    let (wa, sa) = self.self_type(a)?;
                    let (wb, sb) = self.self_type(b)?;
                    let (cw, cs) = (wa.max(wb), sa && sb);
                    let va = self.eval(a, cw, cs)?;
                    let vb = self.eval(b, cw, cs)?;
                    let o = match *op {
                        "==" | "===" => Op::Eq,
                        "!=" | "!==" => Op::Ne,
                        "<" => Op::Lt,
                        "<=" => Op::Le,
                        ">" => Op::Gt,
                        _ => Op::Ge,
                    };
                    self.binary(o, va, vb, cs, 1).extend(w, false)
                }
                "&&" | "||" => {
                    let (wa, sa) = self.self_type(a)?;
                    let (wb, sb) = self.self_type(b)?;
                    let va = self.eval(a, wa, sa)?;
                    let vb = self.eval(b, wb, sb)?;
                    let o = if *op == "&&" { Op::LogicAnd } else { Op::LogicOr };
                    self.binary(o, va, vb, false, 1).extend(w, false)
                }
                o => return err(format!("binary {o} not supported yet")),
            },
            Expr::Ternary(c, t, f) => {
                let (wc, sc) = self.self_type(c)?;
                let vc = self.eval(c, wc, sc)?;
                let sel = self.truth(vc);
                let vt = self.eval(t, w, s)?;
                let vf = self.eval(f, w, s)?;
                self.mux(sel, vf, vt)
            }
            Expr::Concat(items) => {
                let mut out = SigSpec::default();
                for i in items.iter().rev() {
                    let (wi, si) = self.self_type(i)?;
                    let v = self.eval(i, wi, si)?;
                    out = out.concat(&v);
                }
                out.extend(w, false)
            }
            Expr::Repeat(n, items) => {
                let k = self.const_int(n)?;
                let one = self.eval(&Expr::Concat(items.clone()), self.self_type(&Expr::Concat(items.clone()))?.0, false)?;
                let mut out = SigSpec::default();
                for _ in 0..k {
                    out = out.concat(&one);
                }
                out.extend(w, false)
            }
            Expr::Index(base, idx) => {
                if let Expr::Ident(n) = base.as_ref() {
                    if self.mems.contains_key(n) {
                        return Ok(self.mem_read(n, idx)?.extend(w, false));
                    }
                }
                let v = self.select(base, idx, idx)?;
                v.extend(w, false)
            }
            Expr::Range(base, msb, lsb) => self.select(base, msb, lsb)?.extend(w, false),
            Expr::IndexedRange { e, base, width, up } => {
                let b = self.const_int(base)?;
                let wd = self.const_int(width)?;
                let (msb, lsb) = if *up { (b + wd - 1, b) } else { (b, b - wd + 1) };
                let mk = |v: i64| Expr::Number { width: None, signed: true, bits: (0..32).map(|i| Some((v >> i) & 1 == 1)).collect() };
                self.select(e, &mk(msb), &mk(lsb))?.extend(w, false)
            }
            Expr::Str(_) | Expr::Real(_) => return err(format!("string or real in a value context: {e:?}")),
            Expr::Call(n, args) => match (n.as_str(), args.as_slice()) {
                ("$signed", [a]) => {
                    let (wa, _) = self.self_type(a)?;
                    self.eval(a, wa, false)?.extend(w, s)
                }
                ("$unsigned", [a]) => {
                    let (wa, _) = self.self_type(a)?;
                    self.eval(a, wa, false)?.extend(w, false)
                }
                ("$clog2", _) => {
                    let c = self.const_eval(e)?;
                    const_spec(&c.bits).extend(w, s)
                }
                _ => return err(format!("unsupported call {n}")),
            },
        })
    }

    /// A constant part select of a net; bits outside its range are x.
    fn select(&mut self, base: &Expr, msb: &Expr, lsb: &Expr) -> R<SigSpec> {
        let Expr::Ident(n) = base else { return err(format!("select of a non-identifier: {base:?}")) };
        let Some(net) = self.nets.get(n).cloned() else { return err(format!("select of undeclared {n}")) };
        let (Ok(hi), Ok(lo)) = (self.const_int(msb), self.const_int(lsb)) else {
            return err(format!("variable index into {n} is not supported yet"));
        };
        // Walk from the select's LSB end upward.
        let (from, to, step): (i64, i64, i64) = if net.msb >= net.lsb { (lo, hi, 1) } else { (lo, hi, -1) };
        let count = (to - from) * step + 1;
        if count <= 0 {
            return err(format!("reversed part select on {n}"));
        }
        let bits = (0..count)
            .map(|k| match net.pos(from + k * step) {
                Some(p) => Bit::Sig(net.id, p),
                None => Bit::X,
            })
            .collect();
        Ok(SigSpec(bits))
    }

    fn mem_read(&mut self, name: &str, addr: &Expr) -> R<SigSpec> {
        let mem = self.mems[name].clone();
        let (wa, sa) = self.self_type(addr)?;
        let a = self.eval(addr, wa, sa)?;
        let data = self.m.wire(mem.width);
        if let Cell::Mem(mm) = &mut self.m.cells[mem.index] {
            mm.reads.push(MemRead { clk: None, addr: a, data: data.clone() });
        }
        Ok(data)
    }

    // ---- assignments -------------------------------------------------------------

    /// The bits an lvalue names.
    fn lvalue(&mut self, e: &Expr) -> R<SigSpec> {
        match e {
            Expr::Ident(n) => match self.nets.get(n) {
                Some(net) => Ok(SigSpec::of(net.id, net.width)),
                None => err(format!("assignment to undeclared {n}")),
            },
            Expr::Index(b, i) => self.select(b, i, i),
            Expr::Range(b, m, l) => self.select(b, m, l),
            Expr::IndexedRange { .. } => {
                let v = self.eval(e, self.self_type(e)?.0, false)?;
                Ok(v)
            }
            Expr::Concat(items) => {
                let mut out = SigSpec::default();
                for i in items.iter().rev() {
                    out = out.concat(&self.lvalue(i)?);
                }
                Ok(out)
            }
            other => err(format!("not an lvalue: {other:?}")),
        }
    }

    /// `rhs` evaluated for an lvalue of `lw` bits.
    fn rvalue(&mut self, rhs: &Expr, lw: u32) -> R<SigSpec> {
        let (rw, rs) = self.self_type(rhs)?;
        let w = lw.max(rw);
        Ok(self.eval(rhs, w, rs)?.slice(0, lw))
    }

    fn assign(&mut self, lhs: &Expr, rhs: &Expr) -> R<()> {
        let l = self.lvalue(lhs)?;
        let r = self.rvalue(rhs, l.width())?;
        self.m.connects.push((l, r));
        Ok(())
    }

    fn set_init(&mut self, name: &str, v: &Expr) -> R<()> {
        let Some(net) = self.nets.get(name).cloned() else { return err(format!("initial value for undeclared {name}")) };
        let c = self.const_eval(v)?;
        let mut bits = c.bits.clone();
        let fill = if c.signed { bits.last().copied().flatten() } else { Some(false) };
        bits.resize(net.width as usize, fill);
        bits.truncate(net.width as usize);
        self.m.signals[net.id.0 as usize].init = Some(bits);
        Ok(())
    }

    fn initial(&mut self, body: &Stmt) -> R<()> {
        match body {
            Stmt::Block(v) => {
                for s in v {
                    self.initial(s)?;
                }
                Ok(())
            }
            Stmt::Assign { lhs: Expr::Ident(n), rhs, .. } if self.nets.contains_key(n) => self.set_init(n, rhs),
            Stmt::Assign { lhs: Expr::Index(b, i), rhs, .. } => {
                let Expr::Ident(n) = b.as_ref() else { return err("initial: indexed non-memory") };
                let Some(mem) = self.mems.get(n).cloned() else { return err(format!("initial: {n} is not a memory")) };
                let a = self.const_int(i)?;
                let c = self.const_eval(rhs)?;
                let mut bits = c.bits.clone();
                bits.resize(mem.width as usize, Some(false));
                if let Cell::Mem(mm) = &mut self.m.cells[mem.index] {
                    mm.init.insert((a - mem.lo) as u32, bits);
                }
                Ok(())
            }
            Stmt::SysCall(n, args) if n == "$readmemh" || n == "$readmemb" => {
                let mut attrs = BTreeMap::new();
                attrs.insert("init_file".to_string(), format!("{:?}", args.first()));
                attrs.insert("init_kind".to_string(), n.clone());
                if let Some(Expr::Ident(m)) = args.get(1) {
                    if let Some(mem) = self.mems.get(m).cloned() {
                        if let Cell::Mem(mm) = &mut self.m.cells[mem.index] {
                            mm.attrs.extend(attrs);
                        }
                    }
                }
                Ok(())
            }
            Stmt::Empty => Ok(()),
            other => err(format!("unsupported in initial: {other:?}")),
        }
    }

    // ---- processes -------------------------------------------------------------------

    /// The current value of `bits` in `env`, falling back to `base` (the
    /// signal itself, for a register; an error for a combinational
    /// variable read before it is assigned).
    fn read_env(env: &Env, bits: &SigSpec) -> SigSpec {
        SigSpec(
            bits.0
                .iter()
                .map(|b| match b {
                    Bit::Sig(id, i) => env.get(id).and_then(|v| v[*i as usize]).unwrap_or(*b),
                    other => *other,
                })
                .collect(),
        )
    }

    fn process(&mut self, sens: &Sens, body: &Stmt) -> R<()> {
        let mut env: Env = BTreeMap::new();
        let mut writes: Vec<(String, SigSpec, SigSpec, SigSpec)> = Vec::new(); // mem, addr, data, enable
        let one = SigSpec(vec![Bit::One]);
        self.exec(body, &mut env, &one, &mut writes, matches!(sens, Sens::Comb))?;
        match sens {
            Sens::Comb => {
                for (id, bits) in env {
                    let w = self.m.signal(id).width;
                    for (i, b) in bits.iter().enumerate() {
                        match b {
                            Some(v) => self.m.connects.push((SigSpec(vec![Bit::Sig(id, i as u32)]), SigSpec(vec![*v]))),
                            None => {
                                return err(format!(
                                    "{}[{i}] of {w} bits is not assigned on every path of an always @(*): a latch, not supported",
                                    self.m.signal(id).name
                                ));
                            }
                        }
                    }
                }
                if !writes.is_empty() {
                    return err("memory write in a combinational process");
                }
            }
            Sens::Edges(edges) => {
                if edges.len() != 1 {
                    return err("asynchronous set/reset processes are not supported yet");
                }
                let (posedge, clk) = &edges[0];
                let (cw, _) = self.self_type(clk)?;
                let clk = self.eval(clk, cw, false)?;
                for (id, bits) in env {
                    let w = self.m.signal(id).width;
                    let q = SigSpec::of(id, w);
                    let d = SigSpec(bits.iter().enumerate().map(|(i, b)| b.unwrap_or(Bit::Sig(id, i as u32))).collect());
                    // Only the assigned bits become flops; the rest stay wires.
                    let assigned: Vec<usize> = (0..w as usize).filter(|&i| bits[i].is_some()).collect();
                    if assigned.len() == w as usize {
                        self.m.cells.push(Cell::Dff { clk: clk.clone(), posedge: *posedge, d, q });
                    } else {
                        for i in assigned {
                            self.m.cells.push(Cell::Dff {
                                clk: clk.clone(),
                                posedge: *posedge,
                                d: SigSpec(vec![d.0[i]]),
                                q: SigSpec(vec![q.0[i]]),
                            });
                        }
                    }
                }
                for (name, addr, data, en) in writes {
                    let mem = self.mems[&name].clone();
                    let en_bits = SigSpec(vec![en.0[0]; mem.width as usize]);
                    if let Cell::Mem(mm) = &mut self.m.cells[mem.index] {
                        mm.writes.push(MemWrite { clk: (clk.clone(), *posedge), addr, data, en: en_bits });
                    }
                }
            }
        }
        Ok(())
    }

    /// Execute `s` on `env` under path condition `cond`.
    fn exec(&mut self, s: &Stmt, env: &mut Env, cond: &SigSpec, writes: &mut Vec<(String, SigSpec, SigSpec, SigSpec)>, comb: bool) -> R<()> {
        match s {
            Stmt::Empty => Ok(()),
            Stmt::Block(v) => {
                for x in v {
                    self.exec(x, env, cond, writes, comb)?;
                }
                Ok(())
            }
            Stmt::Assign { lhs, rhs, blocking } => {
                // A memory write.
                if let Expr::Index(b, i) = lhs {
                    if let Expr::Ident(n) = b.as_ref() {
                        if let Some(mem) = self.mems.get(n).cloned() {
                            let (wa, sa) = self.self_type(i)?;
                            let addr = self.eval(i, wa, sa)?;
                            let data = self.rvalue(rhs, mem.width)?;
                            writes.push((n.clone(), addr, data, cond.clone()));
                            return Ok(());
                        }
                    }
                }
                let l = self.lvalue(lhs)?;
                let r = self.rvalue(rhs, l.width())?;
                // Combinational and blocking reads see earlier assignments.
                let r = if comb || *blocking { Self::read_env_rewrite(self, env, r) } else { r };
                for (lb, rb) in l.0.iter().zip(&r.0) {
                    if let Bit::Sig(id, i) = lb {
                        let w = self.m.signal(*id).width as usize;
                        env.entry(*id).or_insert_with(|| vec![None; w])[*i as usize] = Some(*rb);
                    }
                }
                Ok(())
            }
            Stmt::If { cond: c, then, els } => {
                let (wc, sc) = self.self_type(c)?;
                let vc = self.eval(c, wc, sc)?;
                let vc = if comb { Self::read_env_rewrite(self, env, vc) } else { vc };
                let sel = self.truth(vc);
                let ct = self.binary(Op::LogicAnd, cond.clone(), sel.clone(), false, 1);
                let nsel = self.unary(Op::LogicNot, sel.clone(), false, 1);
                let cf = self.binary(Op::LogicAnd, cond.clone(), nsel, false, 1);
                let mut et = env.clone();
                let mut ef = env.clone();
                self.exec(then, &mut et, &ct, writes, comb)?;
                if let Some(e) = els {
                    self.exec(e, &mut ef, &cf, writes, comb)?;
                }
                self.merge(env, &et, &ef, &sel);
                Ok(())
            }
            Stmt::Case { kind, subject, arms, default } => {
                // A priority chain of ifs, as yosys's proc builds it.
                let mut chain: Option<Stmt> = default.as_ref().map(|d| (**d).clone());
                for (labels, body) in arms.iter().rev() {
                    let mut c: Option<Expr> = None;
                    for l in labels {
                        let eq = match kind {
                            CaseKind::Case => Expr::Binary("==", Box::new(subject.clone()), Box::new(l.clone())),
                            _ => return err("casez/casex not supported yet"),
                        };
                        c = Some(match c {
                            None => eq,
                            Some(prev) => Expr::Binary("||", Box::new(prev), Box::new(eq)),
                        });
                    }
                    let Some(c) = c else { continue };
                    chain = Some(Stmt::If { cond: c, then: Box::new(body.clone()), els: chain.map(Box::new) });
                }
                match chain {
                    Some(s) => self.exec(&s, env, cond, writes, comb),
                    None => Ok(()),
                }
            }
            Stmt::SysCall(..) => Ok(()), // $display and friends: no hardware
        }
    }

    fn read_env_rewrite(&mut self, env: &Env, v: SigSpec) -> SigSpec {
        Self::read_env(env, &v)
    }

    /// After an if: each variable either branch assigned becomes
    /// `sel ? then : else`, an unassigned side keeping the prior value.
    fn merge(&mut self, env: &mut Env, et: &Env, ef: &Env, sel: &SigSpec) {
        let mut ids: Vec<SigId> = et.keys().chain(ef.keys()).copied().collect();
        ids.sort();
        ids.dedup();
        for id in ids {
            let w = self.m.signal(id).width as usize;
            let prior = env.get(&id).cloned().unwrap_or_else(|| vec![None; w]);
            let t = et.get(&id).cloned().unwrap_or_else(|| prior.clone());
            let f = ef.get(&id).cloned().unwrap_or_else(|| prior.clone());
            if t == f {
                env.insert(id, t);
                continue;
            }
            // Bits assigned on neither side stay unassigned; on one side
            // only, the other side reads the signal itself (a register's
            // hold, or for a combinational variable a latch, reported at
            // the end of the process).
            let mut out = prior.clone();
            let both: Vec<usize> = (0..w).filter(|&i| t[i].is_some() || f[i].is_some()).collect();
            if both.is_empty() {
                continue;
            }
            let pick = |v: &Vec<Option<Bit>>, i: usize| v[i].unwrap_or(Bit::Sig(id, i as u32));
            let a = SigSpec(both.iter().map(|&i| pick(&f, i)).collect());
            let b = SigSpec(both.iter().map(|&i| pick(&t, i)).collect());
            let y = self.mux(sel.clone(), a, b);
            for (k, &i) in both.iter().enumerate() {
                // A combinational bit assigned on one side only is still
                // unassigned on the other: keep it None so the latch check
                // sees it, unless it had a prior value.
                if (t[i].is_none() || f[i].is_none()) && prior[i].is_none() && self.is_comb_hint(id) {
                    out[i] = None;
                } else {
                    out[i] = Some(y.0[k]);
                }
            }
            env.insert(id, out);
        }
    }

    /// Registers are written by edge processes; everything else is
    /// combinational. Without that fact here, a one-sided assignment is
    /// treated as a hold of the signal's own value (correct for a
    /// register).
    fn is_comb_hint(&self, _id: SigId) -> bool {
        false
    }

    // ---- instances -------------------------------------------------------------------

    fn instance(&mut self, ty: &str, name: &str, params: &[(String, Expr)], conns: &[(String, Option<Expr>)]) -> R<()> {
        let mut p = BTreeMap::new();
        for (k, v) in params {
            let val = match v {
                Expr::Str(x) => Param::Str(x.clone()),
                // yosys writes a real parameter as a "%f" string.
                Expr::Real(x) => Param::Str(match x.replace('_', "").parse::<f64>() {
                    Ok(v) => format!("{v:.6}"),
                    Err(_) => return err(format!("bad real {x}")),
                }),
                Expr::Number { .. } | Expr::Unary(..) | Expr::Binary(..) | Expr::Ident(_) => match self.const_eval(v) {
                    Ok(c) => Param::Bits(c.bits),
                    Err(_) => Param::Str(format!("{v:?}")),
                },
                Expr::Call(..) => Param::Bits(self.const_eval(v)?.bits),
                other => Param::Str(format!("{other:?}")),
            };
            p.insert(k.clone(), val);
        }
        let mut c = BTreeMap::new();
        for (port, e) in conns {
            let Some(e) = e else { continue };
            let v = match self.lvalue(e) {
                Ok(v) => v,
                Err(_) => {
                    let (w, s) = self.self_type(e)?;
                    self.eval(e, w, s)?
                }
            };
            c.insert(port.clone(), v);
        }
        self.m.cells.push(Cell::Instance { ty: ty.to_string(), name: name.to_string(), params: p, conns: c });
        Ok(())
    }
}

fn const_spec(bits: &[Option<bool>]) -> SigSpec {
    SigSpec(
        bits.iter()
            .map(|b| match b {
                Some(true) => Bit::One,
                Some(false) => Bit::Zero,
                None => Bit::X,
            })
            .collect(),
    )
}

#[allow(dead_code)]
fn dir_name(d: Dir) -> &'static str {
    match d {
        Dir::Input => "input",
        Dir::Output => "output",
        Dir::Inout => "inout",
    }
}
