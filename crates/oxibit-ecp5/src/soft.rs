//! Soft primitives: ECP5 library cells that are plain fabric logic.
//!
//! An instance of one of these is rewritten into word-level netlist cells
//! before lowering, so synthesis maps it like any other logic (and the
//! random simulation check covers it). Semantics follow the Lattice FPGA
//! Libraries Reference Guide and, where yosys has a model, its
//! `techlibs/lattice` simulation cells:
//!
//! - LUT4..LUT8 (`Z = INIT[{..,B,A}]`), PFUMX (`Z = C0 ? ALUT : BLUT`),
//!   L6MUX21 (`Z = SD ? D1 : D0`), CCU2C (two LUT4s with the carry of
//!   `ccu2c_sim.vh`, INJECT1_x).
//! - Flip-flops FD1P3{AX,AY,BX,DX,IX,JX}, FD1S3{AX,AY,BX,DX,IX,JX}: D, SP
//!   (clock enable), CK, PD (preset) / CD (clear); B/D asynchronous, I/J
//!   synchronous with priority over SP; A no set/reset. The power-up value
//!   is the set/reset value (REGSET), AX 0 and AY 1.
//! - Scan flip-flops FL1P3{AY,AZ,BX,DX,IY,JY}, FL1S3{AX,AY}: D0, D1, SD
//!   (SD = 1 loads D1), otherwise as the FD1 family.
//! - Gates AND2..5, ND2..5, OR2..5, NR2..5, XOR2..5 / XOR11 / XOR21,
//!   XNOR2..5 (inputs A, B, C, ...; output Z), INV, BUFBA, VHI, VLO.
//! - MUX21 / MUX41 / MUX81 / MUX161 / MUX321 (D0.., SD or SD1.., Z).
//! - ROM16X1A .. ROM256X1A (AD0.., DO0, INITVAL).
//!
//! Not here: the fabric latches (FD1S1*), which the ECP5 slice has no
//! mode for; an instance of one is an error.
//! The IO latches (IFS1S1*) are IOLOGIC registers (`iol`).

use std::collections::BTreeMap;

use oxibit_hdl::netlist::{Bit, Cell, Module, Op, Param, SigSpec};

use crate::{Result, err};

/// Whether `ty` is a soft primitive this module rewrites.
pub fn is_soft(ty: &str) -> bool {
    kind(ty).is_some()
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Kind {
    Lut(usize),
    Pfumx,
    L6mux,
    Ccu2c,
    Ff { scan: bool, enable: bool, rs: Rs, init: bool },
    Gate { op: Op, n: usize, invert: bool },
    Inv,
    Buf,
    Const(bool),
    Mux(usize),
    Rom(usize),
}

/// The flop's set / reset.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Rs {
    None,
    /// Asynchronous; `true` presets.
    Async(bool),
    /// Synchronous, over the enable; `true` presets.
    Sync(bool),
}

fn kind(ty: &str) -> Option<Kind> {
    use Kind::*;
    let ff = |scan, enable, rs, init| Some(Ff { scan, enable, rs, init });
    Some(match ty {
        "LUT4" => Lut(4),
        "LUT5" => Lut(5),
        "LUT6" => Lut(6),
        "LUT7" => Lut(7),
        "LUT8" => Lut(8),
        "PFUMX" => Pfumx,
        "L6MUX21" => L6mux,
        "CCU2C" => Ccu2c,
        "FD1P3AX" => return ff(false, true, Rs::None, false),
        "FD1P3AY" => return ff(false, true, Rs::None, true),
        "FD1P3BX" => return ff(false, true, Rs::Async(true), true),
        "FD1P3DX" => return ff(false, true, Rs::Async(false), false),
        "FD1P3IX" => return ff(false, true, Rs::Sync(false), false),
        "FD1P3JX" => return ff(false, true, Rs::Sync(true), true),
        "FD1S3AX" => return ff(false, false, Rs::None, false),
        "FD1S3AY" => return ff(false, false, Rs::None, true),
        "FD1S3BX" => return ff(false, false, Rs::Async(true), true),
        "FD1S3DX" => return ff(false, false, Rs::Async(false), false),
        "FD1S3IX" => return ff(false, false, Rs::Sync(false), false),
        "FD1S3JX" => return ff(false, false, Rs::Sync(true), true),
        "FL1P3AZ" => return ff(true, true, Rs::None, false),
        "FL1P3AY" => return ff(true, true, Rs::None, true),
        "FL1P3BX" => return ff(true, true, Rs::Async(true), true),
        "FL1P3DX" => return ff(true, true, Rs::Async(false), false),
        "FL1P3IY" => return ff(true, true, Rs::Sync(false), false),
        "FL1P3JY" => return ff(true, true, Rs::Sync(true), true),
        "FL1S3AX" => return ff(true, false, Rs::None, false),
        "FL1S3AY" => return ff(true, false, Rs::None, true),
        "INV" => Inv,
        "BUFBA" => Buf,
        "VHI" => Const(true),
        "VLO" => Const(false),
        "MUX21" => Mux(1),
        "MUX41" => Mux(2),
        "MUX81" => Mux(3),
        "MUX161" => Mux(4),
        "MUX321" => Mux(5),
        "ROM16X1A" => Rom(4),
        "ROM32X1A" => Rom(5),
        "ROM64X1A" => Rom(6),
        "ROM128X1A" => Rom(7),
        "ROM256X1A" => Rom(8),
        _ => return gate(ty),
    })
}

fn gate(ty: &str) -> Option<Kind> {
    let (op, invert, rest) = if let Some(r) = ty.strip_prefix("XNOR") {
        (Op::Xor, true, r)
    } else if let Some(r) = ty.strip_prefix("XOR") {
        (Op::Xor, false, r)
    } else if let Some(r) = ty.strip_prefix("AND") {
        (Op::And, false, r)
    } else if let Some(r) = ty.strip_prefix("ND") {
        (Op::And, true, r)
    } else if let Some(r) = ty.strip_prefix("OR") {
        (Op::Or, false, r)
    } else if let Some(r) = ty.strip_prefix("NR") {
        (Op::Or, true, r)
    } else {
        return None;
    };
    let n: usize = rest.parse().ok()?;
    let ok = match (op, invert) {
        (Op::Xor, false) => (2..=5).contains(&n) || n == 11 || n == 21,
        _ => (2..=5).contains(&n),
    };
    ok.then_some(Kind::Gate { op, n, invert })
}

/// Input pin names `A`, `B`, ... (`n` of them).
fn letters(n: usize) -> Vec<String> {
    (0..n).map(|i| char::from(b'A' + u8::try_from(i).expect("small")).to_string()).collect()
}

/// A parameter as `width` bits, LSB first. Strings may be hex
/// (`"0x1234"`), binary (`"0b0101"`) or decimal.
fn param_bits(params: &BTreeMap<String, Param>, name: &str, width: usize, inst: &str) -> Result<Vec<bool>> {
    let v: Vec<bool> = match params.get(name) {
        None => vec![false; width],
        Some(Param::Bits(b)) => b.iter().map(|x| *x == Some(true)).collect(),
        Some(Param::Int(i)) => (0..64).map(|k| (i >> k) & 1 == 1).collect(),
        Some(Param::Str(s)) => {
            let s = s.trim().replace('_', "");
            let (digits, radix) = if let Some(h) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
                (h.to_owned(), 16)
            } else if let Some(b) = s.strip_prefix("0b").or_else(|| s.strip_prefix("0B")) {
                (b.to_owned(), 2)
            } else {
                (s.clone(), 10)
            };
            if radix == 10 {
                let x: u128 = digits.parse().map_err(|_| err!("{inst}: {name} = {s:?} is not a number"))?;
                (0..128).map(|k| (x >> k) & 1 == 1).collect()
            } else {
                let per = if radix == 16 { 4 } else { 1 };
                let mut bits = Vec::new();
                for ch in digits.chars().rev() {
                    let d = ch.to_digit(radix).ok_or_else(|| err!("{inst}: {name} = {s:?} is not a number"))?;
                    for k in 0..per {
                        bits.push((d >> k) & 1 == 1);
                    }
                }
                bits
            }
        }
    };
    let mut v = v;
    v.resize(width.max(v.len()), false);
    v.truncate(width);
    Ok(v)
}

fn param_str(params: &BTreeMap<String, Param>, name: &str, default: &str) -> String {
    match params.get(name) {
        Some(Param::Str(s)) => s.clone(),
        _ => default.to_owned(),
    }
}

fn konst(bits: &[bool]) -> SigSpec {
    SigSpec(bits.iter().map(|&b| if b { Bit::One } else { Bit::Zero }).collect())
}

struct Ctx<'a> {
    m: &'a mut Module,
    cells: Vec<Cell>,
}

impl Ctx<'_> {
    fn wire(&mut self) -> SigSpec {
        self.m.wire(1)
    }
    fn mux(&mut self, s: SigSpec, a: SigSpec, b: SigSpec) -> SigSpec {
        let y = self.wire();
        self.cells.push(Cell::Mux { a, b, s, y: y.clone() });
        y
    }
    fn unary(&mut self, op: Op, a: SigSpec) -> SigSpec {
        let y = self.wire();
        self.cells.push(Cell::Unary { op, a, a_signed: false, y: y.clone() });
        y
    }
    fn binary(&mut self, op: Op, a: SigSpec, b: SigSpec) -> SigSpec {
        let y = self.wire();
        self.cells.push(Cell::Binary { op, a, b, signed: false, y: y.clone() });
        y
    }
    /// `table[sel]`, `sel` LSB first.
    fn lookup(&mut self, table: &[bool], sel: SigSpec) -> SigSpec {
        let y = self.wire();
        self.cells.push(Cell::Binary { op: Op::Shiftx, a: konst(table), b: sel, signed: false, y: y.clone() });
        y
    }
    fn select(&mut self, data: SigSpec, sel: SigSpec) -> SigSpec {
        let y = self.wire();
        self.cells.push(Cell::Binary { op: Op::Shiftx, a: data, b: sel, signed: false, y: y.clone() });
        y
    }
    /// `y = a`, through a buffer so `y` keeps a driver of its own.
    fn drive(&mut self, y: SigSpec, a: SigSpec) {
        self.m.connects.push((y, a));
    }
}

/// Rewrite every soft-primitive instance in `m`. Returns how many.
pub fn rewrite(m: &mut Module) -> Result<usize> {
    let cells = std::mem::take(&mut m.cells);
    let mut cx = Ctx { m, cells: Vec::with_capacity(cells.len()) };
    let mut n = 0;
    for c in cells {
        let Cell::Instance { ty, name, params, conns, .. } = &c else {
            cx.cells.push(c);
            continue;
        };
        if ty.starts_with("FD1S1") {
            return Err(err!("instance {name}: {ty} is a latch; the ECP5 slice has no latch mode"));
        }
        if is_lutram(ty) {
            let canon = canonical_lutram(ty, name, params, conns)?;
            cx.cells.push(canon);
            n += 1;
            continue;
        }
        let Some(k) = kind(ty) else {
            cx.cells.push(c);
            continue;
        };
        one(&mut cx, k, ty, name, params, conns)?;
        n += 1;
    }
    let Ctx { m, cells } = cx;
    m.cells = cells;
    Ok(n)
}

fn one(
    cx: &mut Ctx<'_>,
    k: Kind,
    ty: &str,
    name: &str,
    params: &BTreeMap<String, Param>,
    conns: &BTreeMap<String, SigSpec>,
) -> Result<()> {
    let pin = |p: &str| -> Result<SigSpec> {
        match conns.get(p) {
            Some(s) if s.width() == 1 => Ok(s.clone()),
            Some(s) => Err(err!("{ty} {name}: pin {p} is {} bits, not 1", s.width())),
            None => Ok(SigSpec(vec![Bit::Zero])),
        }
    };
    let out = |p: &str| conns.get(p).filter(|s| s.width() == 1).cloned();
    let cat = |v: &[SigSpec]| SigSpec(v.iter().flat_map(|s| s.0.iter().copied()).collect());
    match k {
        Kind::Lut(n) => {
            let ins: Vec<SigSpec> = letters(n).iter().map(|p| pin(p)).collect::<Result<_>>()?;
            let init = param_bits(params, "INIT", 1 << n, name)?;
            if let Some(z) = out("Z") {
                let y = cx.lookup(&init, cat(&ins));
                cx.drive(z, y);
            }
        }
        Kind::Pfumx => {
            if let Some(z) = out("Z") {
                let y = cx.mux(pin("C0")?, pin("BLUT")?, pin("ALUT")?);
                cx.drive(z, y);
            }
        }
        Kind::L6mux => {
            if let Some(z) = out("Z") {
                let y = cx.mux(pin("SD")?, pin("D0")?, pin("D1")?);
                cx.drive(z, y);
            }
        }
        Kind::Ccu2c => ccu2c(cx, name, params, &pin, &out)?,
        Kind::Ff { scan, enable, rs, init } => {
            let Some(q) = out("Q") else { return Ok(()) };
            let d = if scan { cx.mux(pin("SD")?, pin("D0")?, pin("D1")?) } else { pin("D")? };
            let mut next = if enable { cx.mux(pin("SP")?, q.clone(), d) } else { d };
            let rs_pin = match rs {
                Rs::None => None,
                Rs::Async(true) | Rs::Sync(true) => Some(pin("PD")?),
                Rs::Async(false) | Rs::Sync(false) => Some(pin("CD")?),
            };
            if let (Rs::Sync(v), Some(r)) = (rs, rs_pin.clone()) {
                next = cx.mux(r, next, konst(&[v]));
            }
            let clk = pin("CK")?;
            match (rs, rs_pin) {
                (Rs::Async(v), Some(r)) => cx.cells.push(Cell::Adff {
                    clk,
                    posedge: true,
                    arst: r,
                    arst_high: true,
                    value: vec![v],
                    d: next,
                    q: q.clone(),
                }),
                _ => cx.cells.push(Cell::Dff { clk, posedge: true, d: next, q: q.clone() }),
            }
            set_init(cx.m, &q, init);
        }
        Kind::Gate { op, n, invert } => {
            let ins: Vec<SigSpec> = letters(n).iter().map(|p| pin(p)).collect::<Result<_>>()?;
            if let Some(z) = out("Z") {
                let red = match op {
                    Op::And => Op::ReduceAnd,
                    Op::Or => Op::ReduceOr,
                    _ => Op::ReduceXor,
                };
                let mut y = cx.unary(red, cat(&ins));
                if invert {
                    y = cx.unary(Op::Not, y);
                }
                cx.drive(z, y);
            }
        }
        Kind::Inv => {
            if let Some(z) = out("Z") {
                let y = cx.unary(Op::Not, pin("A")?);
                cx.drive(z, y);
            }
        }
        Kind::Buf => {
            if let Some(z) = out("Z") {
                cx.drive(z, pin("A")?);
            }
        }
        Kind::Const(v) => {
            if let Some(z) = out("Z") {
                cx.drive(z, konst(&[v]));
            }
        }
        Kind::Mux(sel) => {
            let data: Vec<SigSpec> = (0..1 << sel).map(|i| pin(&format!("D{i}"))).collect::<Result<_>>()?;
            let sels: Vec<SigSpec> = if sel == 1 {
                vec![pin("SD")?]
            } else {
                (1..=sel).map(|i| pin(&format!("SD{i}"))).collect::<Result<_>>()?
            };
            if let Some(z) = out("Z") {
                let y = cx.select(cat(&data), cat(&sels));
                cx.drive(z, y);
            }
        }
        Kind::Rom(a) => {
            let ad: Vec<SigSpec> = (0..a).map(|i| pin(&format!("AD{i}"))).collect::<Result<_>>()?;
            let init = param_bits(params, "INITVAL", 1 << a, name)?;
            if let Some(z) = out("DO0") {
                let y = cx.lookup(&init, cat(&ad));
                cx.drive(z, y);
            }
        }
    }
    Ok(())
}

/// CCU2C as `ccu2c_sim.vh`: per half a LUT4 `f` (INITx) and a LUT2 `g`
/// (INITx[3:0]); `S = f ^ cin'`, `cout = (~f & g') | (f & cin)`, where
/// INJECT1_x = "YES" zeroes `cin'` and `g'`.
fn ccu2c(
    cx: &mut Ctx<'_>,
    name: &str,
    params: &BTreeMap<String, Param>,
    pin: &dyn Fn(&str) -> Result<SigSpec>,
    out: &dyn Fn(&str) -> Option<SigSpec>,
) -> Result<()> {
    let mut cin = pin("CIN")?;
    for h in 0..2 {
        let init = param_bits(params, &format!("INIT{h}"), 16, name)?;
        let inject = param_str(params, &format!("INJECT1_{h}"), "YES") == "YES";
        let ins = SigSpec(
            ["A", "B", "C", "D"]
                .iter()
                .map(|p| pin(&format!("{p}{h}")).map(|s| s.0[0]))
                .collect::<Result<_>>()?,
        );
        let f = cx.lookup(&init, ins.clone());
        let (cin_g, g) = if inject {
            (konst(&[false]), konst(&[false]))
        } else {
            (cin.clone(), cx.lookup(&init[..4], ins.slice(0, 2)))
        };
        if let Some(s) = out(&format!("S{h}")) {
            let y = cx.binary(Op::Xor, f.clone(), cin_g);
            cx.drive(s, y);
        }
        let nf = cx.unary(Op::Not, f.clone());
        let a = cx.binary(Op::And, nf, g);
        let b = cx.binary(Op::And, f, cin.clone());
        cin = cx.binary(Op::Or, a, b);
    }
    if let Some(co) = out("COUT") {
        cx.drive(co, cin);
    }
    Ok(())
}

/// Give the flop output bit `q` its power-up value.
fn set_init(m: &mut Module, q: &SigSpec, v: bool) {
    for b in &q.0 {
        if let Bit::Sig(id, i) = *b {
            let s = &mut m.signals[id.0 as usize];
            let w = s.width as usize;
            let init = s.init.get_or_insert_with(|| vec![None; w]);
            if let Some(slot) = init.get_mut(i as usize) {
                *slot = Some(v);
            }
        }
    }
}

/// The kept distributed-RAM primitive: TRELLIS_DPR16X4 with scalar pins
/// (DI0..3, WAD0..3, RAD0..3, WRE, WCK, DO0..3), INITVAL as 64 bits
/// (word `w` in bits `4w..4w+3`), WCKMUX (WCK / INV), WREMUX (WRE / INV /
/// 0 / 1). `rewrite` turns DPR16X4C, SPR16X4C and TRELLIS_DPR16X4 (vector
/// or scalar pins) into it.
pub const LUTRAM: &str = "TRELLIS_DPR16X4S";

fn is_lutram(ty: &str) -> bool {
    matches!(ty, "DPR16X4C" | "SPR16X4C" | "TRELLIS_DPR16X4")
}

/// Pin `p` bit `i` of 4: `p[i]` from a 4-bit vector or scalar `p<i>`.
fn bit4(conns: &BTreeMap<String, SigSpec>, p: &str, i: usize) -> Option<SigSpec> {
    if let Some(s) = conns.get(&format!("{p}{i}")) {
        return Some(s.clone());
    }
    conns.get(p).filter(|s| s.width() == 4).map(|s| s.slice(u32::try_from(i).expect("small"), 1))
}

fn canonical_lutram(
    ty: &str,
    name: &str,
    params: &BTreeMap<String, Param>,
    conns: &BTreeMap<String, SigSpec>,
) -> Result<Cell> {
    let spr = ty == "SPR16X4C";
    let (wad, rad, wck) = if spr { ("AD", "AD", "CK") } else { ("WAD", "RAD", "WCK") };
    let mut c = BTreeMap::new();
    for i in 0..4 {
        for (from, to) in [("DI", "DI"), (wad, "WAD"), (rad, "RAD"), ("DO", "DO")] {
            if let Some(s) = bit4(conns, from, i) {
                c.insert(format!("{to}{i}"), s);
            }
        }
    }
    for (from, to) in [("WRE", "WRE"), (wck, "WCK")] {
        if let Some(s) = conns.get(from) {
            c.insert(to.to_owned(), s.clone());
        }
    }
    let mut p = BTreeMap::new();
    let init = param_bits(params, "INITVAL", 64, name)?;
    p.insert("INITVAL".to_owned(), Param::Bits(init.into_iter().map(Some).collect()));
    p.insert("WCKMUX".to_owned(), Param::Str(param_str(params, "WCKMUX", "WCK")));
    p.insert("WREMUX".to_owned(), Param::Str(param_str(params, "WREMUX", "WRE")));
    Ok(Cell::Instance {
        ty: LUTRAM.to_owned(),
        name: name.to_owned(),
        params: p,
        conns: c,
        attrs: BTreeMap::new(),
    })
}
