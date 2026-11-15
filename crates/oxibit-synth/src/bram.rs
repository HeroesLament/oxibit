//! Memories to RAMB18E1 / RAMB36E1.
//!
//! Runs on the emitted netlist. A memory with one synchronous read port
//! and at most one write port with a single enable becomes columns of
//! block RAMs, in yosys's xc7 conventions (techlibs/xilinx/brams_xc6v_map.v)
//! because oxibit's bramcfg already matches those on silicon:
//!
//! - TDP: write on port A, read on port B. SDP (36 / 72 wide): read on A,
//!   write on B, data split low half A, high half B.
//! - Data in 9-bit groups for the 9 / 18 / 36 / 72 widths: 8 bits on DI,
//!   the ninth on DIP.
//! - Address at bit log2(width) up (6 for x72, 5 for x36), RAMB36 ADDR[15]
//!   tied high, WRITE_MODE READ_FIRST on both ports (the read port is
//!   non-transparent).
//!
//! Asynchronous-read memories up to 128 deep become distributed RAM
//! (`lutram`), up to 512 deep as banks of RAM128X1D with a LUT choosing
//! the bank (`lutram_banked`).
//!
//! Before that, each memory is put in a shape the primitives take
//! (`normalize`):
//!
//! - Address bits above what the memory's size needs are dropped. A write
//!   with any of them set is past the end, which `$mem_v2` ignores, so the
//!   write enable is gated off by them; a read past the end is x, so the
//!   read takes whatever word the low bits pick.
//! - A memory with several read ports and at most one write port becomes
//!   one copy per read port, each written by that write port.
//!
//! No depth cascading yet: a memory deeper than the deepest fitting shape
//! stays a `$mem_v2`. Nothing here is checked by the equivalence oracle (the
//! RAMB models in yosys's cells_sim are stubs); `synth` keeps the netlist
//! from before this pass as the checking view.

use std::collections::BTreeMap;

use oxibit_hdl::netlist::{Bit, Cell, Memory, Module, Param, SigSpec};
use oxibit_prof::hot;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Shape {
    /// RAMB36 (else RAMB18).
    big: bool,
    sdp: bool,
    width: usize,
    depth: usize,
}

const SHAPES: &[Shape] = &[
    Shape {
        big: false,
        sdp: false,
        width: 1,
        depth: 16384,
    },
    Shape {
        big: false,
        sdp: false,
        width: 2,
        depth: 8192,
    },
    Shape {
        big: false,
        sdp: false,
        width: 4,
        depth: 4096,
    },
    Shape {
        big: false,
        sdp: false,
        width: 9,
        depth: 2048,
    },
    Shape {
        big: false,
        sdp: false,
        width: 18,
        depth: 1024,
    },
    Shape {
        big: false,
        sdp: true,
        width: 36,
        depth: 512,
    },
    Shape {
        big: true,
        sdp: false,
        width: 1,
        depth: 32768,
    },
    Shape {
        big: true,
        sdp: false,
        width: 2,
        depth: 16384,
    },
    Shape {
        big: true,
        sdp: false,
        width: 4,
        depth: 8192,
    },
    Shape {
        big: true,
        sdp: false,
        width: 9,
        depth: 4096,
    },
    Shape {
        big: true,
        sdp: false,
        width: 18,
        depth: 2048,
    },
    Shape {
        big: true,
        sdp: false,
        width: 36,
        depth: 1024,
    },
    Shape {
        big: true,
        sdp: true,
        width: 72,
        depth: 512,
    },
];

impl Shape {
    fn parity(&self) -> bool {
        self.width % 9 == 0
    }
    /// Data / parity bits per entry.
    fn dw(&self) -> usize {
        if self.parity() { self.width / 9 * 8 } else { self.width }
    }
    fn pw(&self) -> usize {
        if self.parity() { self.width / 9 } else { 0 }
    }
    /// Lowest address bit used.
    fn shift(&self) -> usize {
        match self.width {
            1 => 0,
            2 => 1,
            4 => 2,
            9 => 3,
            18 => 4,
            36 => 5,
            _ => 6,
        }
    }
    /// (data index, parity index) of bit `j` of an entry.
    fn place(&self, j: usize) -> (Option<usize>, Option<usize>) {
        if self.parity() {
            let (byte, k) = (j / 9, j % 9);
            if k < 8 { (Some(byte * 8 + k), None) } else { (None, Some(byte)) }
        } else {
            (Some(j), None)
        }
    }
}

#[derive(Debug, Default, Clone)]
pub struct BramStats {
    pub ramb18: usize,
    pub ramb36: usize,
    pub ram32m: usize,
    pub ram64m: usize,
    pub ram128x1d: usize,
    pub mapped: usize,
    /// Copies made of multi-read memories (one per read port past the first).
    pub copies: usize,
    /// Write ports gated off past the memory's end.
    pub guarded: usize,
    /// LUTs for the guards and the bank selects.
    pub luts: usize,
    /// Memories left as `$mem_v2`, with the reason.
    pub left: Vec<(String, &'static str)>,
}

fn pick(width: usize, depth: usize) -> Option<(Shape, usize)> {
    SHAPES
        .iter()
        .filter(|s| s.depth >= depth)
        .map(|s| (*s, width.div_ceil(s.width)))
        .min_by_key(|(s, tiles)| (tiles * if s.big { 2 } else { 1 }, *tiles))
}

fn konst(v: u64, w: usize) -> SigSpec {
    SigSpec::constant(v, w as u32)
}

fn rep(b: Bit, w: usize) -> SigSpec {
    SigSpec(vec![b; w])
}

/// Why `mem` can't be mapped, if it can't.
fn eligible(mem: &Memory) -> Result<(), &'static str> {
    if mem.reads.len() != 1 {
        return Err("read ports != 1");
    }
    if mem.reads[0].clk.is_none() {
        return Err("asynchronous read");
    }
    if mem.writes.len() > 1 {
        return Err("write ports > 1");
    }
    if mem.offset != 0 {
        return Err("nonzero offset");
    }
    if let Some(w) = mem.writes.first() {
        if w.en.0.iter().any(|b| *b != w.en.0[0]) {
            return Err("per-bit write enables");
        }
    }
    Ok(())
}

/// A LUT computing `f` (of the inputs as an integer, `ins[0]` the low bit).
fn lut(m: &mut Module, out: &mut Vec<Cell>, st: &mut BramStats, ins: &[Bit], f: impl Fn(usize) -> bool) -> Bit {
    let o = m.wire(1).0[0];
    lut_to(out, st, ins, f, o);
    o
}

/// `lut` driving `o`.
fn lut_to(out: &mut Vec<Cell>, st: &mut BramStats, ins: &[Bit], f: impl Fn(usize) -> bool, o: Bit) {
    let mut conns = BTreeMap::new();
    for (i, b) in ins.iter().enumerate() {
        conns.insert(format!("I{i}"), SigSpec(vec![*b]));
    }
    conns.insert("O".to_string(), SigSpec(vec![o]));
    let init: Vec<Option<bool>> = (0..1usize << ins.len()).map(|x| Some(f(x))).collect();
    out.push(Cell::Instance {
        ty: format!("LUT{}", ins.len()),
        name: format!("oxsyn_memlut{}", st.luts),
        params: BTreeMap::from([("INIT".to_string(), Param::Bits(init))]),
        conns,
    });
    st.luts += 1;
}

/// Whether any of `bits` is set, as a tree of 6-input LUTs.
fn any(m: &mut Module, out: &mut Vec<Cell>, st: &mut BramStats, bits: &[Bit]) -> Bit {
    let mut level: Vec<Bit> = bits.iter().copied().filter(|b| *b != Bit::Zero).collect();
    if level.contains(&Bit::One) {
        return Bit::One;
    }
    if level.is_empty() {
        return Bit::Zero;
    }
    while level.len() > 1 {
        level = level.chunks(6).map(|c| if c.len() == 1 { c[0] } else { lut(m, out, st, c, |x| x != 0) }).collect();
    }
    level[0]
}

/// Address bits a memory of `size` words needs.
fn abits_for(size: u32) -> usize {
    (u32::BITS - size.max(1).saturating_sub(1).leading_zeros()) as usize
}

/// `mem` in shapes the primitives take: address cut to the size (writes
/// past the end gated off), one copy per read port when it has one write
/// port or none. `Err` (with `mem` unchanged) for a memory with a nonzero
/// offset or per-bit write enables, which the mapping reports.
fn normalize(m: &mut Module, out: &mut Vec<Cell>, st: &mut BramStats, mem: &Memory) -> Vec<Memory> {
    if mem.offset != 0 || mem.writes.iter().any(|w| w.en.0.iter().any(|b| *b != w.en.0[0])) {
        return vec![mem.clone()];
    }
    let k = abits_for(mem.size);
    let mut mem = mem.clone();
    for w in &mut mem.writes {
        if w.addr.width() as usize > k {
            let past = any(m, out, st, &w.addr.0[k..]);
            if past != Bit::Zero {
                let en = lut(m, out, st, &[w.en.0[0], past], |x| x == 0b01);
                w.en = SigSpec(vec![en; w.en.0.len()]);
                st.guarded += 1;
            }
            w.addr = SigSpec(w.addr.0[..k].to_vec());
        }
    }
    for r in &mut mem.reads {
        if r.addr.width() as usize > k {
            r.addr = SigSpec(r.addr.0[..k].to_vec());
        }
    }
    if mem.reads.len() > 1 && mem.writes.len() <= 1 {
        st.copies += mem.reads.len() - 1;
        return mem
            .reads
            .iter()
            .enumerate()
            .map(|(i, r)| Memory {
                name: format!("{}$rd{i}", mem.name),
                reads: vec![r.clone()],
                ..mem.clone()
            })
            .collect();
    }
    vec![mem]
}

/// Map one memory (already normalized), its cells named after `tag`.
fn map_one(m: &mut Module, out: &mut Vec<Cell>, st: &mut BramStats, mem: &Memory, tag: &str) -> Result<(), &'static str> {
    if mem.reads.len() == 1 && mem.reads[0].clk.is_none() {
        let cells = lutram(m, mem, tag, st).or_else(|e| if e == "asynchronous read deeper than 128" { lutram_banked(m, out, mem, tag, st) } else { Err(e) })?;
        out.extend(cells);
        return Ok(());
    }
    eligible(mem)?;
    // Every address the ports can present must land inside the shape,
    // so size by the address width too.
    let aw = mem.reads[0].addr.width().max(mem.writes.first().map_or(0, |w| w.addr.width()));
    let need = (mem.size as usize).max(1usize << aw.min(20));
    let (shape, tiles) = pick(mem.width as usize, need).ok_or("too deep")?;
    for t in 0..tiles {
        out.push(tile(m, mem, shape, t, tag));
        if shape.big {
            st.ramb36 += 1;
        } else {
            st.ramb18 += 1;
        }
    }
    Ok(())
}

#[hot]
pub fn map(m: &mut Module) -> BramStats {
    let mut st = BramStats::default();
    let cells = std::mem::take(&mut m.cells);
    let mut out = Vec::with_capacity(cells.len());
    for (mi, cell) in cells.into_iter().enumerate() {
        let Cell::Mem(mem) = &cell else {
            out.push(cell);
            continue;
        };
        // Map into a scratch list, so a memory that can't be mapped whole
        // leaves nothing half-built behind.
        let mut cells = Vec::new();
        let mut scratch = st.clone();
        let parts = normalize(m, &mut cells, &mut scratch, mem);
        let several = parts.len() > 1;
        let mut res = Ok(());
        for (i, part) in parts.iter().enumerate() {
            let tag = if several { format!("{mi}r{i}") } else { mi.to_string() };
            res = res.and_then(|_| map_one(m, &mut cells, &mut scratch, part, &tag));
        }
        match res {
            Ok(()) => {
                st = scratch;
                st.mapped += 1;
                out.extend(cells);
            }
            Err(why) => {
                st.left.push((mem.name.clone(), why));
                out.push(cell);
            }
        }
    }
    m.cells = out;
    st
}

/// An asynchronous-read memory with at most one write port, 129 to 512
/// deep: per data bit, one RAM128X1D per 128 words, the write enable of
/// each gated by the high write address bits, the read picked by the high
/// read address bits.
fn lutram_banked(m: &mut Module, out: &mut Vec<Cell>, mem: &Memory, tag: &str, st: &mut BramStats) -> Result<Vec<Cell>, &'static str> {
    if mem.writes.len() > 1 {
        return Err("write ports > 1");
    }
    let rd = &mem.reads[0];
    let wr = mem.writes.first();
    let aw = rd.addr.width().max(wr.map_or(0, |w| w.addr.width())) as usize;
    let banks = 1usize << aw.saturating_sub(7);
    if aw <= 7 || banks > 4 {
        return Err("asynchronous read deeper than 512");
    }
    let bit = |a: &SigSpec, i: usize| a.0.get(i).copied().unwrap_or(Bit::Zero);
    let low = |a: &SigSpec| SigSpec((0..7).map(|i| bit(a, i)).collect());
    let hi = |a: &SigSpec| -> Vec<Bit> { (7..aw).map(|i| bit(a, i)).collect() };
    let (wclk, wpos) = wr.map(|w| w.clk.clone()).unwrap_or((konst(0, 1), true));
    let we = wr.map(|w| w.en.0[0]).unwrap_or(Bit::Zero);
    let waddr = wr.map(|w| w.addr.clone()).unwrap_or(konst(0, aw));
    let wes: Vec<Bit> = (0..banks)
        .map(|b| {
            let mut ins = vec![we];
            ins.extend(hi(&waddr));
            lut(m, out, st, &ins, |x| x & 1 == 1 && x >> 1 == b)
        })
        .collect();
    let width = mem.width as usize;
    let mut cells = Vec::new();
    for i in 0..width {
        let mut outs = Vec::new();
        for (b, web) in wes.iter().enumerate() {
            let dpo = m.wire(1).0[0];
            let mut conns = BTreeMap::new();
            conns.insert("WCLK".to_string(), wclk.clone());
            conns.insert("WE".to_string(), SigSpec(vec![*web]));
            conns.insert("A".to_string(), low(&waddr));
            conns.insert("DPRA".to_string(), low(&rd.addr));
            conns.insert("D".to_string(), SigSpec(vec![wr.map(|w| w.data.0[i]).unwrap_or(Bit::Zero)]));
            conns.insert("DPO".to_string(), SigSpec(vec![dpo]));
            conns.insert("SPO".to_string(), m.wire(1));
            let mut params = BTreeMap::new();
            let init = (0..128).map(|a| Some(mem.init.get(&((b * 128 + a) as u32)).and_then(|w| w.get(i).copied().flatten()) == Some(true))).collect();
            params.insert("INIT".to_string(), Param::Bits(init));
            if !wpos {
                params.insert("IS_WCLK_INVERTED".to_string(), Param::Bits(vec![Some(true)]));
            }
            cells.push(Cell::Instance {
                ty: "RAM128X1D".into(),
                name: format!("oxsyn_lutram{tag}_{i}_{b}"),
                params,
                conns,
            });
            st.ram128x1d += 1;
            outs.push(dpo);
        }
        // Read select: the bank outputs as the low inputs, the high read
        // address bits (the bank number) above them.
        let mut ins = outs;
        ins.extend(hi(&rd.addr));
        lut_to(&mut cells, st, &ins, |x| (x >> (x >> banks)) & 1 == 1, rd.data.0[i]);
    }
    Ok(cells)
}

/// An asynchronous-read memory with at most one write port, up to 128
/// deep, as distributed RAM (models: yosys's xilinx cells_sim.v):
/// RAM32M (6 bits each: ports A/B/C read at the read address, port D
/// carries the write address), RAM64M (3 bits each), RAM128X1D (1 bit).
fn lutram(m: &mut Module, mem: &Memory, tag: &str, st: &mut BramStats) -> Result<Vec<Cell>, &'static str> {
    if mem.writes.len() > 1 {
        return Err("write ports > 1");
    }
    if mem.offset != 0 {
        return Err("nonzero offset");
    }
    let wr = mem.writes.first();
    if let Some(w) = wr {
        if w.en.0.iter().any(|b| *b != w.en.0[0]) {
            return Err("per-bit write enables");
        }
    }
    let rd = &mem.reads[0];
    let aw = rd.addr.width().max(wr.map_or(0, |w| w.addr.width()));
    let need = (mem.size as usize).max(1usize << aw.min(20));
    let (ty, bits, abits) = match need {
        0..=32 => ("RAM32M", 6, 5),
        33..=64 => ("RAM64M", 3, 6),
        65..=128 => ("RAM128X1D", 1, 7),
        _ => return Err("asynchronous read deeper than 128"),
    };
    let ext = |a: &SigSpec| -> SigSpec {
        let mut v: Vec<Bit> = a.0.iter().take(abits).copied().collect();
        v.resize(abits, Bit::Zero);
        SigSpec(v)
    };
    let raddr = ext(&rd.addr);
    let waddr = wr.map(|w| ext(&w.addr)).unwrap_or(konst(0, abits));
    let (wclk, wpos) = wr.map(|w| w.clk.clone()).unwrap_or((konst(0, 1), true));
    let we = SigSpec(vec![wr.map(|w| w.en.0[0]).unwrap_or(Bit::Zero)]);
    let width = mem.width as usize;
    let wbit = |i: usize| -> Bit {
        if i < width {
            wr.map(|w| w.data.0[i]).unwrap_or(Bit::Zero)
        } else {
            Bit::Zero
        }
    };
    let init_bit = |a: usize, i: usize| -> bool { i < width && mem.init.get(&(a as u32)).and_then(|w| w.get(i).copied().flatten()) == Some(true) };
    let mut cells = Vec::new();
    for t in 0..width.div_ceil(bits) {
        let lo = t * bits;
        let rbit = |i: usize, m: &mut Module| -> Bit { if i < width { rd.data.0[i] } else { m.wire(1).0[0] } };
        let mut conns = BTreeMap::new();
        let mut params = BTreeMap::new();
        conns.insert("WCLK".to_string(), wclk.clone());
        conns.insert("WE".to_string(), we.clone());
        if !wpos {
            params.insert("IS_WCLK_INVERTED".to_string(), Param::Bits(vec![Some(true)]));
        }
        match ty {
            "RAM32M" | "RAM64M" => {
                let per = bits / 3;
                for (p, port) in ["A", "B", "C"].iter().enumerate() {
                    let b0 = lo + p * per;
                    conns.insert(format!("ADDR{port}"), raddr.clone());
                    conns.insert(format!("DI{port}"), SigSpec((0..per).map(|k| wbit(b0 + k)).collect()));
                    let o: Vec<Bit> = (0..per).map(|k| rbit(b0 + k, m)).collect();
                    conns.insert(format!("DO{port}"), SigSpec(o));
                    let init: Vec<Option<bool>> = (0..64).map(|i| Some(init_bit(i / per, b0 + i % per))).collect();
                    params.insert(format!("INIT_{port}"), Param::Bits(init));
                }
                conns.insert("ADDRD".to_string(), waddr.clone());
                conns.insert("DID".to_string(), konst(0, per));
                conns.insert("DOD".to_string(), m.wire(per as u32));
                params.insert("INIT_D".to_string(), Param::Bits(vec![Some(false); 64]));
                if ty == "RAM32M" {
                    st.ram32m += 1;
                } else {
                    st.ram64m += 1;
                }
            }
            _ => {
                conns.insert("A".to_string(), waddr.clone());
                conns.insert("DPRA".to_string(), raddr.clone());
                conns.insert("D".to_string(), SigSpec(vec![wbit(lo)]));
                conns.insert("DPO".to_string(), SigSpec(vec![rbit(lo, m)]));
                conns.insert("SPO".to_string(), m.wire(1));
                params.insert("INIT".to_string(), Param::Bits((0..128).map(|a| Some(init_bit(a, lo))).collect()));
                st.ram128x1d += 1;
            }
        }
        cells.push(Cell::Instance {
            ty: ty.into(),
            name: format!("oxsyn_lutram{tag}_{t}"),
            params,
            conns,
        });
    }
    Ok(cells)
}

/// One column of `mem`: bits `t * width ..` of every word.
fn tile(m: &mut Module, mem: &Memory, s: Shape, t: usize, tag: &str) -> Cell {
    let (abits, dbits, pbits, webits_a, webits_b) = if s.big { (16, 32, 4, 4, 8) } else { (14, 16, 2, 2, 4) };
    let rd = &mem.reads[0];
    let (rclk, rpos) = rd.clk.clone().expect("synchronous read");
    let lo = t * s.width;
    let hi = (lo + s.width).min(mem.width as usize);

    // Address bus: memory address at `shift` up, ADDR[15] high on RAMB36.
    let addr = |a: &SigSpec| -> SigSpec {
        let mut v = vec![Bit::Zero; abits];
        for (i, b) in a.0.iter().enumerate() {
            if s.shift() + i < 15.min(abits) {
                v[s.shift() + i] = *b;
            }
        }
        if s.big {
            v[15] = Bit::One;
        }
        SigSpec(v)
    };

    // Data buses, as (DI A, DIP A, DI B, DIP B); SDP splits a double-wide
    // word across both ports.
    let mut di = vec![Bit::Zero; 2 * dbits];
    let mut dip = vec![Bit::Zero; 2 * pbits];
    let mut dout: Vec<Option<Bit>> = vec![None; 2 * dbits];
    let mut doutp: Vec<Option<Bit>> = vec![None; 2 * pbits];
    let wr = mem.writes.first();
    for j in 0..hi - lo {
        let (d, p) = s.place(j);
        let wbit = wr.map(|w| w.data.0[lo + j]).unwrap_or(Bit::Zero);
        let rbit = rd.data.0[lo + j];
        if let Some(d) = d {
            di[d] = wbit;
            dout[d] = Some(rbit);
        }
        if let Some(p) = p {
            dip[p] = wbit;
            doutp[p] = Some(rbit);
        }
    }
    let fill = |v: Vec<Option<Bit>>, m: &mut Module| -> SigSpec { SigSpec(v.into_iter().map(|b| b.unwrap_or_else(|| m.wire(1).0[0])).collect()) };
    let dout = fill(dout, m);
    let doutp = fill(doutp, m);
    let half = |v: &SigSpec, w: usize, k: usize| SigSpec(v.0[k * w..(k + 1) * w].to_vec());

    let mut conns = BTreeMap::new();
    let mut params = BTreeMap::new();
    let one = Bit::One;
    let en_w = wr.map(|w| w.en.0[0]).unwrap_or(Bit::Zero);
    let (wclk, wpos) = wr.map(|w| w.clk.clone()).unwrap_or((konst(0, 1), true));
    let (waddr, raddr) = (wr.map(|w| addr(&w.addr)).unwrap_or(konst(0, abits)), addr(&rd.addr));
    let di = SigSpec(di);
    let dip = SigSpec(dip);
    if s.sdp {
        conns.insert("CLKARDCLK".to_string(), rclk.clone());
        conns.insert("CLKBWRCLK".to_string(), wclk.clone());
        conns.insert("ADDRARDADDR".to_string(), raddr);
        conns.insert("ADDRBWRADDR".to_string(), waddr);
        conns.insert("DIADI".to_string(), half(&di, dbits, 0));
        conns.insert("DIBDI".to_string(), half(&di, dbits, 1));
        conns.insert("DIPADIP".to_string(), half(&dip, pbits, 0));
        conns.insert("DIPBDIP".to_string(), half(&dip, pbits, 1));
        conns.insert("DOADO".to_string(), half(&dout, dbits, 0));
        conns.insert("DOBDO".to_string(), half(&dout, dbits, 1));
        conns.insert("DOPADOP".to_string(), half(&doutp, pbits, 0));
        conns.insert("DOPBDOP".to_string(), half(&doutp, pbits, 1));
        conns.insert("WEA".to_string(), konst(0, webits_a));
        conns.insert("WEBWE".to_string(), rep(en_w, webits_b));
        params.insert("READ_WIDTH_A".to_string(), Param::Int(s.width as i64));
        params.insert("READ_WIDTH_B".to_string(), Param::Int(0));
        params.insert("WRITE_WIDTH_A".to_string(), Param::Int(0));
        params.insert("WRITE_WIDTH_B".to_string(), Param::Int(s.width as i64));
        if !rpos {
            params.insert("IS_CLKARDCLK_INVERTED".into(), Param::Bits(vec![Some(true)]));
        }
        if !wpos {
            params.insert("IS_CLKBWRCLK_INVERTED".into(), Param::Bits(vec![Some(true)]));
        }
    } else {
        conns.insert("CLKARDCLK".to_string(), wclk.clone());
        conns.insert("CLKBWRCLK".to_string(), rclk.clone());
        conns.insert("ADDRARDADDR".to_string(), waddr);
        conns.insert("ADDRBWRADDR".to_string(), raddr);
        conns.insert("DIADI".to_string(), half(&di, dbits, 0));
        conns.insert("DIPADIP".to_string(), half(&dip, pbits, 0));
        conns.insert("DIBDI".to_string(), konst(0, dbits));
        conns.insert("DIPBDIP".to_string(), konst(0, pbits));
        conns.insert("DOBDO".to_string(), half(&dout, dbits, 0));
        conns.insert("DOPBDOP".to_string(), half(&doutp, pbits, 0));
        let dummy = m.wire((dbits + pbits) as u32);
        conns.insert("DOADO".to_string(), SigSpec(dummy.0[..dbits].to_vec()));
        conns.insert("DOPADOP".to_string(), SigSpec(dummy.0[dbits..].to_vec()));
        conns.insert("WEA".to_string(), rep(en_w, webits_a));
        conns.insert("WEBWE".to_string(), konst(0, webits_b));
        params.insert("READ_WIDTH_A".to_string(), Param::Int(0));
        params.insert("READ_WIDTH_B".to_string(), Param::Int(s.width as i64));
        params.insert("WRITE_WIDTH_A".to_string(), Param::Int(s.width as i64));
        params.insert("WRITE_WIDTH_B".to_string(), Param::Int(0));
        if !wpos {
            params.insert("IS_CLKARDCLK_INVERTED".into(), Param::Bits(vec![Some(true)]));
        }
        if !rpos {
            params.insert("IS_CLKBWRCLK_INVERTED".into(), Param::Bits(vec![Some(true)]));
        }
    }
    for k in ["ENARDEN", "ENBWREN"] {
        conns.insert(k.to_string(), SigSpec(vec![one]));
    }
    for k in ["REGCEAREGCE", "REGCEB", "RSTRAMARSTRAM", "RSTRAMB", "RSTREGARSTREG", "RSTREGB"] {
        conns.insert(k.to_string(), konst(0, 1));
    }
    params.insert("RAM_MODE".to_string(), Param::Str(if s.sdp { "SDP" } else { "TDP" }.into()));
    params.insert("WRITE_MODE_A".to_string(), Param::Str("READ_FIRST".into()));
    params.insert("WRITE_MODE_B".to_string(), Param::Str("READ_FIRST".into()));
    params.insert("DOA_REG".to_string(), Param::Int(0));
    params.insert("DOB_REG".to_string(), Param::Int(0));
    if s.big {
        params.insert("RAM_EXTENSION_A".to_string(), Param::Str("NONE".into()));
        params.insert("RAM_EXTENSION_B".to_string(), Param::Str("NONE".into()));
    }
    let ow = if s.big { 36 } else { 18 };
    for k in ["INIT_A", "INIT_B", "SRVAL_A", "SRVAL_B"] {
        params.insert(k.to_string(), Param::Bits(vec![Some(false); ow]));
    }

    // Initial contents: entry a's data bits at a * dw + d, parity at a * pw + p.
    let (ninit, ninitp) = if s.big { (128, 16) } else { (64, 8) };
    let mut data = vec![false; ninit * 256];
    let mut par = vec![false; ninitp * 256];
    for (&a, word) in &mem.init {
        let a = a as usize;
        if a >= s.depth {
            continue;
        }
        for j in 0..hi - lo {
            if word.get(lo + j).copied().flatten() != Some(true) {
                continue;
            }
            match s.place(j) {
                (Some(d), _) => data[a * s.dw() + d] = true,
                (_, Some(p)) => par[a * s.pw() + p] = true,
                _ => {}
            }
        }
    }
    for (i, chunk) in data.chunks(256).enumerate() {
        params.insert(format!("INIT_{i:02X}"), Param::Bits(chunk.iter().map(|b| Some(*b)).collect()));
    }
    for (i, chunk) in par.chunks(256).enumerate() {
        params.insert(format!("INITP_{i:02X}"), Param::Bits(chunk.iter().map(|b| Some(*b)).collect()));
    }
    let ty = if s.big { "RAMB36E1" } else { "RAMB18E1" };
    Cell::Instance {
        ty: ty.into(),
        name: format!("oxsyn_bram{tag}_{t}"),
        params,
        conns,
    }
}

pub struct Bram;

impl crate::stage::Pass for Bram {
    const NAME: &'static str = "bram";
    const EMITS: &'static [crate::caps::Mode] = &[
        crate::caps::Mode::Ramb18,
        crate::caps::Mode::Ramb36,
        crate::caps::Mode::Ram32m,
        crate::caps::Mode::Ram64m,
        crate::caps::Mode::Ram128x1d,
    ];
    type Stats = BramStats;
    fn enabled(p: &crate::policy::PassSet) -> bool {
        p.bram
    }
    fn json(s: &BramStats) -> serde_json::Value {
        serde_json::json!({ "ramb18": s.ramb18, "ramb36": s.ramb36, "ram32m": s.ram32m, "ram64m": s.ram64m,
                            "ram128x1d": s.ram128x1d, "mapped": s.mapped, "copies": s.copies, "guarded": s.guarded,
                            "luts": s.luts, "left": s.left.len() })
    }
}

impl crate::stage::PhysicalPass for Bram {
    const EVIDENCE: crate::policy::PostCheckEvidence = crate::policy::PostCheckEvidence::CoSimulation {
        harness: "models/cosim.sh over examples/memcases (block RAM on models/ramb.v, LUTRAM on cells_sim)",
    };
    fn run(&self, m: &mut crate::stage::Physical) -> BramStats {
        map(&mut m.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shapes() {
        // yosys picks one RAMB36 x36 for 1024 x 26.
        let (s, n) = pick(26, 1024).unwrap();
        assert_eq!((s.big, s.sdp, s.width, n), (true, false, 36, 1));
        // 512 deep x64: one RAMB36 SDP x72.
        let (s, n) = pick(64, 512).unwrap();
        assert_eq!((s.big, s.sdp, s.width, n), (true, true, 72, 1));
        // 128 x 26: one RAMB18 SDP x36.
        let (s, n) = pick(26, 128).unwrap();
        assert_eq!((s.big, s.sdp, s.width, n), (false, true, 36, 1));
        // 9-bit packing: bit 8 is parity 0, bit 9 data 8.
        let s = Shape {
            big: true,
            sdp: false,
            width: 36,
            depth: 1024,
        };
        assert_eq!(s.place(8), (None, Some(0)));
        assert_eq!(s.place(9), (Some(8), None));
        assert!(pick(1, 65536).is_none());
    }

    use oxibit_hdl::netlist::{MemRead, MemWrite};

    /// A `size` x `width` memory, async reads at `aw`-bit addresses, one
    /// write port.
    fn memory(m: &mut Module, size: u32, width: u32, aw: u32, reads: usize) -> Memory {
        let clk = m.wire(1);
        let we = m.wire(1).0[0];
        let reads = (0..reads)
            .map(|_| MemRead {
                clk: None,
                addr: m.wire(aw),
                data: m.wire(width),
            })
            .collect();
        Memory {
            name: "mm".into(),
            width,
            size,
            offset: 0,
            init: BTreeMap::new(),
            attrs: BTreeMap::new(),
            reads,
            writes: vec![MemWrite {
                clk: (clk, true),
                addr: m.wire(aw),
                data: m.wire(width),
                en: SigSpec(vec![we; width as usize]),
            }],
        }
    }

    fn count(m: &Module, ty: &str) -> usize {
        m.cells.iter().filter(|c| matches!(c, Cell::Instance { ty: t, .. } if t == ty)).count()
    }

    #[test]
    fn read_ports_become_copies() {
        let mut m = Module::new("t");
        let mem = memory(&mut m, 32, 4, 5, 3);
        m.cells.push(Cell::Mem(mem));
        let st = map(&mut m);
        assert!(st.left.is_empty(), "{:?}", st.left);
        assert_eq!((st.copies, st.ram32m), (2, 3));
        assert_eq!(count(&m, "RAM32M"), 3);
    }

    #[test]
    fn deep_async_banks_and_guards() {
        // 512 deep behind a 10-bit address: the top bit is past the end
        // (one guard LUT); four banks of RAM128X1D per bit, a write-enable
        // LUT per bank and a select LUT per bit.
        let mut m = Module::new("t");
        let mem = memory(&mut m, 512, 2, 10, 1);
        m.cells.push(Cell::Mem(mem));
        let st = map(&mut m);
        assert!(st.left.is_empty(), "{:?}", st.left);
        assert_eq!((st.guarded, st.ram128x1d), (1, 8));
        assert_eq!(st.luts, 1 + 4 + 2);
        assert_eq!(count(&m, "RAM128X1D"), 8);
        assert!(!m.cells.iter().any(|c| matches!(c, Cell::Mem(_))));
    }

    #[test]
    fn bank_select_truth_table() {
        // Four bank outputs on I0..I3, bank number on I4..I5: O = I[bank].
        let banks = 4;
        let f = |x: usize| (x >> (x >> banks)) & 1 == 1;
        for bank in 0..4 {
            for outs in 0..16usize {
                let x = outs | bank << banks;
                assert_eq!(f(x), (outs >> bank) & 1 == 1);
            }
        }
    }
}
