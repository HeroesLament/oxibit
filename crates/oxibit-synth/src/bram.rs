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
//! Asynchronous-read memories up to 512 deep become distributed RAM
//! (`lutram`; banked RAM128X1D above 128).
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
        if mem.reads.len() == 1 && mem.reads[0].clk.is_none() {
            match lutram(m, mem, mi, &mut st) {
                Ok(cells) => {
                    out.extend(cells);
                    st.mapped += 1;
                }
                Err(why) => {
                    st.left.push((mem.name.clone(), why));
                    out.push(cell);
                }
            }
            continue;
        }
        if let Err(why) = eligible(mem) {
            st.left.push((mem.name.clone(), why));
            out.push(cell);
            continue;
        }
        // Every address the ports can present must land inside the shape,
        // so size by the address width too.
        let aw = mem.reads[0].addr.width().max(mem.writes.first().map_or(0, |w| w.addr.width()));
        let need = (mem.size as usize).max(1usize << aw.min(20));
        let Some((shape, tiles)) = pick(mem.width as usize, need) else {
            st.left.push((mem.name.clone(), "too deep"));
            out.push(cell);
            continue;
        };
        for t in 0..tiles {
            out.push(tile(m, mem, shape, t, mi));
            if shape.big {
                st.ramb36 += 1;
            } else {
                st.ramb18 += 1;
            }
        }
        st.mapped += 1;
    }
    m.cells = out;
    st
}

/// An asynchronous-read memory with at most one write port as distributed
/// RAM (models: yosys's xilinx cells_sim.v): RAM32M (6 bits each: ports
/// A/B/C read at the read address, port D carries the write address),
/// RAM64M (3 bits each), RAM128X1D (1 bit). Deeper than 128, up to 512:
/// two or four banks of RAM128X1D per bit, each bank's WE a LUT of the
/// enable and the high write-address bits, the read a LUT mux of the
/// banks' DPO on the high read-address bits.
///
/// Bits are grouped into lanes by write-enable bit (per-bit enables, as
/// EHDL emits for a partial-word write); each lane gets its own columns
/// with its enable on WE.
///
/// The RAM is sized by `size`, not by the address width: a read past
/// `size` is X in `$mem_v2`, so its high address bits are ignored. A
/// write past `size` must write nothing, so when the write address has
/// bits above the word index (other than literal zeros) WE is gated by a
/// NOR of them.
fn lutram(m: &mut Module, mem: &Memory, mi: usize, st: &mut BramStats) -> Result<Vec<Cell>, &'static str> {
    if mem.writes.len() > 1 {
        return Err("write ports > 1");
    }
    if mem.offset != 0 {
        return Err("nonzero offset");
    }
    let wr = mem.writes.first();
    let rd = &mem.reads[0];
    let need = (mem.size as usize).max(1).next_power_of_two();
    let (ty, bits, abits, banks) = match need {
        0..=32 => ("RAM32M", 6, 5, 1usize),
        33..=64 => ("RAM64M", 3, 6, 1),
        65..=128 => ("RAM128X1D", 1, 7, 1),
        129..=256 => ("RAM128X1D", 1, 7, 2),
        257..=512 => ("RAM128X1D", 1, 7, 4),
        _ => return Err("asynchronous read deeper than 512"),
    };
    let bank_bits = banks.trailing_zeros() as usize;
    // Address bits lo..lo+n, zero past the port's width.
    let ext = |a: &SigSpec, lo: usize, n: usize| -> SigSpec {
        SigSpec((lo..lo + n).map(|i| a.0.get(i).copied().unwrap_or(Bit::Zero)).collect())
    };
    let raddr = ext(&rd.addr, 0, abits);
    let rsel = ext(&rd.addr, abits, bank_bits);
    let waddr = wr.map(|w| ext(&w.addr, 0, abits)).unwrap_or(konst(0, abits));
    let wsel = wr.map(|w| ext(&w.addr, abits, bank_bits)).unwrap_or(konst(0, bank_bits));
    let (wclk, wpos) = wr.map(|w| w.clk.clone()).unwrap_or((konst(0, 1), true));
    let width = mem.width as usize;
    let mut cells = Vec::new();

    // Write address bits above the word index: the write lands only when
    // all are zero.
    let span = abits + bank_bits;
    let whigh: Vec<Bit> = wr
        .map(|w| w.addr.0.iter().skip(span).copied().filter(|b| *b != Bit::Zero).collect())
        .unwrap_or_default();
    let w_ok: Option<Bit> = if whigh.is_empty() {
        None
    } else {
        Some(nor_tree(m, &mut cells, &whigh, &format!("oxsyn_lutram{mi}_wok")))
    };

    // Lanes: bit indices sharing one write-enable bit, in first-seen order.
    let mut lanes: Vec<(Bit, Vec<usize>)> = Vec::new();
    for i in 0..width {
        let en = wr.map(|w| w.en.0[i]).unwrap_or(Bit::Zero);
        match lanes.iter_mut().find(|(e, _)| *e == en) {
            Some((_, v)) => v.push(i),
            None => lanes.push((en, vec![i])),
        }
    }
    let wbit = |i: Option<usize>| -> Bit {
        match i {
            Some(i) => wr.map(|w| w.data.0[i]).unwrap_or(Bit::Zero),
            None => Bit::Zero,
        }
    };
    let init_bit = |a: usize, i: Option<usize>| -> bool {
        i.is_some_and(|i| mem.init.get(&(a as u32)).and_then(|w| w.get(i).copied().flatten()) == Some(true))
    };

    let mut col = 0usize;
    for (en, idxs) in &lanes {
        // Per lane and bank, the write enable: the lane's enable when the
        // high write-address bits select the bank.
        let bank_we: Vec<SigSpec> = (0..banks)
            .map(|b| {
                if banks == 1 && w_ok.is_none() {
                    return SigSpec(vec![*en]);
                }
                let o = m.wire(1);
                // inputs: enable, [in range], bank select bits
                let mut ins = vec![*en];
                ins.extend(w_ok);
                let sel_at = ins.len();
                ins.extend(wsel.0.iter().copied());
                let tt: Vec<Option<bool>> = (0..1usize << ins.len())
                    .map(|p| Some(p & ((1 << sel_at) - 1) == (1 << sel_at) - 1 && (p >> sel_at) == b))
                    .collect();
                cells.push(lut_cell(format!("oxsyn_lutram{mi}_we{col}_{b}"), &ins, tt, o.0[0]));
                o
            })
            .collect();
        for chunk in idxs.chunks(bits) {
            let at = |k: usize| chunk.get(k).copied();
            let mut rbit = |k: usize, m: &mut Module| -> Bit {
                match at(k) {
                    Some(i) => rd.data.0[i],
                    None => m.wire(1).0[0],
                }
            };
            match ty {
                "RAM32M" | "RAM64M" => {
                    let mut conns = BTreeMap::new();
                    let mut params = BTreeMap::new();
                    conns.insert("WCLK".to_string(), wclk.clone());
                    conns.insert("WE".to_string(), bank_we[0].clone());
                    if !wpos {
                        params.insert("IS_WCLK_INVERTED".to_string(), Param::Bits(vec![Some(true)]));
                    }
                    let per = bits / 3;
                    for (p, port) in ["A", "B", "C"].iter().enumerate() {
                        let k0 = p * per;
                        conns.insert(format!("ADDR{port}"), raddr.clone());
                        conns.insert(format!("DI{port}"), SigSpec((0..per).map(|k| wbit(at(k0 + k))).collect()));
                        let o: Vec<Bit> = (0..per).map(|k| rbit(k0 + k, m)).collect();
                        conns.insert(format!("DO{port}"), SigSpec(o));
                        let init: Vec<Option<bool>> = (0..64).map(|i| Some(init_bit(i / per, at(k0 + i % per)))).collect();
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
                    cells.push(Cell::Instance {
                        ty: ty.into(),
                        name: format!("oxsyn_lutram{mi}_{col}"),
                        params,
                        conns,
                    });
                }
                _ => {
                    // One bit, one RAM128X1D per bank, then the bank mux.
                    let dpo: Vec<Bit> = (0..banks).map(|_| m.wire(1).0[0]).collect();
                    for (b, we) in bank_we.iter().enumerate() {
                        let mut conns = BTreeMap::new();
                        let mut params = BTreeMap::new();
                        conns.insert("WCLK".to_string(), wclk.clone());
                        conns.insert("WE".to_string(), we.clone());
                        if !wpos {
                            params.insert("IS_WCLK_INVERTED".to_string(), Param::Bits(vec![Some(true)]));
                        }
                        conns.insert("A".to_string(), waddr.clone());
                        conns.insert("DPRA".to_string(), raddr.clone());
                        conns.insert("D".to_string(), SigSpec(vec![wbit(at(0))]));
                        conns.insert("DPO".to_string(), SigSpec(vec![dpo[b]]));
                        conns.insert("SPO".to_string(), m.wire(1));
                        params.insert(
                            "INIT".to_string(),
                            Param::Bits((0..128).map(|a| Some(init_bit(b * 128 + a, at(0)))).collect()),
                        );
                        st.ram128x1d += 1;
                        cells.push(Cell::Instance {
                            ty: ty.into(),
                            name: format!("oxsyn_lutram{mi}_{col}_{b}"),
                            params,
                            conns,
                        });
                    }
                    let out = rbit(0, m);
                    if banks == 1 {
                        m.connects.push((SigSpec(vec![out]), SigSpec(vec![dpo[0]])));
                    } else {
                        // inputs: the banks' DPO, then the select bits
                        let mut ins = dpo.clone();
                        ins.extend(rsel.0.iter().copied());
                        let tt: Vec<Option<bool>> = (0..1usize << ins.len())
                            .map(|p| Some((p >> (p >> banks)) & 1 == 1))
                            .collect();
                        cells.push(lut_cell(format!("oxsyn_lutram{mi}_rd{col}"), &ins, tt, out));
                    }
                }
            }
            col += 1;
        }
    }
    Ok(cells)
}

/// One bit that is 1 when every bit of `v` is 0: NOR LUTs over groups of
/// six, then AND LUTs over the group results until one is left.
fn nor_tree(m: &mut Module, cells: &mut Vec<Cell>, v: &[Bit], name: &str) -> Bit {
    let mut level: Vec<Bit> = Vec::new();
    for (g, grp) in v.chunks(6).enumerate() {
        let o = m.wire(1).0[0];
        let tt = (0..1usize << grp.len()).map(|p| Some(p == 0)).collect();
        cells.push(lut_cell(format!("{name}_n{g}"), grp, tt, o));
        level.push(o);
    }
    let mut round = 0;
    while level.len() > 1 {
        let mut up = Vec::new();
        for (g, grp) in level.chunks(6).enumerate() {
            let o = m.wire(1).0[0];
            let all = (1usize << grp.len()) - 1;
            let tt = (0..1usize << grp.len()).map(|p| Some(p == all)).collect();
            cells.push(lut_cell(format!("{name}_a{round}_{g}"), grp, tt, o));
            up.push(o);
        }
        level = up;
        round += 1;
    }
    level[0]
}

/// A LUTk instance (`k` = inputs) with an LSB-first truth table.
fn lut_cell(name: String, ins: &[Bit], tt: Vec<Option<bool>>, o: Bit) -> Cell {
    let mut conns = BTreeMap::new();
    for (i, b) in ins.iter().enumerate() {
        conns.insert(format!("I{i}"), SigSpec(vec![*b]));
    }
    conns.insert("O".to_string(), SigSpec(vec![o]));
    Cell::Instance {
        ty: format!("LUT{}", ins.len()),
        name,
        params: BTreeMap::from([("INIT".to_string(), Param::Bits(tt))]),
        conns,
    }
}

/// One column of `mem`: bits `t * width ..` of every word.
fn tile(m: &mut Module, mem: &Memory, s: Shape, t: usize, mi: usize) -> Cell {
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
        name: format!("oxsyn_bram{mi}_{t}"),
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
                            "ram128x1d": s.ram128x1d, "mapped": s.mapped, "left": s.left.len() })
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
}

#[cfg(test)]
mod pb_lutram_tests {
    use super::*;
    use oxibit_hdl::netlist::{MemRead, MemWrite};

    fn amem(m: &mut Module, size: u32, aw: u32, en: Vec<Bit>) -> Memory {
        let width = en.len() as u32;
        let clk = m.wire(1);
        Memory {
            name: "m".into(),
            width,
            size,
            offset: 0,
            init: Default::default(),
            attrs: Default::default(),
            writes: vec![MemWrite {
                clk: (clk, true),
                addr: m.wire(aw),
                data: m.wire(width),
                en: SigSpec(en),
            }],
            reads: vec![MemRead {
                clk: None,
                addr: m.wire(aw),
                data: m.wire(width),
            }],
        }
    }

    fn count(cells: &[Cell], ty: &str) -> usize {
        cells.iter().filter(|c| matches!(c, Cell::Instance { ty: t, .. } if t == ty)).count()
    }

    #[test]
    fn deep_async_read_is_banked() {
        let mut m = Module::new("t");
        let e = m.wire(1).0[0];
        let x = amem(&mut m, 512, 9, vec![e, e]);
        let mut st = BramStats::default();
        let cells = lutram(&mut m, &x, 0, &mut st).unwrap();
        // 2 bits x 4 banks; one WE decode per bank (one lane); one read mux per bit
        assert_eq!(count(&cells, "RAM128X1D"), 8);
        assert_eq!(count(&cells, "LUT3"), 4);
        assert_eq!(count(&cells, "LUT6"), 2);
    }

    #[test]
    fn eight_bit_address_on_128_words_is_one_bank_with_a_gate() {
        let mut m = Module::new("t");
        let e = m.wire(1).0[0];
        let x = amem(&mut m, 128, 8, vec![e]);
        let mut st = BramStats::default();
        let cells = lutram(&mut m, &x, 0, &mut st).unwrap();
        // sized by the 128 words; address bit 7 set means no write
        assert_eq!(count(&cells, "RAM128X1D"), 1);
        assert_eq!(count(&cells, "LUT1"), 1);
        assert_eq!(count(&cells, "LUT2"), 1);
        assert_eq!(count(&cells, "LUT3"), 0);
    }

    #[test]
    fn per_bit_enables_get_their_own_columns() {
        let mut m = Module::new("t");
        let (e0, e1) = (m.wire(1).0[0], m.wire(1).0[0]);
        let x = amem(&mut m, 32, 5, vec![e0, e0, e1, e1]);
        let mut st = BramStats::default();
        let cells = lutram(&mut m, &x, 0, &mut st).unwrap();
        let wes: Vec<SigSpec> = cells
            .iter()
            .filter_map(|c| match c {
                Cell::Instance { ty, conns, .. } if ty == "RAM32M" => Some(conns["WE"].clone()),
                _ => None,
            })
            .collect();
        assert_eq!(wes, vec![SigSpec(vec![e0]), SigSpec(vec![e1])]);
    }

    #[test]
    fn wide_write_address_gates_we() {
        let mut m = Module::new("t");
        let e = m.wire(1).0[0];
        // 64 words, 16-bit address ports: bits 6..15 must be zero to write
        let x = amem(&mut m, 64, 16, vec![e, e, e]);
        let mut st = BramStats::default();
        let cells = lutram(&mut m, &x, 0, &mut st).unwrap();
        assert_eq!(count(&cells, "RAM64M"), 1);
        // NOR of 10 bits: two NOR LUTs (6 + 4), one AND, then en & ok
        assert_eq!(count(&cells, "LUT6"), 1);
        assert_eq!(count(&cells, "LUT4"), 1);
        assert_eq!(count(&cells, "LUT2"), 2);
    }

    #[test]
    fn bank_mux_truth_table() {
        // inputs d0..d3, s0, s1: out = d[s]
        let tt: Vec<bool> = (0..64usize).map(|p| (p >> (p >> 4)) & 1 == 1).collect();
        for p in 0..64usize {
            let sel = p >> 4;
            assert_eq!(tt[p], (p >> sel) & 1 == 1);
            assert_eq!(tt[p], [p & 1, (p >> 1) & 1, (p >> 2) & 1, (p >> 3) & 1][sel] == 1);
        }
    }
}
