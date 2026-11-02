//! Memories with port shapes the RAM mappers cannot take, reshaped before
//! lowering (after `memdff`, so read-data flops are already in the ports).
//!
//! - **Merge.** Consecutive write ports on one clock merge into one port
//!   when they write the same address (the later port wins each bit it
//!   enables) or when SAT proves their enables are never on together
//!   (`exclusive`, as yosys's `memory_share` does): address and data
//!   then come from the second port when its enable is on. Only adjacent
//!   ports merge, so the priority order between ports that stay separate
//!   is unchanged.
//! - **Split.** At most one write port and several read ports: one copy
//!   per read port. Every copy sees the same writes from the same initial
//!   contents, so at every cycle each holds the original's contents and
//!   each read port reads what it read before. `bram` then maps each copy
//!   (one block RAM column, or LUTRAM, per read port).
//! - **Flops.** Several write ports, all on one clock edge, at most
//!   `FF_BITS` bits: a register per word, initialised to the word's
//!   initial contents (the flop's INIT). The write ports apply in order, so a later port
//!   wins a same-cycle collision (Verilog statement order, which is how the
//!   front end orders them). A read is an address mux tree over the word
//!   registers; a synchronous read registers the tree's output on its own
//!   clock, which reads the value from before this cycle's writes
//!   (read-old, the convention the RAM mapping uses).
//!
//! A write to an address no word has (wider address, or past `size`)
//! writes nothing; a read past `size` returns some word, where `$mem_v2`
//! returns X. Both are word-level rewrites ahead of lowering, so the
//! equivalence check sees them, against a gold side whose memories are
//! mapped to flops (`memory_map`).

use oxibit_hdl::netlist::{Bit, Cell, MemRead, MemWrite, Memory, Module, Op, SigSpec};

use crate::exclusive::{Exclusive, ExclusiveStats};
use oxibit_prof::hot;

/// Flop-mapped memories are capped at this many bits.
pub const FF_BITS: u64 = 4096;

#[derive(Debug, Default, Clone)]
pub struct MemPortsStats {
    /// Write ports removed by merging, and the SAT calls behind them.
    pub merged_ports: usize,
    pub sat: ExclusiveStats,
    /// Memories split per read port, and the copies made.
    pub split: usize,
    pub copies: usize,
    /// Memories turned into word registers, and the flop bits used.
    pub flops: usize,
    pub flop_bits: u64,
    /// Multi-port memories left alone, with the reason.
    pub left: Vec<(String, &'static str)>,
}

/// Why `mem` cannot become flops, if it cannot.
fn flop_eligible(mem: &Memory) -> Result<(), &'static str> {
    if mem.writes.is_empty() {
        return Err("no write port");
    }
    let clk0 = &mem.writes[0].clk;
    if mem.writes.iter().any(|w| w.clk != *clk0) {
        return Err("write ports on different clocks");
    }
    if mem.offset != 0 {
        return Err("nonzero offset");
    }
    if mem.size as u64 * mem.width as u64 > FF_BITS {
        return Err("too many bits for flops");
    }
    Ok(())
}

fn konst(v: u64, w: u32) -> SigSpec {
    SigSpec::constant(v, w)
}

/// `y = s ? b : a`, as a new wire.
fn mux(m: &mut Module, out: &mut Vec<Cell>, a: SigSpec, b: SigSpec, s: Bit) -> SigSpec {
    let y = m.wire(a.width());
    out.push(Cell::Mux {
        a,
        b,
        s: SigSpec(vec![s]),
        y: y.clone(),
    });
    y
}

fn binary(m: &mut Module, out: &mut Vec<Cell>, op: Op, a: SigSpec, b: SigSpec, w: u32) -> SigSpec {
    let y = m.wire(w);
    out.push(Cell::Binary {
        op,
        a,
        b,
        signed: false,
        y: y.clone(),
    });
    y
}

/// The word registers' next values, written by every port in order.
fn next_words(m: &mut Module, out: &mut Vec<Cell>, mem: &Memory, q: &[SigSpec]) -> Vec<SigSpec> {
    let width = mem.width as usize;
    let mut next: Vec<SigSpec> = q.to_vec();
    for wp in &mem.writes {
        let aw = wp.addr.width();
        for (w, cur) in next.iter_mut().enumerate() {
            // An address no word has never matches: compare the whole address.
            if aw < 64 && (w as u64) >> aw != 0 {
                continue;
            }
            let hit = binary(m, out, Op::Eq, wp.addr.clone(), konst(w as u64, aw), 1).0[0];
            // Runs of bits under one enable bit share a select.
            let mut parts: Vec<Bit> = Vec::with_capacity(width);
            let mut i = 0;
            while i < width {
                let en = wp.en.0[i];
                let mut j = i + 1;
                while j < width && wp.en.0[j] == en {
                    j += 1;
                }
                let lo = i as u32;
                let n = (j - i) as u32;
                let seg = match en {
                    Bit::Zero => cur.slice(lo, n),
                    Bit::One => mux(m, out, cur.slice(lo, n), wp.data.slice(lo, n), hit),
                    e => {
                        let sel = binary(m, out, Op::And, SigSpec(vec![hit]), SigSpec(vec![e]), 1).0[0];
                        mux(m, out, cur.slice(lo, n), wp.data.slice(lo, n), sel)
                    }
                };
                parts.extend(seg.0);
                i = j;
            }
            *cur = SigSpec(parts);
        }
    }
    next
}

/// An address mux tree over `words`, selected by the low address bits.
fn read_tree(m: &mut Module, out: &mut Vec<Cell>, words: &[SigSpec], addr: &SigSpec) -> SigSpec {
    let mut level: Vec<SigSpec> = words.to_vec();
    let mut k = 0usize;
    while level.len() > 1 {
        let s = addr.0.get(k).copied().unwrap_or(Bit::Zero);
        let mut up = Vec::with_capacity(level.len().div_ceil(2));
        for pair in level.chunks(2) {
            up.push(match pair {
                [a, b] => mux(m, out, a.clone(), b.clone(), s),
                [a] => a.clone(),
                _ => unreachable!(),
            });
        }
        level = up;
        k += 1;
    }
    level.pop().unwrap_or_else(|| konst(0, 0))
}

/// Give the fresh word register `q` the memory's initial word `w`.
fn set_init(m: &mut Module, q: &SigSpec, w: &[Option<bool>]) {
    if let Some(Bit::Sig(id, _)) = q.0.first() {
        let mut v = w.to_vec();
        v.resize(q.0.len(), None);
        m.signals[id.0 as usize].init = Some(v);
    }
}

fn to_flops(m: &mut Module, out: &mut Vec<Cell>, mem: &Memory) {
    let (clk, posedge) = mem.writes[0].clk.clone();
    let q: Vec<SigSpec> = (0..mem.size).map(|_| m.wire(mem.width)).collect();
    for (a, word) in &mem.init {
        if let Some(qw) = q.get(*a as usize) {
            set_init(m, qw, word);
        }
    }
    let next = next_words(m, out, mem, &q);
    for (qw, d) in q.iter().zip(next) {
        out.push(Cell::Dff {
            clk: clk.clone(),
            posedge,
            d,
            q: qw.clone(),
        });
    }
    for rd in &mem.reads {
        let v = read_tree(m, out, &q, &rd.addr);
        match &rd.clk {
            None => m.connects.push((rd.data.clone(), v)),
            Some((rclk, rpos)) => out.push(Cell::Dff {
                clk: rclk.clone(),
                posedge: *rpos,
                d: v,
                q: rd.data.clone(),
            }),
        }
    }
}

fn split(mem: &Memory) -> Vec<Memory> {
    mem.reads
        .iter()
        .enumerate()
        .map(|(k, rd)| Memory {
            name: format!("{}_r{k}", mem.name),
            reads: vec![MemRead {
                clk: rd.clk.clone(),
                addr: rd.addr.clone(),
                data: rd.data.clone(),
            }],
            ..mem.clone()
        })
        .collect()
}

/// Groups of consecutive write ports that merge: same clock, and either
/// the same address as every port so far in the group, or enables proven
/// exclusive with the group's.
fn plan_groups(ex: &mut Exclusive, mem: &Memory) -> Vec<Vec<usize>> {
    let mut groups: Vec<Vec<usize>> = vec![vec![0]];
    let mut shared_addr: Option<SigSpec> = Some(mem.writes[0].addr.clone());
    for j in 1..mem.writes.len() {
        let w = &mem.writes[j];
        let g = groups.last().unwrap();
        let mergeable = w.clk == mem.writes[g[0]].clk
            && (shared_addr.as_ref() == Some(&w.addr) || {
                let en: Vec<Bit> = g.iter().flat_map(|&i| mem.writes[i].en.0.iter().copied()).collect();
                ex.exclusive(&en, &w.en.0)
            });
        if mergeable {
            if shared_addr.as_ref() != Some(&w.addr) {
                shared_addr = None;
            }
            groups.last_mut().unwrap().push(j);
        } else {
            groups.push(vec![j]);
            shared_addr = Some(w.addr.clone());
        }
    }
    groups
}

/// One write port standing for a planned group (see `plan_groups`).
fn merge_group(m: &mut Module, out: &mut Vec<Cell>, mem: &Memory, g: &[usize]) -> MemWrite {
    let mut cur = mem.writes[g[0]].clone();
    let mut shared = true;
    for &j in &g[1..] {
        let w = &mem.writes[j];
        let width = cur.data.width();
        let en = binary(m, out, Op::Or, cur.en.clone(), w.en.clone(), width);
        if shared && w.addr == cur.addr {
            // same address: per bit, the later port's data where it writes
            let mut data = Vec::with_capacity(width as usize);
            let mut i = 0usize;
            while i < width as usize {
                let e = w.en.0[i];
                let mut k = i + 1;
                while k < width as usize && w.en.0[k] == e {
                    k += 1;
                }
                let (lo, n) = (i as u32, (k - i) as u32);
                let seg = match e {
                    Bit::Zero => cur.data.slice(lo, n),
                    Bit::One => w.data.slice(lo, n),
                    _ => mux(m, out, cur.data.slice(lo, n), w.data.slice(lo, n), e),
                };
                data.extend(seg.0);
                i = k;
            }
            cur = MemWrite {
                clk: cur.clk,
                addr: cur.addr,
                data: SigSpec(data),
                en,
            };
        } else {
            // exclusive enables: the second port's address and data when it writes
            shared = false;
            let sel = if w.en.0.iter().all(|b| *b == w.en.0[0]) {
                w.en.0[0]
            } else {
                let y = m.wire(1);
                out.push(Cell::Unary {
                    op: Op::ReduceOr,
                    a: w.en.clone(),
                    a_signed: false,
                    y: y.clone(),
                });
                y.0[0]
            };
            let aw = cur.addr.width().max(w.addr.width());
            let addr = mux(m, out, cur.addr.extend(aw, false), w.addr.extend(aw, false), sel);
            let data = mux(m, out, cur.data.clone(), w.data.clone(), sel);
            cur = MemWrite {
                clk: cur.clk,
                addr,
                data,
                en,
            };
        }
    }
    cur
}

#[hot]
pub fn reshape(m: &mut Module) -> MemPortsStats {
    let mut st = MemPortsStats::default();
    // Phase 1, read-only: which write ports merge.
    let plans: Vec<Option<Vec<Vec<usize>>>> = {
        let mut ex = Exclusive::new(m);
        let plans = m
            .cells
            .iter()
            .map(|c| match c {
                Cell::Mem(mem) if mem.writes.len() > 1 => Some(plan_groups(&mut ex, mem)),
                _ => None,
            })
            .collect();
        st.sat = ex.stats;
        plans
    };
    // Phase 2: rewrite.
    let cells = std::mem::take(&mut m.cells);
    let mut out = Vec::with_capacity(cells.len());
    for (ci, cell) in cells.into_iter().enumerate() {
        let Cell::Mem(orig) = &cell else {
            out.push(cell);
            continue;
        };
        let merged;
        let mem = match &plans[ci] {
            Some(groups) if groups.len() < orig.writes.len() => {
                st.merged_ports += orig.writes.len() - groups.len();
                let writes = groups.iter().map(|g| merge_group(m, &mut out, orig, g)).collect();
                merged = Memory {
                    writes,
                    ..orig.clone()
                };
                &merged
            }
            _ => orig,
        };
        if mem.writes.len() <= 1 && mem.reads.len() <= 1 {
            out.push(Cell::Mem(mem.clone()));
            continue;
        }
        if mem.writes.len() <= 1 && mem.reads.len() > 1 {
            let copies = split(mem);
            st.split += 1;
            st.copies += copies.len();
            out.extend(copies.into_iter().map(Cell::Mem));
            continue;
        }
        if mem.writes.len() > 1 {
            match flop_eligible(mem) {
                Ok(()) => {
                    st.flops += 1;
                    st.flop_bits += mem.size as u64 * mem.width as u64;
                    to_flops(m, &mut out, mem);
                }
                Err(why) => {
                    st.left.push((mem.name.clone(), why));
                    out.push(Cell::Mem(mem.clone()));
                }
            }
            continue;
        }
        out.push(Cell::Mem(mem.clone()));
    }
    m.cells = out;
    st
}

/// Multi-port memories into shapes `bram` maps (copies per read port) or
/// into flops (several write ports).
pub struct MemPorts;

impl crate::stage::Pass for MemPorts {
    const NAME: &'static str = "memports";
    const EMITS: &'static [crate::caps::Mode] = &[];
    type Stats = MemPortsStats;
    fn enabled(p: &crate::policy::PassSet) -> bool {
        p.memports
    }
    fn json(s: &MemPortsStats) -> serde_json::Value {
        serde_json::json!({
            "split": s.split,
            "copies": s.copies,
            "flops": s.flops,
            "flop_bits": s.flop_bits,
            "merged_ports": s.merged_ports,
            "sat_calls": s.sat.calls,
            "sat_proven": s.sat.proven,
            "sat_refuted": s.sat.refuted,
            "sat_unknown": s.sat.unknown,
            "sat_cone_bits": s.sat.cone_bits,
            "left": s.left,
        })
    }
}

impl crate::stage::RtlPass for MemPorts {
    fn run(&self, m: &mut crate::stage::Rtl) -> MemPortsStats {
        reshape(&mut m.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxibit_hdl::netlist::MemWrite;

    fn mem(m: &mut Module, size: u32, width: u32, writes: usize, reads: usize, sync: bool) -> Memory {
        let clk = m.wire(1);
        let aw = 32 - (size - 1).leading_zeros();
        Memory {
            name: "m".into(),
            width,
            size,
            offset: 0,
            init: Default::default(),
            attrs: Default::default(),
            writes: (0..writes)
                .map(|_| MemWrite {
                    clk: (clk.clone(), true),
                    addr: m.wire(aw),
                    data: m.wire(width),
                    en: SigSpec(vec![m.wire(1).0[0]; width as usize]),
                })
                .collect(),
            reads: (0..reads)
                .map(|_| MemRead {
                    clk: sync.then(|| (clk.clone(), true)),
                    addr: m.wire(aw),
                    data: m.wire(width),
                })
                .collect(),
        }
    }

    #[test]
    fn splits_one_write_many_reads() {
        let mut m = Module::new("t");
        let x = mem(&mut m, 256, 36, 1, 3, true);
        m.cells.push(Cell::Mem(x.clone()));
        let st = reshape(&mut m);
        assert_eq!((st.split, st.copies, st.flops), (1, 3, 0));
        let mems: Vec<&Memory> = m
            .cells
            .iter()
            .filter_map(|c| if let Cell::Mem(x) = c { Some(x) } else { None })
            .collect();
        assert_eq!(mems.len(), 3);
        for (k, c) in mems.iter().enumerate() {
            assert_eq!(c.reads.len(), 1);
            assert_eq!(c.reads[0], x.reads[k]);
            assert_eq!(c.writes, x.writes);
        }
    }

    #[test]
    fn multi_write_becomes_flops() {
        let mut m = Module::new("t");
        let x = mem(&mut m, 32, 6, 2, 2, false);
        m.cells.push(Cell::Mem(x));
        let st = reshape(&mut m);
        assert_eq!((st.flops, st.flop_bits), (1, 192));
        let dffs = m.cells.iter().filter(|c| matches!(c, Cell::Dff { .. })).count();
        assert_eq!(dffs, 32);
        assert!(!m.cells.iter().any(|c| matches!(c, Cell::Mem(_))));
        // two asynchronous reads: two connects from the mux trees
        assert_eq!(m.connects.len(), 2);
    }

    #[test]
    fn sync_read_gets_a_register() {
        let mut m = Module::new("t");
        let x = mem(&mut m, 8, 4, 2, 1, true);
        m.cells.push(Cell::Mem(x));
        reshape(&mut m);
        let dffs = m.cells.iter().filter(|c| matches!(c, Cell::Dff { .. })).count();
        assert_eq!(dffs, 8 + 1);
    }

    #[test]
    fn initial_contents_reach_the_registers() {
        let mut m = Module::new("t");
        let mut x = mem(&mut m, 4, 3, 2, 1, false);
        x.init.insert(2, vec![Some(true), Some(false), Some(true)]);
        m.cells.push(Cell::Mem(x));
        let st = reshape(&mut m);
        assert_eq!(st.flops, 1);
        let qs: Vec<&SigSpec> = m
            .cells
            .iter()
            .filter_map(|c| if let Cell::Dff { q, .. } = c { Some(q) } else { None })
            .collect();
        let init = |q: &SigSpec| match q.0[0] {
            Bit::Sig(id, _) => m.signals[id.0 as usize].init.clone(),
            _ => None,
        };
        assert_eq!(init(qs[2]), Some(vec![Some(true), Some(false), Some(true)]));
        assert_eq!(init(qs[0]), None);
    }

    #[test]
    fn exclusive_writers_merge_into_one_port() {
        let mut m = Module::new("t");
        let mut x = mem(&mut m, 32, 4, 2, 1, false);
        // enables: state == 1 and state == 2
        let state = m.wire(3);
        for (k, w) in x.writes.iter_mut().enumerate() {
            let y = m.wire(1);
            m.cells.push(Cell::Binary {
                op: Op::Eq,
                a: state.clone(),
                b: SigSpec::constant(k as u64 + 1, 3),
                signed: false,
                y: y.clone(),
            });
            w.en = SigSpec(vec![y.0[0]; 4]);
        }
        m.cells.push(Cell::Mem(x));
        let st = reshape(&mut m);
        assert_eq!((st.merged_ports, st.flops), (1, 0));
        let mems: Vec<&Memory> = m.cells.iter().filter_map(|c| if let Cell::Mem(x) = c { Some(x) } else { None }).collect();
        assert_eq!(mems.len(), 1);
        assert_eq!(mems[0].writes.len(), 1);
    }

    #[test]
    fn same_address_writers_merge() {
        let mut m = Module::new("t");
        let mut x = mem(&mut m, 32, 4, 2, 1, false);
        x.writes[1].addr = x.writes[0].addr.clone();
        m.cells.push(Cell::Mem(x));
        let st = reshape(&mut m);
        assert_eq!((st.merged_ports, st.flops, st.sat.calls), (1, 0, 0));
    }

    #[test]
    fn independent_writers_stay_flops() {
        let mut m = Module::new("t");
        let x = mem(&mut m, 32, 4, 2, 1, false);
        m.cells.push(Cell::Mem(x));
        let st = reshape(&mut m);
        assert_eq!((st.merged_ports, st.flops), (0, 1));
        assert_eq!(st.sat.refuted, 1);
    }

    #[test]
    fn big_multi_write_is_left() {
        let mut m = Module::new("t");
        let x = mem(&mut m, 1024, 36, 2, 1, true);
        m.cells.push(Cell::Mem(x));
        let st = reshape(&mut m);
        assert_eq!(st.left.len(), 1);
        assert!(m.cells.iter().any(|c| matches!(c, Cell::Mem(_))));
    }
}
