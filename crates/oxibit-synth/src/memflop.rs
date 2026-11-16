//! Memories no RAM primitive can hold, as flops (yosys `memory_map`).
//!
//! A memory with more than one write port fits neither block RAM nor
//! distributed RAM (each has one write port), so it becomes one register
//! per word, named `MEMID[i]` as `memory_map` names them, so the
//! equivalence check pairs them with the source's. Each word's next value
//! applies the write ports in order, a later port winning where two write
//! the same bits (a `$mem_v2` priority mask that puts every later port over
//! every earlier one, which is the order elaboration emits them in); the
//! word keeps the memory's initial contents. A read port is a tree of
//! multiplexers on its address over the words; a synchronous one registers
//! that (read-first: it samples the words before the edge's writes land).
//! Words past the end of the memory read 0 (x in yosys's model).
//!
//! The memory must have no address offset and every write port the same
//! clock edge; otherwise it is left for the RAM mapping to report.

use oxibit_hdl::netlist::{Bit, Cell, Memory, Module, Op, SigSpec, Signal};
use oxibit_prof::hot;

#[derive(Debug, Default, Clone)]
pub struct MemFlopStats {
    /// Memories turned into registers.
    pub memories: usize,
    /// Register bits they became.
    pub bits: usize,
    /// Multi-write memories left alone, with the reason.
    pub left: Vec<(String, &'static str)>,
}

/// Why `mem` can't become flops, if it can't.
fn eligible(mem: &Memory) -> Result<(), &'static str> {
    if mem.offset != 0 {
        return Err("nonzero offset");
    }
    let Some(first) = mem.writes.first() else {
        return Err("no write port");
    };
    if mem.writes.iter().any(|w| w.clk != first.clk) {
        return Err("write ports on different clocks");
    }
    Ok(())
}

fn binary(m: &mut Module, op: Op, a: SigSpec, b: SigSpec, w: u32) -> SigSpec {
    let y = m.wire(w);
    m.cells.push(Cell::Binary {
        op,
        a,
        b,
        signed: false,
        y: y.clone(),
    });
    y
}

fn mux(m: &mut Module, a: SigSpec, b: SigSpec, s: Bit) -> SigSpec {
    let y = m.wire(a.width());
    m.cells.push(Cell::Mux {
        a,
        b,
        s: SigSpec(vec![s]),
        y: y.clone(),
    });
    y
}

/// The registers and logic for `mem`.
fn expand(m: &mut Module, mem: &Memory) -> usize {
    let width = mem.width;
    let words = mem.size as usize;
    let (clk, posedge) = mem.writes[0].clk.clone();
    let mut q = Vec::with_capacity(words);
    for w in 0..words {
        let init = mem.init.get(&(w as u32)).cloned();
        let id = m.add_signal(Signal {
            name: format!("{}[{w}]", mem.name),
            width,
            signed: false,
            port: None,
            init,
        });
        q.push(SigSpec::of(id, width));
    }
    for (w, qw) in q.iter().enumerate() {
        let mut v = qw.clone();
        for wr in &mem.writes {
            let aw = wr.addr.width();
            // A word past what the address can reach is never written.
            if aw < 64 && (w as u64) >> aw != 0 {
                continue;
            }
            let hit = binary(m, Op::Eq, wr.addr.clone(), SigSpec::constant(w as u64, aw), 1).0[0];
            let en0 = wr.en.0[0];
            v = if wr.en.0.iter().all(|b| *b == en0) {
                let s = binary(m, Op::And, SigSpec(vec![en0]), SigSpec(vec![hit]), 1).0[0];
                mux(m, v, wr.data.clone(), s)
            } else {
                let sel = binary(m, Op::And, wr.en.clone(), SigSpec(vec![hit; width as usize]), width);
                let keep = m.wire(width);
                m.cells.push(Cell::Unary {
                    op: Op::Not,
                    a: sel.clone(),
                    a_signed: false,
                    y: keep.clone(),
                });
                let old = binary(m, Op::And, v, keep, width);
                let new = binary(m, Op::And, wr.data.clone(), sel, width);
                binary(m, Op::Or, old, new, width)
            };
        }
        m.cells.push(Cell::Dff {
            clk: clk.clone(),
            posedge,
            d: v,
            q: qw.clone(),
        });
    }
    for rd in &mem.reads {
        // Address bits that select among the words; the rest only reach
        // past the end.
        let sel = (usize::BITS - (words.max(1) - 1).leading_zeros()) as usize;
        let mut level: Vec<SigSpec> = (0..1usize << sel)
            .map(|w| q.get(w).cloned().unwrap_or_else(|| SigSpec(vec![Bit::Zero; width as usize])))
            .collect();
        for k in 0..sel {
            let s = rd.addr.0.get(k).copied().unwrap_or(Bit::Zero);
            level = level.chunks(2).map(|p| mux(m, p[0].clone(), p[1].clone(), s)).collect();
        }
        let mut data = level.pop().unwrap_or_else(|| SigSpec(vec![Bit::Zero; width as usize]));
        let high: Vec<Bit> = rd.addr.0.iter().skip(sel).copied().collect();
        if !high.is_empty() {
            let out = m.wire(1);
            m.cells.push(Cell::Unary {
                op: Op::ReduceOr,
                a: SigSpec(high),
                a_signed: false,
                y: out.clone(),
            });
            data = mux(m, data, SigSpec(vec![Bit::Zero; width as usize]), out.0[0]);
        }
        match &rd.clk {
            None => m.connects.push((rd.data.clone(), data)),
            Some((c, p)) => m.cells.push(Cell::Dff {
                clk: c.clone(),
                posedge: *p,
                d: data,
                q: rd.data.clone(),
            }),
        }
    }
    words * width as usize
}

#[hot]
pub fn map(m: &mut Module) -> MemFlopStats {
    let mut st = MemFlopStats::default();
    let cells = std::mem::take(&mut m.cells);
    let mut keep = Vec::with_capacity(cells.len());
    let mut take = Vec::new();
    for cell in cells {
        match &cell {
            Cell::Mem(mem) if mem.writes.len() > 1 => match eligible(mem) {
                Ok(()) => take.push(cell),
                Err(why) => {
                    st.left.push((mem.name.clone(), why));
                    keep.push(cell);
                }
            },
            _ => keep.push(cell),
        }
    }
    m.cells = keep;
    for cell in take {
        let Cell::Mem(mem) = cell else { unreachable!() };
        st.bits += expand(m, &mem);
        st.memories += 1;
    }
    st
}

/// Multi-write memories to registers.
pub struct MemFlop;

impl crate::stage::Pass for MemFlop {
    const NAME: &'static str = "memflop";
    const EMITS: &'static [crate::caps::Mode] = &[];
    type Stats = MemFlopStats;
    fn enabled(p: &crate::policy::PassSet) -> bool {
        p.memflop
    }
    fn json(s: &MemFlopStats) -> serde_json::Value {
        serde_json::json!({ "memories": s.memories, "bits": s.bits, "left": s.left })
    }
}

impl crate::stage::RtlPass for MemFlop {
    fn run(&self, m: &mut crate::stage::Rtl) -> MemFlopStats {
        map(&mut m.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxibit_hdl::netlist::{MemRead, MemWrite};
    use std::collections::BTreeMap;

    #[test]
    fn two_write_ports_become_words() {
        let mut m = Module::new("t");
        let clk = m.wire(1);
        let (a0, a1, ra) = (m.wire(2), m.wire(2), m.wire(3));
        let (d0, e0, e1, rd) = (m.wire(3), m.wire(1), m.wire(1), m.wire(3));
        m.cells.push(Cell::Mem(Memory {
            name: "mm".into(),
            width: 3,
            size: 4,
            offset: 0,
            init: BTreeMap::from([(1, vec![Some(true), Some(false), Some(true)])]),
            attrs: BTreeMap::new(),
            reads: vec![MemRead {
                clk: None,
                addr: ra,
                data: rd,
            }],
            writes: vec![
                MemWrite {
                    clk: (clk.clone(), true),
                    addr: a0,
                    data: d0,
                    en: SigSpec(vec![e0.0[0]; 3]),
                },
                MemWrite {
                    clk: (clk.clone(), true),
                    addr: a1,
                    data: SigSpec::constant(0, 3),
                    en: SigSpec(vec![e1.0[0]; 3]),
                },
            ],
        }));
        let st = map(&mut m);
        assert_eq!((st.memories, st.bits), (1, 12));
        assert!(!m.cells.iter().any(|c| matches!(c, Cell::Mem(_))));
        assert_eq!(m.cells.iter().filter(|c| matches!(c, Cell::Dff { .. })).count(), 4);
        let w1 = m.lookup("mm[1]").expect("word register named after the memory");
        assert_eq!(m.signal(w1).init, Some(vec![Some(true), Some(false), Some(true)]));
        // The read data are driven by the multiplexer tree.
        assert_eq!(m.connects.len(), 1);
    }
}
