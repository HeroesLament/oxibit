//! Merge read-data flops into memory read ports (yosys `memory_dff`, the
//! output-register case).
//!
//! An asynchronous read port whose every data bit feeds only the D input
//! of a plain `$dff` bit, all on one clock edge and without an initial
//! value, becomes a synchronous read port whose data are those flops'
//! outputs. The flop samples the read before the same edge's writes land,
//! so the port is non-transparent (read-first), which is what `$mem_v2`
//! with a zero transparency mask means; yosys's `memory_nordff` undoes the
//! merge exactly, which is how the result is checked.

use std::collections::HashMap;

use oxibit_hdl::netlist::{Bit, Cell, Module, SigSpec};
use oxibit_prof::hot;

#[hot]
pub fn merge(m: &mut Module) -> usize {
    // Canonical bit -> index, through connections.
    let mut offs = Vec::with_capacity(m.signals.len() + 1);
    let mut n = 0;
    for s in &m.signals {
        offs.push(n);
        n += s.width as usize;
    }
    let mut parent: Vec<u32> = (0..n as u32).collect();
    fn find(p: &mut [u32], mut x: usize) -> usize {
        while p[x] as usize != x {
            let g = p[p[x] as usize];
            p[x] = g;
            x = g as usize;
        }
        x
    }
    let idx = |b: Bit| match b {
        Bit::Sig(id, i) => Some(offs[id.0 as usize] + i as usize),
        _ => None,
    };
    let mut tied = vec![false; n];
    for (l, r) in &m.connects {
        for (a, b) in l.0.iter().zip(&r.0) {
            match (idx(*a), idx(*b)) {
                (Some(x), Some(y)) => {
                    let (rx, ry) = (find(&mut parent, x), find(&mut parent, y));
                    if rx != ry {
                        parent[ry] = rx as u32;
                        tied[rx] |= tied[ry];
                    }
                }
                (Some(x), None) | (None, Some(x)) => {
                    let r = find(&mut parent, x);
                    tied[r] = true;
                }
                _ => {}
            }
        }
    }
    let mut canon = |b: Bit| -> Option<usize> {
        let x = idx(b)?;
        let r = find(&mut parent, x);
        if tied[r] { None } else { Some(r) }
    };
    // Uses of every canonical bit as a cell input or a port.
    let mut uses = vec![0u32; n];
    let count = |s: &SigSpec, uses: &mut Vec<u32>, canon: &mut dyn FnMut(Bit) -> Option<usize>| {
        for b in &s.0 {
            if let Some(r) = canon(*b) {
                uses[r] += 1;
            }
        }
    };
    for c in &m.cells {
        match c {
            Cell::Unary { a, .. } => count(a, &mut uses, &mut canon),
            Cell::Binary { a, b, .. } => {
                count(a, &mut uses, &mut canon);
                count(b, &mut uses, &mut canon);
            }
            Cell::Mux { a, b, s, .. } => {
                count(a, &mut uses, &mut canon);
                count(b, &mut uses, &mut canon);
                count(s, &mut uses, &mut canon);
            }
            Cell::Dff { clk, d, .. } => {
                count(clk, &mut uses, &mut canon);
                count(d, &mut uses, &mut canon);
            }
            Cell::Adff { clk, arst, d, .. } => {
                count(clk, &mut uses, &mut canon);
                count(arst, &mut uses, &mut canon);
                count(d, &mut uses, &mut canon);
            }
            Cell::Instance { conns, .. } => {
                // Directions unknown here: count every connection.
                for s in conns.values() {
                    count(s, &mut uses, &mut canon);
                }
            }
            Cell::Mem(mem) => {
                for r in &mem.reads {
                    count(&r.addr, &mut uses, &mut canon);
                    if let Some((c, _)) = &r.clk {
                        count(c, &mut uses, &mut canon);
                    }
                }
                for w in &mem.writes {
                    count(&w.clk.0, &mut uses, &mut canon);
                    count(&w.addr, &mut uses, &mut canon);
                    count(&w.data, &mut uses, &mut canon);
                    count(&w.en, &mut uses, &mut canon);
                }
            }
        }
    }
    for s in &m.signals.clone() {
        if s.port.is_some() {
            let id = m.lookup(&s.name).unwrap();
            count(&SigSpec::of(id, s.width), &mut uses, &mut canon);
        }
    }
    // Flop bit fed by each canonical bit: (cell, bit).
    let mut ff_of: HashMap<usize, (usize, usize)> = HashMap::new();
    for (ci, c) in m.cells.iter().enumerate() {
        if let Cell::Dff { d, q, .. } = c {
            for (i, b) in d.0.iter().enumerate() {
                let init = match q.0[i] {
                    Bit::Sig(id, k) => m.signal(id).init.as_ref().and_then(|v| v.get(k as usize).copied().flatten()).is_some(),
                    _ => true,
                };
                if init {
                    continue;
                }
                if let Some(r) = canon(*b) {
                    ff_of.entry(r).or_insert((ci, i));
                }
            }
        }
    }
    // Choose the ports to merge.
    let mut removed: HashMap<usize, Vec<bool>> = HashMap::new();
    let mut plans: Vec<(usize, usize, (SigSpec, bool), SigSpec)> = Vec::new();
    for (ci, c) in m.cells.iter().enumerate() {
        let Cell::Mem(mem) = c else { continue };
        'port: for (pi, r) in mem.reads.iter().enumerate() {
            if r.clk.is_some() {
                continue;
            }
            let mut clk: Option<(SigSpec, bool)> = None;
            let mut q = Vec::new();
            let mut bits = Vec::new();
            for b in &r.data.0 {
                let Some(cb) = canon(*b) else { continue 'port };
                if uses[cb] != 1 {
                    continue 'port;
                }
                let Some(&(fc, fi)) = ff_of.get(&cb) else {
                    continue 'port;
                };
                let Cell::Dff {
                    clk: fclk, posedge, q: fq, ..
                } = &m.cells[fc]
                else {
                    unreachable!()
                };
                match &clk {
                    None => clk = Some((fclk.clone(), *posedge)),
                    Some((c0, p0)) => {
                        if canon(c0.0[0]) != canon(fclk.0[0]) || *p0 != *posedge {
                            continue 'port;
                        }
                    }
                }
                q.push(fq.0[fi]);
                bits.push((fc, fi));
            }
            let Some(clk) = clk else { continue };
            for (fc, fi) in bits {
                let w = match &m.cells[fc] {
                    Cell::Dff { q, .. } => q.width() as usize,
                    _ => unreachable!(),
                };
                removed.entry(fc).or_insert_with(|| vec![false; w])[fi] = true;
            }
            plans.push((ci, pi, clk, SigSpec(q)));
        }
    }
    let merged = plans.len();
    for (ci, pi, clk, q) in plans {
        if let Cell::Mem(mem) = &mut m.cells[ci] {
            mem.reads[pi].clk = Some(clk);
            mem.reads[pi].data = q;
        }
    }
    // Drop the merged flop bits.
    for (fc, gone) in removed {
        if let Cell::Dff { clk, posedge, d, q } = &m.cells[fc] {
            let keep: Vec<usize> = (0..gone.len()).filter(|i| !gone[*i]).collect();
            let nd = SigSpec(keep.iter().map(|i| d.0[*i]).collect());
            let nq = SigSpec(keep.iter().map(|i| q.0[*i]).collect());
            m.cells[fc] = Cell::Dff {
                clk: clk.clone(),
                posedge: *posedge,
                d: nd,
                q: nq,
            };
        }
    }
    m.cells.retain(|c| !matches!(c, Cell::Dff { q, .. } if q.width() == 0));
    merged
}

/// Read-data flops into memory read ports (always on: the RAM mapping
/// needs synchronous read ports).
pub struct MemDff;

impl crate::stage::Pass for MemDff {
    const NAME: &'static str = "memdff";
    const EMITS: &'static [crate::caps::Mode] = &[];
    type Stats = usize;
    fn enabled(_: &crate::policy::PassSet) -> bool {
        true
    }
    fn json(s: &usize) -> serde_json::Value {
        serde_json::json!({ "mem_rd_ff": s })
    }
}

impl crate::stage::RtlPass for MemDff {
    fn run(&self, m: &mut crate::stage::Rtl) -> usize {
        merge(&mut m.0)
    }
}
