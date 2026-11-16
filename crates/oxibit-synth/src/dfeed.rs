//! Flop D inputs fed through a LUT (bypass-pin relief).
//!
//! A 7-series slice feeds a flop's D internally only from the same
//! slice's LUT or carry chain; any other source (another flop's Q, a DSP,
//! block RAM, I/O buffer or PS7 output, a port) enters through one of the
//! slice's four bypass pins (AX..DX). Each slice has eight flops, so a
//! netlist with many such flops runs the bypass pins and their switch-box
//! muxes (BYP_ALT / BYP_BOUNCE) out of capacity and routing stalls there.
//! This pass puts a pass-through LUT1 in front of each synthesis flop whose
//! D has such a source, so the packer can pair LUT and flop and D arrives
//! internally, the LUT's input coming through the plentiful IMUX routing.
//! An identity buffer, after the checking view.
//!
//! Measured (2026-10-31): on the PHY (nextpnr placement seed 2, oxibit-pnr
//! 40 iterations) overused nodes at the end fall from 660 to 263, flop D
//! inputs needing a bypass pin from 5,412 to 1,788; 3,611 LUT1s added.
//! Checked beyond the unit test by the ABC path on the emitted netlist
//! (srl, ffmerge, bram off so flops correspond) over the corpus, dfeed on
//! against off: 372 designs, 260,064 / 260,065 points proven, none failed.

use std::collections::{BTreeMap, HashMap};

use oxibit_hdl::netlist::{Bit, Cell, Dir, Module, Param, SigSpec};
use oxibit_prof::hot;

use crate::lib_ports::PrimPorts;

#[derive(Debug, Default, Clone)]
pub struct DFeedStats {
    pub buffered: usize,
    /// Buffered flops by the kind of cell driving D.
    pub by_source: BTreeMap<String, usize>,
}

/// Cell types whose outputs reach a flop D without a bypass pin when packed
/// in the same slice.
fn internal_source(ty: &str) -> bool {
    ty.starts_with("LUT")
        || matches!(
            ty,
            "CARRY4" | "SRLC32E" | "SRL16E" | "RAM32M" | "RAM64M" | "RAM128X1D" | "MUXF7" | "MUXF8"
        )
}

fn is_flop(ty: &str) -> bool {
    matches!(ty, "FDRE" | "FDSE" | "FDCE" | "FDPE")
}

#[hot]
pub fn insert(m: &mut Module, dirs: &PrimPorts) -> DFeedStats {
    let mut st = DFeedStats::default();
    // Driver kind of every bit an instance output drives.
    let mut drv: HashMap<Bit, String> = HashMap::new();
    for c in &m.cells {
        if let Cell::Instance { ty, conns, .. } = c {
            let Some(pd) = dirs.get(ty) else { continue };
            for (p, s) in conns {
                if pd.get(p).is_some_and(|x| x.dir != Dir::Input) {
                    for b in &s.0 {
                        drv.insert(*b, ty.clone());
                    }
                }
            }
        }
    }
    let inputs: Vec<(oxibit_hdl::netlist::SigId, u32)> = m
        .signals
        .iter()
        .enumerate()
        .filter(|(_, s)| s.port == Some(Dir::Input))
        .map(|(i, s)| (oxibit_hdl::netlist::SigId(i as u32), s.width))
        .collect();
    for (id, w) in inputs {
        for i in 0..w {
            drv.entry(Bit::Sig(id, i)).or_insert_with(|| "port".into());
        }
    }
    let mut new = Vec::new();
    let mut k = 0;
    for ci in 0..m.cells.len() {
        let (d, name) = match &m.cells[ci] {
            Cell::Instance { ty, name, conns, .. } if is_flop(ty) && name.starts_with("oxsyn_") => match conns.get("D") {
                Some(s) if s.0.len() == 1 => (s.0[0], name.clone()),
                _ => continue,
            },
            _ => continue,
        };
        let Some(src) = drv.get(&d) else { continue };
        if internal_source(src) {
            continue;
        }
        let src = src.clone();
        let w = m.wire(1).0[0];
        new.push(Cell::Instance {
            ty: "LUT1".into(),
            name: format!("oxsyn_dfeed{k}${}", name.trim_start_matches("oxsyn_")),
            params: BTreeMap::from([("INIT".to_string(), Param::Bits(vec![Some(false), Some(true)]))]),
            conns: BTreeMap::from([("I0".to_string(), SigSpec(vec![d])), ("O".to_string(), SigSpec(vec![w]))]),
        });
        if let Cell::Instance { conns, .. } = &mut m.cells[ci] {
            conns.insert("D".into(), SigSpec(vec![w]));
        }
        *st.by_source.entry(src).or_default() += 1;
        st.buffered += 1;
        k += 1;
    }
    m.cells.extend(new);
    st
}

pub struct DFeed<'a> {
    pub dirs: &'a PrimPorts,
}

impl crate::stage::Pass for DFeed<'_> {
    const NAME: &'static str = "dfeed";
    const EMITS: &'static [crate::caps::Mode] = &[crate::caps::Mode::Lut];
    type Stats = DFeedStats;
    fn enabled(p: &crate::policy::PassSet) -> bool {
        p.dfeed
    }
    fn json(s: &DFeedStats) -> serde_json::Value {
        serde_json::json!({ "buffered": s.buffered, "by_source": s.by_source })
    }
}

impl crate::stage::PhysicalPass for DFeed<'_> {
    const EVIDENCE: crate::policy::PostCheckEvidence = crate::policy::PostCheckEvidence::Inspection {
        tests: &["dfeed::tests::buffers_non_lut_sources"],
    };
    fn run(&self, m: &mut crate::stage::Physical) -> DFeedStats {
        insert(&mut m.0, self.dirs)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::lib_ports::Port;

    fn inst(ty: &str, name: &str, conns: &[(&str, Bit)]) -> Cell {
        Cell::Instance {
            ty: ty.into(),
            name: name.into(),
            params: BTreeMap::new(),
            conns: conns.iter().map(|(p, b)| (p.to_string(), SigSpec(vec![*b]))).collect(),
        }
    }

    #[test]
    fn buffers_non_lut_sources() {
        let mut m = Module::new("t");
        let w = m.wire(4);
        let mut dirs = PrimPorts::new();
        for (ty, outs) in [("FDRE", vec!["Q"]), ("LUT1", vec!["O"])] {
            let mut p = HashMap::new();
            for pin in ["D", "C", "CE", "R", "I0"] {
                p.insert(
                    pin.to_string(),
                    Port {
                        dir: Dir::Input,
                        width: Some(1),
                    },
                );
            }
            for o in outs {
                p.insert(
                    o.to_string(),
                    Port {
                        dir: Dir::Output,
                        width: Some(1),
                    },
                );
            }
            dirs.insert(ty.to_string(), p);
        }
        // a: Q -> b.D (flop to flop: buffered); LUT -> c.D (not buffered).
        m.cells.push(inst("FDRE", "oxsyn_a", &[("D", Bit::One), ("Q", w.0[0])]));
        m.cells.push(inst("FDRE", "oxsyn_b", &[("D", w.0[0]), ("Q", w.0[1])]));
        m.cells.push(inst("LUT1", "oxsyn_l", &[("I0", w.0[1]), ("O", w.0[2])]));
        m.cells.push(inst("FDRE", "oxsyn_c", &[("D", w.0[2]), ("Q", w.0[3])]));
        let st = insert(&mut m, &dirs);
        assert_eq!(st.buffered, 1);
        assert_eq!(st.by_source["FDRE"], 1);
        let Cell::Instance { conns, .. } = &m.cells[1] else { panic!() };
        assert_ne!(conns["D"].0[0], w.0[0]);
    }
}
