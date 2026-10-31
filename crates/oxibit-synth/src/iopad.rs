//! I/O buffers on top-level ports (yosys `iopadmap` as `synth_xilinx`
//! runs it): IBUF on each input bit and OBUF on each output bit that the
//! design does not already connect to a pad pin (IBUFDS.I/IB, OBUFDS.O/OB,
//! IOBUF.IO, ...) or to the PS7, whose pins are dedicated pads. nextpnr
//! wants every port bit on an I/O buffer. Inout ports without a pad
//! primitive are reported, not buffered (that needs the tristate).
//! Runs after the checking view is taken: a buffer is the identity.

use std::collections::{BTreeMap, HashMap, HashSet};

use oxibit_hdl::netlist::{Bit, Cell, Dir, Module, SigId, SigSpec};
use oxibit_prof::hot;

#[derive(Debug, Default, Clone)]
pub struct IoPadStats {
    pub ibuf: usize,
    pub obuf: usize,
    /// Port bits already on a pad pin or the PS7.
    pub padded: usize,
    /// Inout bits with no pad primitive.
    pub inout_bare: Vec<String>,
}

/// Pins that sit on a package pad.
fn pad_pin(ty: &str, pin: &str) -> bool {
    matches!(
        (ty, pin),
        ("IBUF" | "IBUFG" | "IBUF_IBUFDISABLE" | "IBUF_INTERMDISABLE", "I")
            | ("IBUFDS" | "IBUFGDS" | "IBUFDS_DIFF_OUT" | "IBUFDS_GTE2", "I" | "IB")
            | ("OBUF" | "OBUFT", "O")
            | ("OBUFDS" | "OBUFTDS", "O" | "OB")
            | ("IOBUF" | "IOBUF_INTERMDISABLE", "IO")
            | ("IOBUFDS" | "IOBUFDS_DIFF_OUT", "IO" | "IOB")
    ) || ty == "PS7"
}

fn rewrite(s: &mut SigSpec, map: &HashMap<Bit, Bit>) {
    for b in s.0.iter_mut() {
        if let Some(t) = map.get(b) {
            *b = *t;
        }
    }
}

fn buf(ty: &str, name: String, i: Bit, o: Bit) -> Cell {
    Cell::Instance {
        ty: ty.into(),
        name,
        params: BTreeMap::new(),
        conns: BTreeMap::from([("I".to_string(), SigSpec(vec![i])), ("O".to_string(), SigSpec(vec![o]))]),
    }
}

#[hot]
pub fn insert(m: &mut Module) -> IoPadStats {
    let mut st = IoPadStats::default();
    // Bits on pad pins (either side of a pad primitive's pad pin).
    let mut padded: HashSet<Bit> = HashSet::new();
    for c in &m.cells {
        if let Cell::Instance { ty, conns, .. } = c {
            for (pin, s) in conns {
                if pad_pin(ty, pin) {
                    padded.extend(s.0.iter().copied());
                }
            }
        }
    }
    let ports: Vec<(SigId, String, u32, Dir)> = m
        .signals
        .iter()
        .enumerate()
        .filter_map(|(i, s)| s.port.map(|d| (SigId(i as u32), s.name.clone(), s.width, d)))
        .collect();
    let mut new_cells = Vec::new();
    // Inputs: IBUF, readers move to its output.
    let mut in_map: HashMap<Bit, Bit> = HashMap::new();
    for (id, name, w, d) in &ports {
        for i in 0..*w {
            let p = Bit::Sig(*id, i);
            match d {
                Dir::Input if padded.contains(&p) => st.padded += 1,
                Dir::Input => {
                    let o = m.wire(1).0[0];
                    new_cells.push(buf("IBUF", format!("oxsyn_ibuf${name}[{i}]"), p, o));
                    in_map.insert(p, o);
                    st.ibuf += 1;
                }
                Dir::Inout if padded.contains(&p) => st.padded += 1,
                Dir::Inout => st.inout_bare.push(format!("{name}[{i}]")),
                Dir::Output => {}
            }
        }
    }
    if !in_map.is_empty() {
        for c in &mut m.cells {
            match c {
                Cell::Instance { conns, .. } => conns.values_mut().for_each(|s| rewrite(s, &in_map)),
                Cell::Unary { a, .. } => rewrite(a, &in_map),
                Cell::Binary { a, b, .. } => {
                    rewrite(a, &in_map);
                    rewrite(b, &in_map);
                }
                Cell::Mux { a, b, s, .. } => {
                    rewrite(a, &in_map);
                    rewrite(b, &in_map);
                    rewrite(s, &in_map);
                }
                Cell::Dff { clk, d, .. } => {
                    rewrite(clk, &in_map);
                    rewrite(d, &in_map);
                }
                Cell::Adff { clk, arst, d, .. } => {
                    rewrite(clk, &in_map);
                    rewrite(arst, &in_map);
                    rewrite(d, &in_map);
                }
                Cell::Mem(mem) => {
                    for r in &mut mem.reads {
                        rewrite(&mut r.addr, &in_map);
                        if let Some((c, _)) = &mut r.clk {
                            rewrite(c, &in_map);
                        }
                    }
                    for wr in &mut mem.writes {
                        rewrite(&mut wr.clk.0, &in_map);
                        rewrite(&mut wr.addr, &in_map);
                        rewrite(&mut wr.data, &in_map);
                        rewrite(&mut wr.en, &in_map);
                    }
                }
            }
        }
        for (_, r) in &mut m.connects {
            rewrite(r, &in_map);
        }
    }
    // Outputs: a port bit assigned from a net gets an OBUF instead of the
    // assignment, unless that net is a pad pin already.
    let out_ports: HashMap<SigId, String> = ports.iter().filter(|p| p.3 == Dir::Output).map(|p| (p.0, p.1.clone())).collect();
    let mut keep = Vec::with_capacity(m.connects.len());
    for (l, r) in std::mem::take(&mut m.connects) {
        let (mut kl, mut kr) = (Vec::new(), Vec::new());
        for (p, n) in l.0.iter().zip(&r.0) {
            let Bit::Sig(id, i) = p else {
                kl.push(*p);
                kr.push(*n);
                continue;
            };
            let Some(name) = out_ports.get(id) else {
                kl.push(*p);
                kr.push(*n);
                continue;
            };
            if padded.contains(n) || padded.contains(p) {
                st.padded += 1;
                kl.push(*p);
                kr.push(*n);
                continue;
            }
            new_cells.push(buf("OBUF", format!("oxsyn_obuf${name}[{i}]"), *n, *p));
            st.obuf += 1;
        }
        if !kl.is_empty() {
            keep.push((SigSpec(kl), SigSpec(kr)));
        }
    }
    m.connects = keep;
    m.cells.extend(new_cells);
    st
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxibit_hdl::netlist::Signal;

    fn port(m: &mut Module, name: &str, w: u32, d: Dir) -> SigSpec {
        let id = m.add_signal(Signal {
            name: name.into(),
            width: w,
            signed: false,
            port: Some(d),
            init: None,
        });
        SigSpec::of(id, w)
    }

    fn count(m: &Module, t: &str) -> usize {
        m.cells.iter().filter(|c| matches!(c, Cell::Instance { ty, .. } if ty == t)).count()
    }

    #[test]
    fn buffers_bare_ports_only() {
        let mut m = Module::new("t");
        let a = port(&mut m, "a", 2, Dir::Input);
        let dp = port(&mut m, "dp", 1, Dir::Input);
        let dn = port(&mut m, "dn", 1, Dir::Input);
        let y = port(&mut m, "y", 2, Dir::Output);
        let w = m.wire(1);
        m.cells.push(Cell::Instance {
            ty: "IBUFDS".into(),
            name: "u".into(),
            params: BTreeMap::new(),
            conns: BTreeMap::from([
                ("I".to_string(), dp.clone()),
                ("IB".to_string(), dn.clone()),
                ("O".to_string(), w.clone()),
            ]),
        });
        // y = {w, a[0]}
        m.connects.push((y.clone(), SigSpec(vec![a.0[0], w.0[0]])));
        let st = insert(&mut m);
        assert_eq!((st.ibuf, st.obuf, st.padded), (2, 2, 2));
        assert_eq!((count(&m, "IBUF"), count(&m, "OBUF")), (2, 2));
        assert!(m.connects.is_empty());
        // The OBUF for y[0] reads a[0]'s IBUF output, not the port.
        let obuf_in: Vec<Bit> = m
            .cells
            .iter()
            .filter_map(|c| match c {
                Cell::Instance { ty, conns, .. } if ty == "OBUF" => Some(conns["I"].0[0]),
                _ => None,
            })
            .collect();
        assert!(!obuf_in.contains(&a.0[0]));
    }
}
