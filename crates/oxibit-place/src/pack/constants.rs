//! Constant pins (nextpnr-xilinx `pack_constants`, pin lists from its
//! `pins.cc`, 7-series primitives only).
//!
//! Every input on a constant ends on the GND or VCC net. A pin left
//! unconnected that the primitive needs at a fixed value (`tied`) is tied to
//! it. A pin tied to zero that has an `IS_<pin>_INVERTED` parameter is tied
//! to VCC with the inversion set instead: VCC is the easier net to route.

use std::collections::BTreeSet;

use serde_json::Value;

use super::Packer;
use crate::netlist::{CellId, Dir};

/// A parameter value as nextpnr writes an integer `Property`: a binary
/// string, `width` bits.
pub fn prop(v: u64, width: usize) -> Value {
    Value::String((0..width).rev().map(|i| if i < 64 && v >> i & 1 == 1 { '1' } else { '0' }).collect())
}

/// `prop(v, 32)`, nextpnr's default width.
pub fn prop32(v: u64) -> Value {
    prop(v, 32)
}

fn bus(base: &str, n: usize) -> Vec<String> {
    (0..n).map(|i| format!("{base}[{i}]")).collect()
}

/// Pins with an `IS_<pin>_INVERTED` parameter.
pub fn invertible(ty: &str, pin: &str) -> bool {
    let list: &[&str] = match ty {
        "BUFGCTRL" => &["CE0", "CE1", "S0", "S1", "IGNORE0", "IGNORE1"],
        "BUFHCE" => &["CE"],
        "FDRE" | "FDSE" | "FDCE" | "FDPE" => &["C"],
        "SRL16E" | "SRLC32E" => &["CLK"],
        "RAM128X1D" | "RAM128X1S" | "RAM256X1S" | "RAM32M" | "RAM32X1D" | "RAM32X1S" | "RAM32X2S" | "RAM64M" | "RAM64X1D" | "RAM64X1S" => &["WCLK"],
        "RAMB18E1" | "RAMB36E1" => &[
            "CLKARDCLK",
            "CLKBWRCLK",
            "ENARDEN",
            "ENBWREN",
            "RSTRAMARSTRAM",
            "RSTRAMB",
            "RSTREGARSTREG",
            "RSTREGB",
        ],
        "FIFO18E1" | "FIFO36E1" => &["RDCLK", "RDEN", "RSTREG", "RST", "WRCLK", "WREN"],
        "IDDR" => &["C", "CK"],
        "IDDR_2CLK" => &["CB", "C"],
        "ODDR" => &["CK", "D1", "D2"],
        "IDELAYE2" => &["C", "IDATAIN"],
        "ODELAYE2" => &["C", "ODATAIN"],
        "ISERDESE2" => &["CLKB", "CLKDIVP", "CLKDIV", "CLK", "OCLKB", "OCLK"],
        "OSERDESE2" => &["CLKDIV", "CLK", "T1", "T2", "T3", "T4"],
        "LDCE" => &["CLR", "G"],
        "LDPE" => &["G", "PRE"],
        "MMCME2_ADV" => &["CLKINSEL", "PSEN", "PSINCDEC", "PWRDWN", "RST"],
        "PLLE2_ADV" => &["CLKINSEL", "PWRDWN", "RST"],
        "XADC" => &["CONVSTCLK", "DCLK"],
        "BUFMRCE" => &["CE"],
        "DSP48E1" => {
            return pin == "CARRYIN"
                || [("ALUMODE[", 4), ("INMODE[", 5), ("OPMODE[", 7)].iter().any(|(b, n)| {
                    pin.strip_prefix(b)
                        .and_then(|r| r.strip_suffix(']'))
                        .and_then(|i| i.parse::<usize>().ok())
                        .is_some_and(|i| i < *n)
                });
        }
        _ => &[],
    };
    list.contains(&pin)
}

/// Pins tied to a fixed value when left unconnected, with the value.
pub fn tied(ty: &str) -> Vec<(String, bool)> {
    let mut v: Vec<(String, bool)> = Vec::new();
    let mut add = |p: &str, x: bool| v.push((p.to_string(), x));
    match ty {
        "RAMB18E1" | "RAMB36E1" => {
            let wea = if ty == "RAMB18E1" { 2 } else { 4 };
            for p in bus("WEA", wea) {
                add(&p, true);
            }
            for p in bus("WEBWE", 4) {
                add(&p, true);
            }
            for (p, x) in [
                ("CLKARDCLK", false),
                ("CLKBWRCLK", false),
                ("ENARDEN", false),
                ("ENBWREN", false),
                ("REGCEAREGCE", true),
                ("REGCEB", true),
                ("RSTRAMARSTRAM", false),
                ("RSTRAMB", false),
                ("RSTREGARSTREG", false),
                ("RSTREGB", false),
            ] {
                add(p, x);
            }
        }
        "BUFGCTRL" => {
            for i in 0..2 {
                add(&format!("S{i}"), false);
                add(&format!("IGNORE{i}"), false);
                add(&format!("CE{i}"), false);
            }
        }
        "OSERDESE2" => {
            add("RST", false);
            for i in 1..=8 {
                add(&format!("D{i}"), false);
            }
            for i in 1..=4 {
                add(&format!("T{i}"), false);
            }
            add("OCE", true);
            add("TCE", true);
        }
        "IDELAYE2" => {
            for p in ["REGRST", "LDPIPEEN", "CINVCTRL"] {
                add(p, false);
            }
        }
        "PLLE2_ADV" => {
            for p in ["CLKFBIN", "CLKIN1", "CLKIN2", "DCLK", "DEN", "DWE", "PWRDWN", "RST"] {
                add(p, false);
            }
            add("CLKINSEL", true);
            for p in bus("DADDR", 7).into_iter().chain(bus("DI", 16)) {
                add(&p, false);
            }
        }
        "DSP48E1" => {
            for p in [
                "CLK",
                "RSTA",
                "RSTALLCARRYIN",
                "RSTALUMODE",
                "RSTB",
                "RSTC",
                "RSTCTRL",
                "RSTD",
                "RSTINMODE",
                "RSTM",
                "RSTP",
                "CARRYIN",
                "CEA1",
                "CEA2",
                "CEAD",
                "CEALUMODE",
                "CEB1",
                "CEB2",
                "CEC",
                "CECARRYIN",
                "CECTRL",
                "CED",
                "CEINMODE",
                "CEM",
                "CEP",
            ] {
                add(p, false);
            }
            for (b, n) in [
                ("A", 30),
                ("B", 18),
                ("C", 48),
                ("D", 25),
                ("ALUMODE", 4),
                ("CARRYINSEL", 3),
                ("INMODE", 5),
                ("OPMODE", 7),
            ] {
                for p in bus(b, n) {
                    add(&p, false);
                }
            }
        }
        _ => {}
    }
    v
}

/// Tie constant pins. `first`: also move the inputs on the netlist's
/// constant nets through the inversion rule (nextpnr does this for the
/// front end's GND/VCC cells, once; later calls only fill tied pins).
pub fn pack_constants_with(p: &mut Packer, first: bool) {
    let mut todo: Vec<(CellId, String, bool)> = Vec::new();
    for c in p.nl.cell_ids() {
        let cell = p.nl.cell(c);
        for (port, val) in tied(&cell.ty) {
            let driven = cell.net(&port).is_some_and(|n| p.nl.driver(n).is_some());
            if !driven {
                todo.push((c, port, val));
            }
        }
        if first {
            let (gnd, vcc) = (p.nl.gnd, p.nl.vcc);
            for (port, x) in &cell.ports {
                if x.dir != Dir::Input {
                    continue;
                }
                if x.net == Some(gnd) {
                    todo.push((c, port.clone(), false));
                } else if x.net == Some(vcc) {
                    todo.push((c, port.clone(), true));
                }
            }
        }
    }
    let mut seen: BTreeSet<(CellId, String)> = BTreeSet::new();
    let mut inverted = 0;
    for (c, port, mut val) in todo {
        if !seen.insert((c, port.clone())) {
            continue;
        }
        p.nl.add_port(c, &port, Dir::Input);
        let ty = p.nl.cell(c).ty.clone();
        if !val && invertible(&ty, &port) {
            p.nl.cell_mut(c).params.insert(format!("IS_{port}_INVERTED"), prop32(1));
            val = true;
            inverted += 1;
        }
        let n = if val { p.nl.vcc } else { p.nl.gnd };
        p.nl.connect(c, &port, n);
    }
    p.count("constants.inverted_to_vcc", inverted);
}

pub fn pack_constants(p: &mut Packer) {
    pack_constants_with(p, true);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn props_and_pin_lists() {
        assert_eq!(prop32(1), Value::String(format!("{}1", "0".repeat(31))));
        assert_eq!(prop(1, 2), Value::String("01".into()));
        assert!(invertible("DSP48E1", "OPMODE[6]"));
        assert!(!invertible("DSP48E1", "OPMODE[7]"));
        assert!(invertible("FDRE", "C"));
        assert!(tied("BUFGCTRL").contains(&("CE0".to_string(), false)));
    }
}
