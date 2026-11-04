//! DSP48E1 (nextpnr-xilinx `pack_dsps`).
//!
//! `DSP48E1` -> `DSP48E1_DSP48E1`, bus pins flattened. Constant ties on the
//! pins the tile can hold at a constant itself (D, RSTD, CARRYINSEL2, CED,
//! CEAD, CEINMODE, CEALUMODE) come off the routing and are listed in
//! `DSP_GND_PINS` / `DSP_VCC_PINS`; a cascade input on GND is disconnected.
//! A PCOUT -> PCIN pair (oxibit-synth's cascaded multiplier rows) is
//! counted here and placed by `placer` as one object on both sites of a DSP
//! tile. Other cascades (ACOUT/BCOUT, CARRYCASCOUT, MULTSIGNOUT, chains
//! longer than a pair) are counted as unhandled.

use std::collections::BTreeMap;

use serde_json::json;

use super::{Packer, Rule};

fn cascade_in(p: &str) -> bool {
    p == "CARRYCASCIN"
        || p == "MULTSIGNIN"
        || p.starts_with("ACIN")
        || p.starts_with("BCIN")
        || p.starts_with("PCIN")
}

fn cascade_out(p: &str) -> bool {
    p == "CARRYCASCOUT"
        || p == "MULTSIGNOUT"
        || p.starts_with("ACOUT")
        || p.starts_with("BCOUT")
        || p.starts_with("PCOUT")
}

fn tile_constant(p: &str) -> bool {
    [
        "D",
        "RSTD",
        "CARRYINSEL2",
        "CED",
        "CEAD",
        "CEINMODE",
        "CEALUMODE",
    ]
    .iter()
    .any(|s| p.starts_with(s))
}

pub fn pack_dsps(p: &mut Packer) {
    let rules: BTreeMap<String, Rule> = [("DSP48E1".to_string(), Rule::to("DSP48E1_DSP48E1"))]
        .into_iter()
        .collect();
    let n = p.generic_xform(&rules);
    let (gnd, vcc) = (p.nl.gnd, p.nl.vcc);
    let mut cascades = 0;
    let mut pc_bits = 0;
    for c in p.nl.cells_of("DSP48E1_DSP48E1") {
        let mut gnd_pins =
            p.nl.cell(c)
                .str_attr("DSP_GND_PINS")
                .unwrap_or("")
                .to_string();
        let mut vcc_pins =
            p.nl.cell(c)
                .str_attr("DSP_VCC_PINS")
                .unwrap_or("")
                .to_string();
        let ports: Vec<(String, Option<crate::netlist::NetId>)> =
            p.nl.cell(c)
                .ports
                .iter()
                .map(|(k, v)| (k.clone(), v.net))
                .collect();
        for (port, net) in ports {
            if cascade_in(&port) && net == Some(gnd) {
                p.nl.disconnect(c, &port);
                continue;
            }
            if cascade_out(&port) && net.is_some_and(|n| !p.nl.net(n).users.is_empty()) {
                if port.starts_with("PCOUT") {
                    pc_bits += 1;
                } else {
                    cascades += 1;
                }
            }
            if tile_constant(&port) {
                if net == Some(gnd) {
                    p.nl.disconnect(c, &port);
                    gnd_pins.push_str(&format!(" {port}"));
                } else if net == Some(vcc) {
                    p.nl.disconnect(c, &port);
                    vcc_pins.push_str(&format!(" {port}"));
                }
            }
        }
        p.nl.cell_mut(c)
            .attrs
            .insert("DSP_GND_PINS".into(), json!(gnd_pins));
        p.nl.cell_mut(c)
            .attrs
            .insert("DSP_VCC_PINS".into(), json!(vcc_pins));
    }
    p.count("dsp", n);
    p.count("dsp.cascade_unhandled", cascades);
    p.count("dsp.pcout_bits", pc_bits);
}
