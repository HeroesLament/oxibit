//! Global clock buffers and the PS7 (nextpnr-xilinx `prepare_clocking`,
//! `pack_gbs`, and the PS7 rule of `pack_io`).
//!
//! `BUFG` -> `BUFGCTRL` with I on I0 and the controls tied for a plain
//! buffer (CE0, S0, IGNORE0 high; S1 low as VCC inverted); the second
//! constant pass ties the remaining controls. Clock buffer and PS7 bels are
//! chosen by the placer.

use std::collections::BTreeMap;

use super::{Packer, Rule};

pub fn prepare_clocking(p: &mut Packer) {
    let mut n = 0;
    for c in p.nl.cell_ids() {
        let ty = p.nl.cell(c).ty.clone();
        match ty.as_str() {
            "MMCME2_BASE" => p.nl.cell_mut(c).ty = "MMCME2_ADV".into(),
            "PLLE2_BASE" => p.nl.cell_mut(c).ty = "PLLE2_ADV".into(),
            "BUFG" | "BUFGCE" => {
                p.nl.cell_mut(c).ty = "BUFGCTRL".into();
                p.nl.rename_port(c, "I", "I0");
                if ty == "BUFG" {
                    p.tie_port(c, "CE0", true, true);
                } else {
                    p.nl.rename_port(c, "CE", "CE0");
                }
                p.tie_port(c, "S0", true, true);
                p.tie_port(c, "S1", false, true);
                p.tie_port(c, "IGNORE0", true, true);
                n += 1;
            }
            _ => {}
        }
    }
    p.count("clock_buffers", n);
}

pub fn pack_gbs(p: &mut Packer) {
    let rules: BTreeMap<String, Rule> = [("BUFGCTRL".to_string(), Rule::to("BUFGCTRL")), ("PS7".to_string(), Rule::to("PS7_PS7"))]
        .into_iter()
        .collect();
    p.generic_xform(&rules);
}
