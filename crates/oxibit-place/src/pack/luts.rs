//! LUTs and inverters (nextpnr-xilinx `pack_luts`, `pack_inverters`).

use std::collections::BTreeMap;

use super::constants::prop;
use super::{Packer, Rule};

/// `INV` -> `LUT1` with INIT `01` (the CMT pin folding nextpnr also does
/// here is for PLL/MMCM control pins; none in the designs this packs yet).
pub fn pack_inverters(p: &mut Packer) {
    let mut n = 0;
    for c in p.nl.cells_of("INV") {
        p.nl.cell_mut(c).params.insert("INIT".into(), prop(1, 2));
        p.nl.rename_port(c, "I", "I0");
        p.nl.cell_mut(c).ty = "LUT1".into();
        n += 1;
    }
    p.count("inverters", n);
}

/// `LUT1`..`LUT6` -> `SLICE_LUTX`: `I<k>` on `A<k+1>`, `O` on `O6`.
pub fn pack_luts(p: &mut Packer) {
    let mut rules = BTreeMap::new();
    for k in 1..=6 {
        let mut r = Rule::to("SLICE_LUTX").port("O", "O6");
        for i in 0..k {
            r = r.port(&format!("I{i}"), &format!("A{}", i + 1));
        }
        rules.insert(format!("LUT{k}"), r);
    }
    let n = p.generic_xform(&rules);
    p.count("luts", n);
}
