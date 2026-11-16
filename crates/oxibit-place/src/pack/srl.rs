//! Shift registers (nextpnr-xilinx `pack_srls`).
//!
//! `SRL16E` / `SRLC32E` become `SLICE_LUTX` cells in SRL mode: `CE` on `WE`,
//! `D` on `DI2` / `DI1`, `Q` on `O6`, `Q31` on `MC31`, the address moved up
//! one pin (`A[0]` on `A2`, ...) with `A1` (and for SRL16E `A6`) tied high.
//! Q31 cascades (shift registers deeper than 32 kept in one slice) are not
//! clustered yet: a design that uses `Q31` is reported, not packed wrong.

use std::collections::BTreeMap;

use serde_json::json;

use super::{Packer, Rule};

pub fn pack_srls(p: &mut Packer) {
    let mut rules = BTreeMap::new();
    rules.insert(
        "SRL16E".to_string(),
        Rule::to("SLICE_LUTX")
            .port("CLK", "CLK")
            .port("CE", "WE")
            .port("D", "DI2")
            .port("Q", "O6")
            .attr("X_LUT_AS_SRL", json!("1")),
    );
    rules.insert(
        "SRLC32E".to_string(),
        Rule::to("SLICE_LUTX")
            .port("CLK", "CLK")
            .port("CE", "WE")
            .port("D", "DI1")
            .port("Q", "O6")
            .port("Q31", "MC31")
            .attr("X_LUT_AS_SRL", json!("1")),
    );
    let n = p.generic_xform(&rules);
    let vcc = p.nl.vcc;
    let mut cascades = 0;
    for c in p.nl.cells_of("SLICE_LUTX") {
        let orig = p.nl.cell(c).str_attr("X_ORIG_TYPE").map(str::to_string);
        let (top, ties): (usize, &[&str]) = match orig.as_deref() {
            Some("SRL16E") => (3, &["A1", "A6"]),
            Some("SRLC32E") => (4, &["A1"]),
            _ => continue,
        };
        for i in (0..=top).rev() {
            p.nl.rename_port(c, &format!("A{i}"), &format!("A{}", i + 2));
        }
        // As nextpnr: the X_ORIG_PORT markers keep the pre-shift names.
        for t in ties {
            p.nl.add_port(c, t, crate::netlist::Dir::Input);
            p.nl.connect(c, t, vcc);
        }
        if p.nl.cell(c).net("MC31").is_some_and(|n| !p.nl.net(n).users.is_empty()) {
            cascades += 1;
        }
    }
    p.count("srls", n);
    p.count("srl.q31_unhandled", cascades);
}
