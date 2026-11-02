//! Wide multiplexers (nextpnr-xilinx `constrain_muxf_tree`,
//! `finalise_muxfs`). User MUXF7/MUXF8 trees (`pack_muxfs`) are not
//! packed yet: oxibit-synth emits none, and they are counted if seen.

use std::collections::BTreeMap;

use super::{Offset, Packer, Rule, slot};
use crate::netlist::CellId;

fn abs_z(p: &Packer, c: CellId) -> Option<i32> {
    let root = *p.cluster_of.get(&c)?;
    if root == c {
        return p.clusters[&root].root_abs_z;
    }
    p.clusters[&root]
        .members
        .iter()
        .find(|(m, _)| *m == c)
        .and_then(|(_, o)| o.abs.then_some(o.z))
}

/// Put the tree under `curr` in `base`'s cluster: a MUXF7 at the F7 slot of
/// its letter pair, a MUXF8 at F8, leaf LUTs at the 6LUTs; I0 leaves go
/// `spacing` letters above I1 leaves. LUTs already placed by a cluster are
/// left where they are.
pub fn constrain_muxf_tree(p: &mut Packer, curr: CellId, base: CellId, zoffset: i32) {
    let cty = p.nl.cell(curr).ty.clone();
    if cty == "SLICE_LUTX" && p.is_constrained(curr) {
        return;
    }
    let base_z = match p.nl.cell(base).ty.as_str() {
        "MUXF7" => slot::F7MUX,
        "MUXF8" => slot::F8MUX,
        _ => abs_z(p, base).unwrap_or(0),
    };
    let (f, spacing) = match cty.as_str() {
        "MUXF7" => (slot::F7MUX, 1),
        "MUXF8" => (slot::F8MUX, 2),
        _ => (slot::LUT6, 0),
    };
    let curr_z = zoffset * 16 + f;
    if curr != base {
        p.add_member(
            base,
            curr,
            Offset {
                dx: 0,
                dy: 0,
                z: curr_z - base_z,
                abs: false,
            },
        );
    } else {
        p.make_root(base, None);
    }
    if cty == "MUXF7" || cty == "MUXF8" {
        for (port, off) in [("I0", spacing), ("I1", 0)] {
            if let Some(n) = p.nl.cell(curr).net(port)
                && let Some((d, _)) = p.nl.driver(n)
            {
                constrain_muxf_tree(p, d, base, zoffset + off);
            }
        }
    }
}

/// MUXF7/MUXF8 -> `SELMUX2_1`: inputs on `0` / `1`, select on `S0`, output
/// on `OUT`.
pub fn finalise_muxfs(p: &mut Packer) {
    let r = Rule::to("SELMUX2_1").port("I0", "0").port("I1", "1").port("S", "S0").port("O", "OUT");
    let rules: BTreeMap<String, Rule> = [("MUXF7".to_string(), r.clone()), ("MUXF8".to_string(), r)].into_iter().collect();
    let n = p.generic_xform(&rules);
    p.count("muxf", n);
}
