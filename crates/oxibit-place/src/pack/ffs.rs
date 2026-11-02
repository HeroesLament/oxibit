//! Flops and LUT-flop pairing (nextpnr-xilinx `pack_ffs`, `pack_lutffs`,
//! `can_add_ff_to_cluster`).

use std::collections::BTreeMap;

use serde_json::json;

use super::constants::prop32;
use super::{Offset, Packer, Rule, slot};
use crate::netlist::{CellId, NetId};

/// `FDRE`/`FDSE`/`FDCE`/`FDPE` (and the `_1` clock-inverted forms) ->
/// `SLICE_FFX`: `C` on `CK`, the reset or set on `SR`; synchronous ones get
/// `X_FFSYNC`.
pub fn pack_ffs(p: &mut Packer) {
    let mut rules = BTreeMap::new();
    for (ty, sr, sync) in [("FDCE", "CLR", false), ("FDPE", "PRE", false), ("FDRE", "R", true), ("FDSE", "S", true)] {
        let mut r = Rule::to("SLICE_FFX").port("C", "CK").port(sr, "SR");
        if sync {
            r = r.attr("X_FFSYNC", json!("1"));
        }
        rules.insert(format!("{ty}_1"), r.clone().param("IS_C_INVERTED", prop32(1)));
        rules.insert(ty.to_string(), r);
    }
    let n = p.generic_xform(&rules);
    p.count("ffs", n);
}

fn inverted_sr(p: &Packer, c: CellId) -> bool {
    let cell = p.nl.cell(c);
    ["IS_R_INVERTED", "IS_S_INVERTED", "IS_CLR_INVERTED", "IS_PRE_INVERTED"]
        .iter()
        .any(|k| cell.int_param(k).unwrap_or(0) != 0)
}

/// The control set a slice half holding `ff` would need.
#[derive(Debug, PartialEq)]
struct Ctrl {
    clk: Option<NetId>,
    ce: Option<NetId>,
    sr: Option<NetId>,
    clk_inv: bool,
    sr_inv: bool,
    latch: bool,
    sync: bool,
}

fn ctrl(p: &Packer, ff: CellId) -> Ctrl {
    let c = p.nl.cell(ff);
    Ctrl {
        clk: c.net("CK"),
        ce: c.net("CE"),
        sr: c.net("SR"),
        clk_inv: c.int_param("IS_C_INVERTED").unwrap_or(0) == 1,
        sr_inv: inverted_sr(p, ff),
        latch: c.attrs.contains_key("X_FF_AS_LATCH"),
        sync: c.attrs.contains_key("X_FFSYNC"),
    }
}

/// Whether `ff` can join the cluster of `anchor` (a LUT or CARRY4 already in
/// one) in the same tile: the clock must match any write clock or flop there,
/// and the whole control set must match a flop there.
fn can_add_ff_to_cluster(p: &Packer, anchor: CellId, anchor_off: (i32, i32), ff: CellId) -> bool {
    let root = p.cluster_of[&anchor];
    let mut clk = None;
    let mut found_wclk = false;
    let mut found_ff: Option<Ctrl> = None;
    let mut look = |c: CellId, off: (i32, i32)| {
        if off != anchor_off {
            return;
        }
        let cell = p.nl.cell(c);
        if cell.ty == "SLICE_LUTX" && cell.net("CLK").is_some() {
            found_wclk = true;
            clk = cell.net("CLK");
        }
        if cell.ty == "SLICE_FFX" {
            let k = ctrl(p, c);
            clk = k.clk;
            found_ff = Some(k);
        }
    };
    look(root, (0, 0));
    for (m, o) in &p.clusters[&root].members {
        look(*m, (o.dx, o.dy));
    }
    if !found_wclk && found_ff.is_none() {
        return true;
    }
    let mine = ctrl(p, ff);
    if mine.clk != clk {
        return false;
    }
    match found_ff {
        None => true,
        Some(k) => k == mine,
    }
}

/// Offset of `c` in its cluster ((0, 0) for a root).
fn offset_of(p: &Packer, c: CellId) -> Offset {
    let root = p.cluster_of[&c];
    if root == c {
        return Offset {
            dx: 0,
            dy: 0,
            z: p.clusters[&root].root_abs_z.unwrap_or(0),
            abs: p.clusters[&root].root_abs_z.is_some(),
        };
    }
    p.clusters[&root].members.iter().find(|(m, _)| *m == c).map(|(_, o)| *o).expect("member")
}

/// Pair each flop with the LUT (or carry output) driving its D, as nextpnr
/// does: a LUT not yet in a cluster takes the first flop it drives,
/// whatever its fanout; a LUT or CARRY4 already in a cluster takes a flop
/// only when the flop is the net's sole user and the control set fits.
pub fn pack_lutffs(p: &mut Packer) {
    let mut pairs = 0;
    for ff in p.nl.cell_ids() {
        if p.nl.cell(ff).ty != "SLICE_FFX" || p.is_constrained(ff) {
            continue;
        }
        let Some(d) = p.nl.cell(ff).net("D") else { continue };
        let Some((drv, port)) = p.nl.driver(d).map(|(c, s)| (c, s.to_string())) else {
            continue;
        };
        if p.nl.cell(ff).attrs.contains_key("BEL") || p.nl.cell(drv).attrs.contains_key("BEL") {
            continue;
        }
        let sole_user = p.nl.net(d).users.len() == 1;
        let dty = p.nl.cell(drv).ty.clone();
        if dty == "SLICE_LUTX" && port == "O6" {
            if p.is_constrained(drv) {
                let o = offset_of(p, drv);
                if !sole_user || !can_add_ff_to_cluster(p, drv, (o.dx, o.dy), ff) {
                    continue;
                }
                let root = p.cluster_of[&drv];
                p.add_member(
                    root,
                    ff,
                    Offset {
                        dx: o.dx,
                        dy: o.dy,
                        z: o.z + (slot::FF - slot::LUT6),
                        abs: o.abs,
                    },
                );
            } else {
                p.add_member(
                    drv,
                    ff,
                    Offset {
                        dx: 0,
                        dy: 0,
                        z: slot::FF - slot::LUT6,
                        abs: false,
                    },
                );
            }
            pairs += 1;
        } else if dty == "CARRY4" && p.is_constrained(drv) {
            let Some(z) = port.strip_prefix('O').filter(|s| s.len() == 1).and_then(|s| s.parse::<i32>().ok()) else {
                continue;
            };
            let o = offset_of(p, drv);
            if !sole_user || !can_add_ff_to_cluster(p, drv, (o.dx, o.dy), ff) {
                continue;
            }
            let root = p.cluster_of[&drv];
            p.add_member(
                root,
                ff,
                Offset {
                    dx: o.dx,
                    dy: o.dy,
                    z: o.z + (slot::at(z, slot::FF) - slot::CARRY4),
                    abs: o.abs,
                },
            );
            pairs += 1;
        }
    }
    p.count("lutff_pairs", pairs);
}
