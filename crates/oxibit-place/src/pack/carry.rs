//! Carry chains (nextpnr-xilinx `split_carry4s`, `pack_carries`).
//!
//! Every CARRY4 is split into four MUXCY/XORCY pairs, the pairs are chained
//! by their carry nets, and each chain is cut into CARRY4s again: one per
//! four positions, stacked down a column (`dy = -(i/4 + i/100)`, skipping
//! the tiles with no CARRY4). Each position's S and DI must come from a LUT
//! that can sit at that position (S on the 6LUT, DI on the 5LUT, five inputs
//! between them); otherwise a feed-through LUT1 carries the signal there.

use std::collections::{BTreeMap, BTreeSet, HashSet};

use super::constants::prop32;
use super::{Offset, Packer, Rule, slot};
use crate::netlist::{CellId, Dir, NetId};

fn create(p: &mut Packer, ty: &str, name: &str) -> CellId {
    let c = p.nl.add_cell(name, ty);
    let ports: &[(&str, Dir)] = match ty {
        "MUXCY" => &[("CI", Dir::Input), ("DI", Dir::Input), ("S", Dir::Input), ("O", Dir::Output)],
        "XORCY" => &[("CI", Dir::Input), ("LI", Dir::Input), ("O", Dir::Output)],
        "CARRY4" => &[],
        _ => &[],
    };
    for (n, d) in ports {
        p.nl.add_port(c, n, *d);
    }
    if ty == "CARRY4" {
        p.nl.add_port(c, "CI", Dir::Input);
        p.nl.add_port(c, "CYINIT", Dir::Input);
        for i in 0..4 {
            p.nl.add_port(c, &format!("DI[{i}]"), Dir::Input);
            p.nl.add_port(c, &format!("S[{i}]"), Dir::Input);
            p.nl.add_port(c, &format!("CO[{i}]"), Dir::Output);
            p.nl.add_port(c, &format!("O[{i}]"), Dir::Output);
        }
    }
    c
}

fn split_carry4s(p: &mut Packer) {
    for c in p.nl.cells_of("CARRY4") {
        let name = p.nl.cell(c).name.clone();
        let mut cin = p.nl.cell(c).net("CI");
        if cin.is_none() || cin == Some(p.nl.gnd) {
            cin = p.nl.cell(c).net("CYINIT");
        }
        p.nl.disconnect(c, "CI");
        p.nl.disconnect(c, "CYINIT");
        for i in 0..4 {
            let xorcy = create(p, "XORCY", &format!("{name}$split$xorcy{i}"));
            let muxcy = create(p, "MUXCY", &format!("{name}$split$muxcy{i}"));
            if let Some(n) = cin {
                p.nl.connect(muxcy, "CI", n);
                p.nl.connect(xorcy, "CI", n);
            }
            p.nl.move_port(c, &format!("DI[{i}]"), muxcy, "DI");
            if let Some(s) = p.nl.cell(c).net(&format!("S[{i}]")) {
                p.nl.connect(muxcy, "S", s);
            }
            p.nl.move_port(c, &format!("S[{i}]"), xorcy, "LI");
            p.nl.move_port(c, &format!("O[{i}]"), xorcy, "O");
            let mut co = p.nl.cell(c).net(&format!("CO[{i}]"));
            p.nl.disconnect(c, &format!("CO[{i}]"));
            if co.is_none() {
                co = Some(p.nl.add_net_unique(&format!("{name}$intnet$$split$co{i}")));
            }
            let co = co.expect("set");
            p.nl.connect(muxcy, "O", co);
            cin = Some(co);
        }
        p.nl.remove_cell(c);
    }
}

/// A carry net's users other than the next MUXCY's and XORCY's CI.
fn illegal_fanout(p: &Packer, n: NetId) -> bool {
    let users = &p.nl.net(n).users;
    if users.len() > 2 {
        return true;
    }
    let (mut muxcy, mut xorcy) = (None, None);
    for (c, port) in users {
        match p.nl.cell(*c).ty.as_str() {
            "MUXCY" if muxcy.is_none() && port == "CI" => muxcy = Some(*c),
            "XORCY" if xorcy.is_none() && port == "CI" => xorcy = Some(*c),
            _ => return true,
        }
    }
    if let (Some(m), Some(x)) = (muxcy, xorcy) {
        return p.nl.cell(m).net("S") != p.nl.cell(x).net("LI");
    }
    false
}

struct Group {
    muxcys: Vec<CellId>,
    xorcys: Vec<Option<CellId>>,
}

pub fn pack_carries(p: &mut Packer) {
    split_carry4s(p);
    let roots: Vec<CellId> =
        p.nl.cells_of("MUXCY")
            .into_iter()
            .filter(|&m| match p.nl.cell(m).net("CI") {
                None => true,
                Some(ci) => p.nl.driver(ci).is_none_or(|(d, _)| p.nl.cell(d).ty != "MUXCY") || illegal_fanout(p, ci),
            })
            .collect();

    let mut groups = Vec::new();
    for root in roots {
        let mut g = Group {
            muxcys: Vec::new(),
            xorcys: Vec::new(),
        };
        let mut muxcy = root;
        let mut mux_ci;
        loop {
            g.muxcys.push(muxcy);
            mux_ci = p.nl.cell(muxcy).net("CI");
            g.xorcys.push(None);
            if let Some(s) = p.nl.cell(muxcy).net("S") {
                for (u, port) in p.nl.net(s).users.clone() {
                    if p.nl.cell(u).ty == "XORCY" && port == "LI" && p.nl.cell(u).net("CI") == mux_ci {
                        *g.xorcys.last_mut().expect("pushed") = Some(u);
                        break;
                    }
                }
            }
            mux_ci = p.nl.cell(muxcy).net("O");
            let Some(o) = mux_ci else { break };
            if illegal_fanout(p, o) {
                break;
            }
            match p.nl.net(o).users.iter().find(|(u, _)| p.nl.cell(*u).ty == "MUXCY") {
                Some((u, _)) => muxcy = *u,
                None => break,
            }
        }
        if let Some(o) = mux_ci {
            let users = p.nl.net(o).users.clone();
            if users.len() == 1 && p.nl.cell(users[0].0).ty == "XORCY" && users[0].1 == "CI" {
                // A trailing XORCY packs into the chain behind a dummy MUXCY.
                let xorcy = users[0].0;
                let name = format!("{}$legal_muxcy$", p.nl.cell(xorcy).name);
                let dummy = create(p, "MUXCY", &name);
                p.nl.connect(dummy, "CI", o);
                if let Some(li) = p.nl.cell(xorcy).net("LI") {
                    p.nl.connect(dummy, "S", li);
                }
                g.muxcys.push(dummy);
                g.xorcys.push(Some(xorcy));
            } else if !users.is_empty() {
                // Other users: feed the carry out through a XORCY whose LI is
                // a zero LUT.
                let base = p.nl.net(o).name.clone();
                let vcc = p.nl.vcc;
                let zero = p.create_lut(&format!("{base}$feed$zero"), &[vcc], None, prop32(1));
                let feed_xor = create(p, "XORCY", &format!("{base}$feed$xor"));
                let dummy = create(p, "MUXCY", &format!("{base}$feed$muxcy"));
                let last = p.nl.driver(o).map(|(c, _)| c).expect("muxcy drives it");
                p.nl.disconnect(last, "O");
                let zn = p.nl.add_net_unique(&format!("{base}$feed$zero$o"));
                p.nl.connect(zero, "O", zn);
                p.nl.connect(feed_xor, "LI", zn);
                p.nl.connect(dummy, "S", zn);
                let cn = p.nl.add_net_unique(&format!("{base}$feed$co"));
                p.nl.connect(last, "O", cn);
                p.nl.connect(feed_xor, "CI", cn);
                p.nl.connect(dummy, "CI", cn);
                p.nl.connect(feed_xor, "O", o);
                g.muxcys.push(dummy);
                g.xorcys.push(Some(feed_xor));
            }
        }
        groups.push(g);
    }

    let lut_types: HashSet<&str> = ["LUT1", "LUT2", "LUT3", "LUT4", "LUT5"].into_iter().collect();
    let mut packed: BTreeSet<CellId> = BTreeSet::new();
    let (mut feeds, mut chains) = (0usize, 0usize);
    for g in &groups {
        chains += 1;
        let mut c4s: Vec<CellId> = Vec::new();
        for i in 0..g.muxcys.len() {
            let z = (i % 4) as i32;
            let (muxcy, xorcy) = (g.muxcys[i], g.xorcys[i]);
            let dy = -((i / 4 + i / (4 * 25)) as i32);
            if z == 0 {
                let name = format!("{}$PACKED_CARRY4$", p.nl.cell(muxcy).name);
                let c4 = create(p, "CARRY4", &name);
                if i == 0 {
                    p.make_root(c4, Some(slot::CARRY4));
                } else {
                    p.add_member(
                        c4s[0],
                        c4,
                        Offset {
                            dx: 0,
                            dy,
                            z: slot::CARRY4,
                            abs: true,
                        },
                    );
                }
                c4s.push(c4);
            }
            let c4 = *c4s.last().expect("created");
            let root = c4s[0];
            if z == 0 && i == 0 {
                p.nl.move_port(muxcy, "CI", c4, "CYINIT");
            } else if z == 0 {
                p.nl.move_port(muxcy, "CI", c4, "CI");
            } else {
                p.nl.disconnect(muxcy, "CI");
            }
            if z == 3 {
                p.nl.move_port(muxcy, "O", c4, "CO[3]");
            } else {
                p.nl.disconnect(muxcy, "O");
            }
            p.nl.move_port(muxcy, "S", c4, &format!("S[{z}]"));
            p.nl.move_port(muxcy, "DI", c4, &format!("DI[{z}]"));
            packed.insert(muxcy);
            if let Some(x) = xorcy {
                p.nl.move_port(x, "O", c4, &format!("O[{z}]"));
                p.nl.disconnect(x, "LI");
                p.nl.disconnect(x, "DI");
                packed.insert(x);
            }
            // S and DI from LUTs that can sit at this position, else feeds.
            let s_net = p.nl.cell(c4).net(&format!("S[{z}]"));
            let di_net = p.nl.cell(c4).net(&format!("DI[{z}]"));
            let mut inputs: BTreeSet<NetId> = BTreeSet::new();
            let mut s_inputs = 0;
            let lut_driver = |p: &Packer, n: NetId| -> Option<CellId> {
                let (d, _) = p.nl.driver(n)?;
                (p.nl.net(n).users.len() == 1 && lut_types.contains(p.nl.cell(d).ty.as_str())).then_some(d)
            };
            let mut s_lut = s_net.and_then(|n| lut_driver(p, n));
            if let Some(l) = s_lut {
                for j in 0..5 {
                    if let Some(x) = p.nl.cell(l).net(&format!("I{j}")) {
                        inputs.insert(x);
                        s_inputs += 1;
                    }
                }
            }
            let mut di_lut = di_net.and_then(|n| lut_driver(p, n));
            if let Some(l) = di_lut {
                for j in 0..5 {
                    if let Some(x) = p.nl.cell(l).net(&format!("I{j}")) {
                        inputs.insert(x);
                    }
                }
            }
            let mut count = inputs.len();
            if s_lut.is_none() {
                count += 1;
            }
            if di_lut.is_none() {
                count += 1;
            }
            if count > 5 {
                di_lut = None;
                if s_inputs > 4 {
                    s_lut = None;
                }
            }
            if s_lut.is_none()
                && let Some(n) = s_net
            {
                s_lut = Some(p.feed_through_lut(n, &[(c4, format!("S[{z}]"))]));
                feeds += 1;
            }
            if di_lut.is_none()
                && let Some(n) = di_net
            {
                di_lut = Some(p.feed_through_lut(n, &[(c4, format!("DI[{z}]"))]));
                feeds += 1;
            }
            if let Some(l) = s_lut {
                p.add_member(
                    root,
                    l,
                    Offset {
                        dx: 0,
                        dy,
                        z: slot::at(z, slot::LUT6),
                        abs: true,
                    },
                );
            }
            if let Some(l) = di_lut {
                p.add_member(
                    root,
                    l,
                    Offset {
                        dx: 0,
                        dy,
                        z: slot::at(z, slot::LUT5),
                        abs: true,
                    },
                );
            }
        }
    }
    for c in packed {
        p.nl.remove_cell(c);
    }

    // MUXCYs and XORCYs left out of every chain become soft logic.
    let mut rules = BTreeMap::new();
    rules.insert(
        "MUXCY".to_string(),
        Rule::to("LUT3")
            .port("DI", "I0")
            .port("CI", "I1")
            .port("S", "I2")
            .param("INIT", prop32(0xCA)),
    );
    rules.insert(
        "XORCY".to_string(),
        Rule::to("LUT2").port("CI", "I0").port("LI", "I1").param("INIT", prop32(0x6)),
    );
    let blasted = p.generic_xform(&rules);
    // Bus brackets off the CARRY4 ports; CI on CIN.
    let rule = Rule::to("CARRY4").port("CI", "CIN");
    for c in p.nl.cells_of("CARRY4") {
        p.xform(c, &rule);
    }
    p.count("carry.chains", chains);
    p.count("carry.feed_luts", feeds);
    p.count("carry.blasted", blasted);
}
