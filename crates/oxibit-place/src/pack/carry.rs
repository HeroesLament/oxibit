//! Carry chains (nextpnr-xilinx `split_carry4s`, `pack_carries`).
//!
//! Every CARRY4 is split into four MUXCY/XORCY pairs, the pairs are chained
//! by their carry nets, and each chain is cut into CARRY4s again: one per
//! four positions, stacked down a column (`dy = -(i/4 + i/100)`, skipping
//! the tiles with no CARRY4). Each position's S must come from a LUT that
//! can sit at that position, on its 6LUT; otherwise a feed-through LUT1
//! carries the signal there. DI comes from a LUT on the 5LUT when one fits
//! beside S (five inputs between them), else on the letter's bypass pin
//! (AX..DX): the CY0 mux takes O5 or that pin, so DI needs no LUT of its
//! own (nextpnr-xilinx feeds it through a LUT1 instead).

use std::collections::{BTreeMap, BTreeSet, HashSet};

use super::constants::prop32;
use super::{Offset, Packer, Rule, slot};
use crate::netlist::{CellId, Dir, NetId};

/// A chain member's row offset for CARRY4 row `k` (0 at the root): the
/// tile-grid offset nextpnr-xilinx writes, one HCLK row skipped per 25
/// CARRY4s, which `place::import` reads against another tool's placement.
fn chain_dy(k: usize) -> i32 {
    -((k + k / 25) as i32)
}

/// The inverse of `chain_dy`: the CARRY4 row (0 at the root, counting up
/// its slice column) of a chain member at row offset `dy`. Placement sites
/// a chain by rows, not by this offset (`legalise_spread`).
pub fn chain_row(dy: i32) -> usize {
    let d = (-dy).max(0) as usize;
    d - d / 26
}

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

    // A chain whose carry-in is a signal takes it on CYINIT, which enters
    // the bottom slice on AX. Position 0's DI enters on AX too (the CY0
    // mux), constant or not, and the pin holds one net: the carry-in would
    // be left with CIN, which no general routing reaches. So position 0's
    // DI must come from the 5LUT instead:
    // - a constant DI beside an S LUT of at most four inputs (five with the
    //   constant LUT's own): a constant LUT of its own, which the loop
    //   below seats on the 5LUT (`carry.cyinit_const_di`);
    // - otherwise the chain gets a first position that only passes the
    //   carry (S = 1, no DI), the real positions one up
    //   (`carry.cyinit_shift`). Most chains are a multiple of four long, so
    //   that costs a slice; it is the fallback.
    let (mut cyinit_shift, mut cyinit_const_di) = (0usize, 0usize);
    for g in groups.iter_mut() {
        let root = g.muxcys[0];
        let ci = p.nl.cell(root).net("CI");
        let di = p.nl.cell(root).net("DI");
        let live = ci.is_some_and(|n| n != p.nl.gnd && n != p.nl.vcc);
        let (Some(ci), true, true) = (ci, live, di.is_some() && di != ci) else { continue };
        let base = p.nl.cell(root).name.clone();
        let vcc = p.nl.vcc;
        let gnd = p.nl.gnd;
        // Inputs of the LUT that alone drives S, the constant LUT's VCC
        // input counted with them; no such LUT means a one-input feed.
        let s_inputs = p.nl.cell(root).net("S").and_then(|s| {
            let (d, _) = p.nl.driver(s)?;
            // S also feeds the XORCY's LI: the MUXCY and it are one position.
            let sole = p.nl.net(s).users.iter().all(|(u, _)| *u == root || Some(*u) == g.xorcys[0]);
            (sole && p.nl.cell(d).ty.starts_with("LUT")).then(|| {
                let mut ins: BTreeSet<NetId> = (0..6).filter_map(|j| p.nl.cell(d).net(&format!("I{j}"))).collect();
                ins.insert(vcc);
                ins.len()
            })
        });
        if let (Some(d), true) = (di.filter(|&d| d == gnd || d == vcc), s_inputs.is_none_or(|n| n <= 5)) {
            // INIT over the one input, tied high: bit 1 is the output.
            let k = p.create_lut(&format!("{base}$cyinit$di"), &[vcc], None, prop32(if d == vcc { 2 } else { 1 }));
            let kn = p.nl.add_net_unique(&format!("{base}$cyinit$di$o"));
            p.nl.connect(k, "O", kn);
            p.nl.disconnect(root, "DI");
            p.nl.connect(root, "DI", kn);
            cyinit_const_di += 1;
            continue;
        }
        let one = p.create_lut(&format!("{base}$cyinit$one"), &[vcc], None, prop32(2));
        let on = p.nl.add_net_unique(&format!("{base}$cyinit$one$o"));
        p.nl.connect(one, "O", on);
        let pass = create(p, "MUXCY", &format!("{base}$cyinit$muxcy"));
        p.nl.connect(pass, "S", on);
        let cn = p.nl.add_net_unique(&format!("{base}$cyinit$co"));
        p.nl.disconnect(root, "CI");
        p.nl.connect(root, "CI", cn);
        if let Some(x) = g.xorcys[0] {
            p.nl.disconnect(x, "CI");
            p.nl.connect(x, "CI", cn);
        }
        p.nl.connect(pass, "CI", ci);
        p.nl.connect(pass, "O", cn);
        g.muxcys.insert(0, pass);
        g.xorcys.insert(0, None);
        cyinit_shift += 1;
    }

    let lut5_types: HashSet<&str> = ["LUT1", "LUT2", "LUT3", "LUT4", "LUT5"].into_iter().collect();
    let mut packed: BTreeSet<CellId> = BTreeSet::new();
    let (mut feeds, mut chains, mut di_bypass) = (0usize, 0usize, 0usize);
    for g in &groups {
        chains += 1;
        let mut c4s: Vec<CellId> = Vec::new();
        for i in 0..g.muxcys.len() {
            let z = (i % 4) as i32;
            let (muxcy, xorcy) = (g.muxcys[i], g.xorcys[i]);
            let dy = chain_dy(i / 4);
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
            // S from a LUT that can sit on this position's 6LUT, else a
            // feed. DI from a LUT on its 5LUT if one fits beside S, else on
            // the bypass pin.
            let s_net = p.nl.cell(c4).net(&format!("S[{z}]"));
            let di_net = p.nl.cell(c4).net(&format!("DI[{z}]"));
            let sole_lut = |p: &Packer, n: NetId| -> Option<CellId> {
                let (d, _) = p.nl.driver(n)?;
                (p.nl.net(n).users.len() == 1 && p.nl.cell(d).ty.starts_with("LUT")).then_some(d)
            };
            let lut_inputs = |p: &Packer, l: CellId| -> BTreeSet<NetId> { (0..6).filter_map(|j| p.nl.cell(l).net(&format!("I{j}"))).collect() };
            let mut s_lut = s_net.and_then(|n| sole_lut(p, n));
            let s_ins = s_lut.map(|l| lut_inputs(p, l)).unwrap_or_default();
            let di_lut = di_net.and_then(|n| sole_lut(p, n)).filter(|&l| {
                // A fractured letter: both LUTs on A1..A5 (A6 high), the S
                // LUT's input (a feed's one) counted when S needs a feed.
                let s_count = if s_lut.is_some() { s_ins.len() } else { 1 };
                let mut all = s_ins.clone();
                all.extend(lut_inputs(p, l));
                let all = all.len() + usize::from(s_lut.is_none());
                lut5_types.contains(p.nl.cell(l).ty.as_str()) && s_count <= 5 && all <= 5
            });
            if s_lut.is_none()
                && let Some(n) = s_net
            {
                s_lut = Some(p.feed_through_lut(n, &[(c4, format!("S[{z}]"))]));
                feeds += 1;
            }
            if di_lut.is_none() && di_net.is_some() {
                di_bypass += 1;
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
    p.count("carry.di_bypass", di_bypass);
    p.count("carry.cyinit_shift", cyinit_shift);
    p.count("carry.cyinit_const_di", cyinit_const_di);
    p.count("carry.blasted", blasted);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chain_row_inverts_the_offset() {
        for k in 0..2000 {
            assert_eq!(chain_row(chain_dy(k)), k, "row {k}, offset {}", chain_dy(k));
        }
        // The old formula, written per position: the same offsets.
        for i in 0..8000usize {
            assert_eq!(chain_dy(i / 4), -((i / 4 + i / (4 * 25)) as i32));
        }
    }
}
