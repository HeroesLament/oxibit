//! A slice's single-owner pins.
//!
//! Each letter (A..D) has one bypass input, AX..DX. Several bels can read
//! it (CARRY4 CYINIT on A, CARRY4 DI through the CY0 mux, the letter's flop
//! or 5FF D, a LUTRAM's DI1), but the pin carries one net. `BypassPins`
//! holds one claim per letter and refuses a second net on a taken pin, so
//! a packer or fill that asks first cannot double-book it, and `check`
//! finds any slice in a placed design that did.

use std::collections::{BTreeMap, HashMap};

use oxibit_pnr::packed::Loc;

use crate::netlist::{CellId, NetId};
use crate::pack::Packer;

/// What reads a letter's bypass pin.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum XUse {
    /// CARRY4 CYINIT (letter A).
    CyInit,
    /// CARRY4 DI through the CY0 mux.
    CarryDi,
    /// The letter's flop, D from outside the letter.
    FfD,
    /// The letter's 5FF, D from outside the letter.
    Ff5D,
    /// A LUTRAM's DI1.
    RamDi,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct XClaim {
    pub by: XUse,
    pub net: NetId,
}

/// A second net asked of a taken pin.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Clash {
    /// 0..3 for A..D.
    pub letter: u8,
    pub held: XClaim,
    pub wanted: XClaim,
}

/// The four bypass pins of one slice.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct BypassPins([Option<XClaim>; 4]);

impl BypassPins {
    /// Take letter `letter`'s pin for `net`. A pin already carrying `net`
    /// is shared; one carrying another net is a clash and stays as it was.
    pub fn claim(&mut self, letter: u8, by: XUse, net: NetId) -> Result<(), Clash> {
        let wanted = XClaim { by, net };
        let slot = &mut self.0[usize::from(letter & 3)];
        match *slot {
            Some(held) if held.net != net => Err(Clash {
                letter,
                held,
                wanted,
            }),
            Some(_) => Ok(()),
            None => {
                *slot = Some(wanted);
                Ok(())
            }
        }
    }

    /// Whether `claim` would succeed, without taking the pin.
    pub fn free_for(&self, letter: u8, net: NetId) -> bool {
        self.0[usize::from(letter & 3)].is_none_or(|held| held.net == net)
    }

    pub fn get(&self, letter: u8) -> Option<XClaim> {
        self.0[usize::from(letter & 3)]
    }

    /// Pins in use.
    pub fn used(&self) -> u32 {
        self.0.iter().filter(|s| s.is_some()).count() as u32
    }
}

/// The bypass pins a slice's cells need, given their bels (`A6LUT`, `AFF`,
/// `A5FF`, `CARRY4`, ...): an input needs the pin when its net is live and
/// not driven inside the slice (by a LUT of the same letter, or by a carry
/// or wide mux of any letter). Claims are made carry first, then flops,
/// then LUTRAM, so a clash names the later user.
pub fn bypass_claims(p: &Packer, cells: &[(CellId, String)]) -> (BypassPins, Vec<Clash>) {
    let members: HashMap<CellId, u8> = cells.iter().map(|(c, b)| (*c, b.as_bytes()[0])).collect();
    let inside = |n: NetId, l: u8| match p.nl.driver(n) {
        Some((d, _)) => members
            .get(&d)
            .is_some_and(|&dl| p.nl.cell(d).ty != "SLICE_LUTX" || dl == l),
        None => false,
    };
    let live = |c: CellId, port: &str| p.nl.cell(c).net(port).filter(|&n| !p.nl.is_const(n));
    let mut wants: Vec<(u8, u8, XUse, NetId)> = Vec::new(); // (order, letter, use, net)
    for (c, b) in cells {
        let l = b.as_bytes()[0];
        match p.nl.cell(*c).ty.as_str() {
            "CARRY4" => {
                if let Some(n) = live(*c, "CYINIT") {
                    wants.push((0, 0, XUse::CyInit, n));
                }
                for i in 0..4u8 {
                    if let Some(n) = live(*c, &format!("DI{i}")).filter(|&n| !inside(n, b'A' + i)) {
                        wants.push((0, i, XUse::CarryDi, n));
                    }
                }
            }
            "SLICE_FFX" => {
                if let Some(n) = live(*c, "D").filter(|&n| !inside(n, l)) {
                    let by = if b.ends_with("5FF") {
                        XUse::Ff5D
                    } else {
                        XUse::FfD
                    };
                    wants.push((1, l.wrapping_sub(b'A'), by, n));
                }
            }
            "SLICE_LUTX" => {
                if let Some(n) = live(*c, "DI1") {
                    wants.push((2, l.wrapping_sub(b'A'), XUse::RamDi, n));
                }
            }
            _ => {}
        }
    }
    wants.sort_by_key(|w| (w.0, w.1));
    let mut x = BypassPins::default();
    let mut clashes = Vec::new();
    for (_, l, by, n) in wants {
        if let Err(e) = x.claim(l, by, n) {
            clashes.push(e);
        }
    }
    (x, clashes)
}

/// What a letter sends out of the slice through its output mux
/// (AMUX..DMUX, OUTMUX): one of them per letter.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MuxUse {
    /// The 5LUT's O5, read by anything but the letter's own flops or the
    /// carry (those take it inside the slice).
    Lut5,
    /// The 5FF's Q: it has no other way out.
    Ff5,
    /// A carry output (O or CO) read outside the slice or by another
    /// letter's flop.
    Carry,
}

/// Letters of one slice whose output mux more than one cell needs.
pub fn outmux_clashes(p: &Packer, cells: &[(CellId, String)]) -> Vec<(u8, Vec<MuxUse>)> {
    let letter_of: HashMap<CellId, u8> = cells.iter().map(|(c, b)| (*c, b.as_bytes()[0])).collect();
    // Does net `n` leave letter `l` other than into its own flops or the carry?
    let leaves = |n: NetId, l: u8| {
        p.nl.net(n)
            .users
            .iter()
            .any(|(u, _)| match letter_of.get(u) {
                Some(&ul) => {
                    let ty = p.nl.cell(*u).ty.as_str();
                    !(ty == "CARRY4" || (ty == "SLICE_FFX" && ul == l))
                }
                None => true,
            })
    };
    let output = |c: CellId| {
        p.nl.cell(c)
            .ports
            .values()
            .filter_map(|port| port.net)
            .find(|&n| !p.nl.is_const(n) && p.nl.driver(n).is_some_and(|(d, _)| d == c))
    };
    let mut uses: BTreeMap<u8, Vec<MuxUse>> = BTreeMap::new();
    for (c, b) in cells {
        let l = b.as_bytes()[0];
        match (p.nl.cell(*c).ty.as_str(), &b[1..]) {
            ("SLICE_LUTX", "5LUT") => {
                if output(*c).is_some_and(|n| leaves(n, l)) {
                    uses.entry(l).or_default().push(MuxUse::Lut5);
                }
            }
            ("SLICE_FFX", "5FF") => uses.entry(l).or_default().push(MuxUse::Ff5),
            ("CARRY4", _) => {
                for k in 0..4u8 {
                    let ll = b'A' + k;
                    let names = [
                        format!("O{k}"),
                        format!("O[{k}]"),
                        format!("CO{k}"),
                        format!("CO[{k}]"),
                    ];
                    let busy = names.iter().any(|port| {
                        p.nl.cell(*c)
                            .net(port)
                            .is_some_and(|n| !p.nl.is_const(n) && leaves(n, ll))
                    });
                    if busy {
                        uses.entry(ll).or_default().push(MuxUse::Carry);
                    }
                }
            }
            _ => {}
        }
    }
    uses.into_iter().filter(|(_, u)| u.len() > 1).collect()
}

/// Every slice letter in a placed design whose output mux two cells need,
/// as `X{x}Y{y}/{site} {letter}: uses`.
pub fn outmux_check(p: &Packer) -> Vec<String> {
    let mut by_site: BTreeMap<(i32, i32, String), Vec<(CellId, String)>> = BTreeMap::new();
    for c in p.nl.cell_ids() {
        if let Some(Loc::Grid { x, y, site, bel }) = &p.nl.cell(c).bel
            && site.starts_with("SLICE")
        {
            by_site
                .entry((*x, *y, site.clone()))
                .or_default()
                .push((c, bel.clone()));
        }
    }
    by_site
        .into_iter()
        .flat_map(|((x, y, site), cells)| {
            outmux_clashes(p, &cells)
                .into_iter()
                .map(move |(l, u)| format!("X{x}Y{y}/{site} {}MUX: {u:?}", l as char))
                .collect::<Vec<_>>()
        })
        .collect()
}

/// Every slice in a placed design whose cells ask two nets of one bypass
/// pin, by site (`X{x}Y{y}/{site}`).
pub fn check(p: &Packer) -> Vec<(String, Clash)> {
    let mut by_site: BTreeMap<(i32, i32, String), Vec<(CellId, String)>> = BTreeMap::new();
    for c in p.nl.cell_ids() {
        if let Some(Loc::Grid { x, y, site, bel }) = &p.nl.cell(c).bel
            && site.starts_with("SLICE")
        {
            by_site
                .entry((*x, *y, site.clone()))
                .or_default()
                .push((c, bel.clone()));
        }
    }
    by_site
        .into_iter()
        .flat_map(|((x, y, site), cells)| {
            let at = format!("X{x}Y{y}/{site}");
            bypass_claims(p, &cells)
                .1
                .into_iter()
                .map(move |cl| (at.clone(), cl))
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn one_net_per_pin() {
        let mut x = BypassPins::default();
        assert!(x.claim(0, XUse::CyInit, NetId(7)).is_ok());
        // the same net on the same pin is shared (CYINIT and DI0 from one net)
        assert!(x.claim(0, XUse::CarryDi, NetId(7)).is_ok());
        assert!(!x.free_for(0, NetId(8)));
        let e = x.claim(0, XUse::CarryDi, NetId(8)).expect_err("taken");
        assert_eq!(e.held.by, XUse::CyInit);
        assert_eq!(x.get(0).map(|c| c.net), Some(NetId(7)));
        assert!(x.free_for(1, NetId(8)));
        assert!(x.claim(1, XUse::FfD, NetId(8)).is_ok());
        assert_eq!(x.used(), 2);
    }
}
