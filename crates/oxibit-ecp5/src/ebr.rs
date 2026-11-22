//! DP16KD block RAMs in the flow: bel choice, pins and tie-offs, and the
//! configuration.
//!
//! A DP16KD's settings (`EBR<z>.*`) sit in the MIB_EBR tile of its bel and
//! the two east of it (EBR0 at `MIB_EBR0` uses `MIB_EBR0`/`MIB_EBR1`, EBR1
//! at `MIB_EBR2` uses `MIB_EBR2..4`, and so on, as the prjtrellis
//! `041-ebr_config` fuzzer lists them); each setting is written into every
//! one of those tiles that has it. The initial contents are addressed by
//! the block's write ID: `EBR<z>.WID` in the configuration and the
//! `.bram_init` index (the bitstream's `LSC_EBR_ADDRESS`) are the same
//! number.
//!
//! Port A writes and port B reads.
//!
//! Unused inputs are tied the way nextpnr's `write_bram` ties them, fuse
//! for fuse, since that is the pattern known to work on silicon. Address
//! and data pins sit on CIB data wires (A..D), whose constant muxes offer
//! both levels, and are tied to the level wanted. Every control pin (CLK,
//! CE, OCE, WE, RST, CS) is tied "high" through [`tie_high`]: `1` on a CE
//! wire, `0` on an LSR or CLK wire (the only option those offer, which
//! nextpnr writes for a high tie). A control pin that must be low (an
//! unused clock, write enable or reset) then has its own `<PIN>MUX INV`
//! set. The chip selects are all tied high and `CSDECODE` is 111, so the
//! block is always selected. In x18 the two lane enables go on
//! `ADA0`/`ADA1` (byte enables) when they differ; when they are one
//! signal it drives `WEA` and the byte enables are tied high.
//!
//! Port B is never configured x9. On silicon (LFE5U-85F) a port B set
//! to `DATA_WIDTH_B` 9 behaves as x18: reads drop the word address LSB
//! and writes need the x18 byte enables. The database's only "B not 18"
//! fuse (in the tile two east, e.g. `MIB_EBR4` F1B0 for EBR1) is set and
//! is not enough; port A at 9 works for reads and writes. So an x9
//! memory reads port B at x18 (`Ebr::read_width`) and synthesis picks
//! the 9-bit word in the fabric.
//!
//! Word settings are MSB first in the database (the fuzzer wrote
//! `0b<bit0><bit1>..`), so `WID` and `CSDECODE` are written bit reversed,
//! as nextpnr does; the `.bram_init` index is the plain WID.

use std::collections::{BTreeMap, HashMap};

use crate::bels::{BelId, Bels};
use crate::chip::Level;
use crate::config::{ChipConfig, TileConfig};
use crate::db::DeviceDb;
use crate::graph::Graph;
use crate::io::{tie, tie_high};
use crate::netlist::{Cell, Ebr, EbrWidth, NetId, Netlist, Polarity};
use crate::{Result, err};

/// How a DP16KD pin is used.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Use {
    /// Driven by the net.
    Sink(NetId),
    /// A clock input on the net.
    Clock(NetId),
    /// Drives the net.
    Source(NetId),
    /// An address or data pin tied to a constant.
    Tie(Level),
    /// A control pin tied to a constant: high through the CIB, low by
    /// also setting the pin's own inverter.
    Control(Level),
}

/// Every pin of `e` the design uses or ties, by pin name.
pub fn pin_uses(e: &Ebr) -> Vec<(String, Use)> {
    let mut v: Vec<(String, Use)> = Vec::new();
    let addr = |v: &mut Vec<(String, Use)>,
                port: char,
                lsb: usize,
                nets: &[NetId],
                low: &[(usize, Level)]| {
        let mut driven = [false; 14];
        for &(k, level) in low {
            v.push((format!("AD{port}{k}"), Use::Tie(level)));
            driven[k] = true;
        }
        for (k, &n) in nets.iter().enumerate() {
            v.push((format!("AD{port}{}", k + lsb), Use::Sink(n)));
            driven[k + lsb] = true;
        }
        for (k, d) in driven.iter().enumerate() {
            if !d {
                v.push((format!("AD{port}{k}"), Use::Tie(Level::Low)));
            }
        }
    };
    let cs = |v: &mut Vec<(String, Use)>, port: char| {
        for k in 0..3 {
            v.push((format!("CS{port}{k}"), Use::Control(Level::High)));
        }
    };
    // Port B: read.
    let r = &e.read;
    v.push(("CLKB".to_owned(), Use::Clock(r.clk)));
    addr(&mut v, 'B', e.read_width.addr_lsb(), &r.addr, &[]);
    for (j, d) in r.data.iter().enumerate() {
        if let Some(d) = d {
            v.push((format!("DOB{j}"), Use::Source(*d)));
        }
    }
    v.push(("CEB".to_owned(), Use::Control(Level::High)));
    v.push(("OCEB".to_owned(), Use::Control(Level::High)));
    v.push(("RSTB".to_owned(), Use::Control(Level::Low)));
    v.push(("WEB".to_owned(), Use::Control(Level::Low)));
    cs(&mut v, 'B');
    // Port A: write.
    match &e.write {
        Some(w) => {
            v.push(("CLKA".to_owned(), Use::Clock(w.clk)));
            let same = w.we[1].is_none() || w.we[0] == w.we[1];
            let we0 = w.we[0];
            match (e.width, same) {
                (EbrWidth::X9, _) | (EbrWidth::X18, true) => {
                    v.push((
                        "WEA".to_owned(),
                        we0.map_or(Use::Control(Level::Low), Use::Sink),
                    ));
                    let be: &[(usize, Level)] = if e.width == EbrWidth::X18 {
                        &[(0, Level::High), (1, Level::High)]
                    } else {
                        &[]
                    };
                    addr(&mut v, 'A', e.width.addr_lsb(), &w.addr, be);
                }
                (EbrWidth::X18, false) => {
                    v.push(("WEA".to_owned(), Use::Control(Level::High)));
                    let mut tmp = Vec::new();
                    addr(&mut tmp, 'A', e.width.addr_lsb(), &w.addr, &[]);
                    for (pin, u) in tmp {
                        let be = match pin.as_str() {
                            "ADA0" => w.we[0],
                            "ADA1" => w.we[1],
                            _ => {
                                v.push((pin, u));
                                continue;
                            }
                        };
                        v.push((pin, be.map_or(Use::Tie(Level::Low), Use::Sink)));
                    }
                }
            }
            for (j, d) in w.data.iter().enumerate() {
                if let Some(d) = d {
                    v.push((format!("DIA{j}"), Use::Sink(*d)));
                }
            }
            v.push(("CEA".to_owned(), Use::Control(Level::High)));
            v.push(("OCEA".to_owned(), Use::Control(Level::High)));
        }
        None => {
            v.push(("CLKA".to_owned(), Use::Control(Level::Low)));
            v.push(("WEA".to_owned(), Use::Control(Level::Low)));
            v.push(("CEA".to_owned(), Use::Control(Level::High)));
            v.push(("OCEA".to_owned(), Use::Control(Level::High)));
        }
    }
    v.push(("RSTA".to_owned(), Use::Control(Level::Low)));
    cs(&mut v, 'A');
    v
}

/// A DP16KD bel for every block RAM cell, nearest `near` (row, col) first.
pub fn assign(bels: &Bels, nl: &Netlist, near: (f64, f64)) -> Result<HashMap<usize, BelId>> {
    let mut free: Vec<BelId> = (0..bels.bels.len())
        .map(|i| BelId::try_from(i).expect("fits"))
        .filter(|&b| bels.template(b).ty == "DP16KD")
        .collect();
    let d = |b: BelId| {
        let bel = bels.bels[b as usize];
        let dy = f64::from(bel.y) - near.0;
        let dx = f64::from(bel.x) - near.1;
        dx * dx + dy * dy
    };
    free.sort_by(|&a, &b| d(a).total_cmp(&d(b)).then(a.cmp(&b)));
    let mut out = HashMap::new();
    let mut it = free.into_iter();
    for (i, c) in nl.cells.iter().enumerate() {
        if matches!(c, Cell::Ebr(_)) {
            let b = it
                .next()
                .ok_or_else(|| err!("more block RAMs than the device has"))?;
            out.insert(i, b);
        }
    }
    Ok(out)
}

/// The tiles holding `EBR<z>.*` for a bel at (x, y).
fn holds(bx: i16, by: i16, x: i16, y: i16) -> bool {
    y == by && (bx..=bx + 2).contains(&x)
}

fn set_enum(
    g: &Graph,
    db: &DeviceDb,
    bel: (i16, i16, &str),
    name: &str,
    value: &str,
    tiles: &mut BTreeMap<String, TileConfig>,
) -> Result<()> {
    let full = format!("{}.{name}", bel.2);
    let mut n = 0;
    for t in g.tiles.iter().filter(|t| holds(bel.0, bel.1, t.x, t.y)) {
        let Some(e) = db.tile_bits(&t.ty)?.enums.get(&full) else {
            continue;
        };
        if !e.options.contains_key(value) {
            return Err(err!("{}: {full} has no option {value}", t.name));
        }
        tiles
            .entry(t.name.clone())
            .or_default()
            .enums
            .push((full.clone(), value.to_owned()));
        n += 1;
    }
    if n == 0 {
        return Err(err!("no tile at R{}C{} has {full}", bel.1, bel.0));
    }
    Ok(())
}

fn set_word(
    g: &Graph,
    db: &DeviceDb,
    bel: (i16, i16, &str),
    name: &str,
    value: &[bool],
    tiles: &mut BTreeMap<String, TileConfig>,
) -> Result<()> {
    let full = format!("{}.{name}", bel.2);
    let mut n = 0;
    for t in g.tiles.iter().filter(|t| holds(bel.0, bel.1, t.x, t.y)) {
        let Some(w) = db.tile_bits(&t.ty)?.words.get(&full) else {
            continue;
        };
        if w.bits.len() != value.len() {
            return Err(err!(
                "{}: {full} is {} bits, not {}",
                t.name,
                w.bits.len(),
                value.len()
            ));
        }
        tiles
            .entry(t.name.clone())
            .or_default()
            .words
            .push((full.clone(), value.to_vec()));
        n += 1;
    }
    if n == 0 {
        return Err(err!("no tile at R{}C{} has {full}", bel.1, bel.0));
    }
    Ok(())
}

/// Write every block RAM's settings, ties and initial contents.
pub fn configure(
    g: &Graph,
    db: &DeviceDb,
    bels: &Bels,
    nl: &Netlist,
    at: &HashMap<usize, BelId>,
    cc: &mut ChipConfig,
) -> Result<()> {
    let mut cells: Vec<(usize, BelId)> = at.iter().map(|(&c, &b)| (c, b)).collect();
    cells.sort_unstable();
    for (wid, (c, b)) in cells.into_iter().enumerate() {
        let Cell::Ebr(e) = &nl.cells[c] else {
            return Err(err!("cell {c} is not a block RAM"));
        };
        let bel = bels.bels[b as usize];
        let name = bels.template(b).name.clone();
        let loc = (bel.x, bel.y, name.as_str());
        let wname = |w: EbrWidth| match w {
            EbrWidth::X9 => "9",
            EbrWidth::X18 => "18",
        };
        let tiles = &mut cc.tiles;
        set_enum(g, db, loc, "MODE", "DP16KD", tiles)?;
        set_enum(g, db, loc, "DP16KD.DATA_WIDTH_A", wname(e.width), tiles)?;
        if e.read_width == EbrWidth::X9 {
            return Err(err!("{name}: port B cannot be x9 (it behaves as x18)"));
        }
        set_enum(g, db, loc, "DP16KD.DATA_WIDTH_B", wname(e.read_width), tiles)?;
        set_enum(g, db, loc, "DP16KD.WRITEMODE_A", "NORMAL", tiles)?;
        set_enum(g, db, loc, "DP16KD.WRITEMODE_B", "NORMAL", tiles)?;
        set_enum(g, db, loc, "REGMODE_A", "NOREG", tiles)?;
        set_enum(g, db, loc, "REGMODE_B", "NOREG", tiles)?;
        set_enum(g, db, loc, "RESETMODE", "SYNC", tiles)?;
        let wid_u16 = u16::try_from(wid).map_err(|_| err!("too many block RAMs"))?;
        let wid_bits: Vec<bool> = (0..9).map(|k| (wid >> (8 - k)) & 1 == 1).collect();
        set_word(g, db, loc, "WID", &wid_bits, tiles)?;
        if e.read.clk_polarity == Polarity::Neg {
            set_enum(g, db, loc, "CLKBMUX", "INV", tiles)?;
        }
        if e.write.as_ref().is_some_and(|w| w.clk_polarity == Polarity::Neg) {
            set_enum(g, db, loc, "CLKAMUX", "INV", tiles)?;
        }
        for (pin, u) in pin_uses(e) {
            let wire = || {
                bels.pin_wire(b, &pin)
                    .ok_or_else(|| err!("{name} has no pin {pin}"))
            };
            let at = |er| err!("{name} R{}C{} {pin}: {er}", bel.y, bel.x);
            match u {
                Use::Tie(level) => {
                    tie(g, db, wire()?, level, tiles).map_err(at)?;
                }
                Use::Control(level) => {
                    tie_high(g, db, wire()?, tiles).map_err(at)?;
                    if level == Level::Low {
                        if pin.starts_with("CS") {
                            return Err(at(err!("a chip select cannot be tied low")));
                        }
                        set_enum(g, db, loc, &format!("{pin}MUX"), "INV", tiles)?;
                    }
                }
                _ => {}
            }
        }
        set_word(g, db, loc, "CSDECODE_A", &[true; 3], tiles)?;
        set_word(g, db, loc, "CSDECODE_B", &[true; 3], tiles)?;
        // Always written, zeros included: a block left out keeps whatever
        // it powered up with.
        cc.bram_data.insert(wid_u16, e.init.clone());
    }
    Ok(())
}
