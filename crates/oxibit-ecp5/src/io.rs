//! PIO settings and constant tie-offs, as a design's configuration needs
//! them.
//!
//! A PIO's settings are spread over the tiles beside its pad: on the left
//! and right edges the three PIC tiles (`PICL0`/`PICL1`/`PICL2`, rows
//! `row..=row + 2`) each carry part of `PIO<l>.BASE_TYPE` and the other
//! enums, with their own bits. An enum is written into every tile of the
//! pad that has it; writing only the first leaves the pad half configured
//! (on `PIOC` at R35C0 the output enable bits of `OUTPUT_LVCMOS33` are in
//! `PICL2`, the rest in `PICL1`).
//!
//! A fabric input left unrouted is not low: the PIO's tristate input
//! (`PADDT<l>_PIO`) reaches the routing through fixed connections to a
//! CIB input (`JB0` of the CIB two rows down), and an output pad with T
//! unrouted stays tristated. CIB inputs have a constant mux
//! (`CIB.<wire>MUX`, options `0` and `1`); `tie` follows a wire uphill over
//! fixed connections to the first such mux and sets it.

use std::collections::{BTreeMap, HashSet, VecDeque};

use crate::chip::Level;
use crate::config::TileConfig;
use crate::db::DeviceDb;
use crate::graph::{Graph, PipKind, Wire};
use crate::{Result, err};

/// A PIO: its pad's tile location and letter.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PioSite {
    pub row: i16,
    pub col: i16,
    pub pio: char,
}

/// The tiles whose settings belong to a pad, as nextpnr's `get_pio_tile`
/// and `get_pic_tile` place them: on the left and right edges `PICL1` /
/// `PICR1` one row down, and the PIC tile of the pair (`PICL0` at the
/// pad's row for A / B, `PICL2` two rows down for C / D); on the top
/// `PIOT0` (row 0) and `PICT0` (row 1) for A, `PIOT1` / `PICT1` one column
/// east for B; on the bottom `PICB0` for A, `PICB1` one column east for B.
fn near(g: &Graph, site: PioSite, x: i16, y: i16) -> bool {
    let (row, col) = (site.row, site.col);
    let b = i16::from(site.pio == 'B');
    if row == 0 {
        x == col + b && (y == 0 || y == 1)
    } else if row == g.height - 1 {
        x == col + b && y == row
    } else {
        let pic = if matches!(site.pio, 'A' | 'B') { row } else { row + 2 };
        x == col && (y == row + 1 || y == pic)
    }
}

/// Set `PIO<l>.<name>` to `value` in every tile near the pad that has it.
/// Errors if no tile has the enum, or a tile has it without that option.
/// Returns how many tiles were set.
pub fn pio_enum(
    g: &Graph,
    db: &DeviceDb,
    site: PioSite,
    name: &str,
    value: &str,
    tiles: &mut BTreeMap<String, TileConfig>,
) -> Result<usize> {
    site_enum(g, db, site, &format!("PIO{}.{name}", site.pio), value, tiles)
}

/// Set the enum `full` (`PIO<l>.*`, `IOLOGIC<l>.*`) to `value` in every
/// tile near the pad that has it.
pub fn site_enum(
    g: &Graph,
    db: &DeviceDb,
    site: PioSite,
    full: &str,
    value: &str,
    tiles: &mut BTreeMap<String, TileConfig>,
) -> Result<usize> {
    let full = full.to_owned();
    let mut n = 0;
    for t in g
        .tiles
        .iter()
        .filter(|t| near(g, site, t.x, t.y))
    {
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
        return Err(err!("no tile near R{}C{} has {full}", site.row, site.col));
    }
    Ok(n)
}

/// Tie `w` high the way nextpnr's `tie_cib_signal(.., true)` does: walk
/// uphill over fixed connections to the first CIB constant mux and select
/// `0` on an LSR or CLK wire (the only constant those offer, which nextpnr
/// writes for a high tie) or `1` on any other.
pub fn tie_high(
    g: &Graph,
    db: &DeviceDb,
    w: Wire,
    tiles: &mut BTreeMap<String, TileConfig>,
) -> Result<(String, String)> {
    let mut seen = HashSet::from([w]);
    let mut queue = VecDeque::from([w]);
    while let Some(w) = queue.pop_front() {
        let base = &g.names[w.name as usize];
        let value = if base.starts_with("JLSR") || base.starts_with("JCLK") {
            "0"
        } else {
            "1"
        };
        let mux = format!("CIB.{base}MUX");
        for t in g.tiles.iter().filter(|t| t.x == w.x && t.y == w.y) {
            let has = db
                .tile_bits(&t.ty)?
                .enums
                .get(&mux)
                .is_some_and(|e| e.options.contains_key(value));
            if has {
                tiles
                    .entry(t.name.clone())
                    .or_default()
                    .enums
                    .push((mux.clone(), value.to_owned()));
                return Ok((t.name.clone(), mux));
            }
        }
        for a in g.uphill(w) {
            if free_arc(g, db, &a)? && seen.insert(a.from) {
                queue.push_back(a.from);
            }
        }
    }
    Err(err!(
        "{}: no constant mux uphill over fixed or default connections",
        g.wire_name(w)
    ))
}

/// Tie the input `w` to a constant: follow it uphill over fixed
/// connections to a CIB input with a constant mux and set that mux.
/// Returns the tile and enum set.
pub fn tie(
    g: &Graph,
    db: &DeviceDb,
    w: Wire,
    level: Level,
    tiles: &mut BTreeMap<String, TileConfig>,
) -> Result<(String, String)> {
    let mut seen = HashSet::from([w]);
    let mut queue = VecDeque::from([w]);
    while let Some(w) = queue.pop_front() {
        let base = &g.names[w.name as usize];
        // The CIB CLK and LSR muxes have one constant, "0", and it is
        // high (nextpnr `tie_cib_signal`); the CE muxes have "1". None of
        // the three gives a low: that takes a routed constant.
        let clk_lsr = base.starts_with("JLSR") || base.starts_with("JCLK");
        let value = match level {
            Level::High if clk_lsr => "0",
            Level::Low if clk_lsr || base.starts_with("JCE") => {
                return Err(err!("{}: a CIB {base} input cannot be tied low", g.wire_name(w)));
            }
            Level::Low => "0",
            Level::High => "1",
        };
        let mux = format!("CIB.{base}MUX");
        for t in g.tiles.iter().filter(|t| t.x == w.x && t.y == w.y) {
            let has = db
                .tile_bits(&t.ty)?
                .enums
                .get(&mux)
                .is_some_and(|e| e.options.contains_key(value));
            if has {
                tiles
                    .entry(t.name.clone())
                    .or_default()
                    .enums
                    .push((mux.clone(), value.to_owned()));
                return Ok((t.name.clone(), mux));
            }
        }
        for a in g.uphill(w) {
            if free_arc(g, db, &a)? && seen.insert(a.from) {
                queue.push_back(a.from);
            }
        }
    }
    Err(err!(
        "{}: no constant mux uphill over fixed or default connections",
        g.wire_name(w)
    ))
}

/// Set the word `full` to `bits` (LSB first) in every tile near the pad
/// that has it.
pub fn site_word(
    g: &Graph,
    db: &DeviceDb,
    site: PioSite,
    full: &str,
    bits: &[bool],
    tiles: &mut BTreeMap<String, TileConfig>,
) -> Result<usize> {
    let mut n = 0;
    for t in g
        .tiles
        .iter()
        .filter(|t| near(g, site, t.x, t.y))
    {
        let Some(w) = db.tile_bits(&t.ty)?.words.get(full) else {
            continue;
        };
        if w.bits.len() != bits.len() {
            return Err(err!("{}: {full} is {} bits, not {}", t.name, w.bits.len(), bits.len()));
        }
        tiles
            .entry(t.name.clone())
            .or_default()
            .words
            .push((full.to_owned(), bits.to_vec()));
        n += 1;
    }
    if n == 0 {
        return Err(err!("no tile near R{}C{} has {full}", site.row, site.col));
    }
    Ok(n)
}

/// An arc a constant can pass with no setting: a fixed connection, or the
/// mux arc with no bits (the mux's state when nothing is selected), as on
/// the MULT18 A / B inputs (JMULTA8 <- JMUIA8 is the default; JPO8, the
/// cascade, is the one with a bit). nextpnr's tie_cib_signal walks every
/// uphill pip; this keeps to the arcs that need no configuration.
fn free_arc(g: &Graph, db: &DeviceDb, a: &crate::graph::Arc) -> Result<bool> {
    let pip = g.pip(a);
    if pip.kind == PipKind::Fixed {
        return Ok(true);
    }
    let ty = &g.tiles[a.tile as usize].ty;
    Ok(db
        .tile_bits(ty)?
        .muxes
        .get(&pip.sink)
        .and_then(|m| m.arcs.get(&pip.source))
        .is_some_and(|bg| bg.0.is_empty()))
}
