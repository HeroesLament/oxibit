//! PIO settings and constant tie-offs, as a design's configuration needs
//! them.
//!
//! A PIO's settings are spread over the tiles beside its pad: on the left
//! and right edges the three PIC tiles (`PICL0`/`PICL1`/`PICL2`, rows
//! `row..=row + 2`) each carry part of `PIO<l>.BASE_TYPE` and the other
//! enums, with their own bits. An enum is written into every tile near the
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

/// The tiles whose settings belong to a pad at (row, col): the PIC tiles
/// below it on the left and right edges, beside it on the top and bottom.
fn near(row: i16, col: i16, x: i16, y: i16) -> bool {
    (x == col && (row..=row + 2).contains(&y)) || (y == row && (col..=col + 1).contains(&x))
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
    let full = format!("PIO{}.{name}", site.pio);
    let mut n = 0;
    for t in g
        .tiles
        .iter()
        .filter(|t| near(site.row, site.col, t.x, t.y))
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
            if g.pip(&a).kind == PipKind::Fixed && seen.insert(a.from) {
                queue.push_back(a.from);
            }
        }
    }
    Err(err!(
        "{}: no constant mux uphill over fixed connections",
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
    let value = match level {
        Level::Low => "0",
        Level::High => "1",
    };
    let mut seen = HashSet::from([w]);
    let mut queue = VecDeque::from([w]);
    while let Some(w) = queue.pop_front() {
        let mux = format!("CIB.{}MUX", g.names[w.name as usize]);
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
            if g.pip(&a).kind == PipKind::Fixed && seen.insert(a.from) {
                queue.push_back(a.from);
            }
        }
    }
    Err(err!(
        "{}: no constant mux uphill over fixed connections",
        g.wire_name(w)
    ))
}
