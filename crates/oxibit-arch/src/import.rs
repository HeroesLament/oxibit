//! Builds the tables from a Project Trellis `tiledata` tree.

use std::collections::{BTreeMap, HashMap};
use std::path::Path;

use oxibit_ecp5::db::{parse_bits_db, BitGroup, DeviceDb, TileBits};

use crate::model::{Chip, Device, Fuse, Groups, Strings, Tile, TileType};

struct Interner {
    ids: HashMap<String, u32>,
    strings: Strings,
}

impl Interner {
    fn id(&mut self, s: &str) -> Result<u32, String> {
        if let Some(&id) = self.ids.get(s) {
            return Ok(id);
        }
        let id = self.strings.push(s)?;
        self.ids.insert(s.to_owned(), id);
        Ok(id)
    }
}

fn run(group: &BitGroup) -> Result<Vec<u32>, String> {
    group
        .0
        .iter()
        .map(|b| {
            Fuse::new(b.frame, b.bit, b.inv)
                .map(Fuse::raw)
                .ok_or_else(|| format!("fuse F{}B{} does not fit the packing", b.frame, b.bit))
        })
        .collect()
}

/// `want` is `(key, [(item, bits)])` in sorted key order.
fn flatten(i: &mut Interner, want: &[(&String, Vec<(&String, &BitGroup)>)]) -> Result<Groups, String> {
    let mut g = Groups { key_start: vec![0], item_start: vec![0], ..Groups::default() };
    for (key, items) in want {
        g.keys.push(i.id(key)?);
        for (name, bits) in items {
            g.item_name.push(i.id(name)?);
            g.fuses.extend(run(bits)?);
            g.item_start.push(u32::try_from(g.fuses.len()).map_err(|_| "too many fuses".to_owned())?);
        }
        g.key_start.push(u32::try_from(g.item_name.len()).map_err(|_| "too many items".to_owned())?);
    }
    Ok(g)
}

/// The same lists `flatten` takes, from a parsed tile type.
#[must_use]
pub fn mux_lists(tb: &TileBits) -> Vec<(&String, Vec<(&String, &BitGroup)>)> {
    tb.muxes.iter().map(|(k, m)| (k, m.arcs.iter().collect())).collect()
}

#[must_use]
pub fn enum_lists(tb: &TileBits) -> Vec<(&String, Vec<(&String, &BitGroup)>)> {
    tb.enums.iter().map(|(k, e)| (k, e.options.iter().collect())).collect()
}

/// Builds a chip from parsed tile types, given in name order.
///
/// # Errors
///
/// When a fuse does not fit the packing or a table outgrows `u32`.
pub fn from_tile_bits(tiles: &BTreeMap<String, TileBits>) -> Result<Chip, String> {
    let mut i = Interner { ids: HashMap::new(), strings: Strings::default() };
    let mut tile_types = Vec::new();
    for (name, tb) in tiles {
        let name = i.id(name)?;
        let muxes = flatten(&mut i, &mux_lists(tb))?;
        let enums = flatten(&mut i, &enum_lists(tb))?;
        tile_types.push(TileType { name, muxes, enums });
    }
    Ok(Chip { strings: i.strings, tile_types, devices: Vec::new() })
}

fn u32_of(v: usize, what: &str) -> Result<u32, String> {
    u32::try_from(v).map_err(|_| format!("{what} does not fit u32"))
}

/// Adds a device's geometry and tile grid. Its tile types must already be in
/// the chip.
///
/// # Errors
///
/// When a tile names a type the chip does not hold, or a value overflows.
pub fn add_device(chip: &mut Chip, db: &DeviceDb) -> Result<(), String> {
    let mut i = Interner { ids: HashMap::new(), strings: std::mem::take(&mut chip.strings) };
    for id in 0..i.strings.len() {
        let s = i.strings.get(u32_of(id, "string id")?).to_owned();
        i.ids.insert(s, u32_of(id, "string id")?);
    }
    let mut tiles = Vec::new();
    for (name, t) in &db.tiles {
        let ty = chip
            .tile_types
            .binary_search_by(|x| i.strings.get(x.name).cmp(t.ty.as_str()))
            .map_err(|_| format!("tile {name}: no tile type {}", t.ty))?;
        tiles.push(Tile {
            name: i.id(name)?,
            ty: u32_of(ty, "tile type index")?,
            start_frame: u32_of(t.start_frame, "start_frame")?,
            start_bit: u32_of(t.start_bit, "start_bit")?,
            frames: u32_of(t.frames, "frames")?,
            bits: u32_of(t.bits, "bits")?,
        });
    }
    let n = &db.info;
    let device = Device {
        name: i.id(&n.name)?,
        family: i.id(&n.family)?,
        idcode: n.idcode,
        frames: u32_of(n.frames, "frames")?,
        bits_per_frame: u32_of(n.bits_per_frame, "bits_per_frame")?,
        pad_bits_after_frame: u32_of(n.pad_bits_after_frame, "pad_bits_after_frame")?,
        pad_bits_before_frame: u32_of(n.pad_bits_before_frame, "pad_bits_before_frame")?,
        max_row: u32_of(n.max_row, "max_row")?,
        max_col: u32_of(n.max_col, "max_col")?,
        tiles,
    };
    chip.strings = i.strings;
    chip.devices.push(device);
    Ok(())
}

/// Reads every `<root>/<family>/tiledata/<type>/bits.db`; returns the parsed
/// tile types and the bytes of text read.
///
/// # Errors
///
/// On an unreadable or unparsable file.
pub fn read_tiledata(root: &Path, family: &str) -> Result<(BTreeMap<String, TileBits>, usize), String> {
    let dir = root.join(family).join("tiledata");
    let mut out = BTreeMap::new();
    let mut bytes = 0;
    for e in std::fs::read_dir(&dir).map_err(|e| format!("{}: {e}", dir.display()))? {
        let e = e.map_err(|e| e.to_string())?;
        let file = e.path().join("bits.db");
        let Ok(text) = std::fs::read_to_string(&file) else { continue };
        bytes += text.len();
        let tb = parse_bits_db(&text).map_err(|err| format!("{}: {err}", file.display()))?;
        out.insert(e.file_name().to_string_lossy().into_owned(), tb);
    }
    Ok((out, bytes))
}
