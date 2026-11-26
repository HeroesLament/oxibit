//! Builds the tables from a Project Trellis `tiledata` tree.

use std::collections::{BTreeMap, HashMap};
use std::path::Path;

use oxibit_ecp5::db::{parse_bits_db, BitGroup, DeviceDb, TileBits};

use serde_json::Value;

use crate::model::{
    Chip, Device, Fuse, Groups, Package, Pin, PioMeta, Quadrant, Spine, Strings, Tap, Tile, TileType, NONE, NO_DEFAULT,
};

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

fn empty_groups() -> Groups {
    Groups { key_start: vec![0], item_start: vec![0], ..Groups::default() }
}

fn push_item(g: &mut Groups, i: &mut Interner, name: &str, bits: &BitGroup) -> Result<(), String> {
    g.item_name.push(i.id(name)?);
    g.fuses.extend(run(bits)?);
    g.item_start.push(u32::try_from(g.fuses.len()).map_err(|_| "too many fuses".to_owned())?);
    Ok(())
}

fn close_key(g: &mut Groups) -> Result<(), String> {
    g.key_start.push(u32::try_from(g.item_name.len()).map_err(|_| "too many items".to_owned())?);
    Ok(())
}

/// `want` is `(key, [(item, bits)])` in sorted key order.
fn flatten(i: &mut Interner, want: &[(&String, Vec<(&String, &BitGroup)>)]) -> Result<Groups, String> {
    let mut g = empty_groups();
    for (key, items) in want {
        g.keys.push(i.id(key)?);
        for (name, bits) in items {
            push_item(&mut g, i, name, bits)?;
        }
        close_key(&mut g)?;
    }
    Ok(g)
}

fn tile_type(i: &mut Interner, name: u32, tb: &TileBits) -> Result<TileType, String> {
    let muxes = flatten(i, &mux_lists(tb))?;
    let enums = flatten(i, &enum_lists(tb))?;
    let mut enum_def = Vec::new();
    for e in tb.enums.values() {
        enum_def.push(match &e.defval {
            Some(s) => i.id(s)?,
            None => NO_DEFAULT,
        });
    }
    let mut words = empty_groups();
    let mut word_def_start = vec![0];
    let mut word_def = Vec::new();
    for (key, w) in &tb.words {
        words.keys.push(i.id(key)?);
        for (n, bits) in w.bits.iter().enumerate() {
            push_item(&mut words, i, &n.to_string(), bits)?;
        }
        close_key(&mut words)?;
        word_def.extend(w.defval.iter().map(|b| u8::from(*b)));
        word_def_start.push(u32::try_from(word_def.len()).map_err(|_| "too many default bits".to_owned())?);
    }
    let mut fixed = Vec::new();
    for (sink, source) in &tb.fixed_conns {
        fixed.push(i.id(sink)?);
        fixed.push(i.id(source)?);
    }
    Ok(TileType { name, muxes, enums, enum_def, words, word_def_start, word_def, fixed })
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
        tile_types.push(tile_type(&mut i, name, tb)?);
    }
    Ok(Chip { strings: i.strings, tile_types, devices: Vec::new() })
}

fn u32_of(v: usize, what: &str) -> Result<u32, String> {
    u32::try_from(v).map_err(|_| format!("{what} does not fit u32"))
}

fn read_json(path: &Path) -> Result<Value, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    serde_json::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))
}

/// Only the keys in `allowed`; anything else would be dropped silently.
fn only_keys(v: &Value, allowed: &[&str], what: &str) -> Result<(), String> {
    let obj = v.as_object().ok_or_else(|| format!("{what}: not an object"))?;
    match obj.keys().find(|k| !allowed.contains(&k.as_str())) {
        Some(k) => Err(format!("{what}: unexpected key {k}")),
        None => Ok(()),
    }
}

fn num(v: &Value, key: &str, what: &str) -> Result<u32, String> {
    v.get(key)
        .and_then(Value::as_u64)
        .and_then(|n| u32::try_from(n).ok())
        .ok_or_else(|| format!("{what}: missing or bad {key}"))
}

fn text(i: &mut Interner, v: &Value, key: &str, what: &str) -> Result<u32, String> {
    let s = v.get(key).and_then(Value::as_str).ok_or_else(|| format!("{what}: missing or bad {key}"))?;
    i.id(s)
}

fn opt_text(i: &mut Interner, v: &Value, key: &str, what: &str) -> Result<u32, String> {
    match v.get(key) {
        None => Ok(NONE),
        Some(Value::String(s)) => i.id(s),
        Some(_) => Err(format!("{what}: bad {key}")),
    }
}

fn section<'a>(v: &'a Value, key: &str) -> Result<&'a serde_json::Map<String, Value>, String> {
    v.get(key).and_then(Value::as_object).ok_or_else(|| format!("missing {key}"))
}

type Landmarks = (Vec<Quadrant>, Vec<Tap>, Vec<Spine>);

fn read_globals(i: &mut Interner, path: &Path) -> Result<Landmarks, String> {
    let g = read_json(path)?;
    only_keys(&g, &["quadrants", "taps", "spines"], "globals.json")?;
    let mut quadrants = Vec::new();
    for (name, r) in section(&g, "quadrants")? {
        only_keys(r, &["x0", "y0", "x1", "y1"], name)?;
        quadrants.push(Quadrant {
            name: i.id(name)?,
            x0: num(r, "x0", name)?,
            y0: num(r, "y0", name)?,
            x1: num(r, "x1", name)?,
            y1: num(r, "y1", name)?,
        });
    }
    let mut taps = Vec::new();
    for (name, t) in section(&g, "taps")? {
        only_keys(t, &["lx0", "lx1", "rx0", "rx1"], name)?;
        taps.push(Tap {
            name: i.id(name)?,
            lx0: num(t, "lx0", name)?,
            lx1: num(t, "lx1", name)?,
            rx0: num(t, "rx0", name)?,
            rx1: num(t, "rx1", name)?,
        });
    }
    let mut spines = Vec::new();
    for (name, s) in section(&g, "spines")? {
        only_keys(s, &["x", "y"], name)?;
        spines.push(Spine { name: i.id(name)?, x: num(s, "x", name)?, y: num(s, "y", name)? });
    }
    Ok((quadrants, taps, spines))
}

fn read_iodb(i: &mut Interner, path: &Path) -> Result<(Vec<Package>, Vec<PioMeta>), String> {
    let d = read_json(path)?;
    only_keys(&d, &["packages", "pio_metadata"], "iodb.json")?;
    let mut packages = Vec::new();
    for (pkg, pins) in section(&d, "packages")? {
        let mut out = Vec::new();
        for (pin, v) in pins.as_object().ok_or_else(|| format!("{pkg}: not an object"))? {
            only_keys(v, &["col", "row", "pio"], pin)?;
            out.push(Pin { name: i.id(pin)?, col: num(v, "col", pin)?, row: num(v, "row", pin)?, pio: text(i, v, "pio", pin)? });
        }
        packages.push(Package { name: i.id(pkg)?, pins: out });
    }
    let mut pio_meta = Vec::new();
    for m in d.get("pio_metadata").and_then(Value::as_array).ok_or("missing pio_metadata")? {
        only_keys(m, &["bank", "col", "row", "pio", "dqs", "function"], "pio_metadata")?;
        pio_meta.push(PioMeta {
            bank: num(m, "bank", "pio_metadata")?,
            col: num(m, "col", "pio_metadata")?,
            row: num(m, "row", "pio_metadata")?,
            pio: text(i, m, "pio", "pio_metadata")?,
            dqs: opt_text(i, m, "dqs", "pio_metadata")?,
            function: opt_text(i, m, "function", "pio_metadata")?,
        });
    }
    Ok((packages, pio_meta))
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
    let dir = db.root.join(&n.family).join(&n.name);
    let (quadrants, taps, spines) = read_globals(&mut i, &dir.join("globals.json"))?;
    let (packages, pio_meta) = read_iodb(&mut i, &dir.join("iodb.json"))?;
    let device = Device {
        quadrants,
        taps,
        spines,
        packages,
        pio_meta,
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
