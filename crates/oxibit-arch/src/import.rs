//! Builds the tables from a Project Trellis `tiledata` tree.

use std::collections::{BTreeMap, HashMap};
use std::path::Path;

use oxibit_ecp5::db::{parse_bits_db, BitGroup, TileBits};

use crate::model::{Chip, Fuse, Groups, Strings, TileType};

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
    Ok(Chip { strings: i.strings, tile_types })
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
