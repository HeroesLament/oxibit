//! Rebuilds `oxibit-ecp5`'s `DeviceDb` from a chip, so the flat tables can be
//! checked against the text they came from, field for field.

use std::collections::{BTreeMap, BTreeSet};
use std::path::PathBuf;

use oxibit_ecp5::db::{BitGroup, ConfigBit, DeviceDb, DeviceInfo, Enum, Mux, TileBits, TileInfo, Word};

use crate::model::{Chip, Fuse, TileType, NO_DEFAULT};

fn group(fuses: &[u32]) -> BitGroup {
    BitGroup(
        fuses
            .iter()
            .map(|&r| {
                let f = Fuse::from_raw(r);
                ConfigBit { frame: f.frame(), bit: f.bit(), inv: f.inverted() }
            })
            .collect(),
    )
}

fn tile_bits(chip: &Chip, t: &TileType) -> TileBits {
    let s = &chip.strings;
    let mut tb = TileBits::default();
    for k in 0..t.muxes.keys.len() {
        let arcs: BTreeMap<String, BitGroup> =
            t.muxes.items(k).map(|(src, f)| (s.get(src).to_owned(), group(f))).collect();
        tb.muxes.insert(s.get(t.muxes.keys[k]).to_owned(), Mux { arcs });
    }
    for k in 0..t.enums.keys.len() {
        let options = t.enums.items(k).map(|(o, f)| (s.get(o).to_owned(), group(f))).collect();
        let defval = (t.enum_def[k] != NO_DEFAULT).then(|| s.get(t.enum_def[k]).to_owned());
        tb.enums.insert(s.get(t.enums.keys[k]).to_owned(), Enum { options, defval });
    }
    for k in 0..t.words.keys.len() {
        let bits = t.words.items(k).map(|(_, f)| group(f)).collect();
        let d = &t.word_def[t.word_def_start[k] as usize..t.word_def_start[k + 1] as usize];
        let defval = d.iter().map(|&b| b != 0).collect();
        tb.words.insert(s.get(t.words.keys[k]).to_owned(), Word { bits, defval });
    }
    tb.fixed_conns = t.fixed.chunks_exact(2).map(|p| (s.get(p[0]).to_owned(), s.get(p[1]).to_owned())).collect();
    tb
}

/// The `DeviceDb` of device `device`, holding the tile types its grid uses.
///
/// # Errors
///
/// When `device` is out of range.
pub fn to_device_db(chip: &Chip, device: usize, root: PathBuf) -> Result<DeviceDb, String> {
    let d = chip.devices.get(device).ok_or_else(|| "no such device".to_owned())?;
    let s = &chip.strings;
    let info = DeviceInfo {
        family: s.get(d.family).to_owned(),
        name: s.get(d.name).to_owned(),
        idcode: d.idcode,
        frames: d.frames as usize,
        bits_per_frame: d.bits_per_frame as usize,
        pad_bits_after_frame: d.pad_bits_after_frame as usize,
        pad_bits_before_frame: d.pad_bits_before_frame as usize,
        max_row: d.max_row as usize,
        max_col: d.max_col as usize,
    };
    let mut tiles = BTreeMap::new();
    let mut used = BTreeSet::new();
    for t in &d.tiles {
        let ty = t.ty as usize;
        used.insert(ty);
        let name = s.get(t.name).to_owned();
        tiles.insert(
            name.clone(),
            TileInfo {
                name,
                ty: s.get(chip.tile_types[ty].name).to_owned(),
                start_frame: t.start_frame as usize,
                start_bit: t.start_bit as usize,
                frames: t.frames as usize,
                bits: t.bits as usize,
            },
        );
    }
    let bits = used
        .into_iter()
        .map(|ty| (s.get(chip.tile_types[ty].name).to_owned(), tile_bits(chip, &chip.tile_types[ty])))
        .collect();
    Ok(DeviceDb { root, info, tiles, bits })
}
