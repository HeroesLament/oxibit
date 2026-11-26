//! The in-memory tables.

use std::ops::Range;

/// One configuration bit packed in a `u32`: frame (16 bits), bit (15 bits),
/// and a flag for a bit that is set by being cleared.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Fuse(u32);

impl Fuse {
    /// `None` when the frame or bit does not fit the packing.
    #[must_use]
    pub fn new(frame: usize, bit: usize, inverted: bool) -> Option<Fuse> {
        let f = u16::try_from(frame).ok()?;
        let b = u16::try_from(bit).ok().filter(|b| *b < 0x8000)?;
        Some(Fuse(u32::from(f) | (u32::from(b) << 16) | (u32::from(inverted) << 31)))
    }

    #[must_use]
    pub fn frame(self) -> usize {
        (self.0 & 0xffff) as usize
    }

    #[must_use]
    pub fn bit(self) -> usize {
        ((self.0 >> 16) & 0x7fff) as usize
    }

    #[must_use]
    pub fn inverted(self) -> bool {
        self.0 >> 31 == 1
    }

    #[must_use]
    pub fn raw(self) -> u32 {
        self.0
    }

    #[must_use]
    pub fn from_raw(raw: u32) -> Fuse {
        Fuse(raw)
    }
}

/// Interned strings: one text buffer and the end offset of each string.
#[derive(Default, Clone, PartialEq, Eq, Debug)]
pub struct Strings {
    text: String,
    ends: Vec<u32>,
}

impl Strings {
    /// Appends a string, returning its id.
    ///
    /// # Errors
    ///
    /// When the table outgrows `u32` offsets.
    pub fn push(&mut self, s: &str) -> Result<u32, String> {
        let id = u32::try_from(self.ends.len()).map_err(|_| "string table too large".to_owned())?;
        self.text.push_str(s);
        let end = u32::try_from(self.text.len()).map_err(|_| "string table too large".to_owned())?;
        self.ends.push(end);
        Ok(id)
    }

    /// Rebuilds a table from its parts, checking the offsets.
    ///
    /// # Errors
    ///
    /// When offsets decrease, overrun the text, or split a character.
    pub fn from_parts(text: String, ends: Vec<u32>) -> Result<Strings, String> {
        let mut prev = 0usize;
        for &e in &ends {
            let e = e as usize;
            if e < prev || e > text.len() || !text.is_char_boundary(e) {
                return Err("bad string offsets".to_owned());
            }
            prev = e;
        }
        if prev != text.len() {
            return Err("string text longer than its offsets".to_owned());
        }
        Ok(Strings { text, ends })
    }

    #[must_use]
    pub fn len(&self) -> usize {
        self.ends.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.ends.is_empty()
    }

    #[must_use]
    pub fn text(&self) -> &str {
        &self.text
    }

    #[must_use]
    pub fn ends(&self) -> &[u32] {
        &self.ends
    }

    /// The string with this id; ids come from this table.
    #[must_use]
    pub fn get(&self, id: u32) -> &str {
        let i = id as usize;
        let end = self.ends[i] as usize;
        let start = if i == 0 { 0 } else { self.ends[i - 1] as usize };
        &self.text[start..end]
    }
}

/// Keys, each with named items, each item with a run of fuses, all flat.
///
/// A routing mux is a key (the sink wire) whose items are the source wires,
/// each with the fuses that select it. An enum is a key whose items are its
/// options. `keys` is sorted by name so a key is found by binary search.
#[derive(Default, Clone, PartialEq, Eq, Debug)]
pub struct Groups {
    pub keys: Vec<u32>,
    pub key_start: Vec<u32>,
    pub item_name: Vec<u32>,
    pub item_start: Vec<u32>,
    pub fuses: Vec<u32>,
}

fn monotone_to(v: &[u32], end: usize) -> bool {
    v.first() == Some(&0) && v.windows(2).all(|w| w[0] <= w[1]) && v.last().map(|l| *l as usize) == Some(end)
}

impl Groups {
    /// Checks that every offset and string id is in range.
    ///
    /// # Errors
    ///
    /// Describes the first inconsistency.
    pub fn validate(&self, strings: usize) -> Result<(), String> {
        if self.key_start.len() != self.keys.len() + 1 || !monotone_to(&self.key_start, self.item_name.len()) {
            return Err("bad key offsets".to_owned());
        }
        if self.item_start.len() != self.item_name.len() + 1 || !monotone_to(&self.item_start, self.fuses.len()) {
            return Err("bad item offsets".to_owned());
        }
        if self.keys.iter().chain(&self.item_name).any(|&s| s as usize >= strings) {
            return Err("string id out of range".to_owned());
        }
        Ok(())
    }

    /// Index of the key named `name`.
    #[must_use]
    pub fn find(&self, strings: &Strings, name: &str) -> Option<usize> {
        self.keys.binary_search_by(|&k| strings.get(k).cmp(name)).ok()
    }

    #[must_use]
    pub fn item_range(&self, key: usize) -> Range<usize> {
        self.key_start[key] as usize..self.key_start[key + 1] as usize
    }

    /// The items of a key: `(name id, raw fuses)`.
    pub fn items(&self, key: usize) -> impl Iterator<Item = (u32, &[u32])> + '_ {
        self.item_range(key).map(move |i| {
            let run = self.item_start[i] as usize..self.item_start[i + 1] as usize;
            (self.item_name[i], &self.fuses[run])
        })
    }
}

/// The configuration knowledge of one tile type.
#[derive(Default, Clone, PartialEq, Eq, Debug)]
pub struct TileType {
    pub name: u32,
    pub muxes: Groups,
    pub enums: Groups,
}

/// One tile of a device: its type and its window of configuration memory.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Tile {
    pub name: u32,
    /// Index into [`Chip::tile_types`].
    pub ty: u32,
    pub start_frame: u32,
    pub start_bit: u32,
    pub frames: u32,
    pub bits: u32,
}

/// One device: frame geometry and its tiles, sorted by name.
#[derive(Default, Clone, PartialEq, Eq, Debug)]
pub struct Device {
    pub name: u32,
    pub family: u32,
    pub idcode: u32,
    pub frames: u32,
    pub bits_per_frame: u32,
    pub pad_bits_after_frame: u32,
    pub pad_bits_before_frame: u32,
    pub max_row: u32,
    pub max_col: u32,
    pub tiles: Vec<Tile>,
}

impl Device {
    /// The tile named `name`; tiles are sorted by name.
    #[must_use]
    pub fn tile(&self, strings: &Strings, name: &str) -> Option<&Tile> {
        self.tiles.binary_search_by(|t| strings.get(t.name).cmp(name)).ok().map(|i| &self.tiles[i])
    }
}

/// A device family's tile types, its devices, and the string table they share.
#[derive(Default, Clone, PartialEq, Eq, Debug)]
pub struct Chip {
    pub strings: Strings,
    pub tile_types: Vec<TileType>,
    pub devices: Vec<Device>,
}

impl Chip {
    /// Checks every table.
    ///
    /// # Errors
    ///
    /// Describes the first inconsistency.
    pub fn validate(&self) -> Result<(), String> {
        let n = self.strings.len();
        for t in &self.tile_types {
            if t.name as usize >= n {
                return Err("tile type name out of range".to_owned());
            }
            t.muxes.validate(n)?;
            t.enums.validate(n)?;
        }
        for d in &self.devices {
            if d.name as usize >= n || d.family as usize >= n {
                return Err("device name out of range".to_owned());
            }
            for t in &d.tiles {
                if t.name as usize >= n || t.ty as usize >= self.tile_types.len() {
                    return Err("tile reference out of range".to_owned());
                }
            }
            if d.tiles.windows(2).any(|w| self.strings.get(w[0].name) >= self.strings.get(w[1].name)) {
                return Err("tiles not sorted by name".to_owned());
            }
        }
        Ok(())
    }

    /// Index of the device named `name`.
    #[must_use]
    pub fn device(&self, name: &str) -> Option<usize> {
        self.devices.iter().position(|d| self.strings.get(d.name) == name)
    }

    /// Index of the tile type named `name`; tile types are sorted by name.
    #[must_use]
    pub fn tile_type(&self, name: &str) -> Option<usize> {
        self.tile_types.binary_search_by(|t| self.strings.get(t.name).cmp(name)).ok()
    }
}
