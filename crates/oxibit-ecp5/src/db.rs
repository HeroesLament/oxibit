//! Project Trellis database, one device at a time.
//!
//! `root` is Trellis's `database` directory (holding `devices.json`). A
//! device's tile grid is `<root>/<family>/<device>/tilegrid.json`; a tile
//! type's bits are `<root>/<family>/tiledata/<type>/bits.db`. Every tile type
//! in the grid is loaded up front, so a configuration never reaches the
//! filesystem.
//!
//! `bits.db` is read with the same tokenisation as libtrellis's stream
//! operators (`Util.hpp`): `#` starts a comment that ends the line, a record
//! ends at the next line starting with `.` or at end of file, and a bit group
//! ends at end of line or at a lone `-` (the empty group).

use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::{Path, PathBuf};

use serde_json::Value;

use crate::{Result, err};

/// One configuration bit of a tile, relative to the tile's CRAM window.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ConfigBit {
    pub frame: usize,
    pub bit: usize,
    /// The bit is set by being cleared (`!F<f>B<b>`).
    pub inv: bool,
}

impl ConfigBit {
    /// `F<frame>B<bit>`, optionally prefixed `!`.
    pub fn parse(s: &str) -> Result<ConfigBit> {
        let (inv, rest) = match s.strip_prefix('!') {
            Some(r) => (true, r),
            None => (false, s),
        };
        let rest = rest
            .strip_prefix('F')
            .ok_or_else(|| err!("config bit {s:?}: no F"))?;
        let (frame, bit) = rest
            .split_once('B')
            .ok_or_else(|| err!("config bit {s:?}: no B"))?;
        let frame = frame
            .parse()
            .map_err(|e| err!("config bit {s:?}: frame: {e}"))?;
        let bit = bit
            .parse()
            .map_err(|e| err!("config bit {s:?}: bit: {e}"))?;
        Ok(ConfigBit { frame, bit, inv })
    }
}

/// A set of bits that are set (or cleared) together.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct BitGroup(pub BTreeSet<ConfigBit>);

/// The arcs into one routing mux sink, by source wire name.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Mux {
    pub arcs: BTreeMap<String, BitGroup>,
}

/// A multi-bit setting; `bits[i]` holds bit `i` of the value.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Word {
    pub bits: Vec<BitGroup>,
    pub defval: Vec<bool>,
}

/// A setting with named options.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Enum {
    pub options: BTreeMap<String, BitGroup>,
    pub defval: Option<String>,
}

/// The bit database of one tile type.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct TileBits {
    pub muxes: BTreeMap<String, Mux>,
    pub words: BTreeMap<String, Word>,
    pub enums: BTreeMap<String, Enum>,
    /// `(sink, source)` wires joined with no configuration bit.
    pub fixed_conns: Vec<(String, String)>,
}

/// Family and frame geometry of a device, from `devices.json`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DeviceInfo {
    pub family: String,
    pub name: String,
    pub idcode: u32,
    pub frames: usize,
    pub bits_per_frame: usize,
    pub pad_bits_after_frame: usize,
    pub pad_bits_before_frame: usize,
    /// Highest tile row and column of the grid.
    pub max_row: usize,
    pub max_col: usize,
}

/// A tile's place in the grid and its CRAM window.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TileInfo {
    pub name: String,
    pub ty: String,
    pub start_frame: usize,
    pub start_bit: usize,
    /// Frames in the window (`cols` in `tilegrid.json`).
    pub frames: usize,
    /// Bits per frame in the window (`rows` in `tilegrid.json`).
    pub bits: usize,
}

#[derive(Debug, Clone)]
pub struct DeviceDb {
    pub root: PathBuf,
    pub info: DeviceInfo,
    /// By tile name, which is the order libtrellis visits tiles in.
    pub tiles: BTreeMap<String, TileInfo>,
    /// By tile type.
    pub bits: BTreeMap<String, TileBits>,
}

impl DeviceDb {
    pub fn load(root: &Path, device: &str) -> Result<DeviceDb> {
        let info = device_info(root, device)?;
        let dev_dir = root.join(&info.family).join(&info.name);
        let tiles = tilegrid(&dev_dir.join("tilegrid.json"))?;
        let mut bits = BTreeMap::new();
        for ty in tiles
            .values()
            .map(|t| t.ty.as_str())
            .collect::<BTreeSet<_>>()
        {
            let path = root
                .join(&info.family)
                .join("tiledata")
                .join(ty)
                .join("bits.db");
            let text = fs::read_to_string(&path).map_err(|e| err!("{}: {e}", path.display()))?;
            let tb = parse_bits_db(&text).map_err(|e| err!("{}: {}", path.display(), e.0))?;
            bits.insert(ty.to_owned(), tb);
        }
        Ok(DeviceDb {
            root: root.to_owned(),
            info,
            tiles,
            bits,
        })
    }

    pub fn tile_bits(&self, ty: &str) -> Result<&TileBits> {
        self.bits
            .get(ty)
            .ok_or_else(|| err!("no bit database for tile type {ty}"))
    }
}

fn json_usize(v: &Value, key: &str, ctx: &str) -> Result<usize> {
    let n = v.get(key).ok_or_else(|| err!("{ctx}: no {key}"))?;
    match n {
        Value::Number(n) => n.as_u64().and_then(|n| usize::try_from(n).ok()),
        Value::String(s) => s.parse().ok(),
        _ => None,
    }
    .ok_or_else(|| err!("{ctx}: {key} is not a count"))
}

fn device_info(root: &Path, device: &str) -> Result<DeviceInfo> {
    let path = root.join("devices.json");
    let text = fs::read_to_string(&path).map_err(|e| err!("{}: {e}", path.display()))?;
    let v: Value = serde_json::from_str(&text).map_err(|e| err!("{}: {e}", path.display()))?;
    let families = v
        .get("families")
        .and_then(Value::as_object)
        .ok_or_else(|| err!("devices.json: no families"))?;
    for (family, fam) in families {
        let Some(dev) = fam.get("devices").and_then(|d| d.get(device)) else {
            continue;
        };
        let idcode = dev
            .get("idcode")
            .and_then(Value::as_str)
            .ok_or_else(|| err!("{device}: no idcode"))?;
        let idcode = u32::from_str_radix(idcode.trim_start_matches("0x"), 16)
            .map_err(|e| err!("{device}: idcode {idcode:?}: {e}"))?;
        return Ok(DeviceInfo {
            family: family.clone(),
            name: device.to_owned(),
            idcode,
            frames: json_usize(dev, "frames", device)?,
            bits_per_frame: json_usize(dev, "bits_per_frame", device)?,
            pad_bits_after_frame: json_usize(dev, "pad_bits_after_frame", device)?,
            pad_bits_before_frame: json_usize(dev, "pad_bits_before_frame", device)?,
            max_row: json_usize(dev, "max_row", device)?,
            max_col: json_usize(dev, "max_col", device)?,
        });
    }
    Err(err!("device {device} not in {}", path.display()))
}

/// The device with this IDCODE, by name.
pub fn device_for_idcode(root: &Path, idcode: u32) -> Result<String> {
    let path = root.join("devices.json");
    let text = fs::read_to_string(&path).map_err(|e| err!("{}: {e}", path.display()))?;
    let v: Value = serde_json::from_str(&text).map_err(|e| err!("{}: {e}", path.display()))?;
    let families = v
        .get("families")
        .and_then(Value::as_object)
        .ok_or_else(|| err!("devices.json: no families"))?;
    for fam in families.values() {
        let Some(devices) = fam.get("devices").and_then(Value::as_object) else {
            continue;
        };
        for (name, dev) in devices {
            let id = dev
                .get("idcode")
                .and_then(Value::as_str)
                .map(|s| u32::from_str_radix(s.trim_start_matches("0x"), 16));
            if let Some(Ok(id)) = id {
                if id == idcode {
                    return Ok(name.clone());
                }
            }
        }
    }
    Err(err!(
        "no device with IDCODE 0x{idcode:08x} in {}",
        path.display()
    ))
}

fn tilegrid(path: &Path) -> Result<BTreeMap<String, TileInfo>> {
    let text = fs::read_to_string(path).map_err(|e| err!("{}: {e}", path.display()))?;
    let v: Value = serde_json::from_str(&text).map_err(|e| err!("{}: {e}", path.display()))?;
    let grid = v
        .as_object()
        .ok_or_else(|| err!("{}: not an object", path.display()))?;
    let mut tiles = BTreeMap::new();
    for (name, t) in grid {
        let ty = t
            .get("type")
            .and_then(Value::as_str)
            .ok_or_else(|| err!("tile {name}: no type"))?;
        tiles.insert(
            name.clone(),
            TileInfo {
                name: name.clone(),
                ty: ty.to_owned(),
                start_frame: json_usize(t, "start_frame", name)?,
                start_bit: json_usize(t, "start_bit", name)?,
                frames: json_usize(t, "cols", name)?,
                bits: json_usize(t, "rows", name)?,
            },
        );
    }
    Ok(tiles)
}

/// libtrellis's stream tokenisation (`Util.hpp`) over a text.
#[derive(Debug)]
pub(crate) struct Tokens<'a> {
    s: &'a [u8],
    pos: usize,
}

impl<'a> Tokens<'a> {
    pub(crate) fn new(text: &'a str) -> Tokens<'a> {
        Tokens {
            s: text.as_bytes(),
            pos: 0,
        }
    }

    fn peek(&self) -> Option<u8> {
        self.s.get(self.pos).copied()
    }

    /// Spaces and tabs (libtrellis `skip_blank(in, false)`).
    fn skip_spaces(&mut self) {
        while matches!(self.peek(), Some(b' ' | b'\t')) {
            self.pos += 1;
        }
    }

    /// Spaces, tabs and line ends (libtrellis `skip_blank(in, true)`).
    fn skip_blank_lines(&mut self) {
        while matches!(self.peek(), Some(b' ' | b'\t' | b'\n' | b'\r')) {
            self.pos += 1;
        }
    }

    /// End of line or file; a comment counts as end of line.
    pub(crate) fn eol(&mut self) -> bool {
        self.skip_spaces();
        match self.peek() {
            None | Some(b'\n') => true,
            Some(b'#') => {
                while self.peek().is_some_and(|c| c != b'\n') {
                    self.pos += 1;
                }
                true
            }
            Some(_) => false,
        }
    }

    /// Past blank lines and comments.
    fn skip(&mut self) {
        self.skip_blank_lines();
        while self.peek() == Some(b'#') {
            self.eol();
            self.skip_blank_lines();
        }
    }

    /// End of record: a `.` line, or end of file.
    pub(crate) fn eor(&mut self) -> bool {
        self.skip();
        matches!(self.peek(), None | Some(b'.'))
    }

    pub(crate) fn eof(&mut self) -> bool {
        self.skip();
        self.peek().is_none()
    }

    /// `in >> string`: leading whitespace (newlines included) skipped, then
    /// up to the next whitespace.
    pub(crate) fn word(&mut self) -> Result<&'a str> {
        while self.peek().is_some_and(|c| c.is_ascii_whitespace()) {
            self.pos += 1;
        }
        let start = self.pos;
        while self.peek().is_some_and(|c| !c.is_ascii_whitespace()) {
            self.pos += 1;
        }
        if start == self.pos {
            return Err(err!("unexpected end of input at byte {start}"));
        }
        std::str::from_utf8(&self.s[start..self.pos]).map_err(|e| err!("byte {start}: {e}"))
    }

    /// `getline`: the rest of the line, without its newline.
    pub(crate) fn rest_of_line(&mut self) -> Result<&'a str> {
        let start = self.pos;
        while self.peek().is_some_and(|c| c != b'\n') {
            self.pos += 1;
        }
        let line =
            std::str::from_utf8(&self.s[start..self.pos]).map_err(|e| err!("byte {start}: {e}"))?;
        if self.peek() == Some(b'\n') {
            self.pos += 1;
        }
        Ok(line)
    }

    /// `ss.get()`: one character, whatever it is.
    pub(crate) fn skip_char(&mut self) {
        if self.pos < self.s.len() {
            self.pos += 1;
        }
    }

    /// A bit group: bits up to end of line, or a lone `-`.
    fn bit_group(&mut self) -> Result<BitGroup> {
        let mut g = BitGroup::default();
        while !self.eol() {
            let s = self.word()?;
            if s == "-" {
                break;
            }
            g.0.insert(ConfigBit::parse(s)?);
        }
        Ok(g)
    }
}

/// A bit vector as written by libtrellis: the last character is bit 0.
pub(crate) fn parse_bitvec(s: &str) -> Result<Vec<bool>> {
    s.bytes()
        .rev()
        .map(|c| match c {
            b'0' => Ok(false),
            b'1' => Ok(true),
            _ => Err(err!("bit vector {s:?}: {:?} is not 0 or 1", char::from(c))),
        })
        .collect()
}

pub fn parse_bits_db(text: &str) -> Result<TileBits> {
    let mut t = Tokens::new(text);
    let mut db = TileBits::default();
    while !t.eof() {
        match t.word()? {
            ".mux" => {
                let sink = t.word()?.to_owned();
                let mut mux = Mux::default();
                while !t.eor() {
                    let source = t.word()?.to_owned();
                    mux.arcs.insert(source, t.bit_group()?);
                }
                db.muxes.insert(sink, mux);
            }
            ".config" => {
                let name = t.word()?.to_owned();
                let defval = if t.eol() {
                    None
                } else {
                    Some(parse_bitvec(t.word()?)?)
                };
                let mut bits = Vec::new();
                while !t.eor() {
                    bits.push(t.bit_group()?);
                }
                let defval = defval.unwrap_or_else(|| vec![false; bits.len()]);
                if defval.len() != bits.len() {
                    return Err(err!(
                        "word {name}: default has {} bits, word has {}",
                        defval.len(),
                        bits.len()
                    ));
                }
                db.words.insert(name, Word { bits, defval });
            }
            ".config_enum" => {
                let name = t.word()?.to_owned();
                let defval = if t.eol() {
                    None
                } else {
                    Some(t.word()?.to_owned())
                };
                let mut options = BTreeMap::new();
                while !t.eor() {
                    let opt = t.word()?.to_owned();
                    options.insert(opt, t.bit_group()?);
                }
                db.enums.insert(name, Enum { options, defval });
            }
            ".fixed_conn" => {
                let sink = t.word()?.to_owned();
                let source = t.word()?.to_owned();
                db.fixed_conns.push((sink, source));
            }
            other => return Err(err!("unexpected token {other:?}")),
        }
    }
    Ok(db)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn records() {
        let db = parse_bits_db(
            "# Routing Mux Bits\n.mux A0\nE1 F1B2 !F6B3\nF5 -\n\n.config W 01\nF0B0\nF0B1\n\n\
             .config_enum E OFF\nOFF -\nON F2B2 # comment\n.fixed_conn X Y\n",
        )
        .expect("parses");
        let a0 = &db.muxes["A0"];
        assert_eq!(a0.arcs["E1"].0.len(), 2);
        assert!(a0.arcs["F5"].0.is_empty());
        assert_eq!(db.words["W"].defval, vec![true, false]);
        assert_eq!(db.enums["E"].defval.as_deref(), Some("OFF"));
        assert_eq!(db.enums["E"].options["ON"].0.len(), 1);
        assert_eq!(db.fixed_conns, vec![("X".to_owned(), "Y".to_owned())]);
    }
}
