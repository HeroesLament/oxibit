//! FASM -> frames: port of prjxray's `FasmAssembler` plus the part-level
//! steps of `utils/fasm2frames.py` (required features, STEPDOWN
//! propagation across an I/O bank). Output is the full, zero-filled frame
//! set fasm2frames writes without `--sparse`.

use std::collections::{BTreeMap, HashMap, HashSet};

use oxibit_bitstream::FRAME_WORDS;
use oxibit_bitstream::layout::FrameAddress;
use oxibit_bitstream::writer::Frame;

use crate::db::{Bits, Database, TileInfo, TileSegbits};
use crate::parse::{SetFeature, parse_line};

#[derive(Debug)]
pub enum Error {
    Parse(String),
    /// Features the database does not know, one message per feature.
    Lookup(Vec<String>),
    /// Two lines want the same bit at different values.
    Inconsistent(String),
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::Parse(s) => write!(f, "FASM syntax: {s}"),
            Error::Lookup(v) => write!(f, "{} unknown feature(s):\n{}", v.len(), v.join("\n")),
            Error::Inconsistent(s) => write!(f, "inconsistent bits: {s}"),
        }
    }
}

type Key = (u32, i64, u32); // frame, word (may be negative, see db::Bits), bit

pub struct Assembler<'a> {
    db: &'a Database,
    bits: HashMap<Key, (bool, usize)>,
    lines: Vec<String>,
    set_features: Vec<SetFeature>,
}

/// Assemble a whole FASM file the way `fasm2frames --part P` does.
pub fn assemble(db: &Database, fasm: &str) -> Result<BTreeMap<FrameAddress, Frame>, Error> {
    let mut a = Assembler { db, bits: HashMap::new(), lines: Vec::new(), set_features: Vec::new() };
    let mut missing = Vec::new();

    for raw in fasm.lines() {
        if let Some(sf) = parse_line(raw).map_err(Error::Parse)? {
            a.add(&sf, raw, &mut missing)?;
        }
    }
    for raw in &db.required_features {
        if let Some(sf) = parse_line(raw).map_err(Error::Parse)? {
            a.add(&sf, raw, &mut missing)?;
        }
    }
    if !missing.is_empty() {
        return Err(Error::Lookup(missing));
    }

    a.stepdown()?;
    Ok(a.frames())
}

impl<'a> Assembler<'a> {
    fn add(&mut self, sf: &SetFeature, raw: &str, missing: &mut Vec<String>) -> Result<(), Error> {
        self.set_features.push(sf.clone());
        let Some((tile, feature)) = sf.feature.split_once('.') else {
            missing.push(format!("no tile in '{raw}'"));
            return Ok(());
        };
        let line_no = self.lines.len();
        self.lines.push(raw.trim().to_string());

        for address in sf.addresses() {
            match self.feature_bits(tile, feature, address) {
                Ok(bits) => {
                    for (key, set) in bits {
                        self.set_bit(key, set, line_no)?;
                    }
                }
                Err(msg) => missing.push(format!("{msg} from line '{}'", raw.trim())),
            }
        }
        Ok(())
    }

    fn set_bit(&mut self, key: Key, set: bool, line_no: usize) -> Result<(), Error> {
        // fasm_assembler.frame_set/frame_clear: word >= 101 is reported and skipped.
        if key.1 >= FRAME_WORDS as i64 {
            eprintln!("oxibit: invalid word address {} in line: {}", key.1, self.lines[line_no]);
            return Ok(());
        }
        match self.bits.get(&key) {
            Some(&(prev, prev_line)) if prev != set => Err(Error::Inconsistent(format!(
                "'{}' wants bit {key:?} {} but '{}' {} it",
                self.lines[line_no],
                if set { "set" } else { "clear" },
                self.lines[prev_line],
                if prev { "set" } else { "cleared" }
            ))),
            Some(_) => Ok(()),
            None => {
                self.bits.insert(key, (set, line_no));
                Ok(())
            }
        }
    }

    /// `enable_feature` + `feature_to_bits` (plain or aliased tile).
    fn feature_bits(&self, tile: &str, feature: &str, address: u32) -> Result<Vec<(Key, bool)>, String> {
        let info = self.db.tiles.get(tile).ok_or_else(|| format!("tile {tile} not in tilegrid"))?;
        let db_k = format!("{}.{}", info.tile_type, feature);
        let own = self.segbits(&info.tile_type)?;

        let aliased = info.bits.iter().any(|(_, _, a)| a.is_some());
        if own.ppips.contains(&db_k) {
            return Ok(vec![]);
        }
        if !aliased {
            let bits_map: Vec<(u32, Bits)> = info.bits.iter().map(|(b, bits, _)| (*b, bits.clone())).collect();
            return to_bits(own, &bits_map, &db_k, address).ok_or_else(|| missing(info, &db_k));
        }

        // TileSegbitsAlias: rename tile type and sites, shift the offset.
        let alias_type = info.bits.iter().find_map(|(_, _, a)| a.as_ref()).unwrap().tile_type.clone();
        let mut parts: Vec<String> = db_k.split('.').map(String::from).collect();
        parts[0] = alias_type.clone();
        for (_, _, alias) in &info.bits {
            if let (Some(alias), true) = (alias, parts.len() > 1) {
                if let Some((_, to)) = alias.sites.iter().find(|(from, _)| *from == parts[1]) {
                    parts[1] = to.clone();
                }
            }
        }
        let alias_feature = parts.join(".");
        let bits_map: Vec<(u32, Bits)> = info
            .bits
            .iter()
            .map(|(b, bits, a)| {
                let start = a.as_ref().map(|a| a.start_offset).unwrap_or(0);
                (*b, Bits { offset: bits.offset - start, ..bits.clone() })
            })
            .collect();
        to_bits(self.segbits(&alias_type)?, &bits_map, &alias_feature, address).ok_or_else(|| missing(info, &db_k))
    }

    fn segbits(&self, tile_type: &str) -> Result<&'a TileSegbits, String> {
        self.db.segbits.get(&tile_type.to_uppercase()).ok_or_else(|| format!("no tile type {tile_type} in database"))
    }

    /// The STEPDOWN pass of fasm2frames: a STEPDOWN tag on any IOB33 site of
    /// a bank is repeated on every unused IOB33 site of that bank, and the
    /// bank's HCLK_IOI3 gets `.STEPDOWN`.
    fn stepdown(&mut self) -> Result<(), Error> {
        let mut tags: HashMap<String, HashSet<String>> = HashMap::new();
        let mut used: HashSet<(String, String)> = HashSet::new();

        for sf in &self.set_features {
            if sf.is_zero() {
                continue;
            }
            let parts: Vec<&str> = sf.feature.splitn(3, '.').collect();
            if parts.len() < 3 {
                continue;
            }
            let (tile, site, tag) = (parts[0], parts[1], parts[2]);
            if tile.contains("IOB33") {
                used.insert((tile.to_string(), site.to_string()));
            }
            if tag.contains("STEPDOWN") {
                let bank = self
                    .db
                    .tile_to_bank
                    .get(tile)
                    .ok_or_else(|| Error::Lookup(vec![format!("STEPDOWN on {tile}, which is in no bank")]))?;
                tags.entry(bank.clone()).or_default().insert(tag.to_string());
            }
        }

        let mut extra = Vec::new();
        for (bank, bank_tags) in &tags {
            for tile in self.db.bank_to_tiles.get(bank).into_iter().flatten() {
                if tile.contains("IOB33") {
                    for site in self.db.tiles.get(tile).map(|t| t.sites.clone()).unwrap_or_default() {
                        let y = site.chars().last().and_then(|c| c.to_digit(10)).unwrap_or(0) % 2;
                        let site = format!("IOB_Y{y}");
                        if used.contains(&(tile.clone(), site.clone())) {
                            continue;
                        }
                        for tag in bank_tags {
                            extra.push(format!("{tile}.{site}.{tag}"));
                        }
                    }
                }
                if tile.contains("HCLK_IOI3") {
                    extra.push(format!("{tile}.STEPDOWN"));
                }
            }
        }

        let mut missing = Vec::new();
        for raw in &extra {
            if let Some(sf) = parse_line(raw).map_err(Error::Parse)? {
                self.add(&sf, raw, &mut missing)?;
            }
        }
        if missing.is_empty() { Ok(()) } else { Err(Error::Lookup(missing)) }
    }

    /// `get_frames(sparse=False)`: every frame of every tile, zeroed, with
    /// the set bits ORed in.
    fn frames(&self) -> BTreeMap<FrameAddress, Frame> {
        let mut frames: BTreeMap<FrameAddress, Frame> = BTreeMap::new();
        for info in self.db.tiles.values() {
            for (_, bits, _) in &info.bits {
                for f in bits.base..bits.base + bits.frames {
                    frames.entry(FrameAddress(f)).or_insert([0; FRAME_WORDS]);
                }
            }
        }
        for (&(frame, word, bit), &(set, _)) in &self.bits {
            let f = frames.entry(FrameAddress(frame)).or_insert([0; FRAME_WORDS]);
            if set {
                // Negative words index from the end, as a Python list does.
                let w = if word < 0 { word + FRAME_WORDS as i64 } else { word } as usize;
                f[w] |= 1 << bit;
            }
        }
        frames
    }
}

fn missing(info: &TileInfo, db_k: &str) -> String {
    format!("Segment DB {}, key {db_k} not found", info.tile_type)
}

/// `TileSegbits.feature_to_bits` + `map_bit_to_frame`.
fn to_bits(seg: &TileSegbits, bits_map: &[(u32, Bits)], feature: &str, address: u32) -> Option<Vec<(Key, bool)>> {
    // Checked here as well as by the caller: for an aliased tile this is
    // the ALIAS type's ppips (e.g. RIOI3 for RIOI3_SING), which prjxray
    // consults inside the aliased lookup.
    if seg.ppips.contains(feature) {
        return Some(vec![]);
    }
    let map_bit = |block: u32, column: u32, bit: u32, set: bool| -> Option<(Key, bool)> {
        let bits = &bits_map.iter().find(|(b, _)| *b == block)?.1;
        let word_bit = bits.offset * 32 + bit as i64;
        Some(((bits.base + column, word_bit.div_euclid(32), word_bit.rem_euclid(32) as u32), set))
    };

    if address == 0 {
        for (block, map) in &seg.by_block {
            if let Some(sb) = map.get(feature) {
                return sb.iter().map(|b| map_bit(*block, b.column, b.bit, b.set)).collect();
            }
        }
    }
    let (block, full) = seg.addresses.get(feature)?.get(&address)?;
    let map = &seg.by_block.iter().find(|(b, _)| b == block)?.1;
    map.get(full)?.iter().map(|b| map_bit(*block, b.column, b.bit, b.set)).collect()
}

/// fasm2frames' `.frm` text: one line per frame, ascending address.
pub fn to_frm(frames: &BTreeMap<FrameAddress, Frame>) -> String {
    let mut s = String::with_capacity(frames.len() * 1120);
    for (a, words) in frames {
        s.push_str(&format!("0x{:08X} ", a.0));
        let w: Vec<String> = words.iter().map(|w| format!("0x{w:08X}")).collect();
        s.push_str(&w.join(","));
        s.push('\n');
    }
    s
}
