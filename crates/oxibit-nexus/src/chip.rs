//! A configured device: per-tile CRAM windows, the whole-chip CRAM, and the
//! IP configuration space.
//!
//! `Chip::from_fasm` applies features in prjoxide's order, which matters
//! because features can share bits (an enum option clears what another set,
//! a later tile's window overwrites an overlapping earlier one):
//! 1. every tile's `always_on` bits;
//! 2. FASM tiles in name order; within a plain tile, `BASE_` enums, other
//!    enums, words, pips, unknown bits; within a tilegroup, the same enum
//!    phases then words, each applied to every member tile that has it;
//!    `IP_` prefixes go to the IP space;
//! 3. tile windows copied into the chip CRAM in tile name order.

use std::collections::{BTreeMap, HashMap};

use crate::bels::{bel_tiles, tile_bels};
use crate::db::{DeviceDb, TileBits};
use crate::fasm::{FasmTile, ParsedFasm};
use crate::{Result, err};

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BitMatrix {
    pub frames: usize,
    pub bits: usize,
    data: Vec<bool>,
}

impl BitMatrix {
    pub fn new(frames: usize, bits: usize) -> BitMatrix {
        BitMatrix { frames, bits, data: vec![false; frames * bits] }
    }
    pub fn get(&self, frame: usize, bit: usize) -> bool {
        self.data[frame * self.bits + bit]
    }
    pub fn set(&mut self, frame: usize, bit: usize, val: bool) {
        self.data[frame * self.bits + bit] = val;
    }
    /// Bounds-checked set, for bits that come from the database or FASM.
    pub fn try_set(&mut self, frame: usize, bit: usize, val: bool) -> Result<()> {
        if frame >= self.frames || bit >= self.bits {
            return Err(err!("F{frame}B{bit} outside {}x{}", self.frames, self.bits));
        }
        self.set(frame, bit, val);
        Ok(())
    }
    pub fn any(&self) -> bool {
        self.data.iter().any(|&b| b)
    }
    fn copy_window(&mut self, from: &BitMatrix, start_frame: usize, start_bit: usize) {
        for f in 0..from.frames {
            let dst = (f + start_frame) * self.bits + start_bit;
            self.data[dst..dst + from.bits].copy_from_slice(&from.data[f * from.bits..(f + 1) * from.bits]);
        }
    }
}

#[derive(Clone, Debug)]
pub struct Tile {
    pub name: String,
    pub tiletype: String,
    pub x: u32,
    pub y: u32,
    pub start_frame: usize,
    pub start_bit: usize,
    pub cram: BitMatrix,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Settings {
    pub background: bool,
    pub multiboot: bool,
    pub compress: bool,
}

pub struct Chip<'d> {
    pub db: &'d DeviceDb,
    pub variant: String,
    pub cram: BitMatrix,
    pub tiles: Vec<Tile>,
    pub ipconfig: BTreeMap<u32, u8>,
    pub metadata: Vec<String>,
    pub settings: Settings,
    pub tap_frame_count: usize,
    by_name: HashMap<String, usize>,
    by_loc: HashMap<(u32, u32), Vec<usize>>,
    pub tilegroups: HashMap<String, Vec<usize>>,
}

fn set_bits<'a>(
    cram: &mut BitMatrix,
    bits: impl IntoIterator<Item = &'a crate::db::ConfigBit>,
    val: impl Fn(bool) -> bool,
) -> Result<()> {
    for b in bits {
        cram.try_set(b.frame, b.bit, val(b.invert))?;
    }
    Ok(())
}

impl<'d> Chip<'d> {
    /// An unconfigured chip: every bit zero, tilegroups built.
    pub fn new(db: &'d DeviceDb, variant: &str) -> Result<Chip<'d>> {
        if !db.data.variants.contains_key(variant) {
            return Err(err!("{} has no variant {variant:?}", db.device));
        }
        let (frames, bpf) = (db.data.frames, db.data.bits_per_frame);
        let mut tiles = Vec::with_capacity(db.tiles.len());
        for (name, t) in &db.tiles {
            if t.start_frame + t.frames > frames || t.start_bit + t.bits > bpf {
                return Err(err!("tile {name} window lies outside the CRAM"));
            }
            tiles.push(Tile {
                name: name.clone(),
                tiletype: t.tiletype.clone(),
                x: t.x,
                y: t.y,
                start_frame: t.start_frame,
                start_bit: t.start_bit,
                cram: BitMatrix::new(t.frames, t.bits),
            });
        }
        let by_name = tiles.iter().enumerate().map(|(i, t)| (t.name.clone(), i)).collect();
        let mut by_loc: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
        for (i, t) in tiles.iter().enumerate() {
            by_loc.entry((t.x, t.y)).or_default().push(i);
        }
        let mut c = Chip {
            db,
            variant: variant.to_string(),
            cram: BitMatrix::new(frames, bpf),
            tiles,
            ipconfig: BTreeMap::new(),
            metadata: Vec::new(),
            settings: Settings::default(),
            tap_frame_count: if db.device == "LFCPNX-100" { 42 } else { 24 },
            by_name,
            by_loc,
            tilegroups: HashMap::new(),
        };
        c.create_tilegroups()?;
        Ok(c)
    }

    pub fn tiles_at(&self, x: u32, y: u32) -> &[usize] {
        self.by_loc.get(&(x, y)).map_or(&[], Vec::as_slice)
    }

    pub fn tile_index(&self, name: &str) -> Option<usize> {
        self.by_name.get(name).copied()
    }

    fn create_tilegroups(&mut self) -> Result<()> {
        let mut groups = HashMap::new();
        for t in 0..self.tiles.len() {
            let tile = &self.tiles[t];
            for bel in tile_bels(&tile.tiletype, self.db.tile_bits(&tile.tiletype)) {
                let name = format!("R{}C{}_{}", tile.y as i32 + bel.rel_y, tile.x as i32 + bel.rel_x, bel.name);
                groups.insert(name, bel_tiles(self, t, &bel)?);
            }
        }
        let global = (0..self.tiles.len())
            .filter(|&i| self.tiles[i].tiletype != "PLC" && self.tiles[i].tiletype != "CIB")
            .collect();
        groups.insert("GLOBAL".to_string(), global);
        self.tilegroups = groups;
        Ok(())
    }

    pub fn from_fasm(db: &'d DeviceDb, fasm: &ParsedFasm) -> Result<Chip<'d>> {
        let dev = DeviceDb::device_of(fasm)?;
        if dev != db.device {
            return Err(err!("FASM is for {dev}, database opened for {}", db.device));
        }
        let variant = fasm.attr("oxide.device_variant").unwrap_or("");
        let mut chip = Chip::new(db, variant)?;
        chip.metadata
            .extend(fasm.attrs.iter().filter(|(k, _)| k == "oxide.meta").map(|(_, v)| v.clone()));
        for t in chip.tiles.iter_mut() {
            let tb = db.tile_bits(&t.tiletype);
            set_bits(&mut t.cram, &tb.always_on, |_| true)?;
        }
        for (tn, ft) in &fasm.tiles {
            if let Some(ip) = tn.strip_prefix("IP_") {
                chip.configure_ip(ip, ft)?;
            } else if chip.tilegroups.contains_key(tn) {
                chip.apply_tilegroup(tn, ft)?;
            } else {
                let i = chip.tile_index(tn).ok_or_else(|| err!("no tile or tilegroup named {tn}"))?;
                let tb = db.tile_bits(&chip.tiles[i].tiletype);
                tile_from_fasm(&mut chip.tiles[i], tb, ft)?;
            }
        }
        chip.tiles_to_cram();
        Ok(chip)
    }

    pub fn tiles_to_cram(&mut self) {
        for t in &self.tiles {
            self.cram.copy_window(&t.cram, t.start_frame, t.start_bit);
        }
    }

    fn apply_tilegroup(&mut self, group: &str, ft: &FasmTile) -> Result<()> {
        let members = self.tilegroups[group].clone();
        let db = self.db;
        for base in [true, false] {
            for (k, v) in ft.enums.iter().filter(|(k, _)| k.starts_with("BASE_") == base && !k.starts_with("UNKNOWN.")) {
                let mut found = false;
                for &i in &members {
                    let tb = db.tile_bits(&self.tiles[i].tiletype);
                    if let Some(en) = tb.enums.get(k) {
                        let opt = en.options.get(v).ok_or_else(|| {
                            err!("no option {v} for enum {k} in tile {}; valid: {}", self.tiles[i].name, keys(&en.options))
                        })?;
                        set_bits(&mut self.tiles[i].cram, opt, |inv| !inv)?;
                        found = true;
                    }
                }
                if !found {
                    return Err(err!("no enum {k} in tilegroup {group}"));
                }
            }
        }
        for (k, v) in &ft.words {
            let mut found = false;
            for &i in &members {
                let tb = db.tile_bits(&self.tiles[i].tiletype);
                if let Some(w) = tb.words.get(k) {
                    if v.bits() > w.bits.len() {
                        return Err(err!("word {k} in tile {}: value has {} bits, word {}", self.tiles[i].name, v.bits(), w.bits.len()));
                    }
                    for (j, wb) in w.bits.iter().enumerate() {
                        let bv = v.bit(j);
                        set_bits(&mut self.tiles[i].cram, wb, |inv| inv != bv)?;
                    }
                    found = true;
                }
            }
            if !found {
                return Err(err!("no word {k} in tilegroup {group}"));
            }
        }
        Ok(())
    }

    pub fn ip_baseaddr(&self, ip: &str) -> Result<u32> {
        let region = |n: &str| self.db.baseaddrs.get(n).map(|r| r.addr).ok_or_else(|| err!("no IP named {n}"));
        if let Some(wid) = ip.strip_prefix("EBR_WID") {
            let base = region("EBR_WID0")?;
            let step = region("EBR_WID1")?.wrapping_sub(base);
            let wid: u32 = wid.parse().map_err(|_| err!("bad EBR IP name {ip}"))?;
            Ok(base.wrapping_add(wid.wrapping_mul(step)))
        } else {
            region(ip)
        }
    }

    pub fn ip_type(ip: &str) -> Result<&'static str> {
        Ok(if ip.starts_with("EBR_WID") {
            "EBR_INIT"
        } else if ip.starts_with("PLL_") {
            "PLL_CORE"
        } else if ip.starts_with("TDPHY_") {
            "DPHY_CORE"
        } else if ip.starts_with("TPCIE_") {
            "PCIE_CORE"
        } else if ip.starts_with("LRAM_") {
            "LRAM_INIT"
        } else {
            return Err(err!("no IP data for {ip}"));
        })
    }

    pub fn set_ip_bit(&mut self, offset: u32, word: u32, bit: u32, value: bool) {
        let byte = self.ipconfig.entry(offset.wrapping_add(word)).or_insert(0);
        if value {
            *byte |= 1 << bit;
        } else {
            *byte &= !(1 << bit);
        }
    }

    fn configure_ip(&mut self, ip: &str, ft: &FasmTile) -> Result<()> {
        if ip == "UNKNOWN" {
            for (k, v) in &ft.words {
                let addr = k
                    .strip_prefix("0x")
                    .and_then(|h| u32::from_str_radix(h, 16).ok())
                    .ok_or_else(|| err!("IP_UNKNOWN word {k} is not 0x<addr>"))?;
                for i in 0..8 {
                    self.set_ip_bit(0, addr, i, v.bit(i as usize));
                }
            }
            return Ok(());
        }
        let base = self.ip_baseaddr(ip)?;
        let tb: &TileBits = self.db.ip_bits(Chip::ip_type(ip)?)?;
        if ip.starts_with("PLL_") {
            self.set_ip_bit(base, 0, 0, true);
        } else if ip.starts_with("LRAM_") {
            if !ft.enums.is_empty() {
                return Err(err!("{ip}: LRAM takes INITVAL words only"));
            }
            for i in 0..81920 {
                self.set_ip_bit(base, i, 0, false);
            }
            let w = tb.words.get("INITVAL_00").ok_or_else(|| err!("LRAM_INIT has no INITVAL_00"))?;
            for (k, v) in &ft.words {
                let n = k
                    .strip_prefix("INITVAL_")
                    .and_then(|h| u32::from_str_radix(h, 16).ok())
                    .ok_or_else(|| err!("{ip}: bad LRAM word {k}"))?;
                let off = 0x280u32.wrapping_mul(n);
                for (i, wb) in w.bits.iter().enumerate() {
                    let bv = v.bit(i);
                    for b in wb {
                        self.set_ip_bit(base.wrapping_add(off), b.frame as u32, b.bit as u32, b.invert != bv);
                    }
                }
            }
            return Ok(());
        }
        for (k, v) in &ft.enums {
            let en = tb.enums.get(k).ok_or_else(|| err!("no enum {k} in IP {ip}"))?;
            let opt = en
                .options
                .get(v)
                .ok_or_else(|| err!("no option {v} for enum {k} in IP {ip}; valid: {}", keys(&en.options)))?;
            for b in opt {
                self.set_ip_bit(base, b.frame as u32, b.bit as u32, !b.invert);
            }
        }
        for (k, v) in &ft.words {
            let w = tb.words.get(k).ok_or_else(|| err!("no word {k} in IP {ip}"))?;
            if v.bits() > w.bits.len() {
                return Err(err!("word {k} in IP {ip}: value has {} bits, word {}", v.bits(), w.bits.len()));
            }
            for (i, wb) in w.bits.iter().enumerate() {
                let bv = v.bit(i);
                for b in wb {
                    self.set_ip_bit(base, b.frame as u32, b.bit as u32, b.invert != bv);
                }
            }
        }
        Ok(())
    }

    pub fn idcode(&self) -> u32 {
        // Checked in `new`.
        self.db.data.variants[&self.variant].idcode
    }

    /// Flat frame index of a frame address; None outside the device.
    pub fn frame_addr_to_idx(&self, addr: u32) -> Option<usize> {
        let a = addr as usize;
        let tap = self.tap_frame_count;
        let idx = match addr {
            0x0000..=0x7FFF => (self.cram.frames - 1).checked_sub(a)?,
            0x8000..=0x800F => (15 - (a - 0x8000)) + 16 + tap,
            0x8010..=0x801F => 15 - (a - 0x8010),
            0x8020..=0x81FF => (tap - 1).checked_sub(a - 0x8020)? + 16,
            _ => return None,
        };
        (idx < self.cram.frames).then_some(idx)
    }

    /// Bytes per bus frame of an IP region.
    pub fn bus_frame_size(addr: u32) -> Result<usize> {
        match addr >> 28 {
            0 => Ok(1),
            2 => Ok(5),
            3 => Ok(4),
            _ => Err(err!("no bus frame size for address 0x{addr:08x}")),
        }
    }
}

fn keys<V>(m: &BTreeMap<String, V>) -> String {
    m.keys().cloned().collect::<Vec<_>>().join(", ")
}

fn tile_from_fasm(t: &mut Tile, tb: &TileBits, ft: &FasmTile) -> Result<()> {
    for base in [true, false] {
        for (k, v) in ft.enums.iter().filter(|(k, _)| k.starts_with("BASE_") == base && !k.starts_with("UNKNOWN.")) {
            let en = tb.enums.get(k).ok_or_else(|| err!("no enum {k} in tile {}", t.name))?;
            let opt = en
                .options
                .get(v)
                .ok_or_else(|| err!("no option {v} for enum {k} in tile {}; valid: {}", t.name, keys(&en.options)))?;
            set_bits(&mut t.cram, opt, |inv| !inv)?;
        }
    }
    for (k, v) in &ft.words {
        let w = tb.words.get(k).ok_or_else(|| err!("no word {k} in tile {}", t.name))?;
        if v.bits() > w.bits.len() {
            return Err(err!("word {k} in tile {}: value has {} bits, word {}", t.name, v.bits(), w.bits.len()));
        }
        for (i, wb) in w.bits.iter().enumerate() {
            let bv = v.bit(i);
            set_bits(&mut t.cram, wb, |inv| inv != bv)?;
        }
    }
    for (to, from) in &ft.pips {
        match tb.pips.get(to).and_then(|ps| ps.iter().find(|p| &p.from_wire == from)) {
            Some(p) => set_bits(&mut t.cram, &p.bits, |inv| !inv)?,
            None => {
                let fixed = tb.conns.get(to).is_some_and(|cs| cs.iter().any(|c| &c.from_wire == from));
                if !fixed {
                    return Err(err!("no pip {from}.{to} in tile {}", t.name));
                }
            }
        }
    }
    for &(f, b) in &ft.unknowns {
        t.cram.try_set(f, b, true)?;
    }
    Ok(())
}
