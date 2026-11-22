//! A configured device: the whole-chip CRAM, with each tile a window onto
//! it, plus the EBR contents and the header metadata.
//!
//! `Chip::from_config` applies a `ChipConfig` in libtrellis's order
//! (`ChipConfig::to_chip`), which matters because settings can share bits
//! (an enum option sets bits another cleared, two tiles' windows overlap):
//! 1. every tile of the device, in name order, with its `.tile` settings or
//!    none (an empty configuration still writes defaults). Within a tile:
//!    arcs, `BASE_` enums, words, other enums, unknown bits, then the
//!    default of every word and enum the tile's settings did not name;
//! 2. tile groups in file order: the same phases on every member tile that
//!    has the setting, with no defaults;
//! 3. a `.tile` naming a tile the device does not have is an error, after
//!    everything else was applied (as in libtrellis).

use std::collections::{BTreeMap, BTreeSet};

use crate::config::{ChipConfig, TileConfig};
use crate::db::{BitGroup, ConfigBit, DeviceDb, DeviceInfo, Enum, TileBits, TileInfo, Word};
use crate::{Result, err};

/// ECP5 EBR initialisation size, in 9-bit words per block.
pub const ECP5_BRAM_WORDS: usize = 2048;

/// The value a configuration bit is written with.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Level {
    Low,
    High,
}

impl Level {
    /// The level that makes `b` read as set: low for an inverted bit.
    fn setting(b: ConfigBit) -> Level {
        if b.inv { Level::Low } else { Level::High }
    }

    /// The level that makes `b` read as clear.
    fn clearing(b: ConfigBit) -> Level {
        if b.inv { Level::High } else { Level::Low }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BitMatrix {
    pub frames: usize,
    pub bits: usize,
    data: Vec<bool>,
}

impl BitMatrix {
    pub fn new(frames: usize, bits: usize) -> BitMatrix {
        BitMatrix {
            frames,
            bits,
            data: vec![false; frames * bits],
        }
    }

    pub fn get(&self, frame: usize, bit: usize) -> bool {
        self.data[frame * self.bits + bit]
    }

    /// Bounds-checked write.
    pub fn put(&mut self, frame: usize, bit: usize, level: Level) -> Result<()> {
        if frame >= self.frames || bit >= self.bits {
            return Err(err!("F{frame}B{bit} outside {}x{}", self.frames, self.bits));
        }
        self.data[frame * self.bits + bit] = level == Level::High;
        Ok(())
    }

    pub fn any(&self) -> bool {
        self.data.iter().any(|&b| b)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Chip {
    pub info: DeviceInfo,
    pub cram: BitMatrix,
    /// Header strings (`.comment` lines).
    pub metadata: Vec<String>,
    pub usercode: u32,
    /// Control register 0, as `LSC_PROG_CNTRL0` writes it.
    pub ctrl0: u32,
    /// EBR initialisation by block index.
    pub bram_data: BTreeMap<u16, Vec<u16>>,
    pub bram_data_size: usize,
}

/// One tile's window onto the chip CRAM.
struct TileView<'a> {
    cram: &'a mut BitMatrix,
    tile: &'a TileInfo,
}

impl TileView<'_> {
    fn put(&mut self, b: ConfigBit, level: Level) -> Result<()> {
        if b.frame >= self.tile.frames || b.bit >= self.tile.bits {
            return Err(err!(
                "{}: F{}B{} outside its {}x{} window",
                self.tile.name,
                b.frame,
                b.bit,
                self.tile.frames,
                self.tile.bits
            ));
        }
        self.cram.put(
            self.tile.start_frame + b.frame,
            self.tile.start_bit + b.bit,
            level,
        )
    }

    fn set_group(&mut self, g: &BitGroup) -> Result<()> {
        g.0.iter().try_for_each(|&b| self.put(b, Level::setting(b)))
    }

    fn clear_group(&mut self, g: &BitGroup) -> Result<()> {
        g.0.iter()
            .try_for_each(|&b| self.put(b, Level::clearing(b)))
    }

    fn set_word(&mut self, name: &str, w: &Word, value: &[bool]) -> Result<()> {
        if value.len() != w.bits.len() {
            return Err(err!(
                "{}: word {name} has {} bits, value has {}",
                self.tile.name,
                w.bits.len(),
                value.len()
            ));
        }
        for (g, &v) in w.bits.iter().zip(value) {
            if v {
                self.set_group(g)?;
            } else {
                self.clear_group(g)?;
            }
        }
        Ok(())
    }

    /// `_NONE_` sets nothing; an option sets its bits and clears none.
    fn set_enum(&mut self, name: &str, e: &Enum, value: &str) -> Result<()> {
        if value == "_NONE_" {
            return Ok(());
        }
        let g = e
            .options
            .get(value)
            .ok_or_else(|| err!("{}: enum {name} has no option {value}", self.tile.name))?;
        self.set_group(g)
    }
}

/// Whether settings apply to one tile (with defaults) or to a member of a
/// tile group (settings the tile lacks are skipped, names it has are
/// recorded, no defaults).
enum Scope<'m> {
    Tile,
    Group(&'m mut BTreeSet<String>),
}

const BASE_PREFIX: &str = "BASE_";

fn apply(
    view: &mut TileView<'_>,
    db: &TileBits,
    cfg: &TileConfig,
    mut scope: Scope<'_>,
) -> Result<()> {
    let tile_info: &TileInfo = view.tile;
    let tile = tile_info.name.as_str();
    for (sink, source) in &cfg.arcs {
        let mux = db
            .muxes
            .get(sink)
            .ok_or_else(|| err!("{tile}: no mux for sink {sink}"))?;
        let arc = mux
            .arcs
            .get(source)
            .ok_or_else(|| err!("{tile}: sink {sink} has no driver named {source}"))?;
        view.set_group(arc)?;
    }
    let mut found_words = BTreeSet::new();
    let mut found_enums = BTreeSet::new();
    let enums = |view: &mut TileView<'_>,
                 scope: &mut Scope<'_>,
                 base: bool,
                 found: &mut BTreeSet<String>| {
        for (name, value) in cfg
            .enums
            .iter()
            .filter(|(n, _)| n.starts_with(BASE_PREFIX) == base)
        {
            let e = match (db.enums.get(name), &*scope) {
                (Some(e), _) => e,
                (None, Scope::Group(_)) => continue,
                (None, Scope::Tile) => return Err(err!("{tile}: no enum named '{name}'")),
            };
            if let Scope::Group(matched) = scope {
                if !e.options.contains_key(value) {
                    continue;
                }
                matched.insert(name.clone());
            }
            view.set_enum(name, e, value)?;
            found.insert(name.clone());
        }
        Ok(())
    };
    enums(view, &mut scope, true, &mut found_enums)?;
    for (name, value) in &cfg.words {
        let w = match (db.words.get(name), &mut scope) {
            (Some(w), Scope::Group(matched)) => {
                matched.insert(name.clone());
                w
            }
            (Some(w), Scope::Tile) => w,
            (None, Scope::Group(_)) => continue,
            (None, Scope::Tile) => return Err(err!("{tile}: no word named '{name}'")),
        };
        view.set_word(name, w, value)?;
        found_words.insert(name.clone());
    }
    enums(view, &mut scope, false, &mut found_enums)?;
    for &(frame, bit) in &cfg.unknowns {
        view.put(
            ConfigBit {
                frame,
                bit,
                inv: false,
            },
            Level::High,
        )?;
    }
    if let Scope::Tile = scope {
        for (name, w) in &db.words {
            if !found_words.contains(name) {
                view.set_word(name, w, &w.defval)?;
            }
        }
        for (name, e) in &db.enums {
            if let (false, Some(d)) = (found_enums.contains(name), &e.defval) {
                view.set_enum(name, e, d)?;
            }
        }
    }
    Ok(())
}

impl Chip {
    pub fn from_config(db: &DeviceDb, cc: &ChipConfig) -> Result<Chip> {
        if cc.device != db.info.name {
            return Err(err!(
                "configuration is for {}, database is {}",
                cc.device,
                db.info.name
            ));
        }
        if db.info.family != "ECP5" {
            return Err(err!(
                "{}: family {} is not ECP5",
                db.info.name,
                db.info.family
            ));
        }
        let mut cram = BitMatrix::new(db.info.frames, db.info.bits_per_frame);
        let empty = TileConfig::default();
        for (name, tile) in &db.tiles {
            let bits = db.tile_bits(&tile.ty)?;
            let cfg = cc.tiles.get(name).unwrap_or(&empty);
            apply(
                &mut TileView {
                    cram: &mut cram,
                    tile,
                },
                bits,
                cfg,
                Scope::Tile,
            )?;
        }
        for tg in &cc.tilegroups {
            let mut matched = BTreeSet::new();
            for name in &tg.tiles {
                let tile = db
                    .tiles
                    .get(name)
                    .ok_or_else(|| err!("tile group: no tile {name:?}"))?;
                let bits = db.tile_bits(&tile.ty)?;
                apply(
                    &mut TileView {
                        cram: &mut cram,
                        tile,
                    },
                    bits,
                    &tg.config,
                    Scope::Group(&mut matched),
                )?;
            }
            for (name, _) in &tg.config.words {
                if !matched.contains(name) {
                    return Err(err!("config word {name} matched in no tilegroup tiles"));
                }
            }
            for (name, _) in &tg.config.enums {
                if !matched.contains(name) {
                    return Err(err!("config enum {name} matched in no tilegroup tiles"));
                }
            }
        }
        if let Some(name) = cc.tiles.keys().find(|n| !db.tiles.contains_key(*n)) {
            return Err(err!("tile {name} does not exist in chip {}", db.info.name));
        }
        Ok(Chip {
            info: db.info.clone(),
            cram,
            metadata: cc.metadata.clone(),
            usercode: 0,
            ctrl0: 0x4000_0000,
            bram_data: cc.bram_data.clone(),
            bram_data_size: ECP5_BRAM_WORDS,
        })
    }
}
