//! Frames -> features: every segbits feature a frame set enables, and the
//! set bits no enabled feature accounts for.
//!
//! A feature is enabled when each of its bits reads as the database lists
//! it (set, or `!` clear). Features with no set bit (all `!`) are skipped:
//! they read as enabled wherever their bits are clear. Names are the tile's
//! name and the segbits name after its tile type (`INT_L_X2Y3.IMUX_L4.GFAN0`,
//! `CLBLL_L_X2Y3.SLICEL_X0.ALUT.INIT[05]`); an aliased tile's are in its
//! alias type's names, as [`crate::assemble::assemble_named`] reports them.
//!
//! Against what a FASM file asked for, this is the bitstream's own account:
//! a feature it enables that the FASM did not set is one the silicon sees
//! anyway (its bits are a subset of others'), and a set bit nothing claims
//! is configuration no feature explains.

use std::collections::{BTreeMap, BTreeSet, HashSet};

use oxibit_bitstream::FRAME_WORDS;
use oxibit_bitstream::layout::FrameAddress;
use oxibit_bitstream::writer::Frame;

use crate::db::{Database, SegBit};

/// The frame word holding the ECC (bits 12:0), which no feature uses.
const ECC_WORD: usize = 50;
const ECC_MASK: u32 = 0x1FFF;

/// A set configuration bit: frame address, word, bit.
pub type BitAt = (u32, usize, u32);

#[derive(Debug, Default)]
pub struct Decoded {
    pub features: BTreeSet<String>,
    /// Set bits no enabled feature sets, outside the ECC.
    pub unclaimed: Vec<BitAt>,
}

/// One tile's view of the frames: `column` -> frame, words from `offset`.
struct Window<'f> {
    columns: Vec<Option<&'f Frame>>,
    offset: i64,
}

impl Window<'_> {
    fn at(&self, b: &SegBit) -> Option<(usize, usize, u32)> {
        let wb = self.offset * 32 + b.bit as i64;
        let w = wb.div_euclid(32);
        let w = if w < 0 { w + FRAME_WORDS as i64 } else { w };
        if !(0..FRAME_WORDS as i64).contains(&w) {
            return None;
        }
        Some((b.column as usize, w as usize, wb.rem_euclid(32) as u32))
    }

    fn reads(&self, b: &SegBit) -> Option<bool> {
        let (c, w, bit) = self.at(b)?;
        let v = match self.columns.get(c)? {
            Some(f) => f[w] >> bit & 1 == 1,
            None => false,
        };
        Some(v)
    }
}

pub fn decode(db: &Database, frames: &BTreeMap<FrameAddress, Frame>) -> Decoded {
    let mut out = Decoded::default();
    let mut claimed: HashSet<BitAt> = HashSet::new();
    for (tile, info) in &db.tiles {
        for (block, bits, alias) in &info.bits {
            let (tt, offset) = match alias {
                Some(a) => (a.tile_type.as_str(), bits.offset - a.start_offset),
                None => (info.tile_type.as_str(), bits.offset),
            };
            let Some(seg) = db.segbits.get(&tt.to_uppercase()) else { continue };
            let Some((_, map)) = seg.by_block.iter().find(|(b, _)| b == block) else { continue };
            let columns: Vec<Option<&Frame>> = (0..bits.frames).map(|c| frames.get(&FrameAddress(bits.base + c))).collect();
            if columns.iter().all(Option::is_none) {
                continue;
            }
            let win = Window { columns, offset };
            for (feature, sb) in map {
                if !sb.iter().any(|b| b.set) {
                    continue;
                }
                if !sb.iter().all(|b| win.reads(b) == Some(b.set)) {
                    continue;
                }
                let rest = feature.split_once('.').map(|x| x.1).unwrap_or(feature);
                out.features.insert(format!("{tile}.{rest}"));
                for b in sb.iter().filter(|b| b.set) {
                    if let Some((c, w, bit)) = win.at(b) {
                        claimed.insert((bits.base + c as u32, w, bit));
                    }
                }
            }
        }
    }
    for (a, f) in frames {
        for (w, &word) in f.iter().enumerate() {
            let word = if w == ECC_WORD { word & !ECC_MASK } else { word };
            for bit in 0..32 {
                if word >> bit & 1 == 1 && !claimed.contains(&(a.0, w, bit)) {
                    out.unclaimed.push((a.0, w, bit));
                }
            }
        }
    }
    out
}

/// The tile whose bits a set bit most likely is: the tile covering the
/// frame whose word offset is the largest at or below the word.
pub fn tile_of(db: &Database, (frame, word, _): BitAt) -> Option<String> {
    db.tiles
        .iter()
        .flat_map(|(t, i)| i.bits.iter().map(move |(_, b, _)| (t, b)))
        .filter(|(_, b)| (b.base..b.base + b.frames).contains(&frame) && b.offset <= word as i64)
        .max_by_key(|(t, b)| (b.offset, std::cmp::Reverse((*t).clone())))
        .map(|(t, _)| t.clone())
}
