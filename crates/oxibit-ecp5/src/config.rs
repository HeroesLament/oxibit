//! Trellis text configurations (`.config`), as `ChipConfig::from_string`
//! reads them.
//!
//! ```text
//! .device LFE5U-85F
//! .comment Part: LFE5U-85F-6CABGA381
//! .sysconfig MCCLK_FREQ 62
//! .tile R10C10:PLC2
//! arc: A0 H02E0501
//! word: SLICEA.K0.INIT 1010101010101010
//! enum: SLICEA.MODE LOGIC
//! unknown: F3B12
//!
//! .tile_group R1C1:T1 R1C2:T2
//! enum: X Y
//!
//! .bram_init 0
//! 000 001 ...
//! ```

use std::collections::BTreeMap;

use crate::db::{ConfigBit, Tokens, parse_bitvec};
use crate::{Result, err};

/// The settings of one tile, in file order (libtrellis applies them in
/// phases, not in this order; see `Chip::from_config`).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct TileConfig {
    /// `(sink, source)`.
    pub arcs: Vec<(String, String)>,
    pub words: Vec<(String, Vec<bool>)>,
    pub enums: Vec<(String, String)>,
    /// `(frame, bit)` set to 1 with no database meaning.
    pub unknowns: Vec<(usize, usize)>,
}

/// Settings applied to every tile of a group that has them.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct TileGroup {
    pub tiles: Vec<String>,
    pub config: TileConfig,
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ChipConfig {
    pub device: String,
    pub variant: String,
    /// `.comment` lines, which become the bitstream header's metadata.
    pub metadata: Vec<String>,
    /// By tile name; a repeated `.tile` replaces the earlier one.
    pub tiles: BTreeMap<String, TileConfig>,
    pub tilegroups: Vec<TileGroup>,
    pub sysconfig: BTreeMap<String, String>,
    /// EBR initialisation by block index: 9-bit words.
    pub bram_data: BTreeMap<u16, Vec<u16>>,
}

fn tile_config(t: &mut Tokens<'_>) -> Result<TileConfig> {
    let mut tc = TileConfig::default();
    while !t.eor() {
        match t.word()? {
            "arc:" => {
                let sink = t.word()?.to_owned();
                let source = t.word()?.to_owned();
                tc.arcs.push((sink, source));
            }
            "word:" => {
                let name = t.word()?.to_owned();
                tc.words.push((name, parse_bitvec(t.word()?)?));
            }
            "enum:" => {
                let name = t.word()?.to_owned();
                tc.enums.push((name, t.word()?.to_owned()));
            }
            "unknown:" => {
                let s = t.word()?;
                let b = ConfigBit::parse(s)?;
                if b.inv {
                    return Err(err!("unknown bit {s:?} is inverted"));
                }
                tc.unknowns.push((b.frame, b.bit));
            }
            other => return Err(err!("unexpected token {other:?} while reading config text")),
        }
    }
    Ok(tc)
}

impl ChipConfig {
    pub fn parse(text: &str) -> Result<ChipConfig> {
        let mut t = Tokens::new(text);
        let mut cc = ChipConfig::default();
        while !t.eof() {
            match t.word()? {
                ".device" => t.word()?.clone_into(&mut cc.device),
                ".variant" => t.word()?.clone_into(&mut cc.variant),
                ".comment" => {
                    t.skip_char();
                    cc.metadata.push(t.rest_of_line()?.to_owned());
                }
                ".tile" => {
                    let name = t.word()?.to_owned();
                    let tc = tile_config(&mut t)?;
                    cc.tiles.insert(name, tc);
                }
                ".sysconfig" => {
                    let key = t.word()?.to_owned();
                    let value = t.word()?.to_owned();
                    cc.sysconfig.insert(key, value);
                }
                ".bram_init" => {
                    let s = t.word()?;
                    let bram: u16 = s.parse().map_err(|e| err!(".bram_init {s:?}: {e}"))?;
                    let data = cc.bram_data.entry(bram).or_default();
                    while !t.eor() {
                        let s = t.word()?;
                        data.push(
                            u16::from_str_radix(s, 16)
                                .map_err(|e| err!(".bram_init {bram}: {s:?}: {e}"))?,
                        );
                    }
                }
                ".tile_group" => {
                    let line = t.rest_of_line()?;
                    let mut tiles: Vec<String> =
                        line.split_ascii_whitespace().map(str::to_owned).collect();
                    // libtrellis reads the names with `while (ss2) { ss2 >> tile;
                    // push(tile); }`: the extraction that fails at end of line
                    // leaves `tile` as it was, so the last name is pushed twice
                    // (and an empty line yields one empty name).
                    tiles.push(tiles.last().cloned().unwrap_or_default());
                    let config = tile_config(&mut t)?;
                    cc.tilegroups.push(TileGroup { tiles, config });
                }
                other => return Err(err!("unrecognised config entry {other}")),
            }
        }
        Ok(cc)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_entries() {
        let cc = ChipConfig::parse(
            ".device LFE5U-85F\n\n.comment Part: LFE5U-85F-6CABGA381\n.tile R1C1:PLC2\narc: A0 B1\n\
             word: W 0011\nenum: E ON\nunknown: F1B2\n\n.sysconfig MCCLK_FREQ 62\n.bram_init 3\n1ff 000\n",
        )
        .expect("parses");
        assert_eq!(cc.device, "LFE5U-85F");
        assert_eq!(cc.metadata, vec!["Part: LFE5U-85F-6CABGA381".to_owned()]);
        let tc = &cc.tiles["R1C1:PLC2"];
        assert_eq!(tc.arcs, vec![("A0".to_owned(), "B1".to_owned())]);
        assert_eq!(tc.words[0].1, vec![true, true, false, false]);
        assert_eq!(tc.unknowns, vec![(1, 2)]);
        assert_eq!(cc.sysconfig["MCCLK_FREQ"], "62");
        assert_eq!(cc.bram_data[&3], vec![0x1ff, 0]);
    }
}
