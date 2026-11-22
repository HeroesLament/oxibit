//! Reading ECP5 bitstreams back: `.bit` -> `Chip` -> `ChipConfig`, as
//! libtrellis's `Bitstream::read_bit`, `deserialise_chip`,
//! `ChipConfig::from_chip` and `TileBitDatabase::tile_cram_to_config` do it,
//! so that `ChipConfig::to_text` of the result is `ecpunpack`'s output.
//!
//! Reading a tile back is a best-match search, not an inverse of applying:
//! - a mux sink's driver is the arc whose bits all match, the one with the
//!   most bits (ties to the later source name); an arc with no bits is not
//!   reported;
//! - a word is reported when its value differs from its default;
//! - an enum's value is the best-matching option (same rule); none matching
//!   reads `_NONE_` if the enum has a default, nothing otherwise; a match
//!   whose bits equal the default option's is not reported;
//! - every set bit no reported setting accounts for is `unknown:`.

use std::collections::BTreeSet;
use std::fmt::Write as _;
use std::path::Path;

use crate::chip::{BitMatrix, Chip, ECP5_BRAM_WORDS};
use crate::config::{ChipConfig, TileConfig};
use crate::db::{BitGroup, ConfigBit, DeviceDb, TileBits, TileInfo};
use crate::{Result, err};

const PREAMBLE: [u8; 4] = [0xFF, 0xFF, 0xBD, 0xB3];
const CRC16_POLY: u16 = 0x8005;

/// A `.bit` file split into its header strings and the whole file (the
/// stream is found by searching for the preamble, header included).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BitFile<'a> {
    pub metadata: Vec<String>,
    pub bytes: &'a [u8],
}

impl<'a> BitFile<'a> {
    pub fn parse(bytes: &'a [u8]) -> Result<BitFile<'a>> {
        let mut i = 0;
        if bytes.starts_with(b"LSCC") {
            i = 4;
        }
        if bytes.get(i..i + 2) != Some(&[0xFF, 0x00]) {
            return Err(err!(
                "Lattice .BIT files must start with LSCC or 0xFF, 0x00"
            ));
        }
        i += 2;
        let mut metadata = Vec::new();
        let mut cur = Vec::new();
        loop {
            let c = *bytes
                .get(i)
                .ok_or_else(|| err!("end of file before start of bitstream data"))?;
            i += 1;
            match c {
                0xFF => break,
                0 => metadata.push(String::from_utf8_lossy(&std::mem::take(&mut cur)).into_owned()),
                c => cur.push(c),
            }
        }
        Ok(BitFile { metadata, bytes })
    }

    /// The IDCODE of the first `VERIFY_ID`, to find the device by.
    pub fn idcode(&self) -> Result<u32> {
        let mut r = Reader::after_preamble(self.bytes)?;
        while !r.end() {
            let op = r.opcode()?;
            match op {
                0xFF => {}
                // LSC_RESET_CRC, LSC_INIT_ADDRESS, SPI_MODE
                0x3B | 0x46 | 0x79 => r.skip(3)?,
                0xE2 => {
                    r.skip(3)?;
                    return r.u32();
                }
                _ => return Err(err!("command 0x{op:02x} before VERIFY_ID")),
            }
        }
        Err(err!("no VERIFY_ID in bitstream"))
    }
}

struct Reader<'a> {
    s: &'a [u8],
    pos: usize,
    crc: u16,
}

impl<'a> Reader<'a> {
    fn after_preamble(s: &'a [u8]) -> Result<Reader<'a>> {
        let at = s
            .windows(PREAMBLE.len())
            .position(|w| w == PREAMBLE)
            .ok_or_else(|| err!("preamble not found in bitstream"))?;
        Ok(Reader {
            s,
            pos: at + PREAMBLE.len(),
            crc: 0,
        })
    }

    fn end(&self) -> bool {
        self.pos >= self.s.len()
    }

    fn update_crc(&mut self, val: u8) {
        for i in (0..8).rev() {
            let flag = self.crc >> 15 != 0;
            self.crc = (self.crc << 1) | u16::from((val >> i) & 1);
            if flag {
                self.crc ^= CRC16_POLY;
            }
        }
    }

    fn byte(&mut self) -> Result<u8> {
        let b = *self
            .s
            .get(self.pos)
            .ok_or_else(|| err!("unexpected end of bitstream at {}", self.pos))?;
        self.pos += 1;
        self.update_crc(b);
        Ok(b)
    }

    /// Command opcode: a dummy (`0xFF`) does not feed the CRC.
    fn opcode(&mut self) -> Result<u8> {
        let b = *self
            .s
            .get(self.pos)
            .ok_or_else(|| err!("unexpected end of bitstream at {}", self.pos))?;
        self.pos += 1;
        if b != 0xFF {
            self.update_crc(b);
        }
        Ok(b)
    }

    fn skip(&mut self, n: usize) -> Result<()> {
        for _ in 0..n {
            self.byte()?;
        }
        Ok(())
    }

    fn u32(&mut self) -> Result<u32> {
        Ok(u32::from_be_bytes([
            self.byte()?,
            self.byte()?,
            self.byte()?,
            self.byte()?,
        ]))
    }

    fn check_crc(&mut self) -> Result<()> {
        for _ in 0..16 {
            let flag = self.crc >> 15 != 0;
            self.crc <<= 1;
            if flag {
                self.crc ^= CRC16_POLY;
            }
        }
        let actual = self.crc;
        let at = self.pos;
        let expected = u16::from_be_bytes([self.byte()?, self.byte()?]);
        if actual != expected {
            return Err(err!(
                "crc fail at {at}, calculated 0x{actual:04x} but expecting 0x{expected:04x}"
            ));
        }
        self.crc = 0;
        Ok(())
    }

    /// Make at least `need` unread bits available in `bits`.
    fn fill(&mut self, bits: &mut BitBuf, need: u32) -> Result<()> {
        if bits.have < need {
            bits.data = (bits.data << 8) | u32::from(self.byte()?);
            bits.have += 8;
        }
        Ok(())
    }

    /// One compressed frame (libtrellis `get_compressed_bytes`): `0` a zero
    /// byte, `11` + 8 bits a literal, `10` + 4 bits a dictionary index
    /// (0-7 one-hot, 8-15 stored). Bits left at the end are padding.
    fn compressed(&mut self, out: &mut [u8], dict: &[u8; 16]) -> Result<()> {
        let mut bits = BitBuf::default();
        for o in out.iter_mut() {
            self.fill(&mut bits, 1)?;
            *o = if bits.take(1) == 0 {
                0
            } else {
                self.fill(&mut bits, 5)?;
                if bits.take(1) == 1 {
                    self.fill(&mut bits, 8)?;
                    u8::try_from(bits.take(8)).expect("8 bits fit a byte")
                } else {
                    dict[usize::try_from(bits.take(4)).expect("4 bits fit usize")]
                }
            };
        }
        Ok(())
    }
}

/// Unread bits, MSB first: the low `have` bits of `data`.
#[derive(Debug, Default)]
struct BitBuf {
    data: u32,
    have: u32,
}

impl BitBuf {
    fn take(&mut self, n: u32) -> u32 {
        self.have -= n;
        let v = (self.data >> self.have) & ((1 << n) - 1);
        self.data &= (1 << self.have) - 1;
        v
    }
}

/// Parse a bitstream for `db`'s device. Commands as libtrellis reads them
/// for ECP5; `MachXO`-only commands are refused.
#[expect(
    clippy::too_many_lines,
    reason = "one command per arm, as libtrellis's deserialise_chip"
)]
pub fn deserialise(db: &DeviceDb, file: &BitFile<'_>) -> Result<Chip> {
    let info = &db.info;
    let mut r = Reader::after_preamble(file.bytes)?;
    let mut idcode = None;
    let mut ctrl0 = 0x4000_0000;
    let mut usercode = 0;
    let mut cram = BitMatrix::new(info.frames, info.bits_per_frame);
    let mut bram: std::collections::BTreeMap<u16, Vec<u16>> = std::collections::BTreeMap::new();
    let mut dict: Option<[u8; 16]> = None;
    let mut current_ebr: u16 = 0;
    let mut addr_in_ebr = 0usize;
    let bpf = (info.bits_per_frame + info.pad_bits_after_frame + info.pad_bits_before_frame) / 8;
    while !r.end() {
        let op = r.opcode()?;
        match op {
            0xFF => {}
            // LSC_RESET_CRC
            0x3B => {
                r.skip(3)?;
                r.crc = 0;
            }
            // VERIFY_ID
            0xE2 => {
                r.skip(3)?;
                let id = r.u32()?;
                if id != info.idcode {
                    return Err(err!(
                        "IDCODE 0x{id:08x} is not {}'s 0x{:08x}",
                        info.name,
                        info.idcode
                    ));
                }
                idcode = Some(id);
            }
            // LSC_PROG_CNTRL0
            0x22 => {
                r.skip(3)?;
                ctrl0 = r.u32()?;
            }
            // ISC_PROGRAM_DONE
            0x5E => {
                let check = r.byte()? & 0x80 != 0;
                r.skip(2)?;
                if check {
                    r.check_crc()?;
                }
            }
            // ISC_PROGRAM_SECURITY, LSC_INIT_ADDRESS, SPI_MODE (mode byte, 2 zeros)
            0xCE | 0x46 | 0x79 => r.skip(3)?,
            // ISC_PROGRAM_USERCODE
            0xC2 => {
                let check = r.byte()? & 0x80 != 0;
                r.skip(2)?;
                usercode = r.u32()?;
                if check {
                    r.check_crc()?;
                }
                r.crc = 0;
            }
            // LSC_WRITE_COMP_DIC (ECP5: no one-hot dictionary in the stream)
            0x02 => {
                let check = r.byte()? & 0x80 != 0;
                r.skip(2)?;
                let mut d = [0u8; 16];
                for i in (0..8).rev() {
                    d[i] = 1 << i;
                    d[8 + i] = r.byte()?;
                }
                dict = Some(d);
                if check {
                    r.check_crc()?;
                }
            }
            // LSC_PROG_INCR_CMP, LSC_PROG_INCR_RTI
            0xB8 | 0x82 => {
                if idcode.is_none() {
                    return Err(err!("start of bitstream data before chip was identified"));
                }
                let compressed = op == 0xB8;
                let d = match (compressed, dict) {
                    (true, None) => {
                        return Err(err!("compressed data before the compression dictionary"));
                    }
                    (_, d) => d.unwrap_or([0; 16]),
                };
                let p0 = r.byte()?;
                let count = usize::from(u16::from_be_bytes([r.byte()?, r.byte()?]));
                let check = p0 & 0x80 != 0;
                let per_frame = check && p0 & 0x40 == 0;
                let dummy = usize::from(p0 & 0x0F);
                let n = if compressed {
                    bpf + (7 - (bpf - 1) % 8)
                } else {
                    bpf
                };
                let mut buf = vec![0u8; n];
                for i in 0..count {
                    let idx = info
                        .frames
                        .checked_sub(1 + i)
                        .ok_or_else(|| err!("{count} frames, device has {}", info.frames))?;
                    if compressed {
                        r.compressed(&mut buf, &d)?;
                    } else {
                        for b in &mut buf {
                            *b = r.byte()?;
                        }
                    }
                    for j in 0..info.bits_per_frame {
                        let ofs = j + info.pad_bits_after_frame;
                        let v = buf[n - 1 - ofs / 8] >> (ofs % 8) & 1 == 1;
                        cram.put(
                            idx,
                            j,
                            if v {
                                crate::chip::Level::High
                            } else {
                                crate::chip::Level::Low
                            },
                        )?;
                    }
                    if per_frame || (check && i + 1 == count) {
                        r.check_crc()?;
                    }
                    r.skip(dummy)?;
                }
            }
            // LSC_EBR_ADDRESS
            0xF6 => {
                r.skip(3)?;
                let a = r.u32()?;
                current_ebr = u16::try_from((a >> 11) & 0x3FF).expect("masked to 10 bits");
                addr_in_ebr = usize::try_from(a & 0x7FF).expect("masked to 11 bits");
                bram.entry(current_ebr)
                    .or_default()
                    .resize(ECP5_BRAM_WORDS, 0);
            }
            // LSC_EBR_WRITE
            0xB2 => {
                let p0 = r.byte()?;
                let count = u16::from_be_bytes([r.byte()?, r.byte()?]);
                for _ in 0..count {
                    if addr_in_ebr >= ECP5_BRAM_WORDS {
                        addr_in_ebr = 0;
                        current_ebr += 1;
                        bram.entry(current_ebr)
                            .or_default()
                            .resize(ECP5_BRAM_WORDS, 0);
                    }
                    let mut f = [0u16; 9];
                    for b in &mut f {
                        *b = u16::from(r.byte()?);
                    }
                    let words = [
                        (f[0] << 1) | (f[1] >> 7),
                        (f[1] & 0x7F) << 2 | (f[2] >> 6),
                        (f[2] & 0x3F) << 3 | (f[3] >> 5),
                        (f[3] & 0x1F) << 4 | (f[4] >> 4),
                        (f[4] & 0x0F) << 5 | (f[5] >> 3),
                        (f[5] & 0x07) << 6 | (f[6] >> 2),
                        (f[6] & 0x03) << 7 | (f[7] >> 1),
                        (f[7] & 0x01) << 8 | f[8],
                    ];
                    let e = bram.entry(current_ebr).or_default();
                    e.get_mut(addr_in_ebr..addr_in_ebr + 8)
                        .ok_or_else(|| err!("EBR {current_ebr} write past its end"))?
                        .copy_from_slice(&words);
                    addr_in_ebr += 8;
                }
                if p0 & 0x80 != 0 {
                    r.check_crc()?;
                }
            }
            // JUMP (address not parsed), LSC_PROG_SED_CRC (value not kept)
            0x7E | 0xA2 => r.skip(7)?,
            other => {
                return Err(err!(
                    "unknown bitstream command 0x{other:02x} at {}",
                    r.pos - 1
                ));
            }
        }
    }
    if idcode.is_none() {
        return Err(err!("failed to parse bitstream, no valid payload found"));
    }
    Ok(Chip {
        info: info.clone(),
        cram,
        metadata: file.metadata.clone(),
        usercode,
        ctrl0,
        bram_data: bram,
        bram_data_size: ECP5_BRAM_WORDS,
    })
}

/// Bits read through one tile's window.
struct TileRead<'a> {
    cram: &'a BitMatrix,
    tile: &'a TileInfo,
}

impl TileRead<'_> {
    fn get(&self, b: ConfigBit) -> bool {
        self.cram
            .get(self.tile.start_frame + b.frame, self.tile.start_bit + b.bit)
    }

    fn matches(&self, g: &BitGroup) -> bool {
        g.0.iter().all(|&b| self.get(b) != b.inv)
    }
}

/// libtrellis `add_coverage(.., true)`: the bits of a matched group that
/// read 1 (its non-inverted ones).
fn cover_matched(cov: &mut BTreeSet<(usize, usize)>, g: &BitGroup) {
    cov.extend(g.0.iter().filter(|b| !b.inv).map(|b| (b.frame, b.bit)));
}

/// libtrellis `add_coverage(.., false)`: the bits of an unmatched word bit
/// that read 1 when it is clear (its inverted ones).
fn cover_unmatched(cov: &mut BTreeSet<(usize, usize)>, g: &BitGroup) {
    cov.extend(g.0.iter().filter(|b| b.inv).map(|b| (b.frame, b.bit)));
}

/// The best match: all bits set, most bits, ties to the later key.
fn best<'a>(
    groups: impl Iterator<Item = (&'a String, &'a BitGroup)>,
    t: &TileRead<'_>,
) -> Option<(&'a String, &'a BitGroup)> {
    let mut found = None;
    let mut most = 0;
    for (k, g) in groups {
        if t.matches(g) && g.0.len() >= most {
            most = g.0.len();
            found = Some((k, g));
        }
    }
    found
}

fn tile_to_config(t: &TileRead<'_>, db: &TileBits) -> Result<TileConfig> {
    let mut cfg = TileConfig::default();
    let mut cov = BTreeSet::new();
    for (sink, mux) in &db.muxes {
        if let Some((source, g)) = best(mux.arcs.iter(), t) {
            cover_matched(&mut cov, g);
            if !g.0.is_empty() {
                cfg.arcs.push((sink.clone(), source.clone()));
            }
        }
    }
    for (name, w) in &db.words {
        let val: Vec<bool> = w
            .bits
            .iter()
            .map(|g| {
                let m = t.matches(g);
                if m {
                    cover_matched(&mut cov, g);
                } else {
                    cover_unmatched(&mut cov, g);
                }
                m
            })
            .collect();
        if val != w.defval {
            cfg.words.push((name.clone(), val));
        }
    }
    for (name, e) in &db.enums {
        match best(e.options.iter(), t) {
            None => {
                if e.defval.is_some() {
                    cfg.enums.push((name.clone(), "_NONE_".to_owned()));
                }
            }
            Some((opt, g)) => {
                cover_matched(&mut cov, g);
                let is_default = match &e.defval {
                    Some(d) => {
                        let dg = e.options.get(d).ok_or_else(|| {
                            err!("{}: enum {name} default {d} is not an option", t.tile.name)
                        })?;
                        dg == g
                    }
                    None => false,
                };
                if !is_default {
                    cfg.enums.push((name.clone(), opt.clone()));
                }
            }
        }
    }
    for f in 0..t.tile.frames {
        for b in 0..t.tile.bits {
            if t.get(ConfigBit {
                frame: f,
                bit: b,
                inv: false,
            }) && !cov.contains(&(f, b))
            {
                cfg.unknowns.push((f, b));
            }
        }
    }
    Ok(cfg)
}

impl ChipConfig {
    /// libtrellis `ChipConfig::from_chip`.
    pub fn from_chip(db: &DeviceDb, chip: &Chip) -> Result<ChipConfig> {
        let mut cc = ChipConfig {
            device: chip.info.name.clone(),
            metadata: chip.metadata.clone(),
            bram_data: chip.bram_data.clone(),
            ..ChipConfig::default()
        };
        for (name, tile) in &db.tiles {
            let t = TileRead {
                cram: &chip.cram,
                tile,
            };
            cc.tiles
                .insert(name.clone(), tile_to_config(&t, db.tile_bits(&tile.ty)?)?);
        }
        Ok(cc)
    }

    /// libtrellis `ChipConfig::to_string`: empty tiles omitted, EBR words
    /// eight to a line.
    pub fn to_text(&self) -> String {
        let bits = |v: &[bool]| {
            v.iter()
                .rev()
                .map(|&b| if b { '1' } else { '0' })
                .collect::<String>()
        };
        let mut s = String::new();
        let _ = writeln!(s, ".device {}\n", self.device);
        if !self.variant.is_empty() {
            let _ = writeln!(s, ".variant {}\n", self.variant);
        }
        for m in &self.metadata {
            let _ = writeln!(s, ".comment {m}");
        }
        for (k, v) in &self.sysconfig {
            let _ = writeln!(s, ".sysconfig {k} {v}");
        }
        s.push('\n');
        for (name, t) in &self.tiles {
            if t.arcs.is_empty()
                && t.words.is_empty()
                && t.enums.is_empty()
                && t.unknowns.is_empty()
            {
                continue;
            }
            let _ = writeln!(s, ".tile {name}");
            for (sink, source) in &t.arcs {
                let _ = writeln!(s, "arc: {sink} {source}");
            }
            for (n, v) in &t.words {
                let _ = writeln!(s, "word: {n} {}", bits(v));
            }
            for (n, v) in &t.enums {
                let _ = writeln!(s, "enum: {n} {v}");
            }
            for (f, b) in &t.unknowns {
                let _ = writeln!(s, "unknown: F{f}B{b}");
            }
            s.push('\n');
        }
        for (n, data) in &self.bram_data {
            let _ = writeln!(s, ".bram_init {n}");
            for (i, w) in data.iter().enumerate() {
                let _ = write!(s, "{w:03x}{}", if i % 8 == 7 { '\n' } else { ' ' });
            }
            s.push('\n');
        }
        s
    }
}

/// Read a `.bit` file into a configuration, finding the device by IDCODE.
pub fn unpack(root: &Path, bytes: &[u8]) -> Result<ChipConfig> {
    let file = BitFile::parse(bytes)?;
    let id = file.idcode()?;
    let device = crate::db::device_for_idcode(root, id)?;
    let db = DeviceDb::load(root, &device)?;
    let chip = deserialise(&db, &file)?;
    ChipConfig::from_chip(&db, &chip)
}
