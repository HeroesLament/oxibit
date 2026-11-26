//! The binary format: a magic, then length-prefixed little-endian sections.

use crate::model::{Chip, Device, Groups, Strings, Tile, TileType};

const MAGIC: &[u8; 8] = b"OXARCH03";

fn len32(n: usize) -> Result<u32, String> {
    u32::try_from(n).map_err(|_| "section too large".to_owned())
}

fn put_u32(o: &mut Vec<u8>, v: u32) {
    o.extend_from_slice(&v.to_le_bytes());
}

fn put_u32s(o: &mut Vec<u8>, v: &[u32]) -> Result<(), String> {
    put_u32(o, len32(v.len())?);
    for &x in v {
        put_u32(o, x);
    }
    Ok(())
}

fn put_bytes(o: &mut Vec<u8>, b: &[u8]) -> Result<(), String> {
    put_u32(o, len32(b.len())?);
    o.extend_from_slice(b);
    while o.len() % 4 != 0 {
        o.push(0);
    }
    Ok(())
}

fn put_groups(o: &mut Vec<u8>, g: &Groups) -> Result<(), String> {
    put_u32s(o, &g.keys)?;
    put_u32s(o, &g.key_start)?;
    put_u32s(o, &g.item_name)?;
    put_u32s(o, &g.item_start)?;
    put_u32s(o, &g.fuses)
}

/// Serializes a chip.
///
/// # Errors
///
/// When a section outgrows `u32` lengths.
pub fn encode(chip: &Chip) -> Result<Vec<u8>, String> {
    let mut o = MAGIC.to_vec();
    put_bytes(&mut o, chip.strings.text().as_bytes())?;
    put_u32s(&mut o, chip.strings.ends())?;
    put_u32(&mut o, len32(chip.tile_types.len())?);
    for t in &chip.tile_types {
        put_u32(&mut o, t.name);
        put_groups(&mut o, &t.muxes)?;
        put_groups(&mut o, &t.enums)?;
        put_u32s(&mut o, &t.enum_def)?;
        put_groups(&mut o, &t.words)?;
        put_u32s(&mut o, &t.word_def_start)?;
        put_bytes(&mut o, &t.word_def)?;
        put_u32s(&mut o, &t.fixed)?;
    }
    put_u32(&mut o, len32(chip.devices.len())?);
    for d in &chip.devices {
        let head = [
            d.name,
            d.family,
            d.idcode,
            d.frames,
            d.bits_per_frame,
            d.pad_bits_after_frame,
            d.pad_bits_before_frame,
            d.max_row,
            d.max_col,
        ];
        put_u32s(&mut o, &head)?;
        put_u32(&mut o, len32(d.tiles.len())?);
        for t in &d.tiles {
            for v in [t.name, t.ty, t.start_frame, t.start_bit, t.frames, t.bits] {
                put_u32(&mut o, v);
            }
        }
    }
    Ok(o)
}

struct Rd<'a> {
    b: &'a [u8],
    p: usize,
}

impl<'a> Rd<'a> {
    fn take(&mut self, n: usize) -> Result<&'a [u8], String> {
        let e = self.p.checked_add(n).filter(|e| *e <= self.b.len()).ok_or_else(|| "truncated".to_owned())?;
        let s = &self.b[self.p..e];
        self.p = e;
        Ok(s)
    }

    fn u32(&mut self) -> Result<u32, String> {
        let s = self.take(4)?;
        Ok(u32::from_le_bytes([s[0], s[1], s[2], s[3]]))
    }

    fn u32s(&mut self) -> Result<Vec<u32>, String> {
        let n = self.u32()? as usize;
        let bytes = n.checked_mul(4).ok_or_else(|| "section too large".to_owned())?;
        let s = self.take(bytes)?;
        Ok(s.chunks_exact(4).map(|c| u32::from_le_bytes([c[0], c[1], c[2], c[3]])).collect())
    }

    fn bytes(&mut self) -> Result<&'a [u8], String> {
        let n = self.u32()? as usize;
        let s = self.take(n)?;
        self.take((4 - n % 4) % 4)?;
        Ok(s)
    }

    fn groups(&mut self) -> Result<Groups, String> {
        Ok(Groups {
            keys: self.u32s()?,
            key_start: self.u32s()?,
            item_name: self.u32s()?,
            item_start: self.u32s()?,
            fuses: self.u32s()?,
        })
    }
}

/// Reads a chip written by [`encode`] and validates every table.
///
/// # Errors
///
/// On a wrong magic, truncation, or inconsistent tables.
pub fn decode(b: &[u8]) -> Result<Chip, String> {
    if b.get(..8) != Some(MAGIC.as_slice()) {
        return Err("not an oxibit-arch blob, or a different version".to_owned());
    }
    let mut r = Rd { b, p: 8 };
    let text = String::from_utf8(r.bytes()?.to_vec()).map_err(|_| "string text is not UTF-8".to_owned())?;
    let ends = r.u32s()?;
    let strings = Strings::from_parts(text, ends)?;
    let n = r.u32()? as usize;
    let mut tile_types = Vec::new();
    for _ in 0..n {
        let name = r.u32()?;
        let muxes = r.groups()?;
        let enums = r.groups()?;
        let enum_def = r.u32s()?;
        let words = r.groups()?;
        let word_def_start = r.u32s()?;
        let word_def = r.bytes()?.to_vec();
        let fixed = r.u32s()?;
        tile_types.push(TileType { name, muxes, enums, enum_def, words, word_def_start, word_def, fixed });
    }
    let nd = r.u32()? as usize;
    let mut devices = Vec::new();
    for _ in 0..nd {
        let h = r.u32s()?;
        let [name, family, idcode, frames, bits_per_frame, pad_bits_after_frame, pad_bits_before_frame, max_row, max_col] =
            h[..]
        else {
            return Err("bad device header".to_owned());
        };
        let nt = r.u32()? as usize;
        let mut tiles = Vec::new();
        for _ in 0..nt {
            tiles.push(Tile {
                name: r.u32()?,
                ty: r.u32()?,
                start_frame: r.u32()?,
                start_bit: r.u32()?,
                frames: r.u32()?,
                bits: r.u32()?,
            });
        }
        devices.push(Device {
            name,
            family,
            idcode,
            frames,
            bits_per_frame,
            pad_bits_after_frame,
            pad_bits_before_frame,
            max_row,
            max_col,
            tiles,
        });
    }
    if r.p != b.len() {
        return Err("trailing bytes".to_owned());
    }
    let chip = Chip { strings, tile_types, devices };
    chip.validate()?;
    Ok(chip)
}
