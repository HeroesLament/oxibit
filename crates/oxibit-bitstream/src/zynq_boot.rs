//! Zynq-7000 boot images (`BOOT.BIN`, as written by bootgen).
//!
//! Layout, all words little-endian (verified against a bootgen image, see
//! the tests and `docs/formats.md`):
//!
//! ```text
//! 0x000  8 words   ARM vector table
//! 0x020  0xAA995566  width detection (the bitstream sync word, by design)
//! 0x024  'XNLX'    header signature (0x584C4E58)
//! 0x048  checksum  !sum(words 0x020..=0x044)
//! 0x098  byte offset of the image header table
//! 0x09C  byte offset of the partition header table
//! ```
//!
//! Image header table: `version, image count, first partition header (word
//! offset), first image header (word offset)`. Image header: `next image header
//! (word offset), partition header (word offset), reserved, partition count,
//! name` with the name packed big-endian inside little-endian words.
//!
//! Partition header, 16 words: encrypted length, unencrypted length, total
//! length (all in words), load address, exec address, data offset (words),
//! attributes, section count, checksum offset, image header offset, AC
//! offset, 4 reserved, checksum = !sum(words 0..=14). Attribute bits [5:4]
//! give the destination: 1 = PS, 2 = PL.

use serde::Serialize;

use crate::{Error, SYNC};

const SIGNATURE: u32 = 0x584C_4E58; // "XNLX" read as a little-endian word
const PARTITION_HEADER_WORDS: usize = 16;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Destination {
    None,
    Ps,
    Pl,
    Other(u32),
}

#[derive(Debug, Clone, Serialize)]
pub struct Partition {
    pub index: usize,
    pub name: Option<String>,
    pub destination: Destination,
    pub load_address: u32,
    pub exec_address: u32,
    /// Byte offset of the partition data in the image.
    pub offset: usize,
    /// Unencrypted length in bytes.
    pub length: usize,
    /// Encrypted length differs from unencrypted length.
    pub encrypted: bool,
    pub attributes: u32,
    pub checksum_ok: bool,
}

#[derive(Debug, Clone, Serialize)]
pub struct BootImage {
    pub header_checksum_ok: bool,
    pub partitions: Vec<Partition>,
}

impl Partition {
    pub fn data<'a>(&self, image: &'a [u8]) -> &'a [u8] {
        &image[self.offset..self.offset + self.length]
    }
}

/// Width-detection word and signature at their fixed offsets.
pub fn is_boot_image(data: &[u8]) -> bool {
    le(data, 0x20) == Some(SYNC) && le(data, 0x24) == Some(SIGNATURE)
}

pub fn parse(data: &[u8]) -> Result<BootImage, Error> {
    if !is_boot_image(data) {
        return Err(Error::BootImage("no width-detection word and XNLX signature".into()));
    }

    let sum = (0x20..=0x44).step_by(4).try_fold(0u32, |acc, off| Some(acc.wrapping_add(le(data, off)?)));
    let header_checksum_ok = sum.map(|s| !s) == le(data, 0x48);

    let pht = word(data, 0x9C)? as usize;
    let mut partitions = Vec::new();

    for index in 0.. {
        let base = pht + index * PARTITION_HEADER_WORDS * 4;
        let w: Vec<u32> = (0..PARTITION_HEADER_WORDS)
            .map(|i| word(data, base + i * 4))
            .collect::<Result<_, _>>()?;

        // The table ends with an all-zero header (checksum 0xFFFFFFFF).
        if w[..15].iter().all(|&x| x == 0) {
            break;
        }

        let checksum_ok = !w[..15].iter().fold(0u32, |a, &x| a.wrapping_add(x)) == w[15];
        let attributes = w[6];
        let destination = match (attributes >> 4) & 0x3 {
            0 => Destination::None,
            1 => Destination::Ps,
            2 => Destination::Pl,
            other => Destination::Other(other),
        };
        let offset = w[5] as usize * 4;
        let length = w[1] as usize * 4;
        if offset + length > data.len() {
            return Err(Error::BootImage(format!(
                "partition {index} runs past the end of the image ({} + {} > {})",
                offset,
                length,
                data.len()
            )));
        }

        partitions.push(Partition {
            index,
            name: image_name(data, w[9] as usize * 4),
            destination,
            load_address: w[3],
            exec_address: w[4],
            offset,
            length,
            encrypted: w[0] != w[1],
            attributes,
            checksum_ok,
        });
    }

    Ok(BootImage { header_checksum_ok, partitions })
}

/// Name from an image header: words from offset + 16, each holding four
/// characters big-endian, NUL-terminated.
fn image_name(data: &[u8], image_header: usize) -> Option<String> {
    if image_header == 0 {
        return None;
    }
    let mut bytes = Vec::new();
    let mut off = image_header + 16;
    'words: while let Some(w) = le(data, off) {
        for b in w.to_be_bytes() {
            if b == 0 {
                break 'words;
            }
            bytes.push(b);
        }
        off += 4;
        if bytes.len() > 256 {
            return None;
        }
    }
    Some(String::from_utf8_lossy(&bytes).into_owned())
}

fn le(data: &[u8], off: usize) -> Option<u32> {
    let b = data.get(off..off + 4)?;
    Some(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
}

fn word(data: &[u8], off: usize) -> Result<u32, Error> {
    le(data, off).ok_or_else(|| Error::BootImage(format!("truncated at byte 0x{off:X}")))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A minimal image with one PL partition, laid out like bootgen's.
    fn tiny_image() -> Vec<u8> {
        let mut img = vec![0u8; 0x200];
        let put = |img: &mut Vec<u8>, off: usize, v: u32| img[off..off + 4].copy_from_slice(&v.to_le_bytes());
        put(&mut img, 0x20, SYNC);
        put(&mut img, 0x24, SIGNATURE);
        let sum = (0x20..=0x44).step_by(4).fold(0u32, |a, o| a.wrapping_add(le(&img, o).unwrap()));
        put(&mut img, 0x48, !sum);
        put(&mut img, 0x9C, 0x100);
        // image header at 0xC0 named "system_top.bit"
        for (i, chunk) in b"system_top.bit\0\0".chunks(4).enumerate() {
            put(&mut img, 0xC0 + 16 + i * 4, u32::from_be_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]));
        }
        // partition header at 0x100: 4 words of data at 0x180
        let hdr = [4, 4, 4, 0, 0, 0x180 / 4, 0x20, 1, 0, 0xC0 / 4, 0, 0, 0, 0, 0];
        for (i, v) in hdr.iter().enumerate() {
            put(&mut img, 0x100 + i * 4, *v);
        }
        put(&mut img, 0x100 + 60, !hdr.iter().fold(0u32, |a, &x| a.wrapping_add(x)));
        img
    }

    #[test]
    fn parses_tiny_image() {
        let img = tiny_image();
        let boot = parse(&img).unwrap();
        assert!(boot.header_checksum_ok);
        assert_eq!(boot.partitions.len(), 1);
        let p = &boot.partitions[0];
        assert_eq!(p.destination, Destination::Pl);
        assert_eq!(p.name.as_deref(), Some("system_top.bit"));
        assert_eq!((p.offset, p.length), (0x180, 16));
        assert!(p.checksum_ok && !p.encrypted);
    }
}
