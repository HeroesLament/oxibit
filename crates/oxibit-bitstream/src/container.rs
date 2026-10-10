//! Reading and writing the three bitstream containers.

use serde::Serialize;

use crate::{Error, SYNC};

/// How the configuration words were (or will be) packaged.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Container {
    /// `.bit`: TLV header, then big-endian words.
    Bit,
    /// Header-less, big-endian words. The sync word reads `AA 99 55 66`.
    RawBin,
    /// Header-less, each word byte-swapped. The sync word reads `66 55 99 AA`.
    /// This is what the Zynq-7000 PCAP and Linux `fpga_manager` accept.
    SwappedBin,
}

/// The `.bit` header records. Each is a NUL-terminated string in the file.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize)]
pub struct BitHeader {
    /// Record `a`: design name, often followed by `;UserID=...;Version=...`
    /// (Vivado) or `;Generator=...` (xc7frames2bit).
    pub design: Option<String>,
    /// Record `b`: part name, e.g. `xc7z020clg400-1`.
    pub part: Option<String>,
    /// Record `c`: build date.
    pub date: Option<String>,
    /// Record `d`: build time.
    pub time: Option<String>,
}

/// Configuration words in canonical (big-endian, as the device sees them)
/// order, plus where they came from.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Bitstream {
    pub container: Container,
    pub header: Option<BitHeader>,
    pub words: Vec<u32>,
}

/// The fixed first field of every `.bit` Xilinx tools write: a 9-byte opaque
/// preamble behind a u16 length.
const BIT_MAGIC: [u8; 4] = [0x00, 0x09, 0x0F, 0xF0];
const BIT_PREAMBLE: [u8; 9] = [0x0F, 0xF0, 0x0F, 0xF0, 0x0F, 0xF0, 0x0F, 0xF0, 0x00];

impl Bitstream {
    /// Read any of the three containers, detecting which it is.
    ///
    /// A `.bit` is recognised by its fixed preamble. Otherwise the data must
    /// be word-aligned from byte 0 and contain the sync word at a word
    /// boundary in one of the two byte orders.
    pub fn parse(data: &[u8]) -> Result<Self, Error> {
        if data.starts_with(&BIT_MAGIC) {
            let (header, payload) = parse_bit(data)?;
            let words = be_words(payload)?;
            find_sync(&words).ok_or(Error::NoSync)?;
            return Ok(Bitstream { container: Container::Bit, header: Some(header), words });
        }

        if crate::zynq_boot::is_boot_image(data) {
            return Err(Error::IsBootImage);
        }

        let be = be_words(data)?;
        if find_sync(&be).is_some() {
            return Ok(Bitstream { container: Container::RawBin, header: None, words: be });
        }

        let swapped: Vec<u32> = be.iter().map(|w| w.swap_bytes()).collect();
        if find_sync(&swapped).is_some() {
            return Ok(Bitstream { container: Container::SwappedBin, header: None, words: swapped });
        }

        Err(Error::NoSync)
    }

    /// Big-endian bytes, no header (`RawBin`).
    pub fn to_raw_bin(&self) -> Vec<u8> {
        self.words.iter().flat_map(|w| w.to_be_bytes()).collect()
    }

    /// Byte-swapped words, no header (`SwappedBin`): the Zynq PCAP form.
    ///
    /// For a `.bit` input this is byte-for-byte what ehdl's
    /// `Hw.Xilinx.Bit2Bin.to_bin!/1` produces: the whole `e` record, swapped.
    pub fn to_swapped_bin(&self) -> Vec<u8> {
        self.words.iter().flat_map(|w| w.to_le_bytes()).collect()
    }

    /// A `.bit` with the given header. Records that are `None` are omitted,
    /// except `b` (part), which tools require.
    pub fn to_bit(&self, header: &BitHeader) -> Result<Vec<u8>, Error> {
        let part = header
            .part
            .as_deref()
            .ok_or_else(|| Error::BitHeader("a .bit needs a part name (record b)".into()))?;

        let mut out = Vec::with_capacity(self.words.len() * 4 + 128);
        out.extend_from_slice(&(BIT_PREAMBLE.len() as u16).to_be_bytes());
        out.extend_from_slice(&BIT_PREAMBLE);
        out.extend_from_slice(&1u16.to_be_bytes());

        let records = [
            (b'a', header.design.as_deref()),
            (b'b', Some(part)),
            (b'c', header.date.as_deref()),
            (b'd', header.time.as_deref()),
        ];
        for (key, value) in records {
            if let Some(v) = value {
                let len = u16::try_from(v.len() + 1)
                    .map_err(|_| Error::BitHeader(format!("record {} too long", key as char)))?;
                out.push(key);
                out.extend_from_slice(&len.to_be_bytes());
                out.extend_from_slice(v.as_bytes());
                out.push(0);
            }
        }

        let payload = self.to_raw_bin();
        let len = u32::try_from(payload.len())
            .map_err(|_| Error::BitHeader("configuration data over 4 GiB".into()))?;
        out.push(b'e');
        out.extend_from_slice(&len.to_be_bytes());
        out.extend_from_slice(&payload);
        Ok(out)
    }
}

/// Index of the first sync word, if any.
pub fn find_sync(words: &[u32]) -> Option<usize> {
    words.iter().position(|&w| w == SYNC)
}

fn be_words(data: &[u8]) -> Result<Vec<u32>, Error> {
    if data.len() % 4 != 0 {
        return Err(Error::Unaligned { bytes: data.len() });
    }
    Ok(data
        .chunks_exact(4)
        .map(|c| u32::from_be_bytes([c[0], c[1], c[2], c[3]]))
        .collect())
}

/// Split a `.bit` into its header records and the `e` record payload.
///
/// Layout: `u16 len, len bytes (opaque), u16 0x0001`, then records keyed by
/// one byte. `a`..`d` have a u16 length; `e` has a u32 length and is the
/// configuration data. Anything after `e` is ignored, as Xilinx tools do.
fn parse_bit(data: &[u8]) -> Result<(BitHeader, &[u8]), Error> {
    let mut r = Reader { data, pos: 0 };

    let preamble_len = r.u16()? as usize;
    r.take(preamble_len)?;

    let marker = r.u16()?;
    if marker != 1 {
        return Err(Error::BitHeader(format!(
            "expected 0x0001 after the preamble, found 0x{marker:04X}"
        )));
    }

    let mut header = BitHeader::default();
    loop {
        let key = r.u8()?;
        match key {
            b'a' | b'b' | b'c' | b'd' => {
                let len = r.u16()? as usize;
                let value = c_string(r.take(len)?);
                match key {
                    b'a' => header.design = Some(value),
                    b'b' => header.part = Some(value),
                    b'c' => header.date = Some(value),
                    _ => header.time = Some(value),
                }
            }
            b'e' => {
                let len = r.u32()? as usize;
                let payload = r.take(len)?;
                return Ok((header, payload));
            }
            other => {
                return Err(Error::BitHeader(format!(
                    "unknown record key 0x{other:02X} at byte {}",
                    r.pos - 1
                )));
            }
        }
    }
}

fn c_string(bytes: &[u8]) -> String {
    let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    String::from_utf8_lossy(&bytes[..end]).into_owned()
}

struct Reader<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    fn take(&mut self, n: usize) -> Result<&'a [u8], Error> {
        let end = self.pos.checked_add(n).filter(|&e| e <= self.data.len()).ok_or_else(|| {
            Error::BitHeader(format!(
                "truncated: wanted {n} bytes at offset {}, file is {} bytes",
                self.pos,
                self.data.len()
            ))
        })?;
        let s = &self.data[self.pos..end];
        self.pos = end;
        Ok(s)
    }

    fn u8(&mut self) -> Result<u8, Error> {
        Ok(self.take(1)?[0])
    }

    fn u16(&mut self) -> Result<u16, Error> {
        let b = self.take(2)?;
        Ok(u16::from_be_bytes([b[0], b[1]]))
    }

    fn u32(&mut self) -> Result<u32, Error> {
        let b = self.take(4)?;
        Ok(u32::from_be_bytes([b[0], b[1], b[2], b[3]]))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_words() -> Vec<u32> {
        vec![0xFFFF_FFFF, 0x0000_00BB, 0x1122_0044, 0xFFFF_FFFF, SYNC, 0x2000_0000]
    }

    #[test]
    fn bit_roundtrip_and_swap() {
        let bs = Bitstream { container: Container::RawBin, header: None, words: sample_words() };
        let header = BitHeader {
            design: Some("t.frames;Generator=test".into()),
            part: Some("xc7z020clg400-1".into()),
            date: Some("2026/09/28".into()),
            time: Some("20:00:00".into()),
        };
        let bit = bs.to_bit(&header).unwrap();
        let back = Bitstream::parse(&bit).unwrap();
        assert_eq!(back.container, Container::Bit);
        assert_eq!(back.header.as_ref(), Some(&header));
        assert_eq!(back.words, bs.words);

        let swapped = back.to_swapped_bin();
        assert_eq!(&swapped[16..20], &[0x66, 0x55, 0x99, 0xAA]);
        let again = Bitstream::parse(&swapped).unwrap();
        assert_eq!(again.container, Container::SwappedBin);
        assert_eq!(again.words, bs.words);

        let raw = Bitstream::parse(&back.to_raw_bin()).unwrap();
        assert_eq!(raw.container, Container::RawBin);
    }

    #[test]
    fn rejects_unaligned_and_syncless() {
        assert_eq!(Bitstream::parse(&[0xFF; 7]), Err(Error::Unaligned { bytes: 7 }));
        assert_eq!(Bitstream::parse(&[0xFF; 8]), Err(Error::NoSync));
    }
}
