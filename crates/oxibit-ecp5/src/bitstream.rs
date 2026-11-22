//! ECP5 bitstream serialisation, as libtrellis's `Bitstream::serialise_chip`
//! writes it for the ECP5 family (`BitstreamOptions`: frames in reverse
//! order, 4 dummy bytes after the preamble, CRC after every frame and one
//! `0xFF` byte after it, 12 bytes of SECURITY/SED space).
//!
//! Stream, after the `.bit` header (`FF 00`, each metadata string with a NUL,
//! `FF`):
//! 1. preamble `FF FF BD B3`, 4 dummy `FF` bytes, optional `SPI_MODE`;
//! 2. `LSC_RESET_CRC`, `VERIFY_ID` with the IDCODE, `LSC_PROG_CNTRL0`;
//! 3. `LSC_INIT_ADDRESS`, `LSC_PROG_INCR_RTI` (`0x91`, frame count), every
//!    frame MSB-first with its CRC16 and a dummy byte; or, compressed,
//!    `LSC_WRITE_COMP_DIC` with an 8-byte dictionary and
//!    `LSC_PROG_INCR_CMP` with every frame coded (`write_compressed_frames`);
//! 4. SECURITY/SED space, `ISC_PROGRAM_USERCODE` with its CRC16;
//! 5. per initialised EBR: `LSC_EBR_ADDRESS`, `LSC_EBR_WRITE` and 256
//!    72-bit frames (eight 9-bit words each), CRC16;
//! 6. `ISC_PROGRAM_DONE`, 4 dummy bytes.
//!
//! The CRC is CRC-16 with polynomial `0x8005`, initial 0, fed MSB first and
//! finished by shifting out 16 zero bits; dummy (`insert_dummy`) bytes do
//! not feed it, every written byte does.

use crate::chip::Chip;
use crate::{Result, err};

/// Command opcodes (libtrellis `BitstreamCommand`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
enum Cmd {
    SpiMode = 0x79,
    LscResetCrc = 0x3B,
    VerifyId = 0xE2,
    LscProgCntrl0 = 0x22,
    LscInitAddress = 0x46,
    LscProgIncrRti = 0x82,
    IscProgramUsercode = 0xC2,
    LscEbrAddress = 0xF6,
    LscEbrWrite = 0xB2,
    IscProgramDone = 0x5E,
    LscWriteCompDic = 0x02,
    LscProgIncrCmp = 0xB8,
}

const PREAMBLE: [u8; 4] = [0xFF, 0xFF, 0xBD, 0xB3];
const CRC16_POLY: u16 = 0x8005;

/// ECP5 values of libtrellis's `BitstreamOptions`.
const DUMMY_BYTES_AFTER_PREAMBLE: usize = 4;
const CRC_META: u8 = 0x91;
const DUMMY_BYTES_AFTER_FRAME: usize = 1;
const SECURITY_SED_SPACE: usize = 12;

/// `MCCLK_FREQ` settings (MHz) and their `ctrl0` bits.
const FREQUENCIES: [(&str, u32); 6] = [
    ("2.4", 0x00),
    ("4.8", 0x01),
    ("9.7", 0x20),
    ("19.4", 0x30),
    ("38.8", 0x38),
    ("62.0", 0x3b),
];
const SPI_MODES: [(&str, u8); 3] = [("fast-read", 0x49), ("dual-spi", 0x51), ("qspi", 0x59)];

/// The `ecppack` options this serialiser supports.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Options {
    /// Master clock frequency, one of `FREQUENCIES` (`--freq`).
    pub freq: Option<String>,
    /// `--spimode`.
    pub spimode: Option<String>,
    /// `--compress`: frames written with `LSC_PROG_INCR_CMP`.
    pub compress: bool,
    /// `--usercode`.
    pub usercode: Option<u32>,
    /// `--idcode`.
    pub idcode: Option<u32>,
}

impl Options {
    /// The options `ecppack` takes from a configuration's `.sysconfig`, with
    /// `over` (its command line) taking precedence.
    pub fn from_sysconfig(
        sys: &std::collections::BTreeMap<String, String>,
        over: &Options,
    ) -> Options {
        let freq = sys.get("MCCLK_FREQ").map(|f| {
            if f == "62" {
                "62.0".to_owned()
            } else {
                f.clone()
            }
        });
        let compress = sys.get("COMPRESS_CONFIG").is_some_and(|v| v == "ON");
        Options {
            freq: over.freq.clone().or(freq),
            spimode: over.spimode.clone(),
            compress: over.compress || compress,
            usercode: over.usercode,
            idcode: over.idcode,
        }
    }
}

#[derive(Debug, Default)]
struct Writer {
    data: Vec<u8>,
    crc: u16,
}

impl Writer {
    fn update_crc(&mut self, val: u8) {
        for i in (0..8).rev() {
            let flag = self.crc >> 15 != 0;
            self.crc = (self.crc << 1) | u16::from((val >> i) & 1);
            if flag {
                self.crc ^= CRC16_POLY;
            }
        }
    }

    fn byte(&mut self, b: u8) {
        self.data.push(b);
        self.update_crc(b);
    }

    fn bytes(&mut self, bs: &[u8]) {
        for &b in bs {
            self.byte(b);
        }
    }

    fn cmd(&mut self, c: Cmd) {
        self.byte(c as u8);
    }

    fn zeros(&mut self, n: usize) {
        for _ in 0..n {
            self.byte(0);
        }
    }

    /// `0xFF` bytes that do not feed the CRC.
    fn dummy(&mut self, n: usize) {
        self.data.extend(std::iter::repeat_n(0xFF, n));
    }

    fn u16(&mut self, v: u16) {
        self.bytes(&v.to_be_bytes());
    }

    fn u32(&mut self, v: u32) {
        self.bytes(&v.to_be_bytes());
    }

    /// Shift out 16 zero bits, write the CRC, start a new one.
    fn crc16(&mut self) {
        for _ in 0..16 {
            let flag = self.crc >> 15 != 0;
            self.crc <<= 1;
            if flag {
                self.crc ^= CRC16_POLY;
            }
        }
        let crc = self.crc;
        self.u16(crc);
        self.crc = 0;
    }
}

/// The frame as bytes: bit `j` at offset `j + pad_bits_after_frame`,
/// counted from the last byte's LSB.
fn frame_bytes(chip: &Chip, frame: usize, out: &mut [u8]) {
    out.fill(0);
    let last = out.len() - 1;
    for j in 0..chip.info.bits_per_frame {
        let ofs = j + chip.info.pad_bits_after_frame;
        out[last - ofs / 8] |= u8::from(chip.cram.get(frame, j)) << (ofs % 8);
    }
}

/// The bit position of a byte with exactly one bit set.
fn one_hot(b: u8) -> Option<u8> {
    #[expect(
        clippy::cast_possible_truncation,
        reason = "trailing_zeros of a u8 is at most 8"
    )]
    b.is_power_of_two().then(|| b.trailing_zeros() as u8)
}

/// MSB-first bit stream into a `Writer`, a byte at a time.
#[derive(Debug, Default)]
struct Bits {
    buf: u8,
    n: u8,
}

impl Bits {
    fn push(&mut self, w: &mut Writer, one: Bit) {
        if one == Bit::One {
            self.buf |= 1 << (7 - self.n);
        }
        self.n += 1;
        if self.n == 8 {
            self.flush(w);
        }
    }

    /// The low `len` bits of `x`, most significant first.
    fn push_n(&mut self, w: &mut Writer, x: u32, len: u32) {
        for i in (0..len).rev() {
            self.push(w, if x >> i & 1 == 1 { Bit::One } else { Bit::Zero });
        }
    }

    /// Write a partial byte, zero padded.
    fn flush(&mut self, w: &mut Writer) {
        if self.n != 0 {
            w.byte(self.buf);
            self.buf = 0;
            self.n = 0;
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Bit {
    Zero,
    One,
}

/// libtrellis `write_compressed_frames` (ECP5: no one-hot dictionary).
///
/// The dictionary is the 8 most frequent byte values over all frames that
/// are neither zero nor one-hot (those have their own codes), ties to the
/// larger value, as `std::priority_queue<pair<count, byte>>` orders them;
/// it is written most frequent last. Each frame is front-padded with zero
/// bits to a multiple of 64 bits of input, then each byte is coded:
/// `0` zero byte, `100xxx` one-hot (bit `xxx`), `101xxx` dictionary entry,
/// `11` + 8 bits literal; the frame is flushed to a byte, then CRC16 and one
/// `0xFF` byte as uncompressed frames have.
fn write_compressed_frames(w: &mut Writer, frames: &[Vec<u8>]) -> Result<()> {
    let mut histogram = [0usize; 256];
    for b in frames.iter().flatten() {
        histogram[usize::from(*b)] += 1;
    }
    let mut candidates: Vec<(usize, u8)> = (1..=255u8)
        .filter(|&b| one_hot(b).is_none())
        .map(|b| (histogram[usize::from(b)], b))
        .collect();
    candidates.sort_unstable_by(|a, b| b.cmp(a));
    let dict: Vec<u8> = candidates.iter().take(8).map(|&(_, b)| b).collect();

    w.cmd(Cmd::LscWriteCompDic);
    w.zeros(3);
    for &b in dict.iter().rev() {
        w.byte(b);
    }
    w.cmd(Cmd::LscProgIncrCmp);
    w.byte(CRC_META);
    let n = u16::try_from(frames.len())
        .map_err(|_| err!("{} frames do not fit the frame count field", frames.len()))?;
    w.u16(n);

    let mut bits = Bits::default();
    for fr in frames {
        let pad = (8 - fr.len() % 8) % 8;
        for _ in 0..pad {
            bits.push(w, Bit::Zero);
        }
        for &b in fr {
            if b == 0 {
                bits.push(w, Bit::Zero);
            } else if let Some(oh) = one_hot(b) {
                bits.push_n(w, 0b100, 3);
                bits.push_n(w, u32::from(oh), 3);
            } else if let Some(j) = dict.iter().position(|&d| d == b) {
                bits.push_n(w, 0b101, 3);
                bits.push_n(w, u32::try_from(j).expect("dictionary index below 8"), 3);
            } else {
                bits.push_n(w, 0b11, 2);
                bits.push_n(w, u32::from(b), 8);
            }
        }
        bits.flush(w);
        w.crc16();
        for _ in 0..DUMMY_BYTES_AFTER_FRAME {
            w.byte(0xFF);
        }
    }
    Ok(())
}

/// Eight 9-bit words as nine bytes, first word's MSB first.
fn ebr_frame(w: &[u16]) -> [u8; 9] {
    // Truncations to u8 are the byte packing: each byte takes the low bits
    // of one word shifted against the next.
    #[expect(
        clippy::cast_possible_truncation,
        reason = "packing 9-bit words into bytes"
    )]
    let b = |v: u16| v as u8;
    [
        b(w[0] >> 1),
        b((w[0] & 0x01) << 7 | (w[1] >> 2)),
        b((w[1] & 0x03) << 6 | (w[2] >> 3)),
        b((w[2] & 0x07) << 5 | (w[3] >> 4)),
        b((w[3] & 0x0F) << 4 | (w[4] >> 5)),
        b((w[4] & 0x1F) << 3 | (w[5] >> 6)),
        b((w[5] & 0x3F) << 2 | (w[6] >> 7)),
        b((w[6] & 0x7F) << 1 | (w[7] >> 8)),
        b(w[7]),
    ]
}

/// The whole `.bit` file.
pub fn serialise(chip: &Chip, opts: &Options) -> Result<Vec<u8>> {
    let mut ctrl0 = chip.ctrl0;
    if let Some(f) = &opts.freq {
        let (_, bits) = FREQUENCIES
            .iter()
            .find(|(n, _)| n == f)
            .ok_or_else(|| err!("bad frequency option {f}"))?;
        ctrl0 |= bits;
    }
    let mut w = Writer::default();
    w.bytes(&PREAMBLE);
    w.dummy(DUMMY_BYTES_AFTER_PREAMBLE);
    if let Some(m) = &opts.spimode {
        let (_, mode) = SPI_MODES
            .iter()
            .find(|(n, _)| n == m)
            .ok_or_else(|| err!("bad spimode option {m}"))?;
        w.cmd(Cmd::SpiMode);
        w.byte(*mode);
        w.zeros(2);
    }
    w.cmd(Cmd::LscResetCrc);
    w.zeros(3);
    w.crc = 0;
    w.cmd(Cmd::VerifyId);
    w.zeros(3);
    w.u32(opts.idcode.unwrap_or(chip.info.idcode));
    w.cmd(Cmd::LscProgCntrl0);
    w.zeros(3);
    w.u32(ctrl0);
    w.cmd(Cmd::LscInitAddress);
    w.zeros(3);

    let frames = chip.info.frames;
    let bytes_per_frame = (chip.info.bits_per_frame
        + chip.info.pad_bits_after_frame
        + chip.info.pad_bits_before_frame)
        / 8;
    if opts.compress {
        let mut all = vec![vec![0u8; bytes_per_frame]; frames];
        for (i, buf) in all.iter_mut().enumerate() {
            frame_bytes(chip, frames - 1 - i, buf);
        }
        write_compressed_frames(&mut w, &all)?;
    } else {
        let frames16 = u16::try_from(frames)
            .map_err(|_| err!("{frames} frames do not fit the frame count field"))?;
        w.cmd(Cmd::LscProgIncrRti);
        w.byte(CRC_META);
        w.u16(frames16);
        let mut buf = vec![0u8; bytes_per_frame];
        for i in 0..frames {
            frame_bytes(chip, frames - 1 - i, &mut buf);
            w.bytes(&buf);
            w.crc16();
            for _ in 0..DUMMY_BYTES_AFTER_FRAME {
                w.byte(0xFF);
            }
        }
    }
    w.dummy(SECURITY_SED_SPACE);

    w.cmd(Cmd::IscProgramUsercode);
    w.byte(0x80);
    w.zeros(2);
    w.u32(opts.usercode.unwrap_or(chip.usercode));
    w.crc16();

    let ebr_frames =
        u16::try_from(chip.bram_data_size >> 3).map_err(|_| err!("EBR size too large"))?;
    for (&addr, data) in &chip.bram_data {
        if data.len() < chip.bram_data_size {
            return Err(err!(
                "EBR {addr}: {} words initialised, {} needed",
                data.len(),
                chip.bram_data_size
            ));
        }
        w.cmd(Cmd::LscEbrAddress);
        w.zeros(3);
        w.u32(u32::from(addr) << 11);
        w.cmd(Cmd::LscEbrWrite);
        w.byte(0xD0);
        w.u16(ebr_frames);
        for words in data[..chip.bram_data_size].chunks_exact(8) {
            w.bytes(&ebr_frame(words));
        }
        w.crc16();
    }

    w.cmd(Cmd::IscProgramDone);
    w.zeros(3);
    w.dummy(4);

    let mut out = vec![0xFF, 0x00];
    for m in &chip.metadata {
        out.extend_from_slice(m.as_bytes());
        out.push(0);
    }
    out.push(0xFF);
    out.extend_from_slice(&w.data);
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_of_known_bytes() {
        // A CRC16 written after its own input checks to zero when the reader
        // feeds both through and shifts out 16 more zero bits.
        let mut w = Writer::default();
        w.bytes(&[0xE2, 0, 0, 0, 0x41, 0x11, 0x30, 0x43]);
        w.crc16();
        let mut r = Writer::default();
        r.bytes(&w.data);
        for _ in 0..16 {
            let flag = r.crc >> 15 != 0;
            r.crc <<= 1;
            if flag {
                r.crc ^= CRC16_POLY;
            }
        }
        assert_eq!(r.crc, 0, "CRC over data and its own CRC is zero");
    }

    #[test]
    fn ebr_frame_packs_nine_bit_words() {
        assert_eq!(ebr_frame(&[0x1FF; 8]), [0xFF; 9]);
        assert_eq!(
            ebr_frame(&[0x100, 0, 0, 0, 0, 0, 0, 0x001]),
            [0x80, 0, 0, 0, 0, 0, 0, 0, 0x01]
        );
    }
}
