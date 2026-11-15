//! Nexus bitstream serialisation.
//!
//! Container: `LSCC`, metadata strings between `FF 00` and `00 FF`, then
//! preamble `FF FF BD B3`. Commands are one opcode byte and three operand
//! bytes, big-endian payloads. A running CRC-16 (poly 0x8005, init 0, bits
//! MSB first, flushed with 16 zero bits) covers every byte except `FF`
//! padding and the `LSCC` magic and metadata text; it is reset after each
//! inserted CRC and by `LSC_RESET_CRC`.
//!
//! Frames: `bits_per_frame` CRAM bits, high index first, placed above
//! `frame_ecc_bits` (14) of ECC (poly 0x202D over the frame bits, flushed
//! with 14 zero bits), right aligned in whole bytes. Order of writes:
//! 32 IO frames from address 0x8000, the main frames from address 0 (which
//! maps to the last flat frame), the TAP frames from 0x8020; then power
//! control, IP bus writes, usercode with CRC check, DONE.
//!
//! Compressed frames (`LSC_PROG_INCR_CMP`) code each byte as `0` (zero),
//! `10xxxx` (dictionary entry 15-x) or `11xxxxxxxx` (literal), padded to 64
//! bits per frame; the dictionary holds the 16 commonest frame bytes.

use crate::chip::Chip;
use crate::{Result, err};

const COMMENT_START: [u8; 2] = [0xFF, 0x00];
const COMMENT_END: [u8; 2] = [0x00, 0xFF];
const PREAMBLE: [u8; 4] = [0xFF, 0xFF, 0xBD, 0xB3];

const LSC_RESET_CRC: u8 = 0x3B;
const VERIFY_ID: u8 = 0xE2;
const LSC_WRITE_COMP_DIC: u8 = 0x02;
const LSC_PROG_CNTRL0: u8 = 0x22;
const LSC_INIT_ADDRESS: u8 = 0x46;
const LSC_WRITE_ADDRESS: u8 = 0xB4;
const LSC_PROG_INCR_CMP: u8 = 0xB8;
const LSC_PROG_INCR_RTI: u8 = 0x82;
const ISC_PROGRAM_USERCODE: u8 = 0xC2;
const LSC_BUS_ADDRESS: u8 = 0xF6;
const LSC_BUS_WRITE: u8 = 0x72;
const ISC_PROGRAM_DONE: u8 = 0x5E;
const LSC_POWER_CTRL: u8 = 0x56;

const CRC16_POLY: u16 = 0x8005;
const ECC_POLY: u16 = 0x202D;

struct W {
    data: Vec<u8>,
    crc: u16,
    ecc: u16,
    dic: [u8; 16],
}

impl W {
    fn crc_byte(&mut self, v: u8) {
        for i in (0..8).rev() {
            let top = self.crc >> 15;
            self.crc = (self.crc << 1) | ((v >> i) & 1) as u16;
            if top != 0 {
                self.crc ^= CRC16_POLY;
            }
        }
    }
    fn ecc_bit(&mut self, v: bool) {
        let top = self.ecc >> 13;
        self.ecc = ((self.ecc << 1) | v as u16) & 0x3FFF;
        if top != 0 {
            self.ecc ^= ECC_POLY;
        }
    }
    fn ecc_final(&mut self) -> u16 {
        for _ in 0..14 {
            self.ecc_bit(false);
        }
        self.ecc
    }
    fn byte(&mut self, b: u8) {
        self.data.push(b);
        self.crc_byte(b);
    }
    fn bytes(&mut self, bs: &[u8]) {
        for &b in bs {
            self.byte(b);
        }
    }
    fn zeros(&mut self, n: usize) {
        for _ in 0..n {
            self.byte(0);
        }
    }
    fn padding(&mut self, n: usize) {
        self.data.extend(std::iter::repeat_n(0xFF, n));
    }
    fn u16(&mut self, h: u16) {
        self.bytes(&h.to_be_bytes());
    }
    fn u32(&mut self, w: u32) {
        self.bytes(&w.to_be_bytes());
    }
    fn insert_crc(&mut self) {
        for _ in 0..16 {
            let top = self.crc >> 15;
            self.crc <<= 1;
            if top != 0 {
                self.crc ^= CRC16_POLY;
            }
        }
        self.u16(self.crc);
        self.crc = 0;
    }
    fn cmd(&mut self, op: u8) {
        self.byte(op);
        self.zeros(3);
    }
    fn frame_addr(&mut self, addr: u32) {
        self.cmd(LSC_WRITE_ADDRESS);
        self.u32(addr);
    }

    /// One frame's bytes, `len` long, ECC in the low 14 bits.
    fn frame_bytes(&mut self, c: &Chip, frame: usize, len: usize, with_ecc: bool) -> Vec<u8> {
        let bpf = c.db.data.bits_per_frame;
        let pad = c.db.data.frame_ecc_bits + c.db.data.pad_bits_after_frame;
        let mut fb = vec![0u8; len];
        self.ecc = 0;
        for j in (0..bpf).rev() {
            let ofs = j + pad;
            let v = c.cram.get(frame, j);
            if with_ecc {
                self.ecc_bit(v);
            }
            if v {
                fb[len - 1 - ofs / 8] |= 1 << (ofs % 8);
            }
        }
        if with_ecc {
            let ecc = self.ecc_final();
            fb[len - 2] |= ((ecc >> 8) & 0x3F) as u8;
            fb[len - 1] |= (ecc & 0xFF) as u8;
        }
        fb
    }

    fn frame_index(c: &Chip, addr: u32) -> Result<usize> {
        c.frame_addr_to_idx(addr).ok_or_else(|| err!("frame address 0x{addr:08x} outside {}", c.db.device))
    }

    fn frames(&mut self, c: &Chip, start: u32, count: usize) -> Result<()> {
        self.byte(LSC_PROG_INCR_RTI);
        self.byte(0x91);
        self.u16(count16(count)?);
        let len = (c.db.data.bits_per_frame + 14 + 7) / 8;
        for f in 0..count {
            let idx = W::frame_index(c, start + f as u32)?;
            let fb = self.frame_bytes(c, idx, len, true);
            self.bytes(&fb);
            self.insert_crc();
            self.byte(0xFF);
        }
        Ok(())
    }

    fn comp_frames(&mut self, c: &Chip, start: u32, count: usize) -> Result<()> {
        self.byte(LSC_PROG_INCR_CMP);
        self.byte(0xD4);
        self.u16(count16(count)?);
        let len = 8 * ((c.db.data.bits_per_frame + 14 + 63) / 64);
        for f in 0..count {
            let idx = W::frame_index(c, start + f as u32)?;
            let fb = self.frame_bytes(c, idx, len, true);
            self.comp_frame(&fb);
            if f + 1 == count {
                self.insert_crc();
            }
            for _ in 0..4 {
                self.byte(0xFF);
            }
        }
        Ok(())
    }

    fn comp_dic(&mut self, c: &Chip) {
        let len = (c.db.data.bits_per_frame + 14 + 7) / 8;
        let mut hist = [0usize; 256];
        for f in 0..c.cram.frames {
            for b in self.frame_bytes(c, f, len, false) {
                hist[b as usize] += 1;
            }
        }
        let mut pairs: Vec<(usize, u8)> = (0..256).map(|i| (hist[i], i as u8)).collect();
        pairs.sort();
        for i in 0..16 {
            self.dic[i] = pairs[240 + i].1;
        }
    }

    fn comp_frame(&mut self, frame: &[u8]) {
        let mut buf = 0u8;
        let mut n = 0usize;
        let mut out = Vec::new();
        let mut put = |bit: bool, out: &mut Vec<u8>| {
            if bit {
                buf |= 1 << (7 - n);
            }
            n += 1;
            if n == 8 {
                out.push(buf);
                n = 0;
                buf = 0;
            }
        };
        for &b in frame {
            if b == 0 {
                put(false, &mut out);
                continue;
            }
            if let Some(i) = self.dic.iter().position(|&d| d == b) {
                let code = (0b10u16 << 4) | (15 - i) as u16;
                for k in (0..6).rev() {
                    put(code & (1 << k) != 0, &mut out);
                }
                continue;
            }
            let code = (0b11u16 << 8) | b as u16;
            for k in (0..10).rev() {
                put(code & (1 << k) != 0, &mut out);
            }
        }
        if n != 0 {
            out.push(buf);
        }
        self.bytes(&out);
    }

    fn ip_config(&mut self, c: &Chip) -> Result<()> {
        let mut chunks: Vec<(u32, Vec<u8>)> = Vec::new();
        let mut last: Option<u32> = None;
        for (&addr, &val) in c.ipconfig.iter().filter(|(a, _)| *a & 0xFF00_0000 != 0x0E00_0000) {
            let fresh = match (last, chunks.last()) {
                (Some(l), Some((_, bytes))) => l.wrapping_add(1) != addr || bytes.len() >= 40960,
                _ => true,
            };
            if fresh {
                chunks.push((addr, Vec::new()));
            }
            if let Some((_, bytes)) = chunks.last_mut() {
                bytes.push(val);
            }
            last = Some(addr);
        }
        // PLL bytes go one at a time, highest address first.
        for (&addr, &val) in c.ipconfig.iter().filter(|(a, _)| *a & 0xFF00_0000 == 0x0E00_0000).rev() {
            chunks.push((addr, vec![val]));
        }
        for (start, bytes) in chunks {
            self.cmd(LSC_BUS_ADDRESS);
            let mut adj = start;
            if adj & 0xFF00_0000 == 0x2E00_0000 {
                // LRAM: byte offsets in 10-bit-per-8 units.
                let ls = adj & 0x1FFFF;
                adj = (adj & 0xFFFE_0000) | ((ls * 8) / 10);
            }
            self.u32(adj);
            self.padding(9);
            let fs = Chip::bus_frame_size(start)?;
            let nframes = bytes.len().div_ceil(fs);
            self.byte(LSC_BUS_WRITE);
            self.byte(0xD0);
            self.u16(count16(nframes)?);
            for i in 0..fs * nframes {
                self.byte(bytes.get(i).copied().unwrap_or(0));
            }
            self.insert_crc();
        }
        Ok(())
    }
}

fn count16(n: usize) -> Result<u16> {
    u16::try_from(n).map_err(|_| err!("{n} frames in one write"))
}

/// The bitstream for a configured chip.
pub fn serialise(c: &Chip) -> Result<Vec<u8>> {
    let mut b = W { data: Vec::new(), crc: 0, ecc: 0, dic: [0; 16] };
    b.data.extend_from_slice(b"LSCC");
    b.bytes(&COMMENT_START);
    for (i, m) in c.metadata.iter().enumerate() {
        b.data.extend_from_slice(m.as_bytes());
        if i + 1 < c.metadata.len() {
            b.byte(0x00);
        }
    }
    b.bytes(&COMMENT_END);
    b.bytes(&PREAMBLE);
    b.padding(20);
    for _ in 0..2 {
        b.cmd(LSC_RESET_CRC);
        b.crc = 0;
        b.padding(4);
    }
    b.cmd(VERIFY_ID);
    b.u32(c.idcode());
    let s = c.settings;
    let mut ctrl0 = 0u32;
    if s.background {
        ctrl0 |= 0x2780_0000;
    }
    if s.multiboot {
        ctrl0 |= 1 << 19;
    }
    b.cmd(LSC_PROG_CNTRL0);
    b.u32(ctrl0);
    if s.compress {
        b.comp_dic(c);
        b.cmd(LSC_WRITE_COMP_DIC);
        let dic = b.dic;
        b.bytes(&dic);
    }
    let tap = c.tap_frame_count;
    let main = c
        .cram
        .frames
        .checked_sub(32 + tap)
        .ok_or_else(|| err!("{} has fewer frames than IO and TAP need", c.db.device))?;
    let section = |b: &mut W, start: u32, count: usize| -> Result<()> {
        if s.compress { b.comp_frames(c, start, count) } else { b.frames(c, start, count) }
    };
    b.frame_addr(0x8000);
    section(&mut b, 0x8000, 32)?;
    b.padding(17);
    b.cmd(LSC_INIT_ADDRESS);
    section(&mut b, 0x0000, main)?;
    b.padding(17);
    b.frame_addr(0x8020);
    section(&mut b, 0x8020, tap)?;
    b.padding(17);
    b.byte(LSC_POWER_CTRL);
    b.zeros(2);
    b.byte(0x01);
    b.padding(512);
    b.ip_config(c)?;
    b.byte(ISC_PROGRAM_USERCODE);
    b.byte(0x80); // check CRC
    b.zeros(2);
    b.u32(0);
    b.insert_crc();
    b.padding(15);
    b.cmd(ISC_PROGRAM_DONE);
    b.padding(4);
    Ok(b.data)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc16_of_a_known_sequence() {
        // CRC-16/BUYPASS ("UMTS"): poly 0x8005, init 0, no reflection.
        // check("123456789") = 0xFEE8. Ours is the same register run
        // non-augmented then flushed with 16 zero bits, which is equivalent.
        let mut w = W { data: Vec::new(), crc: 0, ecc: 0, dic: [0; 16] };
        w.bytes(b"123456789");
        w.insert_crc();
        let n = w.data.len();
        assert_eq!(&w.data[n - 2..], &[0xFE, 0xE8]);
    }
}
