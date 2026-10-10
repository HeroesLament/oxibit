//! The configuration CRC the device computes over register writes.
//!
//! Algorithm transcribed from prjxray,
//! `lib/include/prjxray/xilinx/xc7series/crc.h` (`icap_crc`): CRC-32C
//! (Castagnoli, reflected polynomial 0x82F63B78) shifted over 37 bits per
//! write, the 32-bit data word followed by the 5-bit register address.
//!
//! That is an ordinary reflected CRC-32C over a 37-bit message sent LSB
//! first, so the 32 data bits go through a byte table (four lookups) and
//! only the 5 address bits are shifted one at a time. The bit-serial
//! transcription is kept as [`icap_crc_bitwise`] and the tests hold the two
//! equal; the table version is what makes checking a 4 MB bitstream cheap on
//! a Cortex-A9.

/// CRC-32C, reflected.
pub const CASTAGNOLI: u32 = 0x82F6_3B78;

const TABLE: [u32; 256] = {
    let mut t = [0u32; 256];
    let mut i = 0;
    while i < 256 {
        let mut c = i as u32;
        let mut k = 0;
        while k < 8 {
            c = if c & 1 != 0 { (c >> 1) ^ CASTAGNOLI } else { c >> 1 };
            k += 1;
        }
        t[i] = c;
        i += 1;
    }
    t
};

/// Extend `prev` with one (register address, data word) write.
#[inline]
pub fn icap_crc(addr: u32, data: u32, prev: u32) -> u32 {
    let mut crc = prev;
    let mut d = data;
    for _ in 0..4 {
        crc = TABLE[((crc ^ d) & 0xFF) as usize] ^ (crc >> 8);
        d >>= 8;
    }
    let mut a = addr & 0x1F;
    for _ in 0..5 {
        crc = if (crc ^ a) & 1 != 0 { (crc >> 1) ^ CASTAGNOLI } else { crc >> 1 };
        a >>= 1;
    }
    crc
}

/// prjxray's bit-serial form, verbatim. Reference for the tests.
pub fn icap_crc_bitwise(addr: u32, data: u32, prev: u32) -> u32 {
    const ADDRESS_BITS: u32 = 5;
    const DATA_BITS: u32 = 32;

    let poly = (CASTAGNOLI as u64) << 1;
    let mut val = (((addr & 0x1F) as u64) << 32) | data as u64;
    let mut crc = prev as u64;

    for _ in 0..(ADDRESS_BITS + DATA_BITS) {
        if (val & 1) != (crc & 1) {
            crc ^= poly;
        }
        val >>= 1;
        crc >>= 1;
    }
    crc as u32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn table_matches_bitwise() {
        // xorshift: deterministic, covers all addresses and many data/crc values.
        let mut s: u64 = 0x9E37_79B9_7F4A_7C15;
        let mut next = || {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            s
        };
        for _ in 0..200_000 {
            let r = next();
            let (addr, data, prev) = ((r >> 59) as u32, r as u32, (r >> 27) as u32);
            assert_eq!(icap_crc(addr, data, prev), icap_crc_bitwise(addr, data, prev), "{addr} {data:#x} {prev:#x}");
        }
        for addr in 0..32 {
            for &data in &[0, 1, 0x8000_0000, 0xFFFF_FFFF, 0xAA99_5566] {
                assert_eq!(icap_crc(addr, data, 0), icap_crc_bitwise(addr, data, 0));
            }
        }
    }
}
