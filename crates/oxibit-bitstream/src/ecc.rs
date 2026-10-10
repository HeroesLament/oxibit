//! The 13-bit ECC each 7-series configuration frame carries in word 50.
//! Port of prjxray `xc7series::updateECC` / `icap_ecc`.

use crate::FRAME_WORDS;

const ECC_WORD: usize = 0x32;

fn icap_ecc(idx: u32, mut data: u32, mut ecc: u32) -> u32 {
    let mut val = idx * 32;
    if idx > 0x25 {
        val += 0x1360;
    } else if idx > 0x6 {
        val += 0x1340;
    } else {
        val += 0x1320;
    }
    if idx == 0x32 {
        data &= 0xFFFF_E000;
    }
    for i in 0..32 {
        if data & 1 != 0 {
            ecc ^= val + i;
        }
        data >>= 1;
    }
    if idx == 0x64 {
        let mut v = ecc & 0xFFF;
        v ^= v >> 8;
        v ^= v >> 4;
        v ^= v >> 2;
        v ^= v >> 1;
        ecc ^= (v & 1) << 12;
    }
    ecc
}

/// Recompute the ECC bits (word 50, bits 12:0) in place.
pub fn update_ecc(frame: &mut [u32; FRAME_WORDS]) {
    frame[ECC_WORD] &= 0xFFFF_E000;
    let mut ecc = 0;
    for (i, &w) in frame.iter().enumerate() {
        ecc = icap_ecc(i as u32, w, ecc);
    }
    frame[ECC_WORD] |= ecc & 0x1FFF;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_frame_has_zero_ecc() {
        let mut f = [0u32; FRAME_WORDS];
        update_ecc(&mut f);
        assert_eq!(f[ECC_WORD], 0);
    }
}
