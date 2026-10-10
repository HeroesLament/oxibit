//! The PL's register window (M_AXI_GP0/GP1).
//!
//! Bounds and alignment handling is carried over from nervezynq's `fabric`
//! crate unchanged: every access is checked before any is issued, so a bad
//! offset is an error, never a stray bus access. What no check here can
//! prevent is an access with no configured slave behind it; that is the
//! guard's job, and the only way to reach this type from outside the crate
//! is `Guard::access`.

use std::io;

use crate::mem::PhysMap;

/// M_AXI_GP0 starts here; M_AXI_GP1 ends at PL_END.
const PL_START: u64 = 0x4000_0000;
const PL_END: u64 = 0xC000_0000;

pub struct ZynqAperture {
    map: PhysMap,
}

impl ZynqAperture {
    /// Mapping issues no bus traffic, so this is safe with the PL blank.
    pub(crate) fn open(base: u64, len: usize) -> io::Result<Self> {
        Ok(ZynqAperture { map: PhysMap::open(base, len)? })
    }

    /// A window onto PS-side memory: DDR, SLCR, DevC, PS peripherals. Refuses
    /// any range overlapping M_AXI_GP0/GP1 (`0x4000_0000..0xC000_0000`), the
    /// only addresses whose slaves live in the PL, so this can never be used to
    /// reach an unconfigured fabric behind the guard's back.
    pub fn open_ps(base: u64, len: usize) -> io::Result<Self> {
        let end = base.checked_add(len as u64).ok_or_else(|| invalid("range overflows".into()))?;
        if base < PL_END && end > PL_START {
            return Err(invalid(format!(
                "0x{base:X}+0x{len:X} overlaps the PL master ports 0x{PL_START:X}..0x{PL_END:X}; \
                 PL access goes through the guard (oxibit port / load / peek)"
            )));
        }
        Self::open(base, len)
    }

    fn check(&self, offset: usize, count: usize) -> io::Result<()> {
        if offset % 4 != 0 {
            return Err(invalid(format!("offset 0x{offset:X} is not 4-byte aligned")));
        }
        let end = count
            .checked_mul(4)
            .and_then(|b| offset.checked_add(b))
            .ok_or_else(|| invalid("span overflows an address".into()))?;
        if end > self.map.len() {
            return Err(invalid(format!(
                "span 0x{offset:X}..0x{end:X} is outside the 0x{:X}-byte window at 0x{:X}",
                self.map.len(),
                self.map.phys()
            )));
        }
        Ok(())
    }
}

fn invalid(msg: String) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidInput, msg)
}

impl oxibit_guard::Aperture for ZynqAperture {
    fn base(&self) -> u64 {
        self.map.phys()
    }

    fn len(&self) -> usize {
        self.map.len()
    }

    fn read32(&self, offset: usize) -> io::Result<u32> {
        self.check(offset, 1)?;
        Ok(self.map.r32(offset))
    }

    fn write32(&self, offset: usize, value: u32) -> io::Result<()> {
        self.check(offset, 1)?;
        self.map.w32(offset, value);
        Ok(())
    }

    fn read_block(&self, offset: usize, count: usize) -> io::Result<Vec<u32>> {
        self.check(offset, count)?;
        Ok((0..count).map(|i| self.map.r32(offset + i * 4)).collect())
    }

    fn read_repeat(&self, offset: usize, count: usize) -> io::Result<Vec<u32>> {
        self.check(offset, 1)?;
        Ok((0..count).map(|_| self.map.r32(offset)).collect())
    }

    fn write_block(&self, offset: usize, values: &[u32]) -> io::Result<()> {
        self.check(offset, values.len())?;
        for (i, v) in values.iter().enumerate() {
            self.map.w32(offset + i * 4, *v);
        }
        Ok(())
    }
}
