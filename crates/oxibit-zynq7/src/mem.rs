//! Uncached `/dev/mem` mappings.

use std::fs::OpenOptions;
use std::io;
use std::os::unix::fs::OpenOptionsExt;
use std::os::unix::io::AsRawFd;

/// A mapped physical window. Access is volatile, 32-bit, and the caller
/// checks bounds; this type only guarantees the mapping outlives it.
pub struct PhysMap {
    base: *mut u8,
    phys: u64,
    len: usize,
}

// SAFETY: the pointer is an mmap result owned exclusively by this value and
// unmapped only in Drop; a mapping belongs to the process, not a thread.
// Not Sync: concurrent MMIO through shared references is a hardware race.
unsafe impl Send for PhysMap {}

impl PhysMap {
    /// Map `len` bytes at `phys`. Both must be page-aligned.
    ///
    /// `O_SYNC` makes the kernel map the range uncached, which on ARM is also
    /// strongly ordered: writes reach the device (or DDR, for the DMA buffer)
    /// in program order, and reads are never answered from a cache line.
    ///
    /// `mmap64` rather than `mmap`: `off_t` is 32-bit signed on armv7, and
    /// every Zynq peripheral is above 2 GiB. glibc's 32-bit `mmap` would shift
    /// the negative offset arithmetically and map a different page.
    pub fn open(phys: u64, len: usize) -> io::Result<Self> {
        let page = page_size();
        if phys % page as u64 != 0 || len == 0 || len % page != 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("0x{phys:X}+0x{len:X} is not page-aligned ({page} bytes)"),
            ));
        }

        let file = OpenOptions::new().read(true).write(true).custom_flags(libc::O_SYNC).open("/dev/mem")?;

        // SAFETY: valid fd, null hint, page-aligned length and offset.
        let addr = unsafe {
            libc::mmap64(
                std::ptr::null_mut(),
                len,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                file.as_raw_fd(),
                phys as libc::off64_t,
            )
        };
        if addr == libc::MAP_FAILED {
            return Err(io::Error::other(format!(
                "mmap /dev/mem 0x{phys:X}+0x{len:X}: {}",
                io::Error::last_os_error()
            )));
        }
        // The mapping holds its own reference; the fd can go.
        drop(file);

        Ok(PhysMap { base: addr as *mut u8, phys, len })
    }

    pub fn phys(&self) -> u64 {
        self.phys
    }

    pub fn len(&self) -> usize {
        self.len
    }

    /// Volatile read. Panics if out of bounds or misaligned: callers in this
    /// crate pass constants or checked offsets, so that is a bug, not input.
    pub fn r32(&self, off: usize) -> u32 {
        assert!(off % 4 == 0 && off + 4 <= self.len, "PhysMap read 0x{off:X} out of range");
        // SAFETY: in bounds and aligned per the assert; mapping live for &self.
        unsafe { std::ptr::read_volatile(self.base.add(off) as *const u32) }
    }

    pub fn w32(&self, off: usize, v: u32) {
        assert!(off % 4 == 0 && off + 4 <= self.len, "PhysMap write 0x{off:X} out of range");
        // SAFETY: as r32.
        unsafe { std::ptr::write_volatile(self.base.add(off) as *mut u32, v) }
    }
}

impl Drop for PhysMap {
    fn drop(&mut self) {
        // SAFETY: exactly what mmap returned.
        unsafe { libc::munmap(self.base as *mut libc::c_void, self.len) };
    }
}

pub fn page_size() -> usize {
    // SAFETY: sysconf with a valid name.
    let n = unsafe { libc::sysconf(libc::_SC_PAGESIZE) };
    if n > 0 { n as usize } else { 4096 }
}

impl PhysMap {
    /// Write `words` from offset 0, two words per access where possible.
    /// The mapping is page-aligned, so 8-byte steps stay 8-byte aligned.
    pub fn write_words(&self, words: &[u32]) {
        assert!(words.len() * 4 <= self.len(), "write past the end of the mapping");
        let pairs = words.len() / 2;
        for i in 0..pairs {
            let v = (words[2 * i] as u64) | ((words[2 * i + 1] as u64) << 32);
            // SAFETY: in bounds (asserted), 8-byte aligned (page-aligned base).
            unsafe { std::ptr::write_volatile(self.base.add(i * 8) as *mut u64, v) }
        }
        if words.len() % 2 == 1 {
            self.w32((words.len() - 1) * 4, words[words.len() - 1]);
        }
    }

    /// Index of the first word that does not read back as `words`, if any.
    pub fn first_mismatch(&self, words: &[u32]) -> Option<usize> {
        assert!(words.len() * 4 <= self.len(), "read past the end of the mapping");
        let pairs = words.len() / 2;
        for i in 0..pairs {
            // SAFETY: as in write_words.
            let v = unsafe { std::ptr::read_volatile(self.base.add(i * 8) as *const u64) };
            if v as u32 != words[2 * i] {
                return Some(2 * i);
            }
            if (v >> 32) as u32 != words[2 * i + 1] {
                return Some(2 * i + 1);
            }
        }
        if words.len() % 2 == 1 && self.r32((words.len() - 1) * 4) != words[words.len() - 1] {
            return Some(words.len() - 1);
        }
        None
    }
}

/// Complete all outstanding memory accesses before the next instruction.
/// Used between staging the bitstream and starting the DMA that reads it.
pub fn dsb() {
    #[cfg(target_arch = "arm")]
    // SAFETY: a barrier instruction; no memory or register effects.
    unsafe {
        std::arch::asm!("dsb sy", options(nostack, preserves_flags))
    };
    #[cfg(not(target_arch = "arm"))]
    std::sync::atomic::fence(std::sync::atomic::Ordering::SeqCst);
}
