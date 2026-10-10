//! Facts read from the live device tree (`/proc/device-tree`).

use std::fs;
use std::path::{Path, PathBuf};

const DT: &str = "/proc/device-tree";

/// A physically contiguous, `no-map` reserved-memory region.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Region {
    pub base: u64,
    pub size: usize,
}

/// The PCAP DMA buffer: the first `no-map` child of `/reserved-memory` whose
/// node name starts with `pcap` (nerves_system_libresdr declares
/// `pcap-readback@3fc00000`, 4 MiB).
///
/// `no-map` keeps the kernel from ever creating a cacheable mapping of it, so
/// what the CPU writes through an uncached `/dev/mem` mapping is what the
/// DevC DMA reads from DDR. That is the whole reason to use it: the DMA does
/// not snoop the caches, and userspace on Cortex-A9 cannot clean them to the
/// point of coherency.
pub fn pcap_buffer() -> Option<Region> {
    let dir = Path::new(DT).join("reserved-memory");
    let mut names: Vec<PathBuf> = fs::read_dir(&dir).ok()?.filter_map(|e| e.ok().map(|e| e.path())).collect();
    names.sort();

    names.into_iter().find_map(|node| {
        let name = node.file_name()?.to_str()?;
        if !name.starts_with("pcap") || !node.join("no-map").exists() {
            return None;
        }
        let reg = fs::read(node.join("reg")).ok()?;
        match reg.len() {
            8 => Some(Region { base: be32(&reg[0..4]) as u64, size: be32(&reg[4..8]) as usize }),
            16 => Some(Region {
                base: (be32(&reg[0..4]) as u64) << 32 | be32(&reg[4..8]) as u64,
                size: ((be32(&reg[8..12]) as u64) << 32 | be32(&reg[12..16]) as u64) as usize,
            }),
            _ => None,
        }
    })
}

/// `ps-clk-frequency` from the SLCR clock controller node, wherever the board
/// DT placed it. The PS reference is 33.333 MHz on most Zynq boards and
/// 50 MHz on the LibreSDR, so it is read, never assumed.
pub fn ps_clk_hz() -> Option<u32> {
    find(Path::new(DT), "ps-clk-frequency", 4).and_then(|p| fs::read(p).ok()).filter(|b| b.len() == 4).map(|b| be32(&b))
}

fn find(dir: &Path, file: &str, depth: usize) -> Option<PathBuf> {
    let candidate = dir.join(file);
    if candidate.is_file() {
        return Some(candidate);
    }
    if depth == 0 {
        return None;
    }
    let mut subdirs: Vec<PathBuf> =
        fs::read_dir(dir).ok()?.filter_map(|e| e.ok()).map(|e| e.path()).filter(|p| p.is_dir()).collect();
    subdirs.sort();
    subdirs.iter().find_map(|d| find(d, file, depth - 1))
}

fn be32(b: &[u8]) -> u32 {
    u32::from_be_bytes([b[0], b[1], b[2], b[3]])
}
