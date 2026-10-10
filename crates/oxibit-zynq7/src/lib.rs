//! Zynq-7000 backend for [`oxibit_guard`].
//!
//! Everything here reaches hardware through `/dev/mem` mappings opened with
//! `O_SYNC` (uncached, strongly ordered):
//!
//! | Region | Physical | Why |
//! |---|---|---|
//! | SLCR + DevC | `0xF800_0000` + 0x8000 | level shifters, AXI resets, clocks, IDCODE; the PCAP |
//! | DMA buffer | a `no-map` reserved-memory node | bitstream staging for the PCAP DMA |
//! | Aperture | e.g. `0x4000_0000` (M_AXI_GP0) | the PL's register space |
//!
//! Linux only: the crate is empty elsewhere so the workspace still builds on
//! a development host.

#[cfg(target_os = "linux")]
mod aperture;
#[cfg(target_os = "linux")]
mod devc;
#[cfg(target_os = "linux")]
pub mod dt;
#[cfg(target_os = "linux")]
mod mem;
#[cfg(target_os = "linux")]
pub mod regs;

#[cfg(target_os = "linux")]
pub use aperture::ZynqAperture;
#[cfg(target_os = "linux")]
pub use devc::{Config, Zynq7};
