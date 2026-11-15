//! Lattice Nexus bitstreams (LIFCL CrossLink-NX, LFD2NX Certus-NX) from FASM.
//!
//! The device data is prjoxide's database (CC0): `devices.json`, each
//! device's `tilegrid.json` and `baseaddr.json`, and the per tile type and
//! per IP type bit databases in RON. Feature semantics follow prjoxide's
//! `Chip::from_fasm` exactly, in the same order, so that for the same FASM
//! and database the bitstream is byte-identical to `prjoxide pack`. That is
//! the test this crate is held to.
//!
//! Layout of a configuration:
//! - CRAM: `frames` x `bits_per_frame` bits. Each tile owns a window of it
//!   (`start_frame`, `start_bit`, `frames`, `bits`).
//! - IP space: byte-addressed bus writes for EBR and LRAM contents, PLLs,
//!   DPHY and PCIe, located by `baseaddr.json`.
//! - A FASM tile prefix is a tile name, a bel tilegroup name
//!   (`R<y>C<x>_<bel>`, configuration spread over the bel's tiles), or
//!   `IP_<region>`.

pub mod bels;
pub mod bitstream;
pub mod chip;
pub mod db;
pub mod emit;
pub mod fasm;
pub mod flow;
pub mod graph;
pub mod netlist;
pub mod place;
pub mod route;

pub use bitstream::serialise;
pub use chip::{BitMatrix, Chip, Settings, Tile};
pub use db::DeviceDb;
pub use fasm::{FasmTile, ParsedFasm, Value};

use std::fmt;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Error(pub String);

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for Error {}

macro_rules! err {
    ($($arg:tt)*) => { $crate::Error(format!($($arg)*)) };
}
pub(crate) use err;

pub type Result<T> = std::result::Result<T, Error>;
