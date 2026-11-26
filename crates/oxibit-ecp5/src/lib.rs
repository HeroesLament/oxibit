//! Lattice ECP5 (LFE5U, LFE5UM, LFE5UM5G) bitstreams from Trellis text
//! configurations.
//!
//! The device data is Project Trellis's database: `devices.json`, each
//! device's `tilegrid.json`, and the per tile type bit databases
//! (`<family>/tiledata/<type>/bits.db`). Configuration semantics follow
//! libtrellis's `ChipConfig::to_chip` and `TileBitDatabase::config_to_tile_cram`
//! exactly, in the same order, and the serialiser follows
//! `Bitstream::serialise_chip`, so that for the same `.config` and
//! database the bitstream is byte-identical to `ecppack`. That is the test
//! this crate is held to (`examples/ecp5_pack.rs`, `fuzz.sh`).
//!
//! Layout of a configuration:
//! - CRAM: `frames` x `bits_per_frame` bits, one matrix for the whole chip.
//!   Each tile is a window onto it (`start_frame`, `start_bit`, `cols`
//!   frames, `rows` bits); windows are views, so a later tile's write to a
//!   shared bit wins, as in libtrellis.
//! - EBR initialisation: per block, 2048 9-bit words, written after the
//!   CRAM with `LSC_EBR_ADDRESS` / `LSC_EBR_WRITE`.
//! - A tile bit is `F<frame>B<bit>` relative to its window, `!` inverted.

pub mod bels;
pub mod bitstream;
pub mod chip;
pub mod clk;
pub mod clock;
pub mod config;
pub mod db;
pub mod dqs;
pub mod dsp;
pub mod ebr;
pub mod pll;
pub mod soft;
pub mod sys;
pub mod flow;
pub mod graph;
pub mod io;
pub mod iol;
pub mod lpf;
pub mod netlist;
pub mod pack;
pub mod pads;
pub mod route;
pub mod sim;
pub mod source;
pub mod synth;
pub mod unpack;

pub use bels::{Bels, SliceMode};
pub use bitstream::{Options, serialise};
pub use chip::{BitMatrix, Chip, Level};
pub use config::{ChipConfig, TileConfig};
pub use db::{DeviceDb, DeviceInfo, TileBits, TileInfo};
pub use graph::Graph;
pub use source::Source;
pub use unpack::{BitFile, deserialise, unpack, unpack_from};

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
