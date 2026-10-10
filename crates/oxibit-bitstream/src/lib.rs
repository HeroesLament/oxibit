//! Xilinx 7-series configuration bitstreams.
//!
//! A bitstream is a sequence of 32-bit configuration words. The same words
//! travel in three containers, and confusing them is the classic silent
//! failure: the device configures nothing and nothing says why.
//!
//! | Container | Header | Word byte order | Consumers |
//! |---|---|---|---|
//! | [`Container::Bit`] | TLV (design, part, date, time) | big-endian | Vivado, JTAG tools, `xc7frames2bit` |
//! | [`Container::RawBin`] | none | big-endian | `write_bitstream -bin_file`, SPI flash images |
//! | [`Container::SwappedBin`] | none | byte-swapped per word | Zynq-7000 PCAP, Linux `fpga_manager` |
//!
//! [`Bitstream`] holds the words once, in canonical order, whatever they were
//! read from. [`packet`] walks them as configuration packets, [`crc`]
//! re-computes the device's CRC, [`part`] maps IDCODEs to dies, and
//! [`analysis`] puts it together into a [`analysis::Report`].

pub mod analysis;
pub mod container;
pub mod crc;
pub mod ecc;
pub mod layout;
pub mod packet;
pub mod part;
pub mod writer;
pub mod zynq_boot;

pub use analysis::{Report, analyze, analyze_with};
pub use container::{BitHeader, Bitstream, Container};

/// The synchronisation word. Everything before it is padding and bus-width
/// detection; packets start immediately after it.
pub const SYNC: u32 = 0xAA99_5566;

/// Words per configuration frame on 7-series devices.
pub const FRAME_WORDS: usize = 101;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Error {
    /// The `.bit` TLV header is malformed or truncated.
    BitHeader(String),
    /// Configuration data is not a whole number of 32-bit words.
    Unaligned { bytes: usize },
    /// No sync word in either byte order.
    NoSync,
    /// A packet header claims more words than remain.
    Truncated { index: usize, header: u32, need: usize, have: usize },
    /// A header word whose type field is neither 1 nor 2 while synced.
    BadPacketType { index: usize, header: u32 },
    /// A type 2 packet with no preceding type 1 to supply its register.
    OrphanType2 { index: usize, header: u32 },
    /// A Zynq boot image that cannot be read.
    BootImage(String),
    /// The data is a Zynq boot image, not a bare bitstream.
    IsBootImage,
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::BitHeader(m) => write!(f, "bad .bit header: {m}"),
            Error::Unaligned { bytes } => {
                write!(f, "configuration data is {bytes} bytes, not a multiple of 4")
            }
            Error::NoSync => write!(f, "no sync word (AA995566) in either byte order"),
            Error::Truncated { index, header, need, have } => write!(
                f,
                "packet 0x{header:08X} at word {index} needs {need} words, only {have} remain"
            ),
            Error::BadPacketType { index, header } => {
                write!(f, "word {index} (0x{header:08X}) is not a type 1 or type 2 packet header")
            }
            Error::OrphanType2 { index, header } => {
                write!(f, "type 2 packet 0x{header:08X} at word {index} has no preceding type 1")
            }
            Error::BootImage(m) => write!(f, "bad Zynq boot image: {m}"),
            Error::IsBootImage => {
                write!(f, "this is a Zynq boot image; read its PL partition with zynq_boot")
            }
        }
    }
}

impl std::error::Error for Error {}
