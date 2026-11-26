//! Literal, source-code form of a tile type.
//!
//! Generated files (see `emit`) build one `TileTypeLit` as a `static`, so a
//! tile type can be read, diffed and reviewed as Rust. The flat tables in
//! `model` are what ships; this form is for people.

/// One configuration bit: frame, bit, and whether it is set by being cleared.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Bit {
    pub frame: u16,
    pub bit: u16,
    pub inverted: bool,
}

/// A bit set by being set.
#[must_use]
pub const fn f(frame: u16, bit: u16) -> Bit {
    Bit { frame, bit, inverted: false }
}

/// A bit set by being cleared (`!F<frame>B<bit>`).
#[must_use]
pub const fn nf(frame: u16, bit: u16) -> Bit {
    Bit { frame, bit, inverted: true }
}

/// One way into a mux: the source wire and the bits that select it.
#[derive(Debug)]
pub struct Arc {
    pub source: &'static str,
    pub bits: &'static [Bit],
}

/// A routing mux: its sink wire and the arcs that can drive it.
#[derive(Debug)]
pub struct MuxLit {
    pub sink: &'static str,
    pub arcs: &'static [Arc],
}

/// A setting with named options.
#[derive(Debug)]
pub struct EnumLit {
    pub name: &'static str,
    pub default: Option<&'static str>,
    pub options: &'static [(&'static str, &'static [Bit])],
}

/// A multi-bit setting; `bits[i]` holds bit `i` of the value.
#[derive(Debug)]
pub struct WordLit {
    pub name: &'static str,
    pub default: &'static [bool],
    pub bits: &'static [&'static [Bit]],
}

/// Everything the database knows about one tile type.
#[derive(Debug)]
pub struct TileTypeLit {
    pub name: &'static str,
    pub muxes: &'static [MuxLit],
    pub enums: &'static [EnumLit],
    pub words: &'static [WordLit],
    /// `(sink, source)` wires joined with no configuration bit.
    pub fixed: &'static [(&'static str, &'static str)],
}

/// A tile of a device: its type and its window of configuration memory.
#[derive(Debug)]
pub struct TileLit {
    pub name: &'static str,
    pub ty: &'static TileTypeLit,
    pub start_frame: u32,
    pub start_bit: u32,
    pub frames: u32,
    pub bits: u32,
}

/// A clock quadrant: its tile rectangle.
#[derive(Debug)]
pub struct QuadrantLit {
    pub name: &'static str,
    pub x0: u32,
    pub y0: u32,
    pub x1: u32,
    pub y1: u32,
}

/// A clock tap: the columns it feeds on each side.
#[derive(Debug)]
pub struct TapLit {
    pub name: &'static str,
    pub lx0: u32,
    pub lx1: u32,
    pub rx0: u32,
    pub rx1: u32,
}

/// A clock spine's tile.
#[derive(Debug)]
pub struct SpineLit {
    pub name: &'static str,
    pub x: u32,
    pub y: u32,
}

/// A package pin and the pad it reaches: tile column and row, and which PIO
/// (`A`, `B`, ...) of that tile.
#[derive(Debug)]
pub struct PinLit {
    pub name: &'static str,
    pub col: u32,
    pub row: u32,
    pub pio: &'static str,
}

#[derive(Debug)]
pub struct PackageLit {
    pub name: &'static str,
    pub pins: &'static [PinLit],
}

/// Bank, DQS group and special function of a PIO.
#[derive(Debug)]
pub struct PioMetaLit {
    pub bank: u32,
    pub col: u32,
    pub row: u32,
    pub pio: &'static str,
    pub dqs: Option<&'static str>,
    pub function: Option<&'static str>,
}

/// One device: frame geometry, clock landmarks, pad data and its tiles, which
/// are sorted by name.
#[derive(Debug)]
pub struct DeviceLit {
    pub name: &'static str,
    pub family: &'static str,
    pub idcode: u32,
    pub frames: u32,
    pub bits_per_frame: u32,
    pub pad_bits_after_frame: u32,
    pub pad_bits_before_frame: u32,
    pub max_row: u32,
    pub max_col: u32,
    pub quadrants: &'static [QuadrantLit],
    pub taps: &'static [TapLit],
    pub spines: &'static [SpineLit],
    pub packages: &'static [PackageLit],
    pub pio_meta: &'static [PioMetaLit],
    pub tiles: &'static [TileLit],
}

impl DeviceLit {
    /// The tile named `name`.
    #[must_use]
    pub fn tile(&self, name: &str) -> Option<&'static TileLit> {
        let tiles = self.tiles;
        tiles.binary_search_by(|t| t.name.cmp(name)).ok().map(|i| &tiles[i])
    }
}
