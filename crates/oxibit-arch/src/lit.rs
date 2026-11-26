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
