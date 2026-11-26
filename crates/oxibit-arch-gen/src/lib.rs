//! Imports upstream chip databases into `oxibit-arch` tables and generates the
//! checked-in family crates from them.
//!
//! Today the Trellis tile files are read through `oxibit-ecp5`'s parser, so
//! regenerating needs that crate to build. A parser of its own here removes
//! that.

pub mod export;
pub mod import;
