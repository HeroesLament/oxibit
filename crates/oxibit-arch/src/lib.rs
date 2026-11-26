//! Chip architecture data as flat, index-addressed tables.
//!
//! Names live once in a string table and everything else is integers: tile
//! types hold CSR-style [`model::Groups`] (key, item, fuse) instead of maps of
//! strings. [`codec`] writes and reads the tables as one versioned blob, and
//! [`import`] builds them from an upstream text database.

pub mod codec;
pub mod emit;
pub mod export;
pub mod import;
pub mod lit;
pub mod model;
