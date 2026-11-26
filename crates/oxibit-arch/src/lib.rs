//! Chip architecture data as flat, index-addressed tables.
//!
//! Names live once in a string table and everything else is integers: tile
//! types hold CSR-style [`model::Groups`] (key, item, fuse) instead of maps of
//! strings. [`codec`] writes and reads the tables as one versioned blob.
//! [`lit`] is the same data as plain Rust statics, the form checked in per
//! family, and [`emit`] writes a tile type in that form.
//!
//! This crate has no dependencies. Importing from an upstream database is
//! `oxibit-arch-gen`'s job.

pub mod codec;
pub mod emit;
pub mod lit;
pub mod model;
