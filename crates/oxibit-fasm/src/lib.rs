//! FASM -> 7-series configuration frames -> full bitstream, against a
//! prjxray database, with output identical to prjxray's
//! `fasm2frames` + `xc7frames2bit` (configuration data word for word; only
//! the `.bit` header's free-text records differ).
//!
//! ```no_run
//! # use std::path::Path;
//! let db = oxibit_fasm::db::Database::open(Path::new("prjxray-db/zynq7"), "xc7z020clg400-1").unwrap();
//! let words = oxibit_fasm::build(&db, &std::fs::read_to_string("top.fasm").unwrap()).unwrap();
//! ```

pub mod assemble;
pub mod db;
pub mod disassemble;
pub mod parse;
pub mod route;

pub use assemble::{Error, assemble, assemble_named, to_frm};

/// FASM text -> configuration words (preamble through the trailing NOPs).
pub fn build(db: &db::Database, fasm: &str) -> Result<Vec<u32>, Error> {
    let frames = assemble(db, fasm)?;
    Ok(oxibit_bitstream::writer::full_configuration(&frames, &db.layout))
}
