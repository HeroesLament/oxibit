//! Packing and placement for 7-series: from oxibit-synth's primitive
//! netlist to the packed, placed design (`oxibit_pnr::packed::Design`) that
//! oxibit-pnr routes.

pub mod compare;
pub mod device;
pub mod netlist;
pub mod pack;
pub mod place;
pub mod xdc;
