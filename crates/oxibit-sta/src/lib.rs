//! Static timing analysis for oxibit's routed designs.
//!
//! - [`sdf`]: cell delays and timing checks from prjxray's SDF files.
//! - [`analyze`]: setup analysis of a routed design, per clock domain.

pub mod analyze;
pub mod sdf;
