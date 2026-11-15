//! Silicon timing characterization from two-clock phase sweeps.
//!
//! A timing-observatory bitstream launches on one MMCM output and captures on
//! another whose phase is stepped through a full period in 1/8-VCO steps. At
//! each step eight slot counters count captures that disagree with a
//! reference flop. Sweeps at several VCO multipliers interleave their grids.
//!
//! - [`sweep`] reads the runner's sweep files into capture-fraction points.
//! - [`edge`] locates each slot's window and fits its edges below the step
//!   size: an error-function step (a pulse for narrow windows) around each
//!   edge, all sweeps of a design merged.
//! - [`absolute`] turns mirrored forward/reverse windows into path delays
//!   with no timing model in them, `(T + f - r) / 2`, and compares units.
//! - [`taps`] fits the MMCM phase-tap spacing from sweeps with the launch
//!   clock moved tap by tap.
//! - [`calib`] is the per-chip calibration file the measurements go into.
//!
//! Nothing here prints; the `oxibit timing` command formats results.

pub mod absolute;
pub mod calib;
pub mod edge;
mod erf;
pub mod sweep;
pub mod taps;

pub use erf::phi;
