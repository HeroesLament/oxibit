//! Place and route for 7-series devices on oxibit's prjxray routing graph.
//!
//! Today: a negotiated-congestion router (`router`), the nets of an already
//! routed design (`nets`), and `reroute`, which routes such a design again
//! on its own placement so the router can be checked against nextpnr.
//! `dsp_patch` re-targets a routed design to new DSP48E1 settings without
//! placing or routing it again.
//! Progress goes through `oxibit_report::Progress`; nothing here prints.

pub mod cellcfg;
pub mod bramcfg;
pub mod clocking;
pub mod delay;
pub mod dsp_patch;
pub mod fasm_out;
pub mod flow;
pub mod iocfg;
pub mod lutinit;
pub mod mmcm;
pub mod nets;
pub mod packed;
pub mod placed;
pub mod reroute;
pub mod router;
