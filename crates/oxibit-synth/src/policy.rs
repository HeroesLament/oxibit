//! What a synthesis run is allowed to do: which passes, the capability
//! floor their output must meet, and the evidence floor for passes the
//! equivalence check cannot see.
//!
//! Passes before the checking view are checked by the miter by
//! construction (`stage::RtlPass`, `stage::AigPass`). Passes after it
//! (`stage::PhysicalPass`) must say how they are known to be right; the
//! type allows no "checked by the miter" answer there, because the miter
//! never sees their output.

use crate::caps::Level;

pub use oxibit_policy::{EvidenceLevel, PostCheckEvidence};

/// Optional passes. The core (lower, map, emit) always runs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PassSet {
    pub memports: bool,
    pub onehot: bool,
    pub regbank: bool,
    pub muxpack: bool,
    pub dsp: bool,
    pub balance: bool,
    pub ffmerge: bool,
    pub srl: bool,
    pub bram: bool,
    pub iopad: bool,
}

impl PassSet {
    pub const ALL: PassSet = PassSet {
        memports: true,
        onehot: true,
        regbank: true,
        muxpack: true,
        dsp: true,
        balance: true,
        ffmerge: true,
        srl: true,
        bram: true,
        iopad: true,
    };
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Policy {
    pub passes: PassSet,
    /// Every mode a pass emits must be at least this well established.
    pub floor: Level,
    /// Post-check passes need at least this evidence.
    pub evidence: EvidenceLevel,
    /// Area-recovery passes after the depth-oriented mapping pass.
    pub area_passes: usize,
    /// Adders and comparators at least this wide use carry chains (0: never).
    pub carry_min: usize,
    /// Multipliers needing more DSP48E1s than this stay in fabric.
    pub dsp_max_per_mul: usize,
    /// Sum a two-chunk row of partial products in the DSP column (a
    /// PCOUT -> PCIN pair) rather than in fabric.
    pub dsp_cascade: bool,
    /// Exact local area recovery passes after the area-flow passes.
    pub exact_passes: usize,
    /// LUT levels the area passes may add over the depth-optimal mapping.
    pub depth_slack: u32,
}

impl Policy {
    /// Development: anything oxibit-pnr can place, any tested pass.
    pub fn exploration() -> Self {
        Policy {
            // regbank has no equivalence check yet (the check view holds
            // memories where the source holds registers): opt in.
            passes: PassSet { regbank: false, ..PassSet::ALL },
            floor: Level::PnrSupported,
            evidence: EvidenceLevel::Inspection,
            area_passes: 2,
            carry_min: 3,
            dsp_max_per_mul: 4,
            dsp_cascade: true,
            exact_passes: 0,
            depth_slack: 0,
        }
    }

    /// For bitstreams that go to a board unattended: silicon-proven modes
    /// only, and only passes with shape proofs or co-simulation behind them.
    pub fn production() -> Self {
        Policy {
            floor: Level::SiliconProven,
            evidence: EvidenceLevel::CoSimulation,
            ..Policy::exploration()
        }
    }
}

impl From<&Policy> for crate::map::MapOpts {
    fn from(p: &Policy) -> Self {
        crate::map::MapOpts {
            area_passes: p.area_passes,
            exact_passes: p.exact_passes,
            depth_slack: p.depth_slack,
        }
    }
}

impl Default for Policy {
    fn default() -> Self {
        Policy::exploration()
    }
}
