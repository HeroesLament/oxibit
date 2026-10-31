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

/// How a post-check pass is known to be correct, weakest first.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PostCheckEvidence {
    /// Nothing yet. Refused unless the policy allows unproven passes.
    Unproven { why: &'static str },
    /// Unit tests plus a rewrite simple enough to read.
    Inspection { tests: &'static [&'static str] },
    /// Random co-simulation of every shape the pass emits against the
    /// reference semantics (not a proof).
    CoSimulation { harness: &'static str },
    /// Each emitted shape proven equivalent to what it replaces.
    ShapeProof { harness: &'static str },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum EvidenceLevel {
    Unproven,
    Inspection,
    CoSimulation,
    ShapeProof,
}

impl PostCheckEvidence {
    pub fn level(&self) -> EvidenceLevel {
        match self {
            PostCheckEvidence::Unproven { .. } => EvidenceLevel::Unproven,
            PostCheckEvidence::Inspection { .. } => EvidenceLevel::Inspection,
            PostCheckEvidence::CoSimulation { .. } => EvidenceLevel::CoSimulation,
            PostCheckEvidence::ShapeProof { .. } => EvidenceLevel::ShapeProof,
        }
    }

    pub fn describe(&self) -> String {
        match self {
            PostCheckEvidence::Unproven { why } => format!("unproven: {why}"),
            PostCheckEvidence::Inspection { tests } => format!("inspection: {}", tests.join(", ")),
            PostCheckEvidence::CoSimulation { harness } => format!("co-simulation: {harness}"),
            PostCheckEvidence::ShapeProof { harness } => format!("shape proof: {harness}"),
        }
    }
}

/// Optional passes. The core (lower, map, emit) always runs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PassSet {
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
}

impl Policy {
    /// Development: anything oxibit-pnr can place, any tested pass.
    pub fn exploration() -> Self {
        Policy {
            passes: PassSet::ALL,
            floor: Level::PnrSupported,
            evidence: EvidenceLevel::Inspection,
            area_passes: 2,
            carry_min: 3,
            dsp_max_per_mul: 4,
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

impl Default for Policy {
    fn default() -> Self {
        Policy::exploration()
    }
}
