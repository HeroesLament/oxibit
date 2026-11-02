//! The vocabulary every oxibit policy is written in: how far a primitive
//! mode is known to work (`Level`), what established it (`Status`), how a
//! pass the equivalence check cannot see is known to be right
//! (`PostCheckEvidence`), and what happened to each pass a policy ran or
//! refused (`PassRecord`).
//!
//! Each stage (oxibit-synth, oxibit-place, later oxibit-pnr) has its own
//! mode enum and capability table; `Caps` and `below_floor` gate any of
//! them the same way.

use std::collections::BTreeMap;
use std::fmt::Debug;

use serde_json::{Value, json};

/// How far a mode is known to work, lowest first.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Level {
    /// The back end cannot place or configure it.
    Unsupported,
    /// The tool can produce it and its behaviour is modeled, nothing more.
    Modeled,
    /// oxibit-pnr places and configures it.
    PnrSupported,
    /// oxibit's FASM for it equals a reference flow's (nextpnr) on a design.
    FasmMatchesReference,
    /// A bitstream using it behaved correctly on a board.
    SiliconProven,
}

impl Level {
    /// The level a policy floor names: `unsupported`, `modeled`, `pnr`,
    /// `fasm`, `silicon`.
    pub fn parse(s: &str) -> Option<Level> {
        Some(match s {
            "unsupported" => Level::Unsupported,
            "modeled" => Level::Modeled,
            "pnr" => Level::PnrSupported,
            "fasm" => Level::FasmMatchesReference,
            "silicon" => Level::SiliconProven,
            _ => return None,
        })
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Status {
    pub level: Level,
    /// What established the level (design, check, board, date).
    pub basis: &'static str,
}

/// A capability table: each mode a stage can produce, and its status.
/// A mode missing from the table is `Unsupported`.
#[derive(Clone, Debug)]
pub struct Caps<M: Ord> {
    pub part: &'static str,
    pub modes: BTreeMap<M, Status>,
}

impl<M: Ord + Copy + Debug> Caps<M> {
    pub fn new(part: &'static str, modes: impl IntoIterator<Item = (M, Level, &'static str)>) -> Self {
        Caps {
            part,
            modes: modes.into_iter().map(|(m, level, basis)| (m, Status { level, basis })).collect(),
        }
    }

    pub fn status(&self, m: M) -> Status {
        self.modes.get(&m).cloned().unwrap_or(Status {
            level: Level::Unsupported,
            basis: "not in the capability table",
        })
    }

    pub fn level(&self, m: M) -> Level {
        self.status(m).level
    }

    /// Modes in `emits` below `floor`, as a refusal reason.
    pub fn below_floor(&self, emits: &[M], floor: Level) -> Option<String> {
        below_floor(emits, |m| self.status(*m), floor)
    }
}

/// Modes in `emits` whose status is below `floor`, as a refusal reason.
pub fn below_floor<M: Debug>(emits: &[M], status: impl Fn(&M) -> Status, floor: Level) -> Option<String> {
    let low: Vec<String> = emits
        .iter()
        .filter_map(|m| {
            let s = status(m);
            (s.level < floor).then(|| format!("{m:?} is {:?} ({})", s.level, s.basis))
        })
        .collect();
    (!low.is_empty()).then(|| format!("below floor {floor:?}: {}", low.join("; ")))
}

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

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Outcome {
    Ran,
    Disabled,
    Refused(String),
}

/// What happened to one pass, for the report.
#[derive(Clone, Debug)]
pub struct PassRecord {
    pub name: &'static str,
    pub stage: &'static str,
    pub evidence: String,
    pub outcome: Outcome,
    pub stats: Value,
}

impl PassRecord {
    pub fn json(&self) -> Value {
        let outcome = match &self.outcome {
            Outcome::Ran => json!("ran"),
            Outcome::Disabled => json!("disabled"),
            Outcome::Refused(why) => json!({"refused": why}),
        };
        json!({"pass": self.name, "stage": self.stage, "evidence": self.evidence, "outcome": outcome, "stats": self.stats})
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
    enum M {
        A,
        B,
    }

    #[test]
    fn missing_mode_is_unsupported() {
        let caps = Caps::new("t", [(M::A, Level::PnrSupported, "a")]);
        assert_eq!(caps.level(M::B), Level::Unsupported);
        assert_eq!(caps.below_floor(&[M::A], Level::PnrSupported), None);
        let why = caps.below_floor(&[M::A, M::B], Level::Modeled).expect("B below");
        assert!(why.contains("B is Unsupported") && !why.contains("A is"));
    }

    #[test]
    fn floor_names() {
        assert_eq!(Level::parse("modeled"), Some(Level::Modeled));
        assert_eq!(Level::parse("silicon"), Some(Level::SiliconProven));
        assert_eq!(Level::parse("x"), None);
    }
}
