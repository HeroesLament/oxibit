//! The synthesis pipeline as typestates, and the pass traits that bind a
//! pass to the stage it may run in.
//!
//! ```text
//! Rtl --RtlPass*--> lower --AigPass*--> map/emit --> Mapped
//! Mapped::split --> (CheckView, Physical) --PhysicalPass*--> netlist
//! ```
//!
//! `CheckView` is what the equivalence check compares against the source;
//! it exists only as the product of `Mapped::split`, which consumes the
//! mapped netlist, so nothing can change the netlist between the check
//! view and the start of the physical passes. A pass that the miter cannot
//! verify can only be written as a `PhysicalPass`, and a `PhysicalPass`
//! must state its own evidence (`policy::PostCheckEvidence`). `Flow` runs
//! every pass through the policy: disabled, refused (a mode below the
//! capability floor or evidence below the evidence floor) or run, and
//! records which.

use oxibit_hdl::netlist::Module;
use oxibit_report::{Event, Progress};
use serde_json::{Value, json};

use crate::caps::{DeviceCaps, Mode};
use crate::lower::Lowered;
use crate::policy::{PassSet, Policy, PostCheckEvidence};

/// Word-level netlist before lowering.
pub struct Rtl(pub(crate) Module);

/// The netlist emit produced, before the checking view is taken.
pub struct Mapped(pub(crate) Module);

/// The netlist the equivalence check sees.
pub struct CheckView(Module);

/// The netlist after the checking view; only `PhysicalPass`es touch it.
pub struct Physical(pub(crate) Module);

impl Rtl {
    pub fn new(m: Module) -> Self {
        Rtl(m)
    }
    pub fn module(&self) -> &Module {
        &self.0
    }
}

impl Mapped {
    pub fn split(self) -> (CheckView, Physical) {
        (CheckView(self.0.clone()), Physical(self.0))
    }
}

impl CheckView {
    pub fn module(&self) -> &Module {
        &self.0
    }
    pub fn into_module(self) -> Module {
        self.0
    }
}

impl Physical {
    pub fn module(&self) -> &Module {
        &self.0
    }
    pub fn into_module(self) -> Module {
        self.0
    }
}

/// Common to every optional pass.
pub trait Pass {
    const NAME: &'static str;
    /// Primitive modes the pass can add to the netlist.
    const EMITS: &'static [Mode];
    type Stats: Default;
    fn enabled(p: &PassSet) -> bool;
    fn json(s: &Self::Stats) -> Value;
}

/// Word-level pass before lowering; the miter checks its result.
pub trait RtlPass: Pass {
    fn run(&self, m: &mut Rtl) -> Self::Stats;
}

/// AIG pass between lowering and mapping; the miter checks its result.
pub trait AigPass: Pass {
    fn run(&self, lw: &mut Lowered) -> Self::Stats;
}

/// Pass after the checking view; the miter never sees its output.
pub trait PhysicalPass: Pass {
    const EVIDENCE: PostCheckEvidence;
    fn run(&self, m: &mut Physical) -> Self::Stats;
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

/// Runs passes under a policy and records the result of each.
pub struct Flow<'a> {
    pub caps: &'a DeviceCaps,
    pub policy: &'a Policy,
    pub rep: &'a dyn Progress,
    pub records: Vec<PassRecord>,
}

impl<'a> Flow<'a> {
    pub fn new(caps: &'a DeviceCaps, policy: &'a Policy, rep: &'a dyn Progress) -> Self {
        Flow {
            caps,
            policy,
            rep,
            records: Vec::new(),
        }
    }

    /// Modes in `emits` below the capability floor, as a refusal reason.
    pub fn below_floor(&self, emits: &[Mode]) -> Option<String> {
        let low: Vec<String> = emits
            .iter()
            .filter(|m| self.caps.level(**m) < self.policy.floor)
            .map(|m| {
                let s = self.caps.status(*m);
                format!("{m:?} is {:?} ({})", s.level, s.basis)
            })
            .collect();
        (!low.is_empty()).then(|| format!("below floor {:?}: {}", self.policy.floor, low.join("; ")))
    }

    fn gate<P: Pass>(&mut self, stage: &'static str, evidence: String, extra: Option<String>) -> bool {
        let outcome = if !P::enabled(&self.policy.passes) {
            Outcome::Disabled
        } else if let Some(why) = self.below_floor(P::EMITS).or(extra) {
            self.rep.emit(Event::Warn {
                code: "pass_refused".into(),
                msg: format!("{}: {why}", P::NAME),
            });
            Outcome::Refused(why)
        } else {
            return true;
        };
        self.records.push(PassRecord {
            name: P::NAME,
            stage,
            evidence,
            outcome,
            stats: Value::Null,
        });
        false
    }

    fn ran<P: Pass>(&mut self, stage: &'static str, evidence: String, s: &P::Stats) {
        let stats = P::json(s);
        self.rep.emit(Event::progress(P::NAME, stats.clone()));
        self.records.push(PassRecord {
            name: P::NAME,
            stage,
            evidence,
            outcome: Outcome::Ran,
            stats,
        });
    }

    pub fn rtl<P: RtlPass>(&mut self, p: &P, m: &mut Rtl) -> P::Stats {
        let ev = "miter".to_string();
        if !self.gate::<P>("rtl", ev.clone(), None) {
            return P::Stats::default();
        }
        let s = p.run(m);
        self.ran::<P>("rtl", ev, &s);
        s
    }

    pub fn aig<P: AigPass>(&mut self, p: &P, lw: &mut Lowered) -> P::Stats {
        let ev = "miter".to_string();
        if !self.gate::<P>("aig", ev.clone(), None) {
            return P::Stats::default();
        }
        let s = p.run(lw);
        self.ran::<P>("aig", ev, &s);
        s
    }

    pub fn physical<P: PhysicalPass>(&mut self, p: &P, m: &mut Physical) -> P::Stats {
        let ev = P::EVIDENCE.describe();
        let weak = (P::EVIDENCE.level() < self.policy.evidence)
            .then(|| format!("evidence {:?} below floor {:?}", P::EVIDENCE.level(), self.policy.evidence));
        if !self.gate::<P>("physical", ev.clone(), weak) {
            return P::Stats::default();
        }
        let s = p.run(m);
        self.ran::<P>("physical", ev, &s);
        s
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::caps::Level;
    use crate::policy::EvidenceLevel;

    struct Wide;
    impl Pass for Wide {
        const NAME: &'static str = "wide";
        const EMITS: &'static [Mode] = &[Mode::MuxF7];
        type Stats = usize;
        fn enabled(_: &PassSet) -> bool {
            true
        }
        fn json(s: &usize) -> Value {
            json!(s)
        }
    }
    impl PhysicalPass for Wide {
        const EVIDENCE: PostCheckEvidence = PostCheckEvidence::ShapeProof { harness: "none" };
        fn run(&self, _: &mut Physical) -> usize {
            1
        }
    }

    struct Loose;
    impl Pass for Loose {
        const NAME: &'static str = "loose";
        const EMITS: &'static [Mode] = &[];
        type Stats = usize;
        fn enabled(_: &PassSet) -> bool {
            true
        }
        fn json(s: &usize) -> Value {
            json!(s)
        }
    }
    impl PhysicalPass for Loose {
        const EVIDENCE: PostCheckEvidence = PostCheckEvidence::Unproven { why: "test" };
        fn run(&self, _: &mut Physical) -> usize {
            1
        }
    }

    fn phys() -> Physical {
        Mapped(Module::new("t")).split().1
    }

    #[test]
    fn unsupported_mode_is_refused() {
        let caps = DeviceCaps::xc7z020();
        let policy = Policy::exploration();
        let mut flow = Flow::new(&caps, &policy, &oxibit_report::Silent);
        assert_eq!(flow.physical(&Wide, &mut phys()), 0);
        assert!(matches!(&flow.records[0].outcome, Outcome::Refused(w) if w.contains("MuxF7")));
    }

    #[test]
    fn evidence_floor() {
        let caps = DeviceCaps::xc7z020();
        let mut policy = Policy::exploration();
        let mut flow = Flow::new(&caps, &policy, &oxibit_report::Silent);
        assert_eq!(flow.physical(&Loose, &mut phys()), 0);
        assert!(matches!(&flow.records[0].outcome, Outcome::Refused(w) if w.contains("evidence")));
        policy.evidence = EvidenceLevel::Unproven;
        let mut flow = Flow::new(&caps, &policy, &oxibit_report::Silent);
        assert_eq!(flow.physical(&Loose, &mut phys()), 1);
        assert_eq!(flow.records[0].outcome, Outcome::Ran);
    }

    #[test]
    fn floor_follows_promotion() {
        let mut caps = DeviceCaps::xc7z020();
        let policy = Policy::exploration();
        caps.modes.insert(
            Mode::MuxF7,
            crate::caps::Status {
                level: Level::PnrSupported,
                basis: "test",
            },
        );
        let mut flow = Flow::new(&caps, &policy, &oxibit_report::Silent);
        assert_eq!(flow.physical(&Wide, &mut phys()), 1);
    }
}
