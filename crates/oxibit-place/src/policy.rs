//! What a packing and placement run is allowed to do.
//!
//! Each packing strategy that changes what ends up in a slice states the
//! slice configurations it produces (`PackMode`); `PlaceCaps` records how
//! far each is known to work on the part, and `PackPolicy::plan` checks
//! every chosen strategy against the policy's floor before anything is
//! packed. The core carry-chain shape below the floor fails the run; an
//! optional strategy below it is refused, falls back to the shape the
//! core already uses, and the refusal is recorded. The packer and the
//! slice fill take a `PackPlan`, which only `plan` makes, so nothing
//! reaches them unchecked.

use std::collections::BTreeMap;

pub use oxibit_policy::{Caps, Level, Outcome, PassRecord, Status};
use serde_json::{Value, json};

use crate::placer::{FillOptions, PinCost};

/// A slice configuration packing can produce.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum PackMode {
    /// Carry position with S from its 6LUT and DI from its 5LUT (CY0 = O5),
    /// a LUT1 feed-through where no LUT can sit there.
    CarryDiLut5,
    /// Carry DI from the letter's bypass pin, AX..DX (CY0 = AX..DX); the S
    /// LUT may then read six inputs.
    CarryDiBypass,
    /// Two LUTs on one letter: O6 and O5, at most five distinct inputs, A6
    /// tied high.
    FracturedLetter,
    /// A flop on a carry output's own letter (FFMUX = XOR) while the output
    /// also leaves the slice (OUTMUX = XOR).
    CarryOutFfShared,
}

pub type PlaceCaps = Caps<PackMode>;

/// xc7z020 as recorded.
pub fn xc7z020() -> PlaceCaps {
    use Level::*;
    Caps::new(
        "xc7z020clg400-1",
        [
            (
                PackMode::CarryDiLut5,
                FasmMatchesReference,
                "fasm_equiv vs nextpnr routes of libresdr_radio, tx (nextpnr packs DI on the 5LUT)",
            ),
            (
                PackMode::FracturedLetter,
                PnrSupported,
                "place::legalise shares A1..A5, ties A6; PHY import equals nextpnr's packing",
            ),
            (
                PackMode::CarryDiBypass,
                Modeled,
                "placed.rs leaves ACY0 at AX, placer counts the X pin; never routed on a design",
            ),
            (
                PackMode::CarryOutFfShared,
                PnrSupported,
                "FFMUX and OUTMUX are separate site pips from CARRY4_XOR in oxibit-pnr; the pair not yet compared with a reference FASM",
            ),
        ],
    )
}

/// Where a carry position's DI comes from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CarryDi {
    /// From the 5LUT: the DI LUT when it shares the S LUT's five inputs,
    /// else a feed-through LUT1 (nextpnr's packing).
    Lut5,
    /// From the 5LUT when the DI LUT shares the S LUT's inputs, else the
    /// bypass pin, unless CYINIT already holds that pin.
    Bypass,
}

impl CarryDi {
    pub fn emits(self) -> &'static [PackMode] {
        match self {
            CarryDi::Lut5 => &[PackMode::CarryDiLut5],
            CarryDi::Bypass => &[PackMode::CarryDiLut5, PackMode::CarryDiBypass],
        }
    }

    pub fn parse(s: &str) -> Option<CarryDi> {
        match s {
            "lut5" => Some(CarryDi::Lut5),
            "bypass" => Some(CarryDi::Bypass),
            _ => None,
        }
    }
}

/// Which flops fed by a carry output go on that output's letter.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CarryFf {
    /// Only a flop that is the output's sole user (nextpnr's packing).
    SoleUser,
    /// Also when the output has other users: it leaves the slice through
    /// the letter's output mux as well.
    Shared,
}

impl CarryFf {
    pub fn emits(self) -> &'static [PackMode] {
        match self {
            CarryFf::SoleUser => &[],
            CarryFf::Shared => &[PackMode::CarryOutFfShared],
        }
    }
}

/// Where the slice fill may put a lone LUT on a 5LUT. Each produces
/// `PackMode::FracturedLetter`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Lut5Fill {
    /// Beside a carry chain's S LUT, when the letter's MUX pin is free.
    pub chain: bool,
    /// Beside a LUT that feeds its letter's flop.
    pub ff_letter: bool,
    /// Beside another lone LUT.
    pub pairs: bool,
}

impl Lut5Fill {
    pub const ALL: Lut5Fill = Lut5Fill {
        chain: true,
        ff_letter: true,
        pairs: true,
    };
    pub const NONE: Lut5Fill = Lut5Fill {
        chain: false,
        ff_letter: false,
        pairs: false,
    };
}

#[derive(Clone, Debug, PartialEq)]
pub struct PackPolicy {
    /// Every mode a strategy produces must be at least this well established.
    pub floor: Level,
    pub carry_di: CarryDi,
    pub carry_ff: CarryFf,
    pub lut5: Lut5Fill,
    /// Constant folding nextpnr-xilinx does not do (off for a like-for-like
    /// comparison with nextpnr's packing).
    pub fold: bool,
    /// See `FillOptions::bypass_ffs_per_slice`.
    pub bypass_ffs_per_slice: usize,
    /// What the annealer pays for besides wirelength.
    pub objective: PlaceObjective,
}

/// Placement cost terms beside wirelength. They change where cells go,
/// never what a slice holds, so they need no capability level.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlaceObjective {
    /// All input pins per CLB tile (56).
    pub pins: PinCost,
    /// Bypass pins per CLB tile (AX..DX of both slices, 8).
    pub bypass: PinCost,
}

impl PlaceObjective {
    /// Wirelength only.
    pub const WIRELENGTH: PlaceObjective = PlaceObjective {
        pins: PinCost::OFF_PINS,
        bypass: PinCost::OFF_BYPASS,
    };
}

impl PackPolicy {
    /// Development: anything oxibit-pnr can place.
    pub fn exploration() -> Self {
        PackPolicy {
            floor: Level::PnrSupported,
            carry_di: CarryDi::Lut5,
            carry_ff: CarryFf::Shared,
            lut5: Lut5Fill::ALL,
            fold: true,
            bypass_ffs_per_slice: FillOptions::default().bypass_ffs_per_slice,
            objective: PlaceObjective::WIRELENGTH,
        }
    }

    /// For bitstreams that go to a board unattended.
    pub fn production() -> Self {
        PackPolicy {
            floor: Level::SiliconProven,
            ..PackPolicy::exploration()
        }
    }

    /// nextpnr-xilinx's packing, for `compare` against its output.
    pub fn nextpnr_reference() -> Self {
        PackPolicy {
            fold: false,
            carry_ff: CarryFf::SoleUser,
            ..PackPolicy::exploration()
        }
    }

    /// Check every strategy against `caps`: the core carry shape below the
    /// floor is an error, an optional strategy below it is refused and
    /// falls back.
    pub fn plan(&self, caps: &PlaceCaps) -> Result<PackPlan, String> {
        if let Some(why) = caps.below_floor(CarryDi::Lut5.emits(), self.floor) {
            return Err(format!("carry chains: {why}"));
        }
        let mut records = Vec::new();
        let mut gate = |name: &'static str, stage: &'static str, on: bool, emits: &[PackMode]| -> bool {
            let evidence = emits.iter().map(|m| format!("{m:?}: {}", caps.status(*m).basis)).collect::<Vec<_>>().join("; ");
            let outcome = if !on {
                Outcome::Disabled
            } else if let Some(why) = caps.below_floor(emits, self.floor) {
                Outcome::Refused(why)
            } else {
                Outcome::Ran
            };
            let ran = outcome == Outcome::Ran;
            records.push(PassRecord {
                name,
                stage,
                evidence,
                outcome,
                stats: Value::Null,
            });
            ran
        };
        let bypass = gate("carry_di_bypass", "pack", self.carry_di == CarryDi::Bypass, CarryDi::Bypass.emits());
        let carry_ff = gate("carry_ff_shared", "pack", self.carry_ff == CarryFf::Shared, CarryFf::Shared.emits());
        let frac = [PackMode::FracturedLetter];
        let lut5 = Lut5Fill {
            chain: gate("lut5_chain", "fill", self.lut5.chain, &frac),
            ff_letter: gate("lut5_ff_letter", "fill", self.lut5.ff_letter, &frac),
            pairs: gate("lut5_pairs", "fill", self.lut5.pairs, &frac),
        };
        Ok(PackPlan {
            carry_di: if bypass { CarryDi::Bypass } else { CarryDi::Lut5 },
            carry_ff: if carry_ff { CarryFf::Shared } else { CarryFf::SoleUser },
            fold: self.fold,
            fill: FillOptions {
                bypass_ffs_per_slice: self.bypass_ffs_per_slice,
                lut5,
            },
            objective: self.objective,
            records,
        })
    }
}

impl Default for PackPolicy {
    fn default() -> Self {
        PackPolicy::exploration()
    }
}

/// A policy checked against the part: what packing and the slice fill do.
#[derive(Clone, Debug)]
pub struct PackPlan {
    carry_di: CarryDi,
    carry_ff: CarryFf,
    fold: bool,
    fill: FillOptions,
    objective: PlaceObjective,
    records: Vec<PassRecord>,
}

impl PackPlan {
    pub fn carry_di(&self) -> CarryDi {
        self.carry_di
    }
    pub fn carry_ff(&self) -> CarryFf {
        self.carry_ff
    }
    pub fn fold(&self) -> bool {
        self.fold
    }
    pub fn fill(&self) -> &FillOptions {
        &self.fill
    }
    pub fn objective(&self) -> PlaceObjective {
        self.objective
    }
    pub fn records(&self) -> &[PassRecord] {
        &self.records
    }
    pub fn json(&self) -> Value {
        let by_name: BTreeMap<&str, Value> = self.records.iter().map(|r| (r.name, r.json())).collect();
        json!({"carry_di": format!("{:?}", self.carry_di), "carry_ff": format!("{:?}", self.carry_ff), "fold": self.fold,
               "bypass_ffs_per_slice": self.fill.bypass_ffs_per_slice, "strategies": by_name,
               "objective": {"pins": [self.objective.pins.cap, self.objective.pins.weight],
                             "bypass": [self.objective.bypass.cap, self.objective.bypass.weight]}})
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bypass_needs_a_lowered_floor() {
        let caps = xc7z020();
        let mut pol = PackPolicy {
            carry_di: CarryDi::Bypass,
            ..PackPolicy::exploration()
        };
        let plan = pol.plan(&caps).expect("core is above pnr");
        assert_eq!(plan.carry_di(), CarryDi::Lut5);
        let r = plan.records().iter().find(|r| r.name == "carry_di_bypass").expect("recorded");
        assert!(matches!(&r.outcome, Outcome::Refused(w) if w.contains("CarryDiBypass is Modeled")));
        pol.floor = Level::Modeled;
        assert_eq!(pol.plan(&caps).expect("plan").carry_di(), CarryDi::Bypass);
    }

    #[test]
    fn carry_ff_shared_by_default_not_for_nextpnr() {
        let caps = xc7z020();
        assert_eq!(PackPolicy::exploration().plan(&caps).expect("plan").carry_ff(), CarryFf::Shared);
        let plan = PackPolicy::nextpnr_reference().plan(&caps).expect("plan");
        assert_eq!(plan.carry_ff(), CarryFf::SoleUser);
        let r = plan.records().iter().find(|r| r.name == "carry_ff_shared").expect("recorded");
        assert_eq!(r.outcome, Outcome::Disabled);
    }

    #[test]
    fn production_refuses_the_core() {
        assert!(PackPolicy::production().plan(&xc7z020()).is_err());
    }

    #[test]
    fn fill_strategies_follow_the_floor() {
        let caps = xc7z020();
        let plan = PackPolicy::exploration().plan(&caps).expect("plan");
        assert_eq!(plan.fill().lut5, Lut5Fill::ALL);
        let pol = PackPolicy {
            floor: Level::FasmMatchesReference,
            ..PackPolicy::exploration()
        };
        let plan = pol.plan(&caps).expect("core is fasm-matched");
        assert_eq!(plan.fill().lut5, Lut5Fill::NONE);
        assert!(plan.records().iter().filter(|r| r.stage == "fill").all(|r| matches!(r.outcome, Outcome::Refused(_))));
    }

    #[test]
    fn disabled_is_not_refused() {
        let pol = PackPolicy {
            lut5: Lut5Fill { pairs: false, ..Lut5Fill::ALL },
            ..PackPolicy::exploration()
        };
        let plan = pol.plan(&xc7z020()).expect("plan");
        let r = plan.records().iter().find(|r| r.name == "lut5_pairs").expect("recorded");
        assert_eq!(r.outcome, Outcome::Disabled);
        assert!(!plan.fill().lut5.pairs && plan.fill().lut5.chain);
    }
}
