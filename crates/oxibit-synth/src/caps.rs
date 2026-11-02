//! What the target can do, per primitive mode, and how sure we are of it.
//!
//! Every pass declares the primitive modes it emits (`stage::Pass::EMITS`);
//! the flow compares each against the device's `Level` for that mode and
//! the policy's floor (`policy::Policy::floor`) and refuses a pass whose
//! output the target cannot be trusted with. Levels only go up with
//! evidence: a mode is promoted by recording what showed it (a FASM match
//! against a reference flow, a board run), never by assumption.

use std::collections::BTreeMap;

/// A primitive configuration synthesis can emit.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Mode {
    /// LUT1..LUT6, single output.
    Lut,
    /// Two LUT5 functions packed into one LUT6 (O5 / O6). Not emitted yet.
    Lut62,
    /// FDRE / FDSE.
    FfSync,
    /// FDCE / FDPE.
    FfAsync,
    Carry4,
    MuxF7,
    MuxF8,
    /// DSP48E1, combinational multiply (no internal registers, no cascade).
    Dsp48Comb,
    /// DSP48E1 PCOUT -> PCIN cascade / post-adder use.
    Dsp48Cascade,
    Srlc32e,
    Ramb18,
    Ramb36,
    Ram32m,
    Ram64m,
    Ram128x1d,
    Ibuf,
    Obuf,
}

/// How far a mode is known to work, lowest first.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Level {
    /// The back end cannot place or configure it.
    Unsupported,
    /// Synthesis can emit it and a simulation model exists.
    Modeled,
    /// oxibit-pnr places and configures it.
    PnrSupported,
    /// oxibit's FASM for it equals a reference flow's (nextpnr) on a design.
    FasmMatchesReference,
    /// A bitstream using it behaved correctly on a board.
    SiliconProven,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Status {
    pub level: Level,
    /// What established the level (design, check, board, date).
    pub basis: &'static str,
}

/// Cost-model facts the passes read instead of hard-coding them.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Cost {
    /// LUT inputs.
    pub lut_k: usize,
    /// Two functions per LUT6 (changes what a carry position costs).
    pub lut62_packing: bool,
}

#[derive(Clone, Debug)]
pub struct DeviceCaps {
    pub part: &'static str,
    pub modes: BTreeMap<Mode, Status>,
    pub cost: Cost,
}

impl DeviceCaps {
    pub fn status(&self, m: Mode) -> Status {
        self.modes.get(&m).cloned().unwrap_or(Status {
            level: Level::Unsupported,
            basis: "not listed",
        })
    }

    pub fn level(&self, m: Mode) -> Level {
        self.status(m).level
    }

    /// xc7z020 as oxibit's back end stands. Nothing is recorded as
    /// silicon-proven here: promote a mode with the board run that shows it.
    pub fn xc7z020() -> Self {
        use Level::*;
        let modes = [
            (
                Mode::Lut,
                FasmMatchesReference,
                "fasm_equiv vs nextpnr routes of libresdr_radio and tx: slice cells and LUTs equal",
            ),
            (
                Mode::FfSync,
                FasmMatchesReference,
                "fasm_equiv vs nextpnr routes of libresdr_radio and tx",
            ),
            (
                Mode::Carry4,
                FasmMatchesReference,
                "fasm_equiv vs nextpnr routes of libresdr_radio and tx",
            ),
            (Mode::FfAsync, PnrSupported, "oxibit-pnr cellcfg handles FDCE / FDPE"),
            (
                Mode::Ibuf,
                FasmMatchesReference,
                "io_cfg_check vs nextpnr: radio 109/109, tx 229/229 I/O lines",
            ),
            (
                Mode::Obuf,
                FasmMatchesReference,
                "io_cfg_check vs nextpnr: radio 109/109, tx 229/229 I/O lines",
            ),
            (
                Mode::Ramb18,
                FasmMatchesReference,
                "bramcfg vs nextpnr: radio and tx 1720/1720 BRAM lines",
            ),
            (
                Mode::Ramb36,
                FasmMatchesReference,
                "bramcfg vs nextpnr: radio and tx 1720/1720 BRAM lines",
            ),
            (Mode::Dsp48Comb, PnrSupported, "oxibit-pnr cellcfg DSP48E1 attributes"),
            (Mode::Srlc32e, PnrSupported, "oxibit-pnr cellcfg / lutinit SRLC32E"),
            (Mode::Ram32m, PnrSupported, "oxibit-pnr cellcfg RAMD32 / RAMS32"),
            (Mode::Ram64m, PnrSupported, "oxibit-pnr cellcfg RAMD64E"),
            (Mode::Ram128x1d, PnrSupported, "oxibit-pnr cellcfg RAMD64E"),
            (Mode::MuxF7, Unsupported, "no MUXF7 in oxibit-pnr"),
            (Mode::MuxF8, Unsupported, "no MUXF8 in oxibit-pnr"),
            (Mode::Dsp48Cascade, Unsupported, "no PCIN / PCOUT routing in oxibit-pnr"),
            (Mode::Lut62, Unsupported, "no dual-output LUT packing in oxibit-pnr"),
        ];
        DeviceCaps {
            part: "xc7z020clg400-1",
            modes: modes.into_iter().map(|(m, level, basis)| (m, Status { level, basis })).collect(),
            cost: Cost {
                lut_k: 6,
                lut62_packing: false,
            },
        }
    }
}
