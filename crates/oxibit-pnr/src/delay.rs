//! Interconnect delay of a routed net, by nextpnr-himbaechel's model
//! (`Arch::getPipDelay` with the loads `bindPip` records), so it can be
//! checked against nextpnr's own report and then used to compare and drive
//! routing.
//!
//! For a pip from node `s` to node `d`, in ps:
//!
//! ```text
//! (drive_res(s) + R(s)/2) * load_cap(s) / 1e6       input RC
//! + delay                                           intrinsic, slow corner
//! + (out_res + R(d)/2) * C(d) / 1e6                 output RC
//! ```
//!
//! `load_cap(s)` is the input capacitance of every pip the net uses out of
//! `s`; `drive_res(d)` is the pip's output resistance, plus what drives its
//! input unless the pip is buffered. `R`, `C` are a node's lumped wire RC
//! (`Graph::node_rc`), absent for nodes nextpnr gives no timing class. An
//! arc's delay is the sum over the pips from the source to the sink.
//!
//! nextpnr routes and reports timing with `fast_pip_delays` on
//! (`Arch::route`), which drops the input RC term ([`Model::Fast`]).
//! Against tobs12's report, the intrinsic delays alone ([`Model::Intrinsic`])
//! give every critical-path routing arc to within 0..17 ps after its site
//! pips (20 ps each, which this does not count: the trees here run between
//! site pins); the output RC term as computed here overshoots on long
//! nodes, whose timing classes nextpnr builds differently.

use std::collections::HashMap;

use oxibit_fasm::route::Graph;

/// The cells the timing analysis (`sta`) knows.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CellKind {
    Lut,
    /// F7 or F8 mux.
    Mux,
    Ff,
    Bram,
    Dsp,
    /// CARRY4: combinational, from CIN / CYINIT / DI / S to O and CO.
    Carry,
}

/// A cell delay in ps. `assumed`: a placeholder with no source behind it;
/// a path that uses one says so.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CellDelay {
    pub ps: i64,
    pub assumed: bool,
}

const fn known(ps: i64) -> CellDelay {
    CellDelay { ps, assumed: false }
}

const fn assumed(ps: i64) -> CellDelay {
    CellDelay { ps, assumed: true }
}

/// A port's bus: its name less the bit (`DIADI[3]` and `DIADI3` are `DIADI`).
fn bus(port: &str) -> &str {
    port.trim_end_matches(|c: char| c.is_ascii_digit() || c == '[' || c == ']')
}

// Sources. LUT, mux and flop: the delays nextpnr-himbaechel prints in its
// critical path reports for this part. Block RAM and DSP48E1: the slow
// corner's maximum in prjxray-db's `timings/BRAM_L.sdf` and `DSP_L.sdf`
// (the block RAM's data setup there, 0.737, is the 0.74 nextpnr prints).

/// A CARRY4 pin: its family and bit (`S2` and `S[2]` are (`'S'`, 2); CIN
/// and CYINIT have no bit).
fn carry_pin(port: &str) -> Option<(&str, usize)> {
    match port {
        "CIN" | "CI" => return Some(("CIN", 0)),
        "CYINIT" => return Some(("CYINIT", 0)),
        _ => {}
    }
    let fam = bus(port);
    let bit = port[fam.len()..].trim_matches(|c| c == '[' || c == ']').parse().ok()?;
    matches!(fam, "S" | "DI" | "O" | "CO").then_some((fam, bit))
}

/// CARRY4, slow corner: the largest of rise and fall maxima in prjxray-db's
/// `timings/CLBLL_L.sdf`, ps. Rows CIN, CYINIT, S0..S3; columns O0..O3,
/// CO0..CO3. 0: no arc. DI has no IOPATH there; DIk is given Sk's arcs
/// into the carries (COj, j >= k) and the sums above it (Oj, j > k).
const CARRY4: [[i64; 8]; 6] = [
    [222, 334, 239, 313, 271, 157, 228, 114],
    [482, 598, 584, 642, 536, 494, 592, 580],
    [223, 400, 523, 582, 340, 433, 512, 508],
    [0, 205, 558, 618, 0, 469, 548, 528],
    [0, 0, 226, 330, 0, 0, 292, 376],
    [0, 0, 0, 227, 0, 0, 0, 380],
];

/// A CARRY4 arc's delay, `None` where `from` cannot reach `to`.
fn carry_arc(from: &str, to: &str) -> Option<i64> {
    let (fi, fb) = carry_pin(from)?;
    let (ti, tb) = carry_pin(to)?;
    if tb > 3 || fb > 3 {
        return None;
    }
    let col = match ti {
        "O" => tb,
        "CO" => 4 + tb,
        _ => return None,
    };
    let ps = match fi {
        "CIN" => CARRY4[0][col],
        "CYINIT" => CARRY4[1][col],
        "S" => CARRY4[2 + fb][col],
        // DIk feeds the carry out of bit k, so the sum of bit k never.
        "DI" if ti == "O" && tb == fb => 0,
        "DI" => CARRY4[2 + fb][col],
        _ => return None,
    };
    (ps > 0).then_some(ps)
}

/// Whether a combinational cell has an arc from input `from` to output
/// `to`. Every pair but a CARRY4's lower bits from its higher ones.
pub fn has_arc(kind: CellKind, from: &str, to: &str) -> bool {
    kind != CellKind::Carry || carry_arc(from, to).is_some()
}

/// Through a combinational cell, input pin `from` to output pin `to`.
///
/// A DSP48E1 is timed as an unregistered multiplier (AREG, BREG, MREG and
/// PREG all 0, USE_MULT MULTIPLY, no pre-adder), the slowest arcs of the
/// only unclocked configuration met so far; a control pin (OPMODE, ...)
/// is given the A input's delay.
pub fn comb(kind: CellKind, from: &str, to: &str) -> CellDelay {
    match kind {
        CellKind::Lut if to == "O5" => known(150),
        CellKind::Lut => known(120),
        CellKind::Mux if from.starts_with('S') => known(310),
        CellKind::Mux => known(220),
        CellKind::Dsp => known(match (bus(from), bus(to)) {
            ("A", "ACOUT") => 723,
            ("ACIN", "ACOUT") => 521,
            ("B", "BCOUT") | ("BCIN", "BCOUT") => 735,
            ("B", _) => 3656,
            ("ACIN", _) => 3639,
            ("BCIN", _) => 3410,
            ("C", _) => 1820,
            ("PCIN", _) => 1518,
            _ => 3841,
        }),
        CellKind::Carry => known(carry_arc(from, to).unwrap_or(0)),
        // No combinational arcs.
        CellKind::Ff | CellKind::Bram => known(0),
    }
}

/// Clock to output. `out_reg`: a block RAM's output register is on
/// (DOA_REG or DOB_REG 1).
pub fn clk_to_q(kind: CellKind, out_reg: bool) -> CellDelay {
    match kind {
        CellKind::Ff => known(350),
        CellKind::Bram if out_reg => known(882),
        CellKind::Bram => known(2454),
        // A registered DSP48E1: none met yet.
        _ => assumed(400),
    }
}

/// Setup of input pin `port` before its clock.
pub fn setup(kind: CellKind, port: &str) -> CellDelay {
    match kind {
        CellKind::Ff => known(100),
        CellKind::Bram => known(match bus(port) {
            b if b.starts_with("ADDR") => 566,
            b if b.starts_with("WE") => 532,
            b if b.starts_with("EN") => 466,
            b if b.starts_with("REGCE") => 360,
            b if b.starts_with("RST") => 342,
            _ => 737,
        }),
        // A LUT RAM's write side, a registered DSP48E1.
        _ => assumed(300),
    }
}

// Hold: the fast corner of prjxray-db's `timings/*.sdf` (CLBLL_L, BRAM_L,
// DSP_L): the fast minimum for delays and the fast maximum for hold checks,
// each the extreme over the cell's configurations, so a check against
// these is pessimistic. Routing has no fast-corner model here: a slow
// routing delay times `HOLD_ROUTE_SCALE` stands in for it.

/// The smallest fast-minimum over slow-maximum ratio of any INT_L or INT_R
/// pip (6650 pips; median 0.276). A slow routing delay times this is a
/// lower bound on the fast-corner one, as far as pip intrinsics go.
pub const HOLD_ROUTE_SCALE: f64 = 0.239;

/// Fast-corner minimum through a combinational cell, any arc.
pub fn comb_min(kind: CellKind) -> CellDelay {
    match kind {
        CellKind::Lut => known(42),
        CellKind::Mux => known(19),
        CellKind::Dsp => known(132),
        // CARRY4's smallest fast minimum (CIN to CO3).
        CellKind::Carry => known(39),
        CellKind::Ff | CellKind::Bram => known(0),
    }
}

/// Fast-corner minimum clock to output.
pub fn clk_to_q_min(kind: CellKind) -> CellDelay {
    match kind {
        CellKind::Ff => known(99),
        CellKind::Bram => known(204),
        CellKind::Dsp => known(126),
        CellKind::Lut | CellKind::Mux | CellKind::Carry => assumed(99),
    }
}

/// Fast-corner maximum hold after the capturing clock edge.
pub fn hold(kind: CellKind) -> CellDelay {
    match kind {
        CellKind::Ff => known(241),
        CellKind::Bram => known(667),
        CellKind::Dsp => known(274),
        // A LUT RAM's write side: not among the SDF cells read.
        CellKind::Lut | CellKind::Mux | CellKind::Carry => assumed(241),
    }
}

/// `hold`, by pin where the SDF tells a cell's pins apart: a block RAM's
/// (BRAM_L and BRAM_R, the fast maximum of each pin family). Its data
/// pins set `hold`'s 667; an address needs 360.
pub fn hold_pin(kind: CellKind, port: &str) -> CellDelay {
    if kind != CellKind::Bram {
        return hold(kind);
    }
    const BRAM: [(&str, i64); 6] = [("ADDR", 360), ("EN", 427), ("WE", 197), ("RSTRAM", 453), ("REGCE", 155), ("RSTREG", 67)];
    known(BRAM.iter().find(|(p, _)| port.starts_with(p)).map_or(667, |f| f.1))
}

/// One routed edge: (src node, dst node, tile, type pip index).
pub type TreeEdge = (u32, u32, u32, u32);

/// Which terms a pip's delay has.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Model {
    /// The pip's intrinsic delay only.
    Intrinsic,
    /// Intrinsic and output RC (nextpnr's `fast_pip_delays`).
    Fast,
    /// Intrinsic, input and output RC.
    Full,
}

/// The delay (ps) from `root` to every node of the tree its `edges` form.
pub fn arrivals(g: &Graph, root: u32, edges: &[TreeEdge]) -> HashMap<u32, u64> {
    arrivals_with(g, root, edges, Model::Full)
}

/// `arrivals` under `model`.
pub fn arrivals_with(g: &Graph, root: u32, edges: &[TreeEdge], model: Model) -> HashMap<u32, u64> {
    let mut out_of: HashMap<u32, Vec<TreeEdge>> = HashMap::new();
    let mut load_cap: HashMap<u32, u64> = HashMap::new();
    for &e in edges {
        out_of.entry(e.0).or_default().push(e);
        *load_cap.entry(e.0).or_default() += g.edge_timing(e.2, e.3).in_cap as u64;
    }
    let mut drive_res: HashMap<u32, u64> = HashMap::new();
    let mut at: HashMap<u32, u64> = HashMap::from([(root, 0)]);
    let mut stack = vec![root];
    while let Some(s) = stack.pop() {
        let Some(es) = out_of.get(&s) else { continue };
        for &(_, d, tile, pi) in es {
            if at.contains_key(&d) {
                continue;
            }
            let t = g.edge_timing(tile, pi);
            let src_drive = drive_res.get(&s).copied();
            let mut input_res = src_drive.unwrap_or(0);
            if let Some(rc) = g.node_rc(s) {
                input_res += rc.res as u64 / 2;
            }
            let mut delay = t.delay as u64;
            if model == Model::Full {
                delay += input_res * load_cap.get(&s).copied().unwrap_or(0) / 1_000_000;
            }
            if let (Model::Fast | Model::Full, Some(rc)) = (model, g.node_rc(d)) {
                delay += (t.out_res as u64 + rc.res as u64 / 2) * rc.cap as u64 / 1_000_000;
            }
            let upstream = if t.buffered { 0 } else { src_drive.unwrap_or(0) };
            drive_res.insert(d, upstream + t.out_res as u64);
            at.insert(d, at[&s] + delay);
            stack.push(d);
        }
    }
    at
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn block_ram_clock_to_out_depends_on_the_output_register() {
        assert_eq!(clk_to_q(CellKind::Bram, false), CellDelay { ps: 2454, assumed: false });
        assert_eq!(clk_to_q(CellKind::Bram, true), CellDelay { ps: 882, assumed: false });
    }

    #[test]
    fn block_ram_setup_goes_by_the_pin_bus_in_either_spelling() {
        assert_eq!(setup(CellKind::Bram, "DIPBDIP1").ps, 737);
        assert_eq!(setup(CellKind::Bram, "DIADI[13]").ps, 737);
        assert_eq!(setup(CellKind::Bram, "ADDRARDADDR10").ps, 566);
        assert_eq!(setup(CellKind::Bram, "WEBWE[3]").ps, 532);
        assert_eq!(setup(CellKind::Bram, "ENARDEN").ps, 466);
        assert_eq!(setup(CellKind::Bram, "REGCEAREGCE").ps, 360);
        assert_eq!(setup(CellKind::Bram, "RSTRAMARSTRAM").ps, 342);
    }

    #[test]
    fn dsp_arcs_are_told_apart_by_bus_not_by_prefix() {
        // ACIN and ALUMODE both start with A.
        assert_eq!(comb(CellKind::Dsp, "A[17]", "P[3]").ps, 3841);
        assert_eq!(comb(CellKind::Dsp, "B4", "P47").ps, 3656);
        assert_eq!(comb(CellKind::Dsp, "ACIN29", "P0").ps, 3639);
        assert_eq!(comb(CellKind::Dsp, "ACIN29", "ACOUT29").ps, 521);
        assert_eq!(comb(CellKind::Dsp, "A0", "ACOUT0").ps, 723);
        assert_eq!(comb(CellKind::Dsp, "C[5]", "P[5]").ps, 1820);
        assert_eq!(comb(CellKind::Dsp, "PCIN47", "PCOUT47").ps, 1518);
        assert_eq!(comb(CellKind::Dsp, "ALUMODE2", "P1").ps, 3841);
        assert!(!comb(CellKind::Dsp, "A0", "P0").assumed);
    }

    #[test]
    fn only_unsourced_delays_are_marked_assumed() {
        assert!(!clk_to_q(CellKind::Ff, false).assumed);
        assert!(!setup(CellKind::Ff, "D").assumed);
        assert!(setup(CellKind::Lut, "WE").assumed);
        assert!(clk_to_q(CellKind::Dsp, false).assumed);
        assert!(setup(CellKind::Dsp, "A0").assumed);
    }

    #[test]
    fn carry_arcs_run_upward_only() {
        assert_eq!(comb(CellKind::Carry, "CIN", "CO3").ps, 114);
        assert_eq!(comb(CellKind::Carry, "S0", "O3").ps, 582);
        assert_eq!(comb(CellKind::Carry, "S[1]", "CO[2]").ps, 548);
        assert_eq!(comb(CellKind::Carry, "CYINIT", "O3").ps, 642);
        assert!(has_arc(CellKind::Carry, "S1", "O1"));
        assert!(!has_arc(CellKind::Carry, "S3", "O0"));
        assert!(!has_arc(CellKind::Carry, "S2", "CO1"));
        // DIk: the carry out of bit k and the sums above, not sum k.
        assert!(has_arc(CellKind::Carry, "DI1", "CO1"));
        assert!(has_arc(CellKind::Carry, "DI1", "O2"));
        assert!(!has_arc(CellKind::Carry, "DI1", "O1"));
        assert!(has_arc(CellKind::Lut, "A1", "O6"));
    }

    #[test]
    fn lut_and_mux_delays_follow_the_pins_used() {
        assert_eq!(comb(CellKind::Lut, "A3", "O6").ps, 120);
        assert_eq!(comb(CellKind::Lut, "A3", "O5").ps, 150);
        assert_eq!(comb(CellKind::Mux, "0", "OUT").ps, 220);
        assert_eq!(comb(CellKind::Mux, "S0", "OUT").ps, 310);
    }
}
