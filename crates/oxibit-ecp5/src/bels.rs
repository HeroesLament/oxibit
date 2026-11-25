//! ECP5 bels and their pin wires, as libtrellis's `get_routing_graph_ecp5`
//! places them (`CommonBels`, `Ecp5Bels` in `Bels.cpp`).
//!
//! A bel sits at a grid location (x = col, y = row) with an index `z`, and
//! each pin is a wire: at the bel's location under its raw name, or at
//! (0, 0) for the global clock bels' pins (`G_...`). Pin wire names are not
//! globalised; they are the names the graph already gives the tile's local
//! wires (`A0_SLICE`, `JPADDIA_PIO`, `G_CLKI_LDCC0`).
//!
//! Bels are held as templates (name, type, z, pins with their wire names)
//! shared by every location that has them, like the graph's tile types:
//! an 85F has 42k SLICEs but four SLICE templates.
//!
//! Locations follow libtrellis, including the offsets: a PLL bel is one
//! tile in from its `PLL0_*` tile, the config bels (GSR, JTAGG, OSCG,
//! SEDGA) one row above `EFB0_PICB0`, the ECLK bels two columns out from
//! `ECLK_L`/`ECLK_R`.

use std::collections::HashMap;

use crate::db::DeviceDb;
use crate::graph::{Graph, NameId, Wire};
use crate::{Result, err};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PinDir {
    /// Driven by the routing (the wire's uphill arcs).
    In,
    /// Drives the routing (the wire's downhill arcs).
    Out,
}

/// Where a pin's wire is.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PinAt {
    /// At the bel's location.
    Bel,
    /// At (0, 0): a global.
    Origin,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Pin {
    pub name: String,
    pub dir: PinDir,
    pub at: PinAt,
    pub wire: NameId,
}

/// A bel as it appears at every location that has it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Template {
    pub name: String,
    pub ty: String,
    pub z: i32,
    pub pins: Vec<Pin>,
}

impl Template {
    pub fn pin(&self, name: &str) -> Option<&Pin> {
        self.pins.iter().find(|p| p.name == name)
    }
}

pub type BelId = u32;
pub type TemplateId = u32;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Bel {
    pub template: TemplateId,
    pub x: i16,
    pub y: i16,
}

/// How PLC2 tiles are split into bels.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SliceMode {
    /// One `SLICE` bel per slice (`add_lc`).
    Whole,
    /// `TRELLIS_COMB` and `TRELLIS_FF` per LUT, and `TRELLIS_RAMW` in slice
    /// C (`add_logic_comb`, `add_ff`, `add_ramw`).
    Split,
}

/// A template being built, with pins added in libtrellis's order.
struct Builder<'g> {
    g: &'g mut Graph,
    t: Template,
}

impl<'g> Builder<'g> {
    fn new(g: &'g mut Graph, name: impl Into<String>, ty: impl Into<String>, z: i32) -> Self {
        Builder {
            g,
            t: Template {
                name: name.into(),
                ty: ty.into(),
                z,
                pins: Vec::new(),
            },
        }
    }

    fn pin(&mut self, name: impl Into<String>, dir: PinDir, at: PinAt, wire: &str) -> &mut Self {
        let wire = self.g.intern(wire);
        self.t.pins.push(Pin {
            name: name.into(),
            dir,
            at,
            wire,
        });
        self
    }

    fn input(&mut self, name: impl Into<String>, wire: &str) -> &mut Self {
        self.pin(name, PinDir::In, PinAt::Bel, wire)
    }

    fn output(&mut self, name: impl Into<String>, wire: &str) -> &mut Self {
        self.pin(name, PinDir::Out, PinAt::Bel, wire)
    }

    fn global_input(&mut self, name: impl Into<String>, wire: &str) -> &mut Self {
        self.pin(name, PinDir::In, PinAt::Origin, wire)
    }

    fn global_output(&mut self, name: impl Into<String>, wire: &str) -> &mut Self {
        self.pin(name, PinDir::Out, PinAt::Origin, wire)
    }

    /// `pin` on wire `<J?><pin><suffix>`
    fn input_j(&mut self, pin: &str, j: Prefix, suffix: &str) -> &mut Self {
        let wire = format!("{}{pin}{suffix}", j.as_str());
        self.input(pin, &wire)
    }

    fn output_j(&mut self, pin: &str, j: Prefix, suffix: &str) -> &mut Self {
        let wire = format!("{}{pin}{suffix}", j.as_str());
        self.output(pin, &wire)
    }

    fn done(self) -> Template {
        self.t
    }
}

/// Whether a pin's wire name carries the `J` prefix.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Prefix {
    J,
    None,
}

impl Prefix {
    fn as_str(self) -> &'static str {
        match self {
            Prefix::J => "J",
            Prefix::None => "",
        }
    }
}

const ABCD: [char; 4] = ['A', 'B', 'C', 'D'];

/// `CommonBels::add_lc`.
fn lc(g: &mut Graph, z: usize) -> Template {
    let l = ABCD[z];
    let (lc0, lc1) = (z * 2, z * 2 + 1);
    let mut b = Builder::new(g, format!("SLICE{l}"), "SLICE", z as i32);
    for (i, lc) in [(0, lc0), (1, lc1)] {
        for p in ['A', 'B', 'C', 'D', 'M'] {
            b.input(format!("{p}{i}"), &format!("{p}{lc}_SLICE"));
        }
    }
    if z == 0 {
        b.input("FCI", "FCI_SLICE");
    } else {
        b.input("FCI", &format!("FCI{l}_SLICE"));
    }
    b.input("FXA", &format!("FXA{l}_SLICE"))
        .input("FXB", &format!("FXB{l}_SLICE"))
        .input("CLK", &format!("CLK{z}_SLICE"))
        .input("LSR", &format!("LSR{z}_SLICE"))
        .input("CE", &format!("CE{z}_SLICE"))
        .input("DI0", &format!("DI{lc0}_SLICE"))
        .input("DI1", &format!("DI{lc1}_SLICE"));
    if z < 2 {
        b.input("WD0", &format!("WD0{l}_SLICE"))
            .input("WD1", &format!("WD1{l}_SLICE"));
        for i in 0..4 {
            b.input(format!("WAD{i}"), &format!("WAD{i}{l}_SLICE"));
        }
        b.input("WRE", &format!("WRE{z}_SLICE"))
            .input("WCK", &format!("WCK{z}_SLICE"));
    }
    b.output("F0", &format!("F{lc0}_SLICE"))
        .output("Q0", &format!("Q{lc0}_SLICE"))
        .output("F1", &format!("F{lc1}_SLICE"))
        .output("Q1", &format!("Q{lc1}_SLICE"))
        .output("OFX0", &format!("F5{l}_SLICE"))
        .output("OFX1", &format!("FX{l}_SLICE"));
    if z == 3 {
        b.output("FCO", "FCO_SLICE");
    } else {
        b.output("FCO", &format!("FCO{l}_SLICE"));
    }
    if z == 2 {
        wdo_wado(&mut b);
    }
    b.done()
}

/// The slice C distributed RAM write port outputs.
fn wdo_wado(b: &mut Builder<'_>) {
    for i in 0..4 {
        b.output(format!("WDO{i}"), &format!("WDO{i}C_SLICE"));
    }
    for i in 0..4 {
        b.output(format!("WADO{i}"), &format!("WADO{i}C_SLICE"));
    }
}

/// `CommonBels::add_logic_comb`, `z` the LUT index 0..8.
fn logic_comb(g: &mut Graph, z: usize) -> Template {
    let l = ABCD[z / 2];
    let i = z % 2;
    let mut b = Builder::new(g, format!("SLICE{l}.K{i}"), "TRELLIS_COMB", z as i32 * 4);
    for p in ['A', 'B', 'C', 'D', 'M'] {
        b.input(p.to_string(), &format!("{p}{z}_SLICE"));
    }
    if z < 4 {
        for w in 0..4 {
            b.input(format!("WAD{w}"), &format!("WAD{w}{l}_SLICE"));
        }
        b.input("WD", &format!("WD{i}{l}_SLICE"))
            .input("WRE", &format!("WRE{}_SLICE", z / 2))
            .input("WCK", &format!("WCK{}_SLICE", z / 2));
    }
    if z == 0 {
        b.input("FCI", "FCI_SLICE");
    } else if z % 2 == 0 {
        b.input("FCI", &format!("FCI{l}_SLICE"));
    } else {
        b.input("FCI", &format!("FCI{l}1_SLICE"));
    }
    b.output("F", &format!("F{z}_SLICE"));
    if z % 2 == 0 {
        b.input("F1", &format!("F{}_SLICE", z + 1))
            .output("OFX", &format!("F5{l}_SLICE"));
    } else {
        b.input("FXA", &format!("FXA{l}_SLICE"))
            .input("FXB", &format!("FXB{l}_SLICE"))
            .output("OFX", &format!("FX{l}_SLICE"));
    }
    if z == 7 {
        b.output("FCO", "FCO_SLICE");
    } else if z % 2 == 1 {
        b.output("FCO", &format!("FCO{l}_SLICE"));
    } else {
        b.output("FCO", &format!("FCI{l}1_SLICE"));
    }
    b.done()
}

/// `CommonBels::add_ff`, `z` the LUT index 0..8.
fn ff(g: &mut Graph, z: usize) -> Template {
    let l = ABCD[z / 2];
    let i = z % 2;
    let c = z / 2;
    let mut b = Builder::new(g, format!("SLICE{l}.FF{i}"), "TRELLIS_FF", z as i32 * 4 + 1);
    b.input("DI", &format!("DI{z}_SLICE"))
        .input("M", &format!("M{z}_SLICE"))
        .input("CLK", &format!("CLK{c}_SLICE"))
        .input("LSR", &format!("LSR{c}_SLICE"))
        .input("CE", &format!("CE{c}_SLICE"))
        .output("Q", &format!("Q{z}_SLICE"));
    b.done()
}

/// `CommonBels::add_ramw`.
fn ramw(g: &mut Graph) -> Template {
    let mut b = Builder::new(g, "SLICEC.RAMW", "TRELLIS_RAMW", 4 * 4 + 2);
    for (i, lc) in [(0, 4), (1, 5)] {
        for p in ['A', 'B', 'C', 'D'] {
            b.input(format!("{p}{i}"), &format!("{p}{lc}_SLICE"));
        }
    }
    wdo_wado(&mut b);
    b.done()
}

/// `Ecp5Bels::add_pio`.
fn pio(g: &mut Graph, z: usize) -> Template {
    let l = ABCD[z];
    let mut b = Builder::new(g, format!("PIO{l}"), "PIO", z as i32);
    b.input("I", &format!("PADDO{l}_PIO"))
        .input("T", &format!("PADDT{l}_PIO"))
        .output("O", &format!("JPADDI{l}_PIO"))
        .input("IOLDO", &format!("IOLDO{l}_PIO"))
        .input("IOLTO", &format!("IOLTO{l}_PIO"));
    b.done()
}

/// IOLOGIC on the left and right edges, or the simpler SIOLOGIC on the top
/// and bottom.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum IoLogic {
    Side,
    TopBottom,
}

/// `Ecp5Bels::add_iologic`.
fn iologic(g: &mut Graph, z: usize, kind: IoLogic) -> Template {
    use Prefix::{J, None as N};
    let l = ABCD[z];
    let ss = if kind == IoLogic::TopBottom { "S" } else { "" };
    let suffix = format!("{l}_{ss}IOLOGIC");
    let zz = z as i32 + if kind == IoLogic::TopBottom { 2 } else { 4 };
    let mut b = Builder::new(g, format!("{ss}IOLOGIC{l}"), format!("{ss}IOLOGIC"), zz);
    b.input_j("DI", N, &suffix)
        .output_j("IOLDO", N, &suffix)
        .output_j("IOLDOD", N, &suffix)
        .input_j("IOLDOI", N, &suffix)
        .output_j("IOLTO", N, &suffix)
        .output_j("INDD", N, &suffix)
        .input_j("PADDI", N, &suffix);
    for p in [
        "CLK",
        "CE",
        "LSR",
        "LOADN",
        "MOVE",
        "DIRECTION",
        "TSDATA0",
        "TXDATA0",
        "TXDATA1",
    ] {
        b.input_j(p, J, &suffix);
    }
    for p in ["RXDATA0", "RXDATA1", "INFF", "CFLAG"] {
        b.output_j(p, J, &suffix);
    }
    if kind == IoLogic::Side {
        b.input_j("ECLK", N, &suffix);
        for p in ["TSDATA1", "TXDATA2", "TXDATA3"] {
            b.input_j(p, J, &suffix);
        }
        b.output_j("RXDATA2", J, &suffix)
            .output_j("RXDATA3", J, &suffix);
        if z % 2 == 0 {
            for p in ["TXDATA4", "TXDATA5", "TXDATA6", "SLIP"] {
                b.input_j(p, J, &suffix);
            }
            for p in ["RXDATA4", "RXDATA5", "RXDATA6"] {
                b.output_j(p, J, &suffix);
            }
        }
        for p in [
            "DQSR90", "DQSW270", "DQSW", "RDPNTR0", "RDPNTR1", "RDPNTR2", "WRPNTR0", "WRPNTR1",
            "WRPNTR2",
        ] {
            b.input_j(p, N, &suffix);
        }
    }
    b.done()
}

/// `Ecp5Bels::add_dcc`.
fn dcc(g: &mut Graph, side: char, z: usize) -> Template {
    let id = format!("{side}DCC{z}");
    let mut b = Builder::new(g, id.clone(), "DCCA", z as i32);
    b.global_input("CLKI", &format!("G_CLKI_{id}"))
        .global_input("CE", &format!("G_JCE_{id}"))
        .global_output("CLKO", &format!("G_CLKO_{id}"));
    b.done()
}

/// `Ecp5Bels::add_dcs`.
fn dcs(g: &mut Graph, z: usize) -> Template {
    let mut b = Builder::new(g, format!("DCS{z}"), "DCSC", z as i32 + 4);
    b.global_input("CLK0", &format!("G_CLK0_DCS{z}"))
        .global_input("CLK1", &format!("G_CLK1_DCS{z}"))
        .global_output("DCSOUT", &format!("G_DCSOUT_DCS{z}"))
        .global_input("MODESEL", &format!("G_JMODESEL_DCS{z}"))
        .global_input("SEL0", &format!("G_JSEL0_DCS{z}"))
        .global_input("SEL1", &format!("G_JSEL1_DCS{z}"));
    b.done()
}

/// `Ecp5Bels::add_bram`.
fn bram(g: &mut Graph, z: usize) -> Template {
    let mut b = Builder::new(g, format!("EBR{z}"), "DP16KD", z as i32);
    for i in 0..14 {
        b.input(format!("ADA{i}"), &format!("JADA{i}_EBR"))
            .input(format!("ADB{i}"), &format!("JADB{i}_EBR"));
    }
    for p in [
        "CEA", "CEB", "CLKA", "CLKB", "CSA0", "CSA1", "CSA2", "CSB0", "CSB1", "CSB2",
    ] {
        b.input_j(p, Prefix::J, "_EBR");
    }
    for i in 0..18 {
        b.input(format!("DIA{i}"), &format!("JDIA{i}_EBR"))
            .input(format!("DIB{i}"), &format!("JDIB{i}_EBR"))
            .output(format!("DOA{i}"), &format!("JDOA{i}_EBR"))
            .output(format!("DOB{i}"), &format!("JDOB{i}_EBR"));
    }
    for p in ["OCEA", "OCEB", "RSTA", "RSTB", "WEA", "WEB"] {
        b.input_j(p, Prefix::J, "_EBR");
    }
    b.done()
}

/// `Ecp5Bels::add_mult18`.
fn mult18(g: &mut Graph, z: usize) -> Template {
    let mut b = Builder::new(g, format!("MULT18_{z}"), "MULT18X18D", z as i32);
    let j = Prefix::J;
    for sig in ["CLK", "CE", "RST"] {
        for i in 0..4 {
            b.input_j(&format!("{sig}{i}"), j, "_MULT18");
        }
    }
    for sig in ["SIGNED", "SOURCE"] {
        for c in ["A", "B"] {
            b.input_j(&format!("{sig}{c}"), j, "_MULT18");
        }
    }
    for port in ["A", "B", "C", "SRIA", "SRIB"] {
        for i in 0..18 {
            b.input_j(&format!("{port}{i}"), j, "_MULT18");
        }
    }
    for port in ["ROA", "ROB", "ROC", "SROA", "SROB"] {
        for i in 0..18 {
            b.output_j(&format!("{port}{i}"), j, "_MULT18");
        }
    }
    for i in 0..36 {
        b.output_j(&format!("P{i}"), j, "_MULT18");
    }
    b.output_j("SIGNEDP", j, "_MULT18");
    b.done()
}

/// `Ecp5Bels::add_alu54b`.
fn alu54b(g: &mut Graph, z: usize) -> Template {
    let mut b = Builder::new(g, format!("ALU54_{z}"), "ALU54B", z as i32);
    let j = Prefix::J;
    for sig in ["CLK", "CE", "RST"] {
        for i in 0..4 {
            b.input_j(&format!("{sig}{i}"), j, "_ALU54");
        }
    }
    for p in ["SIGNEDIA", "SIGNEDIB", "SIGNEDCIN"] {
        b.input_j(p, j, "_ALU54");
    }
    for port in ["A", "B", "MA", "MB"] {
        for i in 0..36 {
            b.input_j(&format!("{port}{i}"), j, "_ALU54");
        }
    }
    for port in ["C", "CFB", "CIN"] {
        for i in 0..54 {
            b.input_j(&format!("{port}{i}"), j, "_ALU54");
        }
    }
    for i in 0..11 {
        b.input_j(&format!("OP{i}"), j, "_ALU54");
    }
    for port in ["R", "CO"] {
        for i in 0..54 {
            b.output_j(&format!("{port}{i}"), j, "_ALU54");
        }
    }
    for p in [
        "EQZ",
        "EQZM",
        "EQOM",
        "EQPAT",
        "EQPATB",
        "OVER",
        "UNDER",
        "OVERUNDER",
        "SIGNEDR",
    ] {
        b.output_j(p, j, "_ALU54");
    }
    b.done()
}

/// `Ecp5Bels::add_mult9x9d`. From the local model: MULT9X9D is the
/// 9x9 split mode of the same slice pair MULT18X18D occupies: two
/// 9-bit multipliers per slice half, sharing the C preload and the
/// shift chains (SRIA/SRIB in, SROA/SROB/ROC out, 18-bit product).
/// Shares the J wires of the MULT18 site.
fn mult9x9d(g: &mut Graph, z: usize) -> Template {
    let mut b = Builder::new(g, format!("MULT9_{z}"), "MULT9X9D", z as i32);
    let j = Prefix::J;
    for sig in ["CLK", "CE", "RST"] {
        for i in 0..4 {
            b.input_j(&format!("{sig}{i}"), j, "_MULT18");
        }
    }
    for p in ["SIGNEDA", "SIGNEDB", "SIGNEDP", "SOURCEA", "SOURCEB"] {
        b.input_j(p, j, "_MULT18");
    }
    for port in ["A", "B", "C", "SRIA", "SRIB"] {
        for i in 0..9 {
            b.input_j(&format!("{port}{i}"), j, "_MULT18");
        }
    }
    for port in ["ROA", "ROB", "ROC", "SROA", "SROB"] {
        for i in 0..9 {
            b.output_j(&format!("{port}{i}"), j, "_MULT18");
        }
    }
    for i in 0..18 {
        b.output_j(&format!("P{i}"), j, "_MULT18");
    }
    b.done()
}

/// `Ecp5Bels::add_alu24b`. From the local model: ALU24B is the 24-bit
/// ALU split of the ALU54B datapath (two ALUs per slice), with opcode
/// registers per half (OP0..10), an INPUTCFB feedback register, and
/// the same flag outputs as ALU54B.
fn alu24b(g: &mut Graph, z: usize) -> Template {
    let mut b = Builder::new(g, format!("ALU24_{z}"), "ALU24B", z as i32);
    let j = Prefix::J;
    for sig in ["CLK", "CE", "RST"] {
        for i in 0..4 {
            b.input_j(&format!("{sig}{i}"), j, "_ALU54");
        }
    }
    for p in ["SIGNEDIA", "SIGNEDIB"] {
        b.input_j(p, j, "_ALU54");
    }
    for port in ["MA", "MB"] {
        for i in 0..18 {
            b.input_j(&format!("{port}{i}"), j, "_ALU54");
        }
    }
    for port in ["C", "CFB"] {
        for i in 0..24 {
            b.input_j(&format!("{port}{i}"), j, "_ALU54");
        }
    }
    // CIN shares the C pins' physical wires (the C fabric bus), from the
    // local model: in the 24-bit split the C operand feeds the carry-in
    // mux selected by OPCINSEL.
    for i in 0..24 {
        b.input(&format!("CIN{i}"), &format!("JC{i}_ALU54"));
    }
    // OPADDNSUB/OPCINSEL ride the ALU site's OP fabric wires (OP0..OP10 in
    // the 54-bit mode; in the 24-bit split they are the two opcode mux
    // selects), from the local model.
    for (pin, wire) in [("OPADDNSUB", "OP0"), ("OPCINSEL", "OP1")] {
        b.input(pin, &format!("J{wire}_ALU54"));
    }
    for port in ["R", "CO"] {
        for i in 0..24 {
            b.output_j(&format!("{port}{i}"), j, "_ALU54");
        }
    }
    b.done()
}

/// `Ecp5Bels::add_pradd9a`. From the local model: PRADD9A is the 9-bit
/// pre-adder feeding a MULT9X9D A operand. Control space (to be mapped
/// to fuses): SOURCEA_MODE A_SHIFT/C_SHIFT/A_C_DYNAMIC/HIGHSPEED,
/// SOURCEB_MODE SHIFT/PARALLEL/INTERNAL, FB_MUX SHIFT/SHIFT_BYPASS/
/// DISABLED, SYMMETRY_MODE DIRECT/INTERNAL, plus REG_OPPRE (the
/// pre-adder output register) and REG_INPUTA/B/C.
fn pradd9a(g: &mut Graph, z: usize) -> Template {
    let mut b = Builder::new(g, format!("PRADD9_{z}"), "PRADD9A", z as i32);
    let j = Prefix::J;
    for sig in ["CLK", "CE", "RST"] {
        for i in 0..4 {
            b.input_j(&format!("{sig}{i}"), j, "_MULT18");
        }
    }
    // The pre-adder shares the multiplier site's operand pins: PA/PB/C ride
    // the same physical A/B/C buses as the multiplier's operands, from the
    // local model.
    for (pin, wire) in [("PA", "A"), ("PB", "B")] {
        for i in 0..9 {
            b.input(&format!("{pin}{i}"), &format!("J{wire}{i}_MULT18"));
        }
    }
    for i in 0..9 {
        b.input_j(&format!("C{i}"), j, "_MULT18");
        b.input_j(&format!("SRIA{i}"), j, "_MULT18");
        b.input_j(&format!("SRIB{i}"), j, "_MULT18");
        b.output_j(&format!("SROA{i}"), j, "_MULT18");
        b.output_j(&format!("SROB{i}"), j, "_MULT18");
        b.output_j(&format!("PO{i}"), j, "_MULT18");
    }
    for p in ["SOURCEA", "OPPRE"] {
        b.input_j(p, j, "_MULT18");
    }
    b.done()
}

/// `Ecp5Bels::add_pradd18a`. From the local model: the 18-bit
/// pre-adder, same structure as PRADD9A at full operand width,
/// feeding a MULT18X18D.
fn pradd18a(g: &mut Graph, z: usize) -> Template {
    let mut b = Builder::new(g, format!("PRADD18_{z}"), "PRADD18A", z as i32);
    let j = Prefix::J;
    for sig in ["CLK", "CE", "RST"] {
        for i in 0..4 {
            b.input_j(&format!("{sig}{i}"), j, "_MULT18");
        }
    }
    for port in ["PA", "PB", "C", "SRIA", "SRIB", "SROA", "SROB"] {
        for i in 0..18 {
            b.input_j(&format!("{port}{i}"), j, "_MULT18");
        }
    }
    for p in ["SOURCEA", "SOURCEB", "CEOPPRE", "CASCREG"] {
        b.input_j(p, j, "_MULT18");
    }
    for i in 0..18 {
        b.output_j(&format!("PO{i}"), j, "_MULT18");
    }
    b.done()
}

/// `Ecp5Bels::add_pll`.
fn pll(g: &mut Graph, quad: &str) -> Template {
    let mut b = Builder::new(g, format!("EHXPLL_{quad}"), "EHXPLLL", 0);
    let j = Prefix::J;
    for p in [
        "REFCLK",
        "RST",
        "STDBY",
        "PHASEDIR",
        "PHASELOADREG",
        "PHASESEL0",
        "PHASESEL1",
        "PHASESTEP",
        "PLLWAKESYNC",
        "ENCLKOP",
        "ENCLKOS2",
        "ENCLKOS3",
        "ENCLKOS",
    ] {
        b.input_j(p, j, "_PLL");
    }
    b.input("CLKI", "CLKI_PLL")
        .input("CLKFB", "CLKFB_PLL")
        .output("CLKINTFB", "CLKINTFB_PLL");
    for p in ["LOCK", "INTLOCK", "CLKOP", "CLKOS", "CLKOS2", "CLKOS3"] {
        b.output_j(p, j, "_PLL");
    }
    b.done()
}

/// `Ecp5Bels::add_dcu`: the pins are the `DCU0` tile's fixed connections
/// ending in `_DCU` that are not relative (no N/E/W/S lead).
fn dcu(g: &mut Graph, conns: &[(String, String)]) -> Template {
    let is_pin = |net: &str| net.ends_with("_DCU") && !net.starts_with(['N', 'E', 'W', 'S']);
    let to_pin = |net: &str| {
        let n = net.strip_suffix("_DCU").unwrap_or(net);
        n.strip_prefix('J').unwrap_or(n).to_owned()
    };
    let mut b = Builder::new(g, "DCU", "DCUA", 0);
    for (sink, source) in conns {
        if is_pin(source) {
            b.output(to_pin(source), source);
        }
        if is_pin(sink) {
            b.input(to_pin(sink), sink);
        }
    }
    b.done()
}

/// `Ecp5Bels::add_extref`.
fn extref(g: &mut Graph) -> Template {
    let mut b = Builder::new(g, "EXTREF", "EXTREFB", 1);
    b.input("REFCLKP", "REFCLKP_EXTREF")
        .input("REFCLKN", "REFCLKN_EXTREF")
        .output("REFCLKO", "JREFCLKO_EXTREF");
    b.done()
}

/// `Ecp5Bels::add_pcsclkdiv`.
fn pcsclkdiv(g: &mut Graph, z: usize) -> Template {
    let name = format!("PCSCLKDIV{z}");
    let mut b = Builder::new(g, name.clone(), "PCSCLKDIV", z as i32);
    b.input("CLKI", &format!("CLKI_{name}"))
        .input("RST", &format!("JRST_{name}"))
        .input("SEL0", &format!("JSEL0_{name}"))
        .input("SEL1", &format!("JSEL1_{name}"))
        .input("SEL2", &format!("JSEL2_{name}"))
        .output("CDIV1", &format!("CDIV1_{name}"))
        .output("CDIVX", &format!("CDIVX_{name}"));
    b.done()
}

/// `Ecp5Bels::add_misc`.
fn misc(g: &mut Graph, name: &str) -> Result<Template> {
    use Prefix::{J, None as N};
    let (postfix, z) = match name {
        "GSR" => ("_GSR", 0),
        "JTAGG" => ("_JTAG", 1),
        "OSCG" => ("_OSC", 2),
        "SEDGA" => ("_SED", 3),
        "DTR" => ("_DTR", 0),
        "USRMCLK" => ("_CCLK", 1),
        _ => return Err(err!("unknown bel {name}")),
    };
    let mut b = Builder::new(g, name, name, z);
    match name {
        "GSR" => {
            b.input_j("GSR", J, postfix).input_j("CLK", J, postfix);
        }
        "JTAGG" => {
            for p in ["TCK", "TMS", "TDI", "JTDO2", "JTDO1"] {
                b.input_j(p, J, postfix);
            }
            for p in [
                "TDO", "JTDI", "JTCK", "JRTI2", "JRTI1", "JSHIFT", "JUPDATE", "JRSTN", "JCE2",
                "JCE1",
            ] {
                b.output_j(p, J, postfix);
            }
        }
        "OSCG" => {
            b.global_output("OSC", "G_JOSC_OSC")
                .output_j("SEDSTDBY", N, postfix);
        }
        "SEDGA" => {
            for p in ["SEDENABLE", "SEDSTART", "SEDFRCERR"] {
                b.input_j(p, J, postfix);
            }
            for p in ["SEDDONE", "SEDINPROG", "SEDERR"] {
                b.output_j(p, J, postfix);
            }
            b.input_j("SEDSTDBY", N, postfix);
        }
        "DTR" => {
            b.input_j("STARTPULSE", J, postfix);
            for i in 0..8 {
                b.output_j(&format!("DTROUT{i}"), J, postfix);
            }
        }
        _ => {
            b.input_j("PADDO", J, postfix)
                .input_j("PADDT", J, postfix)
                .output_j("PADDI", J, postfix);
        }
    }
    Ok(b.done())
}

/// The bels `Ecp5Bels::add_ioclk_bel` knows.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum IoClk {
    ClkDivF,
    EclkSyncB,
    EclkBuf,
    EclkBridgeCs,
    BrgEclkSync,
    DllDelD,
    DdrDll,
    DqsBufM,
}

/// `Ecp5Bels::add_ioclk_bel`. `bank` is only part of the ECLKSYNCB name and
/// the ECLKBUF output wire.
fn ioclk(g: &mut Graph, kind: IoClk, i: usize, bank: Option<u32>) -> Template {
    use Prefix::{J, None as N};
    let bank = bank.map_or_else(|| "-1".to_owned(), |b| b.to_string());
    match kind {
        IoClk::ClkDivF => {
            let pf = format!("_CLKDIV{i}");
            let mut b = Builder::new(g, format!("CLKDIV{i}"), "CLKDIVF", i as i32);
            b.input_j("CLKI", N, &pf)
                .input_j("RST", J, &pf)
                .input_j("ALIGNWD", J, &pf)
                .output_j("CDIVX", J, &pf);
            b.done()
        }
        IoClk::EclkSyncB => {
            let pf = format!("_ECLKSYNC{i}");
            let mut b = Builder::new(
                g,
                format!("ECLKSYNC{i}_BK{bank}"),
                "ECLKSYNCB",
                8 + i as i32,
            );
            b.input_j("ECLKI", N, &pf)
                .input_j("STOP", J, &pf)
                .output_j("ECLKO", J, &pf);
            b.done()
        }
        IoClk::EclkBuf => {
            let mut b = Builder::new(g, format!("ECLKBUF{i}"), "TRELLIS_ECLKBUF", 10 + i as i32);
            b.input("ECLKI", &format!("JECLK{i}"))
                .global_output("ECLKO", &format!("G_BANK{bank}ECLK{i}"));
            b.done()
        }
        IoClk::EclkBridgeCs => {
            let pf = format!("_ECLKBRIDGECS{i}");
            let mut b = Builder::new(g, format!("ECLKBRIDGECS{i}"), "ECLKBRIDGECS", 14);
            b.input_j("CLK0", J, &pf)
                .input_j("CLK1", J, &pf)
                .input_j("SEL", J, &pf)
                .output_j("ECSOUT", N, &pf);
            b.done()
        }
        IoClk::BrgEclkSync => {
            let pf = format!("_BRGECLKSYNC{i}");
            let mut b = Builder::new(g, format!("BRGECLKSYNC{i}"), "ECLKSYNCB", 15);
            b.input_j("ECLKI", N, &pf)
                .input_j("STOP", J, &pf)
                .output_j("ECLKO", J, &pf);
            b.done()
        }
        IoClk::DllDelD => {
            let pf = "_DLLDEL";
            let mut b = Builder::new(g, "DLLDEL", "DLLDELD", 12);
            b.input_j("A", J, pf)
                .input_j("DDRDEL", N, pf)
                .input_j("LOADN", J, pf)
                .input_j("MOVE", J, pf)
                .input_j("DIRECTION", J, pf)
                .output_j("Z", N, pf)
                .output_j("CFLAG", J, pf);
            b.done()
        }
        IoClk::DdrDll => {
            let pf = "_DDRDLL";
            let mut b = Builder::new(g, "DDRDLL", "DDRDLL", 0);
            for p in ["CLK", "RST", "UDDCNTLN", "FREEZE"] {
                b.input_j(p, J, pf);
            }
            b.output_j("DDRDEL", N, pf)
                .output_j("LOCK", J, pf)
                .output_j("DIVOSC", J, pf);
            for k in 0..8 {
                b.output_j(&format!("DCNTL{k}"), J, pf);
            }
            b.done()
        }
        IoClk::DqsBufM => {
            let pf = "_DQS";
            let mut b = Builder::new(g, "DQSBUF", "DQSBUFM", 8);
            for p in [
                "DQSI",
                "READ1",
                "READ0",
                "READCLKSEL2",
                "READCLKSEL1",
                "READCLKSEL0",
            ] {
                b.input_j(p, J, pf);
            }
            b.input_j("DDRDEL", N, pf)
                .input_j("ECLK", N, pf)
                .input_j("SCLK", J, pf)
                .input_j("RST", J, pf);
            for k in 0..8 {
                b.input_j(&format!("DYNDELAY{k}"), J, pf);
            }
            for p in [
                "PAUSE",
                "RDLOADN",
                "RDMOVE",
                "RDDIRECTION",
                "WRLOADN",
                "WRMOVE",
                "WRDIRECTION",
            ] {
                b.input_j(p, J, pf);
            }
            for p in ["DQSR90", "DQSW", "DQSW270"] {
                b.output_j(p, J, pf);
            }
            for k in 0..3 {
                b.output_j(&format!("RDPNTR{k}"), N, pf)
                    .output_j(&format!("WRPNTR{k}"), N, pf);
            }
            for p in ["DATAVALID", "BURSTDET", "RDCFLAG", "WRCFLAG"] {
                b.output_j(p, J, pf);
            }
            b.done()
        }
    }
}

/// What to place for one tile: a template key (so each distinct bel is
/// built once) and the location offset from the tile.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
enum Key {
    Lc(usize),
    Comb(usize),
    Ff(usize),
    Ramw,
    Pio(usize),
    IoLogic(usize, IoLogic),
    Dcc(char, usize),
    Dcs(usize),
    Bram(usize),
    Mult18(usize),
    Alu54(usize),
    // The 9x9 split multiplier, 24-bit ALU split, and the pre-adders.
    // Port lists from the local model; fuse bits for these modes are not
    // yet known (Trellis maps only the MULT18/ALU54 bits in MIB_DSP2),
    // so they stay out of dsp.rs TYPES until verified on silicon.
    Mult9(usize),
    Alu24(usize),
    Pradd9(usize),
    Pradd18(usize),
    Pll(&'static str),
    Dcu,
    Extref,
    PcsClkDiv(usize),
    Misc(&'static str),
    IoClk(IoClk, usize, Option<u32>),
}

/// The bels a tile of type `ty` adds, with their (dx, dy) from the tile, in
/// `get_routing_graph_ecp5`'s order.
fn tile_bels(ty: &str, mode: SliceMode) -> Vec<(Key, i16, i16)> {
    let mut v = Vec::new();
    let here = |k: Key| (k, 0, 0);
    if ty == "PLC2" {
        for z in 0..4 {
            match mode {
                SliceMode::Split => {
                    for i in z * 2..(z + 1) * 2 {
                        v.push(here(Key::Comb(i)));
                        v.push(here(Key::Ff(i)));
                    }
                }
                SliceMode::Whole => v.push(here(Key::Lc(z))),
            }
        }
        if mode == SliceMode::Split {
            v.push(here(Key::Ramw));
        }
    }
    if ty.contains("PICL0") || ty.contains("PICR0") {
        for z in 0..4 {
            v.push(here(Key::Pio(z)));
            v.push(here(Key::IoLogic(z, IoLogic::Side)));
        }
    }
    if ty.contains("PIOT0") || (ty.contains("PICB0") && ty != "SPICB0") {
        for z in 0..2 {
            v.push(here(Key::Pio(z)));
            v.push(here(Key::IoLogic(z, IoLogic::TopBottom)));
        }
    }
    if ty == "SPICB0" {
        v.push(here(Key::Pio(0)));
        v.push(here(Key::IoLogic(0, IoLogic::TopBottom)));
    }
    let dccs = match ty {
        "LMID_0" => Some(('L', 14)),
        "RMID_0" => Some(('R', 14)),
        "TMID_0" => Some(('T', 12)),
        "BMID_0V" | "BMID_0H" => Some(('B', 16)),
        _ => None,
    };
    if let Some((side, n)) = dccs {
        for z in 0..n {
            v.push(here(Key::Dcc(side, z)));
        }
    }
    match ty {
        "EBR_CMUX_UL" | "DSP_CMUX_UL" => v.push(here(Key::Dcs(0))),
        "EBR_CMUX_LL" | "EBR_CMUX_LL_25K" => v.push(here(Key::Dcs(1))),
        _ => {}
    }
    match ty {
        "MIB_EBR0" | "EBR_CMUX_UR" | "EBR_CMUX_LR" | "EBR_CMUX_LR_25K" => {
            v.push(here(Key::Bram(0)));
        }
        "MIB_EBR2" => v.push(here(Key::Bram(1))),
        "MIB_EBR4" => v.push(here(Key::Bram(2))),
        "MIB_EBR6" => v.push(here(Key::Bram(3))),
        // Split-mode bels at the same sites, from the local model: a
        // MULT9_x / PRADD9_x pair lives where MULT18_x is, an ALU24_x
        // where ALU54_x is. The two are fuse-exclusive per site; only one
        // family is emitted per tile at a time (the DSP packer picks).
        "MIB_DSP0" => {
            v.push(here(Key::Mult18(0)));
            v.push(here(Key::Mult9(0)));
            v.push(here(Key::Pradd9(0)));
        }
        "MIB_DSP1" => {
            v.push(here(Key::Mult18(1)));
            v.push(here(Key::Mult9(1)));
            v.push(here(Key::Pradd9(1)));
        }
        "MIB_DSP4" => {
            v.push(here(Key::Mult18(4)));
            v.push(here(Key::Mult9(4)));
            v.push(here(Key::Pradd9(4)));
        }
        "MIB_DSP5" => {
            v.push(here(Key::Mult18(5)));
            v.push(here(Key::Mult9(5)));
            v.push(here(Key::Pradd9(5)));
        }
        "MIB_DSP3" => {
            v.push(here(Key::Alu54(3)));
            v.push(here(Key::Alu24(3)));
        }
        "MIB_DSP7" => {
            v.push(here(Key::Alu54(7)));
            v.push(here(Key::Alu24(7)));
        }
        "PLL0_UL" => v.push((Key::Pll("UL"), 1, 0)),
        "PLL0_LL" => v.push((Key::Pll("LL"), 0, -1)),
        "PLL0_LR" => v.push((Key::Pll("LR"), 0, -1)),
        "PLL0_UR" => v.push((Key::Pll("UR"), -1, 0)),
        "DCU0" => {
            v.push(here(Key::Dcu));
            v.push(here(Key::Extref));
        }
        _ => {}
    }
    if ty == "BMID_0H" {
        for z in 0..2 {
            v.push((Key::PcsClkDiv(z), 0, -1));
        }
    }
    match ty {
        "EFB0_PICB0" => {
            for m in ["GSR", "JTAGG", "OSCG", "SEDGA"] {
                v.push((Key::Misc(m), 0, -1));
            }
        }
        "DTR" => v.push((Key::Misc("DTR"), 0, -1)),
        "EFB1_PICB1" => v.push((Key::Misc("USRMCLK"), -5, 0)),
        _ => {}
    }
    if ty == "ECLK_L" || ty == "ECLK_R" {
        use IoClk::{BrgEclkSync, ClkDivF, DllDelD, EclkBridgeCs, EclkBuf, EclkSyncB};
        let left = ty == "ECLK_L";
        let dx = if left { -2 } else { 2 };
        // The I/O banks of the ECLKs at rows y and y + 1.
        let (b0, b1) = if left { (7, 6) } else { (2, 3) };
        let io = |k, i, bank: Option<u32>| Key::IoClk(k, i, bank);
        v.push((io(ClkDivF, 0, None), dx, 0));
        v.push((io(ClkDivF, 1, None), dx, 0));
        for (k, (i, bank, dy)) in [
            (EclkSyncB, (0, b0, 0)),
            (EclkSyncB, (1, b0, 0)),
            (EclkSyncB, (0, b1, 1)),
            (EclkSyncB, (1, b1, 1)),
            (EclkBuf, (0, b0, 0)),
            (EclkBuf, (1, b0, 0)),
            (EclkBuf, (0, b1, 1)),
            (EclkBuf, (1, b1, 1)),
        ] {
            v.push((io(k, i, Some(bank)), dx, dy));
        }
        for dy in -1..=2 {
            v.push((io(DllDelD, 0, None), dx, dy));
        }
        let bridge = usize::from(left);
        v.push((io(EclkBridgeCs, bridge, None), dx, 0));
        v.push((io(BrgEclkSync, bridge, None), dx, 0));
    }
    let ddrdll = match ty {
        "DDRDLL_UL" => Some((-2, -10)),
        "DDRDLL_ULA" => Some((-2, -13)),
        "DDRDLL_UR" => Some((2, -10)),
        "DDRDLL_URA" => Some((2, -13)),
        "DDRDLL_LL" => Some((-2, 13)),
        "DDRDLL_LR" => Some((2, 13)),
        _ => None,
    };
    if let Some((dx, dy)) = ddrdll {
        v.push((Key::IoClk(IoClk::DdrDll, 0, None), dx, dy));
    }
    if ty == "PICL0_DQS2" || ty == "PICR0_DQS2" {
        v.push(here(Key::IoClk(IoClk::DqsBufM, 0, None)));
    }
    v
}

/// The bels of a chip.
#[derive(Debug)]
pub struct Bels {
    pub templates: Vec<Template>,
    pub bels: Vec<Bel>,
    /// Bels by location, `y * width + x`.
    at: Vec<Vec<BelId>>,
    width: i16,
}

impl Bels {
    /// Every bel of the chip, as `get_routing_graph_ecp5` adds them (no
    /// constant bels: those come with `include_const_pips`, which this
    /// graph does not model).
    pub fn build(g: &mut Graph, db: &DeviceDb, mode: SliceMode) -> Result<Bels> {
        let (width, height) = (g.width, g.height);
        let cells =
            usize::try_from(width).expect("positive") * usize::try_from(height).expect("positive");
        let mut out = Bels {
            templates: Vec::new(),
            bels: Vec::new(),
            at: vec![Vec::new(); cells],
            width,
        };
        let mut ids: HashMap<Key, TemplateId> = HashMap::new();
        let tiles: Vec<(String, i16, i16)> =
            g.tiles.iter().map(|t| (t.ty.clone(), t.x, t.y)).collect();
        for (ty, x, y) in tiles {
            for (key, dx, dy) in tile_bels(&ty, mode) {
                let template = if let Some(&id) = ids.get(&key) {
                    id
                } else {
                    let t = make(g, db, &key)?;
                    let id = TemplateId::try_from(out.templates.len()).expect("fits");
                    out.templates.push(t);
                    ids.insert(key, id);
                    id
                };
                let (bx, by) = (x + dx, y + dy);
                if !(0..width).contains(&bx) || !(0..height).contains(&by) {
                    return Err(err!("bel of tile type {ty} at R{y}C{x} lands off the grid"));
                }
                let id = BelId::try_from(out.bels.len()).expect("bel count fits u32");
                out.bels.push(Bel {
                    template,
                    x: bx,
                    y: by,
                });
                let cell = out.cell(bx, by);
                out.at[cell].push(id);
            }
        }
        Ok(out)
    }

    fn cell(&self, x: i16, y: i16) -> usize {
        usize::try_from(y).expect("on grid") * usize::try_from(self.width).expect("positive")
            + usize::try_from(x).expect("on grid")
    }

    pub fn template(&self, bel: BelId) -> &Template {
        &self.templates[self.bels[bel as usize].template as usize]
    }

    /// The bels at a location.
    pub fn at(&self, x: i16, y: i16) -> &[BelId] {
        if x < 0 || y < 0 || x >= self.width {
            return &[];
        }
        self.at.get(self.cell(x, y)).map_or(&[], Vec::as_slice)
    }

    /// The bel named `name` at a location.
    pub fn find(&self, x: i16, y: i16, name: &str) -> Option<BelId> {
        self.at(x, y)
            .iter()
            .copied()
            .find(|&b| self.template(b).name == name)
    }

    /// The wire of a bel's pin.
    pub fn pin_wire(&self, bel: BelId, pin: &str) -> Option<Wire> {
        let b = self.bels[bel as usize];
        let p = self.template(bel).pin(pin)?;
        Some(wire_of(b, p))
    }

    /// Every (pin, wire) of a bel.
    pub fn pins(&self, bel: BelId) -> impl Iterator<Item = (&Pin, Wire)> + '_ {
        let b = self.bels[bel as usize];
        self.template(bel)
            .pins
            .iter()
            .map(move |p| (p, wire_of(b, p)))
    }
}

fn wire_of(b: Bel, p: &Pin) -> Wire {
    match p.at {
        PinAt::Bel => Wire {
            x: b.x,
            y: b.y,
            name: p.wire,
        },
        PinAt::Origin => Wire {
            x: 0,
            y: 0,
            name: p.wire,
        },
    }
}

fn make(g: &mut Graph, db: &DeviceDb, key: &Key) -> Result<Template> {
    Ok(match *key {
        Key::Lc(z) => lc(g, z),
        Key::Comb(z) => logic_comb(g, z),
        Key::Ff(z) => ff(g, z),
        Key::Ramw => ramw(g),
        Key::Pio(z) => pio(g, z),
        Key::IoLogic(z, kind) => iologic(g, z, kind),
        Key::Dcc(side, z) => dcc(g, side, z),
        Key::Dcs(z) => dcs(g, z),
        Key::Bram(z) => bram(g, z),
        Key::Mult18(z) => mult18(g, z),
        Key::Alu54(z) => alu54b(g, z),
        Key::Mult9(z) => mult9x9d(g, z),
        Key::Alu24(z) => alu24b(g, z),
        Key::Pradd9(z) => pradd9a(g, z),
        Key::Pradd18(z) => pradd18a(g, z),
        Key::Pll(quad) => pll(g, quad),
        Key::Dcu => dcu(g, &db.tile_bits("DCU0")?.fixed_conns),
        Key::Extref => extref(g),
        Key::PcsClkDiv(z) => pcsclkdiv(g, z),
        Key::Misc(name) => misc(g, name)?,
        Key::IoClk(kind, i, bank) => ioclk(g, kind, i, bank),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plc2_bels() {
        let whole = tile_bels("PLC2", SliceMode::Whole);
        assert_eq!(whole.len(), 4);
        let split = tile_bels("PLC2", SliceMode::Split);
        assert_eq!(split.len(), 17, "8 COMB, 8 FF, RAMW");
        assert_eq!(tile_bels("PICL0", SliceMode::Whole).len(), 8);
        assert_eq!(tile_bels("PICB0", SliceMode::Whole).len(), 4);
        assert_eq!(tile_bels("SPICB0", SliceMode::Whole).len(), 2);
        assert_eq!(tile_bels("ECLK_L", SliceMode::Whole).len(), 16);
    }
}
