//! IO logic: the registers, DDR gearing and delay cells beside each pad.
//!
//! Every left / right PIO has an IOLOGIC bel and every top / bottom PIO a
//! simpler SIOLOGIC (no DDRX2 gearing, no ECLK, no DQS). Synthesis keeps
//! the user's IO primitives as black-box `Prim` cells (their outputs are
//! cut points, so the random check needs no model of them); this module
//! finds the pad each one sits on, merges all of a pad's primitives onto
//! its IOLOGIC bel the way nextpnr's `pack_iologic` does, and gives:
//!
//! - the bel pin each primitive pin routes to (or none, for a connection
//!   inside the IOLOGIC),
//! - the PIO pin a pad's output and tristate take (`IOLDO` / `IOLTO` when
//!   the IOLOGIC drives them, with `DATAMUX_*` / `TRIMUX_TSREG` set),
//! - the `IOLOGIC<l>.*` settings (`write_iol`).
//!
//! The IOLOGIC has one CLK, one CE and one LSR; its input side and output
//! side each select CLK / inverted / off (`CLKIMUX`, `CLKOMUX`), CE or
//! constant 1 (`CEIMUX`, `CEOMUX`), LSR or 0 (`LSRIMUX`, `LSROMUX`). Two
//! primitives on one pad must agree on the nets they share.
//!
//! The input register and the input DDR sample `DI`, which the `JDI<l>`
//! mux takes from the pad (default) or from the delay line's output
//! (`INDD`); the delay line's input is `PADDI`, fixed from the pad. A
//! primitive reading the pad routes to `PADDI`; one after an input delay
//! to `DI`.
//!
//! Primitives (Lattice FPGA Libraries Reference Guide; yosys
//! `techlibs/lattice/cells_ff.vh` and `cells_bb_ecp5.v`):
//!
//! - IFS1P3{BX,DX,IX,JX} input flops: D, SP (enable), SCLK, PD / CD.
//! - IFS1S1{B,D,I,J} input latches (transparent while SCLK is high: the
//!   IOLOGIC latch is transparent while its clock is low, so CLKIMUX INV).
//! - OFS1P3{BX,DX,IX,JX} output flops; on a pad's T they are the
//!   tristate register.
//! - IDDRX1F (D, SCLK, RST, Q0, Q1), ODDRX1F (SCLK, RST, D0, D1, Q).
//! - DELAYF (A, LOADN, MOVE, DIRECTION, Z, CFLAG), DELAYG (A, Z), on a
//!   pad's input or output.
//!
//! Not yet: the ECLK gearing (IDDRX2F, ODDRX2F, IDDR71B, ODDR71B, OSHX2A)
//! and the DQS family, which need the edge clock network.

use std::collections::{BTreeMap, HashMap};

use crate::bels::{BelId, Bels};
use crate::chip::Level;
use crate::config::TileConfig;
use crate::db::DeviceDb;
use crate::graph::Graph;
use crate::io::{PioSite, site_enum, site_word, tie};
use crate::netlist::{Cell, NetId, Netlist, PrimPin};
use crate::{Result, err};

/// IO primitive port declarations, for synthesis.
pub const PRIMITIVES: &str = "
module IFS1P3BX(input PD, D, SP, SCLK, output Q);
endmodule
module IFS1P3DX(input CD, D, SP, SCLK, output Q);
endmodule
module IFS1P3IX(input CD, D, SP, SCLK, output Q);
endmodule
module IFS1P3JX(input PD, D, SP, SCLK, output Q);
endmodule
module IFS1S1B(input PD, D, SCLK, output Q);
endmodule
module IFS1S1D(input CD, D, SCLK, output Q);
endmodule
module IFS1S1I(input CD, D, SCLK, output Q);
endmodule
module IFS1S1J(input PD, D, SCLK, output Q);
endmodule
module OFS1P3BX(input PD, D, SP, SCLK, output Q);
endmodule
module OFS1P3DX(input CD, D, SP, SCLK, output Q);
endmodule
module OFS1P3IX(input CD, D, SP, SCLK, output Q);
endmodule
module OFS1P3JX(input PD, D, SP, SCLK, output Q);
endmodule
module IDDRX1F(input D, SCLK, RST, output Q0, Q1);
endmodule
module ODDRX1F(input SCLK, RST, D0, D1, output Q);
endmodule
module DELAYF(input A, LOADN, MOVE, DIRECTION, output Z, CFLAG);
endmodule
module DELAYG(input A, output Z);
endmodule
module IDDRX2F(input D, SCLK, ECLK, RST, ALIGNWD, output Q3, Q2, Q1, Q0);
endmodule
module ODDRX2F(input SCLK, ECLK, RST, D3, D2, D1, D0, output Q);
endmodule
module IDDR71B(input D, SCLK, ECLK, RST, ALIGNWD, output Q6, Q5, Q4, Q3, Q2, Q1, Q0);
endmodule
module ODDR71B(input SCLK, ECLK, RST, D6, D5, D4, D3, D2, D1, D0, output Q);
endmodule
module OSHX2A(input D1, D0, SCLK, ECLK, RST, output Q);
endmodule
module IDDRX2DQA(input SCLK, ECLK, DQSR90, D, RST, RDPNTR2, RDPNTR1, RDPNTR0, WRPNTR2, WRPNTR1,
  WRPNTR0, output Q3, Q2, Q1, Q0, QWL);
endmodule
module ODDRX2DQA(input D3, D2, D1, D0, DQSW270, SCLK, ECLK, RST, output Q);
endmodule
module ODDRX2DQSB(input D3, D2, D1, D0, SCLK, ECLK, DQSW, RST, output Q);
endmodule
module TSHX2DQA(input T1, T0, SCLK, ECLK, DQSW270, RST, output Q);
endmodule
module TSHX2DQSA(input T1, T0, SCLK, ECLK, DQSW, RST, output Q);
endmodule
";

/// The IO primitive types.
pub const TYPES: &[&str] = &[
    "IFS1P3BX", "IFS1P3DX", "IFS1P3IX", "IFS1P3JX", "IFS1S1B", "IFS1S1D", "IFS1S1I", "IFS1S1J",
    "OFS1P3BX", "OFS1P3DX", "OFS1P3IX", "OFS1P3JX", "IDDRX1F", "ODDRX1F", "DELAYF", "DELAYG",
    "IDDRX2F", "ODDRX2F", "IDDR71B", "ODDR71B", "OSHX2A", "IDDRX2DQA", "ODDRX2DQA", "ODDRX2DQSB",
    "TSHX2DQA", "TSHX2DQSA",
];

/// Their clock inputs (routed on the global network).
pub const CLOCK_INPUTS: &[(&str, &str)] = &[
    ("IFS1P3BX", "SCLK"),
    ("IFS1P3DX", "SCLK"),
    ("IFS1P3IX", "SCLK"),
    ("IFS1P3JX", "SCLK"),
    ("IFS1S1B", "SCLK"),
    ("IFS1S1D", "SCLK"),
    ("IFS1S1I", "SCLK"),
    ("IFS1S1J", "SCLK"),
    ("OFS1P3BX", "SCLK"),
    ("OFS1P3DX", "SCLK"),
    ("OFS1P3IX", "SCLK"),
    ("OFS1P3JX", "SCLK"),
    ("IDDRX1F", "SCLK"),
    ("ODDRX1F", "SCLK"),
    ("IDDRX2F", "SCLK"),
    ("IDDRX2F", "ECLK"),
    ("ODDRX2F", "SCLK"),
    ("ODDRX2F", "ECLK"),
    ("IDDR71B", "SCLK"),
    ("IDDR71B", "ECLK"),
    ("ODDR71B", "SCLK"),
    ("ODDR71B", "ECLK"),
    ("OSHX2A", "SCLK"),
    ("OSHX2A", "ECLK"),
    ("IDDRX2DQA", "SCLK"),
    ("IDDRX2DQA", "ECLK"),
    ("ODDRX2DQA", "SCLK"),
    ("ODDRX2DQA", "ECLK"),
    ("ODDRX2DQSB", "SCLK"),
    ("ODDRX2DQSB", "ECLK"),
    ("TSHX2DQA", "SCLK"),
    ("TSHX2DQA", "ECLK"),
    ("TSHX2DQSA", "SCLK"),
    ("TSHX2DQSA", "ECLK"),
];

pub fn is_iol(ty: &str) -> bool {
    TYPES.contains(&ty)
}

/// The attribute synthesis leaves on a primitive input tied to a constant
/// (the pin itself is then unconnected): `oxibit.tie.<pin>` = `0` / `1`.
pub fn tie_attr(pin: &str) -> String {
    format!("oxibit.tie.{pin}")
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Kind {
    /// Input flop or latch: REGSET set, synchronous, latch.
    InReg { set: bool, sync: bool, latch: bool },
    /// Output flop (or tristate flop, by where Q goes).
    OutReg { set: bool, sync: bool },
    Iddrx1,
    Oddrx1,
    Delay { dynamic: bool },
    /// IDDRX2F (4 bits) or IDDR71B (7 bits), on SCLK and ECLK.
    Iddrxn { seven: bool },
    /// ODDRX2F / ODDR71B.
    Oddrxn { seven: bool },
    /// OSHX2A: two bits out, shifted on ECLK.
    Oshx2,
    /// IDDRX2DQA: a DQ input, DQSR90 and the read / write pointers from
    /// the DQS group's DQSBUFM.
    Iddrx2Dqa,
    /// ODDRX2DQA (DQSW270) / ODDRX2DQSB (DQSW, the strobe pad).
    Oddrx2Dqa { dqsb: bool },
    /// TSHX2DQA (DQSW270) / TSHX2DQSA (DQSW): the DQ / DQS tristate.
    Tshx2 { dqsa: bool },
}

fn kind(ty: &str) -> Option<Kind> {
    use Kind::*;
    let ireg = |set, sync, latch| Some(InReg { set, sync, latch });
    let oreg = |set, sync| Some(OutReg { set, sync });
    match ty {
        "IFS1P3BX" => ireg(true, false, false),
        "IFS1P3DX" => ireg(false, false, false),
        "IFS1P3IX" => ireg(false, true, false),
        "IFS1P3JX" => ireg(true, true, false),
        "IFS1S1B" => ireg(true, false, true),
        "IFS1S1D" => ireg(false, false, true),
        "IFS1S1I" => ireg(false, true, true),
        "IFS1S1J" => ireg(true, true, true),
        "OFS1P3BX" => oreg(true, false),
        "OFS1P3DX" => oreg(false, false),
        "OFS1P3IX" => oreg(false, true),
        "OFS1P3JX" => oreg(true, true),
        "IDDRX1F" => Some(Iddrx1),
        "ODDRX1F" => Some(Oddrx1),
        "DELAYF" => Some(Delay { dynamic: true }),
        "DELAYG" => Some(Delay { dynamic: false }),
        "IDDRX2F" => Some(Iddrxn { seven: false }),
        "IDDR71B" => Some(Iddrxn { seven: true }),
        "ODDRX2F" => Some(Oddrxn { seven: false }),
        "ODDR71B" => Some(Oddrxn { seven: true }),
        "OSHX2A" => Some(Oshx2),
        "IDDRX2DQA" => Some(Iddrx2Dqa),
        "ODDRX2DQA" => Some(Oddrx2Dqa { dqsb: false }),
        "ODDRX2DQSB" => Some(Oddrx2Dqa { dqsb: true }),
        "TSHX2DQA" => Some(Tshx2 { dqsa: false }),
        "TSHX2DQSA" => Some(Tshx2 { dqsa: true }),
        _ => None,
    }
}

/// One pad's IOLOGIC.
#[derive(Clone, Debug)]
pub struct IoLogic {
    /// The pad's port.
    pub port: usize,
    pub site: PioSite,
    pub bel: BelId,
    /// IOLOGIC (left / right) rather than SIOLOGIC.
    pub side: bool,
    /// `IOLOGIC<l>.<key>` settings.
    pub enums: BTreeMap<String, String>,
    /// `DELAY.DEL_VALUE`, when a delay is used.
    pub del_value: Option<u8>,
    /// `PIO<l>.<key>` settings.
    pub pio: BTreeMap<String, String>,
    /// Bel inputs held at a constant.
    pub ties: BTreeMap<&'static str, Level>,
    /// The nets on the shared CLK, CE and LSR.
    clk: Option<NetId>,
    ce: Option<NetId>,
    lsr: Option<NetId>,
    eclk: Option<NetId>,
    /// The primitives on it, for messages.
    names: Vec<String>,
}

impl IoLogic {
    fn set(&mut self, key: &str, v: &str) -> Result<()> {
        match self.enums.get(key) {
            Some(old) if old != v => Err(err!(
                "IOLOGIC of {} ({}): {key} is both {old} and {v}",
                self.names.join(", "),
                site_name(self.site)
            )),
            _ => {
                self.enums.insert(key.to_owned(), v.to_owned());
                Ok(())
            }
        }
    }

    /// `set_iologic_mode`.
    fn mode(&mut self, mode: &str) -> Result<()> {
        let cur = self.enums.get("MODE").cloned().unwrap_or_else(|| "NONE".to_owned());
        if cur != "NONE" && mode == "IREG_OREG" {
            return Ok(());
        }
        if (cur == "IDDRXN" && mode == "ODDRXN") || (cur == "ODDRXN" && mode == "IDDRXN") {
            self.enums.insert("MODE".to_owned(), "ODDRXN".to_owned());
            return Ok(());
        }
        if cur != "NONE" && cur != "IREG_OREG" && cur != mode {
            return Err(err!(
                "IOLOGIC of {} ({}): modes {cur} and {mode} conflict",
                self.names.join(", "),
                site_name(self.site)
            ));
        }
        if !self.side && !matches!(mode, "IREG_OREG" | "IDDRX1_ODDRX1" | "NONE") {
            return Err(err!(
                "{}: mode {mode} needs a left or right pad (top and bottom have SIOLOGIC)",
                site_name(self.site)
            ));
        }
        self.enums.insert("MODE".to_owned(), mode.to_owned());
        Ok(())
    }

    fn share(&mut self, which: &str, net: NetId) -> Result<()> {
        let slot = match which {
            "CLK" => &mut self.clk,
            "CE" => &mut self.ce,
            "ECLK" => &mut self.eclk,
            _ => &mut self.lsr,
        };
        match *slot {
            Some(n) if n != net => Err(err!(
                "IOLOGIC of {} ({}): two different {which} nets",
                self.names.join(", "),
                site_name(self.site)
            )),
            _ => {
                *slot = Some(net);
                Ok(())
            }
        }
    }

    fn tie(&mut self, pin: &'static str, level: Level) -> Result<()> {
        if self.ties.get(pin).is_some_and(|&l| l != level) {
            return Err(err!("{}: {pin} tied both high and low", site_name(self.site)));
        }
        self.ties.insert(pin, level);
        Ok(())
    }
}

fn site_name(s: PioSite) -> String {
    format!("R{}C{} PIO{}", s.row, s.col, s.pio)
}

/// The IO logic of a netlist.
#[derive(Clone, Debug, Default)]
pub struct Plan {
    pub iols: Vec<IoLogic>,
    /// Per IO primitive cell: its IOLOGIC and, per pin, the bel pin it
    /// routes to (`None`: inside the IOLOGIC).
    pub cells: HashMap<usize, (usize, Vec<(String, Option<&'static str>)>)>,
    /// Nets a pad takes from the IOLOGIC: the PIO pin (`IOLDO`, `IOLTO`)
    /// that replaces `I` / `T`.
    pub pio_pin: HashMap<NetId, &'static str>,
}

impl Plan {
    /// The bel of every IO primitive cell.
    pub fn bels(&self) -> impl Iterator<Item = (usize, BelId)> + '_ {
        self.cells.iter().map(|(&c, &(k, _))| (c, self.iols[k].bel))
    }
}

/// How a pin's value reaches the primitive: a net, a constant, or nothing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum In {
    Net(NetId),
    Tie(bool),
    Open,
}

fn input(p: &crate::netlist::Prim, pin: &str) -> In {
    match p.pin(pin) {
        Some(PrimPin::In(Some(n))) => In::Net(n),
        _ => match p.attrs.get(&tie_attr(pin)).map(String::as_str) {
            Some("1") => In::Tie(true),
            Some("0") => In::Tie(false),
            _ => In::Open,
        },
    }
}

fn output(p: &crate::netlist::Prim, pin: &str) -> Option<NetId> {
    match p.pin(pin) {
        Some(PrimPin::Out(n)) => n,
        _ => None,
    }
}

/// Where a net's pad is, seen from the pad's side.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PadUse {
    /// The pad's output value (`Ob.i`, or `I` of its BB / OBZ).
    Out(usize),
    /// The pad's output enable (`T` of its BB / OBZ).
    Tri(usize),
}

/// Plan the IO logic. `pad_sites[port]` is each port's PIO; `pio_bel` its
/// PIO bel.
pub fn plan(bels: &Bels, nl: &Netlist, pad_sites: &[PioSite]) -> Result<Plan> {
    let drivers = nl.drivers();
    // Each net's sinks, as (cell, pin); the pad sinks separately.
    let mut sinks: HashMap<NetId, Vec<(usize, String)>> = HashMap::new();
    let mut pad_sink: HashMap<NetId, Vec<PadUse>> = HashMap::new();
    for (i, c) in nl.cells.iter().enumerate() {
        match c {
            Cell::Ob { port, i: n } => pad_sink.entry(*n).or_default().push(PadUse::Out(*port)),
            Cell::Prim(p) => {
                for (pin, conn) in &p.pins {
                    if let PrimPin::In(Some(n)) = *conn {
                        match (p.pad, pin.as_str()) {
                            (Some(port), "I") => pad_sink.entry(n).or_default().push(PadUse::Out(port)),
                            (Some(port), "T") => pad_sink.entry(n).or_default().push(PadUse::Tri(port)),
                            _ => sinks.entry(n).or_default().push((i, pin.clone())),
                        }
                    }
                }
            }
            _ => {
                for n in crate::netlist::inputs(c) {
                    sinks.entry(n).or_default().push((i, String::new()));
                }
            }
        }
    }
    // The pad a net comes from: an input pad or a BB's O.
    let from_pad = |n: NetId| -> Option<usize> {
        match drivers[n as usize].map(|d| &nl.cells[d as usize]) {
            Some(Cell::Ib { port, .. }) => Some(*port),
            Some(Cell::Prim(p)) if p.pad.is_some() && output(p, "O") == Some(n) => p.pad,
            _ => None,
        }
    };
    // The only use of a net: one pad sink and nothing else.
    let only_pad = |n: NetId| -> Option<PadUse> {
        let ps = pad_sink.get(&n)?;
        (ps.len() == 1 && sinks.get(&n).is_none_or(Vec::is_empty)).then(|| ps[0])
    };

    let io: Vec<usize> = (0..nl.cells.len())
        .filter(|&i| matches!(&nl.cells[i], Cell::Prim(p) if is_iol(&p.ty)))
        .collect();

    // Delays first: an input delay reads a pad; an output delay drives one.
    // Delay cell -> (pad, input side).
    let mut delay_at: HashMap<usize, (usize, bool)> = HashMap::new();
    for &i in &io {
        let Cell::Prim(p) = &nl.cells[i] else { continue };
        if !matches!(kind(&p.ty), Some(Kind::Delay { .. })) {
            continue;
        }
        let a = match input(p, "A") {
            In::Net(n) => n,
            _ => return Err(err!("{} {}: A is not connected", p.ty, p.name)),
        };
        let z = output(p, "Z").ok_or_else(|| err!("{} {}: Z is not connected", p.ty, p.name))?;
        if let Some(port) = from_pad(a) {
            delay_at.insert(i, (port, true));
        } else if let Some(PadUse::Out(port)) = only_pad(z) {
            delay_at.insert(i, (port, false));
        } else {
            return Err(err!(
                "{} {}: must read an input pad or drive only an output pad",
                p.ty,
                p.name
            ));
        }
    }
    // An input delay's output, by net: its pad.
    let delayed_in: HashMap<NetId, usize> = delay_at
        .iter()
        .filter(|&(_, &(_, inp))| inp)
        .filter_map(|(&i, &(port, _))| match &nl.cells[i] {
            Cell::Prim(p) => output(p, "Z").map(|z| (z, port)),
            _ => None,
        })
        .collect();
    // An output delay's input net, by net: (delay cell, pad).
    let delay_in_net: HashMap<NetId, (usize, usize)> = delay_at
        .iter()
        .filter(|&(_, &(_, inp))| !inp)
        .filter_map(|(&i, &(port, _))| match &nl.cells[i] {
            Cell::Prim(p) => match input(p, "A") {
                In::Net(n) => Some((n, (i, port))),
                _ => None,
            },
            _ => None,
        })
        .collect();

    let mut out = Plan::default();
    let mut by_port: HashMap<usize, usize> = HashMap::new();
    let mut get = |out: &mut Plan, port: usize, name: &str| -> Result<usize> {
        if let Some(&k) = by_port.get(&port) {
            out.iols[k].names.push(name.to_owned());
            return Ok(k);
        }
        let site = pad_sites[port];
        let side = bels_is_side(bels, site);
        let bname = format!("{}IOLOGIC{}", if side { "" } else { "S" }, site.pio);
        let bel = bels
            .find(site.col, site.row, &bname)
            .ok_or_else(|| err!("no {bname} beside {} for {name}", site_name(site)))?;
        let mut iol = IoLogic {
            port,
            site,
            bel,
            side,
            enums: BTreeMap::new(),
            del_value: None,
            pio: BTreeMap::new(),
            ties: BTreeMap::new(),
            clk: None,
            ce: None,
            lsr: None,
            eclk: None,
            names: vec![name.to_owned()],
        };
        // nextpnr's IOLOGIC defaults (create_ecp5_cell).
        for (k, v) in [
            ("MODE", "NONE"),
            ("CLKIMUX", "0"),
            ("CLKOMUX", "0"),
            ("LSRIMUX", "0"),
            ("LSROMUX", "0"),
            ("LSRMUX", "LSR"),
        ] {
            iol.enums.insert(k.to_owned(), v.to_owned());
        }
        out.iols.push(iol);
        let k = out.iols.len() - 1;
        by_port.insert(port, k);
        Ok(k)
    };
    for &i in &io {
        let Cell::Prim(p) = &nl.cells[i] else { continue };
        let k = kind(&p.ty).ok_or_else(|| err!("{}: not an IO primitive", p.ty))?;
        // nextpnr pack_iologic: the DDR / shift primitives default to GSR
        // DISABLED; the flops keep ENABLED (their FF cells default to it).
        let gsr_default = match k {
            Kind::InReg { .. } | Kind::OutReg { .. } => "ENABLED",
            _ => "DISABLED",
        };
        let gsr = p.param_str("GSR").unwrap_or_else(|| gsr_default.to_owned());
        let mut pins: Vec<(String, Option<&'static str>)> = Vec::new();
        let ik;
        match k {
            Kind::InReg { .. } | Kind::Iddrx1 | Kind::Iddrxn { .. } | Kind::Iddrx2Dqa => {
                let d = match input(p, "D") {
                    In::Net(n) => n,
                    _ => return Err(err!("{} {}: D is not connected", p.ty, p.name)),
                };
                let (port, via) = if let Some(port) = from_pad(d) {
                    (port, "PADDI")
                } else if let Some(&port) = delayed_in.get(&d) {
                    (port, "DI")
                } else {
                    return Err(err!(
                        "{} {}: D must come straight from an input pad (or its input delay)",
                        p.ty,
                        p.name
                    ));
                };
                ik = get(&mut out, port, &p.name)?;
                let iol = &mut out.iols[ik];
                pins.push(("D".to_owned(), Some(via)));
                clock(iol, p, &mut pins, true)?;
                let rs = if matches!(k, Kind::Iddrx1 | Kind::Iddrxn { .. } | Kind::Iddrx2Dqa) { "RST" } else if p.pin("PD").is_some() || p.attrs.contains_key(&tie_attr("PD")) { "PD" } else { "CD" };
                lsr(iol, p, rs, &mut pins, true)?;
                iol.set("GSR", &gsr)?;
                if let Kind::InReg { set, sync, latch } = k {
                    iol.mode("IREG_OREG")?;
                    enable(iol, p, &mut pins, true)?;
                    iol.set("FF.INREGMODE", if latch { "LATCH" } else { "FF" })?;
                    // In LATCH mode the IOLOGIC is transparent while its clock
                    // is low (silicon, iol_probe P5); IFS1S1* is transparent
                    // while SCLK is high, so the gate goes in inverted.
                    if latch && iol.enums.get("CLKIMUX").map(String::as_str) == Some("CLK") {
                        iol.enums.insert("CLKIMUX".to_owned(), "INV".to_owned());
                    }
                    iol.set("FF.REGSET", if set { "SET" } else { "RESET" })?;
                    iol.set("SRMODE", if sync { "LSR_OVER_CE" } else { "ASYNC" })?;
                    pins.push(("Q".to_owned(), Some("INFF")));
                } else if let Kind::Iddrxn { seven } = k {
                    if seven && !matches!(iol.site.pio, 'A' | 'C') {
                        return Err(err!("{} {}: IDDR71B only on an A or C pad", p.ty, p.name));
                    }
                    iol.mode("IDDRXN")?;
                    iol.set("IDDRXN.MODE", if seven { "IDDR71" } else { "IDDRX2" })?;
                    edge(iol, p, &mut pins)?;
                    if matches!(input(p, "ALIGNWD"), In::Net(_)) {
                        pins.push(("ALIGNWD".to_owned(), Some("SLIP")));
                    }
                    for q in 0..if seven { 7 } else { 4 } {
                        pins.push((format!("Q{q}"), Some(RXDATA[q])));
                    }
                } else if k == Kind::Iddrx2Dqa {
                    iol.mode("MIDDRX_MODDRX")?;
                    iol.set("MIDDRX.MODE", "MIDDRX2")?;
                    edge(iol, p, &mut pins)?;
                    for q in 0..4 {
                        pins.push((format!("Q{q}"), Some(RXDATA[q])));
                    }
                    if output(p, "QWL").is_some() {
                        pins.push(("QWL".to_owned(), Some("INFF")));
                    }
                    dqs_pins(p, &["DQSR90", "RDPNTR2", "RDPNTR1", "RDPNTR0", "WRPNTR2", "WRPNTR1", "WRPNTR0"], &mut pins)?;
                } else {
                    iol.mode("IDDRX1_ODDRX1")?;
                    pins.push(("Q0".to_owned(), Some("RXDATA0")));
                    pins.push(("Q1".to_owned(), Some("RXDATA1")));
                }
            }
            Kind::OutReg { .. } | Kind::Oddrx1 | Kind::Oddrxn { .. } | Kind::Oshx2 | Kind::Oddrx2Dqa { .. } | Kind::Tshx2 { .. } => {
                let q = output(p, "Q").ok_or_else(|| err!("{} {}: Q is not connected", p.ty, p.name))?;
                let (port, tri, q_pin) = match only_pad(q) {
                    Some(PadUse::Out(port)) => (port, false, Some("IOLDO")),
                    Some(PadUse::Tri(port)) if matches!(k, Kind::OutReg { .. } | Kind::Tshx2 { .. }) => (port, true, Some("IOLTO")),
                    _ => match delay_in_net.get(&q) {
                        Some(&(_, port)) if sinks.get(&q).is_some_and(|s| s.len() == 1) => (port, false, None),
                        _ => {
                            return Err(err!(
                                "{} {}: Q must drive only an output pad (or its output delay)",
                                p.ty,
                                p.name
                            ));
                        }
                    },
                };
                ik = get(&mut out, port, &p.name)?;
                let iol = &mut out.iols[ik];
                if q_pin.is_some() {
                    out.pio_pin.insert(q, if tri { "IOLTO" } else { "IOLDO" });
                }
                pins.push(("Q".to_owned(), q_pin));
                clock(iol, p, &mut pins, false)?;
                let rs = if !matches!(k, Kind::OutReg { .. }) { "RST" } else if p.pin("PD").is_some() || p.attrs.contains_key(&tie_attr("PD")) { "PD" } else { "CD" };
                lsr(iol, p, rs, &mut pins, false)?;
                iol.set("GSR", &gsr)?;
                if let Kind::OutReg { set, sync } = k {
                    iol.mode("IREG_OREG")?;
                    enable(iol, p, &mut pins, false)?;
                    let reg = if tri { "TSREG" } else { "OUTREG" };
                    iol.set(&format!("{reg}.OUTREGMODE"), "FF")?;
                    iol.set(&format!("{reg}.REGSET"), if set { "SET" } else { "RESET" })?;
                    iol.set("SRMODE", if sync { "LSR_OVER_CE" } else { "ASYNC" })?;
                    pins.push(("D".to_owned(), Some(if tri { "TSDATA0" } else { "TXDATA0" })));
                    if q_pin.is_some() {
                        let (key, v) = if tri { ("TRIMUX_TSREG", "IOLTO") } else { ("DATAMUX_OREG", "IOLDO") };
                        iol.pio.insert(key.to_owned(), v.to_owned());
                    }
                } else if let Kind::Oddrxn { seven } = k {
                    if seven && !matches!(iol.site.pio, 'A' | 'C') {
                        return Err(err!("{} {}: ODDR71B only on an A or C pad", p.ty, p.name));
                    }
                    iol.mode("ODDRXN")?;
                    iol.set("ODDRXN.MODE", if seven { "ODDR71" } else { "ODDRX2" })?;
                    // Both sides clocked and reset (`pack_iologic`).
                    both_sides(iol);
                    edge(iol, p, &mut pins)?;
                    for d in 0..if seven { 7 } else { 4 } {
                        pins.push((format!("D{d}"), Some(TXDATA[d])));
                    }
                    if q_pin.is_some() {
                        iol.pio.insert("DATAMUX_ODDR".to_owned(), "IOLDO".to_owned());
                    }
                } else if let Kind::Oddrx2Dqa { dqsb } = k {
                    // nextpnr: MODDRX2, the write clock DQSW270 (DQ) or DQSW
                    // (DQS), RST on both sides.
                    let w = if dqsb { "DQSW" } else { "DQSW270" };
                    iol.mode("MIDDRX_MODDRX")?;
                    iol.set("MODDRX.MODE", "MODDRX2")?;
                    iol.set("MIDDRX_MODDRX.WRCLKMUX", w)?;
                    if let Some(v) = iol.enums.get("LSROMUX").cloned() {
                        iol.enums.insert("LSRIMUX".to_owned(), v);
                    }
                    edge(iol, p, &mut pins)?;
                    for d in 0..4 {
                        pins.push((format!("D{d}"), Some(TXDATA[d])));
                    }
                    dqs_pins(p, if dqsb { &["DQSW"] } else { &["DQSW270"] }, &mut pins)?;
                    if q_pin.is_some() {
                        iol.pio.insert("DATAMUX_MDDR".to_owned(), "IOLDO".to_owned());
                    }
                } else if let Kind::Tshx2 { dqsa } = k {
                    if !tri {
                        return Err(err!("{} {}: Q must drive only a pad's tristate (T)", p.ty, p.name));
                    }
                    let w = if dqsa { "DQSW" } else { "DQSW270" };
                    iol.mode("MIDDRX_MODDRX")?;
                    iol.set("MTDDRX.MODE", "MTSHX2")?;
                    iol.set("MTDDRX.REGSET", "SET")?;
                    iol.set("MTDDRX.DQSW_INVERT", if dqsa { "ENABLED" } else { "DISABLED" })?;
                    iol.set("MIDDRX_MODDRX.WRCLKMUX", w)?;
                    iol.set("IOLTOMUX", "TDDR")?;
                    // The DDR tristate reaches the pad only with the PIO's
                    // TRIMUX_TSREG on IOLTO (silicon, dqs probe: at nextpnr's
                    // PADDT the pad stays released). An instance attribute
                    // (* TRIMUX_TSREG = "PADDT" *) overrides it.
                    let v = p.attrs.get("TRIMUX_TSREG").cloned().unwrap_or_else(|| "IOLTO".to_owned());
                    iol.pio.insert("TRIMUX_TSREG".to_owned(), v);
                    edge(iol, p, &mut pins)?;
                    pins.push(("T0".to_owned(), Some("TSDATA0")));
                    pins.push(("T1".to_owned(), Some("TSDATA1")));
                    dqs_pins(p, if dqsa { &["DQSW"] } else { &["DQSW270"] }, &mut pins)?;
                } else if k == Kind::Oshx2 {
                    iol.mode("MIDDRX_MODDRX")?;
                    iol.set("MODDRX.MODE", "MOSHX2")?;
                    if iol.enums.get("LSROMUX").map(String::as_str) == Some("LSRMUX") {
                        iol.enums.insert("LSRIMUX".to_owned(), "LSRMUX".to_owned());
                    }
                    edge(iol, p, &mut pins)?;
                    pins.push(("D0".to_owned(), Some("TXDATA0")));
                    pins.push(("D1".to_owned(), Some("TXDATA2")));
                    if q_pin.is_some() {
                        iol.pio.insert("DATAMUX_MDDR".to_owned(), "IOLDO".to_owned());
                    }
                } else {
                    iol.mode("IDDRX1_ODDRX1")?;
                    pins.push(("D0".to_owned(), Some("TXDATA0")));
                    pins.push(("D1".to_owned(), Some("TXDATA1")));
                    if q_pin.is_some() {
                        iol.pio.insert("DATAMUX_ODDR".to_owned(), "IOLDO".to_owned());
                    }
                }
            }
            Kind::Delay { dynamic } => {
                let (port, inp) = delay_at[&i];
                ik = get(&mut out, port, &p.name)?;
                let iol = &mut out.iols[ik];
                if inp {
                    iol.mode("IREG_OREG")?;
                    pins.push(("A".to_owned(), Some("PADDI")));
                    pins.push(("Z".to_owned(), Some("INDD")));
                } else {
                    iol.set("DELAY.OUTDEL", "ENABLED")?;
                    let a = match input(p, "A") {
                        In::Net(n) => n,
                        _ => unreachable!("checked above"),
                    };
                    // Driven by this pad's own output register / DDR: the
                    // connection is inside the IOLOGIC.
                    let from_iol = drivers[a as usize].is_some_and(|d| {
                        matches!(&nl.cells[d as usize], Cell::Prim(q) if matches!(kind(&q.ty), Some(Kind::OutReg { .. } | Kind::Oddrx1 | Kind::Oddrxn { .. } | Kind::Oshx2)))
                    });
                    pins.push(("A".to_owned(), if from_iol { None } else { Some("TXDATA0") }));
                    pins.push(("Z".to_owned(), Some("IOLDO")));
                    if let Some(z) = output(p, "Z") {
                        out.pio_pin.insert(z, "IOLDO");
                    }
                }
                let mode = p.param_str("DEL_MODE").unwrap_or_else(|| "USER_DEFINED".to_owned());
                let mut v = lookup_delay(&mode).ok_or_else(|| err!("{} {}: DEL_MODE {mode} is not supported", p.ty, p.name))?;
                if let Some(dv) = p.param_int("DEL_VALUE") {
                    if !(0..128).contains(&dv) {
                        return Err(err!("{} {}: DEL_VALUE {dv} is not 0..127", p.ty, p.name));
                    }
                    v = u8::try_from(dv).expect("checked");
                }
                if iol.del_value.is_some_and(|o| o != v) {
                    return Err(err!("{}: two delays", site_name(iol.site)));
                }
                iol.del_value = Some(v);
                // LOADN low holds DEL_VALUE (nextpnr ties the unused
                // controls low and selects LOADN).
                iol.set("LOADNMUX", "LOADN")?;
                for pin in ["LOADN", "MOVE", "DIRECTION"] {
                    let bp: &'static str = match pin {
                        "LOADN" => "LOADN",
                        "MOVE" => "MOVE",
                        _ => "DIRECTION",
                    };
                    match if dynamic { input(p, pin) } else { In::Open } {
                        In::Net(_) => pins.push((pin.to_owned(), Some(bp))),
                        In::Tie(true) => iol.tie(bp, Level::High)?,
                        _ => iol.tie(bp, Level::Low)?,
                    }
                }
                if dynamic {
                    pins.push(("CFLAG".to_owned(), Some("CFLAG")));
                }
            }
        }
        // A constant on a data input (an ODDR / OSHX / TSHX D or T) is a CIB
        // tie at the IOLOGIC, as for the delay controls: left open, the CIB
        // input reads high (silicon, dqs probe: ODDRX2DQSB D = 0,1,0,1 held
        // the strobe pad at 1).
        for (pin, bp) in &pins {
            let Some(bp) = *bp else { continue };
            if !(bp.starts_with("TXDATA") || bp.starts_with("TSDATA")) {
                continue;
            }
            match input(p, pin) {
                In::Tie(true) => out.iols[ik].tie(bp, Level::High)?,
                In::Tie(false) => out.iols[ik].tie(bp, Level::Low)?,
                _ => {}
            }
        }
        out.cells.insert(i, (ik, pins));
    }
    for iol in &mut out.iols {
        iol.enums.entry("GSR".to_owned()).or_insert_with(|| "DISABLED".to_owned());
    }
    Ok(out)
}

/// RXDATA / TXDATA pins by bit.
const RXDATA: [&str; 7] = ["RXDATA0", "RXDATA1", "RXDATA2", "RXDATA3", "RXDATA4", "RXDATA5", "RXDATA6"];
const TXDATA: [&str; 7] = ["TXDATA0", "TXDATA1", "TXDATA2", "TXDATA3", "TXDATA4", "TXDATA5", "TXDATA6"];

/// The DQS group's signals (DQSR90, DQSW, DQSW270, the pointers) onto the
/// IOLOGIC pins of the same name. They come from the group's DQSBUFM over
/// the DQSG wires; the flow checks the group and does not route them.
fn dqs_pins(p: &crate::netlist::Prim, names: &[&'static str], pins: &mut Vec<(String, Option<&'static str>)>) -> Result<()> {
    for &n in names {
        match input(p, n) {
            In::Net(_) => pins.push((n.to_owned(), Some(n))),
            _ => return Err(err!("{} {}: {n} must come from a DQSBUFM", p.ty, p.name)),
        }
    }
    Ok(())
}

/// ECLK onto the IOLOGIC edge clock (left / right IOLOGIC only).
fn edge(iol: &mut IoLogic, p: &crate::netlist::Prim, pins: &mut Vec<(String, Option<&'static str>)>) -> Result<()> {
    if !iol.side {
        return Err(err!("{} {}: needs a left or right pad (ECLK)", p.ty, p.name));
    }
    match input(p, "ECLK") {
        In::Net(n) => {
            iol.share("ECLK", n)?;
            pins.push(("ECLK".to_owned(), Some("ECLK")));
            Ok(())
        }
        _ => Err(err!("{} {}: ECLK is not connected", p.ty, p.name)),
    }
}

/// The input side clocked and reset like the output side (the gearbox
/// uses both).
fn both_sides(iol: &mut IoLogic) {
    for (o, i) in [("CLKOMUX", "CLKIMUX"), ("LSROMUX", "LSRIMUX")] {
        if let Some(v) = iol.enums.get(o).cloned() {
            iol.enums.insert(i.to_owned(), v);
        }
    }
}

/// Whether the PIO at `site` is on the left or right edge (its IOLOGIC is
/// full, not SIOLOGIC).
fn bels_is_side(bels: &Bels, site: PioSite) -> bool {
    bels.find(site.col, site.row, &format!("IOLOGIC{}", site.pio)).is_some()
}

/// SCLK onto the IOLOGIC clock, input or output side.
fn clock(iol: &mut IoLogic, p: &crate::netlist::Prim, pins: &mut Vec<(String, Option<&'static str>)>, input_side: bool) -> Result<()> {
    let key = if input_side { "CLKIMUX" } else { "CLKOMUX" };
    match input(p, "SCLK") {
        In::Net(n) => {
            iol.share("CLK", n)?;
            iol.enums.insert(key.to_owned(), "CLK".to_owned());
            pins.push(("SCLK".to_owned(), Some("CLK")));
        }
        _ => {
            iol.enums.insert(key.to_owned(), "0".to_owned());
        }
    }
    Ok(())
}

/// The set / reset pin onto the IOLOGIC LSR.
fn lsr(iol: &mut IoLogic, p: &crate::netlist::Prim, pin: &str, pins: &mut Vec<(String, Option<&'static str>)>, input_side: bool) -> Result<()> {
    let key = if input_side { "LSRIMUX" } else { "LSROMUX" };
    match input(p, pin) {
        In::Net(n) => {
            iol.share("LSR", n)?;
            iol.enums.insert(key.to_owned(), "LSRMUX".to_owned());
            pins.push((pin.to_owned(), Some("LSR")));
        }
        In::Tie(true) => {
            iol.enums.insert(key.to_owned(), "LSRMUX".to_owned());
            iol.tie("LSR", Level::High)?;
        }
        In::Tie(false) | In::Open => {
            iol.enums.insert(key.to_owned(), "0".to_owned());
        }
    }
    Ok(())
}

/// SP onto the IOLOGIC CE (`CEMUX CE`), or the constant-1 enable.
fn enable(iol: &mut IoLogic, p: &crate::netlist::Prim, pins: &mut Vec<(String, Option<&'static str>)>, input_side: bool) -> Result<()> {
    let key = if input_side { "CEIMUX" } else { "CEOMUX" };
    match input(p, "SP") {
        In::Net(n) => {
            iol.share("CE", n)?;
            iol.set(key, "CEMUX")?;
            iol.set("CEMUX", "CE")?;
            pins.push(("SP".to_owned(), Some("CE")));
        }
        In::Tie(false) => {
            iol.set(key, "CEMUX")?;
            iol.set("CEMUX", "CE")?;
            iol.tie("CE", Level::Low)?;
        }
        In::Tie(true) | In::Open => {
            iol.set(key, "1")?;
        }
    }
    Ok(())
}

/// `lookup_delay`: DEL_MODE's DEL_VALUE.
fn lookup_delay(mode: &str) -> Option<u8> {
    Some(match mode {
        "USER_DEFINED" => 0,
        "DQS_ALIGNED_X2" => 6,
        "DQS_CMD_CLK" => 9,
        "ECLK_ALIGNED" => 21,
        "ECLK_CENTERED" => 11,
        "ECLKBRIDGE_ALIGNED" => 39,
        "ECLKBRIDGE_CENTERED" => 29,
        "SCLK_ALIGNED" => 50,
        "SCLK_CENTERED" => 39,
        "SCLK_ZEROHOLD" => 59,
        _ => return None,
    })
}

/// Write every IOLOGIC's settings, its pad's mux settings and its ties.
pub fn configure(
    g: &Graph,
    db: &DeviceDb,
    bels: &Bels,
    plan: &Plan,
    tiles: &mut BTreeMap<String, TileConfig>,
) -> Result<()> {
    for iol in &plan.iols {
        let pre = format!("IOLOGIC{}", iol.site.pio);
        for (k, v) in &iol.enums {
            site_enum(g, db, iol.site, &format!("{pre}.{k}"), v, tiles)?;
        }
        if let Some(v) = iol.del_value {
            let bits: Vec<bool> = (0..7).map(|b| (v >> b) & 1 == 1).collect();
            site_word(g, db, iol.site, &format!("{pre}.DELAY.DEL_VALUE"), &bits, tiles)?;
        }
        for (k, v) in &iol.pio {
            site_enum(g, db, iol.site, &format!("PIO{}.{k}", iol.site.pio), v, tiles)?;
        }
        for (&pin, &level) in &iol.ties {
            let w = bels
                .pin_wire(iol.bel, pin)
                .ok_or_else(|| err!("{}: IOLOGIC has no pin {pin}", site_name(iol.site)))?;
            tie(g, db, w, level, tiles)?;
        }
    }
    Ok(())
}
