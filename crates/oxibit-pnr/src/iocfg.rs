//! I/O cell configuration from a placed design: the FASM lines
//! nextpnr-xilinx's `write_io` writes (`write_io_config` per PAD,
//! `write_iol_config` per IOLOGIC cell, the HCLK_IOI3 bank bits) and
//! `write_ilogic_bypass_inversion`, ported from the himbaechel fork
//! (fasm.cc at 655d7df3).
//!
//! HR banks only (IOB33): the parts oxibit targets have no HP bank, and an
//! IOB18 pad is an error rather than a guess.
//!
//! The routing's I/O pseudo-pips (`route::io_pseudo`) depend on two facts
//! from here, which `IoCfg::route_line` applies: a top SING IOI tile names
//! its one site `_Y1`, and an OLOGIC holding an output register takes no
//! `OMUX.D1` bypass.

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};

use oxibit_fasm::route::Graph;
use serde_json::Value;

use crate::cellcfg::int_param;
use crate::lutinit::{Located, Locator};

type R<T> = Result<T, String>;

/// The I/O writers' output.
#[derive(Debug, Default, Clone)]
pub struct IoCfg {
    /// Pad, IOLOGIC and HCLK_IOI3 lines.
    pub lines: BTreeSet<String>,
    /// OLOGIC sites holding an output register, as `TILE/OLOGIC_Yn`.
    pub ologic_registered: HashSet<String>,
    /// IOI SING tiles above their HCLK row: those of every pad and
    /// IOLOGIC cell.
    pub top_sing: HashSet<String>,
    /// IOI tiles seen, with their HCLK_IOI3 tile.
    pub hclk_of: HashMap<String, String>,
}

#[derive(Debug, Default, Clone, Copy)]
struct Bank {
    stepdown: bool,
    vref: bool,
    only_diff: bool,
    tmds_33: bool,
    lvds_25: bool,
}

struct Cell<'a> {
    name: &'a str,
    ty: &'a str,
    v: &'a Value,
    at: Option<Located>,
}

impl Cell<'_> {
    fn attr(&self, k: &str) -> Option<String> {
        prop_str(&self.v["attributes"][k])
    }
    fn param(&self, k: &str) -> Option<String> {
        prop_str(&self.v["parameters"][k])
    }
    fn param_or(&self, k: &str, d: &str) -> String {
        self.param(k).unwrap_or_else(|| d.to_string())
    }
    /// `int_or_default` on the parameters.
    fn int(&self, k: &str, d: i64) -> R<i64> {
        match self.v["parameters"].get(k) {
            None => Ok(d),
            Some(v) => int_param(v).ok_or_else(|| format!("{}: parameter {k} = {v} is not an integer", self.name)),
        }
    }
    /// `bool_or_default` on the parameters (an int, nonzero).
    fn flag(&self, k: &str) -> R<bool> {
        match self.v["parameters"].get(k) {
            None => Ok(false),
            Some(Value::String(s)) if s == "TRUE" => Ok(true),
            Some(Value::String(s)) if s == "FALSE" => Ok(false),
            Some(v) => int_param(v).map(|i| i != 0).ok_or_else(|| format!("{}: parameter {k} = {v} is not a bool", self.name)),
        }
    }
    /// Whether a port is connected (`getPort(p) != nullptr`).
    fn has_port(&self, p: &str) -> bool {
        self.v["connections"][p].as_array().is_some_and(|a| !a.is_empty())
    }
    fn bit(&self, p: &str) -> Option<&Value> {
        self.v["connections"][p].as_array().and_then(|a| a.first())
    }
}

fn prop_str(v: &Value) -> Option<String> {
    match v {
        Value::String(s) => Some(s.clone()),
        Value::Number(n) => Some(n.to_string()),
        _ => None,
    }
}

/// `IOB_X0Y1` -> 1.
fn rel_y(site: &str) -> R<i64> {
    site.rsplit_once('Y').and_then(|(_, y)| y.parse().ok()).ok_or_else(|| format!("site {site}: no Y index"))
}

/// IOI tile -> its HCLK_IOI3 tile, as nextpnr's `hclk_for_ioi`: the tile
/// of a pip driving the IOI's IOCLK0 node.
struct Hclk<'g> {
    g: &'g Graph,
    cache: HashMap<String, String>,
}

impl Hclk<'_> {
    fn of(&mut self, ioi: &str) -> R<String> {
        if let Some(h) = self.cache.get(ioi) {
            return Ok(h.clone());
        }
        let n = ["IOI_IOCLK0", "IOI_SING_IOCLK0"]
            .iter()
            .find_map(|w| self.g.node(&format!("{ioi}/{w}")).ok())
            .ok_or_else(|| format!("{ioi}: no IOI_IOCLK0 wire"))?;
        let mut tiles: Vec<String> = self.g.drivers(n).into_iter().map(|p| p.tile).collect();
        tiles.dedup();
        let h = match &tiles[..] {
            [t] => t.clone(),
            [] => return Err(format!("{ioi}: no pip drives IOCLK0")),
            more => return Err(format!("{ioi}: IOCLK0 driven from several tiles {more:?}")),
        };
        self.cache.insert(ioi.to_string(), h.clone());
        Ok(h)
    }
}

fn top_module(placed: &Value) -> R<&serde_json::Map<String, Value>> {
    let mods = placed["modules"].as_object().ok_or("no modules")?;
    let top = mods
        .values()
        .find(|m| m["attributes"]["top"].is_string() || m["attributes"]["top"].is_number())
        .or_else(|| mods.values().next())
        .ok_or("no top module")?;
    top["cells"].as_object().ok_or_else(|| "no cells".to_string())
}

const IOL_TYPES: [&str; 7] = [
    "ILOGICE3_IFF",
    "OLOGICE2_OUTFF",
    "OLOGICE3_OUTFF",
    "OSERDESE2_OSERDESE2",
    "ISERDESE2_ISERDESE2",
    "IDELAYE2_IDELAYE2",
    "ODELAYE2_ODELAYE2",
];

/// The pad, IOLOGIC and bank lines of a placed design. `g` finds each
/// IOI tile's HCLK_IOI3 tile.
pub fn io_config(placed: &Value, loc: &Locator, g: &Graph) -> R<IoCfg> {
    let cells_v = top_module(placed)?;
    let mut cells: Vec<Cell> = Vec::new();
    for (name, c) in cells_v {
        let ty = c["type"].as_str().unwrap_or("");
        if ty != "PAD" && ty != "INVERTER" && !IOL_TYPES.contains(&ty) && !ty.contains("INBUF") {
            // Only drivers matter for the rest.
            cells.push(Cell { name, ty, v: c, at: None });
            continue;
        }
        let at = c["attributes"]["NEXTPNR_BEL"].as_str().map(|b| loc.locate(b).ok_or_else(|| format!("{name}: no tile for {b}"))).transpose()?;
        cells.push(Cell { name, ty, v: c, at });
    }

    // Net bit -> driving cell types and using cell types.
    let mut driver: HashMap<String, usize> = HashMap::new();
    let mut users: HashMap<String, Vec<usize>> = HashMap::new();
    for (i, c) in cells.iter().enumerate() {
        let dirs = c.v["port_directions"].as_object();
        for (p, bits) in c.v["connections"].as_object().into_iter().flatten() {
            let d = dirs.and_then(|d| d.get(p)).and_then(Value::as_str).unwrap_or("");
            for b in bits.as_array().into_iter().flatten() {
                let k = b.to_string();
                match d {
                    "output" => {
                        driver.insert(k, i);
                    }
                    "input" => users.entry(k).or_default().push(i),
                    _ => {}
                }
            }
        }
    }

    // Bound bels by (tile, site, bel). (The partner-half tests of
    // write_io_config are HP-bank only.)
    let bound: HashSet<(String, String, String)> =
        cells.iter().filter_map(|c| c.at.as_ref().map(|(t, _, s, b)| (t.clone(), s.clone(), b.clone()))).collect();

    let mut hclk = Hclk { g, cache: HashMap::new() };
    let mut banks: BTreeMap<String, Bank> = BTreeMap::new();
    let mut out = IoCfg::default();

    for pad in &cells {
        if pad.ty != "PAD" {
            continue;
        }
        let Some((tile, _, site, _)) = &pad.at else { return Err(format!("{}: unplaced PAD", pad.name)) };
        if tile.contains("IOB18") {
            return Err(format!("{}: {tile} is an HP bank, not supported", pad.name));
        }
        if !tile.contains("IOB33") {
            continue; // GT and other pads: not configured here
        }
        let net = pad.bit("PAD").map(Value::to_string).ok_or_else(|| format!("{}: PAD unconnected", pad.name))?;
        let is_output = driver.contains_key(&net);
        let is_input = users.get(&net).into_iter().flatten().any(|&u| cells[u].ty.contains("INBUF"));
        let mut iostd = pad.attr("IOSTANDARD").unwrap_or_else(|| "LVCMOS33".into());
        let pulltype = pad.attr("PULLTYPE").unwrap_or_else(|| "NONE".into());
        let slew = pad.attr("SLEW").unwrap_or_else(|| "SLOW".into());
        let in_term = pad.attr("IN_TERM");
        if iostd.ends_with("_T_DCI") {
            iostd.truncate(iostd.len() - 6);
        }

        let (px, py) = loc.grid(tile).ok_or_else(|| format!("{tile}: not in the tilegrid"))?;
        let ioi_x = if tile.starts_with("LIOB") {
            px + 1
        } else if tile.starts_with("RIOB") {
            px - 1
        } else {
            return Err(format!("{tile}: unknown IOB side"));
        };
        let ioi = loc.tile_at(ioi_x, py).ok_or_else(|| format!("{tile}: no IOI tile beside it"))?.to_string();
        let h = hclk.of(&ioi)?;
        let (hx, hy) = loc.grid(&h).ok_or_else(|| format!("{h}: not in the tilegrid"))?;
        out.hclk_of.insert(ioi.clone(), h.clone());

        let is_sing = tile.contains("_SING_");
        let is_top_sing = (py, px) < (hy, hx);
        // The pad's routing crosses its IOI tile's pseudo-pips.
        if is_sing && is_top_sing {
            out.top_sing.insert(ioi.clone());
        }
        let ylo = if is_sing { i64::from(is_top_sing) } else { 1 - rel_y(site)? };
        let is_lvcmos = iostd.starts_with("LVCMOS");
        let is_low_volt_lvcmos = matches!(iostd.as_str(), "LVCMOS12" | "LVCMOS15" | "LVCMOS18");
        let has_diff_prefix = iostd.starts_with("DIFF_");
        let is_tmds33 = iostd == "TMDS_33";
        let is_lvds25 = iostd == "LVDS_25";
        let is_lvds = iostd.starts_with("LVDS");
        let only_diff = is_tmds33 || is_lvds25;
        let is_diff = is_tmds33 || is_lvds || has_diff_prefix;
        if has_diff_prefix {
            iostd.drain(..5);
        }
        let is_sstl = matches!(iostd.as_str(), "SSTL12" | "SSTL135" | "SSTL15");

        let bank = banks.entry(h.clone()).or_default();
        bank.only_diff |= only_diff;
        bank.tmds_33 |= is_tmds33;
        bank.lvds_25 |= is_lvds25;

        let pre = format!("{tile}.IOB_Y{ylo}.");
        let mut w = |f: &str| {
            out.lines.insert(format!("{pre}{f}"));
        };

        if is_output {
            let drive = match pad.v["attributes"].get("DRIVE") {
                None => 12,
                Some(v) => int_param(v).ok_or_else(|| format!("{}: DRIVE = {v}", pad.name))?,
            };
            let s = iostd.as_str();
            if s == "SSTL135" {
                w("SSTL135.DRIVE.I_FIXED");
            } else if s == "TMDS_33" && ylo == 0 {
                w("TMDS_33.DRIVE.I_FIXED");
                w("TMDS_33.OUT");
            } else if s == "LVDS_25" && ylo == 0 {
                w("LVDS_25.DRIVE.I_FIXED");
                w("LVDS_25.OUT");
            } else if (s == "LVCMOS15" && drive == 16) || s == "SSTL15" {
                w("LVCMOS15_SSTL15.DRIVE.I16_I_FIXED");
            } else if s == "LVCMOS18" && (drive == 12 || drive == 8) {
                w("LVCMOS18.DRIVE.I12_I8");
            } else if (s == "LVCMOS33" || s == "LVTTL") && (drive == 16 || drive == 12) {
                w("LVCMOS33_LVTTL.DRIVE.I12_I16");
            } else if (s == "LVCMOS33" || s == "LVTTL") && drive == 8 {
                w("LVCMOS33_LVTTL.DRIVE.I12_I8");
            } else if (s == "LVCMOS33" || s == "LVTTL") && drive == 4 {
                w("LVCMOS33_LVTTL.DRIVE.I4");
            } else if drive == 8 && (s == "LVCMOS12" || s == "LVCMOS25") {
                w("LVCMOS12_LVCMOS25.DRIVE.I8");
            } else if drive == 4 && matches!(s, "LVCMOS15" | "LVCMOS18" | "LVCMOS25") {
                w("LVCMOS15_LVCMOS18_LVCMOS25.DRIVE.I4");
            } else if is_lvcmos || s == "LVTTL" {
                w(&format!("{s}.DRIVE.I{drive}"));
            }

            if slew == "SLOW" {
                if s != "LVDS_25" && s != "TMDS_33" {
                    w("LVCMOS12_LVCMOS15_LVCMOS18_LVCMOS25_LVCMOS33_LVTTL_SSTL135_SSTL15.SLEW.SLOW");
                }
            } else if s == "SSTL135" || s == "SSTL15" {
                w("SSTL135_SSTL15.SLEW.FAST");
            } else {
                w("LVCMOS12_LVCMOS15_LVCMOS18_LVCMOS25_LVCMOS33_LVTTL.SLEW.FAST");
            }
        }

        if is_input {
            if !is_diff {
                if matches!(iostd.as_str(), "LVCMOS33" | "LVTTL" | "LVCMOS25") {
                    w("LVCMOS25_LVCMOS33_LVTTL.IN");
                }
                if is_sstl {
                    bank.vref = true;
                    w("SSTL135_SSTL15.IN");
                    if let Some(t) = &in_term {
                        w(&format!("IN_TERM.{t}"));
                    }
                }
                // HR: each half has its own .IN bits.
                if is_low_volt_lvcmos {
                    w("LVCMOS12_LVCMOS15_LVCMOS18.IN");
                }
            } else {
                // nextpnr tests for "TDMS_33", so a TMDS_33 input takes the
                // LVDS_25 receiver bits too.
                w("LVDS_25_SSTL135_SSTL15.IN_DIFF");
                if let Some(t) = &in_term {
                    w(&format!("IN_TERM.{t}"));
                }
            }
            if !is_output {
                w("LVCMOS12_LVCMOS15_LVCMOS18_LVCMOS25_LVCMOS33_LVDS_25_LVTTL_SSTL135_SSTL15_TMDS_33.IN_ONLY");
            }
        }

        let mut is_stepdown = false;
        if is_low_volt_lvcmos || is_sstl {
            if iostd == "SSTL12" {
                return Err(format!("{}: SSTL12 is only available on high performance banks", pad.name));
            }
            w("LVCMOS12_LVCMOS15_LVCMOS18_SSTL135_SSTL15.STEPDOWN");
            bank.stepdown = true;
            is_stepdown = true;
        }

        if is_output && is_diff && is_tmds33 && ylo == 1 {
            w(&format!("IN_TERM.{}", in_term.as_deref().unwrap_or("NONE")));
            w("LVCMOS12_LVCMOS15_LVCMOS18_LVCMOS25_LVCMOS33_LVDS_25_LVTTL_SSTL135_SSTL15_TMDS_33.IN_ONLY");
        }

        w(&format!("PULLTYPE.{pulltype}"));

        if bound.contains(&(tile.clone(), site.clone(), "IOB33S.O_ININV".to_string())) {
            out.lines.insert(format!("{tile}.OUT_DIFF"));
        }
        if is_stepdown && !is_sing {
            out.lines.insert(format!("{tile}.IOB_Y{ylo}.LVCMOS12_LVCMOS15_LVCMOS18_SSTL135_SSTL15.STEPDOWN"));
        }
    }

    for c in &cells {
        if !IOL_TYPES.contains(&c.ty) {
            continue;
        }
        let Some((tile, _, site, _)) = &c.at else { return Err(format!("{}: unplaced {}", c.name, c.ty)) };
        // An IDDR's register reads the delayed input when an IDELAYE2
        // drives D.
        let delayed = c.bit("D").and_then(|b| driver.get(&b.to_string())).is_some_and(|&d| cells[d].ty.contains("IDELAYE2"));
        iol_config(c, tile, site, delayed, loc, &mut hclk, &mut out)?;
    }

    for (h, b) in &banks {
        let mut w = |on: bool, f: &str| {
            if on {
                out.lines.insert(format!("{h}.{f}"));
            }
        };
        w(b.stepdown, "STEPDOWN");
        w(b.vref, "VREF.V_675_MV");
        w(b.only_diff, "ONLY_DIFF_IN_USE");
        w(b.tmds_33, "TMDS_33_IN_USE");
        w(b.lvds_25, "LVDS_25_IN_USE");
    }
    Ok(out)
}

/// `write_int_vector`: `NAME = Wb...`, most significant bit first.
fn vector(name: &str, v: i64, width: u32, invert: bool) -> String {
    let bits: String = (0..width).rev().map(|i| if ((v >> i) & 1 == 1) ^ invert { '1' } else { '0' }).collect();
    format!("{name} = {width}'b{bits}")
}

fn iol_config(c: &Cell, tile: &str, site: &str, delayed: bool, loc: &Locator, hclk: &mut Hclk, out: &mut IoCfg) -> R<()> {
    let is_sing = tile.contains("_SING_");
    let h = hclk.of(tile)?;
    out.hclk_of.insert(tile.to_string(), h.clone());
    let (tx, ty) = loc.grid(tile).ok_or_else(|| format!("{tile}: not in the tilegrid"))?;
    let (hx, hy) = loc.grid(&h).ok_or_else(|| format!("{h}: not in the tilegrid"))?;
    let is_top_sing = (ty, tx) < (hy, hx);
    if is_sing && is_top_sing {
        out.top_sing.insert(tile.to_string());
    }
    let sitetype = site.split('_').next().unwrap_or(site);
    let y = if is_sing { i64::from(is_top_sing) } else { 1 - rel_y(site)? };
    let site_y = format!("{sitetype}_Y{y}");
    let pre = format!("{tile}.{site_y}.");
    let mut lines: Vec<String> = Vec::new();
    let mut w = |f: &str, on: bool| {
        if on {
            lines.push(f.to_string());
        }
    };

    match c.ty {
        "ILOGICE3_IFF" => {
            if !c.has_port("D") {
                return Err(format!("{}: ILOGICE3_IFF has disconnected D input", c.name));
            }
            w("IDDR.IN_USE", true);
            w("IDDR_OR_ISERDES.IN_USE", true);
            w("ISERDES.MODE.MASTER", true);
            w("ISERDES.NUM_CE.N1", true);
            w("IFFDELMUXE3.P0", delayed);
            if c.has_port("O") {
                w(if delayed { "IDELMUXE3.P0" } else { "IDELMUXE3.P1" }, true);
            }
            match c.param_or("DDR_CLK_EDGE", "OPPOSITE_EDGE").as_str() {
                "SAME_EDGE" => w("IFF.DDR_CLK_EDGE.SAME_EDGE", true),
                "OPPOSITE_EDGE" => w("IFF.DDR_CLK_EDGE.OPPOSITE_EDGE", true),
                "SAME_EDGE_PIPELINED" => {}
                e => return Err(format!("{}: unsupported DDR_CLK_EDGE {e}", c.name)),
            }
            if c.param_or("SRTYPE", "SYNC") == "SYNC" {
                w("IFF.SRTYPE.SYNC", true);
            } else {
                w("IFF.SRTYPE.ASYNC", true);
            }
            w("IFF.ZINV_C", !c.flag("IS_CLK_INVERTED")?);
            w("ZINV_D", !c.flag("IS_D_INVERTED")?);
            for i in 1..=4 {
                w(&format!("IFF.ZINIT_Q{i}"), c.int(&format!("INIT_Q{i}"), 0)? == 0);
            }
            if c.attr("X_ORIG_PORT_SR").as_deref().unwrap_or("R") == "R" {
                for i in 1..=4 {
                    w(&format!("IFF.ZSRVAL_Q{i}"), true);
                }
            }
        }
        "OLOGICE2_OUTFF" | "OLOGICE3_OUTFF" => {
            out.ologic_registered.insert(format!("{tile}/{site_y}"));
            let edge = c.param_or("DDR_CLK_EDGE", "OPPOSITE_EDGE");
            w("ODDR.DDR_CLK_EDGE.SAME_EDGE", edge == "SAME_EDGE");
            w("ODDR_TDDR.IN_USE", true);
            w("OQUSED", true);
            w("OSERDES.DATA_RATE_OQ.DDR", true);
            w("OSERDES.DATA_RATE_TQ.BUF", true);
            w("OSERDES.SRTYPE.SYNC", c.param_or("SRTYPE", "SYNC") == "SYNC");
            for d in ["D1", "D2"] {
                w(&format!("IS_{d}_INVERTED"), c.flag(&format!("IS_{d}_INVERTED"))?);
            }
            w("ZINIT_OQ", c.int("INIT", 1)? == 0);
            w("ODDR.SRUSED", c.has_port("SR"));
            w("ZSRVAL_OQ", c.attr("X_ORIG_PORT_SR").as_deref().unwrap_or("R") == "R");
            w("ZINV_CLK", !c.flag("IS_CLK_INVERTED")?);
        }
        "OSERDESE2_OSERDESE2" => {
            w("ODDR.DDR_CLK_EDGE.SAME_EDGE", true);
            w("ODDR.SRUSED", true);
            w("ODDR_TDDR.IN_USE", true);
            let is_cascaded = c.param_or("SERDES_MODE", "MASTER") == "SLAVE";
            w("OQUSED", is_cascaded || c.has_port("OQ"));
            w("ZINV_CLK", !c.flag("IS_CLK_INVERTED")?);
            for t in ["T1", "T2", "T3", "T4"] {
                w(&format!("ZINV_{t}"), (c.has_port(t) || t == "T1") && !c.flag(&format!("IS_{t}_INVERTED"))?);
            }
            for i in 1..=8 {
                w(&format!("IS_D{i}_INVERTED"), c.flag(&format!("IS_D{i}_INVERTED"))?);
            }
            w("ZINIT_OQ", !c.flag("INIT_OQ")?);
            w("ZINIT_TQ", !c.flag("INIT_TQ")?);
            w("ZSRVAL_OQ", !c.flag("SRVAL_OQ")?);
            w("ZSRVAL_TQ", !c.flag("SRVAL_TQ")?);
            w("OSERDES.IN_USE", true);
            let rate = c.param_or("DATA_RATE_OQ", "BUF");
            w(&format!("OSERDES.DATA_RATE_OQ.{}", if c.has_port("OQ") { rate.as_str() } else { "DDR" }), true);
            let tq = if c.has_port("TQ") { c.param_or("DATA_RATE_TQ", "BUF") } else { "BUF".into() };
            w(&format!("OSERDES.DATA_RATE_TQ.{tq}"), true);
            let width = c.int("DATA_WIDTH", 8)?;
            match rate.as_str() {
                "DDR" => w(&format!("OSERDES.DATA_WIDTH.DDR.W{width}"), true),
                "SDR" => w(&format!("OSERDES.DATA_WIDTH.SDR.W{width}"), true),
                _ => w(&format!("OSERDES.DATA_WIDTH.W{width}"), true),
            }
            w("OSERDES.SRTYPE.SYNC", true);
            w("OSERDES.TSRTYPE.SYNC", true);
            w("OSERDES.TRISTATE_WIDTH.W4", c.int("TRISTATE_WIDTH", 4)? == 4);
            w("OSERDES.SERDES_MODE.SLAVE", is_cascaded);
            w("IS_CLKDIV_INVERTED", c.flag("IS_CLKDIV_INVERTED")?);
        }
        "ISERDESE2_ISERDESE2" => {
            let data_rate = c.param_or("DATA_RATE", "");
            w("IDDR_OR_ISERDES.IN_USE", true);
            w("IDDR.IN_USE", data_rate == "DDR");
            w("IFF.DDR_CLK_EDGE.OPPOSITE_EDGE", true);
            w("IFF.SRTYPE.SYNC", true);
            for i in 1..=4 {
                w(&format!("IFF.ZINIT_Q{i}"), !c.flag(&format!("INIT_Q{i}"))?);
                w(&format!("IFF.ZSRVAL_Q{i}"), !c.flag(&format!("SRVAL_Q{i}"))?);
            }
            w("IFF.ZINV_C", !c.flag("IS_CLK_INVERTED")?);
            let oclk_inv = c.flag("IS_OCLK_INVERTED")?;
            w("IFF.INV_OCLK", oclk_inv);
            w("IFF.ZINV_OCLK", !oclk_inv);
            let iobdelay = c.param_or("IOBDELAY", "NONE");
            w("IFFDELMUXE3.P0", iobdelay == "IFD");
            w("ZINV_D", !c.flag("IS_D_INVERTED")? && iobdelay != "IFD");
            w("ISERDES.IN_USE", true);
            let width = c.int("DATA_WIDTH", 8)?;
            let mode = c.param_or("INTERFACE_TYPE", "NETWORKING");
            let rate = c.param_or("DATA_RATE", "DDR");
            w(&format!("ISERDES.{mode}.{rate}.W{width}"), true);
            w(&format!("ISERDES.MODE.{}", c.param_or("SERDES_MODE", "MASTER")), true);
            w(&format!("ISERDES.NUM_CE.N{}", c.int("NUM_CE", 1)?), true);
        }
        "IDELAYE2_IDELAYE2" => {
            let t = |k: &str| c.param_or(k, "FALSE") == "TRUE";
            w("IN_USE", true);
            w("CINVCTRL_SEL", t("CINVCTRL_SEL"));
            w("PIPE_SEL", t("PIPE_SEL"));
            w("HIGH_PERFORMANCE_MODE", t("HIGH_PERFORMANCE_MODE"));
            w(&format!("DELAY_SRC_{}", c.param_or("DELAY_SRC", "IDATAIN")), true);
            w(&format!("IDELAY_TYPE_{}", c.param_or("IDELAY_TYPE", "FIXED")), true);
            let v = c.int("IDELAY_VALUE", 0)?;
            w(&vector("IDELAY_VALUE[4:0]", v, 5, false), true);
            w(&vector("ZIDELAY_VALUE[4:0]", v, 5, true), true);
            w("IS_DATAIN_INVERTED", c.flag("IS_DATAIN_INVERTED")?);
            w("IS_IDATAIN_INVERTED", c.flag("IS_IDATAIN_INVERTED")?);
        }
        "ODELAYE2_ODELAYE2" => {
            let t = |k: &str| c.param_or(k, "FALSE") == "TRUE";
            w("IN_USE", true);
            w("CINVCTRL_SEL", t("CINVCTRL_SEL"));
            w("HIGH_PERFORMANCE_MODE", t("HIGH_PERFORMANCE_MODE"));
            let ty = c.param_or("ODELAY_TYPE", "FIXED");
            w(&format!("ODELAY_TYPE_{ty}"), ty != "FIXED");
            let v = c.int("ODELAY_VALUE", 0)?;
            w(&vector("ODELAY_VALUE[4:0]", v, 5, false), true);
            w(&vector("ZODELAY_VALUE[4:0]", v, 5, true), true);
            w("ZINV_ODATAIN", !c.flag("IS_ODATAIN_INVERTED")?);
        }
        t => return Err(format!("{}: unsupported IOLOGIC {t}", c.name)),
    }
    for l in lines {
        out.lines.insert(format!("{pre}{l}"));
    }
    Ok(())
}

impl IoCfg {
    /// A site route-through's line (`Enable::Relay`) as nextpnr writes the
    /// I/O pseudo-pips: `None` for an
    /// `OMUX` setting on an OLOGIC holding an output register; on a top
    /// SING IOI tile the first `Y0` of the setting becomes `Y1`. `line` is
    /// `TILE.SETTING`; other lines pass unchanged.
    pub fn route_line(&self, line: &str) -> Option<String> {
        let Some((tile, c)) = line.split_once('.') else { return Some(line.to_string()) };
        if !(tile.contains("IOI3") || tile.starts_with("RIOI") || tile.starts_with("LIOI")) {
            return Some(line.to_string());
        }
        if c.contains(".OMUX.") {
            if let Some((site, _)) = c.split_once('.') {
                if self.ologic_registered.contains(&format!("{tile}/{site}")) {
                    return None;
                }
            }
        }
        if self.top_sing.contains(tile) {
            return Some(format!("{tile}.{}", c.replacen("Y0", "Y1", 1)));
        }
        Some(line.to_string())
    }
}

/// `write_ilogic_bypass_inversion`: `ZINV_D` for every ILOGIC whose
/// `*_ILOGIC<n>_D` wire is the destination of a used pip in a left,
/// non-SING `LIOI_` tile (the only tile types whose segbits define it).
/// `pips` are (tile, dst wire).
pub fn ilogic_bypass_inversion<'a>(pips: impl IntoIterator<Item = (&'a str, &'a str)>) -> BTreeSet<String> {
    let mut out = BTreeSet::new();
    for (tile, dst) in pips {
        if !tile.starts_with("LIOI_") || tile.contains("_SING") {
            continue;
        }
        let Some(pos) = dst.find("_ILOGIC") else { continue };
        let Some(idx) = dst[pos + 7..].strip_suffix("_D") else { continue };
        if idx.is_empty() || !idx.chars().all(|c| c.is_ascii_digit()) {
            continue;
        }
        out.insert(format!("{tile}.ILOGIC_Y{}.ZINV_D", idx.parse::<u32>().unwrap_or(0)));
    }
    out
}

/// Whether a FASM line is one the I/O writers own: any line of an IOB tile,
/// an IOLOGIC site setting of an IOI tile (pseudo-pip bits included: the
/// routing writes those), or an HCLK_IOI3 bank bit.
pub fn is_io_config(line: &str) -> bool {
    let f = line.split('#').next().unwrap_or("").split('=').next().unwrap_or("").trim();
    let parts: Vec<&str> = f.split('.').collect();
    let Some(&tile) = parts.first() else { return false };
    if tile.contains("IOB33") || tile.contains("IOB18") {
        return true;
    }
    if tile.starts_with("HCLK_IOI") {
        return matches!(&parts[1..], ["STEPDOWN"] | ["VREF", "V_675_MV"] | ["ONLY_DIFF_IN_USE"] | ["TMDS_33_IN_USE"] | ["LVDS_25_IN_USE"]);
    }
    if tile.contains("IOI") && parts.len() >= 3 {
        let site = parts[1];
        return ["ILOGIC_Y", "OLOGIC_Y", "IDELAY_Y", "ODELAY_Y"].iter().any(|p| site.starts_with(p));
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vectors() {
        assert_eq!(vector("IDELAY_VALUE[4:0]", 0, 5, false), "IDELAY_VALUE[4:0] = 5'b00000");
        assert_eq!(vector("ZIDELAY_VALUE[4:0]", 0, 5, true), "ZIDELAY_VALUE[4:0] = 5'b11111");
        assert_eq!(vector("X[4:0]", 6, 5, false), "X[4:0] = 5'b00110");
    }

    #[test]
    fn route_lines() {
        let mut io = IoCfg::default();
        io.ologic_registered.insert("RIOI3_X73Y85/OLOGIC_Y1".into());
        io.top_sing.insert("RIOI3_SING_X73Y99".into());
        assert_eq!(io.route_line("RIOI3_X73Y85.OLOGIC_Y1.OMUX.D1"), None);
        assert_eq!(io.route_line("RIOI3_X73Y85.OLOGIC_Y1.OQUSED").as_deref(), Some("RIOI3_X73Y85.OLOGIC_Y1.OQUSED"));
        assert_eq!(io.route_line("RIOI3_SING_X73Y99.ILOGIC_Y0.ZINV_D").as_deref(), Some("RIOI3_SING_X73Y99.ILOGIC_Y1.ZINV_D"));
        assert_eq!(io.route_line("INT_L_X1Y1.IMUX_L4.GFAN0").as_deref(), Some("INT_L_X1Y1.IMUX_L4.GFAN0"));
    }

    #[test]
    fn owned() {
        assert!(is_io_config("RIOB33_X73Y55.IOB_Y0.PULLTYPE.NONE"));
        assert!(is_io_config("RIOB33_X73Y73.OUT_DIFF"));
        assert!(is_io_config("HCLK_IOI3_X185Y78.LVDS_25_IN_USE"));
        assert!(!is_io_config("HCLK_IOI3_X185Y78.HCLK_IOI_CK_IGCLK0.HCLK_IOI_CK_BUFHCLK0"));
        assert!(is_io_config("RIOI3_X73Y55.IDELAY_Y0.IDELAY_VALUE[4:0] = 5'b00000"));
        assert!(!is_io_config("RIOI3_X73Y55.IOI_OCLK_0.IOI_IMUX31_1"));
    }
}
