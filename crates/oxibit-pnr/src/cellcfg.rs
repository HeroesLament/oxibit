//! Slice cell configuration from a placed design: the FASM lines
//! nextpnr-xilinx's `write_ffs_config`, `write_luts_config` and
//! `write_carry_config` write for a SLICE, from the cells alone (no routing).
//!
//! Per FF: `ZINI` (INIT is not 1) and `ZRST` (a reset, not a set, type).
//! Per half slice: `LATCH`, `FFSYNC`, `CLKINV` or `NOCLKINV`, `SRUSEDMUX`,
//! `CEUSEDMUX`. Per LUT: `SMALL`, `RAM`, `SRL`; per half `WA7USED`,
//! `WA8USED`. Per CARRY4: the `PRECYINIT` input.
//!
//! The routing-bel settings that follow from site paths (FF data muxes,
//! output muxes, `CY0`, `WEMUX`) are `placed::apply_site_muxes`; LUT INITs
//! are `lutinit`. `DI1MUX` (a SLICEM LUT RAM's data input) is not written.
//!
//! nextpnr writes `NOCLKINV` for both halves of every CLB tile that holds a
//! logic cell, also a half with no FF; so does this.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use serde_json::Value;

use crate::lutinit::{slice_name, Located};

type R<T> = Result<T, String>;

/// (tile, FASM site) of a half slice.
pub type Half = (String, String);

/// A parameter as nextpnr reads it (`int_or_default`): a binary string
/// (yosys writes `"00..01"`), a decimal string, or a number.
fn int_param(v: &Value) -> Option<i64> {
    match v {
        Value::Number(n) => n.as_i64(),
        Value::String(s) => {
            let t = s.trim();
            if !t.is_empty() && t.chars().all(|c| c == '0' || c == '1') {
                i64::from_str_radix(t, 2).ok()
            } else {
                t.parse().ok()
            }
        }
        _ => None,
    }
}

/// What drives a cell input bit: nothing, a constant, or a net.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Drive {
    None,
    Const(bool),
    Net,
}

struct Consts {
    vcc: Option<i64>,
    gnd: Option<i64>,
}

impl Consts {
    fn of(cells: &serde_json::Map<String, Value>) -> Self {
        let bit = |ty: &str| {
            cells.values().find(|c| c["type"] == ty).and_then(|c| c["connections"].as_object()?.values().next()?.get(0)?.as_i64())
        };
        Consts { vcc: bit("PSEUDO_VCC"), gnd: bit("PSEUDO_GND") }
    }

    fn drive(&self, cell: &Value, port: &str) -> Drive {
        let Some(b) = cell["connections"][port].as_array().and_then(|a| a.first()) else { return Drive::None };
        match b {
            Value::String(s) if s == "1" => Drive::Const(true),
            Value::String(s) if s == "0" => Drive::Const(false),
            Value::Number(n) if n.as_i64() == self.vcc => Drive::Const(true),
            Value::Number(n) if n.as_i64() == self.gnd => Drive::Const(false),
            Value::Number(_) => Drive::Net,
            _ => Drive::None,
        }
    }
}

/// The control set every FF of a half slice must share.
#[derive(Debug, Default)]
struct Ctl {
    set: Option<(bool, bool, bool, bool, bool, bool)>, // negedge, latch, sync, clkinv, srused, ceused
}

/// Slice FASM lines (without the `tile.site.` prefix grouping: each line is
/// complete) for every half slice of every CLB tile holding a logic cell.
pub fn slice_config(placed: &Value, locate: &dyn Fn(&str) -> Option<Located>) -> R<BTreeMap<Half, BTreeSet<String>>> {
    let mods = placed["modules"].as_object().ok_or("no modules")?;
    let top = mods
        .values()
        .find(|m| m["attributes"]["top"].is_string() || m["attributes"]["top"].is_number())
        .or_else(|| mods.values().next())
        .ok_or("no top module")?;
    let cells = top["cells"].as_object().ok_or("no cells")?;
    let consts = Consts::of(cells);

    let mut out: BTreeMap<Half, BTreeSet<String>> = BTreeMap::new();
    let mut ctl: HashMap<Half, Ctl> = HashMap::new();
    // (half, LUT letter) -> (small, ram, srl); half -> (wa7, wa8).
    let mut lut_mode: BTreeMap<(Half, char), (bool, bool, bool)> = BTreeMap::new();
    let mut wa: BTreeMap<Half, (bool, bool)> = BTreeMap::new();
    let mut tiles: BTreeSet<(String, String)> = BTreeSet::new(); // (tile, tile type)

    for (name, c) in cells {
        let attrs = &c["attributes"];
        let Some(bel) = attrs["NEXTPNR_BEL"].as_str() else { continue };
        let Some((tile, tile_type, np_site, bel)) = locate(bel) else { continue };
        if !tile_type.starts_with("CLBL") {
            continue;
        }
        let Some(site) = slice_name(&tile_type, &np_site) else { continue };
        tiles.insert((tile.clone(), tile_type.clone()));
        let half: Half = (tile.clone(), site.clone());
        let ty = attrs["X_ORIG_TYPE"].as_str().unwrap_or("");
        let params = &c["parameters"];

        if bel.ends_with("FF") {
            // (zrst, negedge, latch, sync)
            let (zrst, negedge, latch, sync) = match ty {
                "FDRE" => (true, false, false, true),
                "FDRE_1" => (true, true, false, true),
                "FDSE" => (false, false, false, true),
                "FDSE_1" => (false, true, false, true),
                "FDCE" => (true, false, false, false),
                "FDCE_1" => (true, true, false, false),
                "FDPE" => (false, false, false, false),
                "FDPE_1" => (false, true, false, false),
                "LDCE" => (true, false, true, false),
                "LDPE" => (false, false, true, false),
                t => return Err(format!("{name}: unsupported FF type {t:?} at {tile}/{site}.{bel}")),
            };
            // Vivado omits INIT at its default: 1 for the set/preset types.
            let def_init = if matches!(ty, "FDSE" | "FDSE_1" | "FDPE" | "FDPE_1") { 1 } else { 0 };
            let init = params.get("INIT").and_then(int_param).unwrap_or(def_init);
            let lines = out.entry(half.clone()).or_default();
            if init != 1 {
                lines.insert(format!("{tile}.{site}.{bel}.ZINI"));
            }
            if zrst {
                lines.insert(format!("{tile}.{site}.{bel}.ZRST"));
            }
            let inv = params.get("IS_CLK_INVERTED").and_then(int_param).unwrap_or(0) == 1;
            // A 7-series latch is transparent while its clock is low: the
            // gate is inverted by default so G high is transparent.
            let clkinv = if negedge { true } else if latch { !inv } else { inv };
            let srused = consts.drive(c, "SR") == Drive::Net || consts.drive(c, "SR") == Drive::Const(true);
            let ceused = consts.drive(c, "CE") == Drive::Net || consts.drive(c, "CE") == Drive::Const(false);
            let mine = (negedge, latch, sync, clkinv, srused, ceused);
            let e = ctl.entry(half.clone()).or_default();
            match e.set {
                None => e.set = Some(mine),
                Some(s) if s != mine => {
                    return Err(format!("{name}: FF at {tile}/{site}.{bel} disagrees with its half slice's control set ({s:?} vs {mine:?})"))
                }
                Some(_) => {}
            }
        } else if bel.len() == 5 && bel.ends_with("LUT") {
            let letter = bel.chars().next().unwrap_or('?');
            let m = lut_mode.entry((half.clone(), letter)).or_default();
            match ty {
                "RAMD64E" | "RAMS64E" => m.1 = true,
                "RAMD32" | "RAMS32" => {
                    m.1 = true;
                    m.0 = true;
                }
                "SRL16E" => {
                    m.2 = true;
                    m.0 = true;
                }
                "SRLC32E" => m.2 = true,
                _ => {}
            }
            let w = wa.entry(half.clone()).or_default();
            w.0 |= consts.drive(c, "WA7") != Drive::None;
            w.1 |= consts.drive(c, "WA8") != Drive::None;
        } else if bel == "CARRY4" {
            let cin = consts.drive(c, "CIN");
            let cyinit = consts.drive(c, "CYINIT");
            let line = match (params.get("PRECYINIT_CONST").and_then(int_param), cin, cyinit) {
                (Some(k), _, _) if k >= 0 => if k != 0 { "PRECYINIT.C1" } else { "PRECYINIT.C0" },
                (_, Drive::Net, _) => "PRECYINIT.CIN",
                // A dynamic carry-in comes in on AX: the routing bel's
                // setting (`apply_site_muxes`), PRECYINIT.AX.
                (_, _, Drive::Net) => "PRECYINIT.AX",
                (_, _, Drive::Const(true)) => "PRECYINIT.C1",
                (_, Drive::Const(true), Drive::None) => "PRECYINIT.C1",
                _ => "PRECYINIT.C0",
            };
            out.entry(half.clone()).or_default().insert(format!("{tile}.{site}.{line}"));
        }
    }

    // Every half of every used tile: its control set (NOCLKINV with no FF).
    for (tile, tile_type) in &tiles {
        for np in ["SLICE_X0", "SLICE_X1"] {
            let Some(site) = slice_name(tile_type, np) else { continue };
            let half: Half = (tile.clone(), site.clone());
            let lines = out.entry(half.clone()).or_default();
            let (_, latch, sync, clkinv, srused, ceused) = ctl.get(&half).and_then(|c| c.set).unwrap_or_default();
            for (on, what) in [(latch, "LATCH"), (sync, "FFSYNC"), (clkinv, "CLKINV"), (!clkinv, "NOCLKINV"), (srused, "SRUSEDMUX"), (ceused, "CEUSEDMUX")] {
                if on {
                    lines.insert(format!("{tile}.{site}.{what}"));
                }
            }
        }
    }
    for (((tile, site), letter), (small, ram, srl)) in lut_mode {
        let lines = out.entry((tile.clone(), site.clone())).or_default();
        for (on, what) in [(small, "SMALL"), (ram, "RAM"), (srl, "SRL")] {
            if on {
                lines.insert(format!("{tile}.{site}.{letter}LUT.{what}"));
            }
        }
    }
    for ((tile, site), (wa7, wa8)) in wa {
        let lines = out.entry((tile.clone(), site.clone())).or_default();
        if wa7 {
            lines.insert(format!("{tile}.{site}.WA7USED"));
        }
        if wa8 {
            lines.insert(format!("{tile}.{site}.WA8USED"));
        }
    }
    Ok(out)
}

/// The tie-offs nextpnr-xilinx's DSP packer leaves on a DSP48E1 cell
/// (`DSP_GND_PINS`, `DSP_VCC_PINS`: inputs the netlist leaves open), as
/// `write_dsp_cell` writes them: `TILE.DSP_<n>_<PIN>.DSP_<GND|VCC>_<side>`,
/// the value flipped where `IS_<PIN>_INVERTED` is set. `<n>` is the site's
/// place in the tile (odd Y: 1), `<side>` the tile's (`DSP_L`, `DSP_R`).
/// These are pips of the DSP tile, not routing: the router leaves the pin
/// alone.
pub fn dsp_ties(placed: &Value, locate: &dyn Fn(&str) -> Option<Located>) -> R<BTreeSet<String>> {
    let mods = placed["modules"].as_object().ok_or("no modules")?;
    let top = mods
        .values()
        .find(|m| m["attributes"]["top"].is_string() || m["attributes"]["top"].is_number())
        .or_else(|| mods.values().next())
        .ok_or("no top module")?;
    let mut out = BTreeSet::new();
    for (name, c) in top["cells"].as_object().ok_or("no cells")? {
        let attrs = &c["attributes"];
        if attrs["X_ORIG_TYPE"].as_str() != Some("DSP48E1") {
            continue;
        }
        let Some(bel) = attrs["NEXTPNR_BEL"].as_str() else { continue };
        let Some((tile, _, _, _)) = locate(bel) else { return Err(format!("{name}: no tile for {bel}")) };
        let side = tile.chars().nth(4).filter(|c| *c == 'L' || *c == 'R').ok_or_else(|| format!("{name}: {tile} is not a DSP_L/DSP_R tile"))?;
        let y: i64 = bel.split('/').next().and_then(|s| s.rsplit_once('Y')).and_then(|(_, y)| y.parse().ok()).ok_or_else(|| format!("{name}: site of {bel}"))?;
        let n = y & 1;
        for (val, attr) in [(false, "DSP_GND_PINS"), (true, "DSP_VCC_PINS")] {
            for pin in attrs[attr].as_str().unwrap_or("").split(' ').filter(|p| !p.is_empty()) {
                let inv = c["parameters"].get(format!("IS_{pin}_INVERTED")).and_then(int_param).unwrap_or(0) != 0;
                let tie = if val != inv { "VCC" } else { "GND" };
                out.insert(format!("{tile}.DSP_{n}_{pin}.DSP_{tie}_{side}"));
            }
        }
    }
    Ok(out)
}

/// A parameter as a bit vector of `width` (bit 0 first): a binary string
/// (most significant bit first) or a number.
fn bits_param(v: &Value, width: usize) -> Option<Vec<bool>> {
    let mut out = vec![false; width];
    match v {
        Value::String(s) if !s.is_empty() && s.chars().all(|c| c == '0' || c == '1') => {
            for (i, c) in s.chars().rev().enumerate().take(width) {
                out[i] = c == '1';
            }
        }
        _ => {
            let n = int_param(v)?;
            for (i, b) in out.iter_mut().enumerate().take(64) {
                *b = (n >> i) & 1 == 1;
            }
        }
    }
    Some(out)
}

/// A FASM vector value, `N'b` with the most significant bit first.
fn bits_value(bits: &[bool]) -> String {
    let s: String = bits.iter().rev().map(|&b| if b { '1' } else { '0' }).collect();
    format!("{}'b{s}", bits.len())
}

/// DSP48E1 cells' settings, as nextpnr-himbaechel's `write_dsp_cell`
/// writes them (`TILE.DSP48.DSP_<n>.*`): pipeline registers (`AREG_0`,
/// `AREG_2`, `Z*REG[0]`), cascade inputs, D port and SIMD use, PATTERN,
/// MASK and their options, and the control-bus and clock inversions
/// (`ZIS_*_INVERTED`). The tie-offs of open inputs are `dsp_ties`.
pub fn dsp_config(placed: &Value, locate: &dyn Fn(&str) -> Option<Located>) -> R<BTreeSet<String>> {
    let mods = placed["modules"].as_object().ok_or("no modules")?;
    let top = mods
        .values()
        .find(|m| m["attributes"]["top"].is_string() || m["attributes"]["top"].is_number())
        .or_else(|| mods.values().next())
        .ok_or("no top module")?;
    let mut out = BTreeSet::new();
    for (name, c) in top["cells"].as_object().ok_or("no cells")? {
        let attrs = &c["attributes"];
        if attrs["X_ORIG_TYPE"].as_str() != Some("DSP48E1") {
            continue;
        }
        let Some(bel) = attrs["NEXTPNR_BEL"].as_str() else { continue };
        let Some((tile, _, _, _)) = locate(bel) else { return Err(format!("{name}: no tile for {bel}")) };
        let y: i64 = bel.split('/').next().and_then(|s| s.rsplit_once('Y')).and_then(|(_, y)| y.parse().ok()).ok_or_else(|| format!("{name}: site of {bel}"))?;
        let pre = format!("{tile}.DSP48.DSP_{}", y & 1);
        let params = &c["parameters"];
        let int = |p: &str, d: i64| params.get(p).and_then(int_param).unwrap_or(d);
        let flag = |p: &str, d: bool| params.get(p).and_then(int_param).map(|v| v != 0).unwrap_or(d);
        let text = |p: &str, d: &str| params.get(p).and_then(Value::as_str).map(str::to_string).unwrap_or_else(|| d.to_string());
        let boolstr = |p: &str| match params.get(p) {
            Some(Value::String(s)) if s == "TRUE" => true,
            Some(Value::String(s)) if s == "FALSE" => false,
            Some(v) => int_param(v).is_some_and(|n| n != 0),
            None => false,
        };
        let mut put = |f: String| {
            out.insert(format!("{pre}.{f}"));
        };

        // 1 is the same as 2 (UG479); Vivado writes AREG_0, nothing, AREG_2.
        for reg in ["AREG", "BREG"] {
            let v = int(reg, 1);
            if v == 0 || v == 2 {
                put(format!("{reg}_{v}"));
            }
        }
        for (p, f) in [("A_INPUT", "A_INPUT[0]"), ("B_INPUT", "B_INPUT[0]")] {
            if text(p, "DIRECT") == "CASCADE" {
                put(f.into());
            }
        }
        if boolstr("USE_DPORT") {
            put("USE_DPORT[0]".into());
        }
        match text("USE_SIMD", "ONE48").as_str() {
            "TWO24" => put("USE_SIMD_FOUR12_TWO24".into()),
            "FOUR12" => put("USE_SIMD_FOUR12".into()),
            _ => {}
        }
        if let Some(bits) = params.get("PATTERN").and_then(|v| bits_param(v, 48)) {
            put(format!("PATTERN[47:0] = {}", bits_value(&bits)));
        }
        match text("AUTORESET_PATDET", "NO_RESET").as_str() {
            "RESET_MATCH" => put("AUTORESET_PATDET_RESET".into()),
            "RESET_NOT_MATCH" => put("AUTORESET_PATDET_RESET_NOT_MATCH".into()),
            _ => {}
        }
        // prjxray knows 46 MASK bits; the top two are 0 in practice.
        if let Some(mut bits) = params.get("MASK").and_then(|v| bits_param(v, 48)) {
            bits.truncate(46);
            put(format!("MASK[45:0] = {}", bits_value(&bits)));
        }
        match text("SEL_MASK", "MASK").as_str() {
            "C" => put("SEL_MASK_C".into()),
            "ROUNDING_MODE1" => put("SEL_MASK_ROUNDING_MODE1".into()),
            "ROUNDING_MODE2" => put("SEL_MASK_ROUNDING_MODE2".into()),
            _ => {}
        }
        for (p, d, f) in [
            ("ADREG", true, "ZADREG[0]"),
            ("ALUMODEREG", false, "ZALUMODEREG[0]"),
            ("ACASCREG", false, "ZAREG_2_ACASCREG_1"),
            ("BCASCREG", false, "ZBREG_2_BCASCREG_1"),
            ("CARRYINREG", false, "ZCARRYINREG[0]"),
            ("CARRYINSELREG", false, "ZCARRYINSELREG[0]"),
            ("CREG", true, "ZCREG[0]"),
            ("DREG", true, "ZDREG[0]"),
            ("INMODEREG", false, "ZINMODEREG[0]"),
            ("MREG", false, "ZMREG[0]"),
            ("OPMODEREG", false, "ZOPMODEREG[0]"),
            ("PREG", false, "ZPREG[0]"),
            ("IS_CLK_INVERTED", false, "ZIS_CLK_INVERTED"),
            ("IS_CARRYIN_INVERTED", false, "ZIS_CARRYIN_INVERTED"),
        ] {
            if !flag(p, d) {
                put(f.into());
            }
        }
        for (bus, width) in [("ALUMODE", 4), ("INMODE", 5), ("OPMODE", 7)] {
            for i in 0..width {
                let inv = (int(&format!("IS_{bus}_INVERTED"), 0) >> i) & 1 == 1 || flag(&format!("IS_{bus}[{i}]_INVERTED"), false);
                if !inv {
                    put(format!("ZIS_{bus}_INVERTED[{i}]"));
                }
            }
        }
    }
    Ok(out)
}

/// Whether a FASM line is a DSP48E1 setting `dsp_config` writes.
pub fn is_dsp_config(line: &str) -> bool {
    let f = line.split('#').next().unwrap_or("").split('=').next().unwrap_or("").trim();
    let parts: Vec<&str> = f.split('.').collect();
    matches!(parts[..], [t, "DSP48", d, _] if t.starts_with("DSP_") && d.starts_with("DSP_"))
}

/// BUFGCTRL cells, as nextpnr-himbaechel's `write_clocking` writes them:
/// `TILE.BUFGCTRL.<site>.IN_USE`, `INIT_OUT`, `IS_IGNORE0_INVERTED`,
/// `IS_IGNORE1_INVERTED` where those parameters are set, and `ZINV_CE0`,
/// `ZINV_CE1`, `ZINV_S0`, `ZINV_S1` where the matching `IS_*_INVERTED` is
/// not. `<site>` is the BUFGCTRL's place in its tile (`BUFGCTRL_X0Y5`).
pub fn bufg_config(placed: &Value, locate: &dyn Fn(&str) -> Option<Located>) -> R<BTreeSet<String>> {
    let mods = placed["modules"].as_object().ok_or("no modules")?;
    let top = mods
        .values()
        .find(|m| m["attributes"]["top"].is_string() || m["attributes"]["top"].is_number())
        .or_else(|| mods.values().next())
        .ok_or("no top module")?;
    let mut out = BTreeSet::new();
    for (name, c) in top["cells"].as_object().ok_or("no cells")? {
        if c["type"].as_str() != Some("BUFGCTRL") {
            continue;
        }
        let Some(bel) = c["attributes"]["NEXTPNR_BEL"].as_str() else { continue };
        let Some((tile, _, site, _)) = locate(bel) else { return Err(format!("{name}: no tile for {bel}")) };
        let set = |p: &str| c["parameters"].get(p).and_then(int_param).unwrap_or(0) != 0;
        let mut put = |f: &str| {
            out.insert(format!("{tile}.BUFGCTRL.{site}.{f}"));
        };
        put("IN_USE");
        for p in ["INIT_OUT", "IS_IGNORE0_INVERTED", "IS_IGNORE1_INVERTED"] {
            if set(p) {
                put(p);
            }
        }
        for pin in ["CE0", "CE1", "S0", "S1"] {
            if !set(&format!("IS_{pin}_INVERTED")) {
                put(&format!("ZINV_{pin}"));
            }
        }
    }
    Ok(out)
}

/// Whether a FASM line is a BUFGCTRL setting `bufg_config` writes.
pub fn is_bufg_config(line: &str) -> bool {
    let parts: Vec<&str> = line.split('.').collect();
    matches!(parts[..], [t, "BUFGCTRL", site, _] if t.starts_with("CLK_BUFG_") && site.starts_with("BUFGCTRL_X"))
}

/// Whether a FASM line is a DSP tie-off `dsp_ties` writes.
pub fn is_dsp_tie(line: &str) -> bool {
    let parts: Vec<&str> = line.split('.').collect();
    matches!(parts[..], [t, pin, tie] if t.starts_with("DSP_") && pin.starts_with("DSP_") && (tie.starts_with("DSP_GND_") || tie.starts_with("DSP_VCC_")))
}

/// Whether a CLB FASM line is one `slice_config` writes (the others: LUT
/// INITs, pips and pseudo-pips, routing-bel settings).
pub fn is_slice_config(line: &str) -> bool {
    let parts: Vec<&str> = line.split('.').collect();
    if parts.len() < 3 || !parts[0].starts_with("CLBL") || !parts[1].starts_with("SLICE") {
        return false;
    }
    let rest = &parts[2..];
    match rest {
        [x] => matches!(*x, "LATCH" | "FFSYNC" | "CLKINV" | "NOCLKINV" | "SRUSEDMUX" | "CEUSEDMUX" | "WA7USED" | "WA8USED"),
        ["PRECYINIT", _] => true,
        [ff, "ZINI" | "ZRST"] => ff.ends_with("FF"),
        [lut, "SMALL" | "RAM" | "SRL"] => lut.ends_with("LUT"),
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn params() {
        assert_eq!(int_param(&Value::String("00000000000000000000000000000001".into())), Some(1));
        assert_eq!(int_param(&Value::String("0".into())), Some(0));
        assert_eq!(int_param(&Value::String("10".into())), Some(2));
        assert_eq!(int_param(&serde_json::json!(5)), Some(5));
    }

    #[test]
    fn slice_lines() {
        assert!(is_slice_config("CLBLM_R_X39Y104.SLICEM_X0.AFF.ZINI"));
        assert!(is_slice_config("CLBLM_R_X39Y104.SLICEM_X0.A5FF.ZRST"));
        assert!(is_slice_config("CLBLL_L_X2Y3.SLICEL_X1.NOCLKINV"));
        assert!(is_slice_config("CLBLL_L_X2Y3.SLICEL_X1.PRECYINIT.C0"));
        assert!(is_slice_config("CLBLM_R_X2Y3.SLICEM_X0.ALUT.RAM"));
        assert!(!is_slice_config("CLBLM_R_X2Y3.SLICEM_X0.ALUT.INIT[63:0]=64'h0"));
        assert!(!is_slice_config("CLBLM_R_X2Y3.SLICEM_X0.AFFMUX.O6"));
        assert!(!is_slice_config("CLBLL_L_X20Y101.CLBLL_LL_A6.CLBLL_IMUX4"));
        assert!(is_dsp_tie("DSP_R_X59Y65.DSP_1_RSTD.DSP_GND_R"));
        assert!(is_dsp_tie("DSP_L_X34Y65.DSP_0_CEAD.DSP_VCC_L"));
        assert!(!is_dsp_tie("DSP_R_X59Y65.DSP_1_A0.DSP_IMUX5_1"));
        assert!(is_bufg_config("CLK_BUFG_TOP_R_X87Y53.BUFGCTRL.BUFGCTRL_X0Y5.ZINV_CE0"));
        assert!(is_dsp_config("DSP_R_X59Y60.DSP48.DSP_0.ZIS_OPMODE_INVERTED[3]"));
        assert!(is_dsp_config("DSP_R_X59Y60.DSP48.DSP_1.MASK[45:0] = 46'b0"));
        assert!(!is_dsp_config("DSP_R_X59Y60.DSP_0_CEAD.DSP_VCC_R"));
        assert_eq!(bits_param(&Value::String("101".into()), 4), Some(vec![true, false, true, false]));
        assert_eq!(bits_value(&[true, false, true, false]), "4'b0101");
        assert!(!is_bufg_config("CLK_BUFG_TOP_R_X87Y53.CLK_BUFG_BUFGCTRL0_I0.CLK_BUFG_IMUX28_0"));
    }
}
