//! Re-target a routed design to new DSP48E1 settings without placing or
//! routing it again.
//!
//! A DSP48E1's static settings (register stages, SIMD, pattern detector,
//! the per-pin inverters that also carry constant OPMODE/ALUMODE/INMODE/
//! CARRYIN bits) are features of its own site, and its tied pins are pips
//! inside its own tile. When two designs differ only in those, the routed
//! FASM of one becomes the other's by rewriting the DSP tiles.
//!
//! Inputs, all nextpnr/yosys JSON:
//! * the template's routed netlist (placement, and its parameters for the
//!   self-check),
//! * the new design's packed netlist (`nextpnr --pack-only --write`): the
//!   parameters the FASM writer would see,
//! * optionally both designs' synthesized netlists, to check that they
//!   differ only in DSP parameters and in constant values on DSP pins.
//!
//! [`features`] follows nextpnr-xilinx `write_dsp_cell` line for line, with
//! MASK written at its full 48 bits (prjxray has `MASK[47:0]`). Before
//! patching, [`self_check`] regenerates every template DSP tile and compares
//! it bit for bit with the template FASM, so a template written by a
//! different writer is refused rather than silently mixed.

use std::collections::{BTreeMap, BTreeSet};

use oxibit_fasm::db::Database;
use oxibit_fasm::parse::parse_line;
use serde_json::Value;

type R<T> = Result<T, String>;

/// One DSP48E1 cell of a netlist.
#[derive(Debug, Clone)]
pub struct DspCell {
    pub params: BTreeMap<String, String>,
    pub attrs: BTreeMap<String, String>,
}

impl DspCell {
    /// Site from nextpnr's placement attribute (`DSP48_X0Y12/DSP48E1`).
    pub fn site(&self) -> Option<&str> {
        self.attrs.get("NEXTPNR_BEL").map(|b| b.split('/').next().unwrap_or(b))
    }
}

/// The top module of a yosys or nextpnr JSON netlist.
pub fn top_module(netlist: &Value) -> R<&Value> {
    let mods = netlist["modules"].as_object().ok_or("netlist has no modules")?;
    let top = mods
        .values()
        .find(|m| m["attributes"]["top"].as_str().is_some_and(|t| t.trim_start_matches('0') == "1"))
        .or_else(|| mods.get("top"))
        .or_else(|| (mods.len() == 1).then(|| mods.values().next()).flatten());
    top.ok_or_else(|| "cannot tell the top module".to_string())
}

/// DSP48E1 cells by name (yosys type `DSP48E1`, nextpnr `DSP48E1_DSP48E1`).
pub fn dsp_cells(netlist: &Value) -> R<BTreeMap<String, DspCell>> {
    let cells = top_module(netlist)?["cells"].as_object().ok_or("top module has no cells")?;
    let strings = |v: &Value| -> BTreeMap<String, String> {
        v.as_object()
            .map(|o| o.iter().map(|(k, v)| (k.clone(), v.as_str().map(String::from).unwrap_or_else(|| v.to_string()))).collect())
            .unwrap_or_default()
    };
    Ok(cells
        .iter()
        .filter(|(_, c)| c["type"].as_str().is_some_and(|t| t.starts_with("DSP48E1")))
        .map(|(n, c)| (n.clone(), DspCell { params: strings(&c["parameters"]), attrs: strings(&c["attributes"]) }))
        .collect())
}

/// A parameter value as nextpnr reads it: a bit string (MSB first, `x`/`z`
/// read as 0) or a string (yosys marks a string of 0s and 1s with a
/// trailing space).
enum PVal {
    Bits(Vec<bool>),
    Str(String),
}

fn pval(s: &str) -> PVal {
    if !s.is_empty() && s.bytes().all(|b| matches!(b, b'0' | b'1' | b'x' | b'z')) {
        PVal::Bits(s.bytes().rev().map(|b| b == b'1').collect())
    } else {
        PVal::Str(s.strip_suffix(' ').unwrap_or(s).to_string())
    }
}

struct Params<'a>(&'a BTreeMap<String, String>);

impl Params<'_> {
    fn get(&self, n: &str) -> Option<PVal> {
        self.0.get(n).map(|s| pval(s))
    }
    fn int(&self, n: &str, d: u64) -> u64 {
        match self.get(n) {
            None => d,
            Some(PVal::Bits(b)) => b.iter().take(64).enumerate().fold(0, |a, (i, &x)| a | (u64::from(x) << i)),
            Some(PVal::Str(s)) => s.parse().unwrap_or(d),
        }
    }
    fn bool(&self, n: &str, d: bool) -> bool {
        match self.get(n) {
            None => d,
            Some(PVal::Bits(b)) => b.iter().any(|&x| x),
            Some(PVal::Str(s)) => s.eq_ignore_ascii_case("TRUE"),
        }
    }
    fn str(&self, n: &str, d: &str) -> String {
        match self.get(n) {
            Some(PVal::Str(s)) => s,
            _ => d.to_string(),
        }
    }
    /// `boolvec_populate`: `None` when absent, else `width` bits, LSB first.
    fn bitvec(&self, n: &str, width: usize, fill: bool) -> Option<Vec<bool>> {
        let PVal::Bits(b) = self.get(n)? else { return None };
        let mut v = vec![fill; width];
        for (i, &x) in b.iter().take(width).enumerate() {
            v[i] = x;
        }
        Some(v)
    }
}

/// The FASM lines nextpnr writes for one DSP48E1 at `DSP_<idx>` of `tile`
/// (`side` is the tile's `L`/`R`): site features, then tied-pin pips.
pub fn features(cell: &DspCell, tile: &str, idx: u8, side: char) -> Vec<String> {
    let p = Params(&cell.params);
    let pre = format!("{tile}.DSP48.DSP_{idx}.");
    let mut f = Vec::new();
    let bit = |f: &mut Vec<String>, name: &str, on: bool| {
        if on {
            f.push(format!("{pre}{name}"));
        }
    };
    let vec = |f: &mut Vec<String>, name: &str, v: &[bool]| {
        let bits: String = v.iter().rev().map(|&b| if b { '1' } else { '0' }).collect();
        f.push(format!("{pre}{name} = {}'b{bits}", v.len()));
    };

    let areg = p.int("AREG", 1);
    bit(&mut f, &format!("AREG_{areg}"), areg == 0 || areg == 2);
    bit(&mut f, "A_INPUT[0]", p.str("A_INPUT", "DIRECT") == "CASCADE");
    let breg = p.int("BREG", 1);
    bit(&mut f, &format!("BREG_{breg}"), breg == 0 || breg == 2);
    bit(&mut f, "B_INPUT[0]", p.str("B_INPUT", "DIRECT") == "CASCADE");
    bit(&mut f, "USE_DPORT[0]", p.bool("USE_DPORT", false));
    let simd = p.str("USE_SIMD", "ONE48");
    bit(&mut f, "USE_SIMD_FOUR12_TWO24", simd == "TWO24" || simd == "FOUR12");
    bit(&mut f, "USE_SIMD_FOUR12", simd == "FOUR12");
    if let Some(v) = p.bitvec("PATTERN", 48, false) {
        vec(&mut f, "PATTERN[47:0]", &v);
    }
    let ar = p.str("AUTORESET_PATDET", "NO_RESET");
    // RESET enables autoreset, RESET_NOT_MATCH selects the not-match condition. Silicon: RESET_NOT_MATCH alone
    // never resets P (gap_pd/gap_rnm/gap_m46); with both set it resets on not-match (patdet_fix).
    bit(&mut f, "AUTORESET_PATDET_RESET", ar == "RESET_MATCH" || ar == "RESET_NOT_MATCH");
    bit(&mut f, "AUTORESET_PATDET_RESET_NOT_MATCH", ar == "RESET_NOT_MATCH");
    if let Some(v) = p.bitvec("MASK", 48, true) {
        vec(&mut f, "MASK[47:0]", &v);
    }
    let sm = p.str("SEL_MASK", "MASK");
    bit(&mut f, "SEL_MASK_C", sm == "C");
    bit(&mut f, "SEL_MASK_ROUNDING_MODE1", sm == "ROUNDING_MODE1");
    bit(&mut f, "SEL_MASK_ROUNDING_MODE2", sm == "ROUNDING_MODE2");
    bit(&mut f, "ZADREG[0]", !p.bool("ADREG", true));
    bit(&mut f, "ZALUMODEREG[0]", !p.bool("ALUMODEREG", false));
    // Set unless AREG = 2 and ACASCREG = 1 (silicon: clear selects A1 for ACOUT; nextpnr patch 0004).
    let acascreg = p.int("ACASCREG", 1);
    bit(&mut f, "ZAREG_2_ACASCREG_1", !(areg == 2 && acascreg == 1));
    let bcascreg = p.int("BCASCREG", 1);
    bit(&mut f, "ZBREG_2_BCASCREG_1", !(breg == 2 && bcascreg == 1));
    bit(&mut f, "ZCARRYINREG[0]", !p.bool("CARRYINREG", false));
    bit(&mut f, "ZCARRYINSELREG[0]", !p.bool("CARRYINSELREG", false));
    bit(&mut f, "ZCREG[0]", !p.bool("CREG", true));
    bit(&mut f, "ZDREG[0]", !p.bool("DREG", true));
    bit(&mut f, "ZINMODEREG[0]", !p.bool("INMODEREG", false));
    for (bus, width) in [("ALUMODE", 4), ("INMODE", 5), ("OPMODE", 7)] {
        let whole = p.int(&format!("IS_{bus}_INVERTED"), 0);
        for i in 0..width {
            let inv = (whole >> i) & 1 == 1 || p.bool(&format!("IS_{bus}[{i}]_INVERTED"), false);
            bit(&mut f, &format!("ZIS_{bus}_INVERTED[{i}]"), !inv);
        }
    }
    bit(&mut f, "ZMREG[0]", !p.bool("MREG", false));
    bit(&mut f, "ZOPMODEREG[0]", !p.bool("OPMODEREG", false));
    bit(&mut f, "ZPREG[0]", !p.bool("PREG", false));
    bit(&mut f, "ZIS_CLK_INVERTED", !p.bool("IS_CLK_INVERTED", false));
    bit(&mut f, "ZIS_CARRYIN_INVERTED", !p.bool("IS_CARRYIN_INVERTED", false));

    // Tied pins. nextpnr looks the pin's inverter up as IS_<pin>_INVERTED with
    // the pin name as written (its digit strip removes only the literal
    // "0123456789"), so only CLK and CARRYIN can flip a tie.
    for konst in ["GND", "VCC"] {
        let pins = cell.attrs.get(&format!("DSP_{konst}_PINS")).map(String::as_str).unwrap_or("");
        for pin in pins.split_whitespace() {
            let base = pin.replace("0123456789", "");
            let inv = p.bool(&format!("IS_{base}_INVERTED"), false);
            let net = match (konst, inv) {
                ("GND", true) => "VCC",
                ("VCC", true) => "GND",
                _ => konst,
            };
            f.push(format!("{tile}.DSP_{idx}_{pin}.DSP_{net}_{side}"));
        }
    }
    f
}

/// DSP site -> (tile, DSP_0/DSP_1, tile side), from the tilegrid.
pub fn dsp_sites(db: &Database) -> BTreeMap<String, (String, u8, char)> {
    let y = |s: &str| s.rsplit('Y').next().and_then(|n| n.parse::<u32>().ok()).unwrap_or(0);
    let mut out = BTreeMap::new();
    for (tile, t) in &db.tiles {
        let side = match t.tile_type.as_str() {
            "DSP_L" => 'L',
            "DSP_R" => 'R',
            _ => continue,
        };
        let mut sites: Vec<&String> = t.sites.iter().filter(|s| s.starts_with("DSP48_")).collect();
        sites.sort_by_key(|s| y(s));
        for (i, s) in sites.into_iter().enumerate() {
            out.insert(s.clone(), (tile.clone(), i as u8, side));
        }
    }
    out
}

/// FASM lines (or one line) -> the single-bit features they set.
pub fn set_bits<'a>(lines: impl IntoIterator<Item = &'a str>) -> R<BTreeSet<String>> {
    let mut out = BTreeSet::new();
    for l in lines {
        let Some(sf) = parse_line(l)? else { continue };
        match (sf.start, sf.end) {
            (None, _) => {
                if !sf.is_zero() {
                    out.insert(sf.feature);
                }
            }
            (Some(s), None) => {
                if !sf.is_zero() {
                    out.insert(format!("{}[{s}]", sf.feature));
                }
            }
            (Some(_), Some(_)) => {
                for a in sf.addresses() {
                    out.insert(format!("{}[{a}]", sf.feature));
                }
            }
        }
    }
    Ok(out)
}

fn tile_of(line: &str) -> Option<&str> {
    let t = line.split('#').next()?.trim().split('.').next()?;
    (t.starts_with("DSP_L_") || t.starts_with("DSP_R_")).then_some(t)
}

/// Template DSP lines by tile.
fn dsp_lines(fasm: &str) -> BTreeMap<&str, Vec<&str>> {
    let mut m: BTreeMap<&str, Vec<&str>> = BTreeMap::new();
    for l in fasm.lines() {
        if let Some(t) = tile_of(l) {
            m.entry(t).or_default().push(l);
        }
    }
    m
}

/// Placement of every template DSP: cell -> (tile, idx, side).
pub fn placement(cells: &BTreeMap<String, DspCell>, sites: &BTreeMap<String, (String, u8, char)>) -> R<BTreeMap<String, (String, u8, char)>> {
    cells
        .iter()
        .map(|(n, c)| {
            let s = c.site().ok_or_else(|| format!("{n}: no NEXTPNR_BEL (not a placed netlist)"))?;
            let at = sites.get(s).ok_or_else(|| format!("{n}: {s} is not a DSP site of this part"))?;
            Ok((n.clone(), at.clone()))
        })
        .collect()
}

fn lines_by_tile(cells: &BTreeMap<String, DspCell>, place: &BTreeMap<String, (String, u8, char)>) -> BTreeMap<String, Vec<String>> {
    let mut m: BTreeMap<String, Vec<String>> = BTreeMap::new();
    for (n, c) in cells {
        if let Some((tile, idx, side)) = place.get(n) {
            m.entry(tile.clone()).or_default().extend(features(c, tile, *idx, *side));
        }
    }
    m
}

/// A tile whose regenerated features differ from the template FASM.
#[derive(Debug, Clone)]
pub struct Mismatch {
    pub tile: String,
    pub only_regenerated: Vec<String>,
    pub only_template: Vec<String>,
}

/// Regenerate the template's DSP tiles from its own netlist and compare.
pub fn self_check(fasm: &str, cells: &BTreeMap<String, DspCell>, place: &BTreeMap<String, (String, u8, char)>) -> R<Vec<Mismatch>> {
    let ours = lines_by_tile(cells, place);
    let theirs = dsp_lines(fasm);
    let tiles: BTreeSet<&str> = ours.keys().map(String::as_str).chain(theirs.keys().copied()).collect();
    let mut bad = Vec::new();
    for t in tiles {
        let a = set_bits(ours.get(t).into_iter().flatten().map(String::as_str))?;
        let b = set_bits(theirs.get(t).into_iter().flatten().copied())?;
        if a != b {
            bad.push(Mismatch {
                tile: t.to_string(),
                only_regenerated: a.difference(&b).cloned().collect(),
                only_template: b.difference(&a).cloned().collect(),
            });
        }
    }
    Ok(bad)
}

/// The template FASM with every placed DSP tile rewritten from `cells`.
/// Non-DSP lines keep their order; the DSP tiles follow, sorted.
pub fn patch(fasm: &str, cells: &BTreeMap<String, DspCell>, place: &BTreeMap<String, (String, u8, char)>) -> String {
    let ours = lines_by_tile(cells, place);
    let mut out = String::with_capacity(fasm.len());
    for l in fasm.lines() {
        if tile_of(l).is_some_and(|t| ours.contains_key(t)) {
            continue;
        }
        out.push_str(l);
        out.push('\n');
    }
    for lines in ours.values() {
        for l in lines {
            out.push_str(l);
            out.push('\n');
        }
    }
    out
}

/// Check that two synthesized netlists differ only in DSP48E1 parameters
/// and in constant values on DSP48E1 pins (which packing turns into site
/// features and tie pips). `Err` names the first difference.
pub fn same_structure(a: &Value, b: &Value) -> R<()> {
    let (ma, mb) = (top_module(a)?, top_module(b)?);
    if ma["ports"] != mb["ports"] {
        return Err("top-level ports differ".into());
    }
    let (ca, cb) = (ma["cells"].as_object().ok_or("no cells")?, mb["cells"].as_object().ok_or("no cells")?);
    if ca.len() != cb.len() {
        return Err(format!("cell count differs ({} vs {})", ca.len(), cb.len()));
    }
    let constant = |v: &Value| v.as_array().is_some_and(|bits| bits.iter().all(Value::is_string));
    for (name, x) in ca {
        let y = cb.get(name).ok_or_else(|| format!("{name}: only in the template"))?;
        if x["type"] != y["type"] {
            return Err(format!("{name}: cell type differs"));
        }
        let dsp = x["type"].as_str().is_some_and(|t| t.starts_with("DSP48E1"));
        if !dsp && x["parameters"] != y["parameters"] {
            return Err(format!("{name}: parameters differ"));
        }
        let (px, py) = (x["connections"].as_object(), y["connections"].as_object());
        let ports: BTreeSet<&String> = px.into_iter().flatten().chain(py.into_iter().flatten()).map(|(k, _)| k).collect();
        for port in ports {
            let (vx, vy) = (&x["connections"][port], &y["connections"][port]);
            if vx == vy || (dsp && constant(vx) && constant(vy)) {
                continue;
            }
            return Err(format!("{name}.{port}: connection differs"));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cell(params: &[(&str, &str)], attrs: &[(&str, &str)]) -> DspCell {
        DspCell {
            params: params.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect(),
            attrs: attrs.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect(),
        }
    }

    #[test]
    fn registers_inverters_and_ties() {
        let c = cell(
            &[
                ("AREG", "00000000000000000000000000000000"),
                ("BREG", "00000000000000000000000000000010"),
                ("PREG", "00000000000000000000000000000001"),
                ("ADREG", "00000000000000000000000000000000"),
                ("USE_SIMD", "FOUR12"),
                ("IS_OPMODE[1]_INVERTED", "00000000000000000000000000000001"),
                ("IS_ALUMODE_INVERTED", "0101"),
                ("MASK", "001111111111111111111111111111111111111111111111"),
            ],
            &[("DSP_GND_PINS", " RSTD CARRYINSEL2"), ("DSP_VCC_PINS", " CEAD")],
        );
        let f = features(&c, "DSP_R_X9Y20", 1, 'R');
        let has = |s: &str| f.iter().any(|l| l == &format!("DSP_R_X9Y20.DSP48.DSP_1.{s}"));
        assert!(has("AREG_0") && has("BREG_2") && has("ZADREG[0]"));
        assert!(!has("ZPREG[0]") && has("ZMREG[0]"));
        assert!(has("USE_SIMD_FOUR12") && has("USE_SIMD_FOUR12_TWO24"));
        assert!(!has("ZIS_OPMODE_INVERTED[1]") && has("ZIS_OPMODE_INVERTED[0]"));
        assert!(!has("ZIS_ALUMODE_INVERTED[0]") && has("ZIS_ALUMODE_INVERTED[1]") && !has("ZIS_ALUMODE_INVERTED[2]"));
        assert!(has("MASK[47:0] = 48'b001111111111111111111111111111111111111111111111"));
        assert!(f.contains(&"DSP_R_X9Y20.DSP_1_RSTD.DSP_GND_R".to_string()));
        assert!(f.contains(&"DSP_R_X9Y20.DSP_1_CEAD.DSP_VCC_R".to_string()));
    }

    #[test]
    fn carryin_inverter_flips_its_tie() {
        let c = cell(&[("IS_CARRYIN_INVERTED", "1")], &[("DSP_GND_PINS", " CARRYIN")]);
        let f = features(&c, "DSP_L_X14Y45", 0, 'L');
        assert!(f.contains(&"DSP_L_X14Y45.DSP_0_CARRYIN.DSP_VCC_L".to_string()));
        assert!(!f.iter().any(|l| l.ends_with("ZIS_CARRYIN_INVERTED")));
    }

    #[test]
    fn autoreset_not_match_sets_both_bits() {
        let ar = |v: &str| {
            let f = features(&cell(&[("AUTORESET_PATDET", v)], &[]), "DSP_L_X14Y45", 0, 'L');
            let has = |s: &str| f.iter().any(|l| l == &format!("DSP_L_X14Y45.DSP48.DSP_0.{s}"));
            (has("AUTORESET_PATDET_RESET"), has("AUTORESET_PATDET_RESET_NOT_MATCH"))
        };
        assert_eq!(ar("NO_RESET"), (false, false));
        assert_eq!(ar("RESET_MATCH"), (true, false));
        assert_eq!(ar("RESET_NOT_MATCH"), (true, true));
    }

    #[test]
    fn mask_widths_compare_by_bits() {
        let a = set_bits(["T.DSP48.DSP_0.MASK[45:0] = 46'b11"]).unwrap();
        let b = set_bits(["T.DSP48.DSP_0.MASK[47:0] = 48'b000000000000000000000000000000000000000000000011"]).unwrap();
        assert_eq!(a, b);
    }

    #[test]
    fn structure_allows_dsp_constants_only() {
        let net = |opmode: Value, other: Value| {
            serde_json::json!({"modules": {"top": {"attributes": {"top": "1"}, "ports": {},
                "cells": {
                    "d": {"type": "DSP48E1", "parameters": {}, "connections": {"OPMODE": opmode}},
                    "l": {"type": "LUT2", "parameters": {"INIT": "0110"}, "connections": {"I0": other}}
                }}}})
        };
        let a = net(serde_json::json!(["0", "1"]), serde_json::json!([5]));
        assert!(same_structure(&a, &net(serde_json::json!(["1", "1"]), serde_json::json!([5]))).is_ok());
        assert!(same_structure(&a, &net(serde_json::json!([7, "1"]), serde_json::json!([5]))).is_err());
        assert!(same_structure(&a, &net(serde_json::json!(["0", "1"]), serde_json::json!(["0"]))).is_err());
    }
}
