//! Block RAM configuration from a placed design: the FASM lines
//! nextpnr-xilinx's `write_bram` writes for RAMB18E1 and RAMB36E1 cells,
//! ported from the himbaechel fork (fasm.cc at 655d7df3) with its local
//! silicon fix: a RAMB36E1 with `WRITE_WIDTH_B = 72` writes
//! `WRITE_WIDTH_A_18` on both halves (without it the lower half of an SDP
//! x72 RAMB36 reads back one 9-bit value four times over).
//!
//! The fork's refusal of x9 parity on a RAMB36 is gone too: the fault was
//! on the write side (the packer now ties DIP*DIP1 to DIP*DIP0), not here.
//!
//! The cascade bits (`CASCOUT_ARD_ACTIVE`, `CASCOUT_BWR_ACTIVE`) follow the
//! routing: `cascade_lines` takes the routed pips.

use std::collections::{BTreeSet, HashMap};

use serde_json::Value;

use crate::cellcfg::int_param;
use crate::lutinit::Located;

type R<T> = Result<T, String>;

/// The pins of RAMB18E1 and RAMB36E1 with an `IS_<pin>_INVERTED`
/// parameter (nextpnr's `get_invertible_pins`).
const INVERTIBLE: [&str; 8] = [
    "CLKARDCLK",
    "CLKBWRCLK",
    "ENARDEN",
    "ENBWREN",
    "RSTRAMARSTRAM",
    "RSTRAMB",
    "RSTREGARSTREG",
    "RSTREGB",
];

struct Ram<'a> {
    name: &'a str,
    v: &'a Value,
    is_36: bool,
}

impl Ram<'_> {
    fn int(&self, k: &str) -> R<i64> {
        match self.v["parameters"].get(k) {
            None => Ok(0),
            Some(v) => int_param(v).ok_or_else(|| format!("{}: parameter {k} = {v} is not an integer", self.name)),
        }
    }
    fn flag(&self, k: &str) -> R<bool> {
        self.int(k).map(|i| i != 0)
    }
    fn str_or(&self, k: &str, d: &str) -> String {
        self.v["parameters"][k].as_str().map(String::from).unwrap_or_else(|| d.to_string())
    }
    /// A bit-vector parameter, least significant bit first (nextpnr's
    /// `Property::str`); `None` when unset.
    fn bits(&self, k: &str) -> Option<Vec<bool>> {
        let s = self.v["parameters"].get(k)?.as_str()?;
        Some(s.chars().rev().map(|c| c == '1').collect())
    }
}

/// `write_bram_width` for one width parameter of one half.
fn width_lines(c: &Ram, name: &str, is_y1: bool) -> R<Vec<String>> {
    let is_36 = c.is_36;
    let read_width_a = c.int("READ_WIDTH_A")?;
    let write_width_b = c.int("WRITE_WIDTH_B")?;
    let this = c.int(name)?;
    let b_side_reads_half = if is_36 { read_width_a == 72 } else { read_width_a == 36 };
    let a_side_writes_half = if is_36 { write_width_b == 72 } else { write_width_b == 36 };
    if this == 0 && name == "READ_WIDTH_B" && b_side_reads_half {
        return Ok(vec!["READ_WIDTH_B_18".into()]);
    }
    if this == 0 && name == "WRITE_WIDTH_A" && a_side_writes_half {
        return Ok(vec!["WRITE_WIDTH_A_18".into()]);
    }
    if this == 0 {
        return Ok(vec![]);
    }
    let actual = if is_36 && this != 1 { this / 2 } else { this };
    let mut out = Vec::new();
    if ((is_36 && this == 72) || (is_y1 && actual == 36)) && name == "READ_WIDTH_A" {
        out.push(format!("{name}_18"));
    }
    if actual == 36 {
        out.push(format!("SDP_{}_36", &name[..name.len() - 2]));
        match name {
            "WRITE_WIDTH_A" => out.push("WRITE_WIDTH_A_18".into()),
            "WRITE_WIDTH_B" => out.push("WRITE_WIDTH_B_18".into()),
            "READ_WIDTH_B" => out.push("READ_WIDTH_B_18".into()),
            _ => {}
        }
    } else {
        // The unused partner port of a 36-wide (SDP) direction stays unset.
        let dir = &name[..name.len() - 2];
        let mut dir_is_sdp36 = false;
        for ab in ["A", "B"] {
            let w = c.int(&format!("{dir}_{ab}"))?;
            let aw = if is_36 && w != 1 && w != 0 { w / 2 } else { w };
            dir_is_sdp36 |= aw == 36;
        }
        if !(dir_is_sdp36 && actual == 1) {
            out.push(format!("{name}_{actual}"));
        }
    }
    Ok(out)
}

/// `write_vector`: `NAME = Wb...`, most significant bit first.
fn vector(name: &str, bits: &[bool]) -> String {
    let s: String = bits.iter().rev().map(|&b| if b { '1' } else { '0' }).collect();
    format!("{name} = {}'b{s}", bits.len())
}

/// `write_bram_init` for one half.
fn init_lines(c: &Ram, half: usize) -> Vec<String> {
    let mut out = Vec::new();
    for mode in ["", "P"] {
        let n = if mode == "P" { 8 } else { 64 };
        for i in 0..n {
            let mut has = false;
            let mut data = vec![false; 256];
            if c.is_36 {
                for j in 0..2 {
                    if let Some(b) = c.bits(&format!("INIT{mode}_{:02X}", i * 2 + j)) {
                        has = true;
                        for k in (half..256).step_by(2) {
                            if k >= b.len() {
                                break;
                            }
                            data[j * 128 + k / 2] = b[k];
                        }
                    }
                }
            } else if let Some(b) = c.bits(&format!("INIT{mode}_{i:02X}")) {
                has = true;
                for k in 0..256.min(b.len()) {
                    data[k] = b[k];
                }
            }
            if has {
                out.push(vector(&format!("INIT{mode}_{i:02X}[255:0]"), &data));
            }
        }
    }
    out
}

fn half_lines(c: &Ram, half: usize) -> R<Vec<String>> {
    let mut out = vec!["IN_USE".to_string()];
    for w in ["READ_WIDTH_A", "READ_WIDTH_B", "WRITE_WIDTH_A", "WRITE_WIDTH_B"] {
        out.extend(width_lines(c, w, half == 1)?);
    }
    let doa = c.flag("DOA_REG")?;
    let dob = c.flag("DOB_REG")?;
    if doa {
        out.push("DOA_REG".into());
    }
    if dob {
        out.push("DOB_REG".into());
    }
    for p in INVERTIBLE {
        if !c.flag(&format!("IS_{p}_INVERTED"))? {
            out.push(format!("ZINV_{p}"));
        }
    }
    if doa && !c.flag("IS_CLKARDCLK_INVERTED")? {
        out.push("ZINV_REGCLKARDRCLK".into());
    }
    if dob && !c.flag("IS_CLKBWRCLK_INVERTED")? {
        out.push("ZINV_REGCLKB".into());
    }
    for m in ["WRITE_MODE_A", "WRITE_MODE_B"] {
        let mode = c.str_or(m, "WRITE_FIRST");
        if mode != "WRITE_FIRST" {
            out.push(format!("{m}_{mode}"));
        }
    }
    for v in ["ZINIT_A", "ZINIT_B", "ZSRVAL_A", "ZSRVAL_B"] {
        out.push(vector(&format!("{v}[17:0]"), &[true; 18]));
    }
    out.extend(init_lines(c, half));
    Ok(out)
}

/// The block RAM lines of a placed design, but for the cascade bits.
pub fn bram_config(placed: &Value, locate: &dyn Fn(&str) -> Option<Located>) -> R<BTreeSet<String>> {
    let mods = placed["modules"].as_object().ok_or("no modules")?;
    let top = mods
        .values()
        .find(|m| m["attributes"]["top"].is_string() || m["attributes"]["top"].is_number())
        .or_else(|| mods.values().next())
        .ok_or("no top module")?;
    // tile -> [lower, upper]
    let mut tiles: HashMap<String, [Option<Ram>; 2]> = HashMap::new();
    for (name, c) in top["cells"].as_object().ok_or("no cells")? {
        let ty = c["type"].as_str().unwrap_or("");
        let is_36 = match ty {
            "RAMB36E1_RAMB36E1" => true,
            "RAMB18E1_RAMB18E1" => false,
            _ => continue,
        };
        let bel = c["attributes"]["NEXTPNR_BEL"].as_str().ok_or_else(|| format!("{name}: unplaced {ty}"))?;
        let (tile, _, site, _) = locate(bel).ok_or_else(|| format!("{name}: no tile for {bel}"))?;
        if !tile.starts_with("BRAM_") {
            return Err(format!("{name}: {bel} is not in a BRAM tile"));
        }
        let slot = tiles.entry(tile.clone()).or_default();
        let halves: Vec<usize> = if is_36 {
            vec![0, 1]
        } else {
            let y: usize = site.rsplit_once('Y').and_then(|(_, y)| y.parse().ok()).filter(|y| *y <= 1).ok_or_else(|| format!("{name}: site {site}"))?;
            vec![y]
        };
        for h in halves {
            if slot[h].is_some() {
                return Err(format!("{name}: {tile} half {h} holds two cells"));
            }
            slot[h] = Some(Ram { name, v: c, is_36 });
        }
    }
    let mut out = BTreeSet::new();
    for (tile, halves) in &tiles {
        for (h, c) in halves.iter().enumerate() {
            let Some(c) = c else { continue };
            for l in half_lines(c, h)? {
                out.insert(format!("{tile}.RAMB18_Y{h}.{l}"));
            }
        }
        if let Some(c) = &halves[0] {
            if c.is_36 {
                for w in ["READ_WIDTH_A", "READ_WIDTH_B", "WRITE_WIDTH_A", "WRITE_WIDTH_B"] {
                    let bits = c.int(w)?;
                    if bits == 1 || bits == 9 {
                        out.insert(format!("{tile}.RAMB36.BRAM36_{w}_1"));
                    }
                }
            }
            out.insert(format!("{tile}.{}", vector("ZALMOST_EMPTY_OFFSET[12:0]", &[true; 13])));
            out.insert(format!("{tile}.{}", vector("ZALMOST_FULL_OFFSET[12:0]", &[true; 13])));
        }
    }
    Ok(out)
}

/// Whether any RAMB36E1 is half of a cascaded pair (`RAM_EXTENSION_A` or
/// `_B` other than `NONE`): the only cells whose nets use the address
/// cascade.
pub fn uses_cascade(placed: &Value) -> R<bool> {
    let mods = placed["modules"].as_object().ok_or("no modules")?;
    Ok(mods.values().flat_map(|m| m["cells"].as_object().into_iter().flatten()).any(|(_, c)| {
        c["type"].as_str().is_some_and(|t| t.starts_with("RAMB36"))
            && ["RAM_EXTENSION_A", "RAM_EXTENSION_B"].iter().any(|p| c["parameters"][*p].as_str().is_some_and(|v| v != "NONE"))
    }))
}

/// `CASCOUT_ARD_ACTIVE` and `CASCOUT_BWR_ACTIVE` of every BRAM tile whose
/// cascade address outputs a routed pip drives. `pips` are (tile, dst wire).
pub fn cascade_lines<'a>(pips: impl IntoIterator<Item = (&'a str, &'a str)>) -> BTreeSet<String> {
    let mut out = BTreeSet::new();
    for (tile, dst) in pips {
        if !tile.starts_with("BRAM_") {
            continue;
        }
        if dst.starts_with("BRAM_CASCOUT_ADDRARDADDR") {
            out.insert(format!("{tile}.CASCOUT_ARD_ACTIVE"));
        } else if dst.starts_with("BRAM_CASCOUT_ADDRBWRADDR") {
            out.insert(format!("{tile}.CASCOUT_BWR_ACTIVE"));
        }
    }
    out
}

/// Whether a FASM line is a block RAM setting `bram_config` or
/// `cascade_lines` writes.
pub fn is_bram_config(line: &str) -> bool {
    let f = line.split('#').next().unwrap_or("").split('=').next().unwrap_or("").trim();
    let parts: Vec<&str> = f.split('.').collect();
    match parts[..] {
        [t, site, _] if t.starts_with("BRAM_") && site.starts_with("RAMB18_Y") => true,
        [t, "RAMB36", w] if t.starts_with("BRAM_") && w.starts_with("BRAM36_") => true,
        [t, s] if t.starts_with("BRAM_") => {
            s.starts_with("ZALMOST_EMPTY_OFFSET") || s.starts_with("ZALMOST_FULL_OFFSET") || s == "CASCOUT_ARD_ACTIVE" || s == "CASCOUT_BWR_ACTIVE"
        }
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn ram(params: Value, is_36: bool) -> Value {
        json!({"type": if is_36 { "RAMB36E1_RAMB36E1" } else { "RAMB18E1_RAMB18E1" }, "parameters": params})
    }

    #[test]
    fn widths() {
        // RAMB36 SDP x72: read A 72, write B 72.
        let v = ram(json!({"READ_WIDTH_A": "1001000", "WRITE_WIDTH_B": "1001000"}), true);
        let c = Ram { name: "r", v: &v, is_36: true };
        assert_eq!(width_lines(&c, "READ_WIDTH_A", false).unwrap(), vec!["READ_WIDTH_A_18", "SDP_READ_WIDTH_36"]);
        assert_eq!(width_lines(&c, "READ_WIDTH_B", false).unwrap(), vec!["READ_WIDTH_B_18"]);
        assert_eq!(width_lines(&c, "WRITE_WIDTH_A", false).unwrap(), vec!["WRITE_WIDTH_A_18"]);
        assert_eq!(width_lines(&c, "WRITE_WIDTH_B", false).unwrap(), vec!["SDP_WRITE_WIDTH_36", "WRITE_WIDTH_B_18"]);
        // RAMB36 TDP x36 on B read, A write.
        let v = ram(json!({"READ_WIDTH_B": "100100", "WRITE_WIDTH_A": "100100"}), true);
        let c = Ram { name: "r", v: &v, is_36: true };
        assert_eq!(width_lines(&c, "READ_WIDTH_B", false).unwrap(), vec!["READ_WIDTH_B_18"]);
        assert_eq!(width_lines(&c, "READ_WIDTH_A", false).unwrap(), Vec::<String>::new());
    }

    #[test]
    fn init_halves() {
        // INIT_00 bit 0 and bit 1 set: half 0 takes even bits, half 1 odd.
        let v = ram(json!({"INIT_00": "11"}), true);
        let c = Ram { name: "r", v: &v, is_36: true };
        let l0 = init_lines(&c, 0);
        assert_eq!(l0.len(), 1);
        assert!(l0[0].ends_with("01"));
        let l1 = init_lines(&c, 1);
        assert!(l1[0].ends_with("01"));
        let v = ram(json!({"INIT_00": "10"}), true);
        let c = Ram { name: "r", v: &v, is_36: true };
        assert!(init_lines(&c, 0)[0].ends_with("00"));
        assert!(init_lines(&c, 1)[0].ends_with("01"));
    }

    #[test]
    fn owned() {
        assert!(is_bram_config("BRAM_L_X30Y105.RAMB18_Y1.READ_WIDTH_B_18"));
        assert!(is_bram_config("BRAM_L_X30Y105.ZALMOST_FULL_OFFSET[12:0] = 13'b1111111111111"));
        assert!(is_bram_config("BRAM_L_X30Y105.RAMB36.BRAM36_READ_WIDTH_A_1"));
        assert!(is_bram_config("BRAM_L_X30Y105.CASCOUT_ARD_ACTIVE"));
        assert!(!is_bram_config("BRAM_L_X30Y105.BRAM_LOGIC_OUTS_B6_4.BRAM_FIFO36_DOBDOU6"));
    }
}
