//! MMCME2_ADV configuration from the cell's parameters: the FASM lines
//! nextpnr-himbaechel's xilinx `write_mmcm` writes (`TILE.MMCME2_ADV.*`).
//!
//! Per counter (DIVCLK, CLKFBOUT, CLKOUT0..6): high and low time in VCO
//! cycles, EDGE (a half cycle more high), NO_COUNT (divide 1), the phase as
//! a VCO-cycle delay and an eighth-cycle mux, and the fractional divide of
//! CLKOUT0 and CLKFBOUT. A CLKOUTn counter is written only when its output
//! is connected. Then the in-use and pin-inversion bits, STARTUP_WAIT, the
//! compensation mode, and the lock and loop-filter settings, which depend
//! on CLKFBOUT_MULT_F and BANDWIDTH (tables below).
//!
//! The lock and filter tables are nextpnr's (`mmcm_tables.cc`, ISC,
//! Copyright (C) 2019-2023 gatecat, (C) 2023 Hans Baier), from XAPP888.

use std::collections::BTreeSet;

use serde_json::Value;

use crate::lutinit::Located;

type R<T> = Result<T, String>;

const FILTER_LOW: [u16; 64] = [
    0b0010111100, 0b0010111100, 0b0010111100, 0b0010111100, 0b0010011100, 0b0010101100, 0b0010110100, 0b0010001100, 0b0010010100,
    0b0010010100, 0b0010100100, 0b0010111000, 0b0010111000, 0b0010111000, 0b0010111000, 0b0010000100, 0b0010000100, 0b0010000100,
    0b0010011000, 0b0010011000, 0b0010011000, 0b0010011000, 0b0010011000, 0b0010011000, 0b0010011000, 0b0010101000, 0b0010101000,
    0b0010101000, 0b0010101000, 0b0010101000, 0b0010110000, 0b0010110000, 0b0010110000, 0b0010110000, 0b0010110000, 0b0010110000,
    0b0010110000, 0b0010110000, 0b0010110000, 0b0010110000, 0b0010110000, 0b0010110000, 0b0010110000, 0b0010110000, 0b0010110000,
    0b0010110000, 0b0010110000, 0b0010001000, 0b0010001000, 0b0010001000, 0b0010001000, 0b0010001000, 0b0010001000, 0b0010001000,
    0b0010001000, 0b0010001000, 0b0010001000, 0b0010001000, 0b0010001000, 0b0010001000, 0b0010001000, 0b0010001000, 0b0010001000,
    0b0010001000,
];

const FILTER_LOW_SS: [u16; 64] = [
    0b0010111111, 0b0010111111, 0b0010111111, 0b0010111111, 0b0010011111, 0b0010101111, 0b0010110111, 0b0010001111, 0b0010010111,
    0b0010010111, 0b0010100111, 0b0010111011, 0b0010111011, 0b0010111011, 0b0010111011, 0b0010000111, 0b0010000111, 0b0010000111,
    0b0010011011, 0b0010011011, 0b0010011011, 0b0010011011, 0b0010011011, 0b0010011011, 0b0010011011, 0b0010101011, 0b0010101011,
    0b0010101011, 0b0010101011, 0b0010101011, 0b0010110011, 0b0010110011, 0b0010110011, 0b0010110011, 0b0010110011, 0b0010110011,
    0b0010110011, 0b0010110011, 0b0010110011, 0b0010110011, 0b0010110011, 0b0010110011, 0b0010110011, 0b0010110011, 0b0010110011,
    0b0010110011, 0b0010110011, 0b0010001011, 0b0010001011, 0b0010001011, 0b0010001011, 0b0010001011, 0b0010001011, 0b0010001011,
    0b0010001011, 0b0010001011, 0b0010001011, 0b0010001011, 0b0010001011, 0b0010001011, 0b0010001011, 0b0010001011, 0b0010001011,
    0b0010001011,
];

/// HIGH and OPTIMIZED share one table.
const FILTER_HIGH: [u16; 64] = [
    0b0010111100, 0b0100111100, 0b0101101100, 0b0111011100, 0b1101011100, 0b1110101100, 0b1110110100, 0b1111001100, 0b1110010100,
    0b1111010100, 0b1111100100, 0b1101000100, 0b1111100100, 0b1111100100, 0b1111100100, 0b1111100100, 0b1111010100, 0b1111010100,
    0b1100000100, 0b1100000100, 0b1100000100, 0b0101110000, 0b0101110000, 0b0101110000, 0b0101110000, 0b0011010000, 0b0011010000,
    0b0011010000, 0b0011010000, 0b0011010000, 0b0011010000, 0b0011010000, 0b0011010000, 0b0011010000, 0b0011010000, 0b0011010000,
    0b0011010000, 0b0011010000, 0b0011010000, 0b0011010000, 0b0011010000, 0b0010100000, 0b0010100000, 0b0010100000, 0b0010100000,
    0b0010100000, 0b0111000100, 0b0111000100, 0b0100110000, 0b0100110000, 0b0100110000, 0b0100110000, 0b0110000100, 0b0110000100,
    0b0101011000, 0b0101011000, 0b0101011000, 0b0010010000, 0b0010010000, 0b0010010000, 0b0010010000, 0b0100101000, 0b0011110000,
    0b0011110000,
];

/// LockRefDly(5) LockFBDly(5) LockCnt(10) LockSatHigh(10) UnlockCnt(10).
const LOCK: [u64; 63] = [
    0b0011000110111110100011111010010000000001,
    0b0011000110111110100011111010010000000001,
    0b0100001000111110100011111010010000000001,
    0b0101101011111110100011111010010000000001,
    0b0111001110111110100011111010010000000001,
    0b1000110001111110100011111010010000000001,
    0b1001110011111110100011111010010000000001,
    0b1011010110111110100011111010010000000001,
    0b1100111001111110100011111010010000000001,
    0b1110011100111110100011111010010000000001,
    0b1111111111111000010011111010010000000001,
    0b1111111111110011100111111010010000000001,
    0b1111111111101110111011111010010000000001,
    0b1111111111101011110011111010010000000001,
    0b1111111111101000101011111010010000000001,
    0b1111111111100111000111111010010000000001,
    0b1111111111100011111111111010010000000001,
    0b1111111111100010011011111010010000000001,
    0b1111111111100000110111111010010000000001,
    0b1111111111011111010011111010010000000001,
    0b1111111111011101101111111010010000000001,
    0b1111111111011100001011111010010000000001,
    0b1111111111011010100111111010010000000001,
    0b1111111111011001000011111010010000000001,
    0b1111111111011001000011111010010000000001,
    0b1111111111010111011111111010010000000001,
    0b1111111111010101111011111010010000000001,
    0b1111111111010101111011111010010000000001,
    0b1111111111010100010111111010010000000001,
    0b1111111111010100010111111010010000000001,
    0b1111111111010010110011111010010000000001,
    0b1111111111010010110011111010010000000001,
    0b1111111111010010110011111010010000000001,
    0b1111111111010001001111111010010000000001,
    0b1111111111010001001111111010010000000001,
    0b1111111111010001001111111010010000000001,
    0b1111111111001111101011111010010000000001,
    0b1111111111001111101011111010010000000001,
    0b1111111111001111101011111010010000000001,
    0b1111111111001111101011111010010000000001,
    0b1111111111001111101011111010010000000001,
    0b1111111111001111101011111010010000000001,
    0b1111111111001111101011111010010000000001,
    0b1111111111001111101011111010010000000001,
    0b1111111111001111101011111010010000000001,
    0b1111111111001111101011111010010000000001,
    0b1111111111001111101011111010010000000001,
    0b1111111111001111101011111010010000000001,
    0b1111111111001111101011111010010000000001,
    0b1111111111001111101011111010010000000001,
    0b1111111111001111101011111010010000000001,
    0b1111111111001111101011111010010000000001,
    0b1111111111001111101011111010010000000001,
    0b1111111111001111101011111010010000000001,
    0b1111111111001111101011111010010000000001,
    0b1111111111001111101011111010010000000001,
    0b1111111111001111101011111010010000000001,
    0b1111111111001111101011111010010000000001,
    0b1111111111001111101011111010010000000001,
    0b1111111111001111101011111010010000000001,
    0b1111111111001111101011111010010000000001,
    0b1111111111001111101011111010010000000001,
    0b1111111111001111101011111010010000000001,
];

/// A numeric parameter: a real (`"10.000000"`), a binary integer as yosys
/// writes it (`"00..01010"`), or a number.
fn num_param(v: &Value) -> Option<f64> {
    match v {
        Value::Number(n) => n.as_f64(),
        Value::String(s) => {
            let t = s.trim();
            if !t.contains('.') && !t.is_empty() && t.chars().all(|c| c == '0' || c == '1') {
                i64::from_str_radix(t, 2).ok().map(|n| n as f64)
            } else {
                t.parse().ok()
            }
        }
        _ => None,
    }
}

/// `name = W'b...`, most significant bit first.
fn int_vector(name: &str, value: u64, width: usize) -> String {
    let bits: String = (0..width).rev().map(|i| if (value >> i) & 1 == 1 { '1' } else { '0' }).collect();
    format!("{name} = {width}'b{bits}")
}

/// The lines of one counter (`write_mmcm_clkout`).
fn counter(name: &str, cell: &Value, cell_name: &str, out: &mut Vec<String>) -> R<()> {
    let params = &cell["parameters"];
    let num = |p: &str, d: f64| params.get(p).and_then(num_param).unwrap_or(d);
    let divide_param = match name {
        "CLKFBOUT" => "CLKFBOUT_MULT_F".to_string(),
        "CLKOUT0" => "CLKOUT0_DIVIDE_F".to_string(),
        _ => format!("{name}_DIVIDE"),
    };
    let divide = num(&divide_param, 1.0);
    // Only the CLKOUTn counters have a duty cycle; DIVCLK and CLKFBOUT run 50%.
    let duty = if name.starts_with("CLKOUT") { num(&format!("{name}_DUTY_CYCLE"), 0.5) } else { 0.5 };
    let phase = num(&format!("{name}_PHASE"), 0.0);
    let (mut high, mut low, mut phasemux, mut delaytime, mut frac) = (1i64, 1i64, 0i64, 0i64, 0i64);
    let (mut no_count, mut edge) = (false, false);
    if divide <= 1.0 {
        no_count = true;
    } else {
        let whole = divide.floor();
        if name == "CLKOUT0" || name == "CLKFBOUT" {
            frac = ((divide * 8.0).floor() - whole * 8.0) as i64;
        }
        if frac != 0 && duty != 0.5 {
            return Err(format!("{cell_name}: {name}_DUTY_CYCLE must be 0.5 when the divide is fractional"));
        }
        // HIGH_TIME + LOW_TIME = divide; EDGE adds a half cycle high
        // (XAPP888). The high time is rounded to the nearest half cycle.
        let half_cycles = (2.0 * duty * whole + 0.5).floor() as i64;
        high = half_cycles / 2;
        edge = half_cycles % 2 != 0;
        if high < 1 || high > whole as i64 - 1 {
            return Err(format!("{cell_name}: {name}_DUTY_CYCLE={duty} is out of range for divide {whole}"));
        }
        low = whole as i64 - high;
        let phase_eights = ((phase / 360.0) * divide * 8.0).floor() as i64;
        phasemux = phase_eights % 8;
        delaytime = phase_eights / 8;
    }
    let used = matches!(name, "DIVCLK" | "CLKFBOUT") || cell["connections"][name].as_array().is_some_and(|a| !a.is_empty());
    if name == "DIVCLK" {
        out.push(int_vector("DIVCLK_DIVCLK_HIGH_TIME[5:0]", high as u64, 6));
        out.push(int_vector("DIVCLK_DIVCLK_LOW_TIME[5:0]", low as u64, 6));
        if edge {
            out.push("DIVCLK_DIVCLK_EDGE[0]".into());
        }
        if no_count {
            out.push("DIVCLK_DIVCLK_NO_COUNT[0]".into());
        }
        return Ok(());
    }
    if !used {
        return Ok(());
    }
    let fractional = name == "CLKOUT5" || name == "CLKOUT6";
    if (name == "CLKOUT0" || name == "CLKFBOUT") && frac != 0 {
        high -= 1;
        low -= 1;
        // CLKOUT0 drives CLKOUT5_CLKOUT2, CLKFBOUT drives CLKOUT6_CLKOUT2.
        let other = if name == "CLKOUT0" { "CLKOUT5_CLKOUT2_" } else { "CLKOUT6_CLKOUT2_" };
        let shifted = frac >> 1;
        if shifted >= 1 {
            out.push(format!("{other}FRACTIONAL_FRAC_WF_F[0]"));
            out.push(int_vector(&format!("{other}FRACTIONAL_PHASE_MUX_F[1:0]"), shifted as u64, 2));
        }
    }
    out.push(format!("{name}_CLKOUT1_OUTPUT_ENABLE[0]"));
    out.push(int_vector(&format!("{name}_CLKOUT1_HIGH_TIME[5:0]"), high as u64, 6));
    out.push(int_vector(&format!("{name}_CLKOUT1_LOW_TIME[5:0]"), low as u64, 6));
    out.push(int_vector(&format!("{name}_CLKOUT1_PHASE_MUX[2:0]"), phasemux as u64, 3));
    let (edge_f, no_count_f, delay_f) = if fractional {
        ("_CLKOUT2_FRACTIONAL_EDGE[0]", "_CLKOUT2_FRACTIONAL_NO_COUNT[0]", "_CLKOUT2_FRACTIONAL_DELAY_TIME[5:0]")
    } else {
        ("_CLKOUT2_EDGE[0]", "_CLKOUT2_NO_COUNT[0]", "_CLKOUT2_DELAY_TIME[5:0]")
    };
    if edge {
        out.push(format!("{name}{edge_f}"));
    }
    if no_count {
        out.push(format!("{name}{no_count_f}"));
    }
    out.push(int_vector(&format!("{name}{delay_f}"), delaytime as u64, 6));
    if !fractional && frac != 0 {
        out.push(format!("{name}_CLKOUT2_FRAC_EN[0]"));
        out.push(format!("{name}_CLKOUT2_FRAC_WF_R[0]"));
        out.push(int_vector(&format!("{name}_CLKOUT2_FRAC[2:0]"), frac as u64, 3));
    }
    Ok(())
}

/// Every MMCME2_ADV cell's lines.
pub fn mmcm_config(placed: &Value, locate: &dyn Fn(&str) -> Option<Located>) -> R<BTreeSet<String>> {
    let mods = placed["modules"].as_object().ok_or("no modules")?;
    let top = mods
        .values()
        .find(|m| m["attributes"]["top"].is_string() || m["attributes"]["top"].is_number())
        .or_else(|| mods.values().next())
        .ok_or("no top module")?;
    let mut out = BTreeSet::new();
    for (name, c) in top["cells"].as_object().ok_or("no cells")? {
        if c["attributes"]["X_ORIG_TYPE"].as_str() != Some("MMCME2_ADV") {
            continue;
        }
        let Some(bel) = c["attributes"]["NEXTPNR_BEL"].as_str() else { continue };
        let Some((tile, _, _, _)) = locate(bel) else { return Err(format!("{name}: no tile for {bel}")) };
        let params = &c["parameters"];
        let flag = |p: &str| params.get(p).and_then(num_param).is_some_and(|v| v != 0.0);
        let text = |p: &str, d: &str| params.get(p).and_then(Value::as_str).map(str::to_string).unwrap_or_else(|| d.to_string());

        let mut lines: Vec<String> = vec!["IN_USE".into()];
        // A set bit is an inverted pin, ZINV_ names notwithstanding (as
        // nextpnr's PLL writer notes: negating them holds the MMCM in reset).
        for (p, f) in [
            ("IS_PWRDWN_INVERTED", "ZINV_PWRDWN"),
            ("IS_RST_INVERTED", "ZINV_RST"),
            ("IS_PSEN_INVERTED", "ZINV_PSEN"),
            ("IS_PSINCDEC_INVERTED", "ZINV_PSINCDEC"),
            ("IS_CLKINSEL_INVERTED", "INV_CLKINSEL"),
        ] {
            if flag(p) {
                lines.push(f.into());
            }
        }
        for counter_name in ["DIVCLK", "CLKFBOUT", "CLKOUT0", "CLKOUT1", "CLKOUT2", "CLKOUT3", "CLKOUT4", "CLKOUT5", "CLKOUT6"] {
            counter(counter_name, c, name, &mut lines)?;
        }
        let startup_wait = match params.get("STARTUP_WAIT") {
            Some(Value::String(s)) if s == "TRUE" || s == "FALSE" => s == "TRUE",
            Some(v) => num_param(v).is_some_and(|n| n != 0.0),
            None => false,
        };
        if startup_wait {
            lines.push("STARTUP_WAIT".into());
        }
        match text("COMPENSATION", "INTERNAL").as_str() {
            // Both set the same bit.
            "INTERNAL" | "ZHOLD" => lines.push("COMP.Z_ZHOLD".into()),
            other => return Err(format!("{name}: COMPENSATION {other} is not supported (INTERNAL, ZHOLD)")),
        }
        let mult = params.get("CLKFBOUT_MULT_F").and_then(num_param).unwrap_or(5.0) as i64;
        if !(1..=63).contains(&mult) {
            return Err(format!("{name}: CLKFBOUT_MULT_F {mult} is out of range (1..63)"));
        }
        let i = (mult - 1) as usize;
        lines.push(int_vector("LKTABLE[39:0]", LOCK[i], 40));
        let filter = match text("BANDWIDTH", "OPTIMIZED").as_str() {
            "LOW" => FILTER_LOW[i],
            "LOW_SS" => FILTER_LOW_SS[i],
            _ => FILTER_HIGH[i],
        };
        lines.push(int_vector("FILTREG1_RESERVED[11:0]", filter as u64, 12));
        // 0xffff powers every counter (XAPP888's setting).
        lines.push(int_vector("POWER_REG_POWER_REG_POWER_REG[15:0]", 0xffff, 16));
        lines.push("LOCKREG3_RESERVED[0]".into());
        lines.push(int_vector("TABLE[9:0]", 0x3d4, 10));
        for l in lines {
            out.insert(format!("{tile}.MMCME2_ADV.{l}"));
        }
    }
    Ok(out)
}

/// Whether a FASM line is an MMCM setting `mmcm_config` writes.
pub fn is_mmcm_config(line: &str) -> bool {
    let f = line.split('#').next().unwrap_or("").split('=').next().unwrap_or("").trim();
    let mut parts = f.splitn(3, '.');
    matches!((parts.next(), parts.next(), parts.next()), (Some(t), Some("MMCME2_ADV"), Some(_)) if t.starts_with("CMT_TOP"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn params_and_vectors() {
        assert_eq!(num_param(&json!("10.000000")), Some(10.0));
        assert_eq!(num_param(&json!("00000000000000000000000000001010")), Some(10.0));
        assert_eq!(int_vector("X[5:0]", 5, 6), "X[5:0] = 6'b000101");
    }

    #[test]
    fn counters() {
        // divide 10, 50%: high 5, low 5, no edge.
        let c = json!({"parameters": {"CLKOUT1_DIVIDE": "00000000000000000000000000001010"}, "connections": {"CLKOUT1": [5]}});
        let mut out = Vec::new();
        counter("CLKOUT1", &c, "m", &mut out).unwrap();
        assert!(out.contains(&"CLKOUT1_CLKOUT1_HIGH_TIME[5:0] = 6'b000101".to_string()));
        assert!(out.contains(&"CLKOUT1_CLKOUT1_LOW_TIME[5:0] = 6'b000101".to_string()));
        assert!(!out.iter().any(|l| l.contains("EDGE")));
        // divide 7: high 3 plus EDGE, low 4.
        let c = json!({"parameters": {"CLKOUT2_DIVIDE": "00000000000000000000000000000111"}, "connections": {"CLKOUT2": [5]}});
        let mut out = Vec::new();
        counter("CLKOUT2", &c, "m", &mut out).unwrap();
        assert!(out.contains(&"CLKOUT2_CLKOUT1_HIGH_TIME[5:0] = 6'b000011".to_string()));
        assert!(out.contains(&"CLKOUT2_CLKOUT2_EDGE[0]".to_string()));
        // An unconnected output writes nothing.
        let c = json!({"parameters": {}, "connections": {"CLKOUT3": []}});
        let mut out = Vec::new();
        counter("CLKOUT3", &c, "m", &mut out).unwrap();
        assert!(out.is_empty());
        assert!(is_mmcm_config("CMT_TOP_L_LOWER_B_X178Y61.MMCME2_ADV.COMP.Z_ZHOLD"));
        assert!(!is_mmcm_config("CMT_TOP_L_LOWER_B_X178Y61.CMT_LR_LOWER_B_MMCM_PSEN.CMT_TOP_IMUX1_2"));
    }
}
