//! Block RAM (nextpnr-xilinx `pack_bram`).
//!
//! RAMB18E1 / RAMB36E1 -> `RAMB18E1_RAMB18E1` / `RAMB36E1_RAMB36E1`. A
//! RAMB36's control and address pins exist once per half (`...L` / `...U`)
//! and the logical pin drives both. Byte enables narrower than the write
//! width are replicated; simple dual port (write width 36 / 72) moves the
//! write enable onto both halves of WEBWE.

use std::collections::BTreeMap;

use serde_json::json;

use super::{Packer, Rule};
use crate::netlist::{CellId, Dir, NetId};

/// RAMB36E1 logical pins with a lower and an upper bel pin.
fn ul_pins() -> Vec<(String, Vec<String>)> {
    let mut v = Vec::new();
    for base in [
        "CLKARDCLK",
        "CLKBWRCLK",
        "ENARDEN",
        "ENBWREN",
        "REGCEAREGCE",
        "REGCEB",
        "RSTRAMARSTRAM",
        "RSTRAMB",
        "RSTREGARSTREG",
        "RSTREGB",
    ] {
        v.push((base.to_string(), vec![format!("{base}L"), format!("{base}U")]));
    }
    for (base, n) in [("ADDRARDADDR", 15), ("ADDRBWRADDR", 15), ("WEA", 4), ("WEBWE", 8)] {
        for i in 0..n {
            v.push((format!("{base}[{i}]"), vec![format!("{base}L{i}"), format!("{base}U{i}")]));
        }
    }
    v
}

fn rules(sdp: bool) -> BTreeMap<String, Rule> {
    let mut r18 = Rule::to("RAMB18E1_RAMB18E1");
    r18.multi.insert("WEA[0]".into(), vec!["WEA0".into(), "WEA1".into()]);
    r18.multi.insert("WEA[1]".into(), vec!["WEA2".into(), "WEA3".into()]);
    let mut r36 = Rule::to("RAMB36E1_RAMB36E1");
    for (l, b) in ul_pins() {
        r36.multi.insert(l, b);
    }
    r36.multi.entry("ADDRARDADDR[15]".into()).or_insert_with(|| vec!["ADDRARDADDRL15".into()]);
    r36.multi.entry("ADDRBWRADDR[15]".into()).or_insert_with(|| vec!["ADDRBWRADDRL15".into()]);
    if sdp {
        for i in 0..4 {
            r18.multi
                .insert(format!("WEBWE[{i}]"), vec![format!("WEBWE{}", i * 2), format!("WEBWE{}", i * 2 + 1)]);
            r18.multi.insert(format!("WEA[{i}]"), vec![]);
        }
        for i in 0..8 {
            r36.multi.insert(format!("WEBWE[{i}]"), vec![format!("WEBWEL{i}"), format!("WEBWEU{i}")]);
            r36.multi.insert(format!("WEA[{i}]"), vec![]);
        }
    } else {
        for i in 4..8 {
            r36.multi.insert(format!("WEBWE[{i}]"), vec![]);
        }
    }
    [("RAMB18E1".to_string(), r18), ("RAMB36E1".to_string(), r36)].into_iter().collect()
}

fn ensure_tie(p: &mut Packer, c: CellId, port: &str, n: NetId, replace: bool) {
    let exists = p.nl.cell(c).ports.contains_key(port);
    if exists && !replace {
        return;
    }
    p.nl.add_port(c, port, Dir::Input);
    p.nl.connect(c, port, n);
}

pub fn pack_bram(p: &mut Packer) {
    // Constant ties on a cascade input that is not used carry nothing and
    // cannot be routed (CASCADEIN reaches only another BRAM's CASCADEOUT).
    for c in p.nl.cell_ids() {
        let ty = p.nl.cell(c).ty.clone();
        if ty != "RAMB18E1" && ty != "RAMB36E1" {
            continue;
        }
        for (pin, sel) in [("CASCADEINA", "A_INPUT"), ("CASCADEINB", "B_INPUT")] {
            if p.nl.cell(c).str_param(sel).unwrap_or("DIRECT") == "CASCADE" {
                continue;
            }
            if let Some(n) = p.nl.cell(c).net(pin)
                && p.nl
                    .driver(n)
                    .is_none_or(|(d, _)| matches!(p.nl.cell(d).ty.as_str(), "PSEUDO_VCC" | "PSEUDO_GND"))
            {
                p.nl.disconnect(c, pin);
            }
        }
    }

    // Simple dual port first.
    let sdp = rules(true);
    for c in p.nl.cell_ids() {
        let cell = p.nl.cell(c);
        let wwb = cell.int_param("WRITE_WIDTH_B").unwrap_or(0);
        if (cell.ty == "RAMB18E1" && wwb == 36) || (cell.ty == "RAMB36E1" && wwb == 72) {
            let r = sdp[&cell.ty].clone();
            p.xform(c, &r);
        }
    }

    // Byte enables follow the write width.
    for c in p.nl.cell_ids() {
        let ty = p.nl.cell(c).ty.clone();
        if ty != "RAMB18E1" && ty != "RAMB36E1" {
            continue;
        }
        for port in ['A', 'B'] {
            let ww = p.nl.cell(c).int_param(&format!("WRITE_WIDTH_{port}")).unwrap_or(18);
            let we_width = if ty == "RAMB36E1" || port == 'B' { 4 } else { 2 };
            if ww >= 9 * we_width {
                continue;
            }
            let used = (ww / 9).max(1);
            let stem = if port == 'B' { "WEBWE" } else { "WEA" };
            for i in used..we_width {
                let low = p.nl.cell(c).net(&format!("{stem}[{}]", i % used));
                let cur = format!("{stem}[{i}]");
                p.nl.add_port(c, &cur, Dir::Input);
                p.nl.disconnect(c, &cur);
                if let Some(n) = low {
                    p.nl.connect(c, &cur, n);
                }
            }
        }
    }

    let tdp = rules(false);
    let n = p.generic_xform(&tdp);

    let (gnd, vcc) = (p.nl.gnd, p.nl.vcc);
    for c in p.nl.cell_ids() {
        let ty = p.nl.cell(c).ty.clone();
        if ty == "RAMB18E1_RAMB18E1" {
            let wwa = p.nl.cell(c).int_param("WRITE_WIDTH_A").unwrap_or(0);
            for i in (if wwa == 0 { 0 } else { 2 })..4 {
                ensure_tie(p, c, &format!("WEA{i}"), gnd, false);
            }
            if p.nl.cell(c).int_param("WRITE_WIDTH_B").unwrap_or(0) != 36 {
                for i in 4..8 {
                    ensure_tie(p, c, &format!("WEBWE{i}"), gnd, false);
                }
            }
            for t in ["ADDRATIEHIGH0", "ADDRATIEHIGH1", "ADDRBTIEHIGH0", "ADDRBTIEHIGH1"] {
                ensure_tie(p, c, t, vcc, true);
            }
        } else if ty == "RAMB36E1_RAMB36E1" {
            for t in ["ADDRARDADDRL15", "ADDRBWRADDRL15"] {
                ensure_tie(p, c, t, vcc, true);
            }
            for (ww, di, dip, logical) in [
                ("WRITE_WIDTH_A", "DIADI", "DIPADIP", "DIADI[0]"),
                ("WRITE_WIDTH_B", "DIBDI", "DIPBDIP", "DIBDI[0]"),
            ] {
                let w = p.nl.cell(c).int_param(ww).unwrap_or(0);
                if w == 1 {
                    let d0 = p.nl.cell(c).net(&format!("{di}0"));
                    p.nl.disconnect(c, &format!("{di}1"));
                    if let Some(n) = d0 {
                        p.nl.add_port(c, &format!("{di}1"), Dir::Input);
                        p.nl.connect(c, &format!("{di}1"), n);
                    }
                    p.nl.cell_mut(c).attrs.insert(format!("X_ORIG_PORT_{di}1"), json!(logical));
                    p.nl.disconnect(c, &format!("{dip}0"));
                    p.nl.disconnect(c, &format!("{dip}1"));
                }
                if w == 9
                    && let Some(n) = p.nl.cell(c).net(&format!("{dip}0"))
                {
                    p.nl.add_port(c, &format!("{dip}1"), Dir::Input);
                    p.nl.connect(c, &format!("{dip}1"), n);
                    p.nl.cell_mut(c).attrs.insert(format!("X_ORIG_PORT_{dip}1"), json!(format!("{dip}[0]")));
                }
            }
            if p.nl.cell(c).int_param("WRITE_WIDTH_B").unwrap_or(0) != 72 {
                for s in ["L", "U"] {
                    for i in 4..8 {
                        ensure_tie(p, c, &format!("WEBWE{s}{i}"), gnd, false);
                    }
                }
            } else {
                for s in ["L", "U"] {
                    for i in 0..4 {
                        ensure_tie(p, c, &format!("WEA{s}{i}"), gnd, false);
                    }
                }
            }
        }
    }
    p.count("bram", n);
}
