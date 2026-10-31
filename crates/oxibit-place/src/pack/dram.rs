//! Distributed RAM (nextpnr-xilinx `pack_dram`, `create_dram_lut`,
//! `create_dram32_lut`, `create_muxf_tree`).
//!
//! Handled: RAM32M, RAM64M (whole slice), RAM128X1D / RAM128X1S (two or four
//! 64-deep LUTs per port and a MUXF7 decode). Other DRAM primitives are
//! counted as unhandled.

use serde_json::{Value, json};

use super::constants::prop32;
use super::muxf::constrain_muxf_tree;
use super::{Offset, Packer, Rule, slot};
use crate::netlist::{CellId, Dir, NetId};

const HEIGHT: i32 = 4;

#[derive(Clone)]
struct Ctrl {
    wa: Vec<Option<NetId>>,
    wclk: Option<NetId>,
    we: Option<NetId>,
    wclk_inv: bool,
}

fn rule64() -> Rule {
    let mut r = Rule::to("SLICE_LUTX").port("I", "DI1").port("O", "O6").attr("X_LUT_AS_DRAM", json!("1"));
    r.param.insert("IS_CLK_INVERTED".into(), "IS_WCLK_INVERTED".into());
    for i in 0..6 {
        r = r.port(&format!("RADR{i}"), &format!("A{}", i + 1));
    }
    for i in 0..8 {
        r = r.port(&format!("WADR{i}"), &format!("WA{}", i + 1));
    }
    r
}

fn rule32(o5: bool) -> Rule {
    let mut r = Rule::to("SLICE_LUTX").attr("X_LUT_AS_DRAM", json!("1"));
    r = if o5 {
        r.port("I", "DI1").port("O", "O5")
    } else {
        r.port("I", "DI2").port("O", "O6")
    };
    r.param.insert("IS_CLK_INVERTED".into(), "IS_WCLK_INVERTED".into());
    for i in 0..5 {
        r = r
            .port(&format!("RADR{i}"), &format!("A{}", i + 1))
            .port(&format!("WADR{i}"), &format!("WA{}", i + 1));
    }
    r
}

#[allow(clippy::too_many_arguments)]
fn dram_lut(
    p: &mut Packer,
    name: &str,
    base: Option<CellId>,
    cs: &Ctrl,
    address: &[Option<NetId>],
    di: Option<NetId>,
    dout: Option<NetId>,
    z: i32,
    d32: Option<bool>,
) -> CellId {
    let c = p.nl.add_cell(name, if d32.is_some() { "RAMD32" } else { "RAMD64E" });
    let (ra, wa) = if d32.is_some() { (5, 5) } else { (6, 8) };
    for i in 0..ra {
        p.nl.add_port(c, &format!("RADR{i}"), Dir::Input);
    }
    for i in 0..wa {
        p.nl.add_port(c, &format!("WADR{i}"), Dir::Input);
    }
    for n in ["CLK", "I", "WE"] {
        p.nl.add_port(c, n, Dir::Input);
    }
    p.nl.add_port(c, "O", Dir::Output);
    for (i, a) in address.iter().enumerate() {
        if let Some(n) = a {
            p.nl.connect(c, &format!("RADR{i}"), *n);
        }
    }
    let conn = |p: &mut Packer, port: &str, n: Option<NetId>| {
        if let Some(n) = n {
            p.nl.connect(c, port, n);
        }
    };
    conn(p, "I", di);
    conn(p, "O", dout);
    conn(p, "CLK", cs.wclk);
    conn(p, "WE", cs.we);
    for (i, a) in cs.wa.iter().enumerate().take(wa) {
        conn(p, &format!("WADR{i}"), *a);
    }
    p.nl.cell_mut(c).params.insert("IS_WCLK_INVERTED".into(), prop32(cs.wclk_inv as u64));
    let rule = match d32 {
        None => rule64(),
        Some(o5) => rule32(o5),
    };
    p.xform(c, &rule);
    let z = slot::at(z, if d32 == Some(true) { slot::LUT5 } else { slot::LUT6 });
    match base {
        None => p.make_root(c, Some(z)),
        Some(b) => p.add_member(b, c, Offset { dx: 0, dy: 0, z, abs: true }),
    }
    c
}

/// Bits `lo .. lo + n` of a binary-string parameter (MSB first), as a
/// binary string.
fn extract(v: Option<&Value>, total: usize, lo: usize, n: usize) -> Value {
    let s = v.and_then(Value::as_str).unwrap_or("");
    let bits: Vec<bool> = s.chars().rev().map(|c| c == '1').collect();
    let _ = total;
    Value::String(
        (lo..lo + n)
            .rev()
            .map(|i| if bits.get(i).copied().unwrap_or(false) { '1' } else { '0' })
            .collect(),
    )
}

/// MUXF7 (and up) tree over `data`, selected by `select`, rooted on `out`.
fn muxf_tree(p: &mut Packer, base: CellId, name_base: &str, data: Vec<Option<NetId>>, select: &[Option<NetId>], out: Option<NetId>, zoffset: i32) {
    let levels = match data.len() {
        0..=2 => 1,
        3..=4 => 2,
        _ => 3,
    };
    let base_name = p.nl.cell(base).name.clone();
    let mut last = data;
    let mut root = None;
    for i in 0..levels {
        let ty = ["MUXF7", "MUXF8", "MUXF9"][i];
        let mut next = Vec::new();
        for j in 0..last.len() / 2 {
            let output = if i == levels - 1 {
                out
            } else {
                Some(p.nl.add_net_unique(&format!("{base_name}$intnet${name_base}_muxq_{i}_{j}")))
            };
            next.push(output);
            let m = p.nl.add_cell(&format!("{base_name}$intcell${name_base}_muxf_{i}_{j}"), ty);
            for (port, d) in [("I0", Dir::Input), ("I1", Dir::Input), ("S", Dir::Input), ("O", Dir::Output)] {
                p.nl.add_port(m, port, d);
            }
            for (port, n) in [
                ("I0", last[j * 2]),
                ("I1", last[j * 2 + 1]),
                ("S", select.get(i).copied().flatten()),
                ("O", output),
            ] {
                if let Some(n) = n {
                    p.nl.connect(m, port, n);
                }
            }
            if i == levels - 1 {
                root = Some(m);
            }
        }
        last = next;
    }
    if let Some(r) = root {
        constrain_muxf_tree(p, r, base, zoffset);
    }
}

fn get(p: &Packer, c: CellId, port: &str) -> Option<NetId> {
    p.nl.cell(c).net(port)
}

pub fn pack_dram(p: &mut Packer) {
    let (vcc, gnd) = (p.nl.vcc, p.nl.gnd);
    // Tied-low address bits of the 128-deep primitives move to VCC, with
    // the INIT rewritten to match.
    let mut inverted = 0;
    for c in p.nl.cell_ids() {
        let ty = p.nl.cell(c).ty.clone();
        if !matches!(ty.as_str(), "RAM128X1D" | "RAM128X1S") {
            continue;
        }
        let dual = ty == "RAM128X1D";
        for i in 0..6 {
            let a = if ty == "RAM128X1S" { format!("A{i}") } else { format!("A[{i}]") };
            if get(p, c, &a) != Some(gnd) {
                continue;
            }
            let ra = format!("DPRA[{i}]");
            if dual && get(p, c, &ra) != Some(gnd) {
                continue;
            }
            p.nl.connect(c, &a, vcc);
            if dual {
                p.nl.connect(c, &ra, vcc);
            }
            inverted += 1;
            if let Some(Value::String(s)) = p.nl.cell(c).params.get("INIT").cloned() {
                let mut bits: Vec<char> = s.chars().rev().collect();
                for j in 0..bits.len() {
                    if j & (1 << i) != 0 {
                        bits[j] = bits[j & !(1 << i)];
                    }
                }
                p.nl.cell_mut(c)
                    .params
                    .insert("INIT".into(), Value::String(bits.into_iter().rev().collect()));
            }
        }
    }
    p.count("dram.tied_low_to_high", inverted);

    let mut packed = Vec::new();
    let mut unhandled = 0;
    for c in p.nl.cell_ids() {
        let ty = p.nl.cell(c).ty.clone();
        let name = p.nl.cell(c).name.clone();
        let wclk_inv = p.nl.cell(c).int_param("IS_WCLK_INVERTED").unwrap_or(0) != 0;
        match ty.as_str() {
            "RAM128X1D" | "RAM128X1S" => {
                let dual = ty == "RAM128X1D";
                let a = |i: usize| if dual { format!("A[{i}]") } else { format!("A{i}") };
                let cs = Ctrl {
                    wa: (0..7).map(|i| get(p, c, &a(i))).collect(),
                    wclk: get(p, c, "WCLK"),
                    we: get(p, c, "WE"),
                    wclk_inv,
                };
                let init = p.nl.cell(c).params.get("INIT").cloned();
                let (spo_port, spo_name) = if dual { ("SPO", "SPO") } else { ("O", "O") };
                let spo = get(p, c, spo_port);
                let dpo = get(p, c, "DPO");
                p.nl.disconnect(c, spo_port);
                p.nl.disconnect(c, "DPO");
                let d = get(p, c, "D");
                let mut z = HEIGHT - 1;
                let mut base = None;
                let mut spo_pre = Vec::new();
                for i in 0..2 {
                    let n = p.nl.add_net_unique(&format!("{name}$intnet${spo_name}_{i}"));
                    let spr = dram_lut(p, &format!("{name}/ADDR{i}"), base, &cs, &cs.wa[..6], d, Some(n), z, None);
                    base.get_or_insert(spr);
                    spo_pre.push(Some(n));
                    p.nl.cell_mut(spr).params.insert("INIT".into(), extract(init.as_ref(), 128, i * 64, 64));
                    z -= 1;
                }
                let base = base.expect("two cells");
                muxf_tree(p, base, spo_name, spo_pre, &cs.wa[6..], spo, 2);
                if dual {
                    let ra: Vec<Option<NetId>> = (0..7).map(|i| get(p, c, &format!("DPRA[{i}]"))).collect();
                    let mut dpo_pre = Vec::new();
                    for i in 0..2 {
                        let n = p.nl.add_net_unique(&format!("{name}$intnet$DPO_{i}"));
                        let dpr = dram_lut(p, &format!("{name}/DPR{i}"), Some(base), &cs, &ra[..6], d, Some(n), z, None);
                        dpo_pre.push(Some(n));
                        p.nl.cell_mut(dpr).params.insert("INIT".into(), extract(init.as_ref(), 128, i * 64, 64));
                        z -= 1;
                    }
                    muxf_tree(p, base, "DPO", dpo_pre, &ra[6..], dpo, 0);
                }
                packed.push(c);
            }
            "RAM64M" | "RAM32M" => {
                let is64 = ty == "RAM64M";
                let abits = if is64 { 6 } else { 5 };
                let cs = Ctrl {
                    wa: (0..abits).map(|i| get(p, c, &format!("ADDRD[{i}]"))).collect(),
                    wclk: get(p, c, "WCLK"),
                    we: get(p, c, "WE"),
                    wclk_inv,
                };
                let mut base = None;
                for i in 0..4 {
                    let l = (b'A' + i as u8) as char;
                    let address: Vec<Option<NetId>> = (0..abits).map(|j| get(p, c, &format!("ADDR{l}[{j}]"))).collect();
                    let init = p.nl.cell(c).params.get(&format!("INIT_{l}")).cloned();
                    if is64 {
                        let (di, dout) = (get(p, c, &format!("DI{l}")), get(p, c, &format!("DO{l}")));
                        p.nl.disconnect(c, &format!("DI{l}"));
                        p.nl.disconnect(c, &format!("DO{l}"));
                        let d = dram_lut(p, &format!("{name}/DPR{i}"), base, &cs, &address, di, dout, i, None);
                        base.get_or_insert(d);
                        if let Some(v) = init {
                            p.nl.cell_mut(d).params.insert("INIT".into(), v);
                        }
                    } else {
                        for j in 0..2 {
                            let (di, dout) = (get(p, c, &format!("DI{l}[{j}]")), get(p, c, &format!("DO{l}[{j}]")));
                            p.nl.disconnect(c, &format!("DI{l}[{j}]"));
                            p.nl.disconnect(c, &format!("DO{l}[{j}]"));
                            let d = dram_lut(p, &format!("{name}/DPR{i}_{j}"), base, &cs, &address, di, dout, i, Some(j == 0));
                            base.get_or_insert(d);
                            if let Some(Value::String(s)) = &init {
                                // INIT_x interleaves the port's two bits per
                                // entry: this half holds bit j of entry k.
                                let bits: Vec<bool> = s.chars().rev().map(|ch| ch == '1').collect();
                                let half: String = (0..32)
                                    .rev()
                                    .map(|k| if bits.get(k * 2 + j).copied().unwrap_or(false) { '1' } else { '0' })
                                    .collect();
                                p.nl.cell_mut(d).params.insert("INIT".into(), Value::String(half));
                            }
                        }
                    }
                }
                packed.push(c);
            }
            t if t.starts_with("RAM") && (t.contains("X1") || t.ends_with('M')) && !t.starts_with("RAMB") => unhandled += 1,
            _ => {}
        }
    }
    let n = packed.len();
    for c in packed {
        p.nl.remove_cell(c);
    }
    p.count("dram.cells", n);
    p.count("dram.unhandled", unhandled);
}
