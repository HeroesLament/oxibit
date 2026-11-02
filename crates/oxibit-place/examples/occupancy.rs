//! Slice occupancy of a placed design: where packing and placement leave
//! capacity on the table.
//!
//!   occupancy DB_FAMILY_DIR PART PLACED.json GRID.json
//!
//! For every slice site of the part, what its four letters hold: the 6LUT
//! and 5LUT (logic, single-input, or memory, with the distinct nets on
//! their inputs), the two flops, the CARRY4, and the flop control sets.
//!
//! stdout: one JSON line of totals, the gap classes below.
//! GRID.json: `{"cols": [...], "rows": [[...], ...]}`, one row per slice.
//!
//! Gap classes (per letter unless noted):
//! - `empty_slices`: slice sites holding nothing.
//! - `empty_letters`: letters with no LUT and no flop in an occupied slice.
//! - `ff_only_letters`: a flop (or two) and no LUT: the 6LUT position free.
//! - `free5_after_lut1` / `free5_after_lut2_3` / `free5_after_lut4_5`: a
//!   6LUT reading at most five nets with the 5LUT free, by the 6LUT's input
//!   count (room for a 5LUT of 4 / 2-3 / 0-1 more inputs at least).
//! - `lut6_full`: a 6LUT reading six nets (no 5LUT possible).
//! - `single_input_luts`: route-throughs and inverters, by slice kind.
//! - `free_ffs`: flop bels empty in occupied slices.

use std::collections::{BTreeMap, HashMap, HashSet};

use oxibit_place::placer::Sites;
use serde_json::{Value, json};

#[derive(Clone, Copy, PartialEq, Eq)]
enum LutKind {
    Logic,
    Memory,
}

#[derive(Clone, Copy)]
struct Lut {
    inputs: usize,
    kind: LutKind,
    /// The one input net of a single-input LUT.
    only: Option<i64>,
}

#[derive(Clone, Copy, Default)]
struct Letter {
    l6: Option<Lut>,
    l5: Option<Lut>,
    ff: bool,
    ff5: bool,
}

#[derive(Default)]
struct Slice {
    letters: [Letter; 4],
    carry: bool,
    ctrl: HashSet<(Option<i64>, Option<i64>, Option<i64>)>,
}

impl Slice {
    fn kind(&self) -> u8 {
        let any = self.carry
            || self
                .letters
                .iter()
                .any(|l| l.l6.is_some() || l.l5.is_some() || l.ff || l.ff5);
        let mem = self.letters.iter().any(|l| {
            [l.l6, l.l5]
                .iter()
                .flatten()
                .any(|x| x.kind == LutKind::Memory)
        });
        match (any, self.carry, mem) {
            (false, _, _) => 0,
            (_, true, _) => 2,
            (_, _, true) => 3,
            _ => 1,
        }
    }
}

fn read(path: &str) -> Result<Value, String> {
    serde_json::from_str(&std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?)
        .map_err(|e| format!("{path}: {e}"))
}

/// The net id of a one-bit connection, or None for a constant or absent bit.
fn net_of(cell: &Value, port: &str, consts: &HashSet<i64>) -> Option<i64> {
    let b = cell["connections"].get(port)?.as_array()?.first()?;
    let n = b.as_i64()?;
    (!consts.contains(&n)).then_some(n)
}

fn run() -> Result<(), String> {
    let args: Vec<String> = std::env::args().collect();
    let [_, db, part, placed, grid_out] = &args[..] else {
        return Err("usage: occupancy DB_FAMILY_DIR PART PLACED.json GRID.json".into());
    };
    let dev = oxibit_place::device::Device::open(std::path::Path::new(db), part)?;
    let sites = Sites::from_device(&dev);
    let v = read(placed)?;
    // the top: the module with the most cells (the file also lists cell modules)
    let module = v["modules"]
        .as_object()
        .and_then(|m| {
            m.values()
                .max_by_key(|x| x["cells"].as_object().map_or(0, |c| c.len()))
        })
        .ok_or("no module")?;
    let cells = module["cells"].as_object().ok_or("no cells")?;

    // Constant nets: driven by the pseudo constant cells.
    let mut consts: HashSet<i64> = HashSet::new();
    for c in cells.values() {
        let ty = c["type"].as_str().unwrap_or("");
        if ty == "PSEUDO_VCC" || ty == "PSEUDO_GND" {
            for bits in c["connections"]
                .as_object()
                .into_iter()
                .flat_map(|m| m.values())
            {
                consts.extend(
                    bits.as_array()
                        .into_iter()
                        .flatten()
                        .filter_map(Value::as_i64),
                );
            }
        }
    }

    // Net -> (driver cell type, the driver's fanout on that net).
    let mut drv: HashMap<i64, String> = HashMap::new();
    let mut fanout: HashMap<i64, usize> = HashMap::new();
    for c in cells.values() {
        let ty = c["type"].as_str().unwrap_or("").to_string();
        let dirs = &c["port_directions"];
        for (port, bits) in c["connections"].as_object().into_iter().flatten() {
            let out = dirs.get(port).and_then(Value::as_str) == Some("output");
            for n in bits
                .as_array()
                .into_iter()
                .flatten()
                .filter_map(Value::as_i64)
            {
                if out {
                    drv.insert(n, ty.clone());
                } else {
                    *fanout.entry(n).or_default() += 1;
                }
            }
        }
    }
    let mut feed_src: BTreeMap<String, usize> = BTreeMap::new();

    let mut slices: HashMap<String, Slice> = HashMap::new();
    for (name, c) in cells {
        let Some(bel) = c["attributes"].get("NEXTPNR_BEL").and_then(Value::as_str) else {
            continue;
        };
        // `X{gx}Y{gy}/{tile-relative site}.{bel}`: key by tile and site
        let Some((tile, rest)) = bel.split_once('/') else {
            continue;
        };
        let Some((site, b)) = rest.split_once('.') else {
            continue;
        };
        if !site.starts_with("SLICE_") {
            continue;
        }
        let s = slices.entry(format!("{tile}/{site}")).or_default();
        let ty = c["type"].as_str().unwrap_or("");
        if ty == "CARRY4" {
            s.carry = true;
            continue;
        }
        let Some(l) = b.chars().next().and_then(|ch| "ABCD".find(ch)) else {
            continue;
        };
        let letter = &mut s.letters[l];
        match &b[1..] {
            "6LUT" | "5LUT" => {
                let ins: HashSet<i64> = (1..=6)
                    .filter_map(|k| net_of(c, &format!("A{k}"), &consts))
                    .collect();
                let attrs = &c["attributes"];
                let mem =
                    attrs.get("X_LUT_AS_DRAM").is_some() || attrs.get("X_LUT_AS_SRL").is_some();
                let only = if ins.len() == 1 {
                    ins.iter().next().copied()
                } else {
                    None
                };
                let lut = Lut {
                    inputs: ins.len(),
                    kind: if mem { LutKind::Memory } else { LutKind::Logic },
                    only,
                };
                if &b[1..] == "6LUT" {
                    letter.l6 = Some(lut);
                } else {
                    letter.l5 = Some(lut);
                }
            }
            "FF" | "5FF" => {
                if &b[1..] == "FF" {
                    letter.ff = true;
                } else {
                    letter.ff5 = true;
                }
                s.ctrl.insert((
                    net_of(c, "CK", &consts),
                    net_of(c, "CE", &consts),
                    net_of(c, "SR", &consts),
                ));
            }
            _ => {}
        }
        let _ = name;
    }

    let mut tot: BTreeMap<&str, usize> = BTreeMap::new();
    let mut rows: Vec<Value> = Vec::new();
    for site in &sites.slices {
        let s = slices
            .remove(&format!("X{}Y{}/{}", site.x, site.y, site.rel))
            .unwrap_or_default();
        let kind = s.kind();
        let (sx, sy) = site
            .rel
            .trim_start_matches("SLICE_X")
            .split_once('Y')
            .and_then(|(x, y)| Some((x.parse::<i64>().ok()?, y.parse::<i64>().ok()?)))
            .unwrap_or((-1, -1));
        let mut r = BTreeMap::<&str, usize>::new();
        if kind == 0 {
            *tot.entry("empty_slices").or_default() += 1;
        }
        *tot.entry(
            [
                "slices_empty",
                "slices_logic",
                "slices_chain",
                "slices_memory",
            ][kind as usize],
        )
        .or_default() += 1;
        for l in &s.letters {
            let n6 = l.l6.map(|x| x.inputs);
            if l.l6.is_some() {
                *r.entry("lut6").or_default() += 1;
            }
            if l.l5.is_some() {
                *r.entry("lut5").or_default() += 1;
            }
            *r.entry("ff").or_default() += usize::from(l.ff) + usize::from(l.ff5);
            if kind != 0 {
                if l.l6.is_none() && l.l5.is_none() && !l.ff && !l.ff5 && !s.carry {
                    *r.entry("empty_letters").or_default() += 1;
                }
                if l.l6.is_none() && l.l5.is_none() && (l.ff || l.ff5) {
                    *r.entry("ff_only_letters").or_default() += 1;
                }
                *r.entry("free_ffs").or_default() += 2 - usize::from(l.ff) - usize::from(l.ff5);
            }
            if let (Some(n), None) = (n6, l.l5)
                && l.l6.is_some_and(|x| x.kind == LutKind::Logic)
            {
                let k = match n {
                    0 | 1 => "free5_after_lut1",
                    2 | 3 => "free5_after_lut2_3",
                    4 | 5 => "free5_after_lut4_5",
                    _ => "lut6_full",
                };
                *r.entry(k).or_default() += 1;
            }
            for (pos, x) in [("6", l.l6), ("5", l.l5)] {
                let Some(x) = x else { continue };
                if kind == 2 && x.kind == LutKind::Logic && x.inputs == 1 {
                    let n = x.only.expect("one input");
                    let src = drv.get(&n).map(String::as_str).unwrap_or("none");
                    let fan = if fanout.get(&n).copied().unwrap_or(0) > 1 {
                        "multi"
                    } else {
                        "single"
                    };
                    *feed_src
                        .entry(format!("lut{pos}_from_{src}_{fan}"))
                        .or_default() += 1;
                }
            }
            for x in [l.l6, l.l5].into_iter().flatten() {
                if x.kind == LutKind::Logic && x.inputs <= 1 {
                    *r.entry("single_input_luts").or_default() += 1;
                    *tot.entry(if kind == 2 {
                        "single_input_luts_chain"
                    } else {
                        "single_input_luts_logic"
                    })
                    .or_default() += 1;
                }
            }
        }
        for (k, n) in &r {
            *tot.entry(k).or_default() += n;
        }
        let g = |k: &str| r.get(k).copied().unwrap_or(0);
        rows.push(json!([
            sx,
            sy,
            site.x,
            site.y,
            u8::from(site.m),
            kind,
            g("lut6"),
            g("lut5"),
            g("ff"),
            g("single_input_luts"),
            g("free5_after_lut1") + g("free5_after_lut2_3") + g("free5_after_lut4_5"),
            g("empty_letters") + g("ff_only_letters"),
            s.ctrl.len()
        ]));
    }
    *tot.entry("unmatched_slice_sites").or_default() += slices.len();
    let cols = [
        "sx",
        "sy",
        "tile_x",
        "tile_y",
        "slicem",
        "kind",
        "lut6",
        "lut5",
        "ff",
        "single_input",
        "free5_pairable",
        "free_letters",
        "ctrl_sets",
    ];
    std::fs::write(
        grid_out,
        json!({"cols": cols, "kinds": ["empty", "logic", "chain", "memory"], "rows": rows})
            .to_string(),
    )
    .map_err(|e| format!("{grid_out}: {e}"))?;
    println!(
        "{}",
        json!({"occupancy": tot, "chain_single_input_sources": feed_src, "slices": sites.slices.len()})
    );
    Ok(())
}

fn main() {
    if let Err(e) = run() {
        eprintln!("occupancy: {e}");
        std::process::exit(1);
    }
}
