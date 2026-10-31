//! Placement, and the legalisation every placement needs afterwards.
//!
//! `import` takes bels from another placement of the same netlist (nextpnr's,
//! to prove the chain before oxibit's own placer exists): a cell the two
//! name alike takes its bel; a cell named by a packer counter (feed-through
//! LUTs) takes its slot in its cluster, relative to the cluster root.
//!
//! `legalise` is nextpnr-xilinx's `fixup_placement`: where a slice letter
//! holds both a 5LUT and a 6LUT they share pins A1..A5, so their inputs are
//! laid out on common pins, the 5LUT's output becomes O5 and the 6LUT's A6
//! is tied high; a LUT used as memory or shift register beside a 6LUT only
//! needs the A6 tie. Unused PS7 inputs are tied low (an unrouted input would
//! read the interconnect's default, high).

use std::collections::{BTreeMap, BTreeSet, HashMap};

use oxibit_pnr::packed::{Design, Loc};
use serde_json::{Value, json};

use crate::netlist::{CellId, Dir, NetId};
use crate::pack::{Packer, slot};

fn bel_for_slot(z: i32) -> Option<String> {
    let l = (b'A' + (z >> 4) as u8) as char;
    Some(match z & 0xF {
        slot::LUT6 => format!("{l}6LUT"),
        slot::LUT5 => format!("{l}5LUT"),
        slot::FF => format!("{l}FF"),
        slot::FF2 => format!("{l}5FF"),
        slot::CARRY4 => "CARRY4".into(),
        _ => return None,
    })
}

fn slot_of_bel(bel: &str) -> Option<i32> {
    if bel == "CARRY4" {
        return Some(slot::CARRY4);
    }
    let l = bel.chars().next()?;
    if !('A'..='D').contains(&l) {
        return None;
    }
    let letter = (l as u8 - b'A') as i32;
    let f = match &bel[1..] {
        "6LUT" => slot::LUT6,
        "5LUT" => slot::LUT5,
        "FF" => slot::FF,
        "5FF" => slot::FF2,
        _ => return None,
    };
    Some(slot::at(letter, f))
}

fn counter_named(name: &str) -> bool {
    let tail = |marker: &str| {
        name.rfind(marker).is_some_and(|i| {
            let rest = &name[i + marker.len()..];
            !rest.is_empty() && rest.chars().all(|c| c.is_ascii_digit())
        })
    };
    tail("$LUT") || tail("$MUX$") || name.ends_with("$feed$zero")
}

#[derive(Debug, Default)]
pub struct ImportStats {
    pub by_name: usize,
    pub by_cluster: usize,
    pub unplaced: Vec<String>,
    pub collisions: Vec<String>,
}

/// Bels from `theirs` (see the module comment).
pub fn import(p: &mut Packer, theirs: &Design) -> ImportStats {
    let mut st = ImportStats::default();
    for c in p.nl.cell_ids() {
        if p.nl.cell(c).bel.is_some() {
            continue;
        }
        let name = p.nl.cell(c).name.clone();
        if counter_named(&name) {
            continue;
        }
        if let Some(l) = theirs.cells.get(&name).and_then(|t| t.loc.clone()) {
            p.nl.cell_mut(c).bel = Some(l);
            st.by_name += 1;
        }
    }
    // Cluster members still unplaced: root's tile shifted, slot from z.
    let clusters: Vec<(CellId, crate::pack::Cluster)> = p.clusters.iter().map(|(r, cl)| (*r, cl.clone())).collect();
    for (root, cl) in clusters {
        let (root_z, members) = (cl.root_abs_z, cl.members);
        let Some(Loc::Grid { x, y, site, bel }) = p.nl.cell(root).bel.clone() else {
            continue;
        };
        let rz = root_z.or_else(|| slot_of_bel(&bel)).unwrap_or(0);
        for (m, off) in members {
            if p.nl.cell(m).bel.is_some() {
                continue;
            }
            let z = if off.abs { off.z } else { rz + off.z };
            if let Some(b) = bel_for_slot(z) {
                p.nl.cell_mut(m).bel = Some(Loc::Grid {
                    x: x + off.dx,
                    y: y + off.dy,
                    site: site.clone(),
                    bel: b,
                });
                st.by_cluster += 1;
            }
        }
    }
    place_leftover_luts(p, &mut st);
    let mut used: HashMap<String, String> = HashMap::new();
    for c in p.nl.cell_ids() {
        let cell = p.nl.cell(c);
        match &cell.bel {
            None if !matches!(cell.ty.as_str(), "PSEUDO_GND" | "PSEUDO_VCC") => st.unplaced.push(cell.name.clone()),
            Some(l) => {
                if let Some(prev) = used.insert(l.text(), cell.name.clone())
                    && st.collisions.len() < 20
                {
                    st.collisions.push(format!("{} and {} on {}", prev, cell.name, l.text()));
                }
            }
            None => {}
        }
    }
    st
}

/// A LUT the other placement has no bel for (packed differently there): the
/// nearest slice letter, by tile distance from the LUT's first user, whose
/// 6LUT and 5LUT are both free.
fn place_leftover_luts(p: &mut Packer, st: &mut ImportStats) {
    let mut taken: BTreeSet<(i32, i32, String, char)> = BTreeSet::new();
    let mut slices: BTreeSet<(i32, i32, String)> = BTreeSet::new();
    for c in p.nl.cell_ids() {
        if let Some(Loc::Grid { x, y, site, bel }) = &p.nl.cell(c).bel
            && site.starts_with("SLICE")
        {
            slices.insert((*x, *y, site.clone()));
            if bel.ends_with("LUT") {
                taken.insert((*x, *y, site.clone(), bel.chars().next().unwrap_or('?')));
            }
        }
    }
    for c in p.nl.cell_ids() {
        if p.nl.cell(c).bel.is_some() || p.nl.cell(c).ty != "SLICE_LUTX" {
            continue;
        }
        let anchor =
            p.nl.cell(c)
                .net("O6")
                .and_then(|n| p.nl.net(n).users.first().map(|(u, _)| *u))
                .and_then(|u| match &p.nl.cell(u).bel {
                    Some(Loc::Grid { x, y, .. }) => Some((*x, *y)),
                    _ => None,
                });
        let Some((ax, ay)) = anchor else { continue };
        let best = slices
            .iter()
            .flat_map(|(x, y, s)| ['A', 'B', 'C', 'D'].into_iter().map(move |l| (*x, *y, s.clone(), l)))
            .filter(|k| !taken.contains(k))
            .min_by_key(|(x, y, s, l)| ((x - ax).abs() + (y - ay).abs(), s.clone(), *l));
        if let Some((x, y, site, l)) = best {
            taken.insert((x, y, site.clone(), l));
            p.nl.cell_mut(c).bel = Some(Loc::Grid {
                x,
                y,
                site,
                bel: format!("{l}6LUT"),
            });
            st.by_cluster += 1;
        }
    }
}

/// Inputs of a LUT cell as (logical input index, net), from its X_ORIG_PORT
/// markers.
fn lut_inputs(p: &Packer, c: CellId) -> Vec<(usize, NetId)> {
    let cell = p.nl.cell(c);
    let mut v = Vec::new();
    for k in 1..=6 {
        let pin = format!("A{k}");
        let Some(n) = cell.net(&pin) else { continue };
        if let Some(Value::String(o)) = cell.attrs.get(&format!("X_ORIG_PORT_{pin}")) {
            for tok in o.split_whitespace() {
                if let Some(i) = tok.strip_prefix('I').and_then(|s| s.parse().ok()) {
                    v.push((i, n));
                }
            }
        }
    }
    v
}

fn is_memory(p: &Packer, c: CellId) -> bool {
    let a = &p.nl.cell(c).attrs;
    a.contains_key("X_LUT_AS_DRAM") || a.contains_key("X_LUT_AS_SRL")
}

#[derive(Debug, Default)]
pub struct LegaliseStats {
    pub fractured: usize,
    pub memory_a6: usize,
    pub ps7_ties: usize,
    pub over_five: Vec<String>,
}

/// Post-placement legalisation (see the module comment). `ps7_inputs`: the
/// PS7 bel's input pins (from the site model).
pub fn legalise(p: &mut Packer, ps7_inputs: &[String]) -> LegaliseStats {
    let mut st = LegaliseStats::default();
    let vcc = p.nl.vcc;
    // (tile x, y, site, letter) -> (5LUT, 6LUT)
    // (tile x, y, site, letter) -> (5LUT, 6LUT)
    type Letter = (i32, i32, String, char);
    let mut letters: BTreeMap<Letter, (Option<CellId>, Option<CellId>)> = BTreeMap::new();
    for c in p.nl.cell_ids() {
        if p.nl.cell(c).ty != "SLICE_LUTX" {
            continue;
        }
        let Some(Loc::Grid { x, y, site, bel }) = p.nl.cell(c).bel.clone() else {
            continue;
        };
        let l = bel.chars().next().unwrap_or('?');
        let e = letters.entry((x, y, site, l)).or_default();
        if bel.ends_with("5LUT") {
            e.0 = Some(c);
        } else if bel.ends_with("6LUT") {
            e.1 = Some(c);
        }
    }
    for ((_, _, _, _), (l5, l6)) in letters {
        let Some(l5) = l5 else { continue };
        if is_memory(p, l5) {
            if let Some(l6) = l6 {
                p.nl.add_port(l6, "A6", Dir::Input);
                p.nl.connect(l6, "A6", vcc);
                st.memory_a6 += 1;
            }
            continue;
        }
        let i5 = lut_inputs(p, l5);
        let i6 = l6.map(|c| lut_inputs(p, c)).unwrap_or_default();
        let nets: BTreeSet<NetId> = i5.iter().chain(&i6).map(|(_, n)| *n).collect();
        if nets.len() > 5 {
            st.over_five.push(p.nl.cell(l5).name.clone());
        }
        for c in std::iter::once(l5).chain(l6) {
            for k in 1..=6 {
                p.nl.disconnect(c, &format!("A{k}"));
                p.nl.cell_mut(c).attrs.remove(&format!("X_ORIG_PORT_A{k}"));
            }
        }
        for (idx, n) in nets.iter().enumerate() {
            let pin = format!("A{}", idx + 1);
            for (c, ins) in [(Some(l5), &i5), (l6, &i6)] {
                let Some(c) = c else { continue };
                let names: Vec<String> = ins.iter().filter(|(_, x)| x == n).map(|(i, _)| format!("I{i}")).collect();
                if names.is_empty() {
                    continue;
                }
                p.nl.add_port(c, &pin, Dir::Input);
                p.nl.connect(c, &pin, *n);
                p.nl.cell_mut(c).attrs.insert(format!("X_ORIG_PORT_{pin}"), json!(names.join(" ")));
            }
        }
        p.nl.rename_port(l5, "O6", "O5");
        p.nl.cell_mut(l5).attrs.remove("X_ORIG_PORT_O6");
        p.nl.cell_mut(l5).attrs.insert("X_ORIG_PORT_O5".into(), json!("O"));
        if let Some(l6) = l6 {
            p.nl.add_port(l6, "A6", Dir::Input);
            p.nl.connect(l6, "A6", vcc);
        }
        st.fractured += 1;
    }
    let gnd = p.nl.gnd;
    for c in p.nl.cells_of("PS7_PS7") {
        for pin in ps7_inputs {
            if pin.contains("_PAD_") || ["TEST", "DEBUGSELECT", "MIO", "DDR"].iter().any(|s| pin.starts_with(s)) {
                continue;
            }
            let driven = p.nl.cell(c).net(pin).is_some_and(|n| p.nl.driver(n).is_some());
            if driven {
                continue;
            }
            p.nl.add_port(c, pin, Dir::Input);
            p.nl.connect(c, pin, gnd);
            st.ps7_ties += 1;
        }
    }
    st
}

/// The PS7 bel's input pins from a nextpnr-xilinx-meta site type file.
pub fn ps7_inputs(meta_dir: &std::path::Path) -> Result<Vec<String>, String> {
    let path = meta_dir.join("site_type_PS7.json");
    let v: Value =
        serde_json::from_str(&std::fs::read_to_string(&path).map_err(|e| format!("{}: {e}", path.display()))?).map_err(|e| e.to_string())?;
    Ok(v["PS7"]["bels"]["PS7"]["pins"]
        .as_object()
        .into_iter()
        .flatten()
        .filter(|(_, d)| d["dir"].as_str() == Some("INPUT"))
        .map(|(k, _)| k.clone())
        .collect())
}
