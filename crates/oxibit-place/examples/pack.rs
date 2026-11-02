//! Pack a primitive netlist; given nextpnr's placed design of the same
//! netlist, compare the packing, take its bels, legalise, and write the
//! placed design for oxibit-pnr.
//!
//!   pack DB_FAMILY_DIR PART XDC META_DIR SYNTH.json OUT.json [REFERENCE_PLACED.json | --place [SEED]]
//!
//! stdout: JSON lines (pack stats, comparison, placement).

use oxibit_place::{compare, netlist::Netlist, pack, place};
use oxibit_pnr::packed::Design;
use serde_json::{Value, json};

fn read(path: &str) -> Result<Value, String> {
    serde_json::from_str(&std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?).map_err(|e| format!("{path}: {e}"))
}

fn run() -> Result<bool, String> {
    let args: Vec<String> = std::env::args().collect();
    let [_, db, part, xdc, meta, synth, out, rest @ ..] = &args[..] else {
        return Err("usage: pack DB_FAMILY_DIR PART XDC META_DIR SYNTH.json OUT.json [REFERENCE_PLACED.json]".into());
    };
    let t0 = std::time::Instant::now();
    let dev = oxibit_place::device::Device::open(std::path::Path::new(db), part)?;
    let props = oxibit_place::xdc::parse(&std::fs::read_to_string(xdc).map_err(|e| format!("{xdc}: {e}"))?);
    let nl = Netlist::from_json(&read(synth)?)?;
    let fold = std::env::var("OXIBIT_PACK_FOLD").map(|v| v != "0").unwrap_or(true);
    let mut p = pack::pack_with(nl, &dev, &props, fold)?;
    let mut types: std::collections::BTreeMap<String, usize> = Default::default();
    for c in p.nl.cell_ids() {
        *types.entry(p.nl.cell(c).ty.clone()).or_default() += 1;
    }
    println!(
        "{}",
        json!({"pack": {"stats": p.stats, "cells": p.nl.cell_ids().len(), "types": types, "clusters": p.clusters.len(),
                        "clustered_cells": p.cluster_of.len(), "seconds": t0.elapsed().as_secs_f64()}})
    );
    let mut ok = true;
    if rest.first().map(String::as_str) == Some("--place") {
        // oxibit's own placer.
        let seed = rest.get(1).and_then(|s| s.parse().ok()).unwrap_or(1);
        let sites = oxibit_place::placer::Sites::from_device(&dev);
        let mut fopts = oxibit_place::placer::FillOptions::default();
        if let Some(m) = std::env::var("OXIBIT_FILL_BYPASS").ok().and_then(|v| v.parse().ok()) {
            fopts.bypass_ffs_per_slice = m;
        }
        let (macros, fs) = oxibit_place::placer::fill(&p, &fopts);
        println!(
            "{}",
            json!({"fill": {"chains": fs.chains, "chain_slices": fs.chain_slices, "memory_slices": fs.memory_slices, "srl_slices": fs.srl_slices,
                            "logic_slices": fs.logic_slices, "lone_lut_letters": fs.lone_lut_letters, "ff2_used": fs.ff2_used,
                            "control_sets": fs.control_sets, "leftovers": fs.leftovers.len(), "leftover_examples": fs.leftovers.iter().take(6).collect::<Vec<_>>(),
                            "sites": {"slices": sites.slices.len(), "slicem": sites.slices.iter().filter(|s| s.m).count(), "dsp": sites.dsp.len(),
                                      "ramb36": sites.ramb36.len(), "ramb18": sites.ramb18.len(), "bufg": sites.bufg.len(), "ps7": sites.ps7.len()}}})
        );
        let mut opts = oxibit_place::placer::Options { seed, ..Default::default() };
        if let Some(m) = std::env::var("OXIBIT_PLACE_MOVES").ok().and_then(|v| v.parse().ok()) {
            opts.max_moves_per_obj = m;
        }
        if let Some(m) = std::env::var("OXIBIT_PLACE_T0").ok().and_then(|v| v.parse().ok()) {
            opts.t0_scale = m;
        }
        if let Some(m) = std::env::var("OXIBIT_PLACE_INNER").ok().and_then(|v| v.parse().ok()) {
            opts.inner = m;
        }
        if let Some(m) = std::env::var("OXIBIT_PLACE_UNIT").ok().and_then(|v| v.parse().ok()) {
            opts.unit_frac = m;
        }
        if let Some(m) = std::env::var("OXIBIT_PLACE_UNIT_WINDOW").ok().and_then(|v| v.parse().ok()) {
            opts.unit_window = m;
        }
        if let Some(m) = std::env::var("OXIBIT_PLACE_POLISH").ok().and_then(|v| v.parse().ok()) {
            opts.polish = m;
        }
        if let Some(m) = std::env::var("OXIBIT_PLACE_PIN_CAP").ok().and_then(|v| v.parse().ok()) {
            opts.tile_pin_cap = m;
        }
        if let Some(m) = std::env::var("OXIBIT_PLACE_PIN_WEIGHT").ok().and_then(|v| v.parse().ok()) {
            opts.pin_weight = m;
        }
        let ps = oxibit_place::placer::place(&mut p, &dev, &sites, macros, &opts)?;
        println!("{}", json!({"hpwl_measured": oxibit_place::placer::hpwl(&p, 200)}));
        let lg = place::legalise(&mut p, &place::ps7_inputs(std::path::Path::new(meta))?);
        let unplaced: Vec<String> = p.nl.cell_ids().into_iter().filter(|&c| p.nl.cell(c).bel.is_none()).map(|c| p.nl.cell(c).name.clone()).collect();
        ok = unplaced.is_empty() && lg.over_five.is_empty();
        println!(
            "{}",
            json!({"placer": {"objects": ps.objects, "chains": ps.chains_placed, "hpwl_start": ps.hpwl_start, "hpwl_end": ps.hpwl_end,
                              "moves": ps.moves, "accepted": ps.accepted, "seconds": ps.seconds,
                              "pins": {"cap": opts.tile_pin_cap, "weight": opts.pin_weight, "excess_start": ps.pin_excess_start, "excess_end": ps.pin_excess_end,
                                       "tiles_over_cap": ps.tiles_over_cap, "tile_max": ps.tile_pins_max, "bypass_ffs_per_slice": fopts.bypass_ffs_per_slice},
                              "steps": ps.trace.len(), "trace": ps.trace.iter().step_by((ps.trace.len() / 25).max(1)).map(|t| json!([(t.0 * 100.0).round() / 100.0, t.1.round(), (t.2 * 1000.0).round() / 1000.0, t.3])).collect::<Vec<_>>(), "unplaced": unplaced.len(),
                              "unplaced_examples": unplaced.iter().take(6).collect::<Vec<_>>(), "fractured": lg.fractured, "over_five": lg.over_five.len()}})
        );
    } else if let Some(r) = rest.first() {
        let theirs = Design::from_json(&read(r)?)?;
        let diff = compare::compare(&p.nl.to_design(), &theirs);
        println!("{}", json!({"compare": diff.to_json(), "differences": diff.total()}));
        let st = place::import(&mut p, &theirs);
        let lg = place::legalise(&mut p, &place::ps7_inputs(std::path::Path::new(meta))?);
        println!("{}", json!({"reference_hpwl": oxibit_place::placer::hpwl(&p, 200)}));
        ok = st.unplaced.is_empty() && st.collisions.is_empty() && lg.over_five.is_empty();
        println!(
            "{}",
            json!({"place": {"by_name": st.by_name, "by_cluster": st.by_cluster, "unplaced": st.unplaced.len(),
                             "unplaced_examples": st.unplaced.iter().take(8).collect::<Vec<_>>(),
                             "collisions": st.collisions, "fractured": lg.fractured, "memory_a6": lg.memory_a6,
                             "ps7_ties": lg.ps7_ties, "over_five": lg.over_five.iter().take(8).collect::<Vec<_>>()}})
        );
    }
    let d = p.nl.to_design();
    let mut v = d.to_json();
    let (ports, names) = p.nl.ports_and_names();
    v["modules"][&p.nl.top]["ports"] = ports;
    v["modules"][&p.nl.top]["netnames"] = names;
    std::fs::write(out, serde_json::to_string(&v).map_err(|e| e.to_string())?).map_err(|e| format!("{out}: {e}"))?;
    Ok(ok)
}

fn main() {
    match run() {
        Ok(true) => {}
        Ok(false) => std::process::exit(1),
        Err(e) => {
            eprintln!("pack: {e}");
            std::process::exit(3);
        }
    }
}
