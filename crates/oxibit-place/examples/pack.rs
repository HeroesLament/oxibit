//! Pack a primitive netlist; given nextpnr's placed design of the same
//! netlist, compare the packing, take its bels, legalise, and write the
//! placed design for oxibit-pnr.
//!
//!   pack [--packer rules|rules+score] [--gp] DB_FAMILY_DIR PART XDC META_DIR SYNTH.json OUT.json [REFERENCE_PLACED.json]
//!
//! `--gp` runs oxibit's own global placement (`placer`) on the packed
//! slices and reports wirelength stage by stage; with a reference
//! placement, also both wirelengths over the nets the reference covers.
//! The written design then carries that placement, bels and all; a
//! reference placement is only compared against, not imported. Implies `--packer rules+score` unless one is given.
//!
//! `--no-fracture` keeps one LUT to a letter (no 6LUT + 5LUT sharing).
//!
//! `--pins FILE` gives the placer, and the wirelength it reports, the PS
//! pins where they meet the interconnect (FILE from `oxibit-pnr`'s
//! `ps_entries` example) instead of all at the PS site's tile.
//!
//! `--macro-rounds N` (default 0): after placing, up to N rounds of moving
//! each macro to a free site that shortens its nets and placing the slices
//! again.
//!
//! `--no-compact`: leave best-choice's slices as its merges made them,
//! not compacted down to the count order of appearance reaches.
//!
//! `--bins N`: the placer's bin grid, N a side (default 32). Changing it
//! by one or two perturbs the placement without changing the problem,
//! which is how the noise of a comparison is measured.
//!
//! OXIBIT_PT=EVERY[,BETA[,EXP]]: timing-weighted global placement
//! (`placer::timing`), the weights refreshed every EVERY Nesterov
//! iterations (unset: `TimingOpts::default`, every 10). OXIBIT_PT=0 turns
//! it off.
//!
//! `--packer` also packs LUTs and flops into slices (`Packer::slices`) and
//! reports what the slices cut: always for `rules` (order of appearance),
//! and beside it for `rules+score` (best-choice by `merging_score`, held
//! to the slice count `rules` reached).
//!
//! stdout: JSON lines (pack stats, slices, comparison, placement).

use oxibit_place::pack::ctx::ScoreParams;
use oxibit_place::pack::slices::{PackerKind, SliceOpts, pack_slices};
use oxibit_place::{compare, netlist::Netlist, pack, place, placer};
use oxibit_pnr::packed::Design;
use serde_json::{Value, json};

fn read(path: &str) -> Result<Value, String> {
    serde_json::from_str(&std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?).map_err(|e| format!("{path}: {e}"))
}

fn run() -> Result<bool, String> {
    let usage = "usage: pack [--packer rules|rules+score] [--gp] [--no-fracture] [--pins PS_ENTRIES.json] [--macro-rounds N] [--no-compact] [--bins N] DB_FAMILY_DIR PART XDC META_DIR SYNTH.json OUT.json [REFERENCE_PLACED.json]";
    let mut args: Vec<String> = std::env::args().collect();
    let mut packer: Option<PackerKind> = None;
    if let Some(i) = args.iter().position(|a| a == "--packer") {
        packer = Some(args.get(i + 1).ok_or(usage)?.parse()?);
        args.drain(i..=i + 1);
    }
    let gp = args.iter().position(|a| a == "--gp").map(|i| args.remove(i)).is_some();
    let mut pins = placer::PinMap::default();
    if let Some(i) = args.iter().position(|a| a == "--pins") {
        pins = placer::PinMap::from_ps_entries(&read(args.get(i + 1).ok_or(usage)?)?);
        args.drain(i..=i + 1);
    }
    let mut macro_rounds = 0usize;
    if let Some(i) = args.iter().position(|a| a == "--macro-rounds") {
        macro_rounds = args.get(i + 1).ok_or(usage)?.parse().map_err(|e| format!("--macro-rounds: {e}"))?;
        args.drain(i..=i + 1);
    }
    let mut bins: Option<usize> = None;
    if let Some(i) = args.iter().position(|a| a == "--bins") {
        bins = Some(args.get(i + 1).ok_or(usage)?.parse().map_err(|e| format!("--bins: {e}"))?);
        args.drain(i..=i + 1);
    }
    let no_compact = args.iter().position(|a| a == "--no-compact").map(|i| args.remove(i)).is_some();
    let mut params = ScoreParams::default();
    if args.iter().position(|a| a == "--no-fracture").map(|i| args.remove(i)).is_some() {
        params.fracture = None;
    }
    if gp && packer.is_none() {
        packer = Some(PackerKind::RulesScore);
    }
    let [_, db, part, xdc, meta, synth, out, rest @ ..] = &args[..] else {
        return Err(usage.into());
    };
    let t0 = std::time::Instant::now();
    let dev = oxibit_place::device::Device::open(std::path::Path::new(db), part)?;
    let props = oxibit_place::xdc::parse(&std::fs::read_to_string(xdc).map_err(|e| format!("{xdc}: {e}"))?);
    let nl = Netlist::from_json(&read(synth)?)?;
    let mut p = pack::pack(nl, &dev, &props)?;
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
    if let Some(kind) = packer {
        let t = std::time::Instant::now();
        // Slice sites free for slice logic, for the budget below.
        let room = placer::Problem::build(&p, &dev, 200, &pins).ok().map(|prob| prob.slice_room());
        // Order of appearance: reported, and its slice count is the budget
        // best-choice is compacted to unless `--no-compact` (or, on a tight
        // device, `slices::budget`).
        let base = pack_slices(
            &mut p,
            &SliceOpts {
                params,
                ..SliceOpts::default()
            },
        );
        ok &= base.illegal.is_empty();
        let mut v = json!({"packer": if kind == PackerKind::Rules { "rules" } else { "rules+score" }, "rules": base.to_json()});
        let dense = oxibit_place::pack::slices::dense_report(&p, params);
        ok &= dense.illegal.is_empty();
        v["dense"] = dense.to_json();
        let budget = (!no_compact).then(|| oxibit_place::pack::slices::budget(base.stats.slices, dense.stats.slices, room));
        v["room"] = json!(room);
        v["budget"] = json!(budget);
        if kind == PackerKind::RulesScore {
            let scored = pack_slices(
                &mut p,
                &SliceOpts {
                    kind,
                    params,
                    max_slices: budget,
                    ..SliceOpts::default()
                },
            );
            ok &= scored.illegal.is_empty();
            v["rules+score"] = scored.to_json();
        }
        v["seconds"] = json!(t.elapsed().as_secs_f64());
        println!("{}", json!({ "slices": v }));
    }
    if gp {
        let t = std::time::Instant::now();
        // RePlAce release default, `-initial_place_max_fanout 200`.
        let mut opts = placer::GpOpts {
            macro_rounds,
            ..placer::GpOpts::default()
        };
        if let Some(bins) = bins {
            opts.nesterov.bins = bins;
        }
        if let Ok(v) = std::env::var("OXIBIT_PT") {
            let mut parts = v.split(',').map(|s| s.trim().parse::<f64>().map_err(|_| format!("OXIBIT_PT: {s} is not a number")));
            let t = &mut opts.timing;
            t.every = parts.next().transpose()?.unwrap_or(0.0) as usize;
            t.beta = parts.next().transpose()?.unwrap_or(t.beta);
            t.exp = parts.next().transpose()?.unwrap_or(t.exp);
        }
        let rep = placer::place_design(&mut p, &dev, &opts, 200, &pins)?;
        let lg = place::legalise(&mut p, &place::ps7_inputs(std::path::Path::new(meta))?);
        ok &= rep.bels.ok() && lg.over_five.is_empty();
        let mut v = rep.to_json();
        v["ps7_ties"] = json!(lg.ps7_ties);
        v["pins_known"] = json!(pins.len());
        if let Some(r) = rest.first() {
            // Cell-accurate, same netlist, the nets both placements cover.
            let theirs = Design::from_json(&read(r)?)?;
            let xy = |l: Option<&oxibit_pnr::packed::Loc>| l.and_then(|l| placer::loc_xy(&dev, l));
            let ours = placer::cell_net_hpwl(&p, 200, &pins, &|c| xy(p.nl.cell(c).bel.as_ref()));
            let reference = placer::cell_net_hpwl(&p, 200, &pins, &|c| xy(theirs.cells.get(&p.nl.cell(c).name).and_then(|t| t.loc.as_ref())));
            let (a, b, nets) = placer::common_hpwl(&ours, &reference);
            v["reference"] = json!({"nets": nets, "hpwl_ours": a, "hpwl_reference": b, "ratio": a / b});
        }
        v["seconds"] = json!(t.elapsed().as_secs_f64());
        println!("{}", json!({ "gp": v }));
    } else if let Some(r) = rest.first() {
        let mut theirs = Design::from_json(&read(r)?)?;
        // A placement that names bels by absolute site (`SLICE_X1Y2/A5LUT`)
        // goes to the grid form first: `place::legalise` finds the letters
        // holding two LUTs by grid location, and without that pass their
        // 5LUTs keep an O6 port no bel has.
        for cell in theirs.cells.values_mut() {
            if let Some(oxibit_pnr::packed::Loc::Site { site, bel }) = &cell.loc
                && let Some(grid) = dev.grid_loc(site, bel)
            {
                cell.loc = Some(grid);
            }
        }
        let diff = compare::compare(&p.nl.to_design(), &theirs);
        println!("{}", json!({"compare": diff.to_json(), "differences": diff.total()}));
        let st = place::import(&mut p, &theirs);
        let lg = place::legalise(&mut p, &place::ps7_inputs(std::path::Path::new(meta))?);
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
