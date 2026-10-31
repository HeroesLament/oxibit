//! Pack a primitive netlist; given nextpnr's placed design of the same
//! netlist, compare the packing, take its bels, legalise, and write the
//! placed design for oxibit-pnr.
//!
//!   pack DB_FAMILY_DIR PART XDC META_DIR SYNTH.json OUT.json [REFERENCE_PLACED.json]
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
    if let Some(r) = rest.first() {
        let theirs = Design::from_json(&read(r)?)?;
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
