//! Build an ECP5 device's bels, count them by type, and check every pin
//! against the routing graph: an input pin's wire must have an arc into it
//! or another bel's output on it (split-slice F1 and the LUT pair's carry
//! are bel to bel), and an output pin's wire an arc out of it or another
//! bel's input on it.
//!
//!   cargo run --release -p oxibit-ecp5 --example ecp5_bels -- \
//!       [--db <trellis/database>] [--split] DEVICE
//!
//! Pins that fail are grouped by (bel type, pin) with a count and one
//! example wire. The DSP cascade pins (MULT18X18D ROC/SRIA/SROB at the
//! ends of a chain, ALU54B SIGNEDR) dangle in the database itself; any
//! other dangling pin is an error (exit status 1).

#![expect(
    clippy::print_stderr,
    reason = "a command-line example reports on stderr"
)]

use std::collections::{BTreeMap, HashMap};
use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Instant;

use oxibit_ecp5::bels::PinDir;
use oxibit_ecp5::graph::Wire;
use oxibit_ecp5::{Bels, DeviceDb, Graph, SliceMode};

fn default_db() -> PathBuf {
    if let Some(p) = std::env::var_os("OXIBIT_TRELLIS_DB") {
        return PathBuf::from(p);
    }
    let home = std::env::var_os("HOME")
        .map(PathBuf::from)
        .unwrap_or_default();
    home.join("oss-cad-suite/share/trellis/database")
}

fn run() -> Result<bool, String> {
    let mut db_root = default_db();
    let mut mode = SliceMode::Whole;
    let mut device = None;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--db" => db_root = PathBuf::from(args.next().ok_or("--db needs a path")?),
            "--split" => mode = SliceMode::Split,
            _ => device = Some(a),
        }
    }
    let device = device.ok_or("usage: ecp5_bels [--db DIR] [--split] DEVICE")?;
    let t0 = Instant::now();
    let db = DeviceDb::load(&db_root, &device).map_err(|e| e.to_string())?;
    let mut g = Graph::build(&db).map_err(|e| e.to_string())?;
    let t1 = Instant::now();
    let bels = Bels::build(&mut g, &db, mode).map_err(|e| e.to_string())?;
    let t2 = Instant::now();
    eprintln!(
        "{device}: {} bels from {} templates; db+graph {:.2}s, bels {:.3}s",
        bels.bels.len(),
        bels.templates.len(),
        (t1 - t0).as_secs_f64(),
        (t2 - t1).as_secs_f64()
    );
    let mut by_type: BTreeMap<&str, usize> = BTreeMap::new();
    for b in 0..bels.bels.len() {
        let id = u32::try_from(b).expect("bel count fits u32");
        *by_type.entry(bels.template(id).ty.as_str()).or_default() += 1;
    }
    for (ty, n) in &by_type {
        eprintln!("  {ty:<16} {n}");
    }

    // Bel inputs and outputs on each wire.
    let mut on_wire: HashMap<Wire, (u32, u32)> = HashMap::new();
    for b in 0..bels.bels.len() {
        let id = u32::try_from(b).expect("bel count fits u32");
        for (p, w) in bels.pins(id) {
            let e = on_wire.entry(w).or_default();
            match p.dir {
                PinDir::In => e.0 += 1,
                PinDir::Out => e.1 += 1,
            }
        }
    }
    let mut pins = 0usize;
    // (bel type, pin) -> (count, example)
    let mut bad: BTreeMap<(String, String), (usize, String)> = BTreeMap::new();
    for b in 0..bels.bels.len() {
        let id = u32::try_from(b).expect("bel count fits u32");
        let ty = &bels.template(id).ty;
        for (p, w) in bels.pins(id) {
            pins += 1;
            let (ins, outs) = on_wire[&w];
            let connected = match p.dir {
                PinDir::In => outs > 0 || !g.uphill(w).is_empty(),
                PinDir::Out => ins > 0 || !g.downhill(w).is_empty(),
            };
            if !connected {
                let e = bad
                    .entry((ty.clone(), p.name.clone()))
                    .or_insert_with(|| (0, g.wire_name(w)));
                e.0 += 1;
            }
        }
    }
    let t3 = Instant::now();
    let n_bad: usize = bad.values().map(|(n, _)| n).sum();
    eprintln!(
        "{pins} pins checked in {:.2}s, {n_bad} unconnected",
        (t3 - t2).as_secs_f64()
    );
    let mut unexpected = 0;
    for ((ty, pin), (n, example)) in &bad {
        let cascade = (ty == "MULT18X18D"
            && ["ROC", "SRIA", "SROB"].iter().any(|c| pin.starts_with(c)))
            || (ty == "ALU54B" && pin == "SIGNEDR");
        if !cascade {
            unexpected += n;
        }
        let note = if cascade { "" } else { "  UNEXPECTED" };
        eprintln!("  {ty}.{pin}: {n} (e.g. {example}){note}");
    }
    Ok(unexpected == 0)
}

fn main() -> ExitCode {
    match run() {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::FAILURE,
        Err(e) => {
            eprintln!("ecp5_bels: {e}");
            ExitCode::from(2)
        }
    }
}
