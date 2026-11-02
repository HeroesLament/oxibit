//! Read every SDF file of a database's `timings` and print what it holds:
//! cells, arcs and checks per file, and the arcs of the cells given.
//!
//! cargo run --release -p oxibit-sta --example sdf_census -- DB_FAMILY/timings [FILE:CELLTYPE:INSTANCE ...]

use std::collections::BTreeMap;
use std::path::Path;
use std::process::ExitCode;

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    let Some(dir) = args.get(1) else {
        eprintln!("usage: sdf_census DB_FAMILY/timings [FILE:CELLTYPE:INSTANCE ...]");
        return ExitCode::from(3);
    };
    let lib = match oxibit_sta::sdf::Library::load_dir(Path::new(dir)) {
        Ok(l) => l,
        Err(e) => {
            eprintln!("sdf_census: {e}");
            return ExitCode::from(3);
        }
    };
    let mut per_file: BTreeMap<&str, (usize, usize, usize)> = BTreeMap::new();
    for ((file, _, _), c) in lib.cells() {
        let e = per_file.entry(file.as_str()).or_default();
        e.0 += 1;
        e.1 += c.arcs.len();
        e.2 += c.checks.len();
    }
    for (f, (cells, arcs, checks)) in &per_file {
        println!("{f:22} cells {cells:4} arcs {arcs:5} checks {checks:4}");
    }
    println!("total cells {}", lib.len());
    for spec in &args[2..] {
        let p: Vec<&str> = spec.splitn(3, ':').collect();
        let [f, t, i] = p[..] else { continue };
        match lib.get(f, t, i) {
            Some(c) => {
                println!("== {spec}");
                for a in &c.arcs {
                    println!(
                        "  {} -> {}  {}..{} ps",
                        a.from, a.to, a.delay.min, a.delay.max
                    );
                }
                for k in &c.checks {
                    println!(
                        "  {:?} {} vs {:?} {}  {}..{} ps",
                        k.kind, k.pin, k.edge, k.clock, k.value.min, k.value.max
                    );
                }
            }
            None => println!("== {spec}: not found"),
        }
    }
    ExitCode::SUCCESS
}
