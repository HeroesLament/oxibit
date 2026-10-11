//! Build the routing graph over the whole device and check a routed design's
//! FASM against it: every pip line must be a usable edge, and every net must
//! trace from a driver.
//!
//! cargo run --release -p oxibit-fasm --example graph_check -- DB_ROOT PART TOP.FASM

use std::collections::{BTreeMap, HashMap, HashSet};
use std::path::Path;
use std::time::Instant;

use oxibit_fasm::db::Database;
use oxibit_fasm::route::{Enable, Graph, GraphOpts, PipLookup, pip_lines};

fn peak_rss_mb() -> Option<u64> {
    let status = std::fs::read_to_string("/proc/self/status").ok()?;
    let line = status.lines().find(|l| l.starts_with("VmHWM:"))?;
    let kb: u64 = line.split_whitespace().nth(1)?.parse().ok()?;
    Some(kb / 1024)
}

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().collect();
    let [_, db_root, part, fasm_path] = &args[..] else {
        return Err("usage: graph_check DB_ROOT PART TOP.FASM".into());
    };

    let t = Instant::now();
    let db = Database::open(Path::new(db_root), part)?;
    println!("database: {:.2} s", t.elapsed().as_secs_f64());

    let t = Instant::now();
    let defaults = std::env::var_os("DEFAULTS").is_some();
    let g = Graph::build_with(&db, None, GraphOpts { defaults })?;
    println!("default pseudo-pips as edges: {defaults}");
    println!(
        "graph: {} nodes, {} edges, {:.2} s, peak RSS {} MB",
        g.nodes(),
        g.edges(),
        t.elapsed().as_secs_f64(),
        peak_rss_mb().map_or("?".into(), |m| m.to_string())
    );

    let fasm = std::fs::read_to_string(fasm_path).map_err(|e| format!("{fasm_path}: {e}"))?;
    let lines = pip_lines(&fasm);

    // Classify every pip line.
    let mut counts: BTreeMap<&str, usize> = BTreeMap::new();
    let mut bad: BTreeMap<&str, Vec<String>> = BTreeMap::new();
    let mut drives: HashMap<u32, Vec<u32>> = HashMap::new(); // src node -> dst nodes
    let mut driven: HashSet<u32> = HashSet::new();
    for line in &lines {
        let kind = match g.lookup_pip(line) {
            PipLookup::Edge { src, dst, pip, .. } => {
                drives.entry(src).or_default().push(dst);
                driven.insert(dst);
                match pip.enable {
                    Enable::Feature(_) => "edge, feature",
                    Enable::Fixed => "edge, fixed",
                    Enable::Relay(_) => "edge, relay",
                    Enable::Default => "edge, default",
                }
            }
            PipLookup::SameNode(_) => "same node",
            PipLookup::NoPip => "no such usable pip",
            PipLookup::NoWire => "no such wire",
            PipLookup::NoTile => "no such tile",
            PipLookup::Malformed => "malformed",
        };
        *counts.entry(kind).or_default() += 1;
        if !kind.starts_with("edge") {
            bad.entry(kind).or_default().push(line.clone());
        }
    }
    println!("pip lines: {}", lines.len());
    for (k, n) in &counts {
        println!("  {k:24} {n}");
    }
    for (k, v) in &bad {
        let mut v = v.clone();
        v.sort();
        println!("  {k}, first {}:", v.len().min(10));
        for l in v.iter().take(10) {
            println!("    {l}");
        }
    }

    // Net roots: source nodes of the design's pips that no pip in the design
    // drives. Each should be a site output (or a fixed-wired one); report the
    // wire-name families so an odd one stands out.
    let roots: HashSet<u32> = drives.keys().copied().filter(|n| !driven.contains(n)).collect();
    let mut families: BTreeMap<String, usize> = BTreeMap::new();
    for line in &lines {
        if let PipLookup::Edge { src, pip, .. } = g.lookup_pip(line) {
            if roots.contains(&src) {
                let family: String = pip.src.trim_end_matches(|c: char| c.is_ascii_digit()).to_string();
                *families.entry(family).or_default() += 1;
            }
        }
    }
    println!("net roots: {} (src wire families of the pips leaving them):", roots.len());
    for (f, n) in &families {
        println!("  {f:32} {n}");
    }
    println!("peak RSS {} MB", peak_rss_mb().map_or("?".into(), |m| m.to_string()));
    Ok(())
}
