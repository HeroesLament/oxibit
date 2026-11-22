//! Build an ECP5 routing graph, print its size, a wire's arcs, or check the
//! on-demand arcs against the explicit arc list.
//!
//!   cargo run --release -p oxibit-ecp5 --example ecp5_graph -- \
//!       [--db <trellis/database>] [--check] DEVICE [X Y WIRE]
//!
//! `--check`: every explicit arc (libtrellis's list) must be the downhill
//! arc of its source and the uphill arc of its sink, and nothing else:
//! counts and an order-independent hash sum over both directions agree.

#![expect(
    clippy::print_stderr,
    reason = "a command-line example reports on stderr"
)]

use std::collections::HashSet;
use std::hash::BuildHasher;
use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Instant;

use oxibit_ecp5::graph::{Arc, Wire};
use oxibit_ecp5::{DeviceDb, Graph};

fn default_db() -> PathBuf {
    if let Some(p) = std::env::var_os("OXIBIT_TRELLIS_DB") {
        return PathBuf::from(p);
    }
    let home = std::env::var_os("HOME")
        .map(PathBuf::from)
        .unwrap_or_default();
    home.join("oss-cad-suite/share/trellis/database")
}

/// Count and wrapping hash sum of a set of arcs.
#[derive(Debug, Default, PartialEq, Eq)]
struct Tally {
    count: u64,
    sum: u64,
}

impl Tally {
    fn add(&mut self, a: &Arc, hasher: &std::collections::hash_map::RandomState) {
        self.count += 1;
        self.sum = self.sum.wrapping_add(hasher.hash_one(a));
    }
}

fn check(g: &Graph) -> Result<(), String> {
    let rs = std::collections::hash_map::RandomState::new();
    let mut explicit = Tally::default();
    let mut froms: HashSet<Wire> = HashSet::new();
    let mut tos: HashSet<Wire> = HashSet::new();
    for a in g.explicit_arcs() {
        explicit.add(&a, &rs);
        froms.insert(a.from);
        tos.insert(a.to);
    }
    let mut down = Tally::default();
    for &w in &froms {
        for a in g.downhill(w) {
            down.add(&a, &rs);
        }
    }
    let mut up = Tally::default();
    for &w in &tos {
        for a in g.uphill(w) {
            up.add(&a, &rs);
        }
    }
    eprintln!(
        "explicit {} arcs ({} sources, {} sinks); downhill {}; uphill {}",
        explicit.count,
        froms.len(),
        tos.len(),
        down.count,
        up.count
    );
    if down != explicit || up != explicit {
        return Err("on-demand arcs differ from the explicit list".into());
    }
    eprintln!("check: identical");
    Ok(())
}

fn run() -> Result<(), String> {
    let mut db = default_db();
    let mut do_check = false;
    let mut pos = Vec::new();
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--db" => db = PathBuf::from(args.next().ok_or("--db needs a directory")?),
            "--check" => do_check = true,
            _ => pos.push(a),
        }
    }
    let Some(device) = pos.first() else {
        return Err("usage: ecp5_graph [--db DIR] [--check] DEVICE [X Y WIRE]".into());
    };
    let t = Instant::now();
    let ddb = DeviceDb::load(&db, device).map_err(|e| e.to_string())?;
    let t_db = t.elapsed();
    let g = Graph::build(&ddb).map_err(|e| e.to_string())?;
    eprintln!(
        "{device}: {} tiles, {} routing types, {} names; db {:.2?}, graph {:.2?}",
        g.tiles.len(),
        g.types.len(),
        g.names.len(),
        t_db,
        t.elapsed().saturating_sub(t_db)
    );
    if do_check {
        let t = Instant::now();
        check(&g)?;
        eprintln!("check took {:.2?}", t.elapsed());
    }
    if let [_, x, y, wire] = pos.as_slice() {
        let x: i16 = x.parse().map_err(|e| format!("x: {e}"))?;
        let y: i16 = y.parse().map_err(|e| format!("y: {e}"))?;
        let name = g
            .name_id_of(wire)
            .ok_or_else(|| format!("no wire named {wire}"))?;
        let w = Wire { x, y, name };
        for (dir, arcs) in [("downhill", g.downhill(w)), ("uphill", g.uphill(w))] {
            for arc in arcs {
                let p = g.pip(&arc);
                let other = if dir == "downhill" { arc.to } else { arc.from };
                eprintln!(
                    "{dir:8} {:<28} via {} {:?} {} <- {}",
                    g.wire_name(other),
                    g.tiles[arc.tile as usize].name,
                    p.kind,
                    p.sink,
                    p.source
                );
            }
        }
    }
    Ok(())
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("ecp5_graph: {e}");
            ExitCode::FAILURE
        }
    }
}
