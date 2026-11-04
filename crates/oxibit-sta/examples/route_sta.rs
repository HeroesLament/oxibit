//! Route a placed design as `standalone` does (default router options,
//! OXIBIT_MAX_ITERS honoured) and run setup analysis on the result.
//!
//! cargo run --release -p oxibit-sta --example route_sta -- DB_ROOT PART META_DIR PLACED.json
//!
//! The SDF library is DB_ROOT/timings. stdout: per pass, the route's check
//! line, then one `{"timing": ...}` line (`analyze::Report::json`).
//!
//! - STA_PATHS=n: worst paths in full (default 10).
//! - STA_MODEL=intrinsic|fast|full: interconnect model (default intrinsic).
//! - STA_ROUTES_OUT=FILE: write the last pass's routes (`write_routes`).
//! - STA_ROUTES_IN=FILE: analyse these routes instead of routing (one
//!   pass, no FASM check).
//! - STA_NEXTPNR=1: PLACED.json is a nextpnr routed.json; analyse its own
//!   `ROUTING` (tile pips only, site pips are the analysis's 20 ps arcs).
//! - STA_TD=n: n timing-driven passes after the first: each routes again
//!   from scratch with the previous pass's criticalities
//!   (`router::Opts::criticality`). STA_MAX_CRIT, STA_CRIT_EXP set the
//!   router's `max_crit`, `crit_exp`.

use std::io::{Read, Write};
use std::path::Path;
use std::process::ExitCode;
use std::sync::Arc;
use std::sync::atomic::AtomicBool;

use oxibit_fasm::db::Database;
use oxibit_fasm::route::{Graph, GraphOpts};
use oxibit_pnr::delay::Model;
use oxibit_pnr::flow;
use oxibit_pnr::placed::{self, Meta, Perm};
use oxibit_pnr::router::{Hop, Opts};
use oxibit_report::{Human, HumanStyle, Reporter};
use oxibit_sta::{analyze, sdf};
use serde_json::json;

static CANCEL: AtomicBool = AtomicBool::new(false);

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("route_sta: {e}");
            ExitCode::from(3)
        }
    }
}

fn env_num<T: std::str::FromStr>(k: &str) -> Option<T> {
    std::env::var(k).ok().and_then(|v| v.parse().ok())
}

/// Routes as little-endian u32s: net count, then per net its hop count and
/// (src, dst, tile, pi) per hop.
fn write_routes(path: &str, routes: &[Vec<Hop>]) -> Result<(), String> {
    let mut b: Vec<u8> = Vec::new();
    let mut put = |v: u32| b.extend_from_slice(&v.to_le_bytes());
    put(routes.len() as u32);
    for r in routes {
        put(r.len() as u32);
        for h in r {
            put(h.src);
            put(h.dst);
            put(h.tile);
            put(h.pi);
        }
    }
    let mut f = std::fs::File::create(path).map_err(|e| format!("{path}: {e}"))?;
    f.write_all(&b).map_err(|e| format!("{path}: {e}"))
}

fn read_routes(path: &str) -> Result<Vec<Vec<Hop>>, String> {
    let mut b = Vec::new();
    std::fs::File::open(path)
        .and_then(|mut f| f.read_to_end(&mut b))
        .map_err(|e| format!("{path}: {e}"))?;
    let mut at = 0usize;
    let mut get = || -> Result<u32, String> {
        let s = b
            .get(at..at + 4)
            .ok_or_else(|| format!("{path}: truncated"))?;
        at += 4;
        Ok(u32::from_le_bytes([s[0], s[1], s[2], s[3]]))
    };
    let n = get()? as usize;
    let mut out = Vec::with_capacity(n);
    for _ in 0..n {
        let k = get()? as usize;
        let mut r = Vec::with_capacity(k);
        for _ in 0..k {
            r.push(Hop {
                src: get()?,
                dst: get()?,
                tile: get()?,
                pi: get()?,
            });
        }
        out.push(r);
    }
    Ok(out)
}

/// The tile pips of each routed net from nextpnr's `ROUTING` attributes
/// (`wire;pip;strength` triples, a pip `X{x}Y{y}/DST/SRC`). Every part of
/// a net split by exit gets the whole tree: arrival times walk from that
/// part's root, so edges it cannot reach do not count. Returns the routes
/// and (pips, site pips skipped, same-node pips, misses).
fn nextpnr_routes(
    g: &Graph,
    imp: &placed::Import,
    v: &serde_json::Value,
) -> Result<(Vec<Vec<Hop>>, [usize; 4], Vec<String>), String> {
    use oxibit_fasm::route::PipLookup;
    use std::collections::HashMap;
    let by_xy: HashMap<(i32, i32), String> = g
        .tile_names()
        .filter_map(|t| g.tile_xy(t).map(|xy| (xy, t.to_string())))
        .collect();
    let top = v["modules"]
        .as_object()
        .and_then(|m| m.values().find(|m| m["netnames"].is_object()))
        .ok_or("no module with netnames")?;
    let mut by_bit: HashMap<i64, Vec<Hop>> = HashMap::new();
    let mut st = [0usize; 4];
    let mut missed: Vec<String> = Vec::new();
    for (_name, n) in top["netnames"].as_object().unwrap() {
        let Some(rt) = n["attributes"]["ROUTING"].as_str() else { continue };
        let Some(bit) = n["bits"].get(0).and_then(|b| b.as_i64()) else { continue };
        let f: Vec<&str> = rt.split(';').collect();
        let mut hops = Vec::new();
        for t in f.chunks(3) {
            let Some(&pip) = t.get(1) else { continue };
            if pip.is_empty() {
                continue;
            }
            let Some((tile, rest)) = pip.split_once('/') else { continue };
            let Some((dst, src)) = rest.split_once('/') else { continue };
            // site wires (`SLICE_X0Y0.AMUX`) on either side: inside a site,
            // or the site pin's own pip onto its tile wire
            if dst.contains('.') || src.contains('.') || tile == "SITEWIRE" {
                st[1] += 1;
                continue;
            }
            st[0] += 1;
            let xy = tile
                .strip_prefix('X')
                .and_then(|t| t.split_once('Y'))
                .and_then(|(x, y)| Some((x.parse().ok()?, y.parse().ok()?)));
            let Some(name) = xy.and_then(|xy| by_xy.get(&xy)) else {
                st[3] += 1;
                if missed.len() < 12 {
                    missed.push(format!("{pip} (no tile)"));
                }
                continue;
            };
            match g.lookup_pip(&format!("{name}.{dst}.{src}")) {
                PipLookup::Edge { src, dst, tile, pi, .. } => hops.push(Hop { src, dst, tile, pi }),
                PipLookup::SameNode(_) => st[2] += 1,
                _ => {
                    st[3] += 1;
                    if missed.len() < 12 {
                        missed.push(format!("{name}.{dst}.{src}"));
                    }
                }
            }
        }
        by_bit.insert(bit, hops);
    }
    let routes = imp.bits.iter().map(|b| by_bit.get(b).cloned().unwrap_or_default()).collect();
    Ok((routes, st, missed))
}

/// Per net: sinks the tree does not reach from the driver, with examples
/// (stderr): where an imported route and the import disagree on nodes.
fn diag_reach(g: &Graph, imp: &placed::Import, routes: &[Vec<Hop>]) {
    use std::collections::{HashMap, HashSet};
    let (mut nets_bad, mut sinks_bad, mut root_off, mut shown) = (0usize, 0usize, 0usize, 0usize);
    for (i, r) in routes.iter().enumerate() {
        if imp.sink_pins[i].iter().all(|sp| sp.internal) {
            continue;
        }
        let mut out: HashMap<u32, Vec<u32>> = HashMap::new();
        for h in r {
            out.entry(h.src).or_default().push(h.dst);
        }
        let roots: Vec<u32> = imp.nets[i].driver.nodes().to_vec();
        if !roots.iter().any(|n| out.contains_key(n)) && !r.is_empty() {
            root_off += 1;
        }
        let mut seen: HashSet<u32> = roots.iter().copied().collect();
        let mut st = roots.clone();
        while let Some(n) = st.pop() {
            for &d in out.get(&n).map(|v| v.as_slice()).unwrap_or(&[]) {
                if seen.insert(d) {
                    st.push(d);
                }
            }
        }
        let bad: Vec<&placed::SinkPin> =
            imp.sink_pins[i].iter().filter(|sp| !sp.internal && !sp.nodes.iter().any(|n| seen.contains(n))).collect();
        if bad.is_empty() {
            continue;
        }
        nets_bad += 1;
        sinks_bad += bad.len();
        if shown < 8 {
            shown += 1;
            let srcs: HashSet<u32> = r.iter().map(|h| h.src).collect();
            let first: Vec<String> = r.iter().filter(|h| !r.iter().any(|x| x.dst == h.src)).map(|h| g.node_name(h.src)).take(3).collect();
            eprintln!(
                "diag net {} driver {:?} -> {:?}; tree roots {:?}; unreached {} e.g. {}.{} at {:?}; driver in tree {}",
                imp.names[i],
                imp.driver_pins[i],
                roots.iter().map(|&n| g.node_name(n)).collect::<Vec<_>>(),
                first,
                bad.len(),
                imp.cells[bad[0].cell].name,
                bad[0].port,
                bad[0].nodes.iter().take(3).map(|&n| g.node_name(n)).collect::<Vec<_>>(),
                roots.iter().any(|n| srcs.contains(n)),
            );
        }
    }
    eprintln!("diag: {nets_bad} nets with unreached sinks, {sinks_bad} sinks, {root_off} routed nets whose driver starts no hop");
}

/// Each logical net's worst sink criticality, `name<TAB>crit` per line
/// (nets with none left out): the placer's OXIBIT_PLACE_CRIT input.
fn write_crit(path: &str, imp: &placed::Import, rep: &analyze::Report) -> Result<(), String> {
    let mut by: std::collections::BTreeMap<&str, f32> = std::collections::BTreeMap::new();
    for (i, cs) in rep.crit.iter().enumerate() {
        let c = cs.iter().copied().fold(0.0f32, f32::max);
        if c <= 0.0 {
            continue;
        }
        let name = imp.names[i].split('@').next().unwrap_or_default();
        let e = by.entry(name).or_insert(0.0);
        *e = e.max(c);
    }
    let text: String = by.iter().map(|(n, c)| format!("{n}\t{c:.4}\n")).collect();
    std::fs::write(path, text).map_err(|e| format!("{path}: {e}"))
}

fn run() -> Result<(), String> {
    let args: Vec<String> = std::env::args().collect();
    let [_, db_root, part, meta_dir, design] = &args[..] else {
        return Err("usage: route_sta DB_ROOT PART META_DIR PLACED.json".into());
    };
    let r = Reporter::new(
        vec![Box::new(Human::new(std::io::stderr(), HumanStyle::Plain))],
        &CANCEL,
    );
    let lib = sdf::Library::load_dir(&Path::new(db_root).join("timings"))?;
    let db = Database::open(Path::new(db_root), part)?;
    let g = Graph::build_with(&db, None, GraphOpts { defaults: true })?;
    let text = std::fs::read_to_string(design).map_err(|e| format!("{design}: {e}"))?;
    let placed_v: serde_json::Value =
        serde_json::from_str(&text).map_err(|e| format!("{design}: {e}"))?;
    let mut meta = Meta::open(Path::new(meta_dir));
    let imp = placed::import(&g, &placed::site_index(&db), &mut meta, &placed_v, Perm::A5)?;

    let mut so = analyze::Options::default();
    if let Some(n) = env_num("STA_PATHS") {
        so.paths = n;
    }
    so.model = match std::env::var("STA_MODEL").as_deref() {
        Ok("fast") => Model::Fast,
        Ok("full") => Model::Full,
        _ => Model::Intrinsic,
    };
    let report = |pass: u32, rep: &analyze::Report, secs: f64| {
        let mut v = rep.json();
        v["seconds"] = json!(secs);
        v["pass"] = json!(pass);
        println!("{}", json!({ "timing": v }));
        if let Ok(path) = std::env::var("STA_CRIT_OUT") {
            if let Err(e) = write_crit(&path, &imp, rep) {
                eprintln!("route_sta: {e}");
            }
        }
    };

    if std::env::var("STA_NEXTPNR").is_ok() {
        let (routes, st, missed) = nextpnr_routes(&g, &imp, &placed_v)?;
        let empty = routes.iter().filter(|r| r.is_empty()).count();
        println!("{}", json!({"check": {"nextpnr": true, "nets": routes.len(), "empty": empty,
                                         "pips": st[0], "site_pips": st[1], "same_node": st[2],
                                         "missed": st[3], "missed_examples": missed}}));
        if std::env::var("STA_NEXTPNR").as_deref() == Ok("diag") {
            diag_reach(&g, &imp, &routes);
        }
        let t = std::time::Instant::now();
        let rep = analyze::analyze(&g, &imp, &imp.nets, &routes, &lib, so);
        report(0, &rep, t.elapsed().as_secs_f64());
        if let Ok(path) = std::env::var("STA_ROUTES_OUT") {
            write_routes(&path, &routes)?;
        }
        return Ok(());
    }

    if let Ok(path) = std::env::var("STA_ROUTES_IN") {
        let routes = read_routes(&path)?;
        println!("{}", json!({"check": {"loaded": path, "nets": routes.len()}}));
        let t = std::time::Instant::now();
        let rep = analyze::analyze(&g, &imp, &imp.nets, &routes, &lib, so);
        report(0, &rep, t.elapsed().as_secs_f64());
        return Ok(());
    }

    let mut opts = Opts::default();
    if let Some(n) = env_num("OXIBIT_MAX_ITERS") {
        opts.max_iters = n;
    }
    if let Some(v) = env_num("STA_MAX_CRIT") {
        opts.max_crit = v;
    }
    if let Some(v) = env_num("STA_CRIT_EXP") {
        opts.crit_exp = v;
    }
    let td: u32 = env_num("STA_TD").unwrap_or(0);
    for pass in 0..=td {
        let sa = flow::standalone_with(&db, &g, &placed_v, &imp, "", opts.clone(), &r)?;
        println!(
            "{}",
            json!({"check": {"pass": pass, "ok": sa.ok(), "overused": sa.stats.overused,
                             "unroutable": sa.stats.unroutable_sinks, "iterations": sa.stats.iterations,
                             "seconds": sa.stats.seconds}})
        );
        let t = std::time::Instant::now();
        let rep = analyze::analyze(&g, &imp, &sa.nets, &sa.routes, &lib, so);
        report(pass, &rep, t.elapsed().as_secs_f64());
        if pass == td {
            if let Ok(path) = std::env::var("STA_ROUTES_OUT") {
                write_routes(&path, &sa.routes)?;
            }
        } else {
            opts.criticality = Some(Arc::new(rep.router_criticality(&imp)));
        }
    }
    Ok(())
}
