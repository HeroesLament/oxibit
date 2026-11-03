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
    };

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
