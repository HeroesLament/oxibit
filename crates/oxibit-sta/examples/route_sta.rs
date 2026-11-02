//! Route a placed design as `standalone` does (default router options,
//! OXIBIT_MAX_ITERS honoured) and run setup analysis on the result.
//!
//! cargo run --release -p oxibit-sta --example route_sta -- DB_ROOT PART META_DIR PLACED.json
//!
//! The SDF library is DB_ROOT/timings. stdout: the route's check line, then
//! one `{"timing": ...}` line (`analyze::Report::json`). STA_PATHS=n: worst
//! paths in full (default 10). STA_MODEL=intrinsic|fast|full: interconnect
//! model (default intrinsic).

use std::path::Path;
use std::process::ExitCode;
use std::sync::atomic::AtomicBool;

use oxibit_fasm::db::Database;
use oxibit_fasm::route::{Graph, GraphOpts};
use oxibit_pnr::delay::Model;
use oxibit_pnr::flow;
use oxibit_pnr::placed::{self, Meta, Perm};
use oxibit_pnr::router::Opts;
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
    let mut opts = Opts::default();
    if let Some(n) = std::env::var("OXIBIT_MAX_ITERS")
        .ok()
        .and_then(|n| n.parse().ok())
    {
        opts.max_iters = n;
    }
    let sa = flow::standalone_with(&db, &g, &placed_v, &imp, "", opts, &r)?;
    println!(
        "{}",
        json!({"check": {"ok": sa.ok(), "overused": sa.stats.overused, "unroutable": sa.stats.unroutable_sinks}})
    );
    let mut so = analyze::Options::default();
    if let Some(n) = std::env::var("STA_PATHS").ok().and_then(|n| n.parse().ok()) {
        so.paths = n;
    }
    so.model = match std::env::var("STA_MODEL").as_deref() {
        Ok("fast") => Model::Fast,
        Ok("full") => Model::Full,
        _ => Model::Intrinsic,
    };
    let t = std::time::Instant::now();
    let rep = analyze::analyze(&g, &imp, &sa.nets, &sa.routes, &lib, so);
    let mut v = rep.json();
    v["seconds"] = json!(t.elapsed().as_secs_f64());
    println!("{}", json!({"timing": v}));
    Ok(())
}
