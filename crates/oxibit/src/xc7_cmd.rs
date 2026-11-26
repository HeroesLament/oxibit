//! `oxibit build` for 7-series parts: synthesis (or a synthesized netlist),
//! PS7 pin extraction, pack, place, route, FASM and the `.bit`, in one
//! process on one reporter.

use std::collections::{BTreeMap, HashSet, VecDeque};
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use oxibit_fasm::db::Database;
use oxibit_fasm::route::{Graph, GraphOpts};
use oxibit_place::pack::ctx::ScoreParams;
use oxibit_place::pack::slices::{PackerKind, SliceOpts, pack_slices};
use oxibit_place::{netlist::Netlist, pack, place, placer};
use oxibit_pnr::delay::Model;
use oxibit_pnr::placed::{self, Meta, Perm};
use oxibit_pnr::router::Opts;
use oxibit_pnr::sta::TimingSource;
use oxibit_pnr::{flow, sta};
use oxibit_report::{Event, Progress, Reporter};
use serde_json::{Value, json};

use crate::input::Result;
use crate::report::ReportArgs;

pub struct BuildArgs {
    pub input: PathBuf,
    pub part: String,
    pub constraints: Option<PathBuf>,
    pub top: Option<String>,
    pub out: Option<PathBuf>,
    /// Also write `<stem>.placed.json` and `<stem>.fasm`.
    pub keep: bool,
    pub db_root: Option<PathBuf>,
    pub meta: Option<PathBuf>,
    /// Verilog files giving primitive port directions, for synthesis.
    pub libs: Vec<PathBuf>,
    pub bins: Option<usize>,
    /// Clock periods, `name=ps,...` (`*` for the rest): route for timing.
    pub timing: Option<String>,
}

fn dir(flag: Option<PathBuf>, env: &str, what: &str) -> Result<PathBuf> {
    flag.or_else(|| std::env::var_os(env).map(PathBuf::from)).ok_or_else(|| format!("no {what}: give it or set ${env}"))
}

fn read_json(path: &Path) -> Result<Value> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    serde_json::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))
}

/// Verilog to the synthesized netlist, as the `synth` example does.
fn synthesize(src: &str, top: Option<&str>, libs: &[PathBuf], r: &Reporter) -> Result<Value> {
    use oxibit_hdl::{elab, parser, yosys_json};
    use oxibit_synth::caps::DeviceCaps;
    use oxibit_synth::policy::Policy;
    use oxibit_synth::{lib_ports, synth};

    let mut dirs = lib_ports::PrimPorts::new();
    for lib in libs {
        let s = std::fs::read_to_string(lib).map_err(|e| format!("{}: {e}", lib.display()))?;
        dirs.extend(lib_ports::scan(&s));
    }
    r.begin("front");
    let mods = parser::parse(src).map_err(|e| format!("parse: {e}"))?;
    let m = match top {
        Some(t) => mods.iter().find(|m| m.name == t).ok_or_else(|| format!("no module {t}"))?,
        None => mods.last().ok_or("no module")?,
    };
    let m = elab::Elab::module(m).map_err(|e| format!("elaborate: {e}"))?;
    r.end("front");
    let policy = Policy::exploration();
    let caps = DeviceCaps::xc7z020();
    // Deep cones recurse once per cell; give the walk room.
    let res = std::thread::scope(|sc| {
        std::thread::Builder::new()
            .stack_size(1 << 30)
            .spawn_scoped(sc, || synth(&m, &dirs, &caps, &policy, r))
            .map_err(|e| e.to_string())?
            .join()
            .map_err(|_| "synthesis panicked".to_owned())?
    });
    let out = res.map_err(|e| format!("synth: {e}"))?;
    let mut j = yosys_json::write(&out.netlist);
    lib_ports::annotate(&mut j, &dirs);
    Ok(j)
}

/// Where a PS wire meets the interconnect: the first INT pip within
/// `DEPTH` hops, backwards (`back`) or forwards.
fn ps_walk(g: &Graph, start: u32, back: bool) -> Option<[i16; 2]> {
    const DEPTH: u32 = 12;
    let mut seen: HashSet<u32> = HashSet::from([start]);
    let mut queue: VecDeque<(u32, u32)> = VecDeque::from([(start, 0)]);
    while let Some((n, depth)) = queue.pop_front() {
        let mut next: Vec<u32> = Vec::new();
        let mut found = None;
        let mut step = |m: u32, tile: u32, pi: u32| {
            if found.is_none() && g.edge_pip(tile, pi).tile.starts_with("INT") {
                found = Some(g.node_xy(m));
            }
            next.push(m);
        };
        if back {
            for (m, tile, pi) in g.in_edges(n) {
                step(m, tile, pi);
            }
        } else {
            for &(m, tile, pi) in g.out_edges(n) {
                step(m, tile, pi);
            }
        }
        if let Some((x, y)) = found {
            return Some([x, y]);
        }
        if depth < DEPTH {
            for m in next {
                if seen.insert(m) {
                    queue.push_back((m, depth + 1));
                }
            }
        }
    }
    None
}

/// The PS7's pins where they meet the interconnect, for the placer.
fn ps_pins(db: &Database) -> Result<placer::PinMap> {
    let g = Graph::build_with(db, None, GraphOpts::default())?;
    let mut map: BTreeMap<String, Value> = BTreeMap::new();
    for n in 0..g.nodes() as u32 {
        let name = g.node_name(n).to_string();
        if !name.starts_with("PSS") {
            continue;
        }
        let (i, o) = (ps_walk(&g, n, true), ps_walk(&g, n, false));
        if i.is_some() || o.is_some() {
            map.insert(name, json!({"in": i, "out": o}));
        }
    }
    Ok(placer::PinMap::from_ps_entries(&json!(map)))
}

fn done(r: &Reporter, ok: bool, stats: Value, outputs: Vec<String>) {
    r.emit(Event::Done { ok, exit: if ok { 0 } else { 2 }, cancelled: false, phases: r.phases(), peak_rss_mb: None, stats, check: json!({}), outputs });
    r.finish();
}

pub fn build(a: BuildArgs, rep: &ReportArgs) -> Result<ExitCode> {
    let r = rep.reporter(true)?;
    r.emit(Event::Start { cmd: "build".into(), version: env!("CARGO_PKG_VERSION").into(), args: std::env::args().skip(1).collect() });
    match run(&a, &r, rep.text()) {
        Ok((stats, outputs)) => {
            done(&r, true, stats, outputs);
            Ok(ExitCode::SUCCESS)
        }
        Err(e) => {
            done(&r, false, json!({}), vec![]);
            Err(e)
        }
    }
}

fn run(a: &BuildArgs, r: &Reporter, text: bool) -> Result<(Value, Vec<String>)> {
    let stem = a.input.file_stem().and_then(|s| s.to_str()).ok_or("input has no file name")?.to_owned();
    let out = a.out.clone().unwrap_or_else(|| PathBuf::from(format!("{stem}.bit")));
    let db_root = dir(a.db_root.clone(), "OXIBIT_XRAY_DB", "prjxray family directory (--db-root)")?;
    let meta_dir = dir(a.meta.clone(), "OXIBIT_XC7_META", "nextpnr-xilinx-meta directory (--meta)")?;
    let mut outputs: Vec<String> = Vec::new();

    // 1. The netlist: synthesized here from Verilog, or read as given.
    let synth_v = if a.input.extension().is_some_and(|e| e == "json") {
        read_json(&a.input)?
    } else {
        let src = std::fs::read_to_string(&a.input).map_err(|e| format!("{}: {e}", a.input.display()))?;
        synthesize(&src, a.top.as_deref(), &a.libs, r)?
    };

    // 2. Device and database.
    r.begin("db");
    let dev = oxibit_place::device::Device::open(&db_root, &a.part)?;
    let db = Database::open(&db_root, &a.part)?;
    r.end("db");

    // 3. PS7 pins at their interconnect (Zynq): without them every PS pin
    //    sits at the PS site's tile, tens of tiles from where its nets go.
    let mut pins = placer::PinMap::default();
    if a.part.starts_with("xc7z") {
        r.begin("pins");
        pins = ps_pins(&db)?;
        r.end("pins");
    }

    // 4. Pack: cells into clusters, then LUTs and flops into slices
    //    (best-choice, compacted to the budget), as `pack --gp`.
    r.begin("pack");
    let props = match &a.constraints {
        Some(p) => oxibit_place::xdc::parse(&std::fs::read_to_string(p).map_err(|e| format!("{}: {e}", p.display()))?),
        None => oxibit_place::xdc::parse(""),
    };
    let nl = Netlist::from_json(&synth_v)?;
    let mut p = pack::pack(nl, &dev, &props)?;
    let params = ScoreParams::default();
    let room = placer::Problem::build(&p, &dev, 200, &pins).ok().map(|prob| prob.slice_room());
    let base = pack_slices(&mut p, &SliceOpts { params, ..SliceOpts::default() });
    let dense = oxibit_place::pack::slices::dense_report(&p, params);
    let budget = oxibit_place::pack::slices::budget(base.stats.slices, dense.stats.slices, room);
    let scored = pack_slices(&mut p, &SliceOpts { kind: PackerKind::RulesScore, params, max_slices: Some(budget), ..SliceOpts::default() });
    let mut ok = base.illegal.is_empty() && dense.illegal.is_empty() && scored.illegal.is_empty();
    let slices = scored.stats.slices;
    r.end("pack");

    // 5. Place and legalise.
    r.begin("place");
    let mut gp = placer::GpOpts::default();
    if let Some(bins) = a.bins {
        gp.nesterov.bins = bins;
    }
    let prep = placer::place_design(&mut p, &dev, &gp, 200, &pins)?;
    let lg = place::legalise(&mut p, &place::ps7_inputs(&meta_dir)?);
    ok &= prep.bels.ok() && lg.over_five.is_empty();
    let mut placed_v = p.nl.to_design().to_json();
    let (ports, names) = p.nl.ports_and_names();
    placed_v["modules"][&p.nl.top]["ports"] = ports;
    placed_v["modules"][&p.nl.top]["netnames"] = names;
    r.end("place");
    if a.keep {
        let path = format!("{stem}.placed.json");
        std::fs::write(&path, serde_json::to_string(&placed_v).map_err(|e| e.to_string())?).map_err(|e| format!("{path}: {e}"))?;
        outputs.push(path);
    }
    if !ok {
        return Err("placement is not legal (unplaced cells, collisions or overfull LUT sites)".into());
    }

    // 6. Routing graph and the placed design on it.
    r.begin("graph");
    let g = Graph::build_with(&db, None, GraphOpts { defaults: true })?;
    let mut meta = Meta::open(&meta_dir);
    let sites = placed::site_index(&db);
    let imp = placed::import(&g, &sites, &mut meta, &placed_v, Perm::A5)?;
    let cons = match &a.timing {
        Some(s) => Some(sta::Constraints::parse(s)?),
        None => None,
    };
    let design = sta::Design::build(&g, &sites, &placed_v, &imp)?;
    let timer = std::rc::Rc::new(std::cell::RefCell::new(sta::Timer::new(design, &cons.clone().unwrap_or_default(), Model::Intrinsic)));
    let mut opts = Opts::default();
    if cons.is_some() {
        opts.timing = Some(sta::TimingView::new(timer.clone()));
    }
    r.end("graph");

    // 7. Route.
    r.begin("route");
    let sa = flow::standalone_with(&db, &g, &placed_v, &imp, "", opts, r)?;
    r.end("route");

    // 8. Timing of the routed design, measured whether or not it steered.
    r.begin("timing");
    let summary = {
        let mut t = timer.borrow_mut();
        let all: Vec<usize> = (0..sa.nets.len()).collect();
        t.update(&g, &sa.nets, &sa.routes, &all).summary()
    };
    r.end("timing");

    // 9. FASM and the bitstream.
    r.begin("write");
    let fasm = sa.fasm.text();
    if a.keep {
        let path = format!("{stem}.fasm");
        std::fs::write(&path, &fasm).map_err(|e| format!("{path}: {e}"))?;
        outputs.push(path);
    }
    let (routed, resolved) = (sa.ok(), imp.unresolved.counts.is_empty());
    if !routed {
        return Err(format!("routing did not finish clean: {} nodes still overused", sa.stats.overused));
    }
    if !resolved {
        return Err(format!("unresolved pins in the placed design: {:?}", imp.unresolved.counts));
    }
    let bit = crate::build::to_bit(&db, &fasm, &stem)?;
    std::fs::write(&out, &bit).map_err(|e| format!("{}: {e}", out.display()))?;
    outputs.insert(0, out.display().to_string());
    r.end("write");

    let stats = json!({"cells": p.nl.cell_ids().len(), "slices": slices, "nets": sa.nets.len(), "pins_known": pins.len(), "ps7_ties": lg.ps7_ties, "timing": summary});
    if text {
        eprintln!("{stem}: {} cells, {slices} slices, {} nets, timing {}", p.nl.cell_ids().len(), sa.nets.len(), stats["timing"]);
    }
    Ok((stats, outputs))
}

/// The LibreSDR board's pin constraints (AD9363 control, RX and TX buses).
pub const LIBRESDR_XDC: &str = include_str!("../../oxibit-zynq7/boards/libresdr.xdc");
