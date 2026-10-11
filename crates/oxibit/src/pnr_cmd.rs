//! `oxibit pnr route`: route a placed design again with oxibit's router.
//! `oxibit pnr dsp-patch`: re-target a routed design to new DSP48E1
//! settings without placing or routing.
//!
//! stdout carries the result (`--json`: one report object), stderr the
//! human progress, `--events` the JSON Lines event stream (docs/events.md).
//!
//! Exit status: 0 routed and checked clean; 1 ran but the routing is not
//! clean (overuse, unroutable sinks, sinks not reached, new shorts);
//! 2 usage; 3 environment (database, input or output file); 130 cancelled.

use std::path::Path;
use std::process::ExitCode;

use oxibit_fasm::db::Database;
use oxibit_fasm::route::{Graph, GraphOpts};
use oxibit_pnr::{dsp_patch, reroute, router::Opts};
use oxibit_report::{Event, PhaseState, Progress, Reporter};

use crate::report::{ReportArgs, serve_signals};

pub struct RouteArgs<'a> {
    pub input: &'a Path,
    pub output: &'a Path,
    pub db_root: &'a Path,
    pub part: &'a str,
    pub max_iters: u32,
    pub report: &'a ReportArgs,
    pub json: bool,
}

const EXIT_UNCLEAN: u8 = 1;
const EXIT_REFUSED: u8 = 1;
const EXIT_ENV: u8 = 3;
const EXIT_CANCELLED: u8 = 130;

pub fn route(a: RouteArgs) -> ExitCode {
    let r = match a.report.reporter(true) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("oxibit pnr route: {e}");
            return ExitCode::from(EXIT_ENV);
        }
    };
    serve_signals(r.clone());
    let r: &Reporter = &r;
    r.emit(Event::Start { cmd: "pnr route".into(), version: env!("CARGO_PKG_VERSION").into(), args: std::env::args().skip(1).collect() });

    // Environment: database, graph, input. Failures here are exit 3.
    let env = (|| -> Result<(Graph, String), String> {
        r.begin("db");
        let db = Database::open(a.db_root, a.part)?;
        r.end("db");
        r.begin("graph");
        let g = Graph::build_with(&db, None, GraphOpts { defaults: true })?;
        r.emit(Event::progress("graph", serde_json::json!({"nodes": g.nodes(), "edges": g.edges()})));
        r.end("graph");
        let fasm = std::fs::read_to_string(a.input).map_err(|e| format!("{}: {e}", a.input.display()))?;
        Ok((g, fasm))
    })();
    let (g, fasm) = match env {
        Ok(v) => v,
        Err(e) => return finish(r, false, EXIT_ENV, false, serde_json::json!({"error": e}), serde_json::json!({}), vec![], a.json),
    };

    let res = reroute::run(&g, &fasm, Opts { max_iters: a.max_iters, ..Opts::default() }, r);

    r.emit(Event::Phase { phase: "write".into(), state: PhaseState::Begin, ms: None });
    if let Err(e) = std::fs::write(a.output, &res.fasm) {
        r.emit(Event::Warn { code: "write".into(), msg: format!("{}: {e}", a.output.display()) });
        return finish(r, false, EXIT_ENV, false, res.stats_json(), res.check_json(), vec![], a.json);
    }
    r.emit(Event::Phase { phase: "write".into(), state: PhaseState::End, ms: None });

    let (ok, exit) = if res.stats.cancelled {
        (false, EXIT_CANCELLED)
    } else if res.ok() {
        (true, 0)
    } else {
        (false, EXIT_UNCLEAN)
    };
    finish(r, ok, exit, res.stats.cancelled, res.stats_json(), res.check_json(), vec![a.output.display().to_string()], a.json)
}

#[allow(clippy::too_many_arguments)]
fn finish(r: &Reporter, ok: bool, exit: u8, cancelled: bool, stats: serde_json::Value, check: serde_json::Value, outputs: Vec<String>, json: bool) -> ExitCode {
    let done = Event::Done { ok, exit: exit as i32, cancelled, phases: r.phases(), peak_rss_mb: peak_rss_mb(), stats, check, outputs };
    if json {
        let env = oxibit_report::Envelope { v: oxibit_report::SCHEMA, t_ms: r.elapsed_ms(), event: &done };
        println!("{}", serde_json::to_string(&env).unwrap_or_default());
    }
    r.emit(done);
    r.finish();
    ExitCode::from(exit)
}

/// Peak resident set of this process, in MB.
fn peak_rss_mb() -> Option<u64> {
    let mut ru: libc::rusage = unsafe { std::mem::zeroed() };
    if unsafe { libc::getrusage(libc::RUSAGE_SELF, &mut ru) } != 0 {
        return None;
    }
    // ru_maxrss: bytes on macOS, kilobytes on Linux.
    let raw = ru.ru_maxrss as u64;
    let bytes = if cfg!(target_os = "macos") { raw } else { raw * 1024 };
    Some(bytes / (1024 * 1024))
}

pub struct DspPatchArgs<'a> {
    pub template: &'a Path,
    pub template_netlist: &'a Path,
    pub packed: &'a Path,
    pub structure: Option<&'a [std::path::PathBuf]>,
    pub output: &'a Path,
    pub db_root: &'a Path,
    pub part: &'a str,
    pub report: &'a ReportArgs,
    pub json: bool,
}

fn read_json(p: &Path) -> Result<serde_json::Value, String> {
    let text = std::fs::read_to_string(p).map_err(|e| format!("{}: {e}", p.display()))?;
    serde_json::from_str(&text).map_err(|e| format!("{}: {e}", p.display()))
}

/// Exit status: 0 written; 1 refused (the template does not regenerate from
/// its own netlist, the designs differ in more than DSP settings, or the
/// cells do not match); 3 environment.
pub fn dsp_patch(a: DspPatchArgs) -> ExitCode {
    let r = match a.report.reporter(true) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("oxibit pnr dsp-patch: {e}");
            return ExitCode::from(EXIT_ENV);
        }
    };
    let r: &Reporter = &r;
    r.emit(Event::Start { cmd: "pnr dsp-patch".into(), version: env!("CARGO_PKG_VERSION").into(), args: std::env::args().skip(1).collect() });
    let refuse = |stats: serde_json::Value, check: serde_json::Value| finish(r, false, EXIT_REFUSED, false, stats, check, vec![], a.json);
    let env_err = |e: String| finish(r, false, EXIT_ENV, false, serde_json::json!({"error": e}), serde_json::json!({}), vec![], a.json);

    r.begin("db");
    let db = match Database::open(a.db_root, a.part) {
        Ok(db) => db,
        Err(e) => return env_err(e),
    };
    let sites = dsp_patch::dsp_sites(&db);
    r.end("db");

    r.begin("read");
    let inputs = (|| -> Result<_, String> {
        let fasm = std::fs::read_to_string(a.template).map_err(|e| format!("{}: {e}", a.template.display()))?;
        let tcells = dsp_patch::dsp_cells(&read_json(a.template_netlist)?)?;
        let ncells = dsp_patch::dsp_cells(&read_json(a.packed)?)?;
        let synth = match a.structure {
            Some([t, n]) => Some((read_json(t)?, read_json(n)?)),
            _ => None,
        };
        Ok((fasm, tcells, ncells, synth))
    })();
    let (fasm, tcells, ncells, synth) = match inputs {
        Ok(v) => v,
        Err(e) => return env_err(e),
    };
    r.emit(Event::progress("read", serde_json::json!({"template_dsps": tcells.len(), "new_dsps": ncells.len()})));
    r.end("read");

    let mut stats = serde_json::json!({"template_dsps": tcells.len(), "new_dsps": ncells.len()});

    r.begin("structure");
    let structure = match &synth {
        Some((t, n)) => match dsp_patch::same_structure(t, n) {
            Ok(()) => "same".to_string(),
            Err(e) => {
                r.emit(Event::Warn { code: "structure".into(), msg: e.clone() });
                stats["structure"] = e.into();
                return refuse(stats, serde_json::json!({}));
            }
        },
        None => {
            r.emit(Event::Warn { code: "structure_unchecked".into(), msg: "no --structure: synthesized netlists not compared".into() });
            "unchecked".to_string()
        }
    };
    stats["structure"] = structure.into();
    let tnames: std::collections::BTreeSet<_> = tcells.keys().collect();
    let nnames: std::collections::BTreeSet<_> = ncells.keys().collect();
    if tnames != nnames {
        let only: Vec<_> = tnames.symmetric_difference(&nnames).take(5).map(|s| s.to_string()).collect();
        r.emit(Event::Warn { code: "cells".into(), msg: format!("DSP cells differ, e.g. {only:?}") });
        return refuse(stats, serde_json::json!({}));
    }
    r.end("structure");

    r.begin("self_check");
    let place = match dsp_patch::placement(&tcells, &sites) {
        Ok(p) => p,
        Err(e) => return env_err(e),
    };
    let bad = match dsp_patch::self_check(&fasm, &tcells, &place) {
        Ok(b) => b,
        Err(e) => return env_err(e),
    };
    let check = serde_json::json!({
        "tiles": place.values().map(|(t, _, _)| t).collect::<std::collections::BTreeSet<_>>().len(),
        "mismatched_tiles": bad.len(),
        "first": bad.iter().take(3).map(|m| serde_json::json!({"tile": m.tile, "only_regenerated": m.only_regenerated, "only_template": m.only_template})).collect::<Vec<_>>(),
    });
    if !bad.is_empty() {
        r.emit(Event::Warn { code: "self_check".into(), msg: format!("{} template DSP tiles do not regenerate from the template netlist", bad.len()) });
        return refuse(stats, check);
    }
    r.end("self_check");

    r.begin("patch");
    let out = dsp_patch::patch(&fasm, &ncells, &place);
    r.end("patch");
    r.begin("write");
    if let Err(e) = std::fs::write(a.output, out) {
        r.emit(Event::Warn { code: "write".into(), msg: format!("{}: {e}", a.output.display()) });
        return finish(r, false, EXIT_ENV, false, stats, check, vec![], a.json);
    }
    r.end("write");
    finish(r, true, 0, false, stats, check, vec![a.output.display().to_string()], a.json)
}
