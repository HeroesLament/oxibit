//! Parse, elaborate and synthesize a Verilog file to an xc7 netlist.
//!
//! cargo run --release -p oxibit-synth --example synth -- IN.v OUT.json [LIB.v ...]
//!
//! LIB.v files give primitive port directions (yosys's xilinx cells_sim.v
//! and cells_xtra.v, or stubs). stdout: one JSON line of times and counts.
//! When memories become block RAMs, OUT.check.json is the same netlist with
//! the memories still `$mem_v2`, for the equivalence check. Progress goes
//! through oxibit-report (docs/events.md): human on stderr, the JSON Lines
//! stream to the file named by OXIBIT_EVENTS, systemd status when
//! NOTIFY_SOCKET is set.

use std::io::{BufWriter, IsTerminal};
use std::process::ExitCode;
use std::sync::atomic::AtomicBool;
use std::time::Instant;

use oxibit_hdl::{elab, parser, yosys_json};
use oxibit_report::{Event, Human, HumanStyle, JsonLines, Progress, Reporter, Sink};
use oxibit_synth::{Options, lib_ports, synth};

static CANCEL: AtomicBool = AtomicBool::new(false);

fn reporter() -> Reporter {
    let tty = std::io::stderr().is_terminal() && std::env::var_os("CI").is_none() && std::env::var("TERM").map_or(true, |t| t != "dumb");
    let style = if tty { HumanStyle::Live } else { HumanStyle::Plain };
    let mut sinks: Vec<Box<dyn Sink>> = vec![Box::new(Human::new(std::io::stderr(), style))];
    if let Some(p) = std::env::var_os("OXIBIT_EVENTS") {
        match std::fs::File::create(&p) {
            Ok(f) => sinks.push(Box::new(JsonLines(BufWriter::new(f)))),
            Err(e) => eprintln!("{}: {e}", p.to_string_lossy()),
        }
    }
    #[cfg(target_os = "linux")]
    if let Some(n) = oxibit_report::SdNotify::from_env() {
        sinks.push(Box::new(n));
    }
    Reporter::new(sinks, &CANCEL)
}

/// Close the event stream of a failed run.
fn fail(rep: &Reporter, code: u8) -> ExitCode {
    rep.emit(Event::Done {
        ok: false,
        exit: code as i32,
        cancelled: rep.cancelled(),
        phases: rep.phases(),
        peak_rss_mb: None,
        stats: serde_json::Value::Null,
        check: serde_json::Value::Null,
        outputs: vec![],
    });
    rep.finish();
    ExitCode::from(code)
}

fn main() -> ExitCode {
    let _prof = oxibit_prof::init();
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 3 {
        eprintln!("usage: synth IN.v OUT.json [LIB.v ...]");
        return ExitCode::from(3);
    }
    let mut dirs = lib_ports::PortDirs::new();
    for lib in &args[3..] {
        match std::fs::read_to_string(lib) {
            Ok(s) => dirs.extend(lib_ports::scan(&s)),
            Err(e) => {
                eprintln!("{lib}: {e}");
                return ExitCode::from(3);
            }
        }
    }
    let src = match std::fs::read_to_string(&args[1]) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("{}: {e}", args[1]);
            return ExitCode::from(3);
        }
    };
    let rep = reporter();
    rep.emit(Event::Start {
        cmd: "synth".into(),
        version: env!("CARGO_PKG_VERSION").into(),
        args: args[1..].to_vec(),
    });
    let t0 = Instant::now();
    rep.begin("front");
    let mods = match parser::parse(&src) {
        Ok(m) => m,
        Err(e) => {
            eprintln!("parse: {e}");
            return fail(&rep, 1);
        }
    };
    let Some(top) = mods.last() else {
        eprintln!("no module");
        return fail(&rep, 1);
    };
    let m = match elab::Elab::module(top) {
        Ok(m) => m,
        Err(e) => {
            eprintln!("elaborate: {e}");
            return fail(&rep, 1);
        }
    };
    rep.end("front");
    let t_front = t0.elapsed();
    let opt = Options {
        area_passes: std::env::var("OXIBIT_AREA_PASSES").ok().and_then(|s| s.parse().ok()).unwrap_or(2),
        carry_min: std::env::var("OXIBIT_CARRY_MIN").ok().and_then(|s| s.parse().ok()).unwrap_or(3),
        dsp: std::env::var("OXIBIT_DSP").map(|v| v != "0").unwrap_or(true),
        bram: std::env::var("OXIBIT_BRAM").map(|v| v != "0").unwrap_or(true),
    };
    // Deep cones recurse once per cell; give the walk room.
    let res = std::thread::scope(|sc| {
        std::thread::Builder::new()
            .stack_size(1 << 30)
            .spawn_scoped(sc, || synth(&m, &dirs, &opt, &rep))
            .unwrap()
            .join()
            .unwrap()
    });
    let (out, check, r) = match res {
        Ok(x) => x,
        Err(e) => {
            eprintln!("synth: {e}");
            return fail(&rep, 1);
        }
    };
    let t3 = Instant::now();
    rep.begin("write");
    let j = yosys_json::write(&out);
    if let Err(e) = std::fs::write(&args[2], serde_json::to_string(&j).unwrap()) {
        eprintln!("{}: {e}", args[2]);
        return fail(&rep, 3);
    }
    // The checking view, when memories were mapped: OUT.check.json.
    if r.bram.mapped > 0 {
        let path = args[2].strip_suffix(".json").unwrap_or(&args[2]).to_string() + ".check.json";
        if let Err(e) = std::fs::write(&path, serde_json::to_string(&yosys_json::write(&check)).unwrap()) {
            eprintln!("{path}: {e}");
            return fail(&rep, 3);
        }
    }
    rep.end("write");
    let t_write = t3.elapsed();
    let s = &r.stats;
    let line = serde_json::json!({"module": out.name, "front_ms": t_front.as_millis() as u64, "lower_ms": r.lower_ms, "map_ms": r.map_ms,
            "emit_ms": r.emit_ms, "write_ms": t_write.as_millis() as u64, "ands": r.ands, "aig_inputs": r.aig_inputs,
            "depth": r.depth, "undriven": r.undriven, "carry4": r.carry4, "dsp": r.dsp.dsp, "mul_left": r.dsp.mul_left, "mem_rd_ff": r.mem_rd_ff, "dead_ff": r.dead_ff, "dead_chain": r.dead_chain, "lut": s.luts.iter().sum::<usize>(), "lut_by_k": s.luts,
            "inverters": s.inverters, "ff": s.ffs, "named": s.named, "nodes_by_op": r.ands_by_op, "ramb18": r.bram.ramb18,
            "ramb36": r.bram.ramb36, "ram32m": r.bram.ram32m, "ram64m": r.bram.ram64m, "ram128x1d": r.bram.ram128x1d, "mem_mapped": r.bram.mapped, "mem_left": r.bram.left});
    println!("{line}");
    rep.emit(Event::Done {
        ok: true,
        exit: 0,
        cancelled: rep.cancelled(),
        phases: rep.phases(),
        peak_rss_mb: None,
        stats: line,
        check: serde_json::Value::Null,
        outputs: vec![args[2].clone()],
    });
    rep.finish();
    ExitCode::SUCCESS
}
