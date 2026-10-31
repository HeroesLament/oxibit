//! Parse, elaborate and synthesize a Verilog file to an xc7 netlist.
//!
//! cargo run --release -p oxibit-synth --example synth -- IN.v OUT.json [LIB.v ...]
//!
//! LIB.v files give primitive port directions (yosys's xilinx cells_sim.v
//! and cells_xtra.v, or stubs). stdout: one JSON line of times and counts.

use std::process::ExitCode;
use std::time::Instant;

use oxibit_hdl::{elab, parser, yosys_json};
use oxibit_synth::{Options, lib_ports, synth};

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
    let t0 = Instant::now();
    let mods = match parser::parse(&src) {
        Ok(m) => m,
        Err(e) => {
            eprintln!("parse: {e}");
            return ExitCode::from(1);
        }
    };
    let Some(top) = mods.last() else {
        eprintln!("no module");
        return ExitCode::from(1);
    };
    let m = match elab::Elab::module(top) {
        Ok(m) => m,
        Err(e) => {
            eprintln!("elaborate: {e}");
            return ExitCode::from(1);
        }
    };
    let t_front = t0.elapsed();
    let opt = Options {
        area_passes: std::env::var("OXIBIT_AREA_PASSES").ok().and_then(|s| s.parse().ok()).unwrap_or(2),
        carry_min: std::env::var("OXIBIT_CARRY_MIN").ok().and_then(|s| s.parse().ok()).unwrap_or(3),
        dsp: std::env::var("OXIBIT_DSP").map(|v| v != "0").unwrap_or(true),
    };
    // Deep cones recurse once per cell; give the walk room.
    let res = std::thread::Builder::new()
        .stack_size(1 << 30)
        .spawn(move || synth(&m, &dirs, &opt))
        .unwrap()
        .join()
        .unwrap();
    let (out, r) = match res {
        Ok(x) => x,
        Err(e) => {
            eprintln!("synth: {e}");
            return ExitCode::from(1);
        }
    };
    let t3 = Instant::now();
    let j = yosys_json::write(&out);
    if let Err(e) = std::fs::write(&args[2], serde_json::to_string(&j).unwrap()) {
        eprintln!("{}: {e}", args[2]);
        return ExitCode::from(3);
    }
    let t_write = t3.elapsed();
    let s = &r.stats;
    println!(
        "{}",
        serde_json::json!({"module": out.name, "front_ms": t_front.as_millis() as u64, "lower_ms": r.lower_ms, "map_ms": r.map_ms,
            "emit_ms": r.emit_ms, "write_ms": t_write.as_millis() as u64, "ands": r.ands, "aig_inputs": r.aig_inputs,
            "depth": r.depth, "undriven": r.undriven, "carry4": r.carry4, "dsp": r.dsp.dsp, "mul_left": r.dsp.mul_left, "mem_rd_ff": r.mem_rd_ff, "dead_ff": r.dead_ff, "dead_chain": r.dead_chain, "lut": s.luts.iter().sum::<usize>(), "lut_by_k": s.luts,
            "inverters": s.inverters, "ff": s.ffs, "named": s.named, "nodes_by_op": r.ands_by_op})
    );
    ExitCode::SUCCESS
}
