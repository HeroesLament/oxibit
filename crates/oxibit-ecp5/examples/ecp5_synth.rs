//! Synthesize a Verilog file for ECP5 and check the netlist against the
//! lowered design by random simulation.
//!
//!   cargo run --release -p oxibit-ecp5 --example ecp5_synth -- IN.v [--top T] [--cycles N]
#![expect(clippy::print_stderr, reason = "a command-line example reports on stderr")]

use std::process::ExitCode;

use oxibit_ecp5::sim::check_synthesis;
use oxibit_ecp5::synth::{SynthOptions, elaborate, synth_lowered};

fn run() -> Result<bool, String> {
    let mut args = std::env::args().skip(1);
    let (mut input, mut top, mut cycles) = (None, None, 200usize);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--top" => top = args.next(),
            "--cycles" => cycles = args.next().and_then(|s| s.parse().ok()).ok_or("--cycles N")?,
            _ => input = Some(a),
        }
    }
    let input = input.ok_or("usage: ecp5_synth IN.v [--top T] [--cycles N]")?;
    let src = std::fs::read_to_string(&input).map_err(|e| format!("{input}: {e}"))?;
    let t = std::time::Instant::now();
    let m = elaborate(&src, top.as_deref()).map_err(|e| e.to_string())?;
    let (nl, rep, lw) = synth_lowered(&m, SynthOptions::default()).map_err(|e| e.to_string())?;
    eprintln!("{}: {:?} in {:.2}s", nl.top, nl.counts(), t.elapsed().as_secs_f64());
    eprintln!("  luts {} (inverters {}), ccu2 {} ({} halves absorbed), ffs {}, depth {}", rep.luts, rep.inverters, rep.ccu2, rep.carry_halves_absorbed, rep.ffs, rep.depth);
    for w in &rep.warnings {
        eprintln!("  warning: {w}");
    }
    match check_synthesis(&lw, &nl, cycles, 0x0123_4567_89AB_CDEF).map_err(|e| e.to_string())? {
        None => {
            eprintln!("  simulation check: {cycles} cycles x 64 lanes, outputs equal");
            Ok(true)
        }
        Some((c, port)) => {
            eprintln!("  simulation check: MISMATCH at cycle {c} on {port}");
            Ok(false)
        }
    }
}

fn main() -> ExitCode {
    match run() {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::FAILURE,
        Err(e) => {
            eprintln!("ecp5_synth: {e}");
            ExitCode::from(2)
        }
    }
}
