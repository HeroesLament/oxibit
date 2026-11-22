//! Verilog + preferences to an ECP5 bitstream, all in oxibit.
//!
//!   cargo run --release -p oxibit-ecp5 --example ecp5_flow -- \
//!       IN.v PREFS.lpf OUT_PREFIX [--db DIR] [--device LFE5U-85F] [--package CABGA381] [--top T]
//!
//! Writes OUT_PREFIX.config (Trellis text) and OUT_PREFIX.bit.
#![expect(clippy::print_stderr, reason = "a command-line example reports on stderr")]

use std::path::PathBuf;
use std::process::ExitCode;

use oxibit_ecp5::flow::{FlowOptions, run};
use oxibit_ecp5::{Chip, Options, serialise};

fn default_db() -> PathBuf {
    if let Some(p) = std::env::var_os("OXIBIT_TRELLIS_DB") {
        return PathBuf::from(p);
    }
    let home = std::env::var_os("HOME").map(PathBuf::from).unwrap_or_default();
    home.join("oss-cad-suite/share/trellis/database")
}

fn go() -> Result<(), String> {
    let mut db = default_db();
    let mut opts = FlowOptions::default();
    let mut pos: Vec<String> = Vec::new();
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        let mut val = |w: &str| args.next().ok_or(format!("{w} needs a value"));
        match a.as_str() {
            "--db" => db = PathBuf::from(val("--db")?),
            "--device" => opts.device = val("--device")?,
            "--package" => opts.package = val("--package")?,
            "--top" => opts.top = Some(val("--top")?),
            _ => pos.push(a),
        }
    }
    let [input, prefs, out] = pos.as_slice() else {
        return Err("usage: ecp5_flow IN.v PREFS.lpf OUT_PREFIX [--db DIR] [--device D] [--package P] [--top T]".into());
    };
    let src = std::fs::read_to_string(input).map_err(|e| format!("{input}: {e}"))?;
    let lpf = std::fs::read_to_string(prefs).map_err(|e| format!("{prefs}: {e}"))?;
    let t = std::time::Instant::now();
    let (cc, rep) = run(&db, &src, &lpf, &opts).map_err(|e| e.to_string())?;
    eprintln!("cells {:?}", rep.cells);
    eprintln!(
        "synth: luts {} (inverters {}), ccu2 {} ({} halves absorbed), ffs {}; check passed",
        rep.synth.luts, rep.synth.inverters, rep.synth.ccu2, rep.synth.carry_halves_absorbed, rep.synth.ffs
    );
    for w in &rep.synth.warnings {
        eprintln!("  warning: {w}");
    }
    eprintln!("pack: {:?}", rep.pack);
    eprintln!("place: hpwl {:.0}", rep.hpwl);
    for (name, sinks, global) in &rep.clocks {
        eprintln!("clock {name}: {sinks} slices, {}", if *global { "global network" } else { "general routing" });
    }
    eprintln!("route: {} nets, {} arcs, {} passes", rep.nets, rep.arcs, rep.passes);
    eprintln!("times ms {:?}", rep.ms);
    std::fs::write(format!("{out}.config"), cc.to_text()).map_err(|e| format!("{out}.config: {e}"))?;
    let ddb = oxibit_ecp5::DeviceDb::load(&db, &opts.device).map_err(|e| e.to_string())?;
    let chip = Chip::from_config(&ddb, &cc).map_err(|e| e.to_string())?;
    let bits = serialise(&chip, &Options::default()).map_err(|e| e.to_string())?;
    std::fs::write(format!("{out}.bit"), bits).map_err(|e| format!("{out}.bit: {e}"))?;
    eprintln!("wrote {out}.config and {out}.bit in {:.2}s", t.elapsed().as_secs_f64());
    Ok(())
}

fn main() -> ExitCode {
    // Deep designs recurse in elaboration and lowering; give them room.
    let r = std::thread::Builder::new()
        .stack_size(1 << 28)
        .spawn(go)
        .expect("thread")
        .join()
        .expect("join");
    match r {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("ecp5_flow: {e}");
            ExitCode::FAILURE
        }
    }
}
