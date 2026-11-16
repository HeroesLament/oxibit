//! Verilog to a Nexus bitstream, all in oxibit.
//!
//!   cargo run --release -p oxibit-nexus --example flow -- \
//!       [--db DIR] [--device LIFCL-40] [--variant ""] [--package CABGA400] \
//!       design.v pins.pdc out_prefix
//!
//! Writes `<out_prefix>.fasm` and `<out_prefix>.bit`.
//!   openFPGALoader -c ft2232 <out_prefix>.bit     (SRAM)

use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Instant;

use oxibit_nexus::graph::Graph;
use oxibit_nexus::{Chip, DeviceDb, ParsedFasm, flow, serialise};

fn run() -> Result<(), String> {
    let mut db = std::env::var_os("OXIBIT_PRJOXIDE_DB")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from(std::env::var("HOME").unwrap_or_default()).join("src/prjoxide/database"));
    let (mut device, mut variant, mut package) = ("LIFCL-40".to_string(), String::new(), "CABGA400".to_string());
    let mut files = Vec::new();
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        let mut val = |w: &str| args.next().ok_or(format!("{w} needs a value"));
        match a.as_str() {
            "--db" => db = PathBuf::from(val("--db")?),
            "--device" => device = val("--device")?,
            "--variant" => variant = val("--variant")?,
            "--package" => package = val("--package")?,
            _ => files.push(a),
        }
    }
    let [src, pdc, out] = files.as_slice() else {
        return Err("usage: flow [--db DIR] [--device D] [--variant V] [--package P] design.v pins.pdc out_prefix".into());
    };
    let read = |p: &str| std::fs::read_to_string(p).map_err(|e| format!("{p}: {e}"));
    let src_text = read(src)?;
    let pins = flow::parse_pdc(&read(pdc)?).map_err(|e| e.to_string())?;

    let t0 = Instant::now();
    let ddb = DeviceDb::open(&db, &device).map_err(|e| e.to_string())?;
    let chip = Chip::new(&ddb, &variant).map_err(|e| e.to_string())?;
    let g = Graph::build(&chip).map_err(|e| e.to_string())?;
    eprintln!("device: {} nodes, {} arcs ({:.2}s)", g.nodes.len(), g.arcs.len(), t0.elapsed().as_secs_f64());

    let fasm = flow::run(&chip, &g, &src_text, &package, &pins).map_err(|e| e.to_string())?;
    let fasm_path = format!("{out}.fasm");
    std::fs::write(&fasm_path, &fasm).map_err(|e| format!("{fasm_path}: {e}"))?;
    let parsed = ParsedFasm::parse(&fasm).map_err(|e| e.to_string())?;
    let packed = Chip::from_fasm(&ddb, &parsed).map_err(|e| e.to_string())?;
    let bits = serialise(&packed).map_err(|e| e.to_string())?;
    let bit_path = format!("{out}.bit");
    std::fs::write(&bit_path, &bits).map_err(|e| format!("{bit_path}: {e}"))?;
    eprintln!("{fasm_path}, {bit_path}: {} bytes ({:.2}s total)", bits.len(), t0.elapsed().as_secs_f64());
    Ok(())
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("flow: {e}");
            ExitCode::FAILURE
        }
    }
}
