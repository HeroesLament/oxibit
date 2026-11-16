//! Silicon smoke test: drive one pin low from a LUT with INIT 0.
//!
//! Places a LUT in the PLC nearest the pad (by routing distance), routes
//! its output to the PIO's output buffer, configures the PIO as an
//! LVCMOS33 output, and writes FASM and a bitstream. On the CrossLink-NX
//! EVN the LEDs are active low, so the LED lights.
//!
//!   cargo run --release -p oxibit-nexus --example led_on -- \
//!       [--db DIR] [--ball E17] [--package CABGA400] [--variant ""] out_prefix
//!
//! The EVN's part reads IDCODE 0x110F1043, the production LIFCL-40
//! (variant ""); `openFPGALoader --detect` prints it with the version
//! nibble masked, which looks like the ES part's 0x010F1043.
//!
//! Writes `<out_prefix>.fasm` and `<out_prefix>.bit`.

use std::collections::{BTreeMap, HashMap, HashSet, VecDeque};
use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Instant;

use oxibit_nexus::emit::Emit;
use oxibit_nexus::graph::{Arc, Graph, NodeId};
use oxibit_nexus::{Chip, DeviceDb, ParsedFasm, serialise};

/// Global clock network nodes: never used for ordinary signals.
fn is_global(g: &Graph, n: NodeId) -> bool {
    let name = g.name(g.nodes[n as usize].name);
    ["HPBX", "VPSX", "HPRX", "G:"].iter().any(|p| name.starts_with(p)) || n == g.vcc
}

fn run() -> Result<(), String> {
    let mut db = std::env::var_os("OXIBIT_PRJOXIDE_DB")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from(std::env::var("HOME").unwrap_or_default()).join("src/prjoxide/database"));
    let (mut ball, mut package, mut variant) = ("E17".to_string(), "CABGA400".to_string(), String::new());
    let mut out = None;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        let mut val = |w: &str| args.next().ok_or(format!("{w} needs a value"));
        match a.as_str() {
            "--db" => db = PathBuf::from(val("--db")?),
            "--ball" => ball = val("--ball")?,
            "--package" => package = val("--package")?,
            "--variant" => variant = val("--variant")?,
            s => out = Some(s.to_string()),
        }
    }
    let out = out.ok_or("usage: led_on [--db DIR] [--ball B] [--package P] [--variant V] out_prefix")?;

    let t0 = Instant::now();
    let ddb = DeviceDb::open(&db, "LIFCL-40").map_err(|e| e.to_string())?;
    let chip = Chip::new(&ddb, &variant).map_err(|e| e.to_string())?;
    let t1 = Instant::now();
    let g = Graph::build(&chip).map_err(|e| e.to_string())?;
    let t2 = Instant::now();
    eprintln!(
        "db+chip {:.2}s, graph {:.2}s: {} nodes, {} arcs, {} bels",
        (t1 - t0).as_secs_f64(),
        (t2 - t1).as_secs_f64(),
        g.nodes.len(),
        g.arcs.len(),
        g.bels.len()
    );

    let pad = ddb.iodb.pad(&package, &ball).ok_or(format!("no pad {ball} in {package}"))?;
    let (w, h) = (ddb.data.max_col as i32, ddb.data.max_row as i32);
    let (x, y) = match pad.side.as_str() {
        "L" => (0, pad.offset),
        "R" => (w, pad.offset),
        "T" => (pad.offset, 0),
        "B" => (pad.offset, h),
        s => return Err(format!("pad {ball} on side {s:?}")),
    };
    let pio = format!("R{y}C{x}_PIO{}", ['A', 'B'][pad.pio as usize & 1]);
    let io = g.bel(&pio).ok_or(format!("no bel {pio}"))?;
    let target = io.pin("I").ok_or("PIO has no I pin")?;
    eprintln!("{ball}: bank {} {pio}, I = {}", pad.bank, g.node_name(target));

    // LUT F outputs, by node.
    let mut lut_f: HashMap<NodeId, usize> = HashMap::new();
    for (i, b) in g.bels.iter().enumerate() {
        if b.bel.beltype == "OXIDE_COMB" && b.bel.name.ends_with("_LUT0") {
            if let Some(f) = b.pin("F") {
                lut_f.insert(f, i);
            }
        }
    }
    // Backward BFS from the PIO input to the nearest LUT output.
    let mut toward: HashMap<NodeId, Arc> = HashMap::new();
    let mut q = VecDeque::from([target]);
    let mut seen = HashSet::from([target]);
    let mut found = None;
    while let Some(n) = q.pop_front() {
        if lut_f.contains_key(&n) {
            found = Some(n);
            break;
        }
        for a in g.uphill(n) {
            if is_global(&g, a.from) || !seen.insert(a.from) {
                continue;
            }
            toward.insert(a.from, *a);
            q.push_back(a.from);
        }
        if seen.len() > 2_000_000 {
            break;
        }
    }
    let src = found.ok_or("no LUT output reaches the pad")?;
    let lut = &g.bels[lut_f[&src]];
    let mut path = Vec::new();
    let mut n = src;
    while n != target {
        let a = toward[&n];
        path.push(a);
        n = a.to;
    }
    eprintln!("LUT {} -> {pio}: {} arcs", lut.group(), path.len());
    for a in &path {
        eprintln!("  {} -> {} ({:?})", g.node_name(a.from), g.node_name(a.to), a.kind);
    }

    let mut e = Emit::new(&chip, &g);
    e.comment("net led");
    e.route(&path);
    e.out.push('\n');
    e.comment(format!("cell lut0 at {}", lut.group()));
    let plc = chip.tiles[lut.tile as usize].name.replace(':', "__");
    let slice = &lut.bel.name[..6];
    e.line(format!("{plc}.{slice}.K0.INIT[15:0] = 16'b0000000000000000"));
    e.float_pins(lut, &["A", "B", "C", "D"]);
    e.out.push('\n');
    e.comment(format!("cell led at {pio}"));
    let pre = format!("{pio}.{}", io.bel.name);
    e.line(format!("{pre}.BASE_TYPE.OUTPUT_LVCMOS33"));
    e.line(format!("{pre}.PULLMODE.NONE"));
    e.line(format!("{pre}.GLITCHFILTER.OFF"));
    e.line(format!("{pre}.SLEWRATE.MED"));
    e.line(format!("{pre}.TMUX.INV"));
    e.float_pins(io, &["T", "I3CRESEN", "I3CWKPU"]);
    e.used_io.insert(pio.clone());
    e.out.push('\n');
    e.bank_cfg(&BTreeMap::from([(pad.bank, 3300)]));
    e.unused(&HashSet::from([lut.group(), pio.clone()]));

    let fasm_path = format!("{out}.fasm");
    std::fs::write(&fasm_path, &e.out).map_err(|x| format!("{fasm_path}: {x}"))?;
    let parsed = ParsedFasm::parse(&e.out).map_err(|x| x.to_string())?;
    let packed = Chip::from_fasm(&ddb, &parsed).map_err(|x| x.to_string())?;
    let bits = serialise(&packed).map_err(|x| x.to_string())?;
    let bit_path = format!("{out}.bit");
    std::fs::write(&bit_path, &bits).map_err(|x| format!("{bit_path}: {x}"))?;
    eprintln!("{fasm_path}, {bit_path}: {} bytes", bits.len());
    Ok(())
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("led_on: {e}");
            ExitCode::FAILURE
        }
    }
}
