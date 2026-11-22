//! Silicon smoke test: drive one pin high from a LUT.
//!
//! Finds the pad's PIO (`iodb.json`), searches uphill from its output
//! buffer input (`PADDO<pio>_PIO`) for the nearest slice LUT output
//! (`F<n>_SLICE`), never through global clock wires, and writes a Trellis
//! configuration: the LUT's INIT all ones and the slice in LOGIC mode, the
//! routed arcs, the PIO as an LVCMOS33 output in every tile that carries
//! its settings, and T tied low through its CIB constant mux. Then packs
//! it. On the
//! ULX3S the LEDs are active high, so the LED lights.
//!
//!   cargo run --release -p oxibit-ecp5 --example led_on -- \
//!       [--db DIR] [--device LFE5U-85F] [--package CABGA381] [--ball B2] out_prefix
//!
//! Writes `<out_prefix>.config` and `<out_prefix>.bit`.

#![expect(
    clippy::print_stderr,
    reason = "a command-line example reports on stderr"
)]

use std::collections::{BTreeMap, HashMap, HashSet, VecDeque};
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use oxibit_ecp5::graph::{Arc, PipKind, Wire};
use oxibit_ecp5::io::{self, PioSite};
use oxibit_ecp5::{Chip, ChipConfig, DeviceDb, Graph, Level, Options, TileConfig, serialise};

fn default_db() -> PathBuf {
    if let Some(p) = std::env::var_os("OXIBIT_TRELLIS_DB") {
        return PathBuf::from(p);
    }
    let home = std::env::var_os("HOME")
        .map(PathBuf::from)
        .unwrap_or_default();
    home.join("oss-cad-suite/share/trellis/database")
}

/// Global and clock-network wires: never used for this signal.
fn is_global(name: &str) -> bool {
    ["G_", "L_", "R_"].iter().any(|p| name.starts_with(p))
        || ["HPBX", "VPTX", "HPRX", "VPSX", "HSBX"]
            .iter()
            .any(|k| name.contains(k))
}

/// `(row, col, pio letter)` of a package ball.
fn ball_site(
    db: &Path,
    device: &str,
    package: &str,
    ball: &str,
) -> Result<(i16, i16, char), String> {
    let path = db.join("ECP5").join(device).join("iodb.json");
    let text = std::fs::read_to_string(&path).map_err(|e| format!("{}: {e}", path.display()))?;
    let v: serde_json::Value =
        serde_json::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))?;
    let site = &v["packages"][package][ball];
    let num = |k: &str| {
        site[k]
            .as_i64()
            .and_then(|n| i16::try_from(n).ok())
            .ok_or(format!("{package} {ball}: no {k}"))
    };
    let pio = site["pio"]
        .as_str()
        .and_then(|s| s.chars().next())
        .ok_or(format!("{package} {ball}: no pio"))?;
    Ok((num("row")?, num("col")?, pio))
}

/// A slice LUT output: `F<n>_SLICE`, n in 0..8.
fn lut_output(name: &str) -> Option<usize> {
    let n = name.strip_prefix('F')?.strip_suffix("_SLICE")?;
    n.parse().ok().filter(|n: &usize| *n < 8)
}

fn run() -> Result<(), String> {
    let mut db = default_db();
    let (mut device, mut package, mut ball) = (
        "LFE5U-85F".to_owned(),
        "CABGA381".to_owned(),
        "B2".to_owned(),
    );
    let mut out = None;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        let mut val = |w: &str| args.next().ok_or(format!("{w} needs a value"));
        match a.as_str() {
            "--db" => db = PathBuf::from(val("--db")?),
            "--device" => device = val("--device")?,
            "--package" => package = val("--package")?,
            "--ball" => ball = val("--ball")?,
            s => out = Some(s.to_owned()),
        }
    }
    let out =
        out.ok_or("usage: led_on [--db DIR] [--device D] [--package P] [--ball B] out_prefix")?;
    let ddb = DeviceDb::load(&db, &device).map_err(|e| e.to_string())?;
    let g = Graph::build(&ddb).map_err(|e| e.to_string())?;
    let (row, col, pio) = ball_site(&db, &device, &package, &ball)?;
    eprintln!("{ball}: PIO{pio} at R{row}C{col}");

    let target_name = format!("PADDO{pio}_PIO");
    let name = g
        .name_id_of(&target_name)
        .ok_or(format!("no wire {target_name}"))?;
    let target = Wire {
        x: col,
        y: row,
        name,
    };

    // Breadth-first uphill to the nearest LUT output.
    let mut parent: HashMap<Wire, Arc> = HashMap::new();
    let mut seen: HashSet<Wire> = HashSet::from([target]);
    let mut queue = VecDeque::from([target]);
    let mut found = None;
    while let Some(w) = queue.pop_front() {
        if lut_output(&g.names[w.name as usize]).is_some() {
            found = Some(w);
            break;
        }
        for a in g.uphill(w) {
            if is_global(&g.names[a.from.name as usize]) || !seen.insert(a.from) {
                continue;
            }
            parent.insert(a.from, a);
            queue.push_back(a.from);
        }
    }
    let src = found.ok_or("no LUT output reaches the pad")?;
    let lc = lut_output(&g.names[src.name as usize]).ok_or("not a LUT output")?;
    let slice = ['A', 'B', 'C', 'D'][lc / 2];
    let k = lc % 2;

    // Walk back from the LUT output to the pad, collecting configurable arcs.
    let mut cc = ChipConfig {
        device,
        metadata: vec![format!("oxibit led_on {ball}")],
        ..ChipConfig::default()
    };
    // `parent[w]` is the arc from `w` one step toward the pad.
    let mut path = Vec::new();
    let mut w = src;
    while w != target {
        let a = parent[&w];
        path.push(a);
        w = a.to;
    }
    let mut tiles: BTreeMap<String, TileConfig> = BTreeMap::new();
    for a in &path {
        let p = g.pip(a);
        let tile = &g.tiles[a.tile as usize].name;
        eprintln!(
            "  {:<24} -> {:<24} {:?} in {tile}",
            g.wire_name(a.from),
            g.wire_name(a.to),
            p.kind
        );
        if p.kind == PipKind::Configurable {
            tiles
                .entry(tile.clone())
                .or_default()
                .arcs
                .push((p.sink.clone(), p.source.clone()));
        }
    }

    // The slice's LUT: INIT all ones, LOGIC mode.
    let plc = g
        .tiles
        .iter()
        .find(|t| t.x == src.x && t.y == src.y && t.ty == "PLC2")
        .ok_or(format!("no PLC2 at R{}C{}", src.y, src.x))?;
    let t = tiles.entry(plc.name.clone()).or_default();
    t.words
        .push((format!("SLICE{slice}.K{k}.INIT"), vec![true; 16]));
    t.enums
        .push((format!("SLICE{slice}.MODE"), "LOGIC".to_owned()));
    eprintln!("LUT: {} SLICE{slice} K{k}", plc.name);

    // The PIO: an LVCMOS33 output in every tile that carries its settings,
    // and T tied low (an unrouted T leaves the pad tristated).
    let site = PioSite { row, col, pio };
    for (name, value) in [("BASE_TYPE", "OUTPUT_LVCMOS33"), ("DRIVE", "4")] {
        let n = io::pio_enum(&g, &ddb, site, name, value, &mut tiles).map_err(|e| e.to_string())?;
        eprintln!("PIO{pio}.{name} {value}: {n} tiles");
    }
    let t_name = format!("PADDT{pio}_PIO");
    let t_wire = Wire {
        x: col,
        y: row,
        name: g.name_id_of(&t_name).ok_or(format!("no wire {t_name}"))?,
    };
    let (tile, mux) =
        io::tie(&g, &ddb, t_wire, Level::Low, &mut tiles).map_err(|e| e.to_string())?;
    eprintln!("T low: {mux} 0 in {tile}");
    cc.tiles = tiles;

    std::fs::write(format!("{out}.config"), cc.to_text())
        .map_err(|e| format!("{out}.config: {e}"))?;
    let chip = Chip::from_config(&ddb, &cc).map_err(|e| e.to_string())?;
    let bits = serialise(&chip, &Options::default()).map_err(|e| e.to_string())?;
    std::fs::write(format!("{out}.bit"), bits).map_err(|e| format!("{out}.bit: {e}"))?;
    eprintln!(
        "wrote {out}.config and {out}.bit ({} arcs on the path)",
        path.len()
    );
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
