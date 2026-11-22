//! Silicon test: a ripple counter from the 25 MHz pad clock to eight LEDs.
//!
//! Stage i is one slice: LUT K0 inverts the slice's REG0 output (`F = !A`,
//! A routed from Q), REG0 takes D from the LUT (`REG0.SD 1`), CE forced on
//! (`CEMUX 1`), and its clock is the previous stage's Q (stage 0: the clock
//! pad). Two stages per PLC (slices A and B), so the tile's two clock muxes
//! carry two different stage clocks; slice C's K0 (INIT 0) drives both
//! slices' LSR low. The last eight stages drive the LEDs, MSB on LED0, so
//! the LEDs count down in binary, LED0 toggling every 2^(N-1) clock periods
//! (N = 25: 0.67 s at 25 MHz).
//!
//! Everything is placed by hand on `oxibit_ecp5::bels` (SLICE and PIO bel
//! pins) and routed by `oxibit_ecp5::route`.
//!
//!   cargo run --release -p oxibit-ecp5 --example blinky -- [--db DIR] [--stages N] out_prefix
//!
//! ULX3S v3 defaults: LFE5U-85F, CABGA381, clock G2, LEDs B2 C2 C1 D2 D1 E2 E1 H3.
//! Writes `<out_prefix>.config` and `<out_prefix>.bit`.

#![expect(
    clippy::print_stderr,
    reason = "a command-line example reports on stderr"
)]

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::time::Instant;

use oxibit_ecp5::graph::{PipKind, TileLoc, Wire};
use oxibit_ecp5::io::{self, PioSite};
use oxibit_ecp5::route::{self, Net, NetKind};
use oxibit_ecp5::{
    Bels, Chip, ChipConfig, DeviceDb, Graph, Level, Options, SliceMode, TileConfig, serialise,
};

const DEVICE: &str = "LFE5U-85F";
const PACKAGE: &str = "CABGA381";
const CLOCK_BALL: &str = "G2";
const LED_BALLS: [&str; 8] = ["B2", "C2", "C1", "D2", "D1", "E2", "E1", "H3"];

fn default_db() -> PathBuf {
    if let Some(p) = std::env::var_os("OXIBIT_TRELLIS_DB") {
        return PathBuf::from(p);
    }
    let home = std::env::var_os("HOME")
        .map(PathBuf::from)
        .unwrap_or_default();
    home.join("oss-cad-suite/share/trellis/database")
}

/// `(row, col, pio letter)` of a package ball.
fn ball_site(db: &Path, ball: &str) -> Result<(i16, i16, char), String> {
    let path = db.join("ECP5").join(DEVICE).join("iodb.json");
    let text = std::fs::read_to_string(&path).map_err(|e| format!("{}: {e}", path.display()))?;
    let v: serde_json::Value =
        serde_json::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))?;
    let site = &v["packages"][PACKAGE][ball];
    let num = |k: &str| {
        site[k]
            .as_i64()
            .and_then(|n| i16::try_from(n).ok())
            .ok_or(format!("{ball}: no {k}"))
    };
    let pio = site["pio"]
        .as_str()
        .and_then(|s| s.chars().next())
        .ok_or(format!("{ball}: no pio"))?;
    Ok((num("row")?, num("col")?, pio))
}

/// The wire of pin `pin` of bel `bel` at (x, y).
fn pin(bels: &Bels, x: i16, y: i16, bel: &str, pin: &str) -> Result<Wire, String> {
    let b = bels
        .find(x, y, bel)
        .ok_or(format!("no bel {bel} at R{y}C{x}"))?;
    bels.pin_wire(b, pin)
        .ok_or(format!("bel {bel} at R{y}C{x} has no pin {pin}"))
}

/// LUT4 INIT as Trellis word bits (bit i = output for inputs DCBA = i).
fn lut(f: impl Fn(usize) -> bool) -> Vec<bool> {
    (0..16).map(f).collect()
}

#[expect(clippy::too_many_lines, reason = "one straight-line silicon test")]
fn run() -> Result<(), String> {
    let mut db = default_db();
    let mut stages = 25usize;
    let mut out = None;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        let mut val = |w: &str| args.next().ok_or(format!("{w} needs a value"));
        match a.as_str() {
            "--db" => db = PathBuf::from(val("--db")?),
            "--stages" => {
                stages = val("--stages")?
                    .parse()
                    .map_err(|e| format!("--stages: {e}"))?;
            }
            s => out = Some(s.to_owned()),
        }
    }
    let out = out.ok_or("usage: blinky [--db DIR] [--stages N] out_prefix")?;
    if stages < LED_BALLS.len() {
        return Err(format!("need at least {} stages", LED_BALLS.len()));
    }
    let t0 = Instant::now();
    let ddb = DeviceDb::load(&db, DEVICE).map_err(|e| e.to_string())?;
    let mut g = Graph::build(&ddb).map_err(|e| e.to_string())?;
    let bels = Bels::build(&mut g, &ddb, SliceMode::Whole).map_err(|e| e.to_string())?;
    let g = g;
    let mut tiles: BTreeMap<String, TileConfig> = BTreeMap::new();

    // Pads.
    let (crow, ccol, cpio) = ball_site(&db, CLOCK_BALL)?;
    let clk_pad = pin(&bels, ccol, crow, &format!("PIO{cpio}"), "O")?;
    let csite = PioSite {
        row: crow,
        col: ccol,
        pio: cpio,
    };
    io::pio_enum(&g, &ddb, csite, "BASE_TYPE", "INPUT_LVCMOS33", &mut tiles)
        .map_err(|e| e.to_string())?;
    let mut leds = Vec::new();
    for ball in LED_BALLS {
        let (row, col, pio) = ball_site(&db, ball)?;
        leds.push(pin(&bels, col, row, &format!("PIO{pio}"), "I")?);
        // Every tile carrying the PIO's settings, and T tied low: an
        // unrouted T leaves the pad tristated.
        let site = PioSite { row, col, pio };
        for (name, value) in [("BASE_TYPE", "OUTPUT_LVCMOS33"), ("DRIVE", "4")] {
            io::pio_enum(&g, &ddb, site, name, value, &mut tiles).map_err(|e| e.to_string())?;
        }
        let t = pin(&bels, col, row, &format!("PIO{pio}"), "T")?;
        io::tie(&g, &ddb, t, Level::Low, &mut tiles).map_err(|e| e.to_string())?;
    }

    // Placement: PLC2 tiles nearest the clock pad, two stages each.
    let mut plcs: Vec<&TileLoc> = g.tiles.iter().filter(|t| t.ty == "PLC2").collect();
    plcs.sort_by_key(|t| ((t.x - ccol).abs() + (t.y - crow).abs(), t.y, t.x));
    let n_tiles = stages.div_ceil(2);
    let used = &plcs[..n_tiles];

    let mut nets = Vec::new();
    let mut q = Vec::new();
    for (i, t) in (0..stages).map(|i| (i, used[i / 2])) {
        let z = i % 2;
        let slice = ['A', 'B'][z];
        let tc = tiles.entry(t.name.clone()).or_default();
        tc.words
            .push((format!("SLICE{slice}.K0.INIT"), lut(|i| i & 1 == 0)));
        tc.enums
            .push((format!("SLICE{slice}.MODE"), "LOGIC".to_owned()));
        tc.enums
            .push((format!("SLICE{slice}.REG0.SD"), "1".to_owned()));
        tc.enums
            .push((format!("SLICE{slice}.CEMUX"), "1".to_owned()));
        let sl = format!("SLICE{slice}");
        let qw = pin(&bels, t.x, t.y, &sl, "Q0")?;
        nets.push(Net {
            name: format!("fb{i}"),
            source: qw,
            sinks: vec![pin(&bels, t.x, t.y, &sl, "A0")?],
            kind: NetKind::Signal,
        });
        q.push(qw);
    }
    // Clocks: pad -> stage 0, stage i -> stage i + 1. A stage's Q net gets
    // the next clock as an extra sink, so merge into the feedback nets.
    for i in 0..stages {
        let t = used[i / 2];
        let clk = pin(&bels, t.x, t.y, ["SLICEA", "SLICEB"][i % 2], "CLK")?;
        if i == 0 {
            nets.push(Net {
                name: "clk".into(),
                source: clk_pad,
                sinks: vec![clk],
                kind: NetKind::Signal,
            });
        } else {
            nets[i - 1].sinks.push(clk);
        }
    }
    // LEDs: the top eight stages, MSB on LED0.
    for (k, led) in leds.iter().enumerate() {
        nets[stages - 1 - k].sinks.push(*led);
    }
    // LSR low: slice C K0 = 0 in every used tile, to both slices' LSR.
    for (j, t) in used.iter().enumerate() {
        let tc = tiles.entry(t.name.clone()).or_default();
        tc.words.push(("SLICEC.K0.INIT".to_owned(), lut(|_| false)));
        tc.enums
            .push(("SLICEC.MODE".to_owned(), "LOGIC".to_owned()));
        let mut sinks = vec![pin(&bels, t.x, t.y, "SLICEA", "LSR")?];
        if 2 * j + 1 < stages {
            sinks.push(pin(&bels, t.x, t.y, "SLICEB", "LSR")?);
        }
        nets.push(Net {
            name: format!("zero{j}"),
            source: pin(&bels, t.x, t.y, "SLICEC", "F0")?,
            sinks,
            kind: NetKind::Signal,
        });
    }
    let t_setup = t0.elapsed();

    let routes = route::route(&g, &nets, route::Options::default()).map_err(|e| e.to_string())?;
    route::check(&g, &nets, &routes).map_err(|e| e.to_string())?;
    let n_arcs: usize = routes.nets.iter().map(Vec::len).sum();
    eprintln!(
        "{} nets, {n_arcs} arcs, {} passes; setup {:.2?}, route {:.2?}",
        nets.len(),
        routes.passes,
        t_setup,
        t0.elapsed().saturating_sub(t_setup)
    );
    for arcs in &routes.nets {
        for a in arcs {
            let p = g.pip(a);
            if p.kind == PipKind::Configurable {
                let tile = &g.tiles[a.tile as usize].name;
                tiles
                    .entry(tile.clone())
                    .or_default()
                    .arcs
                    .push((p.sink.clone(), p.source.clone()));
            }
        }
    }

    let cc = ChipConfig {
        device: DEVICE.to_owned(),
        metadata: vec![format!(
            "oxibit blinky, {stages}-stage ripple counter, {CLOCK_BALL} -> LEDs"
        )],
        tiles,
        ..ChipConfig::default()
    };
    std::fs::write(format!("{out}.config"), cc.to_text())
        .map_err(|e| format!("{out}.config: {e}"))?;
    let chip = Chip::from_config(&ddb, &cc).map_err(|e| e.to_string())?;
    let bits = serialise(&chip, &Options::default()).map_err(|e| e.to_string())?;
    std::fs::write(format!("{out}.bit"), bits).map_err(|e| format!("{out}.bit: {e}"))?;
    eprintln!(
        "wrote {out}.config and {out}.bit ({} PLC tiles)",
        used.len()
    );
    Ok(())
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("blinky: {e}");
            ExitCode::FAILURE
        }
    }
}
