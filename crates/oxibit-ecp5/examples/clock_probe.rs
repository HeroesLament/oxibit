//! Route the ULX3S 25 MHz pad (G2) to a few slice clocks over the global
//! network and print the tree.
#![expect(clippy::print_stderr, reason = "a command-line example reports on stderr")]
use std::collections::HashSet;
use std::path::PathBuf;
use oxibit_ecp5::clock::{ClockArc, ClockNetwork, ClockOptions, route_clock};
use oxibit_ecp5::{Bels, DeviceDb, Graph, SliceMode};

fn main() {
    let db = std::env::var_os("OXIBIT_TRELLIS_DB").map(PathBuf::from).expect("OXIBIT_TRELLIS_DB");
    let ddb = DeviceDb::load(&db, "LFE5U-85F").expect("db");
    let mut g = Graph::build(&ddb).expect("graph");
    let bels = Bels::build(&mut g, &ddb, SliceMode::Whole).expect("bels");
    let pio = bels.find(0, 47, "PIOA").expect("PIOA at R47C0");
    let src = bels.pin_wire(pio, "O").expect("O");
    let mut sinks = Vec::new();
    for (x, y) in [(2, 44), (3, 44), (62, 11), (112, 90), (40, 60)] {
        let b = bels.find(x, y, "SLICEA").expect("slice");
        sinks.push(bels.pin_wire(b, "CLK").expect("CLK"));
    }
    let t = std::time::Instant::now();
    let cn = ClockNetwork::load(&db, "LFE5U-85F").expect("globals");
    let tree = route_clock(&g, &cn, src, &sinks, &HashSet::new(), ClockOptions::default()).expect("route");
    eprintln!("{:?} in {:.2}s", tree.stats, t.elapsed().as_secs_f64());
    for a in &tree.arcs {
        let kind = match a { ClockArc::Pip(p) => g.tiles[p.tile as usize].name.clone(), ClockArc::Span { .. } => "span".into(), ClockArc::RefCs { .. } => "refcs".into(), ClockArc::Sync { .. } => "eclksync".into(), ClockArc::Dcc { side, index, .. } => format!("DCC {side}{index}") };
        eprintln!("  {} -> {}  [{kind}]", g.wire_name(a.from()), g.wire_name(a.to()));
    }
}
