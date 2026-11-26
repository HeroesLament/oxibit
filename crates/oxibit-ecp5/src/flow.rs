//! Verilog to ECP5 bitstream: synthesis, packing, placement, routing and
//! the Trellis configuration.
//!
//! 1. `synth`: Verilog to ECP5 cells, checked against the lowered design
//!    by random simulation (`sim::check_synthesis`); a mismatch fails the
//!    run.
//! 2. `pack`: cells into PLC2 tiles.
//! 3. Placement: each packed tile, and each carry chain as a run of tiles
//!    east, onto PLC2 sites by oxibit-place's placer (quadratic start,
//!    Nesterov, spreading legalisation). Its x axis is the tile row and its
//!    y axis the column, so a chain's run (`site_up`: the PLC2 tile east)
//!    is the placer's "column". Pads are fixed pins at their PIO tiles.
//! 4. Clocks (nets on flop clocks) on the global network (`clock`), the
//!    rest by PathFinder (`route`) off the clock trees.
//! 5. The configuration: slice modes, LUT INITs, flop settings, the tile
//!    clock / set-reset inversions on whichever `CLK<k>` / `LSR<k>` the
//!    routes used, PIO settings with T tied low on outputs, routed arcs and
//!    the clock trees' DCCs.

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use crate::source::Source;

use oxibit_place::placer::{GpOpts, Inst, InstKind, PNet, Pin, Problem, Site, place};

use crate::bels::{BelId, Bels, SliceMode};
use crate::chip::Level;
use crate::clock::{ClockNetwork, ClockOptions, ClockTree, route_clock, tree_config};
use crate::config::{ChipConfig, TileConfig};
use crate::db::DeviceDb;
use crate::graph::{Graph, PipKind, Wire};
use crate::io::{PioSite, tie};
use crate::iol;
use crate::clk;
use crate::lpf::Lpf;
use crate::ebr::{self, Use};
use crate::netlist::{Cell, NetId, Netlist, Polarity, PrimPin, RegSet, SrMode, clock_nets};
use crate::pll;
use crate::sys;
use crate::pack::{Loc, PackStats, Packed, SliceKind, pack};
use crate::route::{self, Net, NetKind, Routes};
use crate::synth::{SynthOptions, SynthReport, elaborate, synth_lowered};
use crate::{Result, err};

#[derive(Clone, Debug)]
pub struct FlowOptions {
    pub device: String,
    pub package: String,
    pub top: Option<String>,
    pub synth: SynthOptions,
    /// Random-simulation cycles for the synthesis check (0: skip).
    pub check_cycles: usize,
    pub route: route::Options,
    pub clock: ClockOptions,
    /// Nets with more pins than this carry no wirelength in placement.
    pub max_fanout: usize,
}

impl Default for FlowOptions {
    fn default() -> Self {
        FlowOptions {
            device: "LFE5U-85F".to_owned(),
            package: "CABGA381".to_owned(),
            top: None,
            synth: SynthOptions::default(),
            check_cycles: 200,
            route: route::Options::default(),
            clock: ClockOptions::default(),
            max_fanout: 64,
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct FlowReport {
    pub cells: BTreeMap<&'static str, usize>,
    pub synth: SynthReport,
    pub pack: PackStats,
    pub hpwl: f64,
    pub clocks: Vec<(String, usize, bool)>,
    pub nets: usize,
    pub arcs: usize,
    pub passes: usize,
    pub ms: BTreeMap<&'static str, u128>,
}

/// A port bit's pad: the PIO site and its bel.
#[derive(Clone, Copy, Debug)]
struct Pad {
    site: PioSite,
    bel: BelId,
}

/// `(row, col, pio)` of a package ball (`iodb.json`).
pub fn ball_site(root: &Source, device: &str, package: &str, ball: &str) -> Result<PioSite> {
    let v = root.iodb(device)?;
    let s = &v["packages"][package][ball];
    let num = |k: &str| {
        s[k].as_i64()
            .and_then(|n| i16::try_from(n).ok())
            .ok_or_else(|| err!("{package} ball {ball}: no {k}"))
    };
    let pio = s["pio"]
        .as_str()
        .and_then(|p| p.chars().next())
        .ok_or_else(|| err!("{package} ball {ball}: no pio"))?;
    Ok(PioSite {
        row: num("row")?,
        col: num("col")?,
        pio,
    })
}

const ABCD: [char; 4] = ['A', 'B', 'C', 'D'];

/// Per net: its source pin and sink pins, as (bel, pin name).
#[derive(Clone, Debug, Default)]
struct NetPins {
    source: Option<(BelId, String)>,
    sinks: Vec<(BelId, String)>,
    /// Sinks that are flop clocks (slice CLK pins).
    clock_sinks: Vec<(BelId, String)>,
}

fn slice_bel(bels: &Bels, x: i16, y: i16, z: usize) -> Result<BelId> {
    let name = format!("SLICE{}", ABCD[z]);
    bels.find(x, y, &name)
        .ok_or_else(|| err!("no {name} at R{y}C{x}"))
}

fn pin_wire(bels: &Bels, bel: BelId, pin: &str) -> Result<Wire> {
    bels.pin_wire(bel, pin).ok_or_else(|| {
        err!(
            "bel {} has no pin {pin}",
            bels.template(bel).name
        )
    })
}

/// Run the flow on Verilog `src` with preferences `lpf`.
pub fn run(
    root: &Source,
    src: &str,
    lpf_text: &str,
    opts: &FlowOptions,
) -> Result<(ChipConfig, FlowReport)> {
    run_with(root, src, lpf_text, opts, &oxibit_report::Silent)
}

/// `run`, reporting its phases (synth, pack, device, place, route, config)
/// and the router's passes to `progress`.
pub fn run_with(
    root: &Source,
    src: &str,
    lpf_text: &str,
    opts: &FlowOptions,
    progress: &dyn oxibit_report::Progress,
) -> Result<(ChipConfig, FlowReport)> {
    use oxibit_report::{Event, PhaseState};
    let phase = |name: &str, state: PhaseState| {
        progress.emit(Event::Phase { phase: name.into(), state, ms: None });
    };
    let mut report = FlowReport::default();
    let t = std::time::Instant::now();
    phase("synth", PhaseState::Begin);
    // A lap closes its phase and opens the one that follows it.
    let mut lap = |name: &'static str, report: &mut FlowReport, t: &mut std::time::Instant| {
        report.ms.insert(name, t.elapsed().as_millis());
        *t = std::time::Instant::now();
        phase(name, PhaseState::End);
        let next = match name {
            "synth" => Some("pack"),
            "pack" => Some("device"),
            "device" => Some("place"),
            "place" => Some("route"),
            "route" => Some("config"),
            _ => None,
        };
        if let Some(n) = next {
            phase(n, PhaseState::Begin);
        }
    };
    let mut t = t;

    // 1. Synthesis.
    let mut m = elaborate(src, opts.top.as_deref())?;
    // Open drain pads in logic (the OPENDRAIN bits alone leave a BIDIR
    // pad driving high).
    let od: BTreeSet<String> = Lpf::parse(lpf_text)?
        .ports
        .iter()
        .filter(|(_, p)| p.iobuf.get("OPENDRAIN").is_some_and(|v| v.eq_ignore_ascii_case("ON")))
        .map(|(k, _)| k.clone())
        .collect();
    crate::pads::canonicalize(&mut m)?;
    crate::pads::open_drain(&mut m, &od)?;
    let (nl, srep, lw) = synth_lowered(&m, opts.synth)?;
    if opts.check_cycles > 0 {
        if let Some((c, port)) = crate::sim::check_synthesis(&lw, &nl, opts.check_cycles, 0x5EED)? {
            return Err(err!(
                "synthesis check: netlist and design differ at cycle {c} on {port}"
            ));
        }
    }
    report.cells = nl.counts();
    report.synth = srep;
    lap("synth", &mut report, &mut t);

    // 2. Pack.
    let (packed, pstats) = pack(&nl)?;
    report.pack = pstats;
    lap("pack", &mut report, &mut t);

    // Device.
    let db = DeviceDb::load_from(root, &opts.device)?;
    let mut g = Graph::build(&db)?;
    let bels = Bels::build(&mut g, &db, SliceMode::Whole)?;
    let g = g;
    let cnet = ClockNetwork::load(root, &opts.device)?;
    let lpf = Lpf::parse(lpf_text)?;
    lap("device", &mut report, &mut t);

    // Pads.
    let mut pads: Vec<Pad> = Vec::new();
    for p in &nl.ports {
        let site = lpf
            .port(&p.name)
            .and_then(|pp| pp.site.as_ref())
            .ok_or_else(|| err!("{}", lpf.no_locate(&p.name)))?;
        let s = ball_site(root, &opts.device, &opts.package, site)?;
        let bel = bels
            .find(s.col, s.row, &format!("PIO{}", s.pio))
            .ok_or_else(|| err!("no PIO{} at R{}C{} for {}", s.pio, s.row, s.col, p.name))?;
        pads.push(Pad { site: s, bel });
    }

    // Block RAM bels, nearest the pads' centroid.
    #[expect(clippy::cast_precision_loss, reason = "tile coordinates")]
    let near = if pads.is_empty() {
        (f64::from(g.height) / 2.0, f64::from(g.width) / 2.0)
    } else {
        let n = pads.len() as f64;
        (
            pads.iter().map(|p| f64::from(p.site.row)).sum::<f64>() / n,
            pads.iter().map(|p| f64::from(p.site.col)).sum::<f64>() / n,
        )
    };
    let ebr_at = ebr::assign(&bels, &nl, near)?;
    // PLLs, nearest the pad of their reference clock.
    let drivers = nl.drivers();
    let pll_at = pll::assign(&bels, &nl, |i| {
        let Cell::Prim(p) = &nl.cells[i] else { return near };
        let Some(PrimPin::In(Some(n))) = p.pin("CLKI") else { return near };
        match drivers[n as usize].map(|d| &nl.cells[d as usize]) {
            Some(Cell::Ib { port, .. }) => (f64::from(pads[*port].site.row), f64::from(pads[*port].site.col)),
            _ => near,
        }
    })?;
    // Hard blocks on fixed bels: block RAMs and PLLs.
    let mut hard_at = ebr_at.clone();
    let sys_at = sys::assign(&bels, &nl)?;
    hard_at.extend(pll_at.iter().map(|(&k, &v)| (k, v)));
    hard_at.extend(sys_at.iter().map(|(&k, &v)| (k, v)));
    let dsp_at = crate::dsp::assign(&bels, &nl)?;
    hard_at.extend(dsp_at.iter().map(|(&k, &v)| (k, v)));
    // DQSBUFMs beside their DQS pads, DDRDLLAs in their users' corner.
    let dqs_groups = crate::dqs::load_groups(root, &opts.device)?;
    let dqs_at = crate::dqs::assign(
        &bels,
        &nl,
        &dqs_groups,
        |n| match drivers[n as usize].map(|d| &nl.cells[d as usize]) {
            Some(Cell::Ib { port, .. }) => Some(pads[*port].site),
            Some(Cell::Prim(p)) if p.pad.is_some() && p.pins.iter().any(|(pin, c)| pin == "O" && *c == PrimPin::Out(Some(n))) => {
                p.pad.map(|k| pads[k].site)
            }
            _ => None,
        },
        g.height,
    )?;
    hard_at.extend(dqs_at.iter().map(|(&k, &v)| (k, v)));
    // IO logic beside its pads.
    let sites: Vec<PioSite> = pads.iter().map(|p| p.site).collect();
    let iol_plan = iol::plan(&bels, &nl, &sites)?;
    hard_at.extend(iol_plan.bels());
    // Kept clock primitives (gated DCCA, DCSC, ECLKSYNCB, ECLKBRIDGECS,
    // CLKDIVF): a bel whose clock input the driver reaches over the clock
    // network (where the driver is on a bel) and, for an edge clock, whose
    // output reaches the IOLOGIC ECLK pins it drives.
    let clk_at = {
        let hard = &hard_at;
        // The IOLOGIC ECLK pin wires on each net.
        let mut eclk_sinks: HashMap<NetId, Vec<Wire>> = HashMap::new();
        for (&c, (k, map)) in &iol_plan.cells {
            let Cell::Prim(p) = &nl.cells[c] else { continue };
            for (pin, bp) in map {
                if *bp == Some("ECLK") {
                    if let (Some(PrimPin::In(Some(n))), Ok(w)) = (p.pin(pin), pin_wire(&bels, iol_plan.iols[*k].bel, "ECLK")) {
                        eclk_sinks.entry(n).or_default().push(w);
                    }
                }
            }
        }
        // A DQSBUFM's ECLK is on the bank edge clock too.
        for (&c, &b) in &dqs_at {
            let Cell::Prim(p) = &nl.cells[c] else { continue };
            if let (Some(PrimPin::In(Some(n))), Ok(w)) = (p.pin("ECLK"), pin_wire(&bels, b, "ECLK")) {
                eclk_sinks.entry(n).or_default().push(w);
            }
        }
        for v in eclk_sinks.values_mut() {
            v.sort();
        }
        clk::assign(&bels, &nl, |p, b, so_far| {
            let pin = clk::clock_in(&p.ty);
            let reach = |s: Wire, sinks: &[Wire]| {
                route_clock(&g, &cnet, s, sinks, &HashSet::new(), opts.clock).is_ok()
            };
            // Output side: the ECLK pins this edge clock drives.
            if matches!(p.ty.as_str(), "ECLKSYNCB" | "ECLKBRIDGECS") {
                let out = clk::clock_out(&p.ty);
                if let (Some(PrimPin::Out(Some(n))), Ok(o)) = (p.pin(out), pin_wire(&bels, b, out)) {
                    if let Some(s) = eclk_sinks.get(&n) {
                        if !reach(o, s) {
                            return false;
                        }
                    }
                }
            }
            let Some(PrimPin::In(Some(n))) = p.pin(pin) else { return true };
            let Ok(sink) = pin_wire(&bels, b, pin) else { return false };
            let src = match drivers[n as usize].map(|d| (d as usize, &nl.cells[d as usize])) {
                Some((_, Cell::Ib { port, .. })) => pin_wire(&bels, pads[*port].bel, "O").ok(),
                Some((d, Cell::Prim(q))) => hard.get(&d).or_else(|| so_far.get(&d)).and_then(|&qb| {
                    let out = q.pins.iter().find(|(_, c)| *c == PrimPin::Out(Some(n)))?;
                    pin_wire(&bels, qb, &out.0).ok()
                }),
                _ => None,
            };
            src.is_none_or(|s| reach(s, &[sink]))
        })?
    };
    hard_at.extend(clk_at.iter().map(|(&k, &v)| (k, v)));
    let clk_pins = clk::pin_wires(&bels, &clk_at);

    // 3. Placement.
    let (tile_xy, hpwl) = place_tiles(&g, &bels, &nl, &packed, &pads, &hard_at, opts)?;
    report.hpwl = hpwl;
    lap("place", &mut report, &mut t);

    // 4. Routing.
    let pins = net_pins(&bels, &nl, &packed, &tile_xy, &pads, &hard_at, &iol_plan)?;
    let clock_nets: BTreeSet<NetId> = clock_nets(&nl);
    let entries = cib_entries(&g, &bels);
    if std::env::var_os("OXIBIT_DEBUG_CIB").is_some() {
        eprintln!("cib entries: {}", entries.len());
        for e in entries.iter().take(12) {
            eprintln!("  {} at ({},{})", g.wire_name(*e), e.x, e.y);
        }
    }
    let mut trees: Vec<(NetId, ClockTree)> = Vec::new();
    let mut taken: HashSet<Wire> = HashSet::new();
    let mut signal: Vec<(NetId, Net)> = Vec::new();
    let mut src_groups = HashMap::new();
    for (&n, np) in &pins {
        let Some((sbel, spin)) = &np.source else {
            if np.sinks.is_empty() && np.clock_sinks.is_empty() {
                continue;
            }
            return Err(err!("net {} has sinks but no driver", net_name(&nl, n)));
        };
        let source = pin_wire(&bels, *sbel, spin)?;
        // A DQSBUFM's group signal: on the DQSG wires, no route. Every
        // sink must be an IOLOGIC of the same DQS group.
        if bels.template(*sbel).ty == "DQSBUFM" && crate::dqs::GROUP_SIGNALS.contains(&spin.as_str()) {
            let own = crate::dqs::bel_group(&bels, *sbel, &dqs_groups);
            for (b, p) in np.sinks.iter().chain(&np.clock_sinks) {
                let t = bels.template(*b);
                let theirs = crate::dqs::bel_group(&bels, *b, &dqs_groups);
                if !t.name.starts_with("IOLOGIC") || p != spin || own.is_none() || theirs != own {
                    return Err(err!(
                        "net {}: DQSBUFM {spin} (group {}) drives {} {p} (group {}): it reaches only the {spin} of its own group's IOLOGICs",
                        net_name(&nl, n),
                        own.unwrap_or("?"),
                        t.name,
                        theirs.unwrap_or("none")
                    ));
                }
            }
            continue;
        }
        let mut data_sinks: Vec<Wire> = np
            .sinks
            .iter()
            .map(|(b, p)| pin_wire(&bels, *b, p))
            .collect::<Result<_>>()?;
        let clk_sinks: Vec<Wire> = np
            .clock_sinks
            .iter()
            .map(|(b, p)| pin_wire(&bels, *b, p))
            .collect::<Result<BTreeSet<_>>>()?
            .into_iter()
            .collect();
        // Data sinks on origin wires (a gated DCC's CE, a DCS's selects)
        // route to the CIB wire fixed to them.
        for w in &mut data_sinks {
            *w = local_sink(&g, *w);
        }
        if std::env::var_os("OXIBIT_DEBUG_CLOCK").is_some() {
            for (b, p) in &np.clock_sinks {
                let w = pin_wire(&bels, *b, p)?;
                if !g.wire_name(w).contains("_SLICE") {
                    eprintln!("clock sink {}: {} {p} -> {}", net_name(&nl, n), bels.template(*b).name, g.wire_name(w));
                }
            }
        }
        if clock_nets.contains(&n) && !clk_sinks.is_empty() {
            // Other nets keep off the kept clock primitives' pins.
            let own: HashSet<Wire> = clk_sinks.iter().copied().chain([source]).collect();
            let mut taken_n = taken.clone();
            taken_n.extend(clk_pins.iter().filter(|w| !own.contains(w)));
            // A tree with no global wire is general routing near the
            // pads (a fabric signal on an IOLOGIC clock): not a clock tree.
            let tree = route_clock(&g, &cnet, source, &clk_sinks, &taken_n, opts.clock).and_then(|t| {
                if t.wires().any(|w| route::is_global(&g, w)) {
                    Ok(t)
                } else {
                    Err(err!("no global wire"))
                }
            });
            match tree {
                Ok(tree) => {
                    // The source stays open for the net's own data route.
                    taken.extend(tree.wires().filter(|&w| w != source));
                    report
                        .clocks
                        .push((net_name(&nl, n), clk_sinks.len(), true));
                    trees.push((n, tree));
                }
                Err(e) => {
                    if std::env::var_os("OXIBIT_DEBUG_CLOCK").is_some() {
                        eprintln!("clock {}: {e}", net_name(&nl, n));
                    }
                    // Not a dedicated clock source: general routing to a
                    // CIB entry of the global network (a `*PCLKCIB*`
                    // input of a DCC), the tree from there.
                    match via_cib(&g, &cnet, &entries, source, &clk_sinks, &taken_n, opts.clock) {
                        Some((entry, tree)) => {
                            // The entry and the wire it is fixed to stay open for the
                            // general route that reaches the entry.
                            taken.extend(tree.wires().filter(|&w| {
                                w != entry && !g.wire_name(w).contains("PCLKCIB")
                            }));
                            report
                                .clocks
                                .push((net_name(&nl, n), clk_sinks.len(), true));
                            trees.push((n, tree));
                            data_sinks.push(entry);
                        }
                        None => {
                            report
                                .clocks
                                .push((net_name(&nl, n), clk_sinks.len(), false));
                            data_sinks.extend(clk_sinks);
                        }
                    }
                }
            }
        } else {
            data_sinks.extend(clk_sinks);
        }
        data_sinks.sort();
        data_sinks.dedup();
        // A sink on the source's own electrical node needs no route: the
        // DSP's bypassed registers (MULT A -> ROA -> ALU A) and its products
        // to the ALU (P -> MA) are fixed wires.
        let own = route::fixed_group(&g, source, &mut src_groups);
        data_sinks.retain(|w| !own.contains(w));
        if !data_sinks.is_empty() {
            signal.push((
                n,
                Net {
                    name: net_name(&nl, n),
                    source,
                    sinks: data_sinks,
                    kind: NetKind::Signal,
                },
            ));
        }
    }
    if std::env::var_os("OXIBIT_DEBUG_CLOCK").is_some() {
        for (n, t) in &trees {
            let general = t.wires().filter(|&w| !route::is_global(&g, w)).count();
            eprintln!("tree {}: {} arcs, {general} general wire ends", net_name(&nl, *n), t.arcs.len());
        }
        for (n, net) in &signal {
            if clock_nets.contains(n) {
                eprintln!("signal clock net {}: {} sinks", net_name(&nl, *n), net.sinks.len());
            }
        }
    }
    let nets: Vec<Net> = signal.iter().map(|(_, n)| n.clone()).collect();
    let routes = route::route_progress(&g, &nets, opts.route, &taken, progress)?;
    route::check(&g, &nets, &routes)?;
    // Clock trees against each other and against the signal routes: no
    // electrical node may carry two nets.
    {
        let mut groups = HashMap::new();
        let mut owner: HashMap<Wire, (NetId, Wire)> = HashMap::new();
        let mut claim = |w: Wire, n: NetId, what: &str| -> Result<()> {
            for m in route::fixed_group(&g, w, &mut groups) {
                match owner.get(&m) {
                    Some(&(o, ow)) if o != n => {
                        return Err(err!(
                            "{what}: nets {} and {} share {} (via {} and {})",
                            net_name(&nl, o),
                            net_name(&nl, n),
                            g.wire_name(m),
                            g.wire_name(ow),
                            g.wire_name(w)
                        ));
                    }
                    _ => {
                        owner.insert(m, (n, w));
                    }
                }
            }
            Ok(())
        };
        for (n, tree) in &trees {
            for w in tree.wires() {
                claim(w, *n, "clock tree")?;
            }
        }
        for ((n, net), arcs) in signal.iter().zip(&routes.nets) {
            claim(net.source, *n, "route")?;
            for a in arcs {
                claim(a.to, *n, "route")?;
            }
        }
    }
    report.nets = nets.len();
    report.arcs = routes.nets.iter().map(Vec::len).sum();
    report.passes = routes.passes;
    lap("route", &mut report, &mut t);

    // 5. Configuration.
    let mut cc = configure(
        &g, &db, root, &bels, &nl, &packed, &tile_xy, &pads, &lpf, &signal, &routes, &trees, opts,
    )?;
    ebr::configure(&g, &db, &bels, &nl, &ebr_at, &mut cc)?;
    pll::configure(&g, &db, &bels, &nl, &pll_at, &mut cc.tiles)?;
    sys::configure(&g, &db, &nl, &sys_at, &mut cc.tiles)?;
    iol::configure(&g, &db, &bels, &iol_plan, &mut cc.tiles)?;
    clk::configure(&g, &db, &bels, &nl, &clk_at, &mut cc.tiles)?;
    crate::dsp::configure(&g, &db, &bels, &nl, &dsp_at, &mut cc.tiles)?;
    crate::dqs::configure(&g, &db, &bels, &nl, &dqs_at, &mut cc.tiles)?;
    sys::sysconfig(&g, &db, &lpf.sysconfig, &mut cc)?;
    for tc in cc.tiles.values_mut() {
        tc.enums.sort();
        tc.enums.dedup();
    }
    lap("config", &mut report, &mut t);
    Ok((cc, report))
}

fn net_name(nl: &Netlist, n: NetId) -> String {
    nl.net_names
        .get(&n)
        .cloned()
        .unwrap_or_else(|| format!("$n{n}"))
}

/// Place the packed tiles; returns each packed tile's (col, row) and the
/// placed half-perimeter wirelength.
fn place_tiles(
    g: &Graph,
    bels: &Bels,
    nl: &Netlist,
    packed: &Packed,
    pads: &[Pad],
    ebr_at: &HashMap<usize, BelId>,
    opts: &FlowOptions,
) -> Result<(Vec<(i16, i16)>, f64)> {
    // Sites: every PLC2 tile, placer x = row, y = column.
    let plc: Vec<(i16, i16)> = g
        .tiles
        .iter()
        .filter(|t| t.ty == "PLC2")
        .map(|t| (t.x, t.y))
        .collect();
    let index: HashMap<(i16, i16), u32> = plc
        .iter()
        .enumerate()
        .map(|(i, &xy)| (xy, u32::try_from(i).expect("fits")))
        .collect();
    let sites: Vec<Site> = plc
        .iter()
        .map(|&(x, y)| Site {
            x: f64::from(y),
            y: f64::from(x),
            name: format!("R{y}C{x}"),
        })
        .collect();
    // Instances: chains first (one macro each), then the other tiles.
    let mut inst_of_tile: Vec<usize> = vec![usize::MAX; packed.tiles.len()];
    let mut insts: Vec<Inst> = Vec::new();
    let mut chain_rows: Vec<usize> = Vec::new();
    for ch in &packed.chains {
        for &t in ch {
            inst_of_tile[t] = insts.len();
        }
        insts.push(Inst {
            cells: Vec::new(),
            kind: InstKind::Macro,
        });
        chain_rows.push(ch.len());
    }
    for (t, slot) in inst_of_tile.iter_mut().enumerate() {
        if *slot == usize::MAX {
            let _ = t;
            *slot = insts.len();
            insts.push(Inst {
                cells: Vec::new(),
                kind: InstKind::Slice,
            });
            chain_rows.push(0);
        }
    }
    // Nets: every netlist net over the instances and pads it touches,
    // clocks excepted.
    let clock_nets = clock_nets(nl);
    let mut on_net: BTreeMap<NetId, (BTreeSet<u32>, BTreeSet<(i64, i64)>)> = BTreeMap::new();
    let mut touch = |n: NetId, cell: usize, on_net: &mut BTreeMap<NetId, (BTreeSet<u32>, BTreeSet<(i64, i64)>)>| {
        if clock_nets.contains(&n) {
            return;
        }
        let e = on_net.entry(n).or_default();
        match &nl.cells[cell] {
            Cell::Ib { port, .. } | Cell::Ob { port, .. } => {
                let p = pads[*port].site;
                e.1.insert((i64::from(p.row), i64::from(p.col)));
            }
            Cell::Prim(p) if p.pad.is_some() => {
                let s = pads[p.pad.unwrap_or_default()].site;
                e.1.insert((i64::from(s.row), i64::from(s.col)));
            }
            Cell::Ebr(_) | Cell::Prim(_) if packed.loc[cell].is_none() => {
                if let Some(&b) = ebr_at.get(&cell) {
                    let bel = bels.bels[b as usize];
                    e.1.insert((i64::from(bel.y), i64::from(bel.x)));
                }
            }
            _ => {
                if let Some(l) = packed.loc[cell] {
                    e.0.insert(u32::try_from(inst_of_tile[l.tile]).expect("fits"));
                }
            }
        }
    };
    for (i, c) in nl.cells.iter().enumerate() {
        for n in crate::netlist::inputs(c)
            .into_iter()
            .chain(crate::netlist::outputs(c))
        {
            touch(n, i, &mut on_net);
        }
    }
    let mut pnets: Vec<PNet> = Vec::new();
    for (_, (movable, fixed)) in on_net {
        let n_pins = movable.len() + fixed.len();
        if movable.is_empty() || n_pins < 2 || n_pins > opts.max_fanout {
            continue;
        }
        #[expect(clippy::cast_precision_loss, reason = "tile coordinates")]
        let pins: Vec<Pin> = movable
            .into_iter()
            .map(Pin::Inst)
            .chain(fixed.into_iter().map(|(r, c)| Pin::Fixed(r as f64, c as f64)))
            .collect();
        pnets.push(PNet { pins, weight: 1.0 });
    }
    let mut prob = Problem::from_parts(insts, pnets, sites);
    prob.chain_rows = chain_rows;
    prob.site_up = plc
        .iter()
        .map(|&(x, y)| index.get(&(x + 1, y)).copied())
        .collect();
    let mut gp = GpOpts::default();
    let n_sites = plc.len();
    #[expect(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        clippy::cast_precision_loss,
        reason = "bin count"
    )]
    {
        gp.nesterov.bins = ((n_sites as f64 / 12.0).sqrt() as usize).clamp(4, 64);
    }
    let placed = place(&prob, &gp).map_err(|e| err!("place: {e}"))?;
    let mut xy: Vec<(i16, i16)> = vec![(0, 0); packed.tiles.len()];
    for (k, ch) in packed.chains.iter().enumerate() {
        let root = placed.site_of[k].ok_or_else(|| err!("chain {k} has no site"))?;
        let run = prob
            .chain_run(root, ch.len())
            .ok_or_else(|| err!("chain {k}: no run at its site"))?;
        for (&t, &s) in ch.iter().zip(&run) {
            xy[t] = plc[s as usize];
        }
    }
    for (t, &i) in inst_of_tile.iter().enumerate() {
        if prob.chain_rows[i] == 0 {
            let s = placed.site_of[i].ok_or_else(|| err!("tile {t} has no site"))?;
            xy[t] = plc[s as usize];
        }
    }
    Ok((xy, placed.report.hpwl_legal))
}

/// Every net's pins on the placed design.
fn net_pins(
    bels: &Bels,
    nl: &Netlist,
    packed: &Packed,
    xy: &[(i16, i16)],
    pads: &[Pad],
    ebr_at: &HashMap<usize, BelId>,
    iol_plan: &iol::Plan,
) -> Result<BTreeMap<NetId, NetPins>> {
    let mut pins: BTreeMap<NetId, NetPins> = BTreeMap::new();
    let slice_at = |l: Loc| {
        let (x, y) = xy[l.tile];
        slice_bel(bels, x, y, l.z)
    };
    for (i, c) in nl.cells.iter().enumerate() {
        match c {
            Cell::Lut { inputs, z, .. } => {
                let l = packed.loc[i].ok_or_else(|| err!("LUT {i} unpacked"))?;
                let b = slice_at(l)?;
                pins.entry(*z).or_default().source = Some((b, format!("F{}", l.lane)));
                for (k, n) in inputs.iter().enumerate() {
                    if let Some(n) = n {
                        pins.entry(*n)
                            .or_default()
                            .sinks
                            .push((b, format!("{}{}", ABCD[k], l.lane)));
                    }
                }
            }
            Cell::Ccu2 { halves, .. } => {
                let l = packed.loc[i].ok_or_else(|| err!("CCU2C {i} unpacked"))?;
                let b = slice_at(l)?;
                for (j, h) in halves.iter().enumerate() {
                    if let Some(s) = h.s {
                        pins.entry(s).or_default().source = Some((b, format!("F{j}")));
                    }
                    for (k, n) in h.inputs.iter().enumerate() {
                        if let Some(n) = n {
                            pins.entry(*n)
                                .or_default()
                                .sinks
                                .push((b, format!("{}{j}", ABCD[k])));
                        }
                    }
                }
            }
            Cell::Ff { d, q, ctrl, .. } => {
                let l = packed.loc[i].ok_or_else(|| err!("flop {i} unpacked"))?;
                let b = slice_at(l)?;
                pins.entry(*q).or_default().source = Some((b, format!("Q{}", l.lane)));
                if !d_from_lane(nl, packed, l, *d) {
                    pins.entry(*d)
                        .or_default()
                        .sinks
                        .push((b, format!("M{}", l.lane)));
                }
                pins.entry(ctrl.clk)
                    .or_default()
                    .clock_sinks
                    .push((b, "CLK".to_owned()));
                if let Some(lsr) = ctrl.lsr {
                    pins.entry(lsr.net).or_default().sinks.push((b, "LSR".to_owned()));
                }
                if let Some((ce, _)) = ctrl.ce {
                    pins.entry(ce).or_default().sinks.push((b, "CE".to_owned()));
                }
            }
            Cell::Ib { port, o } => {
                pins.entry(*o).or_default().source = Some((pads[*port].bel, "O".to_owned()));
            }
            Cell::Ob { port, i: n } => {
                let pin = iol_plan.pio_pin.get(n).copied().unwrap_or("I");
                pins.entry(*n)
                    .or_default()
                    .sinks
                    .push((pads[*port].bel, pin.to_owned()));
            }
            Cell::Prim(p) if p.ty == crate::soft::LUTRAM => {
                let l = packed.loc[i].ok_or_else(|| err!("distributed RAM {} unpacked", p.name))?;
                let sl = |z: usize| slice_at(Loc { tile: l.tile, z, lane: 0 });
                let (a, bb, c) = (sl(0)?, sl(1)?, sl(2)?);
                for (pin, conn) in &p.pins {
                    let (n, out) = match *conn {
                        PrimPin::In(Some(n)) => (n, false),
                        PrimPin::Out(Some(n)) => (n, true),
                        _ => continue,
                    };
                    let k: usize = pin[pin.len() - 1..].parse().unwrap_or(0);
                    let e = pins.entry(n).or_default();
                    match &pin[..pin.len().min(3)] {
                        // Read address on every DPRAM LUT: A = RAD3, B =
                        // RAD1, C = RAD2, D = RAD0.
                        "RAD" => {
                            let letter = ["D", "B", "C", "A"][k];
                            for s in [a, bb] {
                                for lane in 0..2 {
                                    e.sinks.push((s, format!("{letter}{lane}")));
                                }
                            }
                        }
                        // Write port on slice C (nextpnr dram_to_ramw_split).
                        "WAD" => e.sinks.push((c, ["D0", "B0", "C0", "A0"][k].to_owned())),
                        "DI0" | "DI1" | "DI2" | "DI3" => {
                            e.sinks.push((c, ["C1", "A1", "D1", "B1"][k].to_owned()));
                        }
                        "DO0" | "DO1" | "DO2" | "DO3" if out => {
                            e.source = Some((if k < 2 { a } else { bb }, format!("F{}", k % 2)));
                        }
                        "WRE" => e.sinks.push((a, "WRE".to_owned())),
                        "WCK" => e.clock_sinks.push((a, "WCK".to_owned())),
                        _ => return Err(err!("distributed RAM {}: unknown pin {pin}", p.name)),
                    }
                }
            }
            Cell::Prim(p) if iol_plan.cells.contains_key(&i) => {
                let (k, map) = &iol_plan.cells[&i];
                let b = iol_plan.iols[*k].bel;
                for (pin, bp) in map {
                    let Some(bp) = bp else { continue };
                    match p.pin(pin) {
                        Some(PrimPin::In(Some(n))) => {
                            let e = pins.entry(n).or_default();
                            if matches!(*bp, "CLK" | "ECLK") {
                                e.clock_sinks.push((b, (*bp).to_owned()));
                            } else {
                                e.sinks.push((b, (*bp).to_owned()));
                            }
                        }
                        Some(PrimPin::Out(Some(n))) => {
                            pins.entry(n).or_default().source = Some((b, (*bp).to_owned()));
                        }
                        _ => {}
                    }
                }
            }
            Cell::Prim(p) => {
                let b = match p.pad {
                    Some(port) => pads[port].bel,
                    None => *ebr_at
                        .get(&i)
                        .ok_or_else(|| err!("{} {} has no bel", p.ty, p.name))?,
                };
                for (pin, conn) in &p.pins {
                    match *conn {
                        PrimPin::In(Some(n)) => {
                            let clock = crate::netlist::is_clock_input(&p.ty, pin);
                            // A pad's I / T from its IOLOGIC.
                            let pin = match iol_plan.pio_pin.get(&n) {
                                Some(pp) if p.pad.is_some() && matches!(pin.as_str(), "I" | "T") => (*pp).to_owned(),
                                _ => crate::sys::bel_pin(&p.ty, pin).to_owned(),
                            };
                            let e = pins.entry(n).or_default();
                            if clock {
                                e.clock_sinks.push((b, pin));
                            } else {
                                e.sinks.push((b, pin));
                            }
                        }
                        PrimPin::Out(Some(n)) => {
                            pins.entry(n).or_default().source = Some((b, pin.clone()));
                        }
                        _ => {}
                    }
                }
            }
            Cell::Ebr(e) => {
                let b = *ebr_at.get(&i).ok_or_else(|| err!("block RAM {i} has no bel"))?;
                for (pin, u) in ebr::pin_uses(e) {
                    match u {
                        Use::Sink(n) => pins.entry(n).or_default().sinks.push((b, pin)),
                        Use::Clock(n) => pins.entry(n).or_default().clock_sinks.push((b, pin)),
                        Use::Source(n) => pins.entry(n).or_default().source = Some((b, pin)),
                        Use::Tie(_) | Use::Control(_) => {}
                    }
                }
            }
        }
    }
    for np in pins.values_mut() {
        np.sinks.sort();
        np.sinks.dedup();
        np.clock_sinks.sort();
        np.clock_sinks.dedup();
    }
    Ok(pins)
}

/// Whether a flop at `l` takes D from its own lane's LUT or carry half
/// (through DI) rather than from M.
fn d_from_lane(nl: &Netlist, packed: &Packed, l: Loc, d: NetId) -> bool {
    let Some(s) = packed.tiles[l.tile].slices[l.z] else {
        return false;
    };
    match s.kind {
        SliceKind::Logic(lanes) => lanes[l.lane].lut.is_some_and(|lut| {
            matches!(&nl.cells[lut as usize], Cell::Lut { z, .. } if *z == d)
        }),
        SliceKind::Carry { ccu2, .. } => {
            matches!(&nl.cells[ccu2 as usize], Cell::Ccu2 { halves, .. } if halves[l.lane].s == Some(d))
        }
        SliceKind::Dpram { .. } | SliceKind::Ramw { .. } => false,
    }
}

fn init_word(init: u16) -> Vec<bool> {
    (0..16).map(|j| (init >> j) & 1 == 1).collect()
}

#[expect(clippy::too_many_arguments, reason = "the whole design in, one configuration out")]
fn configure(
    g: &Graph,
    db: &DeviceDb,
    root: &Source,
    bels: &Bels,
    nl: &Netlist,
    packed: &Packed,
    xy: &[(i16, i16)],
    pads: &[Pad],
    lpf: &Lpf,
    signal: &[(NetId, Net)],
    routes: &Routes,
    trees: &[(NetId, ClockTree)],
    opts: &FlowOptions,
) -> Result<ChipConfig> {
    let mut dump: Option<String> = std::env::var_os("OXIBIT_DUMP_FF").map(|_| String::new());
    let mut tiles: BTreeMap<String, TileConfig> = BTreeMap::new();
    let plc_name = |x: i16, y: i16| format!("R{y}C{x}:PLC2");
    // Per tile and net: the clock and set/reset polarity (and mode) its
    // flops want.
    let mut clk_pol: HashMap<((i16, i16), NetId), Polarity> = HashMap::new();
    let mut lsr_cfg: HashMap<((i16, i16), NetId), (Polarity, SrMode)> = HashMap::new();
    for (t, tile) in packed.tiles.iter().enumerate() {
        let (x, y) = xy[t];
        let tc = tiles.entry(plc_name(x, y)).or_default();
        for (z, s) in tile.slices.iter().enumerate() {
            let Some(s) = s else { continue };
            let sl = format!("SLICE{}", ABCD[z]);
            let mut ffs: [Option<u32>; 2] = [None, None];
            match s.kind {
                SliceKind::Logic(lanes) => {
                    tc.enums.push((format!("{sl}.MODE"), "LOGIC".to_owned()));
                    for (i, lane) in lanes.iter().enumerate() {
                        if let Some(lut) = lane.lut {
                            let Cell::Lut { init, .. } = &nl.cells[lut as usize] else {
                                return Err(err!("lane LUT {lut} is not a LUT"));
                            };
                            tc.words.push((format!("{sl}.K{i}.INIT"), init_word(*init)));
                        }
                        ffs[i] = lane.ff;
                    }
                }
                SliceKind::Carry { ccu2, ff } => {
                    let Cell::Ccu2 { halves, .. } = &nl.cells[ccu2 as usize] else {
                        return Err(err!("carry slice cell {ccu2} is not a CCU2C"));
                    };
                    tc.enums.push((format!("{sl}.MODE"), "CCU2".to_owned()));
                    for (j, h) in halves.iter().enumerate() {
                        tc.words.push((format!("{sl}.K{j}.INIT"), init_word(h.init)));
                        tc.enums.push((format!("{sl}.CCU2.INJECT1_{j}"), "NO".to_owned()));
                    }
                    ffs = ff;
                }
                SliceKind::Dpram { ram, half } => {
                    let Cell::Prim(p) = &nl.cells[ram as usize] else {
                        return Err(err!("distributed RAM cell {ram} is not a primitive"));
                    };
                    tc.enums.push((format!("{sl}.MODE"), "DPRAM".to_owned()));
                    let init = lutram_init(p);
                    for lane in 0..2 {
                        let word = lutram_lut_init(init, half * 2 + lane);
                        tc.words.push((format!("{sl}.K{lane}.INIT"), init_word(word)));
                    }
                    if half == 0 {
                        let wre = p.param_str("WREMUX").unwrap_or_else(|| "WRE".to_owned());
                        tc.enums.push(("SLICEA.WREMUX".to_owned(), wre));
                        let wck = p.param_str("WCKMUX").unwrap_or_else(|| "WCK".to_owned());
                        let wck = if wck == "INV" { "INV" } else { "CLK" };
                        tc.enums.push(("CLK1.CLKMUX".to_owned(), wck.to_owned()));
                    }
                }
                SliceKind::Ramw { .. } => {
                    tc.enums.push((format!("{sl}.MODE"), "RAMW".to_owned()));
                    for lane in 0..2 {
                        tc.words.push((format!("{sl}.K{lane}.INIT"), init_word(0)));
                    }
                }
            }
            // Flops: D source, set/reset value; the slice's enable.
            for (i, f) in ffs.iter().enumerate() {
                let Some(f) = f else { continue };
                let Cell::Ff { d, regset, .. } = &nl.cells[*f as usize] else {
                    return Err(err!("lane flop {f} is not a flop"));
                };
                let loc = packed.loc[*f as usize].ok_or_else(|| err!("flop {f} unpacked"))?;
                let sd = if d_from_lane(nl, packed, loc, *d) { "1" } else { "0" };
                if let Some(out) = dump.as_mut() {
                    let Cell::Ff { q, ctrl, .. } = &nl.cells[*f as usize] else { unreachable!() };
                    let name = |n: NetId| net_name(nl, n);
                    out.push_str(&format!(
                        "{{\"q\":{:?},\"tile\":\"R{y}C{x}\",\"slice\":\"{}\",\"reg\":{i},\"sd\":{sd},\"regset\":\"{:?}\",\"d\":{:?},\"clk\":{:?},\"ce\":{:?},\"lsr\":{:?},\"kind\":\"{}\"}}\n",
                        name(*q),
                        ABCD[z],
                        regset,
                        name(*d),
                        name(ctrl.clk),
                        ctrl.ce.map(|(n, p)| format!("{}{}", if p == Polarity::Neg { "!" } else { "" }, name(n))),
                        ctrl.lsr.map(|l| format!("{:?}", l)),
                        if matches!(s.kind, SliceKind::Carry { .. }) { "carry" } else { "logic" },
                    ));
                }
                tc.enums.push((format!("{sl}.REG{i}.SD"), sd.to_owned()));
                tc.enums.push((
                    format!("{sl}.REG{i}.REGSET"),
                    if *regset == RegSet::Set { "SET" } else { "RESET" }.to_owned(),
                ));
            }
            if let Some(c) = s.ctrl {
                let ce = match c.ce {
                    None => "1",
                    Some((_, Polarity::Pos)) => "CE",
                    Some((_, Polarity::Neg)) => "INV",
                };
                tc.enums.push((format!("{sl}.CEMUX"), ce.to_owned()));
                clk_pol.insert(((x, y), c.clk), c.clk_polarity);
                if let Some(l) = c.lsr {
                    lsr_cfg.insert(((x, y), l.net), (l.polarity, l.mode));
                }
            }
        }
    }

    // Routed arcs (deduplicated per tile) and, where a route goes through a
    // tile's CLK<k> / LSR<k>, that resource's inversion and mode.
    let mut arcs: BTreeMap<String, BTreeSet<(String, String)>> = BTreeMap::new();
    let mut shared = |net: NetId, to: Wire, tiles: &mut BTreeMap<String, TileConfig>| {
        let name = &g.names[to.name as usize];
        let at = (to.x, to.y);
        if let Some(k) = ["CLK0", "CLK1"].iter().position(|w| w == name) {
            if clk_pol.get(&(at, net)) == Some(&Polarity::Neg) {
                tiles
                    .entry(plc_name(at.0, at.1))
                    .or_default()
                    .enums
                    .push((format!("CLK{k}.CLKMUX"), "INV".to_owned()));
            }
        }
        if let Some(k) = ["LSR0", "LSR1"].iter().position(|w| w == name) {
            if let Some(&(pol, mode)) = lsr_cfg.get(&(at, net)) {
                let tc = tiles.entry(plc_name(at.0, at.1)).or_default();
                if pol == Polarity::Neg {
                    tc.enums.push((format!("LSR{k}.LSRMUX"), "INV".to_owned()));
                }
                if mode == SrMode::Async {
                    tc.enums.push((format!("LSR{k}.SRMODE"), "ASYNC".to_owned()));
                }
            }
        }
    };
    for ((n, _), net_arcs) in signal.iter().zip(&routes.nets) {
        for a in net_arcs {
            let p = g.pip(a);
            if p.kind == PipKind::Configurable {
                arcs.entry(g.tiles[a.tile as usize].name.clone())
                    .or_default()
                    .insert((p.sink.clone(), p.source.clone()));
            }
            shared(*n, a.to, &mut tiles);
        }
    }
    for (n, tree) in trees {
        tree_config(g, db, tree, &mut tiles)?;
        for a in &tree.arcs {
            shared(*n, a.to(), &mut tiles);
        }
    }
    for (t, set) in arcs {
        tiles.entry(t).or_default().arcs.extend(set);
    }

    // Pads. A pad primitive (BB, OBZ) routes its own T.
    let t_routed: HashSet<usize> = nl
        .cells
        .iter()
        .filter_map(|c| match c {
            Cell::Prim(p) => p.pad,
            _ => None,
        })
        .collect();
    let mut uses = Vec::new();
    for (k, p) in nl.ports.iter().enumerate() {
        let pad = pads[k];
        let dir = if p.bidir {
            "BIDIR"
        } else if p.output {
            "OUTPUT"
        } else {
            "INPUT"
        };
        uses.push(crate::pads::PadUse {
            name: &p.name,
            dir,
            site: pad.site,
            prefs: lpf.port(&p.name).map(|pp| &pp.iobuf),
            attrs: nl.pad_attrs.get(&k),
        });
        if p.output && !t_routed.contains(&k) {
            let t = pin_wire(bels, pad.bel, "T")?;
            tie(g, db, t, Level::Low, &mut tiles)?;
        }
    }
    crate::pads::configure(g, db, root, &opts.device, &uses, &mut tiles)?;
    // Duplicate enums (a tile seen from two nets) once each.
    for tc in tiles.values_mut() {
        tc.enums.sort();
        tc.enums.dedup();
    }
    if let (Some(out), Some(path)) = (dump, std::env::var_os("OXIBIT_DUMP_FF")) {
        std::fs::write(&path, out).map_err(|e| err!("dump: {e}"))?;
    }
    Ok(ChipConfig {
        device: opts.device.clone(),
        metadata: vec![format!("oxibit ecp5 flow: {}", nl.top)],
        tiles,
        ..ChipConfig::default()
    })
}

/// General wires that enter the global clock network: the drivers of the
/// `*PCLKCIB*` inputs of the DCC input muxes.
fn cib_entries(g: &Graph, bels: &crate::bels::Bels) -> Vec<Wire> {
    let mut out = BTreeSet::new();
    for b in 0..bels.bels.len() {
        let b = BelId::try_from(b).expect("fits");
        if bels.template(b).ty != "DCCA" {
            continue;
        }
        let Some(clki) = bels.pin_wire(b, "CLKI") else { continue };
        // The DCC input mux sits one or two wires up from the pin.
        let mut front = vec![clki];
        for _ in 0..3 {
            let mut next = Vec::new();
            for w in front {
                for a in g.uphill(w) {
                    if g.wire_name(a.from).contains("PCLKCIB") {
                        out.extend(g.uphill(a.from).into_iter().map(|e| e.from));
                    } else {
                        next.push(a.from);
                    }
                }
            }
            front = next;
        }
    }
    out.into_iter().collect()
}

/// A clock tree from the CIB entry nearest `source` that reaches every
/// sink (up to eight entries tried).
fn via_cib(
    g: &Graph,
    cnet: &ClockNetwork,
    entries: &[Wire],
    source: Wire,
    sinks: &[Wire],
    taken: &HashSet<Wire>,
    opts: crate::clock::ClockOptions,
) -> Option<(Wire, ClockTree)> {
    let mut by: Vec<Wire> = entries.iter().copied().filter(|e| !taken.contains(e)).collect();
    by.sort_by_key(|e| {
        let (dx, dy) = (i32::from(e.x - source.x), i32::from(e.y - source.y));
        dx * dx + dy * dy
    });
    by.into_iter()
        .take(8)
        .find_map(|e| match route_clock(g, cnet, e, sinks, taken, opts) {
            Ok(t) => Some((e, t)),
            Err(x) => {
                if std::env::var_os("OXIBIT_DEBUG_CIB").is_some() {
                    eprintln!("  via {}: {x}", g.wire_name(e));
                }
                None
            }
        })
}

/// A distributed RAM's 64-bit INITVAL: word `w` in bits `4w..4w+3`.
fn lutram_init(p: &crate::netlist::Prim) -> u64 {
    use oxibit_hdl::netlist::Param;
    match p.params.get("INITVAL") {
        Some(Param::Bits(b)) => b
            .iter()
            .enumerate()
            .fold(0u64, |a, (i, x)| a | (u64::from(*x == Some(true)) << i)),
        _ => 0,
    }
}

/// The physical INIT of LUT `bit` (0..4) of a distributed RAM. The read
/// address reaches the LUT as A = RAD3, B = RAD1, C = RAD2, D = RAD0 (the
/// write address the same way through WAD), so physical index `{D,C,B,A}`
/// holds the word at logical address `{A,C,B,D}` (as nextpnr
/// `dram_to_comb`).
fn lutram_lut_init(init: u64, bit: usize) -> u16 {
    let mut word = 0u16;
    for i in 0..16usize {
        let addr = (if i & 1 != 0 { 8 } else { 0 })
            | (if i & 2 != 0 { 2 } else { 0 })
            | (if i & 4 != 0 { 4 } else { 0 })
            | (if i & 8 != 0 { 1 } else { 0 });
        if (init >> (4 * addr + bit)) & 1 == 1 {
            word |= 1 << i;
        }
    }
    word
}

/// A pin wire at the chip origin (a global-network bel's control input)
/// fed by one fixed connection: the wire feeding it, where a route ends.
fn local_sink(g: &Graph, w: Wire) -> Wire {
    let mut w = w;
    for _ in 0..4 {
        if w.x != 0 || w.y != 0 {
            break;
        }
        let up: Vec<Wire> = g
            .uphill(w)
            .into_iter()
            .filter(|a| g.pip(a).kind == PipKind::Fixed)
            .map(|a| a.from)
            .collect();
        match up.as_slice() {
            [one] => w = *one,
            _ => break,
        }
    }
    w
}
