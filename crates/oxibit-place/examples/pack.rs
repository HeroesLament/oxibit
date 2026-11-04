//! Pack a primitive netlist; given nextpnr's placed design of the same
//! netlist, compare the packing, take its bels, legalise, and write the
//! placed design for oxibit-pnr.
//!
//!   pack DB_FAMILY_DIR PART XDC META_DIR SYNTH.json OUT.json [REFERENCE_PLACED.json | --place [SEED]]
//!
//! stdout: JSON lines (pack stats, comparison, placement).

use oxibit_place::policy::{CarryDi, CarryFf, Level, Lut5Fill, PackPolicy};
use oxibit_place::{compare, netlist::Netlist, pack, place};
use oxibit_pnr::packed::Design;
use serde_json::{Value, json};

fn read(path: &str) -> Result<Value, String> {
    serde_json::from_str(&std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?)
        .map_err(|e| format!("{path}: {e}"))
}

/// The packing policy from the environment:
///   OXIBIT_PACK_POLICY  exploration (default) | production | nextpnr
///   OXIBIT_PACK_FLOOR   unsupported | modeled | pnr | fasm | silicon
///   OXIBIT_CARRY_DI     lut5 | bypass
///   OXIBIT_CARRY_FF     sole | shared
///   OXIBIT_LUT5_FILL    none, or a comma list of chain, ff_letter, pairs
///   OXIBIT_PACK_FOLD    0 | 1
///   OXIBIT_FILL_BYPASS  flops on bypass pins per slice, 1..4
///   OXIBIT_PLACE_PIN_CAP / _PIN_WEIGHT  input pins per CLB tile over which each costs WEIGHT
///   OXIBIT_PLACE_BYP_CAP / _BYP_WEIGHT  the same for bypass pins (8 per tile)
///   OXIBIT_PLACE_DENSITY / _DENSITY_WEIGHT / _DENSITY_BIN  used-slice share per bin over which each slice costs WEIGHT
///   OXIBIT_PLACE_CRIT / _CRIT_K / _CRIT_EXP  net criticalities (route_sta STA_CRIT_OUT): wirelength weight 1 + K * crit^EXP
fn pack_policy() -> Result<PackPolicy, String> {
    let var = |k: &str| std::env::var(k).ok();
    let mut pol = match var("OXIBIT_PACK_POLICY").as_deref() {
        None | Some("exploration") => PackPolicy::exploration(),
        Some("production") => PackPolicy::production(),
        Some("nextpnr") => PackPolicy::nextpnr_reference(),
        Some(o) => {
            return Err(format!(
                "OXIBIT_PACK_POLICY={o}: expected exploration, production or nextpnr"
            ));
        }
    };
    if let Some(v) = var("OXIBIT_PACK_FLOOR") {
        pol.floor = Level::parse(&v).ok_or_else(|| {
            format!("OXIBIT_PACK_FLOOR={v}: expected unsupported, modeled, pnr, fasm or silicon")
        })?;
    }
    if let Some(v) = var("OXIBIT_CARRY_DI") {
        pol.carry_di = CarryDi::parse(&v)
            .ok_or_else(|| format!("OXIBIT_CARRY_DI={v}: expected lut5 or bypass"))?;
    }
    if let Some(v) = var("OXIBIT_CARRY_FF") {
        pol.carry_ff = match v.as_str() {
            "sole" => CarryFf::SoleUser,
            "shared" => CarryFf::Shared,
            o => return Err(format!("OXIBIT_CARRY_FF={o}: expected sole or shared")),
        };
    }
    if let Some(v) = var("OXIBIT_LUT5_FILL") {
        pol.lut5 = Lut5Fill::NONE;
        for w in v
            .split(',')
            .map(str::trim)
            .filter(|w| !w.is_empty() && *w != "none")
        {
            match w {
                "chain" => pol.lut5.chain = true,
                "ff_letter" => pol.lut5.ff_letter = true,
                "pairs" => pol.lut5.pairs = true,
                o => {
                    return Err(format!(
                        "OXIBIT_LUT5_FILL: unknown {o}: expected chain, ff_letter, pairs or none"
                    ));
                }
            }
        }
    }
    if let Some(v) = var("OXIBIT_PACK_FOLD") {
        pol.fold = v != "0";
    }
    if let Some(v) = var("OXIBIT_FILL_BYPASS") {
        pol.bypass_ffs_per_slice = v
            .parse()
            .map_err(|_| format!("OXIBIT_FILL_BYPASS={v}: expected 1..4"))?;
    }
    let num = |k: &str| -> Result<Option<f64>, String> {
        var(k)
            .map(|v| {
                v.parse::<f64>()
                    .map_err(|_| format!("{k}={v}: expected a number"))
            })
            .transpose()
    };
    if let Some(c) = num("OXIBIT_PLACE_PIN_CAP")? {
        pol.objective.pins.cap = c as u32;
    }
    if let Some(w) = num("OXIBIT_PLACE_PIN_WEIGHT")? {
        pol.objective.pins.weight = w;
    }
    if let Some(c) = num("OXIBIT_PLACE_BYP_CAP")? {
        pol.objective.bypass.cap = c as u32;
    }
    if let Some(w) = num("OXIBIT_PLACE_BYP_WEIGHT")? {
        pol.objective.bypass.weight = w;
    }
    if let Some(t) = num("OXIBIT_PLACE_DENSITY")? {
        pol.objective.density.target = t;
    }
    if let Some(w) = num("OXIBIT_PLACE_DENSITY_WEIGHT")? {
        pol.objective.density.weight = w;
    }
    if let Some(b) = num("OXIBIT_PLACE_DENSITY_BIN")? {
        pol.objective.density.bin = b as i32;
    }
    if let Some(v) = var("OXIBIT_PLACE_FLOORPLAN") {
        pol.floorplan = oxibit_place::floorplan::Floorplan::parse(&v)?;
    }
    Ok(pol)
}

use oxibit_report::Progress;

static CANCEL: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

/// Progress through oxibit-report (docs/events.md): human lines on stderr
/// (a live status line on a terminal), the JSON Lines stream to the file
/// OXIBIT_EVENTS names. stdout keeps its one JSON object per line.
fn reporter() -> oxibit_report::Reporter {
    use oxibit_report::{Human, HumanStyle, JsonLines, Reporter, Sink};
    use std::io::IsTerminal;
    let tty = std::io::stderr().is_terminal() && std::env::var_os("CI").is_none();
    let style = if tty {
        HumanStyle::Live
    } else {
        HumanStyle::Plain
    };
    let mut sinks: Vec<Box<dyn Sink>> = vec![Box::new(Human::new(std::io::stderr(), style))];
    if let Some(p) = std::env::var_os("OXIBIT_EVENTS") {
        match std::fs::File::create(&p) {
            Ok(f) => sinks.push(Box::new(JsonLines(std::io::BufWriter::new(f)))),
            Err(e) => eprintln!("{}: {e}", p.to_string_lossy()),
        }
    }
    Reporter::new(sinks, &CANCEL)
}

fn run() -> Result<bool, String> {
    let args: Vec<String> = std::env::args().collect();
    let r = reporter();
    r.emit(oxibit_report::Event::Start {
        cmd: "pack".into(),
        version: env!("CARGO_PKG_VERSION").into(),
        args: args[1..].to_vec(),
    });
    let ok = run_with(&args, &r);
    r.emit(oxibit_report::Event::Done {
        ok: matches!(ok, Ok(true)),
        exit: match ok {
            Ok(true) => 0,
            Ok(false) => 1,
            Err(_) => 3,
        },
        cancelled: r.cancelled(),
        phases: r.phases(),
        peak_rss_mb: None,
        stats: json!({}),
        check: json!({}),
        outputs: vec![],
    });
    r.finish();
    ok
}

fn run_with(args: &[String], r: &oxibit_report::Reporter) -> Result<bool, String> {
    let [_, db, part, xdc, meta, synth, out, rest @ ..] = &args[..] else {
        return Err("usage: pack DB_FAMILY_DIR PART XDC META_DIR SYNTH.json OUT.json [REFERENCE_PLACED.json]".into());
    };
    let t0 = std::time::Instant::now();
    let dev = oxibit_place::device::Device::open(std::path::Path::new(db), part)?;
    let props =
        oxibit_place::xdc::parse(&std::fs::read_to_string(xdc).map_err(|e| format!("{xdc}: {e}"))?);
    let nl = Netlist::from_json(&read(synth)?)?;
    let plan = pack_policy()?.plan(&oxibit_place::policy::xc7z020())?;
    println!("{}", json!({"pack_policy": plan.json()}));
    r.begin("pack");
    let mut p = pack::pack_with(nl, &dev, &props, &plan)?;
    r.end("pack");
    let mut types: std::collections::BTreeMap<String, usize> = Default::default();
    for c in p.nl.cell_ids() {
        *types.entry(p.nl.cell(c).ty.clone()).or_default() += 1;
    }
    println!(
        "{}",
        json!({"pack": {"stats": p.stats, "cells": p.nl.cell_ids().len(), "types": types, "clusters": p.clusters.len(),
                        "clustered_cells": p.cluster_of.len(), "seconds": t0.elapsed().as_secs_f64()}})
    );
    let mut ok = true;
    if rest.first().map(String::as_str) == Some("--place") {
        // oxibit's own placer.
        let seed = rest.get(1).and_then(|s| s.parse().ok()).unwrap_or(1);
        let sites = oxibit_place::placer::Sites::from_device(&dev);
        let fopts = plan.fill();
        let (macros, fs) = oxibit_place::placer::fill(&p, fopts);
        println!(
            "{}",
            json!({"fill": {"chains": fs.chains, "chain_slices": fs.chain_slices, "memory_slices": fs.memory_slices, "srl_slices": fs.srl_slices,
                            "logic_slices": fs.logic_slices, "lone_lut_letters": fs.lone_lut_letters, "ff2_used": fs.ff2_used, "chain_5luts": fs.chain_5luts, "paired_luts": fs.paired_luts, "ff_letter_5luts": fs.ff_letter_5luts,
                            "control_sets": fs.control_sets, "leftovers": fs.leftovers.len(), "leftover_examples": fs.leftovers.iter().take(6).collect::<Vec<_>>(),
                            "sites": {"slices": sites.slices.len(), "slicem": sites.slices.iter().filter(|s| s.m).count(), "dsp": sites.dsp.len(),
                                      "ramb36": sites.ramb36.len(), "ramb18": sites.ramb18.len(), "bufg": sites.bufg.len(), "ps7": sites.ps7.len()}}})
        );
        let mut opts = oxibit_place::placer::Options {
            seed,
            ..Default::default()
        };
        if let Some(m) = std::env::var("OXIBIT_PLACE_MOVES")
            .ok()
            .and_then(|v| v.parse().ok())
        {
            opts.max_moves_per_obj = m;
        }
        if let Some(m) = std::env::var("OXIBIT_PLACE_T0")
            .ok()
            .and_then(|v| v.parse().ok())
        {
            opts.t0_scale = m;
        }
        if let Some(m) = std::env::var("OXIBIT_PLACE_INNER")
            .ok()
            .and_then(|v| v.parse().ok())
        {
            opts.inner = m;
        }
        if let Some(m) = std::env::var("OXIBIT_PLACE_UNIT")
            .ok()
            .and_then(|v| v.parse().ok())
        {
            opts.unit_frac = m;
        }
        if let Some(m) = std::env::var("OXIBIT_PLACE_UNIT_WINDOW")
            .ok()
            .and_then(|v| v.parse().ok())
        {
            opts.unit_window = m;
        }
        // block RAM groups (one memory's RAMB36 pieces up one column):
        // OXIBIT_BRAM_GROUPS=1 turns them on (off by default, see
        // Options::bram_groups)
        if std::env::var("OXIBIT_BRAM_GROUPS").as_deref() == Ok("1") {
            opts.bram_groups = true;
        }
        // timing-driven placement: OXIBIT_PLACE_CRIT names a file of
        // `net<TAB>criticality` lines (route_sta STA_CRIT_OUT); each net's
        // wirelength weighs 1 + K * crit^E (OXIBIT_PLACE_CRIT_K, default 4;
        // OXIBIT_PLACE_CRIT_EXP, default 2)
        if let Ok(path) = std::env::var("OXIBIT_PLACE_CRIT") {
            let k: f64 = std::env::var("OXIBIT_PLACE_CRIT_K").ok().and_then(|v| v.parse().ok()).unwrap_or(4.0);
            let e: f64 = std::env::var("OXIBIT_PLACE_CRIT_EXP").ok().and_then(|v| v.parse().ok()).unwrap_or(2.0);
            let text = std::fs::read_to_string(&path).map_err(|err| format!("{path}: {err}"))?;
            for line in text.lines() {
                let Some((name, c)) = line.rsplit_once('\t') else { continue };
                let Ok(c) = c.trim().parse::<f64>() else { continue };
                opts.net_weights.insert(name.to_string(), 1.0 + k * c.clamp(0.0, 1.0).powf(e));
            }
            println!("{}", json!({"net_weights": {"file": path, "nets": opts.net_weights.len(), "k": k, "exp": e}}));
        }
        if let Some(m) = std::env::var("OXIBIT_PLACE_POLISH")
            .ok()
            .and_then(|v| v.parse().ok())
        {
            opts.polish = m;
        }
        opts.pins = plan.objective().pins;
        opts.bypass = plan.objective().bypass;
        opts.density = plan.objective().density;
        opts.floorplan = plan.floorplan().clone();
        r.begin("place");
        let ps = oxibit_place::placer::place_with(&mut p, &dev, &sites, macros, &opts, r)?;
        r.end("place");
        println!(
            "{}",
            json!({"hpwl_measured": oxibit_place::placer::hpwl(&p, 200)})
        );
        let lg = place::legalise(&mut p, &place::ps7_inputs(std::path::Path::new(meta))?);
        let unplaced: Vec<String> =
            p.nl.cell_ids()
                .into_iter()
                .filter(|&c| p.nl.cell(c).bel.is_none())
                .map(|c| p.nl.cell(c).name.clone())
                .collect();
        let clashes = oxibit_place::slice::check(&p);
        let outmux = oxibit_place::slice::outmux_check(&p);
        let ctrl = oxibit_place::slice::ctrl_check(&p);
        println!(
            "{}",
            json!({"ctrl_check": {"clashes": ctrl.len(), "examples": ctrl.iter().take(6).collect::<Vec<_>>()}})
        );
        println!(
            "{}",
            json!({"outmux_check": {"clashes": outmux.len(), "examples": outmux.iter().take(6).collect::<Vec<_>>()}})
        );
        ok = unplaced.is_empty()
            && lg.over_five.is_empty()
            && clashes.is_empty()
            && outmux.is_empty()
            && ctrl.is_empty();
        let shown: Vec<String> = clashes
            .iter()
            .take(6)
            .map(|(s, c)| {
                format!(
                    "{s} {}X: {:?} holds, {:?} wanted",
                    (b'A' + c.letter) as char,
                    c.held.by,
                    c.wanted.by
                )
            })
            .collect();
        println!(
            "{}",
            json!({"bypass_check": {"clashes": clashes.len(), "examples": shown}})
        );
        println!(
            "{}",
            json!({"placer": {"objects": ps.objects, "chains": ps.chains_placed, "bram_groups": ps.bram_groups, "bram_grouped": ps.bram_grouped, "hpwl_start": ps.hpwl_start, "hpwl_end": ps.hpwl_end,
                              "moves": ps.moves, "accepted": ps.accepted, "seconds": ps.seconds,
                              "pins": {"cap": opts.pins.cap, "weight": opts.pins.weight, "excess_start": ps.pin_excess_start, "excess_end": ps.pin_excess_end,
                                       "tiles_over_cap": ps.tiles_over_cap, "tile_max": ps.tile_pins_max, "bypass_ffs_per_slice": fopts.bypass_ffs_per_slice},
                              "bypass": {"cap": opts.bypass.cap, "weight": opts.bypass.weight, "excess_start": ps.bypass_excess_start, "excess_end": ps.bypass_excess_end,
                                         "tiles_over_cap": ps.bypass_tiles_over_cap, "tile_max": ps.bypass_tile_max, "fill_clashes": ps.bypass_clashes},
                              "density": {"bin": opts.density.bin, "target": opts.density.target, "weight": opts.density.weight,
                                          "excess_start": ps.density_excess_start, "excess_end": ps.density_excess_end, "bins_over": ps.density_bins_over},
                              "floorplan": ps.floorplan.as_ref().map(|f| f.json()), "steps": ps.trace.len(), "trace": ps.trace.iter().step_by((ps.trace.len() / 25).max(1)).map(|t| json!([(t.0 * 100.0).round() / 100.0, t.1.round(), (t.2 * 1000.0).round() / 1000.0, t.3])).collect::<Vec<_>>(), "unplaced": unplaced.len(),
                              "unplaced_examples": unplaced.iter().take(6).collect::<Vec<_>>(), "fractured": lg.fractured, "over_five": lg.over_five.len()}})
        );
    } else if let Some(r) = rest.first() {
        let theirs = Design::from_json(&read(r)?)?;
        let diff = compare::compare(&p.nl.to_design(), &theirs);
        println!(
            "{}",
            json!({"compare": diff.to_json(), "differences": diff.total()})
        );
        let st = place::import(&mut p, &theirs);
        let lg = place::legalise(&mut p, &place::ps7_inputs(std::path::Path::new(meta))?);
        println!(
            "{}",
            json!({"reference_hpwl": oxibit_place::placer::hpwl(&p, 200)})
        );
        ok = st.unplaced.is_empty() && st.collisions.is_empty() && lg.over_five.is_empty();
        println!(
            "{}",
            json!({"place": {"by_name": st.by_name, "by_cluster": st.by_cluster, "unplaced": st.unplaced.len(),
                             "unplaced_examples": st.unplaced.iter().take(8).collect::<Vec<_>>(),
                             "collisions": st.collisions, "fractured": lg.fractured, "memory_a6": lg.memory_a6,
                             "ps7_ties": lg.ps7_ties, "over_five": lg.over_five.iter().take(8).collect::<Vec<_>>()}})
        );
    }
    let d = p.nl.to_design();
    let mut v = d.to_json();
    let (ports, names) = p.nl.ports_and_names();
    v["modules"][&p.nl.top]["ports"] = ports;
    v["modules"][&p.nl.top]["netnames"] = names;
    std::fs::write(out, serde_json::to_string(&v).map_err(|e| e.to_string())?)
        .map_err(|e| format!("{out}: {e}"))?;
    Ok(ok)
}

fn main() {
    match run() {
        Ok(true) => {}
        Ok(false) => std::process::exit(1),
        Err(e) => {
            eprintln!("pack: {e}");
            std::process::exit(3);
        }
    }
}
