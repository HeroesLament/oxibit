//! Synthesis: the word-level netlist from `oxibit-hdl` to an xc7 netlist.
//!
//! `lower` bit-blasts combinational cells into an AIG between cut points
//! (ports, flops, memories, primitive instances) and puts flop enables and
//! synchronous resets on CE / R / S; `map` covers the AIG with 6-input
//! LUTs; `emit` builds the result as a module of primitive instances,
//! which `oxibit_hdl::yosys_json` writes. Every cell synthesis creates is
//! named `oxsyn_*`, so a checker can tell them from the design's own
//! primitive instances.
//!
//! The run is driven by a `policy::Policy` against a `caps::DeviceCaps`:
//! every optional pass is a typed `stage::Pass` bound to the stage it may
//! run in (`stage` has the typestates), states the primitive modes it emits
//! and, after the checking view, its evidence; `stage::Flow` runs, refuses
//! or skips each and records why in `Report::passes`.

pub mod aig;
pub mod balance;
pub mod bram;
pub mod caps;
pub mod dfeed;
pub mod dsp;
pub mod emit;
pub mod ffmerge;
pub mod iopad;
pub mod lib_ports;
pub mod lower;
pub mod map;
pub mod memdff;
pub mod muxchain;
pub mod policy;
pub mod srl;
pub mod stage;

use oxibit_hdl::netlist::Module;
use oxibit_report::{Event, Progress};
use serde_json::json;

use caps::{DeviceCaps, Mode};
use policy::Policy;
use stage::{Flow, Mapped, PassRecord, Rtl};

pub struct Report {
    pub ands: usize,
    pub aig_inputs: usize,
    pub depth: u32,
    pub undriven: usize,
    pub carry4: usize,
    pub dsp: dsp::DspStats,
    pub ands_by_op: std::collections::BTreeMap<&'static str, usize>,
    /// Carry-chain bits per word-level cell type (before the dead-chain sweep).
    pub carry_by_op: std::collections::BTreeMap<&'static str, usize>,
    pub mem_rd_ff: usize,
    pub dead_ff: usize,
    pub dead_chain: usize,
    pub stats: emit::Stats,
    pub bram: bram::BramStats,
    pub srl: srl::SrlStats,
    pub ffmerge: ffmerge::FfMergeStats,
    pub iopad: iopad::IoPadStats,
    pub dfeed: dfeed::DFeedStats,
    pub balance: balance::BalanceStats,
    pub muxpack: muxchain::MuxChainStats,
    /// Every optional pass: ran, disabled or refused, with its evidence.
    pub passes: Vec<PassRecord>,
    pub lower_ms: u64,
    pub map_ms: u64,
    pub emit_ms: u64,
}

/// What `synth` returns.
pub struct Synth {
    /// The netlist for place and route.
    pub netlist: Module,
    /// The netlist the equivalence check compares against the source.
    pub check: Module,
    pub report: Report,
}

/// Synthesize `m` for `caps` under `policy`. `dirs` gives the port
/// directions of the primitives the design instantiates. Phases, per-pass
/// counters and refusals go to `rep` (oxibit-report events).
pub fn synth(m: &Module, dirs: &lib_ports::PrimPorts, caps: &DeviceCaps, policy: &Policy, rep: &dyn Progress) -> Result<Synth, String> {
    let t0 = std::time::Instant::now();
    let mut flow = Flow::new(caps, policy, rep);
    // The core's own output must meet the floor too.
    let mut core = vec![Mode::Lut, Mode::FfSync];
    if policy.carry_min > 0 {
        core.push(Mode::Carry4);
    }
    if let Some(why) = flow.below_floor(&core) {
        return Err(format!("core synthesis output {why}"));
    }
    if caps.cost.lut_k != map::K {
        return Err(format!(
            "{} has {}-input LUTs; the mapper covers with {}-input cuts",
            caps.part,
            caps.cost.lut_k,
            map::K
        ));
    }

    rep.emit(phase("prepare", true));
    let mut rtl = Rtl::new(m.clone());
    let mem_rd_ff = flow.rtl(&memdff::MemDff, &mut rtl);
    let mc = flow.rtl(&muxchain::MuxPack, &mut rtl);
    let ds = flow.rtl(
        &dsp::Dsp {
            max_per_mul: policy.dsp_max_per_mul,
        },
        &mut rtl,
    );
    rep.emit(phase("prepare", false));

    rep.emit(phase("lower", true));
    let mut lw = lower::lower(
        rtl.module(),
        dirs,
        lower::Config {
            carry_min: policy.carry_min,
            lut_k: caps.cost.lut_k,
        },
    )?;
    if lw.ffs.iter().any(|f| matches!(f.kind, lower::FfKind::Fdce | lower::FfKind::Fdpe)) {
        if let Some(why) = flow.below_floor(&[Mode::FfAsync]) {
            return Err(format!("asynchronous flops: {why}"));
        }
    }
    for t in &lw.truncated {
        rep.emit(Event::Warn {
            code: "port_resized".into(),
            msg: format!("connection wider than the primitive port: {t}"),
        });
    }
    let (dead_ff, dead_chain) = lw.sweep();
    let bal = flow.aig(&balance::Balance, &mut lw);
    rep.emit(phase("lower", false));
    rep.emit(Event::progress(
        "lower",
        json!({"ands": lw.aig.ands(), "aig_inputs": lw.aig.inputs.len(), "flops": lw.ffs.len(), "carry_chains": lw.aig.chains.len(),
               "dead_ff": dead_ff, "dead_chain": dead_chain}),
    ));
    let t1 = std::time::Instant::now();

    let outs = lw.outputs();
    rep.emit(phase("map", true));
    let mp = map::map(&lw.aig, &outs, policy.area_passes, rep);
    rep.emit(phase("map", false));
    let t2 = std::time::Instant::now();

    rep.emit(phase("emit", true));
    let (out, stats) = emit::emit(&lw, &mp);
    let (check, mut phys) = Mapped(out).split();
    let fm = flow.physical(&ffmerge::FfMerge, &mut phys);
    let ss = flow.physical(&srl::Srl, &mut phys);
    let bs = flow.physical(&bram::Bram, &mut phys);
    let io = flow.physical(&iopad::IoPad, &mut phys);
    let df = flow.physical(&dfeed::DFeed { dirs }, &mut phys);
    for p in &io.inout_bare {
        rep.emit(Event::Warn {
            code: "inout_unbuffered".into(),
            msg: format!("inout port bit {p} has no pad primitive; left unbuffered"),
        });
    }
    // Flop counts are of the final netlist.
    let mut stats = stats;
    for (ty, n) in ss.by_type.iter().chain(&fm.by_type) {
        if let Some(c) = stats.ffs.get_mut(ty) {
            *c -= n;
        }
    }
    rep.emit(phase("emit", false));
    rep.emit(Event::progress(
        "emit",
        json!({"lut": stats.luts.iter().sum::<usize>(), "ff": stats.ffs.values().sum::<usize>(), "carry4": stats.carry4}),
    ));
    let t3 = std::time::Instant::now();
    let report = Report {
        ands: lw.aig.ands(),
        aig_inputs: lw.aig.inputs.len(),
        depth: mp.depth,
        undriven: lw.undriven,
        carry4: stats.carry4,
        dsp: ds,
        ands_by_op: lw.ands_by_op.clone(),
        carry_by_op: lw.carry_by_op.clone(),
        mem_rd_ff,
        dead_ff,
        dead_chain,
        stats,
        bram: bs,
        srl: ss,
        ffmerge: fm,
        iopad: io,
        dfeed: df,
        balance: bal,
        muxpack: mc,
        passes: flow.records,
        lower_ms: (t1 - t0).as_millis() as u64,
        map_ms: (t2 - t1).as_millis() as u64,
        emit_ms: (t3 - t2).as_millis() as u64,
    };
    Ok(Synth {
        netlist: phys.into_module(),
        check: check.into_module(),
        report,
    })
}

fn phase(name: &str, begin: bool) -> Event {
    Event::Phase {
        phase: name.into(),
        state: if begin {
            oxibit_report::PhaseState::Begin
        } else {
            oxibit_report::PhaseState::End
        },
        ms: None,
    }
}
