//! Synthesis: the word-level netlist from `oxibit-hdl` to an xc7 netlist.
//!
//! `lower` bit-blasts combinational cells into an AIG between cut points
//! (ports, flops, memories, primitive instances) and puts flop enables and
//! synchronous resets on CE / R / S; `map` covers the AIG with 6-input
//! LUTs; `emit` builds the result as a module of primitive instances,
//! which `oxibit_hdl::yosys_json` writes. Every cell synthesis creates is
//! named `oxsyn_*`, so a checker can tell them from the design's own
//! primitive instances.

pub mod aig;
pub mod balance;
pub mod bram;
pub mod dsp;
pub mod emit;
pub mod ffmerge;
pub mod lib_ports;
pub mod lower;
pub mod map;
pub mod memdff;
pub mod muxchain;
pub mod srl;

use oxibit_hdl::netlist::Module;
use oxibit_report::{Event, Progress};
use serde_json::json;

pub struct Options {
    pub area_passes: usize,
    /// Adders / comparators at least this wide use CARRY4 (0: never).
    pub carry_min: usize,
    /// Map multipliers that fit to DSP48E1.
    pub dsp: bool,
    /// Map memories to RAMB18E1 / RAMB36E1.
    pub bram: bool,
    /// Map flop shift registers to SRLC32E.
    pub srl: bool,
    /// Rebalance AND / XOR supergates before mapping.
    pub balance: bool,
    /// Priority mux chains over one selector to parallel form.
    pub muxpack: bool,
}

impl Default for Options {
    fn default() -> Self {
        Options {
            area_passes: 2,
            carry_min: 3,
            dsp: true,
            bram: true,
            srl: true,
            balance: true,
            muxpack: true,
        }
    }
}

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
    pub lower_ms: u64,
    pub map_ms: u64,
    pub emit_ms: u64,
}

/// Synthesize `m`. `dirs` gives the port directions of the primitives the
/// design instantiates. Returns the netlist, the checking view (the same
/// netlist with memories still `$mem_v2`, shift registers and constant or
/// duplicate flops still flops,
/// for the equivalence oracle) and a
/// report. Phases and per-pass counters go to `rep` (oxibit-report events).
pub fn synth(m: &Module, dirs: &lib_ports::PortDirs, opt: &Options, rep: &dyn Progress) -> Result<(Module, Module, Report), String> {
    let t0 = std::time::Instant::now();
    let mut m = m.clone();
    rep.emit(phase("prepare", true));
    let mem_rd_ff = memdff::merge(&mut m);
    let mc = if opt.muxpack {
        muxchain::pack(&mut m)
    } else {
        muxchain::MuxChainStats::default()
    };
    let ds = if opt.dsp { dsp::infer(&mut m, 4) } else { dsp::DspStats::default() };
    rep.emit(phase("prepare", false));
    rep.emit(Event::progress(
        "prepare",
        json!({"mem_rd_ff": mem_rd_ff, "dsp": ds.dsp, "mul_left": ds.mul_left, "mux_chains": mc.chains, "mux_arms": mc.arms}),
    ));
    rep.emit(phase("lower", true));
    let mut lw = lower::lower(&m, dirs, opt.carry_min)?;
    let (dead_ff, dead_chain) = lw.sweep();
    let (ands_pre_balance, _) = if opt.balance { balance::balance(&mut lw) } else { (lw.aig.ands(), 0) };
    rep.emit(phase("lower", false));
    rep.emit(Event::progress(
        "lower",
        json!({"ands": lw.aig.ands(), "aig_inputs": lw.aig.inputs.len(), "flops": lw.ffs.len(), "carry_chains": lw.aig.chains.len(),
               "dead_ff": dead_ff, "dead_chain": dead_chain, "ands_pre_balance": ands_pre_balance}),
    ));
    let t1 = std::time::Instant::now();
    let outs = lw.outputs();
    rep.emit(phase("map", true));
    let mp = map::map(&lw.aig, &outs, opt.area_passes, rep);
    rep.emit(phase("map", false));
    let t2 = std::time::Instant::now();
    rep.emit(phase("emit", true));
    let (mut out, stats) = emit::emit(&lw, &mp);
    let check = out.clone();
    let fm = ffmerge::merge(&mut out);
    let ss = if opt.srl { srl::map(&mut out) } else { srl::SrlStats::default() };
    // Flop counts are of the final netlist.
    let mut stats = stats;
    for (ty, n) in ss.by_type.iter().chain(&fm.by_type) {
        if let Some(c) = stats.ffs.get_mut(ty) {
            *c -= n;
        }
    }
    let bs = if opt.bram { bram::map(&mut out) } else { bram::BramStats::default() };
    rep.emit(phase("emit", false));
    rep.emit(Event::progress(
        "emit",
        json!({"lut": stats.luts.iter().sum::<usize>(), "ff": stats.ffs.values().sum::<usize>(), "carry4": stats.carry4,
               "ramb18": bs.ramb18, "ramb36": bs.ramb36, "lutram": bs.ram32m + bs.ram64m + bs.ram128x1d, "mem_left": bs.left.len(),
               "srlc32e": ss.srlc32e, "srl_ffs": ss.ffs,
               "ff_constant": fm.constant, "ff_duplicate": fm.duplicate}),
    ));
    let t3 = std::time::Instant::now();
    let r = Report {
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
        lower_ms: (t1 - t0).as_millis() as u64,
        map_ms: (t2 - t1).as_millis() as u64,
        emit_ms: (t3 - t2).as_millis() as u64,
    };
    Ok((out, check, r))
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
