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
pub mod dsp;
pub mod emit;
pub mod lib_ports;
pub mod lower;
pub mod map;
pub mod memdff;

use oxibit_hdl::netlist::Module;

pub struct Options {
    pub area_passes: usize,
    /// Adders / comparators at least this wide use CARRY4 (0: never).
    pub carry_min: usize,
    /// Map multipliers that fit to DSP48E1.
    pub dsp: bool,
}

impl Default for Options {
    fn default() -> Self {
        Options {
            area_passes: 2,
            carry_min: 3,
            dsp: true,
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
    pub mem_rd_ff: usize,
    pub dead_ff: usize,
    pub dead_chain: usize,
    pub stats: emit::Stats,
    pub lower_ms: u64,
    pub map_ms: u64,
    pub emit_ms: u64,
}

/// Synthesize `m`. `dirs` gives the port directions of the primitives the
/// design instantiates.
pub fn synth(m: &Module, dirs: &lib_ports::PortDirs, opt: &Options) -> Result<(Module, Report), String> {
    let t0 = std::time::Instant::now();
    let mut m = m.clone();
    let mem_rd_ff = memdff::merge(&mut m);
    let ds = if opt.dsp { dsp::infer(&mut m, 4) } else { dsp::DspStats::default() };
    let mut lw = lower::lower(&m, dirs, opt.carry_min)?;
    let (dead_ff, dead_chain) = lw.sweep();
    let t1 = std::time::Instant::now();
    let outs = lw.outputs();
    let mp = map::map(&lw.aig, &outs, opt.area_passes);
    let t2 = std::time::Instant::now();
    let (out, stats) = emit::emit(&lw, &mp);
    let t3 = std::time::Instant::now();
    let r = Report {
        ands: lw.aig.ands(),
        aig_inputs: lw.aig.inputs.len(),
        depth: mp.depth,
        undriven: lw.undriven,
        carry4: stats.carry4,
        dsp: ds,
        ands_by_op: lw.ands_by_op.clone(),
        mem_rd_ff,
        dead_ff,
        dead_chain,
        stats,
        lower_ms: (t1 - t0).as_millis() as u64,
        map_ms: (t2 - t1).as_millis() as u64,
        emit_ms: (t3 - t2).as_millis() as u64,
    };
    Ok((out, r))
}
