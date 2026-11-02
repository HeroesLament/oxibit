//! Interconnect delay of a routed net, by nextpnr-himbaechel's model
//! (`Arch::getPipDelay` with the loads `bindPip` records), so it can be
//! checked against nextpnr's own report and then used to compare and drive
//! routing.
//!
//! For a pip from node `s` to node `d`, in ps:
//!
//! ```text
//! (drive_res(s) + R(s)/2) * load_cap(s) / 1e6       input RC
//! + delay                                           intrinsic, slow corner
//! + (out_res + R(d)/2) * C(d) / 1e6                 output RC
//! ```
//!
//! `load_cap(s)` is the input capacitance of every pip the net uses out of
//! `s`; `drive_res(d)` is the pip's output resistance, plus what drives its
//! input unless the pip is buffered. `R`, `C` are a node's lumped wire RC
//! (`Graph::node_rc`), absent for nodes nextpnr gives no timing class. An
//! arc's delay is the sum over the pips from the source to the sink.
//!
//! nextpnr routes and reports timing with `fast_pip_delays` on
//! (`Arch::route`), which drops the input RC term ([`Model::Fast`]).
//! Against tobs12's report, the intrinsic delays alone ([`Model::Intrinsic`])
//! give every critical-path routing arc to within 0..17 ps after its site
//! pips (20 ps each, which this does not count: the trees here run between
//! site pins); the output RC term as computed here overshoots on long
//! nodes, whose timing classes nextpnr builds differently.

use std::collections::HashMap;

use oxibit_fasm::route::Graph;

/// One routed edge: (src node, dst node, tile, type pip index).
pub type TreeEdge = (u32, u32, u32, u32);

/// Which terms a pip's delay has.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Model {
    /// The pip's intrinsic delay only.
    Intrinsic,
    /// Intrinsic and output RC (nextpnr's `fast_pip_delays`).
    Fast,
    /// Intrinsic, input and output RC.
    Full,
}

/// The delay (ps) from `root` to every node of the tree its `edges` form.
pub fn arrivals(g: &Graph, root: u32, edges: &[TreeEdge]) -> HashMap<u32, u64> {
    arrivals_with(g, root, edges, Model::Full)
}

/// `arrivals` under `model`.
pub fn arrivals_with(g: &Graph, root: u32, edges: &[TreeEdge], model: Model) -> HashMap<u32, u64> {
    let mut out_of: HashMap<u32, Vec<TreeEdge>> = HashMap::new();
    let mut load_cap: HashMap<u32, u64> = HashMap::new();
    for &e in edges {
        out_of.entry(e.0).or_default().push(e);
        *load_cap.entry(e.0).or_default() += g.edge_timing(e.2, e.3).in_cap as u64;
    }
    let mut drive_res: HashMap<u32, u64> = HashMap::new();
    let mut at: HashMap<u32, u64> = HashMap::from([(root, 0)]);
    let mut stack = vec![root];
    while let Some(s) = stack.pop() {
        let Some(es) = out_of.get(&s) else { continue };
        for &(_, d, tile, pi) in es {
            if at.contains_key(&d) {
                continue;
            }
            let t = g.edge_timing(tile, pi);
            let src_drive = drive_res.get(&s).copied();
            let mut input_res = src_drive.unwrap_or(0);
            if let Some(rc) = g.node_rc(s) {
                input_res += rc.res as u64 / 2;
            }
            let mut delay = t.delay as u64;
            if model == Model::Full {
                delay += input_res * load_cap.get(&s).copied().unwrap_or(0) / 1_000_000;
            }
            if let (Model::Fast | Model::Full, Some(rc)) = (model, g.node_rc(d)) {
                delay += (t.out_res as u64 + rc.res as u64 / 2) * rc.cap as u64 / 1_000_000;
            }
            let upstream = if t.buffered { 0 } else { src_drive.unwrap_or(0) };
            drive_res.insert(d, upstream + t.out_res as u64);
            at.insert(d, at[&s] + delay);
            stack.push(d);
        }
    }
    at
}
