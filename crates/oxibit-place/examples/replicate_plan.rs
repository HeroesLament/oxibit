//! A replication plan (`oxibit_place::replicate`) from a routed design's
//! timed arcs, as `sta` writes them with STA_ARCS.
//!
//! cargo run --release -p oxibit-place --example replicate_plan -- \
//!     ARCS.tsv [KEY=VALUE ...] > plan.tsv
//!
//! Keys, each defaulting to `PlanOpts::default`: `fanout` (fewest sinks),
//! `mean_ps` (least mean routed delay to them), `span` (tiles a group may
//! cover), `groups` (most groups per cell), `budget` (most copies in all).
//! `slack_ps=N` chooses by slack instead: a copy per group of a cell's sinks
//! whose arcs have slack under N ps and are routed `arc_ps` or longer (ARCS
//! from `sta` run with STA_PERIODS); the original keeps the other sinks.
//! `pack` applies the plan with OXIBIT_REPLICATE=plan.tsv.
//!
//! stderr: one JSON line of counts. Same arcs and keys, same plan.

use oxibit_place::replicate::{PlanOpts, plan};
use serde_json::json;

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).ok_or("usage: replicate_plan ARCS.tsv [KEY=VALUE ...]")?;
    let mut o = PlanOpts::default();
    for kv in &args[2..] {
        let (k, v) = kv.split_once('=').ok_or(format!("{kv} is not KEY=VALUE"))?;
        let n: f64 = v.parse().map_err(|_| format!("{v} is not a number"))?;
        match k {
            "fanout" => o.min_fanout = n as usize,
            "mean_ps" => o.min_mean_ps = n,
            "span" => o.span = n as i64,
            "groups" => o.max_groups = n as usize,
            "budget" => o.budget = n as usize,
            "slack_ps" => o.slack_ps = Some(n as i64),
            "arc_ps" => o.min_arc_ps = n as i64,
            other => return Err(format!("unknown key {other}")),
        }
    }
    let arcs = std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?;
    let (text, st) = plan(&arcs, &o)?;
    print!("# replicate_plan from {path}: fanout {} mean_ps {} span {} groups {} budget {}\n{text}", o.min_fanout, o.min_mean_ps, o.span, o.max_groups, o.budget);
    eprintln!(
        "{}",
        json!({"replicate_plan": {"drivers": st.drivers, "candidates": st.candidates, "copied": st.copied, "copies": st.copies, "sinks_moved": st.sinks_moved}})
    );
    Ok(())
}
