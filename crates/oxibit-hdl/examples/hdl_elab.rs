//! Parse and elaborate a Verilog file; write the netlist as yosys JSON.
//!
//! cargo run --release -p oxibit-hdl --example hdl_elab -- IN.v OUT.json
//!
//! stdout: one JSON line with parse and elaboration times and cell counts.

use std::collections::BTreeMap;
use std::process::ExitCode;
use std::time::Instant;

use oxibit_hdl::netlist::Cell;
use oxibit_hdl::{elab, parser, yosys_json};

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    let [_, input, output] = &args[..] else {
        eprintln!("usage: hdl_elab IN.v OUT.json");
        return ExitCode::from(3);
    };
    let src = match std::fs::read_to_string(input) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("{input}: {e}");
            return ExitCode::from(3);
        }
    };
    let t0 = Instant::now();
    let mods = match parser::parse(&src) {
        Ok(m) => m,
        Err(e) => {
            eprintln!("parse: {e}");
            return ExitCode::from(1);
        }
    };
    let t_parse = t0.elapsed();
    let t1 = Instant::now();
    let Some(top) = mods.last() else {
        eprintln!("no module");
        return ExitCode::from(1);
    };
    let m = match elab::Elab::module(top) {
        Ok(m) => m,
        Err(e) => {
            eprintln!("elaborate: {e}");
            return ExitCode::from(1);
        }
    };
    let t_elab = t1.elapsed();
    let mut counts: BTreeMap<String, usize> = BTreeMap::new();
    for c in &m.cells {
        let k = match c {
            Cell::Unary { op, .. } | Cell::Binary { op, .. } => op.yosys().to_string(),
            Cell::Mux { .. } => "$mux".into(),
            Cell::Dff { .. } => "$dff".into(),
            Cell::Adff { .. } => "$adff".into(),
            Cell::Mem(_) => "$mem".into(),
            Cell::Instance { ty, .. } => ty.clone(),
        };
        *counts.entry(k).or_default() += 1;
    }
    let t2 = Instant::now();
    let j = yosys_json::write(&m);
    let t_build = t2.elapsed();
    let t3 = Instant::now();
    if let Err(e) = std::fs::write(output, serde_json::to_string(&j).unwrap()) {
        eprintln!("{output}: {e}");
        return ExitCode::from(3);
    }
    let t_write = t3.elapsed();
    println!(
        "{}",
        serde_json::json!({"module": m.name, "signals": m.signals.len(), "cells": m.cells.len(), "connects": m.connects.len(),
            "parse_ms": t_parse.as_millis() as u64, "elab_ms": t_elab.as_millis() as u64,
            "json_build_ms": t_build.as_millis() as u64, "json_write_ms": t_write.as_millis() as u64, "counts": counts})
    );
    ExitCode::SUCCESS
}
