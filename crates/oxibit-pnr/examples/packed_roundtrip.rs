//! Check `packed::Design` against a placed design: reading it and writing
//! it back gives the same JSON, and every cell types and locates.
//!
//!   packed_roundtrip PLACED.json...
//!
//! stdout: one JSON line per design. Exit 0 when every round trip is exact.

use std::collections::BTreeMap;

use oxibit_pnr::packed::{Design, Kind};
use serde_json::{Value, json};

fn main() {
    let mut ok = true;
    for path in std::env::args().skip(1) {
        let v: Value = match std::fs::read_to_string(&path)
            .map_err(|e| e.to_string())
            .and_then(|t| serde_json::from_str(&t).map_err(|e| e.to_string()))
        {
            Ok(v) => v,
            Err(e) => {
                println!("{}", json!({"design": path, "error": e}));
                ok = false;
                continue;
            }
        };
        let d = match Design::from_json(&v) {
            Ok(d) => d,
            Err(e) => {
                println!("{}", json!({"design": path, "error": e}));
                ok = false;
                continue;
            }
        };
        let same = d.to_json() == v;
        ok &= same;
        let mut kinds: BTreeMap<String, usize> = BTreeMap::new();
        let mut other_types: BTreeMap<String, usize> = BTreeMap::new();
        let (mut grid, mut site, mut unplaced) = (0usize, 0usize, 0usize);
        for c in d.cells.values() {
            *kinds.entry(format!("{:?}", c.kind())).or_default() += 1;
            if c.kind() == Kind::Other {
                *other_types.entry(c.ty.clone()).or_default() += 1;
            }
            match &c.loc {
                Some(oxibit_pnr::packed::Loc::Grid { .. }) => grid += 1,
                Some(oxibit_pnr::packed::Loc::Site { .. }) => site += 1,
                None => unplaced += 1,
            }
        }
        println!(
            "{}",
            json!({"design": path, "round_trip_exact": same, "cells": d.cells.len(), "kinds": kinds,
                   "other_types": other_types, "loc": {"grid": grid, "site": site, "none": unplaced}})
        );
    }
    std::process::exit(if ok { 0 } else { 1 });
}
