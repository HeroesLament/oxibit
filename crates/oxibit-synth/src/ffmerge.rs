//! Constant and duplicate flops (yosys `opt_dff` / `opt_merge` on flops),
//! on the emitted netlist.
//!
//! A synthesis flop (`oxsyn_*` FDRE / FDSE / FDCE / FDPE) that can never
//! leave its INIT value (D is INIT or its own Q, or CE is tied low, and
//! any reset drives INIT too) is replaced by that constant. A synthesis
//! flop identical to another (same type, parameters and C / CE / D / reset
//! nets) is replaced by the other's Q. Repeats until nothing changes,
//! since a merge can make two downstream flops identical. Runs after the
//! checking view is taken: the check keeps every flop (register
//! correspondence by name), and both replacements hold by inspection.

use std::collections::{BTreeMap, HashMap};

use oxibit_hdl::netlist::{Bit, Cell, Module, Param, SigSpec};
use oxibit_prof::hot;

#[derive(Debug, Default, Clone)]
pub struct FfMergeStats {
    pub constant: usize,
    pub duplicate: usize,
    pub by_type: BTreeMap<&'static str, usize>,
}

fn ty_static(t: &str) -> Option<&'static str> {
    match t {
        "FDRE" => Some("FDRE"),
        "FDSE" => Some("FDSE"),
        "FDCE" => Some("FDCE"),
        "FDPE" => Some("FDPE"),
        _ => None,
    }
}

fn bit1(conns: &BTreeMap<String, SigSpec>, p: &str) -> Option<Bit> {
    match conns.get(p) {
        Some(s) if s.0.len() == 1 => Some(s.0[0]),
        _ => None,
    }
}

fn flag(params: &BTreeMap<String, Param>, p: &str) -> bool {
    match params.get(p) {
        Some(Param::Bits(b)) => b.first().copied().flatten().unwrap_or(false),
        Some(Param::Int(v)) => *v != 0,
        _ => false,
    }
}

/// Replacement for the flop's Q when it is stuck at INIT.
fn stuck(ty: &str, params: &BTreeMap<String, Param>, conns: &BTreeMap<String, SigSpec>) -> Option<Bit> {
    let (srp, inv, resetval) = match ty {
        "FDRE" => ("R", "IS_R_INVERTED", false),
        "FDSE" => ("S", "IS_S_INVERTED", true),
        "FDCE" => ("CLR", "IS_CLR_INVERTED", false),
        "FDPE" => ("PRE", "IS_PRE_INVERTED", true),
        _ => return None,
    };
    if flag(params, "IS_D_INVERTED") {
        return None;
    }
    let init = match params.get("INIT") {
        Some(Param::Bits(b)) if b.len() == 1 => b[0]?,
        Some(Param::Int(v)) => *v != 0,
        None => resetval,
        _ => return None,
    };
    let initb = if init { Bit::One } else { Bit::Zero };
    let (d, q, ce, sr) = (bit1(conns, "D")?, bit1(conns, "Q")?, bit1(conns, "CE")?, bit1(conns, srp)?);
    let sr_idle = matches!((sr, flag(params, inv)), (Bit::Zero, false) | (Bit::One, true));
    let reset_keeps = sr_idle || resetval == init;
    let holds = d == initb || d == q || (ce == Bit::Zero && !flag(params, "IS_CE_INVERTED"));
    (holds && reset_keeps).then_some(initb)
}

/// Identity of a flop for merging: type, parameters, every input.
fn key(ty: &str, params: &BTreeMap<String, Param>, conns: &BTreeMap<String, SigSpec>) -> String {
    let mut k = String::from(ty);
    for (p, v) in params {
        k.push_str(&format!("|{p}={v:?}"));
    }
    for (p, v) in conns {
        if p != "Q" {
            k.push_str(&format!("|{p}={:?}", v.0));
        }
    }
    k
}

fn rewrite(s: &mut SigSpec, map: &HashMap<Bit, Bit>) {
    for b in s.0.iter_mut() {
        if let Some(t) = map.get(b) {
            *b = *t;
        }
    }
}

fn rewrite_cell(c: &mut Cell, map: &HashMap<Bit, Bit>) {
    match c {
        Cell::Unary { a, .. } => rewrite(a, map),
        Cell::Binary { a, b, .. } => {
            rewrite(a, map);
            rewrite(b, map);
        }
        Cell::Mux { a, b, s, .. } => {
            rewrite(a, map);
            rewrite(b, map);
            rewrite(s, map);
        }
        Cell::Dff { clk, d, .. } => {
            rewrite(clk, map);
            rewrite(d, map);
        }
        Cell::Adff { clk, arst, d, .. } => {
            rewrite(clk, map);
            rewrite(arst, map);
            rewrite(d, map);
        }
        Cell::Mem(mem) => {
            for r in &mut mem.reads {
                rewrite(&mut r.addr, map);
                if let Some((c, _)) = &mut r.clk {
                    rewrite(c, map);
                }
            }
            for w in &mut mem.writes {
                rewrite(&mut w.clk.0, map);
                rewrite(&mut w.addr, map);
                rewrite(&mut w.data, map);
                rewrite(&mut w.en, map);
            }
        }
        Cell::Instance { conns, .. } => {
            // Outputs are never in the map (only removed flops' Qs are).
            for s in conns.values_mut() {
                rewrite(s, map);
            }
        }
    }
}

#[hot]
pub fn merge(m: &mut Module) -> FfMergeStats {
    let mut st = FfMergeStats::default();
    loop {
        let mut map: HashMap<Bit, Bit> = HashMap::new();
        let mut drop = vec![false; m.cells.len()];
        let mut seen: HashMap<String, Bit> = HashMap::new();
        for (ci, c) in m.cells.iter().enumerate() {
            let Cell::Instance { ty, name, params, conns } = c else { continue };
            let Some(t) = ty_static(ty) else { continue };
            if !name.starts_with("oxsyn_") {
                continue;
            }
            let Some(q) = bit1(conns, "Q") else { continue };
            if let Some(v) = stuck(ty, params, conns) {
                map.insert(q, v);
                drop[ci] = true;
                st.constant += 1;
                *st.by_type.entry(t).or_default() += 1;
                continue;
            }
            match seen.get(&key(ty, params, conns)) {
                Some(&q0) => {
                    map.insert(q, q0);
                    drop[ci] = true;
                    st.duplicate += 1;
                    *st.by_type.entry(t).or_default() += 1;
                }
                None => {
                    seen.insert(key(ty, params, conns), q);
                }
            }
        }
        if map.is_empty() {
            return st;
        }
        let mut k = 0;
        m.cells.retain(|_| {
            let keep = !drop[k];
            k += 1;
            keep
        });
        for c in &mut m.cells {
            rewrite_cell(c, &map);
        }
        // Names and port connections read the replacement too.
        for (_, r) in &mut m.connects {
            rewrite(r, &map);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxibit_hdl::netlist::{Dir, Signal};

    fn ff(m: &mut Module, name: &str, d: Bit, q: Bit, init: bool) {
        m.cells.push(Cell::Instance {
            ty: "FDRE".into(),
            name: format!("oxsyn_{name}"),
            params: BTreeMap::from([("INIT".to_string(), Param::Bits(vec![Some(init)]))]),
            conns: BTreeMap::from([
                ("C".to_string(), SigSpec(vec![Bit::One])),
                ("CE".to_string(), SigSpec(vec![Bit::One])),
                ("R".to_string(), SigSpec(vec![Bit::Zero])),
                ("D".to_string(), SigSpec(vec![d])),
                ("Q".to_string(), SigSpec(vec![q])),
            ]),
        });
    }

    #[test]
    fn constant_and_duplicates() {
        let mut m = Module::new("t");
        let id = m.add_signal(Signal {
            name: "d".into(),
            width: 1,
            signed: false,
            port: Some(Dir::Input),
            init: None,
        });
        let din = SigSpec::of(id, 1).0[0];
        let w = m.wire(5);
        // a and b identical; c reads a, e reads b: identical after the
        // first merge. k is stuck at its INIT (D = INIT).
        ff(&mut m, "a", din, w.0[0], false);
        ff(&mut m, "b", din, w.0[1], false);
        ff(&mut m, "c", w.0[0], w.0[2], false);
        ff(&mut m, "e", w.0[1], w.0[3], false);
        ff(&mut m, "k", Bit::One, w.0[4], true);
        let st = merge(&mut m);
        assert_eq!((st.constant, st.duplicate), (1, 2));
        assert_eq!(m.cells.len(), 2);
    }
}
