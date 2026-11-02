//! Compare two packed designs net for net, ignoring placement and net
//! numbering: oxibit's packing against nextpnr-xilinx's.
//!
//! A net is named by what drives it: `cell.port`, `GND` / `VCC` for the
//! pseudo drivers. Cells the two sides name differently (feed-through LUTs
//! numbered by a counter) are anonymous: named by type and inputs, and
//! compared as a multiset. A LUT's inputs compare by logical port
//! (`X_ORIG_PORT_A<n>`), since the pin a placer or router gives each input
//! is free.

use std::collections::{BTreeMap, HashMap, HashSet};

use oxibit_pnr::packed::{Bit, Cell, Design};
use serde_json::{Value, json};

/// Differences by category, with a few examples each.
#[derive(Debug, Default)]
pub struct Diff {
    pub counts: BTreeMap<String, usize>,
    pub examples: BTreeMap<String, Vec<String>>,
}

impl Diff {
    fn add(&mut self, k: &str, ex: String) {
        *self.counts.entry(k.to_string()).or_default() += 1;
        let v = self.examples.entry(k.to_string()).or_default();
        if v.len() < 5 {
            v.push(ex);
        }
    }

    pub fn to_json(&self) -> Value {
        json!({"counts": self.counts, "examples": self.examples})
    }

    pub fn total(&self) -> usize {
        self.counts
            .iter()
            .filter(|(k, _)| !k.starts_with("ok.") && !k.starts_with("after_place_ties."))
            .map(|(_, v)| v)
            .sum()
    }
}

struct Side<'a> {
    d: &'a Design,
    driver: HashMap<i64, (&'a str, &'a str)>,
    shared: &'a HashSet<String>,
    memo: std::cell::RefCell<HashMap<i64, String>>,
}

impl<'a> Side<'a> {
    fn new(d: &'a Design, shared: &'a HashSet<String>) -> Side<'a> {
        let mut driver = HashMap::new();
        for (name, c) in &d.cells {
            for (p, port) in &c.ports {
                if port.dir == Some(oxibit_pnr::packed::Dir::Output) {
                    for b in port.bits.iter().flatten() {
                        if let Bit::Net(n) = b {
                            driver.insert(*n, (name.as_str(), p.as_str()));
                        }
                    }
                }
            }
        }
        Side {
            d,
            driver,
            shared,
            memo: Default::default(),
        }
    }

    fn canon(&self, b: Bit, depth: usize) -> String {
        let n = match b {
            Bit::Net(n) => n,
            Bit::Zero => return "GND".into(),
            Bit::One => return "VCC".into(),
            _ => return "x".into(),
        };
        if let Some(s) = self.memo.borrow().get(&n) {
            return s.clone();
        }
        let s = match self.driver.get(&n) {
            None => "undriven".to_string(),
            Some((cell, port)) => {
                let c = &self.d.cells[*cell];
                // A LUT's output pin is its bel's: O5 on a 5LUT, O6 on a 6LUT.
                let port = if c.ty == "SLICE_LUTX" && (*port == "O5" || *port == "O6") {
                    "O"
                } else {
                    *port
                };
                match c.ty.as_str() {
                    "PSEUDO_GND" => "GND".into(),
                    "PSEUDO_VCC" => "VCC".into(),
                    _ if self.shared.contains(*cell) || depth > 6 => format!("{cell}.{}", port.replace(['[', ']'], "")),
                    _ => format!("{}<{}>.{}", c.ty, self.inputs(c, depth + 1).join(","), port.replace(['[', ']'], "")),
                }
            }
        };
        self.memo.borrow_mut().insert(n, s.clone());
        s
    }

    /// A cell's inputs as `pin=net`, LUT inputs by logical port.
    fn inputs(&self, c: &Cell, depth: usize) -> Vec<String> {
        let lut = c.ty == "SLICE_LUTX" && !c.attrs.contains_key("X_LUT_AS_SRL") && !c.attrs.contains_key("X_LUT_AS_DRAM");
        let mut v: Vec<String> = Vec::new();
        for (p, port) in &c.ports {
            if port.dir == Some(oxibit_pnr::packed::Dir::Output) {
                continue;
            }
            let Some(bits) = &port.bits else { continue };
            if bits.is_empty() {
                continue;
            }
            let keys: Vec<String> = if lut && p.starts_with('A') {
                match c.orig_port(p) {
                    // After placement one pin can carry several logical
                    // inputs on the same net ("I0 I1").
                    Some(o) => o.split_whitespace().map(str::to_string).collect(),
                    // A pin the placer tied (A6 high on a small LUT).
                    None => continue,
                }
            } else {
                vec![p.clone()]
            };
            let nets: Vec<String> = bits.iter().map(|b| self.canon(*b, depth)).collect();
            for key in keys {
                v.push(format!("{key}={}", nets.join("+")));
            }
        }
        v.sort();
        v
    }
}

fn norm_param(v: &Value) -> String {
    if let Some(s) = v.as_str() {
        let t = s.trim();
        if !t.is_empty() && t.chars().all(|c| c == '0' || c == '1') {
            // Binary: compare by value (widths differ between writers).
            let t = t.trim_start_matches('0');
            return if t.is_empty() { "0".into() } else { format!("b{t}") };
        }
        return t.to_string();
    }
    if let Some(n) = v.as_i64() {
        return if n == 0 { "0".into() } else { format!("b{n:b}") };
    }
    v.to_string()
}

/// Named by a packer's running counter (`$LUT42`, `$MUX$7`): the same name
/// on both sides is a coincidence, not the same cell.
fn counter_named(name: &str) -> bool {
    let tail = |marker: &str| {
        name.rfind(marker).is_some_and(|i| {
            let rest = &name[i + marker.len()..];
            !rest.is_empty() && rest.chars().all(|c| c.is_ascii_digit())
        })
    };
    tail("$LUT") || tail("$MUX$")
}

const IGNORED_ATTRS: &[&str] = &["NEXTPNR_BEL", "X_IO_BEL", "BEL_STRENGTH", "src", "hdlname", "module_not_derived", "keep"];

/// Compare `ours` against `theirs`.
pub fn compare(ours: &Design, theirs: &Design) -> Diff {
    let mut diff = Diff::default();
    let shared: HashSet<String> = ours
        .cells
        .keys()
        .filter(|k| theirs.cells.contains_key(*k) && !counter_named(k))
        .cloned()
        .collect();
    let a = Side::new(ours, &shared);
    let b = Side::new(theirs, &shared);

    for (name, ca) in &ours.cells {
        let Some(cb) = theirs.cells.get(name) else { continue };
        let ty = format!("{}:{}", ca.ty, ca.orig_type().unwrap_or("-"));
        if ca.ty != cb.ty {
            diff.add("type", format!("{name}: {} vs {}", ca.ty, cb.ty));
            continue;
        }
        let mut bad = false;
        for k in ca.params.keys().chain(cb.params.keys()).collect::<std::collections::BTreeSet<_>>() {
            let (x, y) = (ca.params.get(k).map(norm_param), cb.params.get(k).map(norm_param));
            if x != y {
                diff.add(&format!("param.{ty}.{k}"), format!("{name}: {x:?} vs {y:?}"));
                bad = true;
            }
        }
        for k in ca.attrs.keys().chain(cb.attrs.keys()).collect::<std::collections::BTreeSet<_>>() {
            if IGNORED_ATTRS.contains(&k.as_str()) || k.starts_with("X_ORIG_PORT_") {
                continue;
            }
            // Pin lists compare as sets.
            let norm = |v: &Value| {
                if k.ends_with("_PINS") {
                    let mut t: Vec<&str> = v.as_str().unwrap_or("").split_whitespace().collect();
                    t.sort();
                    t.join(" ")
                } else {
                    norm_param(v)
                }
            };
            let (x, y) = (ca.attrs.get(k).map(norm), cb.attrs.get(k).map(norm));
            if x != y {
                diff.add(&format!("attr.{ty}.{k}"), format!("{name}: {x:?} vs {y:?}"));
                bad = true;
            }
        }
        let (ia, ib) = (a.inputs(ca, 0), b.inputs(cb, 0));
        if ia != ib {
            let sa: HashSet<&String> = ia.iter().collect();
            let sb: HashSet<&String> = ib.iter().collect();
            let mut only_a: Vec<&&String> = sa.difference(&sb).collect();
            let mut only_b: Vec<&&String> = sb.difference(&sa).collect();
            only_a.sort();
            only_b.sort();
            // Constant ties nextpnr adds after placement (`fixup_placement`:
            // A6 high beside a used 5LUT, unused PS7 inputs low) are the
            // placer's to add, not the packer's.
            let fixup = only_a.is_empty() && only_b.iter().all(|e| e.ends_with("=VCC") || e.ends_with("=GND"));
            let cat = if fixup { "after_place_ties" } else { "inputs" };
            diff.add(&format!("{cat}.{ty}"), format!("{name}: ours {only_a:?} theirs {only_b:?}"));
            bad = true;
        }
        if !bad {
            diff.add(&format!("ok.{ty}"), String::new());
        }
    }
    // Anonymous cells as a multiset of (type, orig type, inputs).
    let anon = |s: &Side, d: &Design| -> BTreeMap<String, (usize, String)> {
        let mut m: BTreeMap<String, (usize, String)> = BTreeMap::new();
        for (name, c) in &d.cells {
            if shared.contains(name) {
                continue;
            }
            let key = format!("{}:{} <{}>", c.ty, c.orig_type().unwrap_or("-"), s.inputs(c, 0).join(","));
            let e = m.entry(key).or_insert((0, name.clone()));
            e.0 += 1;
        }
        m
    };
    let (ma, mb) = (anon(&a, ours), anon(&b, theirs));
    for (k, (n, ex)) in &ma {
        let m = mb.get(k).map_or(0, |x| x.0);
        let ty = k.split(' ').next().unwrap_or("");
        if *n > m {
            for _ in 0..(*n - m) {
                diff.add(&format!("only_ours.{ty}"), format!("{ex}: {k}"));
            }
        }
        if m > 0 {
            for _ in 0..(*n).min(m) {
                diff.add(&format!("ok.anon.{ty}"), String::new());
            }
        }
    }
    for (k, (n, ex)) in &mb {
        let m = ma.get(k).map_or(0, |x| x.0);
        let ty = k.split(' ').next().unwrap_or("");
        if *n > m {
            for _ in 0..(*n - m) {
                diff.add(&format!("only_theirs.{ty}"), format!("{ex}: {k}"));
            }
        }
    }
    // Examples of "ok" carry no text: drop them.
    diff.examples.retain(|k, _| !k.starts_with("ok."));
    diff
}
