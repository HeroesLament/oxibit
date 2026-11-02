//! Region constraints: groups of cells, found by name, held to a box of
//! the die.
//!
//! A rule names a hierarchy prefix (`pb_eng0_`) and a box of tiles. Cells
//! whose name holds the prefix are seeds. Synthesis leaves most LUTs
//! unnamed, so the labels then spread over short nets: an unnamed cell
//! takes the label its neighbours mostly carry, and one pulled both ways
//! stays free. A placement object (a slice, a chain, a hard block) is held
//! to a region when most of its labelled cells agree; the rest, the logic
//! the groups share, goes anywhere.
//!
//! This is a constraint, not a cost: the annealer never moves a held
//! object out of its box, and a box too small for what it holds fails the
//! run before placement starts. It never changes what a slice holds, so it
//! needs no capability level.

use std::collections::{HashMap, HashSet};

use serde_json::{Value, json};

use crate::netlist::{CellId, NetId};
use crate::pack::Packer;

/// A box of tiles, bounds included.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Region {
    pub x0: i32,
    pub x1: i32,
    pub y0: i32,
    pub y1: i32,
}

impl Region {
    pub fn contains(&self, x: i32, y: i32) -> bool {
        x >= self.x0 && x <= self.x1 && y >= self.y0 && y <= self.y1
    }
}

/// Cells whose name contains `prefix` belong in `region`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Rule {
    pub prefix: String,
    pub region: Region,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Floorplan {
    pub rules: Vec<Rule>,
}

fn span(s: &str) -> Result<(i32, i32), String> {
    if s == "*" {
        return Ok((i32::MIN, i32::MAX));
    }
    let (a, b) = s
        .split_once('-')
        .ok_or_else(|| format!("{s}: expected LO-HI or *"))?;
    let a: i32 = a.parse().map_err(|_| format!("{s}: bad bound"))?;
    let b: i32 = b.parse().map_err(|_| format!("{s}: bad bound"))?;
    if a > b {
        return Err(format!("{s}: empty span"));
    }
    Ok((a, b))
}

impl Floorplan {
    pub fn is_empty(&self) -> bool {
        self.rules.is_empty()
    }

    /// `PREFIX:X0-X1:Y0-Y1[,...]`, `*` for a whole axis. Empty is no rules.
    pub fn parse(s: &str) -> Result<Floorplan, String> {
        let mut rules = Vec::new();
        for part in s.split(',').map(str::trim).filter(|p| !p.is_empty()) {
            let f: Vec<&str> = part.split(':').collect();
            let [prefix, xs, ys] = f[..] else {
                return Err(format!("{part}: expected PREFIX:X0-X1:Y0-Y1"));
            };
            if prefix.is_empty() {
                return Err(format!("{part}: empty prefix"));
            }
            let (x0, x1) = span(xs)?;
            let (y0, y1) = span(ys)?;
            rules.push(Rule {
                prefix: prefix.to_string(),
                region: Region { x0, x1, y0, y1 },
            });
        }
        if rules.len() > u8::MAX as usize {
            return Err("too many floorplan rules".into());
        }
        Ok(Floorplan { rules })
    }

    pub fn json(&self) -> Value {
        let b = |v: i32| {
            if v == i32::MIN || v == i32::MAX {
                Value::Null
            } else {
                json!(v)
            }
        };
        Value::Array(
            self.rules
                .iter()
                .map(|r| json!({"prefix": r.prefix, "x": [b(r.region.x0), b(r.region.x1)], "y": [b(r.region.y0), b(r.region.y1)]}))
                .collect(),
        )
    }

    /// The rule a name falls under: the first whose prefix it contains.
    pub fn rule_of(&self, name: &str) -> Option<u8> {
        self.rules
            .iter()
            .position(|r| name.contains(&r.prefix))
            .map(|i| i as u8)
    }
}

#[derive(Clone, Debug, Default)]
pub struct RuleStats {
    pub prefix: String,
    /// Cells named under the rule.
    pub named: usize,
    /// Unnamed cells that took the label from their neighbours.
    pub spread: usize,
    /// Objects held, and the slices (and SLICEM slices) they need.
    pub objects: usize,
    pub slices: usize,
    pub slices_m: usize,
    /// Slice sites (and SLICEM sites) in the box.
    pub sites: usize,
    pub sites_m: usize,
}

#[derive(Clone, Debug, Default)]
pub struct FloorplanStats {
    pub rules: Vec<RuleStats>,
    /// Cells left without a label.
    pub cells_free: usize,
    pub rounds: usize,
    /// Objects whose labelled cells disagree, and objects with none.
    pub objects_mixed: usize,
    pub objects_free: usize,
}

impl FloorplanStats {
    pub fn json(&self) -> Value {
        json!({
            "rules": self.rules.iter().map(|r| json!({"prefix": r.prefix, "named": r.named, "spread": r.spread, "objects": r.objects,
                "slices": r.slices, "slices_m": r.slices_m, "sites": r.sites, "sites_m": r.sites_m})).collect::<Vec<_>>(),
            "cells_free": self.cells_free, "rounds": self.rounds, "objects_mixed": self.objects_mixed, "objects_free": self.objects_free,
        })
    }
}

/// Label every cell it can: names first, then `rounds` of spreading over
/// nets of at most `max_fanout` pins. A cell takes a label when its
/// neighbours' votes for it are at least twice those for any other.
pub fn label_cells(
    p: &Packer,
    fp: &Floorplan,
    rounds: usize,
    max_fanout: usize,
) -> (HashMap<CellId, u8>, FloorplanStats) {
    let k = fp.rules.len();
    let mut st = FloorplanStats {
        rules: fp
            .rules
            .iter()
            .map(|r| RuleStats {
                prefix: r.prefix.clone(),
                ..Default::default()
            })
            .collect(),
        ..Default::default()
    };
    let ids = p.nl.cell_ids();
    let index: HashMap<CellId, usize> = ids.iter().enumerate().map(|(i, c)| (*c, i)).collect();
    let mut lab: Vec<Option<u8>> = ids
        .iter()
        .map(|&c| fp.rule_of(&p.nl.cell(c).name))
        .collect();
    for r in lab.iter().flatten() {
        st.rules[*r as usize].named += 1;
    }
    let mut nets: Vec<Vec<usize>> = Vec::new();
    let mut seen: HashSet<NetId> = HashSet::new();
    for &c in &ids {
        for port in p.nl.cell(c).ports.values() {
            let Some(nid) = port.net else { continue };
            if p.nl.is_const(nid) || !seen.insert(nid) {
                continue;
            }
            let net = p.nl.net(nid);
            let mut pins: Vec<usize> = net
                .driver
                .iter()
                .chain(net.users.iter())
                .filter_map(|(pc, _)| index.get(pc).copied())
                .collect();
            pins.sort_unstable();
            pins.dedup();
            if pins.len() >= 2 && pins.len() <= max_fanout {
                nets.push(pins);
            }
        }
    }
    let mut votes = vec![0u32; ids.len() * k];
    let mut cnt = vec![0u32; k];
    for _ in 0..rounds {
        votes.iter_mut().for_each(|v| *v = 0);
        for pins in &nets {
            cnt.iter_mut().for_each(|v| *v = 0);
            for &i in pins {
                if let Some(r) = lab[i] {
                    cnt[r as usize] += 1;
                }
            }
            if cnt.iter().all(|&v| v == 0) {
                continue;
            }
            for &i in pins {
                if lab[i].is_none() {
                    for r in 0..k {
                        votes[i * k + r] += cnt[r];
                    }
                }
            }
        }
        let mut changed = 0;
        for i in 0..ids.len() {
            if lab[i].is_some() {
                continue;
            }
            let v = &votes[i * k..(i + 1) * k];
            let (best, top) = v
                .iter()
                .enumerate()
                .max_by_key(|(_, n)| **n)
                .map(|(r, n)| (r, *n))
                .unwrap_or((0, 0));
            let second = v
                .iter()
                .enumerate()
                .filter(|(r, _)| *r != best)
                .map(|(_, n)| *n)
                .max()
                .unwrap_or(0);
            if top > 0 && top >= 2 * second {
                lab[i] = Some(best as u8);
                st.rules[best].spread += 1;
                changed += 1;
            }
        }
        st.rounds += 1;
        if changed == 0 {
            break;
        }
    }
    st.cells_free = lab.iter().filter(|l| l.is_none()).count();
    let labels = ids
        .iter()
        .zip(&lab)
        .filter_map(|(c, l)| l.map(|r| (*c, r)))
        .collect();
    (labels, st)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_rules() {
        let fp = Floorplan::parse("pb_eng0_:0-103:*, pb_eng1_:104-999:10-20").unwrap();
        assert_eq!(fp.rules.len(), 2);
        assert!(fp.rules[0].region.contains(103, -5));
        assert!(!fp.rules[0].region.contains(104, 0));
        assert!(fp.rules[1].region.contains(104, 10) && !fp.rules[1].region.contains(104, 21));
        assert_eq!(fp.rule_of("oxsyn_ff9$pb_eng1_x[3]"), Some(1));
        assert_eq!(fp.rule_of("oxsyn_42"), None);
        assert!(Floorplan::parse("").unwrap().is_empty());
    }

    #[test]
    fn rejects_bad_rules() {
        assert!(Floorplan::parse("a:1-2").is_err());
        assert!(Floorplan::parse("a:5-2:*").is_err());
        assert!(Floorplan::parse(":1-2:*").is_err());
    }
}
