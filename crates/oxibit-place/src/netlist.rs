//! The netlist the packer rewrites: cells with single-bit ports, nets with
//! one driver and their users, both editable in place.
//!
//! Loaded from a yosys-style JSON netlist the way nextpnr's JSON front end
//! reads one: a multi-bit port becomes one port per bit, `NAME[i]`; a 1-bit
//! port keeps its name; the constant bits `0` / `1` connect to the GND / VCC
//! nets; `x` / `z` leave the port unconnected. The constant nets are driven
//! by `PSEUDO_GND` / `PSEUDO_VCC` cells, as the packed design wants them.
//! Top-level ports are kept as a list (`Netlist::ports`) for the I/O packer.

use std::collections::{BTreeMap, HashMap};

use oxibit_pnr::packed::{self, Bit, Design};
use serde_json::{Value, json};

type R<T> = Result<T, String>;

pub use packed::Dir;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct CellId(pub u32);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct NetId(pub u32);

/// A cell port: direction and the net on it, if any.
#[derive(Debug, Clone, PartialEq)]
pub struct Port {
    pub dir: Dir,
    pub net: Option<NetId>,
}

#[derive(Debug, Clone)]
pub struct Cell {
    pub name: String,
    pub ty: String,
    pub params: BTreeMap<String, Value>,
    pub attrs: BTreeMap<String, Value>,
    pub ports: BTreeMap<String, Port>,
    /// A bel fixed by packing (I/O, IOLOGIC) or chosen by placement.
    pub bel: Option<packed::Loc>,
}

impl Cell {
    pub fn net(&self, port: &str) -> Option<NetId> {
        self.ports.get(port).and_then(|p| p.net)
    }

    pub fn str_attr(&self, k: &str) -> Option<&str> {
        self.attrs.get(k).and_then(Value::as_str)
    }

    pub fn str_param(&self, k: &str) -> Option<&str> {
        self.params.get(k).and_then(Value::as_str)
    }

    /// A parameter as an integer: a number, a binary string (yosys writes
    /// integer parameters as 32-bit binary strings) or a decimal string.
    pub fn int_param(&self, k: &str) -> Option<i64> {
        int_value(self.params.get(k)?)
    }
}

/// An integer from a JSON parameter value.
pub fn int_value(v: &Value) -> Option<i64> {
    match v {
        Value::Number(n) => n.as_i64(),
        Value::String(s) => {
            let t = s.trim();
            if t.len() > 1 && t.chars().all(|c| c == '0' || c == '1') {
                i64::from_str_radix(&t[t.len().saturating_sub(63)..], 2).ok()
            } else {
                t.parse().ok()
            }
        }
        _ => None,
    }
}

#[derive(Debug, Clone)]
pub struct Net {
    pub name: String,
    pub driver: Option<(CellId, String)>,
    pub users: Vec<(CellId, String)>,
}

/// A top-level port bit.
#[derive(Debug, Clone)]
pub struct TopPort {
    pub name: String,
    pub dir: Dir,
    pub net: Option<NetId>,
}

#[derive(Debug, Clone)]
pub struct Netlist {
    pub top: String,
    cells: Vec<Option<Cell>>,
    nets: Vec<Option<Net>>,
    cell_by_name: HashMap<String, CellId>,
    net_by_name: HashMap<String, NetId>,
    pub ports: Vec<TopPort>,
    pub gnd: NetId,
    pub vcc: NetId,
    /// The top module's attributes.
    pub attrs: serde_json::Map<String, Value>,
}

impl Netlist {
    pub fn new(top: &str) -> Netlist {
        let mut n = Netlist {
            top: top.to_string(),
            cells: Vec::new(),
            nets: Vec::new(),
            cell_by_name: HashMap::new(),
            net_by_name: HashMap::new(),
            ports: Vec::new(),
            gnd: NetId(0),
            vcc: NetId(0),
            attrs: serde_json::Map::new(),
        };
        n.gnd = n.add_net("$PACKER_GND_NET");
        n.vcc = n.add_net("$PACKER_VCC_NET");
        let g = n.add_cell("$PACKER_GND_DRV", "PSEUDO_GND");
        n.add_port(g, "Y", Dir::Output);
        n.connect(g, "Y", n.gnd);
        let v = n.add_cell("$PACKER_VCC_DRV", "PSEUDO_VCC");
        n.add_port(v, "Y", Dir::Output);
        n.connect(v, "Y", n.vcc);
        n
    }

    /// The top module (attribute `top`, else the one with the most cells) of
    /// a yosys-style JSON netlist.
    pub fn from_json(v: &Value) -> R<Netlist> {
        let mods = v["modules"].as_object().ok_or("no modules")?;
        let (top, m) = mods
            .iter()
            .find(|(_, m)| m["attributes"]["top"].is_string() || m["attributes"]["top"].is_number())
            .or_else(|| mods.iter().max_by_key(|(_, m)| m["cells"].as_object().map_or(0, |c| c.len())))
            .ok_or("no top module")?;
        let mut nl = Netlist::new(top);
        nl.attrs = m["attributes"].as_object().cloned().unwrap_or_default();

        // Net names: a visible name before a hidden one, then the shortest.
        let mut best: HashMap<i64, (bool, usize, String)> = HashMap::new();
        for (name, nn) in m["netnames"].as_object().into_iter().flatten() {
            let hidden = nn["hide_name"].as_i64().unwrap_or(0) != 0;
            let bits = nn["bits"].as_array().cloned().unwrap_or_default();
            for (i, b) in bits.iter().enumerate() {
                let Some(b) = b.as_i64() else { continue };
                let full = if bits.len() == 1 { name.clone() } else { format!("{name}[{i}]") };
                let key = (hidden, full.len(), full);
                match best.get(&b) {
                    Some(k) if (k.0, k.1, &k.2) <= (key.0, key.1, &key.2) => {}
                    _ => {
                        best.insert(b, key);
                    }
                }
            }
        }
        let mut by_bit: HashMap<i64, NetId> = HashMap::new();
        let mut net_of = |nl: &mut Netlist, b: &Value| -> R<Option<NetId>> {
            Ok(match Bit::parse_json(b)? {
                Bit::Zero => Some(nl.gnd),
                Bit::One => Some(nl.vcc),
                Bit::X | Bit::Z => None,
                Bit::Net(b) => Some(match by_bit.get(&b) {
                    Some(&n) => n,
                    None => {
                        let name = best.get(&b).map(|k| k.2.clone()).unwrap_or_else(|| format!("$bit{b}"));
                        let n = nl.add_net_unique(&name);
                        by_bit.insert(b, n);
                        n
                    }
                }),
            })
        };

        for (pname, p) in m["ports"].as_object().into_iter().flatten() {
            let dir = match p["direction"].as_str() {
                Some("input") => Dir::Input,
                Some("output") => Dir::Output,
                _ => Dir::Inout,
            };
            let bits = p["bits"].as_array().cloned().unwrap_or_default();
            for (i, b) in bits.iter().enumerate() {
                let name = if bits.len() == 1 { pname.clone() } else { format!("{pname}[{i}]") };
                let net = net_of(&mut nl, b)?;
                nl.ports.push(TopPort { name, dir, net });
            }
        }

        for (cname, c) in m["cells"].as_object().into_iter().flatten() {
            let ty = c["type"].as_str().ok_or_else(|| format!("cell {cname} has no type"))?;
            let id = nl.add_cell(cname, ty);
            {
                let cell = nl.cell_mut(id);
                cell.params = c["parameters"].as_object().cloned().unwrap_or_default().into_iter().collect();
                cell.attrs = c["attributes"].as_object().cloned().unwrap_or_default().into_iter().collect();
            }
            let dirs = c["port_directions"].as_object().cloned().unwrap_or_default();
            for (p, bits) in c["connections"].as_object().into_iter().flatten() {
                let bits = bits.as_array().cloned().unwrap_or_default();
                let dir = match dirs.get(p).and_then(Value::as_str) {
                    Some("output") => Dir::Output,
                    Some("inout") => Dir::Inout,
                    _ => Dir::Input,
                };
                for (i, b) in bits.iter().enumerate() {
                    let pn = if bits.len() == 1 { p.clone() } else { format!("{p}[{i}]") };
                    nl.add_port(id, &pn, dir);
                    if let Some(n) = net_of(&mut nl, b)? {
                        nl.connect(id, &pn, n);
                    }
                }
            }
        }
        Ok(nl)
    }

    // ---- cells ----

    pub fn add_cell(&mut self, name: &str, ty: &str) -> CellId {
        let id = CellId(self.cells.len() as u32);
        assert!(!self.cell_by_name.contains_key(name), "duplicate cell name {name}");
        self.cells.push(Some(Cell {
            name: name.to_string(),
            ty: ty.to_string(),
            params: BTreeMap::new(),
            attrs: BTreeMap::new(),
            ports: BTreeMap::new(),
            bel: None,
        }));
        self.cell_by_name.insert(name.to_string(), id);
        id
    }

    pub fn has_cell(&self, name: &str) -> bool {
        self.cell_by_name.contains_key(name)
    }

    pub fn cell_id(&self, name: &str) -> Option<CellId> {
        self.cell_by_name.get(name).copied()
    }

    pub fn cell(&self, id: CellId) -> &Cell {
        self.cells[id.0 as usize].as_ref().expect("removed cell")
    }

    pub fn cell_mut(&mut self, id: CellId) -> &mut Cell {
        self.cells[id.0 as usize].as_mut().expect("removed cell")
    }

    /// Live cells, in creation order.
    pub fn cell_ids(&self) -> Vec<CellId> {
        (0..self.cells.len())
            .filter(|&i| self.cells[i].is_some())
            .map(|i| CellId(i as u32))
            .collect()
    }

    pub fn cells_of(&self, ty: &str) -> Vec<CellId> {
        self.cell_ids().into_iter().filter(|&c| self.cell(c).ty == ty).collect()
    }

    /// Remove a cell, disconnecting every port.
    pub fn remove_cell(&mut self, id: CellId) {
        let ports: Vec<String> = self.cell(id).ports.keys().cloned().collect();
        for p in ports {
            self.disconnect(id, &p);
        }
        let name = self.cell(id).name.clone();
        self.cell_by_name.remove(&name);
        self.cells[id.0 as usize] = None;
    }

    // ---- ports ----

    pub fn add_port(&mut self, c: CellId, port: &str, dir: Dir) {
        self.cell_mut(c).ports.entry(port.to_string()).or_insert(Port { dir, net: None });
    }

    /// Connect `port` (added as an input if missing) to `net`, after
    /// disconnecting whatever it had.
    pub fn connect(&mut self, c: CellId, port: &str, net: NetId) {
        self.disconnect(c, port);
        let dir = {
            let cell = self.cell_mut(c);
            let p = cell.ports.entry(port.to_string()).or_insert(Port { dir: Dir::Input, net: None });
            p.net = Some(net);
            p.dir
        };
        let n = self.net_mut(net);
        if dir == Dir::Output {
            n.driver = Some((c, port.to_string()));
        } else {
            n.users.push((c, port.to_string()));
        }
    }

    pub fn disconnect(&mut self, c: CellId, port: &str) {
        let Some(p) = self.cell_mut(c).ports.get_mut(port) else { return };
        let Some(net) = p.net.take() else { return };
        let n = self.net_mut(net);
        if n.driver.as_ref().is_some_and(|(dc, dp)| *dc == c && dp == port) {
            n.driver = None;
        }
        n.users.retain(|(uc, up)| !(*uc == c && up == port));
    }

    /// Rename a port, keeping its net.
    pub fn rename_port(&mut self, c: CellId, old: &str, new: &str) {
        if old == new {
            return;
        }
        let Some(p) = self.cell_mut(c).ports.remove(old) else { return };
        if let Some(net) = p.net {
            let n = self.net_mut(net);
            if let Some((dc, dp)) = &mut n.driver
                && *dc == c
                && dp == old
            {
                *dp = new.to_string();
            }
            for (uc, up) in &mut n.users {
                if *uc == c && up == old {
                    *up = new.to_string();
                }
            }
        }
        self.cell_mut(c).ports.insert(new.to_string(), p);
    }

    /// Move `port` of `from` to `to_port` of `to`, with its net.
    pub fn move_port(&mut self, from: CellId, port: &str, to: CellId, to_port: &str) {
        let Some(p) = self.cell(from).ports.get(port).cloned() else { return };
        self.disconnect(from, port);
        self.cell_mut(from).ports.remove(port);
        self.add_port(to, to_port, p.dir);
        self.cell_mut(to).ports.get_mut(to_port).expect("added").dir = p.dir;
        if let Some(n) = p.net {
            self.connect(to, to_port, n);
        }
    }

    // ---- nets ----

    pub fn add_net(&mut self, name: &str) -> NetId {
        let id = NetId(self.nets.len() as u32);
        assert!(!self.net_by_name.contains_key(name), "duplicate net name {name}");
        self.nets.push(Some(Net {
            name: name.to_string(),
            driver: None,
            users: Vec::new(),
        }));
        self.net_by_name.insert(name.to_string(), id);
        id
    }

    /// A net named `name`, or `name$N` if taken.
    pub fn add_net_unique(&mut self, name: &str) -> NetId {
        if !self.net_by_name.contains_key(name) {
            return self.add_net(name);
        }
        let mut k = 1;
        loop {
            let n = format!("{name}${k}");
            if !self.net_by_name.contains_key(&n) {
                return self.add_net(&n);
            }
            k += 1;
        }
    }

    pub fn net(&self, id: NetId) -> &Net {
        self.nets[id.0 as usize].as_ref().expect("removed net")
    }

    fn net_mut(&mut self, id: NetId) -> &mut Net {
        self.nets[id.0 as usize].as_mut().expect("removed net")
    }

    pub fn net_id(&self, name: &str) -> Option<NetId> {
        self.net_by_name.get(name).copied()
    }

    pub fn is_const(&self, n: NetId) -> bool {
        n == self.gnd || n == self.vcc
    }

    /// The cell driving `n`, if a cell does.
    pub fn driver(&self, n: NetId) -> Option<(CellId, &str)> {
        self.net(n).driver.as_ref().map(|(c, p)| (*c, p.as_str()))
    }

    /// Nets with neither driver nor users nor a top-level port are dropped.
    pub fn sweep_nets(&mut self) {
        let on_port: std::collections::HashSet<NetId> = self.ports.iter().filter_map(|p| p.net).collect();
        for i in 0..self.nets.len() {
            let id = NetId(i as u32);
            let dead =
                self.nets[i].as_ref().is_some_and(|n| n.driver.is_none() && n.users.is_empty()) && !on_port.contains(&id) && !self.is_const(id);
            if dead {
                let name = self.nets[i].take().expect("live").name;
                self.net_by_name.remove(&name);
            }
        }
    }

    /// The packed design: bit numbers in net order from 2, cells unplaced.
    pub fn to_design(&self) -> Design {
        let mut d = Design::new(&self.top);
        let bit = |n: NetId| Bit::Net(n.0 as i64 + 2);
        for c in self.cells.iter().flatten() {
            let mut pc = packed::Cell::new(&c.ty);
            pc.params = c.params.clone();
            pc.attrs = c.attrs.clone();
            pc.loc = c.bel.clone();
            for (p, port) in &c.ports {
                pc.connect(p, port.dir, port.net.map(bit).into_iter().collect());
            }
            d.cells.insert(c.name.clone(), pc);
        }
        d
    }

    /// Ports and net names for `Design::to_json`, so the written netlist is
    /// complete: (ports object, netnames object).
    pub fn ports_and_names(&self) -> (Value, Value) {
        let mut ports = serde_json::Map::new();
        for p in &self.ports {
            let dir = match p.dir {
                Dir::Input => "input",
                Dir::Output => "output",
                Dir::Inout => "inout",
            };
            let bits: Vec<Value> = p.net.map(|n| json!(n.0 as i64 + 2)).into_iter().collect();
            ports.insert(p.name.clone(), json!({"direction": dir, "bits": bits}));
        }
        let mut names = serde_json::Map::new();
        for (i, n) in self.nets.iter().enumerate() {
            if let Some(n) = n {
                names.insert(n.name.clone(), json!({"hide_name": 0, "bits": [i as i64 + 2], "attributes": {}}));
            }
        }
        (Value::Object(ports), Value::Object(names))
    }
}

/// `Bit::parse` for this crate (packed's is private).
trait ParseJson {
    fn parse_json(v: &Value) -> R<Bit>;
}

impl ParseJson for Bit {
    fn parse_json(v: &Value) -> R<Bit> {
        match v {
            Value::Number(n) => n.as_i64().map(Bit::Net).ok_or_else(|| format!("bit {n}")),
            Value::String(s) => match s.as_str() {
                "0" => Ok(Bit::Zero),
                "1" => Ok(Bit::One),
                "x" => Ok(Bit::X),
                "z" => Ok(Bit::Z),
                _ => Err(format!("bit {s:?}")),
            },
            _ => Err(format!("bit {v}")),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn loads_buses_constants_and_ports() {
        let v = json!({"modules": {"top": {
            "attributes": {"top": "1"},
            "ports": {"a": {"direction": "input", "bits": [2, 3]}, "y": {"direction": "output", "bits": [4]}},
            "netnames": {"a": {"hide_name": 0, "bits": [2, 3]}, "y": {"hide_name": 0, "bits": [4]}, "$x": {"hide_name": 1, "bits": [4]}},
            "cells": {"l": {"type": "LUT2", "parameters": {"INIT": "1000"},
                            "port_directions": {"I0": "input", "I1": "input", "O": "output"},
                            "connections": {"I0": [2], "I1": ["1"], "O": [4]}},
                      "w": {"type": "CARRY4", "port_directions": {"S": "input"}, "connections": {"S": [3, "0", "x", 2]}}}
        }}});
        let nl = Netlist::from_json(&v).unwrap();
        assert_eq!(nl.ports.len(), 3);
        let l = nl.cell_id("l").unwrap();
        assert_eq!(nl.cell(l).net("I1"), Some(nl.vcc));
        let y = nl.cell(l).net("O").unwrap();
        assert_eq!(nl.net(y).name, "y");
        assert_eq!(nl.driver(y).map(|(c, p)| (c, p.to_string())), Some((l, "O".to_string())));
        let w = nl.cell_id("w").unwrap();
        assert_eq!(nl.cell(w).net("S[1]"), Some(nl.gnd));
        assert_eq!(nl.cell(w).net("S[2]"), None);
        assert_eq!(nl.net(nl.cell(w).net("S[0]").unwrap()).name, "a[1]");
    }

    #[test]
    fn edits_keep_driver_and_users_consistent() {
        let mut nl = Netlist::new("t");
        let a = nl.add_cell("a", "LUT1");
        nl.add_port(a, "O", Dir::Output);
        let b = nl.add_cell("b", "FDRE");
        nl.add_port(b, "D", Dir::Input);
        let n = nl.add_net("n");
        nl.connect(a, "O", n);
        nl.connect(b, "D", n);
        nl.rename_port(a, "O", "O6");
        assert_eq!(nl.net(n).driver, Some((a, "O6".to_string())));
        let c = nl.add_cell("c", "SLICE_FFX");
        nl.move_port(b, "D", c, "D");
        assert_eq!(nl.net(n).users, vec![(c, "D".to_string())]);
        nl.remove_cell(a);
        assert_eq!(nl.net(n).driver, None);
    }
}
