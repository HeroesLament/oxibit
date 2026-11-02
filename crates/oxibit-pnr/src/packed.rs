//! The packed, placed design the back end works from: cells already split
//! into bel-sized pieces, their ports renamed to bel pins, each cell bound
//! to a bel.
//!
//! This is the boundary between placement and routing. Today two things
//! produce it: nextpnr-xilinx's packed JSON (`Design::from_json`) and, next,
//! oxibit's own packer and placer, which build it directly. The writers
//! downstream still read the JSON shape (`Design::to_json`); they move onto
//! the typed view one at a time, each move checked by an identical FASM.
//!
//! The conventions of the packed netlist live here and nowhere else:
//!
//! - `NEXTPNR_BEL` is the cell's bel, `X{gx}Y{gy}/{SITE}.{BEL}` (`Loc::Grid`:
//!   tile by prjxray grid coordinates, site relative to its tile) or, from
//!   older builds, `{SITE}/{BEL}` by absolute site (`Loc::Site`).
//! - Ports carry bel pin names; `X_ORIG_PORT_<pin>` names the logical port
//!   behind each (a LUT's `A3` reading `I1`, a DSP's `A0` reading `A[0]`).
//!   `X_ORIG_TYPE` is the cell type before packing.
//! - Constant pins hang on the nets `PSEUDO_VCC` / `PSEUDO_GND` drive.
//!
//! What `from_json` does not model (other modules, module ports, netnames,
//! a cell's `hide_name`) is carried as JSON and written back unchanged, so
//! `to_json(from_json(v)) == v` for any yosys-style JSON.

use std::collections::BTreeMap;

use serde_json::{Map, Value, json};

type R<T> = Result<T, String>;

/// A port's direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dir {
    Input,
    Output,
    Inout,
}

impl Dir {
    fn parse(s: &str) -> Option<Dir> {
        match s {
            "input" => Some(Dir::Input),
            "output" => Some(Dir::Output),
            "inout" => Some(Dir::Inout),
            _ => None,
        }
    }

    fn name(self) -> &'static str {
        match self {
            Dir::Input => "input",
            Dir::Output => "output",
            Dir::Inout => "inout",
        }
    }
}

/// One bit of a port connection.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Bit {
    /// A net, by its JSON bit number.
    Net(i64),
    Zero,
    One,
    X,
    Z,
}

impl Bit {
    fn parse(v: &Value) -> R<Bit> {
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

    fn to_json(self) -> Value {
        match self {
            Bit::Net(n) => json!(n),
            Bit::Zero => json!("0"),
            Bit::One => json!("1"),
            Bit::X => json!("x"),
            Bit::Z => json!("z"),
        }
    }

    pub fn net(self) -> Option<i64> {
        match self {
            Bit::Net(n) => Some(n),
            _ => None,
        }
    }
}

/// A cell port: its direction and its bits. The JSON keeps the two in
/// separate maps and either may be missing; so may these.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Port {
    pub dir: Option<Dir>,
    pub bits: Option<Vec<Bit>>,
}

/// Where a cell sits.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Loc {
    /// `X{x}Y{y}/{site}.{bel}`: tile by grid coordinates, site relative to
    /// the tile (`SLICE_X1Y0`), bel as nextpnr names it (`A6LUT`,
    /// `IOB33S.PAD`, `RAMB36E1~RAMB36E1`).
    Grid { x: i32, y: i32, site: String, bel: String },
    /// `{site}/{bel}` by absolute site name (`SLICE_X42Y98/D5FF`).
    Site { site: String, bel: String },
}

impl Loc {
    pub fn parse(s: &str) -> Option<Loc> {
        let (head, rest) = s.split_once('/')?;
        if let Some(t) = head.strip_prefix('X')
            && let Some((x, y)) = t.split_once('Y')
            && let (Ok(x), Ok(y)) = (x.parse(), y.parse())
            && let Some((site, bel)) = rest.split_once('.')
        {
            return Some(Loc::Grid {
                x,
                y,
                site: site.to_string(),
                bel: bel.to_string(),
            });
        }
        Some(Loc::Site {
            site: head.to_string(),
            bel: rest.to_string(),
        })
    }

    pub fn text(&self) -> String {
        match self {
            Loc::Grid { x, y, site, bel } => format!("X{x}Y{y}/{site}.{bel}"),
            Loc::Site { site, bel } => format!("{site}/{bel}"),
        }
    }
}

/// What a packed cell is, from its type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Kind {
    /// `SLICE_LUTX`: a LUT, or one LUT of distributed RAM or a shift register.
    Lut,
    /// `SLICE_FFX`.
    Ff,
    Carry4,
    /// `SELMUX2_1`: MUXF7 / MUXF8.
    WideMux,
    /// `PAD`: a package pin.
    Pad,
    /// `IOB33*_INBUF_EN`, `IOB33*_OUTBUF` and friends.
    IoBuffer,
    /// `OLOGICE3_*`, `ILOGICE3_*`, `IDELAYE2_*`, `IDELAYCTRL_*`.
    IoLogic,
    Inverter,
    /// `BUFGCTRL`, `BUFG`, `BUFHCE`, `BUFH`.
    ClockBuffer,
    Dsp,
    BlockRam,
    Ps7,
    Mmcm,
    PseudoVcc,
    PseudoGnd,
    Other,
}

impl Kind {
    pub fn of(ty: &str) -> Kind {
        match ty {
            "SLICE_LUTX" => Kind::Lut,
            "SLICE_FFX" => Kind::Ff,
            "CARRY4" => Kind::Carry4,
            "SELMUX2_1" => Kind::WideMux,
            "PAD" => Kind::Pad,
            "INVERTER" => Kind::Inverter,
            "BUFGCTRL" | "BUFG" | "BUFHCE" | "BUFH" => Kind::ClockBuffer,
            "PSEUDO_VCC" => Kind::PseudoVcc,
            "PSEUDO_GND" => Kind::PseudoGnd,
            "PS7_PS7" => Kind::Ps7,
            t if t.starts_with("DSP48E1") => Kind::Dsp,
            t if t.starts_with("RAMB18") || t.starts_with("RAMB36") => Kind::BlockRam,
            t if t.starts_with("MMCME2") => Kind::Mmcm,
            t if t.starts_with("IOB") && (t.contains("INBUF") || t.contains("OUTBUF")) => Kind::IoBuffer,
            t if t.starts_with("OLOGIC") || t.starts_with("ILOGIC") || t.starts_with("IDELAY") || t.starts_with("ODELAY") => Kind::IoLogic,
            _ => Kind::Other,
        }
    }
}

/// One packed cell.
#[derive(Debug, Clone, PartialEq)]
pub struct Cell {
    /// The packed type (`SLICE_LUTX`, `DSP48E1_DSP48E1`, ...).
    pub ty: String,
    /// Where it sits; `None` when unplaced.
    pub loc: Option<Loc>,
    /// Parameters, as the JSON writes them (bit strings, strings, numbers).
    pub params: BTreeMap<String, Value>,
    /// Attributes other than `NEXTPNR_BEL`.
    pub attrs: BTreeMap<String, Value>,
    pub ports: BTreeMap<String, Port>,
    /// `NEXTPNR_BEL` text `Loc::parse` did not take, written back as is.
    loc_raw: Option<Value>,
    /// Other cell keys (`hide_name`, ...), written back as is.
    rest: Map<String, Value>,
}

impl Cell {
    pub fn new(ty: &str) -> Cell {
        Cell {
            ty: ty.to_string(),
            loc: None,
            params: BTreeMap::new(),
            attrs: BTreeMap::new(),
            ports: BTreeMap::new(),
            loc_raw: None,
            rest: Map::new(),
        }
    }

    pub fn kind(&self) -> Kind {
        Kind::of(&self.ty)
    }

    /// The type before packing (`X_ORIG_TYPE`), when recorded.
    pub fn orig_type(&self) -> Option<&str> {
        self.attrs.get("X_ORIG_TYPE").and_then(Value::as_str)
    }

    /// The logical port behind bel pin `pin` (`X_ORIG_PORT_<pin>`).
    pub fn orig_port(&self, pin: &str) -> Option<&str> {
        self.attrs.get(&format!("X_ORIG_PORT_{pin}")).and_then(Value::as_str)
    }

    pub fn bits(&self, port: &str) -> &[Bit] {
        self.ports.get(port).and_then(|p| p.bits.as_deref()).unwrap_or(&[])
    }

    /// Connect `port` (direction `dir`) to `bits`.
    pub fn connect(&mut self, port: &str, dir: Dir, bits: Vec<Bit>) {
        self.ports.insert(
            port.to_string(),
            Port {
                dir: Some(dir),
                bits: Some(bits),
            },
        );
    }

    fn from_json(v: &Value) -> R<Cell> {
        let o = v.as_object().ok_or("cell is not an object")?;
        let mut c = Cell::new(o.get("type").and_then(Value::as_str).ok_or("cell without type")?);
        for (k, x) in o {
            match k.as_str() {
                "type" => {}
                "parameters" => c.params = obj(x)?.clone().into_iter().collect(),
                "attributes" => c.attrs = obj(x)?.clone().into_iter().collect(),
                "port_directions" => {
                    for (p, d) in obj(x)? {
                        let d = d.as_str().and_then(Dir::parse).ok_or_else(|| format!("direction {d} of {p}"))?;
                        c.ports.entry(p.clone()).or_default().dir = Some(d);
                    }
                }
                "connections" => {
                    for (p, bs) in obj(x)? {
                        let bs = bs
                            .as_array()
                            .ok_or_else(|| format!("connection {p}"))?
                            .iter()
                            .map(Bit::parse)
                            .collect::<R<Vec<_>>>()?;
                        c.ports.entry(p.clone()).or_default().bits = Some(bs);
                    }
                }
                _ => {
                    c.rest.insert(k.clone(), x.clone());
                }
            }
        }
        if let Some(b) = c.attrs.remove("NEXTPNR_BEL") {
            // Only a form `Loc::text` writes back verbatim becomes typed.
            match b.as_str().and_then(|s| Loc::parse(s).filter(|l| l.text() == s)) {
                Some(l) => c.loc = Some(l),
                None => c.loc_raw = Some(b),
            }
        }
        Ok(c)
    }

    fn to_json(&self) -> Value {
        let mut o = self.rest.clone();
        o.insert("type".into(), json!(self.ty));
        let mut attrs: Map<String, Value> = self.attrs.clone().into_iter().collect();
        if let Some(l) = &self.loc {
            attrs.insert("NEXTPNR_BEL".into(), json!(l.text()));
        } else if let Some(r) = &self.loc_raw {
            attrs.insert("NEXTPNR_BEL".into(), r.clone());
        }
        o.insert("parameters".into(), Value::Object(self.params.clone().into_iter().collect()));
        o.insert("attributes".into(), Value::Object(attrs));
        let dirs: Map<String, Value> = self
            .ports
            .iter()
            .filter_map(|(p, x)| x.dir.map(|d| (p.clone(), json!(d.name()))))
            .collect();
        let conns: Map<String, Value> = self
            .ports
            .iter()
            .filter_map(|(p, x)| {
                x.bits
                    .as_ref()
                    .map(|b| (p.clone(), Value::Array(b.iter().map(|b| b.to_json()).collect())))
            })
            .collect();
        o.insert("port_directions".into(), Value::Object(dirs));
        o.insert("connections".into(), Value::Object(conns));
        Value::Object(o)
    }
}

fn obj(v: &Value) -> R<&Map<String, Value>> {
    v.as_object().ok_or_else(|| format!("expected an object, got {v}"))
}

/// A packed, placed design: the top module's cells, typed.
#[derive(Debug, Clone, PartialEq)]
pub struct Design {
    /// The top module's name.
    pub top: String,
    /// Cells by name (sorted, as the JSON object iterates).
    pub cells: BTreeMap<String, Cell>,
    /// Which cell keys the JSON had, so `to_json` writes the same keys back
    /// (a missing `parameters` stays missing).
    cell_keys: BTreeMap<String, Vec<String>>,
    /// The top module's other keys (`attributes`, `ports`, `netnames`, ...).
    module_rest: Map<String, Value>,
    /// The top module had no `cells` key.
    no_cells: bool,
    /// The other modules, and the file's other keys.
    others: Map<String, Value>,
    root_rest: Map<String, Value>,
}

impl Design {
    /// The top module (attribute `top`, else the first) of a yosys-style
    /// JSON netlist.
    pub fn from_json(v: &Value) -> R<Design> {
        let root = obj(v)?;
        let mods = root.get("modules").and_then(Value::as_object).ok_or("no modules")?;
        let top = mods
            .iter()
            .find(|(_, m)| m["attributes"]["top"].is_string() || m["attributes"]["top"].is_number())
            .or_else(|| mods.iter().next())
            .map(|(k, _)| k.clone())
            .ok_or("no top module")?;
        let m = obj(&mods[&top])?;
        let mut cells = BTreeMap::new();
        let mut cell_keys = BTreeMap::new();
        if let Some(cs) = m.get("cells") {
            for (name, c) in obj(cs)? {
                cells.insert(name.clone(), Cell::from_json(c).map_err(|e| format!("cell {name}: {e}"))?);
                cell_keys.insert(name.clone(), obj(c)?.keys().cloned().collect());
            }
        }
        let module_rest = m
            .iter()
            .filter(|(k, _)| k.as_str() != "cells")
            .map(|(k, x)| (k.clone(), x.clone()))
            .collect();
        let others = mods.iter().filter(|(k, _)| **k != top).map(|(k, x)| (k.clone(), x.clone())).collect();
        let root_rest = root
            .iter()
            .filter(|(k, _)| k.as_str() != "modules")
            .map(|(k, x)| (k.clone(), x.clone()))
            .collect();
        let no_cells = !m.contains_key("cells");
        Ok(Design {
            top,
            cells,
            cell_keys,
            module_rest,
            no_cells,
            others,
            root_rest,
        })
    }

    /// An empty design with top module `top`.
    pub fn new(top: &str) -> Design {
        let mut module_rest = Map::new();
        module_rest.insert("attributes".into(), json!({"top": "00000000000000000000000000000001"}));
        module_rest.insert("ports".into(), json!({}));
        module_rest.insert("netnames".into(), json!({}));
        Design {
            top: top.to_string(),
            cells: BTreeMap::new(),
            cell_keys: BTreeMap::new(),
            module_rest,
            no_cells: false,
            others: Map::new(),
            root_rest: Map::new(),
        }
    }

    /// Back to the yosys-style JSON the writers read.
    pub fn to_json(&self) -> Value {
        let mut m = self.module_rest.clone();
        if !self.no_cells || !self.cells.is_empty() {
            let mut cs = Map::new();
            for (name, c) in &self.cells {
                let mut v = c.to_json();
                // Keys the source cell did not have stay absent.
                if let (Some(keys), Some(o)) = (self.cell_keys.get(name), v.as_object_mut()) {
                    o.retain(|k, x| keys.contains(k) || !is_empty_default(k, x));
                }
                cs.insert(name.clone(), v);
            }
            m.insert("cells".into(), Value::Object(cs));
        }
        let mut mods = self.others.clone();
        mods.insert(self.top.clone(), Value::Object(m));
        let mut root = self.root_rest.clone();
        root.insert("modules".into(), Value::Object(mods));
        Value::Object(root)
    }

    /// Cells of one kind, by name.
    pub fn of_kind(&self, k: Kind) -> impl Iterator<Item = (&String, &Cell)> {
        self.cells.iter().filter(move |(_, c)| c.kind() == k)
    }
}

/// A key `Cell::to_json` always writes, at its empty value.
fn is_empty_default(k: &str, v: &Value) -> bool {
    matches!(k, "parameters" | "attributes" | "port_directions" | "connections") && v.as_object().is_some_and(Map::is_empty)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> Value {
        json!({
            "creator": "nextpnr",
            "modules": {
                "lib": {"cells": {}},
                "top": {
                    "attributes": {"top": "00000000000000000000000000000001"},
                    "ports": {"a": {"direction": "input", "bits": [2]}},
                    "cells": {
                        "l": {"hide_name": 0, "type": "SLICE_LUTX",
                              "parameters": {"INIT": "10"},
                              "attributes": {"NEXTPNR_BEL": "X83Y85/SLICE_X1Y0.A6LUT", "X_ORIG_PORT_A2": "I0", "X_ORIG_TYPE": "LUT1"},
                              "port_directions": {"A2": "input", "A6": "input", "O6": "output", "A1": "input"},
                              "connections": {"A2": [2], "A6": ["1"], "O6": [3], "A1": []}},
                        "f": {"type": "SLICE_FFX",
                              "attributes": {"NEXTPNR_BEL": "SLICE_X42Y98/D5FF"},
                              "port_directions": {"D": "input", "Q": "output"},
                              "connections": {"D": [3], "Q": [4]}},
                        "odd": {"type": "BUFGCTRL", "attributes": {"NEXTPNR_BEL": "weird"}}
                    },
                    "netnames": {"a": {"bits": [2]}}
                }
            }
        })
    }

    #[test]
    fn round_trip_is_identity() {
        let v = sample();
        let d = Design::from_json(&v).unwrap();
        assert_eq!(d.to_json(), v);
    }

    #[test]
    fn typed_view() {
        let d = Design::from_json(&sample()).unwrap();
        assert_eq!(d.top, "top");
        let l = &d.cells["l"];
        assert_eq!(l.kind(), Kind::Lut);
        assert_eq!(l.orig_type(), Some("LUT1"));
        assert_eq!(l.orig_port("A2"), Some("I0"));
        assert_eq!(
            l.loc,
            Some(Loc::Grid {
                x: 83,
                y: 85,
                site: "SLICE_X1Y0".into(),
                bel: "A6LUT".into()
            })
        );
        assert_eq!(l.bits("A6"), &[Bit::One]);
        assert_eq!(l.bits("A1"), &[] as &[Bit]);
        assert_eq!(
            d.cells["f"].loc,
            Some(Loc::Site {
                site: "SLICE_X42Y98".into(),
                bel: "D5FF".into()
            })
        );
        // Not a bel this module can type: kept as text.
        assert_eq!(d.cells["odd"].loc, None);
    }

    #[test]
    fn built_cells_serialize() {
        let mut d = Design::new("top");
        let mut c = Cell::new("SLICE_FFX");
        c.loc = Some(Loc::Grid {
            x: 1,
            y: 2,
            site: "SLICE_X0Y0".into(),
            bel: "AFF".into(),
        });
        c.connect("D", Dir::Input, vec![Bit::Net(7)]);
        d.cells.insert("ff".into(), c);
        let v = d.to_json();
        let back = Design::from_json(&v).unwrap();
        assert_eq!(back.cells["ff"].loc, d.cells["ff"].loc);
        assert_eq!(back.cells["ff"].bits("D"), &[Bit::Net(7)]);
        assert_eq!(v["modules"]["top"]["cells"]["ff"]["attributes"]["NEXTPNR_BEL"], "X1Y2/SLICE_X0Y0.AFF");
    }
}
