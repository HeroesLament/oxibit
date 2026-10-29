//! Nets of a placed, unrouted design, resolved to the site pins oxibit's
//! graph routes between.
//!
//! Input is nextpnr's JSON after `--no-route --write` (or after routing; the
//! routing is ignored). A cell's bel is `X{gx}Y{gy}/{SITE}.{BEL}`: the tile
//! by prjxray grid coordinates and the site tile-relative (`SLICE_X1Y0`,
//! `DSP48_X0Y1`). Older builds name it `{SITE}/{BEL}` by the absolute site
//! (`SLICE_X42Y98/D5FF`), resolved through the tilegrid (`site_index`). The
//! cell's ports are renamed to the bel's pins.
//!
//! Inside a site, bel pins meet over site wires and routing muxes. Those come
//! from the site types of nextpnr-xilinx-meta (CC0): per site variant, each
//! bel pin's site wire, each site pin's wire, and the pips of the routing
//! bels (`class: RBEL`: FF input muxes, output muxes, CE/SR/CLK selects). A
//! sink its driver reaches over routing-bel pips in the same site needs no
//! routing. Every other sink is resolved to the input site pin a
//! routing-bel path leads back to, and the driver to the output site pin a
//! path leads forward to.
//!
//! LUT inputs become pin groups (`Net::pin_groups`): the LUT's INIT can be
//! permuted to read its inputs on any of the allowed pins, so the router
//! picks the pin. `Perm` sets which pins are allowed. A fractured LUT (both
//! the 6LUT and the 5LUT in use) holds A6 high, so its group is A1..A5
//! whatever `Perm` says. A slice holding distributed RAM or a shift
//! register keeps every LUT pin where the placer put it: the write address
//! is shared across the slice's LUTs on the D LUT's pins.
//!
//! Not routed here, and counted in `Import::skipped`: constant nets (driven
//! by `PSEUDO_GND`/`PSEUDO_VCC`), clock nets (driven by a clock buffer), and
//! nets on an IOB pad.

use std::collections::hash_map::Entry;
use std::collections::{BTreeMap, HashMap, HashSet, VecDeque};
use std::fs;
use std::path::{Path, PathBuf};

use oxibit_fasm::db::Database;
use oxibit_fasm::route::{EdgeKind, Graph};
use serde_json::Value;

use crate::nets::{Edge, Net, NetClass};
use crate::router::Hop;

type R<T> = Result<T, String>;

/// Which LUT inputs a net may move between.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Perm {
    /// Every LUT input stays on the pin the placer gave it.
    None,
    /// A1..A5 permute; a net on A6 stays there (nextpnr today).
    A5,
    /// A1..A6 permute, unless the LUT is fractured.
    A6,
}

/// One routing-bel pip, by site wire.
#[derive(Debug, Clone)]
struct Variant {
    /// Site pin -> (wire, primary pin name, is input).
    pins: HashMap<String, (String, String, bool)>,
    /// Bel -> pin -> wire.
    bels: HashMap<String, HashMap<String, String>>,
    /// Bel -> bel type.
    bel_type: HashMap<String, String>,
    /// Wire -> wires a routing-bel pip drives it from.
    back: HashMap<String, Vec<String>>,
    /// Wire -> wires a routing-bel pip drives from it.
    fwd: HashMap<String, Vec<String>>,
    /// Wire -> input site pin on it.
    in_pin: HashMap<String, String>,
    /// Wire -> output site pin on it.
    out_pin: HashMap<String, String>,
    /// (from wire, to wire) -> (routing bel, its input pin): the mux setting
    /// a site path through that pip needs.
    pip_bel: HashMap<(String, String), (String, String)>,
    /// Output wire of a constant bel in the site (CEUSEDVCC, SRUSEDGND,
    /// CYINITVCC, ...) -> its value.
    const_wire: HashMap<String, bool>,
}

/// The site types of nextpnr-xilinx-meta for one family directory.
pub struct Meta {
    dir: PathBuf,
    /// Primary site type -> variant name -> variant.
    types: HashMap<String, Option<HashMap<String, Variant>>>,
}

impl Meta {
    pub fn open(dir: &Path) -> Self {
        Meta { dir: dir.to_path_buf(), types: HashMap::new() }
    }

    fn site_type(&mut self, primary: &str) -> Option<&HashMap<String, Variant>> {
        if !self.types.contains_key(primary) {
            let path = self.dir.join(format!("site_type_{primary}.json"));
            let parsed = fs::read_to_string(&path).ok().and_then(|t| serde_json::from_str::<Value>(&t).ok()).map(|v| parse_site_type(&v));
            self.types.insert(primary.to_string(), parsed);
        }
        self.types.get(primary).and_then(|o| o.as_ref())
    }
}

fn parse_site_type(v: &Value) -> HashMap<String, Variant> {
    let mut out = HashMap::new();
    for (name, var) in v.as_object().into_iter().flatten() {
        let mut pins = HashMap::new();
        let mut in_pin = HashMap::new();
        let mut out_pin = HashMap::new();
        for (pin, p) in var["pins"].as_object().into_iter().flatten() {
            let wire = p["wire"].as_str().unwrap_or("").to_string();
            let primary = p["primary"].as_str().unwrap_or(pin).to_string();
            let input = p["dir"].as_str() == Some("INPUT");
            if input {
                in_pin.insert(wire.clone(), pin.clone());
            } else {
                out_pin.insert(wire.clone(), pin.clone());
            }
            pins.insert(pin.clone(), (wire, primary, input));
        }
        let mut bels = HashMap::new();
        let mut bel_type = HashMap::new();
        let mut rbel = HashSet::new();
        let mut const_wire = HashMap::new();
        for (bel, b) in var["bels"].as_object().into_iter().flatten() {
            let ty = b["type"].as_str().unwrap_or("");
            let value = if bel.ends_with("VCC") || ty.ends_with("VCC") {
                Some(true)
            } else if bel.ends_with("GND") || ty.ends_with("GND") {
                Some(false)
            } else {
                None
            };
            if let Some(value) = value {
                for d in b["pins"].as_object().into_iter().flatten().map(|(_, d)| d) {
                    if d["dir"].as_str() == Some("OUTPUT") {
                        const_wire.insert(d["wire"].as_str().unwrap_or("").to_string(), value);
                    }
                }
            }
            let bp: HashMap<String, String> =
                b["pins"].as_object().into_iter().flatten().map(|(p, d)| (p.clone(), d["wire"].as_str().unwrap_or("").to_string())).collect();
            bels.insert(bel.clone(), bp);
            bel_type.insert(bel.clone(), b["type"].as_str().unwrap_or("").to_string());
            if b["class"].as_str() == Some("RBEL") {
                rbel.insert(bel.clone());
            }
        }
        let mut back: HashMap<String, Vec<String>> = HashMap::new();
        let mut fwd: HashMap<String, Vec<String>> = HashMap::new();
        let mut pip_bel: HashMap<(String, String), (String, String)> = HashMap::new();
        for pip in var["pips"].as_array().into_iter().flatten() {
            let bel = pip["bel"].as_str().unwrap_or("");
            if !rbel.contains(bel) {
                continue; // LUT and carry pass-throughs are not wiring
            }
            let (Some(f), Some(t)) = (pip["from_pin"].as_str(), pip["to_pin"].as_str()) else { continue };
            let (Some(fw), Some(tw)) = (bels[bel].get(f), bels[bel].get(t)) else { continue };
            back.entry(tw.clone()).or_default().push(fw.clone());
            fwd.entry(fw.clone()).or_default().push(tw.clone());
            pip_bel.insert((fw.clone(), tw.clone()), (bel.to_string(), f.to_string()));
        }
        for v in back.values_mut().chain(fwd.values_mut()) {
            v.sort();
            v.dedup();
        }
        out.insert(name.clone(), Variant { pins, bels, bel_type, back, fwd, in_pin, out_pin, pip_bel, const_wire });
    }
    out
}

/// Site wires reachable from `from` over routing-bel pips (`step`), nearest
/// first, `from` included.
fn walk<'a>(from: &'a str, step: &'a HashMap<String, Vec<String>>) -> Vec<String> {
    let mut seen = HashSet::from([from.to_string()]);
    let mut order = vec![from.to_string()];
    let mut q = VecDeque::from([from.to_string()]);
    while let Some(w) = q.pop_front() {
        for n in step.get(&w).into_iter().flatten() {
            if seen.insert(n.clone()) {
                order.push(n.clone());
                q.push_back(n.clone());
            }
        }
    }
    order
}

/// A placed cell port, resolved to its site.
#[derive(Debug, Clone)]
struct Pin {
    tile: String,
    site: String,
    /// The site as nextpnr names it, tile-relative (`SLICE_X0Y0`).
    np_site: String,
    /// Index into `Import`'s site variants: (primary type, variant).
    var: (String, String),
    bel: String,
    port: String,
    wire: String,
    cell: usize,
}

/// How a routed net leaves its driver's site: the site wires from the
/// driver's bel pin to the output site pin (routing-bel pips between them),
/// and the tile wire of that site pin. What another tool needs to join the
/// tile routing to the driver.
#[derive(Debug, Clone)]
pub struct SourcePath {
    pub tile: String,
    /// The site as nextpnr names it, tile-relative (`SLICE_X0Y0`).
    pub np_site: String,
    /// The site's absolute name (`SLICE_X26Y97`), when the tilegrid has it.
    pub abs_site: Option<String>,
    pub site_wires: Vec<String>,
    pub tile_wire: String,
}

/// How a sink pin reaches the bel pin it serves: the tile wire of the input
/// site pin, then the site wires from that pin to the bel pin. For a LUT
/// input the first site wire is the cell's own input (the logical pin): the
/// pip into it from the physical pin is the LUT permutation.
#[derive(Debug, Clone)]
pub struct SinkPath {
    pub tile: String,
    pub np_site: String,
    pub tile_wire: String,
    pub site_wires: Vec<String>,
}

/// Site wires from `from` to `to` over routing-bel pips, both included.
fn site_path(from: &str, to: &str, fwd: &HashMap<String, Vec<String>>) -> Option<Vec<String>> {
    let mut prev: HashMap<String, String> = HashMap::new();
    let mut q = VecDeque::from([from.to_string()]);
    let mut seen = HashSet::from([from.to_string()]);
    while let Some(w) = q.pop_front() {
        if w == to {
            let mut path = vec![w.clone()];
            let mut c = w;
            while let Some(p) = prev.get(&c) {
                path.push(p.clone());
                c = p.clone();
            }
            path.reverse();
            return Some(path);
        }
        for n in fwd.get(&w).into_iter().flatten() {
            if seen.insert(n.clone()) {
                prev.insert(n.clone(), w.clone());
                q.push_back(n.clone());
            }
        }
    }
    None
}

/// What `import` could not resolve, by reason, with a few examples.
#[derive(Debug, Default)]
pub struct Unresolved {
    pub counts: BTreeMap<String, usize>,
    pub examples: BTreeMap<String, Vec<String>>,
}

impl Unresolved {
    fn add(&mut self, why: &str, what: String) {
        *self.counts.entry(why.to_string()).or_default() += 1;
        let ex = self.examples.entry(why.to_string()).or_default();
        if ex.len() < 4 {
            ex.push(what);
        }
    }
}

/// The LUT behind one pin group: where it is, the pin number of each group
/// member, and the bel pins the placer put the net on (`lutinit`).
#[derive(Debug, Clone)]
pub struct LutSlot {
    pub tile: String,
    /// The site as FASM names it (`SLICEL_X0`).
    pub site: String,
    pub letter: char,
    /// Pin number (1..=6) of each member of the group, in group order.
    pub pins: Vec<u8>,
    /// The net's bel pins on this LUT as placed.
    pub from: Vec<u8>,
}

/// Result of `import`.
#[derive(Debug, Default)]
pub struct Import {
    pub nets: Vec<Net>,
    /// Logical net name of each routed net.
    pub names: Vec<String>,
    /// The JSON net bit of each routed net.
    pub bits: Vec<i64>,
    /// How each routed net leaves its driver's site.
    pub sources: Vec<SourcePath>,
    /// Per routed net: for each sink node it may reach (fixed sinks and
    /// every member of its pin groups), how that pin reaches its bel pins.
    pub sink_paths: Vec<Vec<(u32, SinkPath)>>,
    /// Per routed net, the LUT behind each of its pin groups, in order.
    pub group_slots: Vec<Vec<LutSlot>>,
    /// Constant inputs to route from a tie: (value, input site pin node).
    pub const_sinks: Vec<(bool, u32)>,
    /// Constant inputs a site drives itself (CE from CEUSEDVCC, ...).
    pub const_internal: usize,
    /// The routing-bel setting every site path the import chose needs:
    /// (tile, FASM site) -> routing bel -> its input pin (`apply_site_muxes`).
    pub site_muxes: BTreeMap<(String, String), BTreeMap<String, String>>,
    /// Logical nets not routed, by reason.
    pub skipped: BTreeMap<String, usize>,
    /// Sinks reached inside their driver's site.
    pub internal_sinks: usize,
    pub fixed_sinks: usize,
    /// Pin groups by allowed size.
    pub group_sizes: BTreeMap<usize, usize>,
    /// LUT letters (site, A..D) by kind: whole, fractured, memory.
    pub luts: BTreeMap<String, usize>,
    pub unresolved: Unresolved,
}

struct Cell {
    attrs: serde_json::Map<String, Value>,
}

fn top_module(v: &Value) -> R<&Value> {
    let mods = v["modules"].as_object().ok_or("no modules")?;
    mods.values()
        .find(|m| m["attributes"]["top"].is_string() || m["attributes"]["top"].is_number())
        .or_else(|| mods.values().next())
        .ok_or_else(|| "no top module".to_string())
}

/// `X14Y142/SLICE_X0Y0.A6LUT` -> ((14, 142), "SLICE_X0Y0", "A6LUT").
fn parse_bel(s: &str) -> Option<((i32, i32), String, String)> {
    let (tile, rest) = s.split_once('/')?;
    let t = tile.strip_prefix('X')?;
    let (x, y) = t.split_once('Y')?;
    let (site, bel) = rest.split_once('.')?;
    Some(((x.parse().ok()?, y.parse().ok()?), site.to_string(), bel.to_string()))
}

/// Absolute site name (`SLICE_X43Y98`) -> (tile, the site's name relative
/// to its tile as nextpnr writes it: `SLICE_X1Y0`).
pub fn site_index(db: &Database) -> HashMap<String, (String, String)> {
    let mut out = HashMap::new();
    for (tile, info) in &db.tiles {
        let parsed: Vec<(String, String, i64, i64)> =
            info.sites.iter().filter_map(|s| parse_site(s).map(|(p, x, y)| (s.clone(), p, x, y))).collect();
        for (name, prefix, x, y) in &parsed {
            let same = parsed.iter().filter(|(_, p, _, _)| p == prefix);
            let x0 = same.clone().map(|t| t.2).min().unwrap_or(*x);
            let y0 = same.map(|t| t.3).min().unwrap_or(*y);
            out.insert(name.clone(), (tile.clone(), format!("{prefix}_X{}Y{}", x - x0, y - y0)));
        }
    }
    out
}

/// `SLICE_X1Y0` -> ("SLICE", 1, 0).
fn parse_site(s: &str) -> Option<(String, i64, i64)> {
    let i = s.rfind("_X")?;
    let (x, y) = s[i + 2..].split_once('Y')?;
    Some((s[..i].to_string(), x.parse().ok()?, y.parse().ok()?))
}

/// The graph's name for nextpnr's site `np` in `tile`.
fn graph_site(g: &Graph, tile: &str, np: &str) -> Option<String> {
    let (prefix, x, y) = parse_site(np)?;
    let sites = g.sites(tile);
    if prefix == "SLICE" {
        return sites.into_iter().find(|s| s.starts_with("SLICE") && s.ends_with(&format!("_X{x}")));
    }
    let at: Vec<String> = sites.into_iter().filter(|s| s.ends_with(&format!("_X{x}Y{y}"))).collect();
    // nextpnr names BRAM sites by their RAM variants; prjxray by the primary
    // types: RAMBFIFO36E1, and FIFO18E1 for the lower RAMB18.
    let alias = match prefix.as_str() {
        "RAMB36" => "RAMBFIFO36",
        "RAMB18" => "FIFO18",
        p => p,
    };
    at.iter().find(|s| s.starts_with(&prefix) || s.starts_with(alias)).or_else(|| (at.len() == 1).then(|| &at[0])).cloned()
}

/// Find the meta variant and bel for a nextpnr bel string (`A6LUT`,
/// `IOB33S.PAD`, `OUTBUF~IOB33`, `RAMB36E1~RAMB36E1`).
fn find_bel(vars: &HashMap<String, Variant>, primary: &str, spec: &str) -> Option<(String, String)> {
    let (body, hint) = match spec.split_once('~') {
        Some((b, h)) => (b, Some(h)),
        None => (spec, None),
    };
    let (vhint, bel) = match body.rsplit_once('.') {
        Some((v, b)) => (Some(v), b),
        None => (None, body),
    };
    let mut order: Vec<&str> = Vec::new();
    for h in [hint, vhint, Some(primary)].into_iter().flatten() {
        if vars.contains_key(h) && !order.contains(&h) {
            order.push(h);
        }
    }
    let mut rest: Vec<&str> = vars.keys().map(String::as_str).filter(|k| !order.contains(k)).collect();
    rest.sort();
    order.extend(rest);
    order.into_iter().find(|v| vars[*v].bels.contains_key(bel)).map(|v| (v.to_string(), bel.to_string()))
}

fn is_lut_bel(ty: &str) -> bool {
    ty.starts_with("LUT")
}

fn is_mem_cell(c: &Cell) -> bool {
    let a = &c.attrs;
    a.contains_key("X_LUT_AS_DRAM")
        || a.contains_key("X_LUT_AS_SRL")
        || a.get("X_ORIG_TYPE").and_then(Value::as_str).is_some_and(|t| t.starts_with("RAM") || t.starts_with("SRL"))
}

/// Terminals two or more nets claim: a source or fixed sink node, with the
/// nets on it. Any entry is a fault in the import (or an illegal placement):
/// no routing can give one pin to two nets.
pub fn pin_conflicts(nets: &[Net]) -> Vec<(u32, Vec<usize>)> {
    let mut on: HashMap<u32, Vec<usize>> = HashMap::new();
    for (i, n) in nets.iter().enumerate() {
        for &t in std::iter::once(&n.source).chain(&n.sinks) {
            let v = on.entry(t).or_default();
            if v.last() != Some(&i) {
                v.push(i);
            }
        }
    }
    let mut out: Vec<(u32, Vec<usize>)> = on.into_iter().filter(|(_, v)| v.len() > 1).collect();
    out.sort_unstable();
    out
}

/// Resolve a placed design's nets to site pins on `g`.
pub fn import(g: &Graph, sites: &HashMap<String, (String, String)>, meta: &mut Meta, placed: &Value, perm: Perm) -> R<Import> {
    let top = top_module(placed)?;
    let cells_v = top["cells"].as_object().ok_or("no cells")?;

    let abs_of: HashMap<(String, String), String> = sites.iter().map(|(abs, (t, rel))| ((t.clone(), rel.clone()), abs.clone())).collect();
    let mut tile_at: HashMap<(i32, i32), String> = HashMap::new();
    for t in g.tile_names() {
        if let Some(xy) = g.tile_xy(t) {
            tile_at.insert(xy, t.to_string());
        }
    }

    let mut imp = Import::default();
    let mut cells: Vec<Cell> = Vec::new();
    let mut cell_names: Vec<String> = Vec::new();
    // net bit -> driver pin, sink pins
    let mut drivers: HashMap<i64, Pin> = HashMap::new();
    let mut sinks: HashMap<i64, Vec<Pin>> = HashMap::new();
    let mut skip_net: HashMap<i64, &'static str> = HashMap::new();
    let mut const_value: HashMap<i64, bool> = HashMap::new();
    // Bel pins the site model lacks, with their nets: a fractured LUT's 5LUT
    // has no A6, which nextpnr ties high; only a signal net there is a fault.
    let mut missing_pins: Vec<(i64, String)> = Vec::new();
    let mut variants: HashMap<(String, String), Variant> = HashMap::new();
    let mut site_cells: HashMap<(String, String), Vec<(String, usize)>> = HashMap::new(); // (tile, site) -> (bel, cell)

    for (name, c) in cells_v {
        let ty = c["type"].as_str().unwrap_or("").to_string();
        let attrs = c["attributes"].as_object().cloned().unwrap_or_default();
        let ci = cells.len();
        cells.push(Cell { attrs: attrs.clone() });
        cell_names.push(name.clone());
        let conns = c["connections"].as_object().cloned().unwrap_or_default();
        let dirs = c["port_directions"].as_object().cloned().unwrap_or_default();
        let bits = |p: &str| -> Vec<i64> { conns.get(p).and_then(Value::as_array).into_iter().flatten().filter_map(Value::as_i64).collect() };

        let mark = match ty.as_str() {
            "PSEUDO_GND" | "PSEUDO_VCC" => Some("constant"),
            "BUFGCTRL" | "BUFG" | "BUFHCE" | "BUFH" => Some("clock"),
            "PAD" => Some("pad"),
            _ => None,
        };
        if let Some(why) = mark {
            for (p, d) in &dirs {
                if why == "pad" || d.as_str() == Some("output") {
                    for b in bits(p) {
                        skip_net.entry(b).or_insert(why);
                        if why == "constant" {
                            const_value.insert(b, ty == "PSEUDO_VCC");
                        }
                    }
                }
            }
            // A clock buffer is a placed cell: its pins are resolved.
            if why != "clock" {
                continue;
            }
        }

        let Some(bel_s) = attrs.get("NEXTPNR_BEL").and_then(Value::as_str) else {
            imp.unresolved.add("unplaced", name.clone());
            continue;
        };
        let located = match parse_bel(bel_s) {
            Some((xy, np_site, bel_spec)) => tile_at.get(&xy).map(|t| (t.clone(), np_site, bel_spec)),
            // `{SITE}/{BEL}` by absolute site name.
            None => bel_s.split_once('/').and_then(|(abs, bel)| sites.get(abs).map(|(t, rel)| (t.clone(), rel.clone(), bel.to_string()))),
        };
        let Some((tile, np_site, bel_spec)) = located else {
            imp.unresolved.add("bel_name", bel_s.to_string());
            continue;
        };
        let Some(site) = graph_site(g, &tile, &np_site) else {
            imp.unresolved.add("site", format!("{tile} {bel_s}"));
            continue;
        };
        let primary = site.split("_X").next().unwrap_or("").to_string();
        let Some(vars) = meta.site_type(&primary) else {
            imp.unresolved.add("site_type", primary.clone());
            continue;
        };
        let Some((var, bel)) = find_bel(vars, &primary, &bel_spec) else {
            imp.unresolved.add("bel", format!("{primary} {bel_spec}"));
            continue;
        };
        let key = (primary.clone(), var.clone());
        if !variants.contains_key(&key) {
            variants.insert(key.clone(), vars[&var].clone());
        }
        let v = &variants[&key];
        site_cells.entry((tile.clone(), site.clone())).or_default().push((bel.clone(), ci));

        for (p, d) in &dirs {
            let bs = bits(p);
            if bs.is_empty() {
                continue;
            }
            let Some(wire) = v.bels[&bel].get(p.as_str()).cloned() else {
                missing_pins.extend(bs.iter().map(|&b| (b, format!("{primary}.{var}.{bel}.{p}"))));
                continue;
            };
            for b in bs {
                let pin = Pin { tile: tile.clone(), site: site.clone(), np_site: np_site.clone(), var: key.clone(), bel: bel.clone(), port: p.clone(), wire: wire.clone(), cell: ci };
                match d.as_str() {
                    Some("output") => {
                        drivers.insert(b, pin);
                    }
                    _ => sinks.entry(b).or_default().push(pin),
                }
            }
        }
    }

    // LUT letters: whole, fractured, or holding memory (fixed pins).
    #[derive(Default, Clone, Copy)]
    struct Letter {
        has6: bool,
        has5: bool,
        mem: bool,
    }
    let mut letters: HashMap<(String, String, char), Letter> = HashMap::new();
    let mut mem_sites: HashSet<(String, String)> = HashSet::new();
    for ((tile, site), list) in &site_cells {
        for (bel, ci) in list {
            let key = (tile.clone(), site.clone(), bel.chars().next().unwrap_or('?'));
            let l = letters.entry(key).or_default();
            if bel.ends_with("6LUT") {
                l.has6 = true;
            } else if bel.ends_with("5LUT") {
                l.has5 = true;
            } else {
                continue;
            }
            if is_mem_cell(&cells[*ci]) {
                l.mem = true;
                mem_sites.insert((tile.clone(), site.clone()));
            }
        }
    }
    for ((t, s, _), l) in &letters {
        let kind = if l.mem || mem_sites.contains(&(t.clone(), s.clone())) {
            "memory_slice"
        } else if l.has6 && l.has5 {
            "fractured"
        } else {
            "whole"
        };
        *imp.luts.entry(kind.into()).or_default() += 1;
    }
    // LUT pins tied to constants: kept out of every group on that letter.
    let mut const_pins: HashSet<(String, String, char, u8)> = HashSet::new();
    for (b, list) in &sinks {
        if skip_net.get(b) != Some(&"constant") {
            continue;
        }
        for p in list {
            let v = &variants[&p.var];
            if is_lut_bel(&v.bel_type[&p.bel]) {
                if let Some(k) = p.port.strip_prefix('A').and_then(|k| k.parse::<u8>().ok()) {
                    const_pins.insert((p.tile.clone(), p.site.clone(), p.bel.chars().next().unwrap_or('?'), k));
                }
            }
        }
    }

    for (b, what) in missing_pins {
        if skip_net.get(&b) != Some(&"constant") {
            imp.unresolved.add("bel_pin", what);
        }
    }

    let site_pin = |g: &Graph, p: &Pin, pin: &str, un: &mut Unresolved| -> Option<u32> {
        let v = &variants[&p.var];
        let primary = v.pins.get(pin).map(|x| x.1.clone()).unwrap_or_else(|| pin.to_string());
        let n = g.site_pin(&p.tile, &p.site, &primary);
        if n.is_none() {
            un.add("site_pin", format!("{}/{}.{primary}", p.tile, p.site));
        }
        n
    };

    let mut bits: Vec<i64> = sinks.keys().copied().collect();
    bits.sort_unstable();

    // Input site pins each external sink can enter by, nearest first. Some
    // have a choice (a SLICEM LUT's write enable comes through WE or CE);
    // a pin that is some other sink's only way in is not offered to the
    // rest, and pins are handed out first come, so two nets never share one.
    // The pin named for the bel pin comes first (WE before CE, as nextpnr
    // takes it): the site configuration a FASM keeps from nextpnr assumes it.
    let entries = |s: &Pin| -> Vec<String> {
        let sv = &variants[&s.var];
        let mut v: Vec<String> = walk(&s.wire, &sv.back).iter().filter_map(|w| sv.in_pin.get(w).cloned()).collect();
        v.sort_by_key(|c| c != &s.port);
        v
    };
    let internal = |s: &Pin, drv: &Pin| -> bool {
        s.tile == drv.tile && s.site == drv.site && s.var == drv.var && walk(&s.wire, &variants[&s.var].back).contains(&drv.wire)
    };
    let mut forced: HashMap<(String, String, String), i64> = HashMap::new();
    for &b in &bits {
        let (true, Some(drv)) = (skip_net.get(&b).is_none_or(|w| *w == "clock"), drivers.get(&b)) else { continue };
        for s in &sinks[&b] {
            if internal(s, drv) {
                continue;
            }
            if let [only] = &entries(s)[..] {
                forced.entry((s.tile.clone(), s.site.clone(), only.clone())).or_insert(b);
            }
        }
    }
    let mut claimed: HashMap<(String, String, String), i64> = HashMap::new();

    for b in bits {
        let class = match skip_net.get(&b) {
            None => NetClass::Signal,
            Some(&"clock") => NetClass::GlobalClock,
            Some(&"constant") => {
                // Each input is driven inside its site when the site has
                // that constant (a routing-bel path from a constant bel),
                // else from a tie (`constant_nets`).
                let val = const_value.get(&b).copied().unwrap_or(false);
                for s in sinks.get(&b).into_iter().flatten() {
                    let sv = &variants[&s.var];
                    if let Some(cw) = walk(&s.wire, &sv.back).into_iter().find(|w| sv.const_wire.get(w) == Some(&val)) {
                        if let Some(w) = site_path(&cw, &s.wire, &sv.fwd) {
                            note_muxes(&mut imp.site_muxes, &mut imp.unresolved, &s.tile, &s.site, sv, &w);
                            imp.const_internal += 1;
                            continue;
                        }
                    }
                    let cands = entries(s);
                    let free = |c: &String| {
                        let k = (s.tile.clone(), s.site.clone(), c.clone());
                        forced.get(&k).is_none_or(|&o| o == b) && claimed.get(&k).is_none_or(|&o| o == b)
                    };
                    let Some(pin) = cands.iter().find(|c| free(c)).or(cands.first()).cloned() else {
                        imp.unresolved.add("no_input_pin", format!("{}/{} {}.{} (constant)", s.tile, s.site, s.bel, s.port));
                        continue;
                    };
                    claimed.entry((s.tile.clone(), s.site.clone(), pin.clone())).or_insert(b);
                    let Some(n) = site_pin(g, s, &pin, &mut imp.unresolved) else { continue };
                    let entry_wire = sv.pins.get(&pin).map(|x| x.0.clone()).unwrap_or_else(|| pin.clone());
                    if let Some(w) = site_path(&entry_wire, &s.wire, &sv.fwd) {
                        note_muxes(&mut imp.site_muxes, &mut imp.unresolved, &s.tile, &s.site, sv, &w);
                    }
                    imp.const_sinks.push((val, n));
                }
                *imp.skipped.entry("constant".into()).or_default() += 1;
                continue;
            }
            Some(why) => {
                *imp.skipped.entry((*why).into()).or_default() += 1;
                continue;
            }
        };
        let Some(drv) = drivers.get(&b) else {
            *imp.skipped.entry("undriven".into()).or_default() += 1;
            continue;
        };
        let name = format!("{}.{}", cell_names[drv.cell], drv.port);
        let dv = &variants[&drv.var];
        let reach_out = walk(&drv.wire, &dv.fwd);

        // Exits: output site pins the driver reaches, nearest first.
        let exits: Vec<String> = reach_out.iter().filter_map(|w| dv.out_pin.get(w).cloned()).collect();

        let mut fixed: Vec<(String, u32)> = Vec::new(); // (exit kind, node)
        let mut groups: HashMap<(String, String, char), LutGroup> = HashMap::new();
        let mut paths: Vec<(u32, SinkPath)> = Vec::new();
        for s in &sinks[&b] {
            let sv = &variants[&s.var];
            if internal(s, drv) {
                imp.internal_sinks += 1;
                if let Some(w) = site_path(&drv.wire, &s.wire, &sv.fwd) {
                    note_muxes(&mut imp.site_muxes, &mut imp.unresolved, &s.tile, &s.site, sv, &w);
                }
                continue;
            }
            let letter = s.bel.chars().next().unwrap_or('?');
            let lut_k = if is_lut_bel(&sv.bel_type[&s.bel]) { s.port.strip_prefix('A').and_then(|k| k.parse::<u8>().ok()) } else { None };
            let mem = mem_sites.contains(&(s.tile.clone(), s.site.clone()));
            if let (Some(k), false, true) = (lut_k, mem, perm != Perm::None) {
                let l = letters.get(&(s.tile.clone(), s.site.clone(), letter)).copied().unwrap_or_default();
                let fractured = l.has6 && l.has5;
                let top: u8 = match perm {
                    Perm::A6 if !fractured => 6,
                    _ => 5,
                };
                if k <= top {
                    for j in 1..=top {
                        let pin = format!("{letter}{j}");
                        if let (Some(n), Some(tw)) = (g.site_pin(&s.tile, &s.site, &pin), g.site_pin_wire(&s.tile, &s.site, &pin)) {
                            paths.push((n, SinkPath { tile: s.tile.clone(), np_site: s.np_site.clone(), tile_wire: tw, site_wires: vec![s.wire.clone()] }));
                        }
                    }
                    let grp = match groups.entry((s.tile.clone(), s.site.clone(), letter)) {
                        Entry::Occupied(e) => e.into_mut(),
                        Entry::Vacant(e) => {
                            let mut grp = LutGroup::default();
                            for j in 1..=top {
                                if const_pins.contains(&(s.tile.clone(), s.site.clone(), letter, j)) {
                                    continue;
                                }
                                if let Some(n) = site_pin(g, s, &format!("{letter}{j}"), &mut imp.unresolved) {
                                    grp.nodes.push(n);
                                    grp.pins.push(j);
                                }
                            }
                            e.insert(grp)
                        }
                    };
                    if !grp.from.contains(&k) {
                        grp.from.push(k);
                    }
                    continue;
                }
            }
            // Fixed: an input site pin a routing-bel path leads back to, the
            // nearest no other net holds.
            let cands = entries(s);
            let free = |c: &String| {
                let k = (s.tile.clone(), s.site.clone(), c.clone());
                forced.get(&k).is_none_or(|&o| o == b) && claimed.get(&k).is_none_or(|&o| o == b)
            };
            let pin = cands.iter().find(|c| free(c)).or(cands.first()).cloned();
            let Some(pin) = pin else {
                imp.unresolved.add("no_input_pin", format!("{}/{} {}.{}", s.tile, s.site, s.bel, s.port));
                continue;
            };
            claimed.entry((s.tile.clone(), s.site.clone(), pin.clone())).or_insert(b);
            let Some(n) = site_pin(g, s, &pin, &mut imp.unresolved) else { continue };
            let sv = &variants[&s.var];
            let (entry_wire, primary) = sv.pins.get(&pin).map(|x| (x.0.clone(), x.1.clone())).unwrap_or_else(|| (pin.clone(), pin.clone()));
            if let Some(tw) = g.site_pin_wire(&s.tile, &s.site, &primary) {
                let site_wires = site_path(&entry_wire, &s.wire, &sv.fwd).unwrap_or_else(|| vec![entry_wire.clone()]);
                note_muxes(&mut imp.site_muxes, &mut imp.unresolved, &s.tile, &s.site, sv, &site_wires);
                paths.push((n, SinkPath { tile: s.tile.clone(), np_site: s.np_site.clone(), tile_wire: tw, site_wires }));
            }
            let kind = if pin == "CIN" { "COUT" } else { "general" };
            fixed.push((kind.into(), n));
            imp.fixed_sinks += 1;
        }
        if fixed.is_empty() && groups.is_empty() {
            continue;
        }

        // One routed net per exit: carry-in sinks leave through COUT, the
        // rest through the nearest other output pin.
        let general_exit = exits.iter().find(|e| e.as_str() != "COUT").cloned();
        let cout_exit = exits.iter().find(|e| e.as_str() == "COUT").cloned();
        let mut by_exit: BTreeMap<String, ExitSinks> = BTreeMap::new();
        for (kind, n) in fixed {
            let exit = if kind == "COUT" { cout_exit.clone().or(general_exit.clone()) } else { general_exit.clone() };
            let Some(exit) = exit else {
                imp.unresolved.add("no_output_pin", format!("{}/{} {}.{}", drv.tile, drv.site, drv.bel, drv.port));
                continue;
            };
            by_exit.entry(exit).or_default().0.push(n);
        }
        if !groups.is_empty() {
            match &general_exit {
                Some(exit) => {
                    let mut gs: Vec<_> = groups.into_iter().collect();
                    gs.sort_by(|a, b| a.0.cmp(&b.0));
                    for ((tile, site, letter), grp) in gs {
                        *imp.group_sizes.entry(grp.nodes.len()).or_default() += 1;
                        let e = by_exit.entry(exit.clone()).or_default();
                        e.1.push(grp.nodes);
                        e.2.push(LutSlot { tile, site, letter, pins: grp.pins, from: grp.from });
                    }
                }
                None => imp.unresolved.add("no_output_pin", format!("{}/{} {}.{}", drv.tile, drv.site, drv.bel, drv.port)),
            }
        }
        for (exit, (mut fixed, groups, slots)) in by_exit {
            let Some(src) = site_pin(g, drv, &exit, &mut imp.unresolved) else { continue };
            let primary = dv.pins.get(&exit).map(|x| x.1.clone()).unwrap_or_else(|| exit.clone());
            let exit_wire = dv.pins.get(&exit).map(|x| x.0.clone()).unwrap_or_else(|| exit.clone());
            let source = SourcePath {
                tile: drv.tile.clone(),
                np_site: drv.np_site.clone(),
                abs_site: abs_of.get(&(drv.tile.clone(), drv.np_site.clone())).cloned(),
                site_wires: site_path(&drv.wire, &exit_wire, &dv.fwd).unwrap_or_else(|| vec![drv.wire.clone()]),
                tile_wire: g.site_pin_wire(&drv.tile, &drv.site, &primary).unwrap_or_default(),
            };
            note_muxes(&mut imp.site_muxes, &mut imp.unresolved, &drv.tile, &drv.site, dv, &source.site_wires);
            fixed.sort_unstable();
            fixed.dedup();
            imp.nets.push(Net { source: src, sinks: fixed, pin_groups: groups, edges: Vec::new(), class });
            imp.names.push(format!("{name}@{exit}"));
            imp.bits.push(b);
            imp.sources.push(source);
            imp.sink_paths.push(paths.clone());
            imp.group_slots.push(slots);
        }
    }
    Ok(imp)
}

/// Nets for the import's constant inputs (`Import::const_sinks`).
///
/// An input whose interconnect mux reads VCC when unset (a `default`
/// pseudo-pip from the tile's VCC_WIRE, then hard wires) needs no pip: it
/// goes on a `NetClass::Default` net per tie, which the router keeps clear
/// and writes nothing for. Every other input is given to a VCC_WIRE or
/// GND_WIRE tie of its value that reaches it within three pips, hard wires
/// not counted (GND_WIRE, GFAN, IMUX; or a VCC default into an interconnect
/// input, then the pip of a clock tile's control mux), and each tie drives
/// one `NetClass::Constant` net the router routes. Returns the nets and the
/// inputs no tie reaches.
pub fn constant_nets(g: &Graph, imp: &Import) -> (Vec<Net>, Vec<u32>) {
    let mut want: HashMap<u32, bool> = imp.const_sinks.iter().map(|&(v, n)| (n, v)).collect();
    let near: HashSet<(i16, i16)> = want.keys().map(|&n| g.node_xy(n)).collect();
    let close = |n: u32| {
        let (x, y) = g.node_xy(n);
        near.iter().any(|&(a, b)| (a - x).abs() <= 2 && (b - y).abs() <= 2)
    };
    let mut nets = Vec::new();
    // Default reads: VCC_WIRE -default-> mux, then hard wires to the input.
    for tie in g.nodes_with_wire(&["VCC_WIRE".to_string()]) {
        if !close(tie) {
            continue;
        }
        let mut sinks = Vec::new();
        let mut edges = Vec::new();
        for &(d, t, pi) in g.out_edges(tie) {
            if g.edge_kind(t, pi) != EdgeKind::Default {
                continue;
            }
            let mut path = vec![Edge { src: tie, dst: d, tile: t, pi, kind: EdgeKind::Default }];
            let mut at = d;
            let mut hit = want.get(&at) == Some(&true);
            while !hit {
                let next: Vec<_> = g.out_edges(at).iter().filter(|&&(_, t2, p2)| g.edge_kind(t2, p2) == EdgeKind::Fixed).copied().collect();
                let [(d2, t2, p2)] = next[..] else { break };
                path.push(Edge { src: at, dst: d2, tile: t2, pi: p2, kind: EdgeKind::Fixed });
                at = d2;
                hit = want.get(&at) == Some(&true);
                if path.len() > 3 {
                    break;
                }
            }
            if hit {
                want.remove(&at);
                sinks.push(at);
                edges.extend(path);
            }
        }
        if !sinks.is_empty() {
            sinks.sort_unstable();
            nets.push(Net { source: tie, sinks, pin_groups: Vec::new(), edges, class: NetClass::Default });
        }
    }
    let mut by_tie: BTreeMap<u32, Vec<u32>> = BTreeMap::new();
    // Three pips reach nearly every input; the rest (an MMCM's CLKIN2 from
    // GND through GFAN, FAN_ALT, FAN_BOUNCE and a CLK input) search deeper
    // from the ties near them.
    for (depth, radius) in [(3, None), (6, Some(6i16))] {
        let near: HashSet<(i16, i16)> = want.keys().map(|&n| g.node_xy(n)).collect();
        for (val, wire) in [(true, "VCC_WIRE"), (false, "GND_WIRE")] {
            if !want.values().any(|&v| v == val) {
                continue;
            }
            for tie in g.nodes_with_wire(&[wire.to_string()]) {
                if let Some(r) = radius {
                    let (x, y) = g.node_xy(tie);
                    if !near.iter().any(|&(a, b)| (a - x).abs() <= r && (b - y).abs() <= r) {
                        continue;
                    }
                }
                let mut seen = HashSet::from([tie]);
                let mut frontier = vec![tie];
                for level in 0..depth {
                    let mut next = Vec::new();
                    for n in frontier {
                        for &(d, t, pi) in g.out_edges(n) {
                            let ok = match g.edge_kind(t, pi) {
                                EdgeKind::Feature => true,
                                // A VCC tie's default into an interconnect input.
                                EdgeKind::Default => val && level == 0,
                                _ => false,
                            };
                            if !ok || !seen.insert(d) {
                                continue;
                            }
                            // The pip's node and the hard wires on from it.
                            let mut hard = vec![d];
                            let mut i = 0;
                            while i < hard.len() && hard.len() < 16 {
                                for &(d2, t2, p2) in g.out_edges(hard[i]) {
                                    if g.edge_kind(t2, p2) == EdgeKind::Fixed && seen.insert(d2) {
                                        hard.push(d2);
                                    }
                                }
                                i += 1;
                            }
                            for h in hard {
                                if want.get(&h) == Some(&val) {
                                    want.remove(&h);
                                    by_tie.entry(tie).or_default().push(h);
                                }
                                next.push(h);
                            }
                        }
                    }
                    frontier = next;
                }
            }
        }
    }
    nets.extend(by_tie.into_iter().map(|(tie, mut sinks)| {
        sinks.sort_unstable();
        sinks.dedup();
        Net { source: tie, sinks, pin_groups: Vec::new(), edges: Vec::new(), class: NetClass::Constant }
    }));
    let mut missed: Vec<u32> = want.into_keys().collect();
    missed.sort_unstable();
    (nets, missed)
}

/// Record the routing-bel settings a site path needs. Two paths that want
/// one routing bel on different inputs cannot both be built: counted in
/// `unresolved` as `site_mux`.
fn note_muxes(
    muxes: &mut BTreeMap<(String, String), BTreeMap<String, String>>,
    un: &mut Unresolved,
    tile: &str,
    site: &str,
    v: &Variant,
    wires: &[String],
) {
    for pair in wires.windows(2) {
        let Some((bel, pin)) = v.pip_bel.get(&(pair[0].clone(), pair[1].clone())) else { continue };
        let m = muxes.entry((tile.to_string(), site.to_string())).or_default();
        match m.get(bel) {
            Some(p) if p != pin => un.add("site_mux", format!("{tile}/{site} {bel}: {p} and {pin}")),
            Some(_) => {}
            None => {
                m.insert(bel.clone(), pin.clone());
            }
        }
    }
}

/// The FASM line a slice routing-bel setting takes, as nextpnr-xilinx
/// writes it (`write_routing_bel`): `None` for a routing bel this does not
/// manage, `Some(None)` for a managed one whose setting is the default (no
/// line). Managed: the FF data muxes (`AFFMUX`, `A5FFMUX`), the output muxes
/// (`AOUTMUX`), the carry chain's `ACY0` and `PRECYINIT`, and `WEMUX`.
fn mux_line(bel: &str, pin: &str) -> Option<Option<String>> {
    let letter = bel.len() > 1 && "ABCD".contains(&bel[..1]);
    let rest = if letter { &bel[1..] } else { bel };
    match rest {
        "FFMUX" | "5FFMUX" | "OUTMUX" if letter => {
            let pin = match pin {
                "CARRY4_MUX" => "CY",
                "CARRY4_XOR" => "XOR",
                p => p,
            };
            Some(Some(format!("{bel}.{pin}")))
        }
        "CY0" if letter => Some((pin == "O5").then(|| format!("CARRY4.{bel}"))),
        "PRECYINIT" => Some(Some(format!(
            "PRECYINIT.{}",
            match pin {
                "0" => "C0",
                "1" => "C1",
                p => p,
            }
        ))),
        "WEMUX" => Some((pin == "CE").then(|| "WEMUX.CE".to_string())),
        _ => None,
    }
}

/// What `apply_site_muxes` did.
#[derive(Debug, Default, Clone, Copy)]
pub struct SiteMuxes {
    /// Routing bels set from the import's paths.
    pub set: usize,
    /// Lines written (a default setting writes none).
    pub written: usize,
    /// The FASM's own lines for those routing bels, dropped.
    pub dropped: usize,
}

/// Put the import's slice routing-bel settings (`Import::site_muxes`) into
/// `fasm`: the FASM's lines for every routing bel the import sets are
/// replaced; lines for routing bels it does not touch (constant inputs, for
/// one) are kept.
pub fn apply_site_muxes(fasm: &str, imp: &Import) -> (String, SiteMuxes) {
    let mut st = SiteMuxes::default();
    let mut ours: HashSet<(String, String, String)> = HashSet::new();
    let mut lines: Vec<String> = Vec::new();
    for ((tile, site), muxes) in &imp.site_muxes {
        for (bel, pin) in muxes {
            let Some(line) = mux_line(bel, pin) else { continue };
            st.set += 1;
            ours.insert((tile.clone(), site.clone(), bel.clone()));
            if let Some(l) = line {
                lines.push(format!("{tile}.{site}.{l}"));
            }
        }
    }
    let mut out = String::with_capacity(fasm.len());
    for l in fasm.lines() {
        let f = l.split('#').next().unwrap_or("").trim();
        let parts: Vec<&str> = f.split('.').collect();
        let bel = match parts[..] {
            [_, _, "CARRY4", b] => Some(b),
            [_, _, b, ..] => Some(b),
            _ => None,
        };
        if let (Some(bel), true) = (bel, parts.len() >= 3) {
            let key = (parts[0].to_string(), parts[1].to_string(), bel.to_string());
            if ours.contains(&key) && mux_line(bel, "").is_some() {
                st.dropped += 1;
                continue;
            }
        }
        out.push_str(l);
        out.push('\n');
    }
    for l in lines {
        out.push_str(&l);
        out.push('\n');
        st.written += 1;
    }
    (out, st)
}

/// The sinks a net reaches through one exit: fixed sink nodes, pin groups,
/// and the LUT behind each group.
type ExitSinks = (Vec<u32>, Vec<Vec<u32>>, Vec<LutSlot>);

/// One LUT's pin group under construction: the allowed pins' nodes and
/// numbers, and the bel pins the net was placed on.
#[derive(Default)]
struct LutGroup {
    nodes: Vec<u32>,
    pins: Vec<u8>,
    from: Vec<u8>,
}

/// Write `routes` (one hop list per `imp.nets` entry, more entries ignored)
/// into the design JSON `v` as nextpnr `ROUTING` attributes: from each
/// driver's bel pin through its site to the output pin, the tile pips, and
/// into each reached sink's site to its bel pins, every pip at strength 3
/// (placer). router2 takes wires bound that way as reserved for their net
/// and extends them; any stronger binding makes them unavailable even to
/// their own net. nextpnr names a pip TILE/DST/SRC and a wire TILE/WIRE,
/// the tile by its grid position (X55Y33); site wires by their tile-local
/// name (SLICE_X0Y0.AQ): it writes them as SITEWIRE/SITE/WIRE but cannot
/// read that back.
///
/// Returns (nets given a ROUTING, the routing per net bit, hops whose tile
/// has no grid position).
pub fn write_routing(g: &Graph, imp: &Import, routes: &[Vec<Hop>], v: &mut Value) -> R<(usize, HashMap<i64, Vec<String>>, usize)> {
    // nextpnr names a pip TILE/DST/SRC and a wire TILE/WIRE, the tile
    // by its grid position (X55Y33).
    let xy = |tile: &str| g.tile_xy(tile).map(|(x, y)| format!("X{x}Y{y}"));
    let mut routing: HashMap<i64, Vec<String>> = HashMap::new();
    let mut unnamed = 0usize;
    for (i, hops) in routes.iter().take(imp.nets.len()).enumerate() {
        let entry = routing.entry(imp.bits[i]).or_default();
        // From the driver's bel pin: its site wire, the site's routing-bel
        // pips to the output pin, and the pip out to the tile wire, so
        // nextpnr sees one tree from the net's source.
        let sp = &imp.sources[i];
        let Some(t) = xy(&sp.tile) else {
            unnamed += 1;
            continue;
        };
        // nextpnr writes site wires as SITEWIRE/SITE/WIRE but reads only
        // TILE/WIRE, the site wire's tile-local name (SLICE_X0Y0.AQ).
        let site_wire = |w: &str| format!("{t}/{}.{w}", sp.np_site);
        let np = &sp.np_site;
        entry.push(format!("{};;3", site_wire(&sp.site_wires[0])));
        for pair in sp.site_wires.windows(2) {
            entry.push(format!("{};{t}/{np}.{}/{np}.{};3", site_wire(&pair[1]), pair[1], pair[0]));
        }
        let last = sp.site_wires.last().cloned().unwrap_or_default();
        entry.push(format!("{t}/{};{t}/{}/{np}.{last};3", sp.tile_wire, sp.tile_wire));
        // Into each sink's site: the pip from the site pin's tile wire to
        // the first site wire (for a LUT, the permutation onto the cell's
        // input), then the site's routing-bel pips to the bel pin.
        let reached: HashSet<u32> = hops.iter().map(|h| h.dst).collect();
        for (node, sk) in &imp.sink_paths[i] {
            if !reached.contains(node) {
                continue;
            }
            let Some(st) = xy(&sk.tile) else { continue };
            let np = &sk.np_site;
            let first = &sk.site_wires[0];
            let mut chain = vec![format!("{st}/{np}.{first};{st}/{np}.{first}/{};3", sk.tile_wire)];
            for pair in sk.site_wires.windows(2) {
                chain.push(format!("{st}/{np}.{};{st}/{np}.{}/{np}.{};3", pair[1], pair[1], pair[0]));
            }
            for c in chain {
                if !entry.contains(&c) {
                    entry.push(c);
                }
            }
        }
        for h in hops {
            let p = g.edge_pip(h.tile, h.pi);
            let Some(t) = xy(&p.tile) else {
                unnamed += 1;
                continue;
            };
            entry.push(format!("{t}/{};{t}/{}/{};3", p.dst, p.dst, p.src));
        }
    }
    // nextpnr binds each wire to one net once: a net routed through two
    // exits repeats its source chain, and an overused node would bind a
    // wire to two nets. Keep the first binding of every wire.
    let mut bound_by: HashMap<String, i64> = HashMap::new();
    let mut bits: Vec<i64> = routing.keys().copied().collect();
    bits.sort_unstable();
    for b in bits {
        let r = routing.get_mut(&b).unwrap_or_else(|| unreachable!());
        r.retain(|e| {
            let wire = e.split(';').next().unwrap_or("").to_string();
            match bound_by.entry(wire) {
                Entry::Vacant(v) => {
                    v.insert(b);
                    true
                }
                Entry::Occupied(_) => false,
            }
        });
    }
    let mut set = 0usize;
    let mods = v["modules"].as_object_mut().ok_or("no modules")?;
    for m in mods.values_mut() {
        let Some(names) = m["netnames"].as_object_mut() else { continue };
        for (_, n) in names.iter_mut() {
            let Some(b) = n["bits"].get(0).and_then(Value::as_i64) else { continue };
            if let Some(r) = routing.get(&b) {
                n["attributes"]["ROUTING"] = Value::String(r.join(";"));
                set += 1;
            }
        }
    }
    Ok((set, routing, unnamed))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn site_mux_lines() {
        assert_eq!(mux_line("BFFMUX", "BX"), Some(Some("BFFMUX.BX".into())));
        assert_eq!(mux_line("CFFMUX", "CARRY4_XOR"), Some(Some("CFFMUX.XOR".into())));
        assert_eq!(mux_line("AOUTMUX", "CARRY4_MUX"), Some(Some("AOUTMUX.CY".into())));
        assert_eq!(mux_line("A5FFMUX", "IN_B"), Some(Some("A5FFMUX.IN_B".into())));
        assert_eq!(mux_line("ACY0", "O5"), Some(Some("CARRY4.ACY0".into())));
        assert_eq!(mux_line("ACY0", "AX"), Some(None));
        assert_eq!(mux_line("PRECYINIT", "1"), Some(Some("PRECYINIT.C1".into())));
        assert_eq!(mux_line("WEMUX", "WE"), Some(None));
        assert_eq!(mux_line("WEMUX", "CE"), Some(Some("WEMUX.CE".into())));
        assert_eq!(mux_line("CEUSEDMUX", "IN"), None);
        assert_eq!(mux_line("ADI1MUX", "AI"), None);
    }

    #[test]
    fn site_muxes_replace_their_lines() {
        let mut imp = Import::default();
        let site = ("T".to_string(), "SLICEL_X0".to_string());
        let m = imp.site_muxes.entry(site).or_default();
        m.insert("BFFMUX".into(), "BX".into());
        m.insert("ACY0".into(), "AX".into());
        m.insert("SRUSEDMUX".into(), "IN".into());
        let fasm = "T.SLICEL_X0.BFFMUX.O6\nT.SLICEL_X0.CARRY4.ACY0\nT.SLICEL_X0.CARRY4.BCY0\nT.SLICEL_X0.SRUSEDMUX\nT.SLICEL_X0.AFFMUX.XOR\nT.SLICEL_X1.BFFMUX.O6\n";
        let (out, st) = apply_site_muxes(fasm, &imp);
        assert_eq!((st.set, st.written, st.dropped), (2, 1, 2));
        assert_eq!(out, "T.SLICEL_X0.CARRY4.BCY0\nT.SLICEL_X0.SRUSEDMUX\nT.SLICEL_X0.AFFMUX.XOR\nT.SLICEL_X1.BFFMUX.O6\nT.SLICEL_X0.BFFMUX.BX\n");
    }
}
