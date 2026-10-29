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
use oxibit_fasm::route::Graph;
use serde_json::Value;

use crate::nets::{Net, NetClass};

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
        for (bel, b) in var["bels"].as_object().into_iter().flatten() {
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
        for pip in var["pips"].as_array().into_iter().flatten() {
            let bel = pip["bel"].as_str().unwrap_or("");
            if !rbel.contains(bel) {
                continue; // LUT and carry pass-throughs are not wiring
            }
            let (Some(f), Some(t)) = (pip["from_pin"].as_str(), pip["to_pin"].as_str()) else { continue };
            let (Some(fw), Some(tw)) = (bels[bel].get(f), bels[bel].get(t)) else { continue };
            back.entry(tw.clone()).or_default().push(fw.clone());
            fwd.entry(fw.clone()).or_default().push(tw.clone());
        }
        for v in back.values_mut().chain(fwd.values_mut()) {
            v.sort();
            v.dedup();
        }
        out.insert(name.clone(), Variant { pins, bels, bel_type, back, fwd, in_pin, out_pin });
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
    /// Index into `Import`'s site variants: (primary type, variant).
    var: (String, String),
    bel: String,
    port: String,
    wire: String,
    cell: usize,
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

/// Result of `import`.
#[derive(Debug, Default)]
pub struct Import {
    pub nets: Vec<Net>,
    /// Logical net name of each routed net.
    pub names: Vec<String>,
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

/// Resolve a placed design's nets to site pins on `g`.
pub fn import(g: &Graph, sites: &HashMap<String, (String, String)>, meta: &mut Meta, placed: &Value, perm: Perm) -> R<Import> {
    let top = top_module(placed)?;
    let cells_v = top["cells"].as_object().ok_or("no cells")?;

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
                    }
                }
            }
            continue;
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
                let pin = Pin { tile: tile.clone(), site: site.clone(), var: key.clone(), bel: bel.clone(), port: p.clone(), wire: wire.clone(), cell: ci };
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
    for b in bits {
        if let Some(why) = skip_net.get(&b) {
            *imp.skipped.entry((*why).into()).or_default() += 1;
            continue;
        }
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
        let mut groups: HashMap<(String, String, char), Vec<u32>> = HashMap::new();
        for s in &sinks[&b] {
            let sv = &variants[&s.var];
            if s.tile == drv.tile && s.site == drv.site && s.var == drv.var && walk(&s.wire, &sv.back).contains(&drv.wire) {
                imp.internal_sinks += 1;
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
                    if let Entry::Vacant(e) = groups.entry((s.tile.clone(), s.site.clone(), letter)) {
                        let mut pins = Vec::new();
                        for j in 1..=top {
                            if const_pins.contains(&(s.tile.clone(), s.site.clone(), letter, j)) {
                                continue;
                            }
                            if let Some(n) = site_pin(g, s, &format!("{letter}{j}"), &mut imp.unresolved) {
                                pins.push(n);
                            }
                        }
                        e.insert(pins);
                    }
                    continue;
                }
            }
            // Fixed: the input site pin a routing-bel path leads back to.
            let pin = walk(&s.wire, &sv.back).iter().find_map(|w| sv.in_pin.get(w).cloned());
            let Some(pin) = pin else {
                imp.unresolved.add("no_input_pin", format!("{}/{} {}.{}", s.tile, s.site, s.bel, s.port));
                continue;
            };
            let Some(n) = site_pin(g, s, &pin, &mut imp.unresolved) else { continue };
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
        let mut by_exit: BTreeMap<String, (Vec<u32>, Vec<Vec<u32>>)> = BTreeMap::new();
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
                    for (_, pins) in gs {
                        *imp.group_sizes.entry(pins.len()).or_default() += 1;
                        by_exit.entry(exit.clone()).or_default().1.push(pins);
                    }
                }
                None => imp.unresolved.add("no_output_pin", format!("{}/{} {}.{}", drv.tile, drv.site, drv.bel, drv.port)),
            }
        }
        for (exit, (mut fixed, groups)) in by_exit {
            let Some(src) = site_pin(g, drv, &exit, &mut imp.unresolved) else { continue };
            fixed.sort_unstable();
            fixed.dedup();
            imp.nets.push(Net { source: src, sinks: fixed, pin_groups: groups, edges: Vec::new(), class: NetClass::Signal });
            imp.names.push(format!("{name}@{exit}"));
        }
    }
    Ok(imp)
}
