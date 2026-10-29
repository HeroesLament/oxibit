//! Equivalence of two FASMs of one placed design, at the cells: does every
//! cell input pin see the same source, and does every LUT compute the same
//! function of its sources? `placed_check` compares routing; this judges
//! the design the bits build, routing, site muxes and LUT INITs together.
//!
//! cargo run --release -p oxibit-pnr --example fasm_equiv -- \
//!     DB_ROOT PART META_DIR DESIGN.JSON A.FASM B.FASM
//!
//! DESIGN.JSON is the placed design both FASMs build: it says which cell
//! pins the netlist connects, the only ones compared.
//!
//! Per FASM:
//!
//! * every site pin of every tile either FASM names is traced forward over
//!   the FASM's pips (and fixed edges, and the I/O site route-throughs it
//!   configures); a pin reached from another pin, or from a VCC or GND tie,
//!   is driven by it. A hard edge into a node a set pip drives does not
//!   conduct: it is the default input of a mux (a DSP control pin's tie in
//!   the tile), which the pip overrides;
//! * inside a slice, a wire is followed back through the routing bels the
//!   FASM sets (FF data and output muxes, carry inputs, CE/SR selects, ...;
//!   site models of nextpnr-xilinx-meta) to a site input pin, a constant,
//!   or a cell output. A LUT that only passes one input (a routing LUT) is
//!   followed through to that input.
//!
//! Then every connected cell input pin in a slice (FF D/CE/SR, carry, LUT
//! RAM data, address and write enable, ...) must have the same source in
//! both, every LUT the same
//! O6 and O5 over the sources of A1..A6 (reduced to the sources they depend
//! on), and every input site pin outside the slices the same source.
//!
//! One JSON object on stdout; exit 1 if the two differ. FASM_EQUIV_EXAMPLES
//! sets how many examples of each kind of difference it lists (default 6).

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::path::Path;

use oxibit_fasm::db::Database;
use oxibit_fasm::route::{EdgeKind, Graph, GraphOpts, PipLookup};
use oxibit_pnr::lutinit::{self, Inits, Locator, passes};
use serde_json::{Value, json};

type R<T> = Result<T, String>;

/// One FASM traced: its lines, LUT INITs, each pin's source node, and what
/// was driven twice.
struct View {
    lines: HashSet<String>,
    inits: Inits,
    source: HashMap<u32, u32>,
    shorts: Vec<(u32, Vec<u32>)>,
}

/// Where a cell input's value comes from.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
enum Src {
    Const(bool),
    /// A source node outside the slices (a site output pin, a tie).
    Node(u32),
    /// A cell output in a slice: `tile/site.bel.pin`.
    Bel(String),
    /// Nothing drives it: what was left open.
    Undriven(String),
}

/// One slice site type: its wires, routing bels and cells.
#[derive(Default)]
struct SiteModel {
    /// Site pin -> (wire, is input).
    pin_wire: HashMap<String, (String, bool)>,
    /// Wire -> site pin on it.
    wire_pin: HashMap<String, String>,
    /// Routing bel output wire -> routing bel.
    rbel_out: HashMap<String, String>,
    /// (routing bel, input pin) -> wire.
    rbel_in: HashMap<(String, String), String>,
    /// Routing bel -> its input pins.
    rbel_pins: HashMap<String, Vec<String>>,
    /// Cell output wire -> (bel, pin, bel type).
    bel_out: HashMap<String, (String, String, String)>,
    /// Cell inputs: (bel, pin, wire).
    bel_in: Vec<(String, String, String)>,
}

fn load_model(meta: &Path, site_type: &str) -> R<SiteModel> {
    let path = meta.join(format!("site_type_{site_type}.json"));
    let text = std::fs::read_to_string(&path).map_err(|e| format!("{}: {e}", path.display()))?;
    let v: Value = serde_json::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))?;
    let var = &v[site_type];
    let mut m = SiteModel::default();
    for (pin, p) in var["pins"].as_object().into_iter().flatten() {
        let wire = p["wire"].as_str().unwrap_or("").to_string();
        m.wire_pin.insert(wire.clone(), pin.clone());
        m.pin_wire.insert(pin.clone(), (wire, p["dir"].as_str() == Some("INPUT")));
    }
    for (bel, b) in var["bels"].as_object().into_iter().flatten() {
        let ty = b["type"].as_str().unwrap_or("").to_string();
        let rbel = b["class"].as_str() == Some("RBEL");
        for (pin, p) in b["pins"].as_object().into_iter().flatten() {
            let wire = p["wire"].as_str().unwrap_or("").to_string();
            let input = p["dir"].as_str() == Some("INPUT");
            match (rbel, input) {
                (true, true) => {
                    m.rbel_in.insert((bel.clone(), pin.clone()), wire);
                    m.rbel_pins.entry(bel.clone()).or_default().push(pin.clone());
                }
                (true, false) => {
                    m.rbel_out.insert(wire, bel.clone());
                }
                (false, true) => m.bel_in.push((bel.clone(), pin.clone(), wire)),
                (false, false) => {
                    m.bel_out.insert(wire, (bel.clone(), pin.clone(), ty.clone()));
                }
            }
        }
    }
    for v in m.rbel_pins.values_mut() {
        v.sort();
    }
    m.bel_in.sort();
    Ok(m)
}

/// The bare feature lines of a FASM (no value, no address).
fn feature_lines(fasm: &str) -> HashSet<String> {
    fasm.lines()
        .map(|l| l.split('#').next().unwrap_or("").trim())
        .filter(|f| !f.is_empty() && !f.contains('=') && !f.contains('['))
        .map(String::from)
        .collect()
}

/// Nodes `root` drives in a FASM: over set pips and configured relays, and
/// over hard edges except into a node a set pip drives.
fn reach(g: &Graph, root: u32, lines: &HashSet<String>, set_into: &HashSet<u32>) -> HashSet<u32> {
    let mut seen = HashSet::from([root]);
    let mut stack = vec![root];
    while let Some(n) = stack.pop() {
        for &(d, t, pi) in g.out_edges(n) {
            if seen.contains(&d) {
                continue;
            }
            let take = match g.edge_kind(t, pi) {
                EdgeKind::Fixed => !set_into.contains(&d),
                // An unset interconnect mux reads its default (VCC).
                EdgeKind::Default => !set_into.contains(&d),
                // Relays: only the I/O site route-throughs and the clock
                // tiles' buffers. A slice relay (a FF or LUT passing a
                // signal) is a cell the design uses as such.
                EdgeKind::Relay if !(g.edge_site_through(t, pi) || g.edge_pip(t, pi).tile.starts_with("CLK_")) => false,
                EdgeKind::Feature | EdgeKind::Relay => {
                    let ls = g.edge_pip(t, pi).fasm_lines();
                    !ls.is_empty() && ls.iter().all(|l| lines.contains(l))
                }
            };
            if take {
                seen.insert(d);
                stack.push(d);
            }
        }
    }
    seen
}

/// Nodes a set pip drives in a FASM's lines.
fn set_into(g: &Graph, lines: &HashSet<String>) -> HashSet<u32> {
    lines
        .iter()
        .filter_map(|l| match g.lookup_pip(l) {
            PipLookup::Edge { dst, tile, pi, .. } if g.edge_kind(tile, pi) == EdgeKind::Feature => Some(dst),
            _ => None,
        })
        .collect()
}

/// `FASM_EQUIV_WHY=TILE/WIRE`: the path from the node's source to it in
/// one FASM, on stderr (the edges `reach` takes, breadth first).
fn why(g: &Graph, tag: &str, v: &View, target: u32) {
    let Some(&root) = v.source.get(&target) else {
        eprintln!("why {tag}: {} has no source", g.node_name(target));
        return;
    };
    let into = set_into(g, &v.lines);
    let mut prev: HashMap<u32, (u32, u32, u32)> = HashMap::new();
    let mut queue = std::collections::VecDeque::from([root]);
    let mut seen = HashSet::from([root]);
    while let Some(n) = queue.pop_front() {
        if n == target {
            break;
        }
        for &(d, t, pi) in g.out_edges(n) {
            if seen.contains(&d) {
                continue;
            }
            let k = g.edge_kind(t, pi);
            let take = match k {
                EdgeKind::Fixed | EdgeKind::Default => !into.contains(&d),
                EdgeKind::Relay if !(g.edge_site_through(t, pi) || g.edge_pip(t, pi).tile.starts_with("CLK_")) => false,
                EdgeKind::Feature | EdgeKind::Relay => {
                    let ls = g.edge_pip(t, pi).fasm_lines();
                    !ls.is_empty() && ls.iter().all(|l| v.lines.contains(l))
                }
            };
            if take {
                seen.insert(d);
                prev.insert(d, (n, t, pi));
                queue.push_back(d);
            }
        }
    }
    let mut hops = Vec::new();
    let mut n = target;
    while let Some(&(p, t, pi)) = prev.get(&n) {
        hops.push(format!("  {:?} {} ({} -> {})", g.edge_kind(t, pi), g.edge_pip(t, pi).name(), g.node_name(p), g.node_name(n)));
        n = p;
    }
    hops.reverse();
    eprintln!("why {tag}: {} from {}", g.node_name(target), g.node_name(root));
    for h in hops {
        eprintln!("{h}");
    }
}

fn view(g: &Graph, fasm: &str, roots: &[u32]) -> R<View> {
    let lines = feature_lines(fasm);
    let inits = lutinit::fasm_inits(fasm)?;
    let set_into: HashSet<u32> = lines
        .iter()
        .filter_map(|l| match g.lookup_pip(l) {
            PipLookup::Edge { dst, tile, pi, .. } if g.edge_kind(tile, pi) == EdgeKind::Feature => Some(dst),
            _ => None,
        })
        .collect();
    let mut reach_of: HashMap<u32, Vec<u32>> = HashMap::new();
    for &r in roots {
        for n in reach(g, r, &lines, &set_into) {
            if n != r {
                reach_of.entry(n).or_default().push(r);
            }
        }
    }
    // A root another root reaches is on that root's net (a clock buffer's
    // input and output pins, in a chain from the MMCM): the node's source is
    // the upstream root. Two roots neither of which reaches the other is a
    // short.
    let mut source = HashMap::new();
    let mut shorts = Vec::new();
    for (n, mut rs) in reach_of.clone() {
        rs.sort_unstable();
        rs.dedup();
        let upstream: Vec<u32> = rs
            .iter()
            .copied()
            .filter(|r| !reach_of.get(r).is_some_and(|by| by.iter().any(|o| o != r && rs.contains(o))))
            .collect();
        let tops = if upstream.is_empty() { rs.clone() } else { upstream };
        if tops.len() > 1 {
            shorts.push((n, tops.clone()));
        }
        source.insert(n, tops[0]);
    }
    shorts.sort();
    Ok(View { lines, inits, source, shorts })
}

/// The input a routing bel takes in `v`, from the FASM's lines (as
/// nextpnr-xilinx writes them, `placed::apply_site_muxes`).
fn selected(v: &View, tile: &str, site: &str, bel: &str, pins: &[String]) -> Option<String> {
    let has = |f: &str| v.lines.contains(&format!("{tile}.{site}.{f}"));
    let letter = bel.len() > 1 && "ABCD".contains(&bel[..1]);
    let rest = if letter { &bel[1..] } else { bel };
    let fasm_name = |p: &str| -> String {
        match (rest, p) {
            (_, "CARRY4_MUX") => "CY".into(),
            (_, "CARRY4_XOR") => "XOR".into(),
            ("PRECYINIT", "0") => "C0".into(),
            ("PRECYINIT", "1") => "C1".into(),
            (_, p) => p.into(),
        }
    };
    match rest {
        "CY0" if letter => Some(if has(&format!("CARRY4.{bel}")) { "O5".into() } else { format!("{}X", &bel[..1]) }),
        "WEMUX" => Some(if has("WEMUX.CE") { "CE".into() } else { "WE".into() }),
        _ => {
            // A pin whose own name is the FASM's first, then a renamed one.
            let mut order: Vec<&String> = pins.iter().filter(|p| fasm_name(p) == **p).collect();
            order.extend(pins.iter().filter(|p| fasm_name(p) != **p));
            if let Some(p) = order.iter().find(|p| has(&format!("{bel}.{}", fasm_name(p)))) {
                return Some((*p).clone());
            }
            if pins.len() == 1 {
                return Some(pins[0].clone());
            }
            // A one-bit select (CEUSEDMUX, SRUSEDMUX): set takes IN.
            if pins.iter().any(|p| p == "IN") {
                return Some(if has(bel) { "IN".into() } else { pins.iter().find(|p| *p != "IN").cloned().unwrap_or_default() });
            }
            None
        }
    }
}

struct Ctx<'a> {
    g: &'a Graph,
    models: &'a HashMap<String, SiteModel>,
    /// Pin node -> (tile, site, pin).
    pin_at: &'a HashMap<u32, (String, String, String)>,
    pin_node: &'a HashMap<(String, String, String), u32>,
    tie: &'a HashMap<u32, bool>,
}

impl Ctx<'_> {
    fn model(&self, site: &str) -> Option<&SiteModel> {
        self.models.get(site.split("_X").next().unwrap_or(""))
    }

    /// The source of a slice wire in `v`.
    fn wire_src(&self, v: &View, tile: &str, site: &str, wire: &str, depth: u32) -> Src {
        let here = || format!("{tile}/{site}.{wire}");
        let Some(m) = self.model(site) else { return Src::Undriven(here()) };
        if depth > 16 {
            return Src::Undriven(here());
        }
        if let Some(pin) = m.wire_pin.get(wire) {
            if m.pin_wire.get(pin).is_some_and(|p| p.1) {
                let Some(&n) = self.pin_node.get(&(tile.to_string(), site.to_string(), pin.clone())) else { return Src::Undriven(here()) };
                return match v.source.get(&n) {
                    Some(&s) => self.node_src(v, s, depth + 1),
                    None => Src::Undriven(here()),
                };
            }
        }
        if let Some(bel) = m.rbel_out.get(wire) {
            let pins = m.rbel_pins.get(bel).cloned().unwrap_or_default();
            let Some(sel) = selected(v, tile, site, bel, &pins) else { return Src::Undriven(format!("{tile}/{site}.{bel}")) };
            return match m.rbel_in.get(&(bel.clone(), sel)) {
                Some(w) => self.wire_src(v, tile, site, w, depth + 1),
                None => Src::Undriven(format!("{tile}/{site}.{bel}")),
            };
        }
        if let Some((bel, pin, ty)) = m.bel_out.get(wire) {
            if ty.contains("VCC") || bel.ends_with("VCC") {
                return Src::Const(true);
            }
            if ty.contains("GND") || bel.ends_with("GND") {
                return Src::Const(false);
            }
            // A routing LUT: O6 passing one input.
            if pin == "O6" && bel.len() == 5 && bel.ends_with("6LUT") {
                let letter = bel.chars().next().unwrap_or('?');
                if let Some(k) = v.inits.get(&(tile.to_string(), site.to_string(), letter)).and_then(|&i| passes(i)) {
                    return self.wire_src(v, tile, site, &format!("{letter}{k}"), depth + 1);
                }
            }
            return Src::Bel(format!("{tile}/{site}.{bel}.{pin}"));
        }
        Src::Undriven(here())
    }

    /// The source a routed node stands for: a slice output pin is followed
    /// into its slice; a tie is a constant.
    fn node_src(&self, v: &View, n: u32, depth: u32) -> Src {
        if let Some(&b) = self.tie.get(&n) {
            return Src::Const(b);
        }
        if let Some((tile, site, pin)) = self.pin_at.get(&n) {
            if let Some(m) = self.model(site) {
                if let Some((wire, false)) = m.pin_wire.get(pin) {
                    return self.wire_src(v, tile, site, wire, depth + 1);
                }
            }
        }
        Src::Node(n)
    }

    fn name(&self, s: &Src) -> String {
        match s {
            Src::Const(b) => format!("{}", *b as u8),
            Src::Node(n) => self.g.node_name(*n),
            Src::Bel(b) => b.clone(),
            Src::Undriven(w) => format!("undriven {w}"),
        }
    }
}

/// Truth tables of O6 and O5 over `vars` (bit i of the row index is
/// vars[i]), from the LUT's INIT and its pins' sources.
fn tables(init: u64, pins: &[Src; 6], vars: &[Src]) -> (u64, u64) {
    let (mut t6, mut t5) = (0u64, 0u64);
    for row in 0..(1u64 << vars.len()) {
        let value = |s: &Src| match s {
            Src::Const(b) => *b,
            other => row >> vars.iter().position(|x| x == other).unwrap_or(0) & 1 == 1,
        };
        let mut a = 0u64;
        for (k, p) in pins.iter().enumerate() {
            if value(p) {
                a |= 1 << k;
            }
        }
        t6 |= (init >> a & 1) << row;
        t5 |= (init >> (a & 0x1F) & 1) << row;
    }
    (t6, t5)
}

/// The LUT as a function: the sources it depends on, and its O6 and O5
/// tables over them.
fn lut_function(init: u64, pins: &[Src; 6]) -> (Vec<Src>, u64, u64) {
    let mut pins = pins.clone();
    let mut vars: Vec<Src> = pins.iter().filter(|s| !matches!(s, Src::Const(_))).cloned().collect::<BTreeSet<_>>().into_iter().collect();
    loop {
        let (t6, t5) = tables(init, &pins, &vars);
        let rows = 1u64 << vars.len();
        // A source neither table depends on is dropped; its pins then read
        // a constant, which changes nothing.
        let idle = (0..vars.len()).find(|&v| {
            (0..rows).filter(|r| r >> v & 1 == 0).all(|r| {
                let s = r | 1 << v;
                (t6 >> r & 1) == (t6 >> s & 1) && (t5 >> r & 1) == (t5 >> s & 1)
            })
        });
        let Some(v) = idle else { return (vars, t6, t5) };
        let gone = vars.remove(v);
        for p in pins.iter_mut() {
            if *p == gone {
                *p = Src::Const(false);
            }
        }
    }
}

/// The cell input pins in slices the placed design connects: (tile, FASM
/// site, bel, pin).
fn connected_pins(design: &Value, loc: &Locator) -> R<HashSet<(String, String, String, String)>> {
    let mods = design["modules"].as_object().ok_or("no modules")?;
    let top = mods
        .values()
        .find(|m| m["attributes"]["top"].is_string() || m["attributes"]["top"].is_number())
        .or_else(|| mods.values().next())
        .ok_or("no top module")?;
    let mut out = HashSet::new();
    for c in top["cells"].as_object().ok_or("no cells")?.values() {
        let Some(bel) = c["attributes"]["NEXTPNR_BEL"].as_str() else { continue };
        let Some((tile, tile_type, np_site, bel)) = loc.locate(bel) else { continue };
        let Some(site) = lutinit::slice_name(&tile_type, &np_site) else { continue };
        for (port, dir) in c["port_directions"].as_object().into_iter().flatten() {
            let wired = c["connections"][port].as_array().is_some_and(|b| !b.is_empty());
            if dir.as_str() == Some("input") && wired {
                out.insert((tile.clone(), site.clone(), bel.clone(), port.clone()));
            }
        }
    }
    Ok(out)
}

fn main() -> R<()> {
    let args: Vec<String> = std::env::args().collect();
    let [_, db_root, part, meta_dir, design, fa, fb] = &args[..] else {
        return Err("usage: fasm_equiv DB_ROOT PART META_DIR DESIGN.JSON A.FASM B.FASM".into());
    };
    let db = Database::open(Path::new(db_root), part)?;
    let connected = {
        let text = std::fs::read_to_string(design).map_err(|e| format!("{design}: {e}"))?;
        let v: Value = serde_json::from_str(&text).map_err(|e| format!("{design}: {e}"))?;
        connected_pins(&v, &Locator::new(&db)?)?
    };
    let g = Graph::build_with(&db, None, GraphOpts::default())?;
    let ta = std::fs::read_to_string(fa).map_err(|e| format!("{fa}: {e}"))?;
    let tb = std::fs::read_to_string(fb).map_err(|e| format!("{fb}: {e}"))?;
    let mut models = HashMap::new();
    for st in ["SLICEL", "SLICEM"] {
        models.insert(st.to_string(), load_model(Path::new(meta_dir), st)?);
    }

    // Every site pin of every tile either FASM names, and the ties.
    let mut tiles: BTreeSet<String> = BTreeSet::new();
    for l in ta.lines().chain(tb.lines()) {
        if let Some(t) = l.split('.').next().filter(|t| !t.is_empty() && !t.starts_with('#')) {
            tiles.insert(t.trim().to_string());
        }
    }
    let mut pin_at: HashMap<u32, (String, String, String)> = HashMap::new();
    let mut pin_node: HashMap<(String, String, String), u32> = HashMap::new();
    let mut slices: BTreeSet<(String, String)> = BTreeSet::new();
    for t in &tiles {
        for (site, pin, n) in g.tile_site_pins(t) {
            if site.starts_with("SLICE") {
                slices.insert((t.clone(), site.clone()));
            }
            pin_at.insert(n, (t.clone(), site.clone(), pin.clone()));
            pin_node.insert((t.clone(), site, pin), n);
        }
    }
    // The interconnect ties, and a DSP tile's own (its input pins' tie-offs).
    let ties: Vec<u32> = g.nodes_with_wire(&["VCC_WIRE", "GND_WIRE", "DSP_VCC_L", "DSP_VCC_R", "DSP_GND_L", "DSP_GND_R"].map(String::from));
    let tie: HashMap<u32, bool> = ties.iter().map(|&n| (n, g.node_name(n).contains("VCC"))).collect();
    let mut roots: Vec<u32> = pin_at.keys().copied().chain(ties.iter().copied()).collect();
    roots.sort_unstable();
    roots.dedup();

    let va = view(&g, &ta, &roots)?;
    let vb = view(&g, &tb, &roots)?;
    if let Ok(w) = std::env::var("FASM_EQUIV_WHY") {
        for one in w.split(',') {
            let t = g.node(one)?;
            why(&g, "a", &va, t);
            why(&g, "b", &vb, t);
        }
    }
    let cx = Ctx { g: &g, models: &models, pin_at: &pin_at, pin_node: &pin_node, tie: &tie };

    let max_examples: usize = std::env::var("FASM_EQUIV_EXAMPLES").ok().and_then(|v| v.parse().ok()).unwrap_or(6);
    let mut counts: BTreeMap<&str, usize> = BTreeMap::new();
    let mut examples: BTreeMap<&str, Vec<Value>> = BTreeMap::new();
    let mut note = |kind: &'static str, ex: Option<Value>| {
        *counts.entry(kind).or_default() += 1;
        if let Some(ex) = ex {
            let e = examples.entry(kind).or_default();
            if e.len() < max_examples {
                e.push(ex);
            }
        }
    };

    // Cell inputs in the slices.
    let is_lut_in = |bel: &str, pin: &str| bel.ends_with("LUT") && pin.len() == 2 && pin.starts_with('A');
    for (tile, site) in &slices {
        let Some(m) = cx.model(site) else { continue };
        for (bel, pin, wire) in &m.bel_in {
            if is_lut_in(bel, pin) || !connected.contains(&(tile.clone(), site.clone(), bel.clone(), pin.clone())) {
                continue;
            }
            let (a, b) = (cx.wire_src(&va, tile, site, wire, 0), cx.wire_src(&vb, tile, site, wire, 0));
            if a == b {
                if !matches!(a, Src::Undriven(_)) {
                    note("cell_pins_same", None);
                }
                continue;
            }
            note("cell_pins_differ", Some(json!({"pin": format!("{tile}/{site}.{bel}.{pin}"), "a": cx.name(&a), "b": cx.name(&b)})));
        }
        // LUTs as functions of their sources.
        for letter in ['A', 'B', 'C', 'D'] {
            let key = (tile.clone(), site.clone(), letter);
            let (ia, ib) = (va.inits.get(&key).copied(), vb.inits.get(&key).copied());
            if ia.unwrap_or(0) == 0 && ib.unwrap_or(0) == 0 {
                continue;
            }
            // A routing LUT one side has and the other has not: followed
            // through where its output is used.
            let routing_only = |i: Option<u64>, other: Option<u64>| other.unwrap_or(0) == 0 && i.and_then(passes).is_some();
            if routing_only(ia, ib) || routing_only(ib, ia) {
                note("routing_luts", None);
                continue;
            }
            let inputs = |v: &View| -> [Src; 6] { std::array::from_fn(|j| cx.wire_src(v, tile, site, &format!("{letter}{}", j + 1), 0)) };
            let fa = lut_function(ia.unwrap_or(0), &inputs(&va));
            let fb = lut_function(ib.unwrap_or(0), &inputs(&vb));
            if fa == fb {
                note("luts_same", None);
            } else {
                let show = |f: &(Vec<Src>, u64, u64)| json!({"inputs": f.0.iter().map(|s| cx.name(s)).collect::<Vec<_>>(), "o6": format!("{:X}", f.1), "o5": format!("{:X}", f.2)});
                note("luts_differ", Some(json!({"lut": format!("{tile}.{site}.{letter}LUT"), "a": show(&fa), "b": show(&fb)})));
            }
        }
    }

    // Input site pins outside the slices. A BUFHCE is clock routing, not a
    // cell: each router picks its own in the HROW, and the clock it carries
    // is compared at the pins it reaches.
    let mut others: Vec<(&u32, &(String, String, String))> =
        pin_at.iter().filter(|(_, (_, s, _))| !s.starts_with("SLICE") && !s.starts_with("BUFHCE")).collect();
    others.sort_by(|a, b| a.1.cmp(b.1));
    for (&n, (tile, site, pin)) in others {
        let (a, b) = (va.source.get(&n).map(|&s| cx.node_src(&va, s, 0)), vb.source.get(&n).map(|&s| cx.node_src(&vb, s, 0)));
        match (a, b) {
            (None, None) => {}
            (Some(a), Some(b)) if a == b => note("site_pins_same", None),
            (a, b) => note("site_pins_differ", Some(json!({"pin": format!("{tile}/{site}.{pin}"), "a": a.map(|s| cx.name(&s)), "b": b.map(|s| cx.name(&s))}))),
        }
    }
    for (kind, v) in [("shorts_a", &va), ("shorts_b", &vb)] {
        for (n, rs) in &v.shorts {
            note(kind, Some(json!({"node": g.node_name(*n), "sources": rs.iter().map(|&r| g.node_name(r)).collect::<Vec<_>>()})));
        }
    }
    let same = ["cell_pins_differ", "luts_differ", "site_pins_differ", "shorts_a", "shorts_b"].iter().all(|k| !counts.contains_key(k));
    println!("{}", json!({"a": fa, "b": fb, "equivalent": same, "tiles": tiles.len(), "slices": slices.len(), "connected_cell_pins": connected.len(), "counts": counts, "examples": examples}));
    if same { Ok(()) } else { std::process::exit(1) }
}
