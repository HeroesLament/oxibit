//! `oxibit route`: route test nets through chosen pips, as FASM.

use std::collections::{HashMap, HashSet};
use std::path::Path;
use std::process::ExitCode;
use std::time::Instant;

use oxibit_fasm::db::Database;
use oxibit_fasm::route::{Enable, Graph, Window};

use crate::input::Result;

pub struct Args<'a> {
    pub db_root: &'a Path,
    pub part: &'a str,
    pub window: Option<&'a str>,
    pub from: Option<&'a str>,
    pub to: Option<&'a str>,
    pub via: &'a [String],
    pub avoid: &'a [std::path::PathBuf],
    pub check: &'a [std::path::PathBuf],
}

fn parse_window(s: &str) -> Result<Window> {
    let v: Vec<i32> = s.split(',').map(|x| x.trim().parse::<i32>().map_err(|e| format!("window: {e}"))).collect::<Result<_>>()?;
    match v[..] {
        [x0, y0, x1, y1] => Ok(Window { x0, y0, x1, y1 }),
        _ => Err("window is x0,y0,x1,y1 (grid coordinates)".into()),
    }
}

pub fn run(a: Args) -> Result<ExitCode> {
    let t0 = Instant::now();
    let db = Database::open(a.db_root, a.part).map_err(|e| format!("database: {e}"))?;
    let window = a.window.map(parse_window).transpose()?;
    let g = Graph::build(&db, window)?;
    eprintln!("oxibit route: graph {} nodes, {} usable pip edges ({} ms)", g.nodes(), g.edges(), t0.elapsed().as_millis());

    // Consistency: every pip a working design emitted must be a usable edge.
    for path in a.check {
        let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
        let (mut found, mut missing, mut outside, mut pseudo) = (0, Vec::new(), 0, 0);
        for line in text.lines() {
            let f = line.split('#').next().unwrap_or("").trim();
            let parts: Vec<&str> = f.split('.').collect();
            if parts.len() != 3 || f.contains('=') || f.contains('[') {
                continue;
            }
            let (tile, dst, src) = (parts[0], parts[1], parts[2]);
            let (Ok(s), Ok(d)) = (g.node(&format!("{tile}/{src}")), g.node(&format!("{tile}/{dst}"))) else {
                outside += 1;
                continue;
            };
            let pip = format!("{tile}.{dst}.{src}");
            // Pseudo-pips (default/hint) carry no bits: nextpnr writes the
            // line, the assembler sets nothing, and they are not routes.
            let ty = g.tile_type(tile).unwrap_or("");
            let key = format!("{ty}.{dst}.{src}");
            if db.segbits.get(ty).is_some_and(|t| t.ppips.contains(&key) && !t.ppips_always.contains(&key)) {
                pseudo += 1;
                continue;
            }
            match g.route_via(&format!("{tile}/{src}"), std::slice::from_ref(&pip), &format!("{tile}/{dst}"), &HashSet::new()) {
                Ok(_) if s != d => found += 1,
                _ => missing.push(pip),
            }
        }
        eprintln!(
            "check {}: {found} emitted pips are usable edges, {} are not, {pseudo} pseudo-pip lines (no bits), {outside} not resolvable (site features or outside window)",
            path.display(),
            missing.len()
        );
        for m in missing.iter().take(20) {
            eprintln!("  not usable: {m}");
        }
    }

    if let (Some(from), Some(to)) = (a.from, a.to) {
        let mut blocked = HashSet::new();
        for path in a.avoid {
            let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
            blocked.extend(g.occupied(&text));
        }
        let t1 = Instant::now();
        let pips = g.route_via(from, a.via, to, &blocked)?;
        eprintln!("oxibit route: {} pips ({} with bits) in {} ms", pips.len(), pips.iter().filter(|p| p.enable != Enable::Fixed).count(), t1.elapsed().as_millis());
        for p in &pips {
            match p.fasm() {
                Some(f) => println!("{f}"),
                None => println!("# fixed: {}/{} -> {}", p.tile, p.src, p.dst),
            }
        }
    }
    Ok(ExitCode::SUCCESS)
}

// ---------------------------------------------------------------------------
// route-port
// ---------------------------------------------------------------------------

/// `oxibit route-port`: a long-lived router for test campaigns.
///
/// `{:packet, 4}` framing. At start it builds the graph, reads the base
/// design, and strips the nets driven from each `--strip` source (every pip
/// reachable from it through the design's pips and fixed edges). Requests
/// are JSON:
///
/// * `{"op":"info"}` -> the stripped nets.
/// * `{"op":"pips","types":[..]}` -> every usable pip with bits in tiles of
///   those types inside the window.
/// * `{"op":"build","ends":[[from,to],..],"targets":[..],"max_via":N,"avoid":[wire substrings]}` ->
///   one chain per end pair, threading targets nearest-first, routed clear
///   of the stripped base design and of each other.
///
/// Reply: `0x00` + u32 JSON length + JSON (+ `.bit` bytes for `build`), or
/// `0x01` + error text.
pub struct PortArgs<'a> {
    pub db_root: &'a Path,
    pub part: &'a str,
    pub window: Option<&'a str>,
    pub base: &'a Path,
    pub strip: &'a [String],
    /// TILE/WIRE root of the base design's clock net (a BUFG output), for FF relays.
    pub clock: Option<&'a str>,
}

struct Port {
    db: Database,
    g: Graph,
    base: String,
    blocked: HashSet<u32>,
    nets: Vec<serde_json::Value>,
    /// (CLB tile, site) -> the INT pip that clocks it from the base clock net.
    clock_line: HashMap<(String, String), String>,
    /// CLB sites the base design uses.
    used_sites: HashSet<(String, String)>,
    /// Output pins of every CLB site (relay sinks): `connect` routes never
    /// pass through a site.
    site_outputs: HashSet<u32>,
}

pub fn serve(a: PortArgs) -> ExitCode {
    let port = match open_port(&a) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("oxibit route-port: {e}");
            return ExitCode::from(69);
        }
    };
    let mut stdin = std::io::stdin().lock();
    let mut stdout = std::io::stdout().lock();
    loop {
        use std::io::{Read, Write};
        let mut len = [0u8; 4];
        if stdin.read_exact(&mut len).is_err() {
            return ExitCode::SUCCESS;
        }
        let mut req = vec![0u8; u32::from_be_bytes(len) as usize];
        if stdin.read_exact(&mut req).is_err() {
            return ExitCode::from(70);
        }
        let reply = match port.handle(&req) {
            Ok((json, bit)) => {
                let j = json.to_string().into_bytes();
                [vec![0x00], (j.len() as u32).to_be_bytes().to_vec(), j, bit].concat()
            }
            Err(e) => [vec![0x01], e.into_bytes()].concat(),
        };
        let ok = stdout.write_all(&(reply.len() as u32).to_be_bytes()).and_then(|_| stdout.write_all(&reply)).and_then(|_| stdout.flush());
        if ok.is_err() {
            return ExitCode::from(70);
        }
    }
}

fn open_port(a: &PortArgs) -> Result<Port> {
    let t0 = Instant::now();
    let db = Database::open(a.db_root, a.part).map_err(|e| format!("database: {e}"))?;
    let window = a.window.map(parse_window).transpose()?;
    let g = Graph::build(&db, window)?;
    let text = std::fs::read_to_string(a.base).map_err(|e| format!("{}: {e}", a.base.display()))?;
    // Pip lines, plus every other valueless line (site configuration such as
    // `BUFHCE.BUFHCE_X0Y0.IN_USE`), so a trace crosses configured relays.
    let mut on = oxibit_fasm::route::pip_lines(&text);
    on.extend(text.lines().map(|l| l.split('#').next().unwrap_or("").trim()).filter(|f| !f.is_empty() && !f.contains('=')).map(String::from));

    let mut strip: HashSet<String> = HashSet::new();
    let mut nets = Vec::new();
    for src in a.strip {
        let n = g.node(src)?;
        let (nodes, pips) = g.trace(n, &on, false);
        if pips.is_empty() {
            return Err(format!("--strip {src}: the base design drives no pips from it"));
        }
        nets.push(serde_json::json!({"src": src, "pips": pips.len(), "nodes": nodes.len()}));
        strip.extend(pips);
    }
    let base: String = text
        .lines()
        .filter(|l| !strip.contains(l.split('#').next().unwrap_or("").trim()))
        .map(|l| format!("{l}\n"))
        .collect();
    // Sites the base design uses: their outputs are not relays.
    let mut used_sites: HashSet<(String, String)> = HashSet::new();
    for l in base.lines() {
        let f = l.split(['#', '=']).next().unwrap_or("").trim();
        let parts: Vec<&str> = f.splitn(3, '.').collect();
        if let [tile, site, rest] = parts[..] {
            if site.starts_with("SLICE") && rest != "NOCLKINV" {
                used_sites.insert((tile.to_string(), site.to_string()));
            }
        }
    }
    let mut site_nodes = Vec::new();
    for (tile, site) in &used_sites {
        site_nodes.extend(g.relay_outputs(tile, site));
    }
    // FF relays: a site can host one only if the base clock net reaches a
    // pip into its CLK pin.
    let mut clock_line = HashMap::new();
    if let Some(root) = a.clock {
        let (clk_nodes, clk_pips) = g.trace(g.node(root)?, &on, true);
        let mut reach = 0;
        let tiles: Vec<String> = g.tile_names().filter(|t| t.starts_with("CLB")).map(String::from).collect();
        for tile in &tiles {
            for site in g.sites(tile) {
                let Some(clk) = g.site_pin(tile, &site, "CLK") else { continue };
                let mut drivers: Vec<String> = g.feature_drivers(clk, &clk_nodes).iter().filter_map(|p| p.fasm()).collect();
                drivers.sort();
                match drivers.first() {
                    Some(l) => {
                        reach += 1;
                        clock_line.insert((tile.clone(), site.clone()), l.clone());
                    }
                    None => {
                        for (n, lines) in g.site_relays(tile, &site) {
                            if lines.iter().any(|l| l.contains("FFMUX")) {
                                site_nodes.push(n);
                            }
                        }
                    }
                }
            }
        }
        eprintln!("oxibit route-port: clock net {} nodes, {} pips; {reach} CLB sites can be clocked", clk_nodes.len(), clk_pips.len());
    } else {
        // No clock to give them: no FF relays anywhere.
        let tiles: Vec<String> = g.tile_names().filter(|t| t.starts_with("CLB")).map(String::from).collect();
        for tile in &tiles {
            for site in g.sites(tile) {
                for (n, lines) in g.site_relays(tile, &site) {
                    if lines.iter().any(|l| l.contains("FFMUX")) {
                        site_nodes.push(n);
                    }
                }
            }
        }
    }
    let mut blocked = g.occupied(&base);
    blocked.extend(g.exclusion(site_nodes));
    let mut site_outputs = HashSet::new();
    for tile in g.tile_names().filter(|t| t.starts_with("CLB")).map(String::from).collect::<Vec<_>>() {
        for site in g.sites(&tile) {
            site_outputs.extend(g.site_relays(&tile, &site).into_iter().map(|(n, _)| n));
        }
    }
    eprintln!("oxibit route-port: {} slices in use by the base design", used_sites.len());
    eprintln!(
        "oxibit route-port: graph {} nodes, {} edges; stripped {} pips from {} nets; {} nodes blocked ({} ms)",
        g.nodes(),
        g.edges(),
        strip.len(),
        nets.len(),
        blocked.len(),
        t0.elapsed().as_millis()
    );
    Ok(Port { db, g, base, blocked, nets, clock_line, used_sites, site_outputs })
}

impl Port {
    fn handle(&self, req: &[u8]) -> Result<(serde_json::Value, Vec<u8>)> {
        let v: serde_json::Value = serde_json::from_slice(req).map_err(|e| format!("request: {e}"))?;
        let strs = |k: &str| -> Vec<String> { v[k].as_array().into_iter().flatten().filter_map(|x| x.as_str().map(String::from)).collect() };
        match v["op"].as_str() {
            Some("info") => Ok((serde_json::json!({"nets": self.nets, "blocked": self.blocked.len()}), vec![])),
            Some("pips") => {
                let w: Vec<i32> = v["window"].as_array().into_iter().flatten().filter_map(|x| x.as_i64().map(|x| x as i32)).collect();
                let window = match w[..] {
                    [x0, y0, x1, y1] => Some(Window { x0, y0, x1, y1 }),
                    _ => None,
                };
                Ok((serde_json::json!({"pips": self.g.feature_pips(&strs("types"), window)}), vec![]))
            }
            Some("feasible") => {
                let types = strs("tile_types");
                let f = self.g.feasibility(
                    v["from"].as_str().unwrap_or(""),
                    v["to"].as_str().unwrap_or(""),
                    &strs("targets"),
                    (!types.is_empty()).then_some(&types[..]),
                    &self.blocked,
                )?;
                let rows: Vec<serde_json::Value> = f.into_iter().map(|(t, a, b)| serde_json::json!([t, a, b])).collect();
                Ok((serde_json::json!({"targets": rows}), vec![]))
            }
            Some("tiles") => {
                // Tile instances of the given types in a window, with sites.
                let types = strs("types");
                let w: Vec<i32> = v["window"].as_array().into_iter().flatten().filter_map(|x| x.as_i64().map(|x| x as i32)).collect();
                let mut rows = Vec::new();
                for tile in self.g.tile_names() {
                    let Some(ty) = self.g.tile_type(tile) else { continue };
                    if !types.iter().any(|t| t == ty) {
                        continue;
                    }
                    let (x, y) = self.g.tile_xy(tile).unwrap_or((-1, -1));
                    if let [x0, y0, x1, y1] = w[..] {
                        if !(x0..=x1).contains(&x) || !(y0..=y1).contains(&y) {
                            continue;
                        }
                    }
                    rows.push(serde_json::json!([tile, ty, x, y, self.g.sites(tile)]));
                }
                Ok((serde_json::json!({"tiles": rows}), vec![]))
            }
            Some("free_sites") => {
                // CLB sites in the window the base design leaves alone.
                let w: Vec<i32> = v["window"].as_array().into_iter().flatten().filter_map(|x| x.as_i64().map(|x| x as i32)).collect();
                let mut rows = Vec::new();
                for tile in self.g.tile_names().filter(|t| t.starts_with("CLB")) {
                    if let [x0, y0, x1, y1] = w[..] {
                        let Some((x, y)) = self.g.tile_xy(tile) else { continue };
                        if !(x0..=x1).contains(&x) || !(y0..=y1).contains(&y) {
                            continue;
                        }
                    }
                    for site in self.g.sites(tile) {
                        if !self.used_sites.contains(&(tile.to_string(), site.clone())) {
                            let (x, y) = self.g.tile_xy(tile).unwrap_or((-1, -1));
                            rows.push(serde_json::json!([tile, self.g.tile_type(tile), site, self.clock_line.contains_key(&(tile.to_string(), site.clone())), x, y]));
                        }
                    }
                }
                Ok((serde_json::json!({"sites": rows}), vec![]))
            }
            Some("connect") => {
                // Nets as trees from launch pins to DUT pins and DUT pins to
                // capture pins; DUT sites reserved; DUT lines added as given.
                let nets: Vec<(String, Vec<String>)> = v["nets"]
                    .as_array()
                    .into_iter()
                    .flatten()
                    .filter_map(|n| {
                        let to = n["to"].as_array()?.iter().filter_map(|t| t.as_str().map(String::from)).collect();
                        Some((n["from"].as_str()?.to_string(), to))
                    })
                    .collect();
                let mut reserve = Vec::new();
                for r in v["reserve_sites"].as_array().into_iter().flatten() {
                    if let (Some(t), Some(st)) = (r[0].as_str(), r[1].as_str()) {
                        if self.used_sites.contains(&(t.to_string(), st.to_string())) {
                            return Err(format!("{t}.{st} is used by the base design"));
                        }
                        reserve.extend(self.g.site_pin_nodes(t, st));
                    }
                }
                // DUT pins that are net ends must not be reserved against their own net.
                let ends: HashSet<u32> = nets.iter().flat_map(|(f, t)| std::iter::once(f).chain(t.iter())).filter_map(|w| self.g.node(w).ok()).collect();
                reserve.retain(|n| !ends.contains(n));
                let mut blocked = self.blocked.clone();
                blocked.extend(self.site_outputs.iter().copied().filter(|n| !ends.contains(n)));
                blocked.extend(self.g.nodes_with_wire(&strs("avoid")));
                let types = strs("tile_types");
                let routed = self.g.connect(&nets, &reserve, (!types.is_empty()).then_some(&types[..]), &blocked);

                let mut fasm = self.base.clone();
                let mut out = Vec::new();
                for r in &routed {
                    match r {
                        Ok(pips) => {
                            let mut lines: Vec<String> = pips.iter().flat_map(|p| p.fasm_lines()).collect();
                            lines.extend(self.g.wire_extras(pips));
                            let proven: Vec<String> = pips.iter().filter(|p| p.fasm().is_none()).map(|p| p.name()).collect();
                            for l in &lines {
                                fasm.push_str(l);
                                fasm.push('\n');
                            }
                            out.push(serde_json::json!({"pips": lines, "proven": proven}));
                        }
                        Err(e) => out.push(serde_json::json!({"error": e})),
                    }
                }
                for l in strs("lines") {
                    fasm.push_str(&l);
                    fasm.push('\n');
                }
                let bit = if v["bit"].as_bool().unwrap_or(true) { crate::build::to_bit(&self.db, &fasm, "fabfarm-dut")? } else { vec![] };
                Ok((serde_json::json!({"nets": out}), bit))
            }
            Some("explain") => {
                // For chains planned before `proven` existed: re-walk each
                // chain's configured path.
                let mut out = Vec::new();
                for c in v["chains"].as_array().into_iter().flatten() {
                    let (from, to) = (c["from"].as_str().unwrap_or(""), c["to"].as_str().unwrap_or(""));
                    let lines: HashSet<String> = c["lines"].as_array().into_iter().flatten().filter_map(|x| x.as_str().map(String::from)).collect();
                    out.push(match self.g.configured_path(from, to, &lines) {
                        Ok(path) => serde_json::json!({"proven": path.iter().filter(|p| p.fasm().is_none()).map(|p| p.name()).collect::<Vec<_>>()}),
                        Err(e) => serde_json::json!({"error": e}),
                    });
                }
                Ok((serde_json::json!({"chains": out}), vec![]))
            }
            Some("fixed_pips") => {
                // {wires: [...], backward: bool} -> {pips: {wire: [...]}}
                let backward = v["backward"].as_bool().unwrap_or(false);
                let mut m = serde_json::Map::new();
                for w in strs("wires") {
                    let ps = self.g.fixed_pips(&w, backward)?;
                    m.insert(w, serde_json::json!(ps));
                }
                Ok((serde_json::json!({"pips": m}), vec![]))
            }
            Some("assemble") => {
                // `strip`: base lines starting with any of these prefixes are
                // dropped first (a DUT net taking over a mux input the base
                // design ties off, e.g. an unused PS7 input on GND).
                let strip = strs("strip");
                let mut fasm: String = if strip.is_empty() {
                    self.base.clone()
                } else {
                    self.base.lines().filter(|l| !strip.iter().any(|p| l.starts_with(p.as_str()))).flat_map(|l| [l, "\n"]).collect()
                };
                for l in strs("lines") {
                    fasm.push_str(&l);
                    fasm.push('\n');
                }
                let bit = crate::build::to_bit(&self.db, &fasm, "fabfarm-route")?;
                Ok((serde_json::json!({"lines": fasm.lines().count()}), bit))
            }
            Some("build") => {
                let ends: Vec<(String, String)> = v["ends"]
                    .as_array()
                    .into_iter()
                    .flatten()
                    .filter_map(|p| Some((p[0].as_str()?.to_string(), p[1].as_str()?.to_string())))
                    .collect();
                let mut targets = strs("targets");
                // How far the packer looks for the next target (40 was the
                // fixed value before it became a parameter), and how.
                let opts = oxibit_fasm::route::PackOpts {
                    max_via: v["max_via"].as_u64().unwrap_or(8) as usize,
                    target_hops: v["target_hops"].as_u64().unwrap_or(40).min(u16::MAX as u64) as u16,
                    dead_regions: v["dead_regions"].as_bool().unwrap_or(false),
                };
                let mut blocked = self.blocked.clone();
                blocked.extend(self.g.nodes_with_wire(&strs("avoid")));
                let types = strs("tile_types");
                let t0 = std::time::Instant::now();
                let mut st = oxibit_fasm::route::PackStats::default();
                let chains = self.g.pack(&ends, &mut targets, &opts, (!types.is_empty()).then_some(&types[..]), &mut blocked, &mut st);
                let stats = serde_json::json!({
                    "ms": t0.elapsed().as_millis() as u64,
                    "target_searches": st.target_searches,
                    "target_misses": st.target_misses,
                    "target_pruned": st.target_pruned,
                    "dead_nodes": st.dead_nodes,
                    "target_visited": st.target_visited,
                    "target_hit_visited": st.target_hit_visited,
                    "target_hit_max": st.target_hit_max,
                    "finish_searches": st.finish_searches,
                    "finish_misses": st.finish_misses,
                    "finish_visited": st.finish_visited,
                    "rollbacks": st.rollbacks,
                    "chain_ms": st.chain_ms,
                });

                let mut fasm = self.base.clone();
                let mut out = Vec::new();
                for c in &chains {
                    match c {
                        Ok(c) => {
                            let mut lines: Vec<String> = c.pips.iter().flat_map(|p| p.fasm_lines()).collect();
                            lines.extend(self.g.wire_extras(&c.pips));
                            // FF relays need their site clocked.
                            let clocks: Vec<String> = lines
                                .iter()
                                .filter(|l| l.contains("FFMUX."))
                                .filter_map(|l| {
                                    let mut it = l.splitn(3, '.');
                                    let (t, st) = (it.next()?, it.next()?);
                                    self.clock_line.get(&(t.to_string(), st.to_string())).cloned()
                                })
                                .collect();
                            lines.extend(clocks);
                            let mut seen = HashSet::new();
                            lines.retain(|l| seen.insert(l.clone()));
                            for l in &lines {
                                fasm.push_str(l);
                                fasm.push('\n');
                            }
                            // Edges on the path that emit no pip line of their own (hard
                            // wires, `always` pseudo-pips, relays): the chain proves them too.
                            let proven: Vec<String> = c.pips.iter().filter(|p| p.fasm().is_none()).map(|p| p.name()).collect();
                            out.push(serde_json::json!({"pips": lines, "proven": proven, "targets": c.targets}));
                        }
                        Err(e) => out.push(serde_json::json!({"error": e})),
                    }
                }
                let bit = if v["bit"].as_bool().unwrap_or(true) { crate::build::to_bit(&self.db, &fasm, "fabfarm-route")? } else { vec![] };
                Ok((serde_json::json!({"chains": out, "leftover": targets, "stats": stats}), bit))
            }
            other => Err(format!("unknown op {other:?}")),
        }
    }
}
