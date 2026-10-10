//! A prjxray database, read the way `prjxray.db.Database` reads it.
//!
//! `root` is the family directory (`.../prjxray-db/zynq7`). Everything the
//! assembler needs is loaded once at open: the tile grid of the part's
//! fabric, every tile type's segbits and pseudo-pips, the part's required
//! features, its I/O banks and its frame layout. Holding it in memory is
//! what makes `oxibit build-port` fast: a FASM file then costs only its own
//! lines.

use std::collections::{BTreeSet, HashMap, HashSet};
use std::fs;
use std::path::{Path, PathBuf};

use oxibit_bitstream::layout::{BLOCK_RAM, CLB_IO_CLK, FrameLayout};
use serde_json::Value;

#[derive(Debug, Clone)]
pub struct Bits {
    pub base: u32,
    pub frames: u32,
    /// Word offset of this tile inside its frames. i64 because an aliased
    /// tile's offset is shifted down by the alias start offset and can go
    /// negative (prjxray then indexes from the end of the frame; so do we).
    pub offset: i64,
}

#[derive(Debug, Clone)]
pub struct Alias {
    pub tile_type: String,
    pub start_offset: i64,
    pub sites: Vec<(String, String)>,
}

#[derive(Debug, Clone)]
pub struct TileInfo {
    pub tile_type: String,
    /// In tilegrid order; block type is CLB_IO_CLK or BLOCK_RAM.
    pub bits: Vec<(u32, Bits, Option<Alias>)>,
    pub sites: Vec<String>,
}

#[derive(Debug, Clone, Copy)]
pub struct SegBit {
    pub column: u32,
    pub bit: u32,
    pub set: bool,
}

/// base feature -> address -> (block type, full feature name)
pub type FeatureAddresses = HashMap<String, HashMap<u32, (u32, String)>>;

#[derive(Debug, Default)]
pub struct TileSegbits {
    /// CLB_IO_CLK first, then BLOCK_RAM, as prjxray inserts them.
    pub by_block: Vec<(u32, HashMap<String, Vec<SegBit>>)>,
    pub addresses: FeatureAddresses,
    pub ppips: HashSet<String>,
    /// The `always` subset of `ppips`: connections that need no bits.
    pub ppips_always: HashSet<String>,
}

pub struct Database {
    pub root: PathBuf,
    pub part: String,
    /// Fabric directory under `root` (e.g. `xc7z020`).
    pub fabric: String,
    pub layout: FrameLayout,
    pub tiles: HashMap<String, TileInfo>,
    pub segbits: HashMap<String, TileSegbits>,
    pub required_features: Vec<String>,
    pub tile_to_bank: HashMap<String, String>,
    pub bank_to_tiles: HashMap<String, BTreeSet<String>>,
}

type R<T> = Result<T, String>;

fn read(p: &Path) -> R<String> {
    fs::read_to_string(p).map_err(|e| format!("{}: {e}", p.display()))
}

impl Database {
    pub fn open(root: &Path, part: &str) -> R<Self> {
        let fabric = fabric_for_part(root, part)?;
        let part_dir = root.join(part);

        let layout = FrameLayout::from_part_yaml(&read(&part_dir.join("part.yaml"))?)?;
        let tiles = read_tilegrid(&root.join(&fabric).join("tilegrid.json"))?;
        let segbits = read_all_segbits(root)?;

        let required_features = match fs::read_to_string(part_dir.join("required_features.fasm")) {
            Ok(t) => t.lines().map(str::trim).filter(|l| !l.is_empty()).map(String::from).collect(),
            Err(_) => vec![],
        };

        let (tile_to_bank, bank_to_tiles) = read_banks(&part_dir)?;

        Ok(Database { root: root.to_path_buf(), part: part.to_string(), fabric, layout, tiles, segbits, required_features, tile_to_bank, bank_to_tiles })
    }
}

/// `prjxray.util.get_fabric_for_part`: part -> device (mapping/parts.yaml),
/// device -> fabric (mapping/devices.yaml).
fn fabric_for_part(root: &Path, part: &str) -> R<String> {
    let device = yaml_child(&read(&root.join("mapping/parts.yaml"))?, part, "device")
        .ok_or_else(|| format!("{part} not in mapping/parts.yaml"))?;
    yaml_child(&read(&root.join("mapping/devices.yaml"))?, &device, "fabric")
        .ok_or_else(|| format!("{device} not in mapping/devices.yaml"))
}

/// Value of `child:` under the top-level key `key:` in a flat two-level YAML map.
fn yaml_child(text: &str, key: &str, child: &str) -> Option<String> {
    let mut inside = false;
    for line in text.lines() {
        if line.trim_start().starts_with('#') || line.trim().is_empty() {
            continue;
        }
        if !line.starts_with(' ') {
            inside = line.trim_end_matches(':').trim_matches('"') == key
                || line.split(':').next().map(|k| k.trim_matches('"')) == Some(key);
            continue;
        }
        if inside {
            if let Some(v) = line.trim().strip_prefix(&format!("{child}:")) {
                return Some(v.trim().trim_matches(|c| c == '"' || c == '\'').to_string());
            }
        }
    }
    None
}

fn read_tilegrid(path: &Path) -> R<HashMap<String, TileInfo>> {
    let v: Value = serde_json::from_str(&read(path)?).map_err(|e| format!("{}: {e}", path.display()))?;
    let obj = v.as_object().ok_or("tilegrid is not an object")?;
    let mut tiles = HashMap::with_capacity(obj.len());

    for (name, t) in obj {
        let tile_type = t["type"].as_str().ok_or_else(|| format!("{name}: no type"))?.to_string();
        let sites = t["sites"].as_object().map(|s| s.keys().cloned().collect()).unwrap_or_default();
        let mut bits = Vec::new();
        if let Some(b) = t.get("bits").and_then(Value::as_object) {
            for (k, bv) in b {
                let block = match k.as_str() {
                    "CLB_IO_CLK" => CLB_IO_CLK,
                    "BLOCK_RAM" => BLOCK_RAM,
                    other => return Err(format!("{name}: block type {other}")),
                };
                let base = bv["baseaddr"].as_str().and_then(|s| u32::from_str_radix(s.trim_start_matches("0x"), 16).ok());
                let base = base.ok_or_else(|| format!("{name}: baseaddr"))?;
                let frames = bv["frames"].as_u64().ok_or_else(|| format!("{name}: frames"))? as u32;
                let offset = bv["offset"].as_i64().ok_or_else(|| format!("{name}: offset"))?;
                let alias = bv.get("alias").map(|a| Alias {
                    tile_type: a["type"].as_str().unwrap_or("").to_string(),
                    start_offset: a["start_offset"].as_i64().unwrap_or(0),
                    sites: a["sites"]
                        .as_object()
                        .map(|m| m.iter().map(|(k, v)| (k.clone(), v.as_str().unwrap_or("").to_string())).collect())
                        .unwrap_or_default(),
                });
                bits.push((block, Bits { base, frames, offset }, alias));
            }
        }
        tiles.insert(name.clone(), TileInfo { tile_type, bits, sites });
    }
    Ok(tiles)
}

/// Every tile type that has a `tile_type_*.json`, with its segbits (if any)
/// and ppips (if any), keyed by upper-case tile type.
fn read_all_segbits(root: &Path) -> R<HashMap<String, TileSegbits>> {
    let mut out = HashMap::new();
    let entries = fs::read_dir(root).map_err(|e| format!("{}: {e}", root.display()))?;
    for e in entries.flatten() {
        let name = e.file_name().to_string_lossy().to_string();
        let Some(tt) = name.strip_prefix("tile_type_").and_then(|n| n.strip_suffix(".json")) else { continue };
        let lower = tt.to_lowercase();
        let mut ts = TileSegbits::default();

        for (block, file) in [(CLB_IO_CLK, format!("segbits_{lower}.db")), (BLOCK_RAM, format!("segbits_{lower}.block_ram.db"))] {
            let p = root.join(&file);
            if p.is_file() {
                ts.by_block.push((block, read_segbits(&p)?));
            }
        }
        let pp = root.join(format!("ppips_{lower}.db"));
        if pp.is_file() {
            for l in read(&pp)?.lines() {
                if let Some((feature, kind)) = l.trim().split_once(' ') {
                    ts.ppips.insert(feature.to_string());
                    if kind == "always" {
                        ts.ppips_always.insert(feature.to_string());
                    }
                }
            }
        }
        for (block, map) in &ts.by_block {
            for feature in map.keys() {
                if let (Some(s), Some(e)) = (feature.rfind('['), feature.rfind(']')) {
                    if let Ok(addr) = feature[s + 1..e].parse::<u32>() {
                        ts.addresses.entry(feature[..s].to_string()).or_default().insert(addr, (*block, feature.clone()));
                    }
                }
            }
        }
        out.insert(lower.to_uppercase(), ts);
    }
    Ok(out)
}

fn read_segbits(p: &Path) -> R<HashMap<String, Vec<SegBit>>> {
    let mut map = HashMap::new();
    for l in read(p)?.lines() {
        let l = l.trim();
        if l.is_empty() {
            continue;
        }
        let mut parts = l.split(' ');
        let feature = parts.next().unwrap().to_string();
        let mut bits = Vec::new();
        for b in parts {
            let (set, b) = match b.strip_prefix('!') {
                Some(rest) => (false, rest),
                None => (true, b),
            };
            let (c, w) = b.split_once('_').ok_or_else(|| format!("{}: bad bit {b}", p.display()))?;
            let column = c.parse().map_err(|_| format!("{}: bad bit {b}", p.display()))?;
            let bit = w.parse().map_err(|_| format!("{}: bad bit {b}", p.display()))?;
            bits.push(SegBit { column, bit, set });
        }
        if bits.is_empty() {
            return Err(format!("{}: feature with no bits: {l}", p.display()));
        }
        map.insert(feature, bits);
    }
    Ok(map)
}

/// I/O bank membership, from package_pins.csv and part.json "iobanks", as
/// fasm2frames builds it for STEPDOWN propagation.
type Banks = (HashMap<String, String>, HashMap<String, BTreeSet<String>>);

fn read_banks(part_dir: &Path) -> R<Banks> {
    let mut t2b = HashMap::new();
    let mut b2t: HashMap<String, BTreeSet<String>> = HashMap::new();

    let pj: Value = serde_json::from_str(&read(&part_dir.join("part.json"))?).map_err(|e| e.to_string())?;
    if let Some(banks) = pj["iobanks"].as_object() {
        for (bank, loc) in banks {
            let tile = format!("HCLK_IOI3_{}", loc.as_str().unwrap_or(""));
            b2t.entry(bank.clone()).or_default().insert(tile.clone());
            t2b.insert(tile, bank.clone());
        }
    }

    let csv = read(&part_dir.join("package_pins.csv"))?;
    let mut lines = csv.lines();
    let header: Vec<&str> = lines.next().unwrap_or("").split(',').collect();
    let col = |n: &str| header.iter().position(|h| h.trim() == n).ok_or_else(|| format!("package_pins.csv: no {n} column"));
    let (bi, ti) = (col("bank")?, col("tile")?);
    for l in lines {
        let f: Vec<&str> = l.split(',').collect();
        if f.len() <= bi.max(ti) {
            continue;
        }
        let (bank, tile) = (f[bi].trim().to_string(), f[ti].trim().to_string());
        b2t.entry(bank.clone()).or_default().insert(tile.clone());
        t2b.insert(tile, bank);
    }
    Ok((t2b, b2t))
}
