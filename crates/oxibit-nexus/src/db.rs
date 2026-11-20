//! prjoxide database, one device at a time.
//!
//! `root` is prjoxide's `database` directory (holding `devices.json`).
//! Everything a device needs is read at open: its tile grid, IP base
//! addresses, the bit database of every tile type in its grid and of each
//! IP type. A tile type with no `.ron` file has no configuration, which is
//! how prjoxide treats it too.

use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::fs;
use std::path::{Path, PathBuf};

use serde::Deserialize;

use crate::{Result, err};

#[derive(Deserialize)]
struct Devices {
    families: BTreeMap<String, Family>,
}

#[derive(Deserialize)]
struct Family {
    devices: BTreeMap<String, DeviceData>,
}

#[derive(Deserialize, Clone, Debug)]
pub struct Variant {
    pub idcode: u32,
}

#[derive(Deserialize, Clone, Debug)]
pub struct DeviceData {
    pub frames: usize,
    pub bits_per_frame: usize,
    pub pad_bits_after_frame: usize,
    pub pad_bits_before_frame: usize,
    pub frame_ecc_bits: usize,
    pub max_row: u32,
    pub max_col: u32,
    pub variants: BTreeMap<String, Variant>,
}

#[derive(Deserialize, Clone, Debug)]
pub struct TileData {
    pub tiletype: String,
    pub x: u32,
    pub y: u32,
    pub start_bit: usize,
    pub start_frame: usize,
    pub bits: usize,
    pub frames: usize,
}

#[derive(Deserialize)]
struct Tilegrid {
    tiles: BTreeMap<String, TileData>,
}

#[derive(Deserialize, Clone, Debug)]
pub struct AddrRegion {
    pub addr: u32,
    pub abits: u32,
}

#[derive(Deserialize)]
struct BaseAddrs {
    regions: BTreeMap<String, AddrRegion>,
}

#[derive(Deserialize, PartialEq, Eq, PartialOrd, Ord, Clone, Copy, Debug)]
pub struct ConfigBit {
    pub frame: usize,
    pub bit: usize,
    pub invert: bool,
}

#[derive(Deserialize, Clone, Debug)]
pub struct Pip {
    pub from_wire: String,
    pub bits: BTreeSet<ConfigBit>,
}

#[derive(Deserialize, Clone, Debug)]
pub struct Word {
    pub bits: Vec<BTreeSet<ConfigBit>>,
    #[serde(default)]
    pub desc: String,
}

#[derive(Deserialize, Clone, Debug)]
pub struct Enum {
    pub options: BTreeMap<String, BTreeSet<ConfigBit>>,
    #[serde(default)]
    pub desc: String,
}

#[derive(Deserialize, Clone, Debug)]
pub struct Conn {
    pub from_wire: String,
    #[serde(default)]
    pub bidir: bool,
}

/// Bits of one tile type or IP type.
#[derive(Deserialize, Clone, Debug, Default)]
pub struct TileBits {
    pub pips: BTreeMap<String, Vec<Pip>>,
    pub words: BTreeMap<String, Word>,
    pub enums: BTreeMap<String, Enum>,
    pub conns: BTreeMap<String, Vec<Conn>>,
    #[serde(default)]
    pub always_on: BTreeSet<ConfigBit>,
}

/// Global clock network: branches (columns fed from a TAP), spines and
/// HROWs. Used to resolve `BRANCH:`, `SPINE:` and `HROW:` wires.
#[derive(Deserialize, Clone, Debug, Default)]
pub struct Globals {
    pub branches: Vec<Branch>,
    pub spines: Vec<Spine>,
    pub hrows: Vec<Hrow>,
}

#[derive(Deserialize, Clone, Debug)]
pub struct Branch {
    pub branch_col: usize,
    pub from_col: usize,
    pub tap_driver_col: usize,
    pub tap_side: String,
    pub to_col: usize,
}

#[derive(Deserialize, Clone, Debug)]
pub struct Spine {
    pub from_row: usize,
    pub spine_row: usize,
    pub to_row: usize,
}

#[derive(Deserialize, Clone, Debug)]
pub struct Hrow {
    pub hrow_col: usize,
    pub spine_cols: Vec<usize>,
}

impl Globals {
    pub fn branch_sink_to_origin(&self, col: usize) -> Option<usize> {
        self.branches.iter().find(|b| col >= b.from_col && col <= b.to_col).map(|b| b.branch_col)
    }
    pub fn spine_sink_to_origin(&self, x: usize, y: usize) -> Option<(usize, usize)> {
        let col = self
            .hrows
            .iter()
            .flat_map(|h| h.spine_cols.iter())
            .find(|&&c| (x as i64 - c as i64).abs() < 3)?;
        self.spines.iter().find(|s| y >= s.from_row && y <= s.to_row).map(|s| (*col, s.spine_row))
    }
    pub fn hrow_sink_to_origin(&self, x: usize, y: usize) -> Option<(usize, usize)> {
        let near = |a: usize, b: usize| (a as i64 - b as i64).abs() < 3;
        let col = self
            .hrows
            .iter()
            .find(|h| h.spine_cols.iter().any(|&c| near(x, c)) || near(x, h.hrow_col))?
            .hrow_col;
        self.spines
            .iter()
            .find(|s| (y as i64 - s.spine_row as i64).abs() <= 3)
            .map(|s| (col, s.spine_row))
    }
}

/// One bonded or unbonded pad. `pins[i]` is the ball in `packages[i]`
/// ("-" when not bonded); `side`/`offset`/`pio` locate its PIO bel.
#[derive(Deserialize, Clone, Debug)]
pub struct Pad {
    pub bank: i32,
    pub dqs: Vec<i32>,
    pub func: Vec<String>,
    pub offset: i32,
    pub pins: Vec<String>,
    pub pio: i32,
    pub side: String,
    pub vref: i32,
}

#[derive(Deserialize, Clone, Debug, Default)]
pub struct IoDb {
    pub packages: Vec<String>,
    pub pads: Vec<Pad>,
}

impl IoDb {
    /// The pad on `ball` of `package` (e.g. "CABGA400", "E17").
    pub fn pad(&self, package: &str, ball: &str) -> Option<&Pad> {
        let i = self.packages.iter().position(|p| p == package)?;
        self.pads.iter().find(|p| p.pins.get(i).is_some_and(|b| b == ball) && p.pio >= 0)
    }
}

/// IP types prjoxide has a bit database for.
pub const IP_TYPES: [&str; 5] = ["EBR_INIT", "LRAM_INIT", "PLL_CORE", "DPHY_CORE", "PCIE_CORE"];

#[derive(Debug)]
pub struct DeviceDb {
    pub root: PathBuf,
    pub family: String,
    pub device: String,
    pub data: DeviceData,
    /// In name order, which is the order prjoxide copies tile windows in.
    pub tiles: Vec<(String, TileData)>,
    pub baseaddrs: BTreeMap<String, AddrRegion>,
    pub globals: Globals,
    pub iodb: IoDb,
    pub tilebits: HashMap<String, TileBits>,
    pub ipbits: HashMap<String, TileBits>,
    empty: TileBits,
}

fn read(path: &Path) -> Result<String> {
    fs::read_to_string(path).map_err(|e| err!("{}: {e}", path.display()))
}

fn ron_file(path: &Path) -> Result<TileBits> {
    if !path.exists() {
        return Ok(TileBits::default());
    }
    ron::de::from_str(&read(path)?).map_err(|e| err!("{}: {e}", path.display()))
}

impl DeviceDb {
    pub fn open(root: &Path, device: &str) -> Result<DeviceDb> {
        let devices: Devices = serde_json::from_str(&read(&root.join("devices.json"))?)
            .map_err(|e| err!("devices.json: {e}"))?;
        let (family, data) = devices
            .families
            .iter()
            .find_map(|(f, fd)| fd.devices.get(device).map(|d| (f.clone(), d.clone())))
            .ok_or_else(|| err!("no device {device} in {}", root.display()))?;
        let dir = root.join(&family).join(device);
        let grid: Tilegrid = serde_json::from_str(&read(&dir.join("tilegrid.json"))?)
            .map_err(|e| err!("tilegrid.json: {e}"))?;
        let base: BaseAddrs = serde_json::from_str(&read(&dir.join("baseaddr.json"))?)
            .map_err(|e| err!("baseaddr.json: {e}"))?;
        let globals: Globals = serde_json::from_str(&read(&dir.join("globals.json"))?)
            .map_err(|e| err!("globals.json: {e}"))?;
        let iodb: IoDb = serde_json::from_str(&read(&dir.join("iodb.json"))?)
            .map_err(|e| err!("iodb.json: {e}"))?;
        let types: BTreeSet<&str> = grid.tiles.values().map(|t| t.tiletype.as_str()).collect();
        let mut tilebits = HashMap::new();
        for tt in types {
            let path = root.join(&family).join("tiletypes").join(format!("{tt}.ron"));
            tilebits.insert(tt.to_string(), ron_file(&path)?);
        }
        let mut ipbits = HashMap::new();
        for it in IP_TYPES {
            let path = root.join(&family).join("iptypes").join(format!("{it}.ron"));
            ipbits.insert(it.to_string(), ron_file(&path)?);
        }
        Ok(DeviceDb {
            root: root.to_path_buf(),
            family,
            device: device.to_string(),
            data,
            tiles: grid.tiles.into_iter().collect(),
            baseaddrs: base.regions,
            globals,
            iodb,
            tilebits,
            ipbits,
            empty: TileBits::default(),
        })
    }

    /// The device named by a FASM file's `oxide.device` attribute.
    pub fn device_of(fasm: &crate::ParsedFasm) -> Result<&str> {
        fasm.attr("oxide.device").ok_or_else(|| err!("FASM has no oxide.device attribute"))
    }

    pub fn tile_bits(&self, tiletype: &str) -> &TileBits {
        self.tilebits.get(tiletype).unwrap_or(&self.empty)
    }

    pub fn ip_bits(&self, iptype: &str) -> Result<&TileBits> {
        self.ipbits.get(iptype).ok_or_else(|| err!("no IP type {iptype}"))
    }
}
