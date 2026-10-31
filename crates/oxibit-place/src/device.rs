//! The device facts packing and placement need, read from prjxray-db:
//! sites with their types and tiles (`tilegrid.json`), package pins
//! (`package_pins.csv`). A first piece of the device image; for now read
//! from the database directory on each run.

use std::collections::{BTreeMap, HashMap};
use std::path::Path;

use serde_json::Value;

type R<T> = Result<T, String>;

#[derive(Debug, Clone)]
pub struct Tile {
    pub name: String,
    pub ty: String,
    pub grid_x: i32,
    pub grid_y: i32,
    pub clock_region: Option<String>,
    /// Site name -> site type.
    pub sites: BTreeMap<String, String>,
}

#[derive(Debug, Default)]
pub struct Device {
    pub part: String,
    pub tiles: HashMap<String, Tile>,
    /// Site -> its tile.
    pub site_tile: HashMap<String, String>,
    /// Package pin -> IOB site.
    pub pins: HashMap<String, String>,
}

fn read(p: &Path) -> R<String> {
    std::fs::read_to_string(p).map_err(|e| format!("{}: {e}", p.display()))
}

/// `child:` under top-level `key:` in a flat two-level YAML map.
fn yaml_child(text: &str, key: &str, child: &str) -> Option<String> {
    let mut inside = false;
    for line in text.lines() {
        if line.trim_start().starts_with('#') || line.trim().is_empty() {
            continue;
        }
        if !line.starts_with(' ') {
            inside = line.trim_end().trim_end_matches(':').trim().trim_matches('"') == key;
            continue;
        }
        if inside
            && let Some((k, v)) = line.trim().split_once(':')
            && k.trim() == child
        {
            return Some(v.trim().trim_matches('"').to_string());
        }
    }
    None
}

impl Device {
    /// From a prjxray-db family directory (`.../zynq7`) and part name.
    pub fn open(root: &Path, part: &str) -> R<Device> {
        let device =
            yaml_child(&read(&root.join("mapping/parts.yaml"))?, part, "device").ok_or_else(|| format!("{part} not in mapping/parts.yaml"))?;
        let fabric = yaml_child(&read(&root.join("mapping/devices.yaml"))?, &device, "fabric")
            .ok_or_else(|| format!("{device} not in mapping/devices.yaml"))?;
        let grid: Value = serde_json::from_str(&read(&root.join(&fabric).join("tilegrid.json"))?).map_err(|e| format!("tilegrid: {e}"))?;
        let mut d = Device {
            part: part.to_string(),
            ..Device::default()
        };
        for (name, t) in grid.as_object().ok_or("tilegrid is not an object")? {
            let sites: BTreeMap<String, String> = t["sites"]
                .as_object()
                .into_iter()
                .flatten()
                .map(|(s, ty)| (s.clone(), ty.as_str().unwrap_or("").to_string()))
                .collect();
            for s in sites.keys() {
                d.site_tile.insert(s.clone(), name.clone());
            }
            d.tiles.insert(
                name.clone(),
                Tile {
                    name: name.clone(),
                    ty: t["type"].as_str().unwrap_or("").to_string(),
                    grid_x: t["grid_x"].as_i64().unwrap_or(0) as i32,
                    grid_y: t["grid_y"].as_i64().unwrap_or(0) as i32,
                    clock_region: t["clock_region"].as_str().map(String::from),
                    sites,
                },
            );
        }
        let csv = read(&root.join(part).join("package_pins.csv"))?;
        for line in csv.lines().skip(1) {
            let f: Vec<&str> = line.split(',').collect();
            if f.len() >= 3 {
                d.pins.insert(f[0].to_string(), f[2].to_string());
            }
        }
        Ok(d)
    }

    pub fn site_type(&self, site: &str) -> Option<&str> {
        let t = self.site_tile.get(site)?;
        self.tiles[t].sites.get(site).map(String::as_str)
    }

    pub fn tile_of(&self, site: &str) -> Option<&Tile> {
        self.site_tile.get(site).map(|t| &self.tiles[t])
    }

    /// `site.bel` as a grid location: the tile by grid coordinates, the site
    /// relative to the tile's sites of the same prefix (`IOB_X1Y72` in a
    /// two-IOB tile -> `IOB_X0Y1`), as nextpnr writes bels.
    pub fn grid_loc(&self, site: &str, bel: &str) -> Option<oxibit_pnr::packed::Loc> {
        let t = self.tile_of(site)?;
        let (prefix, x, y) = parse_site(site)?;
        let same: Vec<(i64, i64)> = t
            .sites
            .keys()
            .filter_map(|s| parse_site(s))
            .filter(|(p, _, _)| *p == prefix)
            .map(|(_, x, y)| (x, y))
            .collect();
        let x0 = same.iter().map(|v| v.0).min().unwrap_or(x);
        let y0 = same.iter().map(|v| v.1).min().unwrap_or(y);
        Some(oxibit_pnr::packed::Loc::Grid {
            x: t.grid_x,
            y: t.grid_y,
            site: format!("{prefix}_X{}Y{}", x - x0, y - y0),
            bel: bel.to_string(),
        })
    }

    /// The IOLOGIC-side site (`OLOGIC`, `ILOGIC`, `IDELAY`) serving an IOB
    /// site: same coordinates, other prefix (7-series numbers them alike).
    pub fn io_logic_site(&self, iob: &str, prefix: &str) -> Option<String> {
        let s = format!("{prefix}_{}", iob.strip_prefix("IOB_")?);
        self.site_tile.contains_key(&s).then_some(s)
    }

    /// The IDELAYCTRL serving an IOB site: in the HCLK_IOI tile of the same
    /// clock region and I/O column.
    pub fn ioctrl_site(&self, iob: &str) -> Option<String> {
        let t = self.tile_of(iob)?;
        self.tiles
            .values()
            .filter(|h| h.ty.starts_with("HCLK_IOI") && h.clock_region == t.clock_region)
            .min_by_key(|h| (h.grid_x - t.grid_x).abs())
            .and_then(|h| h.sites.iter().find(|(_, ty)| ty.as_str() == "IDELAYCTRL").map(|(s, _)| s.clone()))
    }
}

/// `SLICE_X1Y0` -> ("SLICE", 1, 0).
pub fn parse_site(s: &str) -> Option<(String, i64, i64)> {
    let i = s.rfind("_X")?;
    let (x, y) = s[i + 2..].split_once('Y')?;
    Some((s[..i].to_string(), x.parse().ok()?, y.parse().ok()?))
}
