//! The DDR memory interface blocks: DQSBUFM (one per DQS group, beside
//! its DQS pad) and DDRDLLA (the corner delay-locked loops), as nextpnr's
//! `pack_dqsbuf`, `write_dqsbuf` and DDRDLL writer place and set them.
//!
//! A DQSBUFM sits on the `DQSBUF` bel of the PIC tile of its DQS pad: DQSI
//! must come from a pad that iodb.json names `LDQS<n>` / `RDQS<n>` (an A
//! pad). Its group signals (DQSR90, DQSW, DQSW270, RDPNTR0..2,
//! WRPNTR0..2) reach the IOLOGICs of the group's DQ pads on the DQSG wires,
//! which span the group in silicon and carry no configuration: the flow
//! checks that each sink is in the DQSBUFM's group and does not route them
//! (nextpnr marks them global and leaves them unrouted). Its unused
//! control inputs (READ, READCLKSEL, DYNDELAY, the MOVE / DIRECTION pins)
//! are routed constant 0 (nextpnr `tie_zero`).
//!
//! A DDRDLLA takes the DDRDLL bel of the corner its DQSBUFM users are
//! beside (left / right by column, upper / lower by row); its DDRDEL code
//! reaches the DQSBUFMs over the `G_<corner>DDRDEL` globals.

use std::collections::{BTreeMap, HashMap};
use std::path::Path;

use crate::bels::{BelId, Bels};
use crate::config::TileConfig;
use crate::db::DeviceDb;
use crate::graph::Graph;
use crate::io::PioSite;
use crate::netlist::{Cell, Netlist, Prim, PrimPin};
use crate::{Result, err};

/// Port declarations (the Lattice library's port lists), for synthesis.
pub const PRIMITIVES: &str = "
module DQSBUFM(input DQSI, READ1, READ0, READCLKSEL2, READCLKSEL1, READCLKSEL0, DDRDEL, ECLK,
  SCLK, RST, DYNDELAY7, DYNDELAY6, DYNDELAY5, DYNDELAY4, DYNDELAY3, DYNDELAY2, DYNDELAY1,
  DYNDELAY0, PAUSE, RDLOADN, RDMOVE, RDDIRECTION, WRLOADN, WRMOVE, WRDIRECTION,
  output DQSR90, DQSW, DQSW270, RDPNTR2, RDPNTR1, RDPNTR0, WRPNTR2, WRPNTR1, WRPNTR0,
  DATAVALID, BURSTDET, RDCFLAG, WRCFLAG);
endmodule
module DDRDLLA(input CLK, RST, UDDCNTLN, FREEZE, output DDRDEL, LOCK, DCNTL7, DCNTL6, DCNTL5,
  DCNTL4, DCNTL3, DCNTL2, DCNTL1, DCNTL0);
endmodule
";

pub const TYPES: &[&str] = &["DQSBUFM", "DDRDLLA"];

pub fn is_dqs(ty: &str) -> bool {
    TYPES.contains(&ty)
}

/// Clock inputs (on the clock network; ECLK on the bank's edge clock).
pub fn is_clock_input(ty: &str, pin: &str) -> bool {
    matches!((ty, pin), ("DQSBUFM", "ECLK" | "SCLK" | "DDRDEL") | ("DDRDLLA", "CLK"))
}

/// The DQSBUFM outputs that reach the group's IOLOGICs on the DQSG wires.
pub const GROUP_SIGNALS: &[&str] = &[
    "DQSR90", "DQSW", "DQSW270", "RDPNTR0", "RDPNTR1", "RDPNTR2", "WRPNTR0", "WRPNTR1", "WRPNTR2",
];

/// The DQSBUFM inputs nextpnr ties to 0 when the design leaves them open.
pub const TIE_ZERO: &[&str] = &[
    "RDMOVE", "RDDIRECTION", "WRMOVE", "WRDIRECTION", "READ0", "READ1", "READCLKSEL0", "READCLKSEL1",
    "READCLKSEL2", "DYNDELAY0", "DYNDELAY1", "DYNDELAY2", "DYNDELAY3", "DYNDELAY4", "DYNDELAY5",
    "DYNDELAY6", "DYNDELAY7",
];

/// Each PIO's DQS group (`iodb.json` `dqs`: `RDQ17`, `RDQS17`, `RDQSN17`
/// are all group `R17`) and whether it is the group's DQS pad.
pub fn load_groups(root: &Path, device: &str) -> Result<HashMap<(i16, i16, char), (String, bool)>> {
    let path = root.join("ECP5").join(device).join("iodb.json");
    let text = std::fs::read_to_string(&path).map_err(|e| err!("{}: {e}", path.display()))?;
    let v: serde_json::Value = serde_json::from_str(&text).map_err(|e| err!("{}: {e}", path.display()))?;
    let mut out = HashMap::new();
    for p in v["pio_metadata"].as_array().ok_or(err!("iodb.json: pio_metadata"))? {
        let (Some(dqs), Some(col), Some(row), Some(pio)) = (
            p["dqs"].as_str(),
            p["col"].as_i64(),
            p["row"].as_i64(),
            p["pio"].as_str().and_then(|s| s.chars().next()),
        ) else {
            continue;
        };
        let (Ok(col), Ok(row)) = (i16::try_from(col), i16::try_from(row)) else { continue };
        let side = &dqs[..1];
        let rest = &dqs[1..];
        let (n, strobe) = if let Some(n) = rest.strip_prefix("DQSN") {
            (n, false)
        } else if let Some(n) = rest.strip_prefix("DQS") {
            (n, true)
        } else if let Some(n) = rest.strip_prefix("DQ") {
            (n, false)
        } else {
            continue;
        };
        out.insert((row, col, pio), (format!("{side}{n}"), strobe));
    }
    Ok(out)
}

fn input(p: &Prim, pin: &str) -> Option<u32> {
    match p.pin(pin) {
        Some(PrimPin::In(Some(n))) => Some(n),
        _ => None,
    }
}

/// Bels for the DQSBUFMs (by their DQS pad) and DDRDLLAs (by the corner
/// of their users). `pad_of` gives the pad (as a site) a net comes from.
pub fn assign(
    bels: &Bels,
    nl: &Netlist,
    groups: &HashMap<(i16, i16, char), (String, bool)>,
    pad_of: impl Fn(u32) -> Option<PioSite>,
    height: i16,
) -> Result<HashMap<usize, BelId>> {
    let mut out = HashMap::new();
    for (i, c) in nl.cells.iter().enumerate() {
        let Cell::Prim(p) = c else { continue };
        if p.ty != "DQSBUFM" {
            continue;
        }
        let dqsi = input(p, "DQSI").ok_or_else(|| err!("DQSBUFM {}: DQSI must be connected to a top level input", p.name))?;
        let site = pad_of(dqsi).ok_or_else(|| err!("DQSBUFM {}: DQSI must come straight from a pad", p.name))?;
        match groups.get(&(site.row, site.col, site.pio)) {
            Some((_, true)) => {}
            _ => {
                return Err(err!(
                    "DQSBUFM {}: its DQSI pad (R{}C{} PIO{}) is not a DQS pad",
                    p.name,
                    site.row,
                    site.col,
                    site.pio
                ));
            }
        }
        let b = bels
            .find(site.col, site.row, "DQSBUF")
            .ok_or_else(|| err!("DQSBUFM {}: no DQSBUF bel at R{}C{}", p.name, site.row, site.col))?;
        if out.values().any(|&o| o == b) {
            return Err(err!("DQSBUFM {}: two DQSBUFMs on one DQS group", p.name));
        }
        out.insert(i, b);
    }
    // DDRDLLAs: the corner of the DQSBUFMs they feed.
    let dll_bels: Vec<BelId> = (0..bels.bels.len())
        .map(|i| BelId::try_from(i).expect("fits"))
        .filter(|&b| bels.template(b).ty == "DDRDLL")
        .collect();
    let mid_x = bels.bels.iter().map(|b| b.x).max().unwrap_or(0) / 2;
    for (i, c) in nl.cells.iter().enumerate() {
        let Cell::Prim(p) = c else { continue };
        if p.ty != "DDRDLLA" {
            continue;
        }
        if input(p, "CLK").is_none() {
            return Err(err!("DDRDLLA {}: CLK is not connected", p.name));
        }
        let mut users: Vec<(i16, i16)> = Vec::new();
        if let Some(PrimPin::Out(Some(n))) = p.pin("DDRDEL") {
            for (j, c2) in nl.cells.iter().enumerate() {
                if let Cell::Prim(q) = c2 {
                    if q.ty == "DQSBUFM" && input(q, "DDRDEL") == Some(n) {
                        let b = out[&j];
                        users.push((bels.bels[b as usize].x, bels.bels[b as usize].y));
                    }
                }
            }
        }
        let right = users.iter().any(|&(x, _)| x > mid_x);
        let left = users.iter().any(|&(x, _)| x <= mid_x);
        if right && left {
            return Err(err!("DDRDLLA {}: DDRDEL used on both sides of the chip", p.name));
        }
        let upper = users.iter().all(|&(_, y)| y < height / 2);
        let lower = users.iter().all(|&(_, y)| y >= height / 2);
        let b = dll_bels
            .iter()
            .copied()
            .filter(|&b| !out.values().any(|&o| o == b))
            .find(|&b| {
                let bel = bels.bels[b as usize];
                let r = bel.x > mid_x;
                let u = bel.y < height / 2;
                (users.is_empty() || (r == right && ((u && upper) || (!u && lower))))
            })
            .ok_or_else(|| err!("DDRDLLA {}: no free DDRDLL in the corner of its DQSBUFMs", p.name))?;
        out.insert(i, b);
    }
    Ok(out)
}

/// The DQS group of the IOLOGIC bel `b` (its pad's group).
pub fn bel_group<'a>(bels: &Bels, b: BelId, groups: &'a HashMap<(i16, i16, char), (String, bool)>) -> Option<&'a str> {
    let t = bels.template(b);
    let pio = t.name.strip_prefix("IOLOGIC").or_else(|| t.name.strip_prefix("DQSBUF").map(|_| "A"))?.chars().next()?;
    let bel = bels.bels[b as usize];
    groups.get(&(bel.y, bel.x, pio)).map(|(g, _)| g.as_str())
}

/// Write each DQSBUFM's and DDRDLLA's settings.
pub fn configure(
    g: &Graph,
    db: &DeviceDb,
    bels: &Bels,
    nl: &Netlist,
    at: &HashMap<usize, BelId>,
    tiles: &mut BTreeMap<String, TileConfig>,
) -> Result<()> {
    let mut cells: Vec<(usize, BelId)> = at.iter().map(|(&c, &b)| (c, b)).collect();
    cells.sort_unstable();
    for (c, b) in cells {
        let Cell::Prim(p) = &nl.cells[c] else { continue };
        let bel = bels.bels[b as usize];
        let s = |k: &str, d: &str| p.param_str(k).unwrap_or_else(|| d.to_owned());
        if p.ty == "DQSBUFM" {
            let side = if bel.x < 10 { "PICL" } else { "PICR" };
            let want = [
                (bel.y - 2, format!("{side}1_DQS0")),
                (bel.y - 1, format!("{side}2_DQS1")),
                (bel.y, format!("{side}0_DQS2")),
                (bel.y + 1, format!("{side}1_DQS3")),
            ];
            let mut tnames = Vec::new();
            for (y, ty) in &want {
                let t = g
                    .tiles
                    .iter()
                    .find(|t| t.x == bel.x && t.y == *y && &t.ty == ty)
                    .ok_or_else(|| err!("DQSBUFM {}: no {ty} tile at R{y}C{}", p.name, bel.x))?;
                tnames.push((t.name.clone(), t.ty.clone()));
            }
            let used = |pin: &str| matches!(p.pin(pin), Some(PrimPin::In(Some(_))));
            let del = |adj: &str, val: &str| -> Vec<bool> {
                let mut v = p.param_int(val).unwrap_or(0);
                if s(adj, "PLUS") == "MINUS" {
                    v = (256 - v) & 0xFF;
                }
                (0..8).map(|k| (v >> k) & 1 == 1).collect()
            };
            let enums: Vec<(String, String)> = vec![
                ("DQS.MODE".into(), "DQSBUFM".into()),
                ("DQS.DQS_LI_DEL_ADJ".into(), s("DQS_LI_DEL_ADJ", "PLUS")),
                ("DQS.DQS_LO_DEL_ADJ".into(), s("DQS_LO_DEL_ADJ", "PLUS")),
                ("DQS.WRLOADN_USED".into(), if used("WRLOADN") { "YES" } else { "NO" }.into()),
                ("DQS.RDLOADN_USED".into(), if used("RDLOADN") { "YES" } else { "NO" }.into()),
                ("DQS.PAUSE_USED".into(), if used("PAUSE") { "YES" } else { "NO" }.into()),
                ("DQS.READ_USED".into(), if used("READ0") || used("READ1") { "YES" } else { "NO" }.into()),
                ("DQS.DDRDEL".into(), if used("DDRDEL") { "DDRDEL" } else { "0" }.into()),
                ("DQS.GSR".into(), s("GSR", "DISABLED")),
            ];
            let words = vec![
                ("DQS.DQS_LI_DEL_VAL".to_owned(), del("DQS_LI_DEL_ADJ", "DQS_LI_DEL_VAL")),
                ("DQS.DQS_LO_DEL_VAL".to_owned(), del("DQS_LO_DEL_ADJ", "DQS_LO_DEL_VAL")),
            ];
            for (k, v) in &enums {
                let mut n = 0;
                for (name, ty) in &tnames {
                    if let Some(e) = db.tile_bits(ty)?.enums.get(k) {
                        if !e.options.contains_key(v) {
                            return Err(err!("DQSBUFM {}: {k} has no option {v}", p.name));
                        }
                        tiles.entry(name.clone()).or_default().enums.push((k.clone(), v.clone()));
                        n += 1;
                    }
                }
                if n == 0 {
                    return Err(err!("DQSBUFM {}: no tile of its group has {k}", p.name));
                }
            }
            for (k, v) in &words {
                for (name, ty) in &tnames {
                    if db.tile_bits(ty)?.words.contains_key(k) {
                        tiles.entry(name.clone()).or_default().words.push((k.clone(), v.clone()));
                    }
                }
            }
        } else if p.ty == "DDRDLLA" {
            let height = g.height;
            let mid_x = g.width / 2;
            let ty = format!(
                "DDRDLL_{}{}",
                if bel.y < height / 2 { 'U' } else { 'L' },
                if bel.x > mid_x { 'R' } else { 'L' }
            );
            let t = g
                .tiles
                .iter()
                .find(|t| t.ty == ty)
                .ok_or_else(|| err!("DDRDLLA {}: no {ty} tile", p.name))?;
            for (k, v) in [
                ("DDRDLL.MODE", "DDRDLLA".to_owned()),
                ("DDRDLL.GSR", s("GSR", "DISABLED")),
                ("DDRDLL.FORCE_MAX_DELAY", s("FORCE_MAX_DELAY", "NO")),
            ] {
                let e = db
                    .tile_bits(&t.ty)?
                    .enums
                    .get(k)
                    .ok_or_else(|| err!("DDRDLLA {}: {} has no {k}", p.name, t.name))?;
                if !e.options.contains_key(&v) {
                    return Err(err!("DDRDLLA {}: {k} has no option {v}", p.name));
                }
                tiles.entry(t.name.clone()).or_default().enums.push((k.to_owned(), v));
            }
        }
    }
    Ok(())
}
