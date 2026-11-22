//! Configuration-block primitives: JTAGG.
//!
//! JTAGG has one bel. Its user-side pins (JTCK, JTDI, JSHIFT, JUPDATE,
//! JRSTN, JCE1/2, JRTI1/2 out; JTDO1/2 in) reach the fabric through fixed
//! CIB connections and are routed like any other bel pin. TCK, TMS, TDI
//! and TDO are the dedicated pins and are left unconnected. The ER1 / ER2
//! user instructions (0x32 / 0x38) are enabled by `JTAG.ER1` / `JTAG.ER2`
//! in the EFB tile, ENABLED unless the instance says otherwise (as
//! nextpnr's `write_bitstream` does).

use std::collections::{BTreeMap, HashMap};

use crate::bels::{BelId, Bels};
use crate::config::TileConfig;
use crate::db::DeviceDb;
use crate::graph::Graph;
use crate::netlist::{Cell, Netlist};
use crate::{Result, err};

/// Kept primitive types this module places.
pub const TYPES: &[&str] = &["JTAGG"];

/// The bel of every JTAGG instance (at most one).
pub fn assign(bels: &Bels, nl: &Netlist) -> Result<HashMap<usize, BelId>> {
    let mut out = HashMap::new();
    for (i, c) in nl.cells.iter().enumerate() {
        let Cell::Prim(p) = c else { continue };
        if !TYPES.contains(&p.ty.as_str()) {
            continue;
        }
        let b = (0..bels.bels.len())
            .map(|k| BelId::try_from(k).expect("fits"))
            .find(|&b| bels.template(b).ty == p.ty)
            .ok_or_else(|| err!("{} {}: the device has no {} bel", p.ty, p.name, p.ty))?;
        if out.values().any(|&o| o == b) {
            return Err(err!("{} {}: only one {} per device", p.ty, p.name, p.ty));
        }
        out.insert(i, b);
    }
    Ok(out)
}

/// Write the settings of every placed configuration primitive.
pub fn configure(
    g: &Graph,
    db: &DeviceDb,
    nl: &Netlist,
    at: &HashMap<usize, BelId>,
    cfg: &mut BTreeMap<String, TileConfig>,
) -> Result<()> {
    for &i in at.keys() {
        let Cell::Prim(p) = &nl.cells[i] else { continue };
        if p.ty != "JTAGG" {
            continue;
        }
        for er in ["ER1", "ER2"] {
            let v = p.param_str(er).unwrap_or_else(|| "ENABLED".to_owned());
            let setting = format!("JTAG.{er}");
            let mut n = 0;
            for t in g.tiles.iter().filter(|t| t.ty.starts_with("EFB")) {
                let Some(e) = db.tile_bits(&t.ty)?.enums.get(&setting) else {
                    continue;
                };
                if !e.options.contains_key(v.as_str()) {
                    return Err(err!("JTAGG {}: {er} has no option {v}", p.name));
                }
                cfg.entry(t.name.clone()).or_default().enums.push((setting.clone(), v.clone()));
                n += 1;
            }
            if n == 0 {
                return Err(err!("JTAGG {}: no tile has {setting}", p.name));
            }
        }
    }
    Ok(())
}
