//! EHXPLLL: bel choice and configuration.
//!
//! The settings follow nextpnr's `write_pll` fuse for fuse (dividers and
//! output dividers written minus one, phases as given, loop-filter and
//! MFG words from the instance's attributes with nextpnr's defaults), so a
//! PLL built here sets the same bits a nextpnr build of the same instance
//! does. They live in the `PLL0_*` / `PLL1_*` (or `BANKREF*`) tiles next to
//! the bel and carry no bel prefix; each is written into every tile there
//! that has it.
//!
//! Routing is the flow's: CLKI and CLKFB are clock sinks (the reference
//! reaches CLKI through the PLLREFCS select, see `clock`), CLKOP..CLKOS3
//! drive clock trees through the DCCs, LOCK and the control inputs are
//! general nets. Constant control inputs arrive as routed `$gnd` / `$vcc`
//! nets, as nextpnr routes them, rather than as CIB ties.

use std::collections::{BTreeMap, HashMap};

use crate::bels::{BelId, Bels};
use crate::config::TileConfig;
use crate::db::DeviceDb;
use crate::graph::Graph;
use crate::netlist::{Cell, Netlist, Prim, PrimPin};
use crate::{Result, err};

/// A PLL bel for every EHXPLLL, nearest its `target` (row, col) first.
pub fn assign(
    bels: &Bels,
    nl: &Netlist,
    target: impl Fn(usize) -> (f64, f64),
) -> Result<HashMap<usize, BelId>> {
    let mut free: Vec<BelId> = (0..bels.bels.len())
        .map(|i| BelId::try_from(i).expect("fits"))
        .filter(|&b| bels.template(b).ty == "EHXPLLL")
        .collect();
    let mut out = HashMap::new();
    for (i, c) in nl.cells.iter().enumerate() {
        let Cell::Prim(p) = c else { continue };
        if p.ty != "EHXPLLL" {
            continue;
        }
        let (r, col) = target(i);
        let d = |b: BelId| {
            let bel = bels.bels[b as usize];
            let dy = f64::from(bel.y) - r;
            let dx = f64::from(bel.x) - col;
            dx * dx + dy * dy
        };
        let k = (0..free.len())
            .min_by(|&a, &b| d(free[a]).total_cmp(&d(free[b])))
            .ok_or_else(|| err!("PLL {}: more PLLs than the device has", p.name))?;
        out.insert(i, free.swap_remove(k));
    }
    Ok(out)
}

/// The tiles holding a PLL's settings: `PLL*` and `BANKREF*` tiles within
/// two of the bel.
fn holds(g: &Graph, bx: i16, by: i16) -> Vec<usize> {
    g.tiles
        .iter()
        .enumerate()
        .filter(|(_, t)| {
            (t.ty.starts_with("PLL") || t.ty.starts_with("BANKREF"))
                && (t.x - bx).abs() <= 2
                && (t.y - by).abs() <= 2
        })
        .map(|(i, _)| i)
        .collect()
}

struct Writer<'a> {
    g: &'a Graph,
    db: &'a DeviceDb,
    tiles: Vec<usize>,
    name: &'a str,
}

impl Writer<'_> {
    fn word(&self, setting: &str, value: i64, bits: usize, cfg: &mut BTreeMap<String, TileConfig>) -> Result<()> {
        if value < 0 || (bits < 63 && value >= 1 << bits) {
            return Err(err!("PLL {}: {setting} = {value} does not fit {bits} bits", self.name));
        }
        let v: Vec<bool> = (0..bits).map(|k| (value >> k) & 1 == 1).collect();
        let mut n = 0;
        for &t in &self.tiles {
            let tile = &self.g.tiles[t];
            let Some(w) = self.db.tile_bits(&tile.ty)?.words.get(setting) else {
                continue;
            };
            if w.bits.len() != bits {
                return Err(err!("{}: {setting} is {} bits, not {bits}", tile.name, w.bits.len()));
            }
            cfg.entry(tile.name.clone()).or_default().words.push((setting.to_owned(), v.clone()));
            n += 1;
        }
        if n == 0 {
            return Err(err!("PLL {}: no tile has {setting}", self.name));
        }
        Ok(())
    }

    fn enumv(&self, setting: &str, value: &str, cfg: &mut BTreeMap<String, TileConfig>) -> Result<()> {
        let mut n = 0;
        for &t in &self.tiles {
            let tile = &self.g.tiles[t];
            let Some(e) = self.db.tile_bits(&tile.ty)?.enums.get(setting) else {
                continue;
            };
            if !e.options.contains_key(value) {
                return Err(err!("PLL {}: {setting} has no option {value}", self.name));
            }
            cfg.entry(tile.name.clone()).or_default().enums.push((setting.to_owned(), value.to_owned()));
            n += 1;
        }
        if n == 0 {
            return Err(err!("PLL {}: no tile has {setting}", self.name));
        }
        Ok(())
    }
}

fn attr_int(p: &Prim, name: &str, default: i64) -> Result<i64> {
    match p.attrs.get(name) {
        None => Ok(default),
        Some(v) => {
            let v = v.trim();
            let parsed = if let Some(b) = v.strip_prefix("0b") {
                i64::from_str_radix(b, 2).ok()
            } else {
                v.parse().ok()
            };
            parsed.ok_or_else(|| err!("PLL {}: attribute {name} = {v:?} is not a number", p.name))
        }
    }
}

/// Write every PLL's settings.
pub fn configure(
    g: &Graph,
    db: &DeviceDb,
    bels: &Bels,
    nl: &Netlist,
    pll_at: &HashMap<usize, BelId>,
    cfg: &mut BTreeMap<String, TileConfig>,
) -> Result<()> {
    for (i, c) in nl.cells.iter().enumerate() {
        let Cell::Prim(p) = c else { continue };
        if p.ty != "EHXPLLL" {
            continue;
        }
        let b = *pll_at.get(&i).ok_or_else(|| err!("PLL {} has no bel", p.name))?;
        let bel = bels.bels[b as usize];
        let w = Writer {
            g,
            db,
            tiles: holds(g, bel.x, bel.y),
            name: &p.name,
        };
        let s = |k: &str, d: &str| p.param_str(k).unwrap_or_else(|| d.to_owned());
        let int = |k: &str, d: i64| p.param_int(k).unwrap_or(d);
        w.enumv("MODE", "EHXPLLL", cfg)?;
        w.word("CLKI_DIV", int("CLKI_DIV", 1) - 1, 7, cfg)?;
        w.word("CLKFB_DIV", int("CLKFB_DIV", 1) - 1, 7, cfg)?;
        w.enumv("CLKOP_ENABLE", &s("CLKOP_ENABLE", "ENABLED"), cfg)?;
        w.enumv("CLKOS_ENABLE", &s("CLKOS_ENABLE", "ENABLED"), cfg)?;
        w.enumv("CLKOS2_ENABLE", &s("CLKOS2_ENABLE", "ENABLED"), cfg)?;
        w.enumv("CLKOS3_ENABLE", &s("CLKOS3_ENABLE", "ENABLED"), cfg)?;
        for out in ["CLKOP", "CLKOS", "CLKOS2", "CLKOS3"] {
            w.word(&format!("{out}_DIV"), int(&format!("{out}_DIV"), 8) - 1, 7, cfg)?;
            w.word(&format!("{out}_CPHASE"), int(&format!("{out}_CPHASE"), 0), 7, cfg)?;
            w.word(&format!("{out}_FPHASE"), int(&format!("{out}_FPHASE"), 0), 3, cfg)?;
        }
        w.enumv("FEEDBK_PATH", &s("FEEDBK_PATH", "CLKOP"), cfg)?;
        w.enumv("CLKOP_TRIM_POL", &s("CLKOP_TRIM_POL", "RISING"), cfg)?;
        w.enumv("CLKOP_TRIM_DELAY", &s("CLKOP_TRIM_DELAY", "0"), cfg)?;
        w.enumv("CLKOS_TRIM_POL", &s("CLKOS_TRIM_POL", "RISING"), cfg)?;
        w.enumv("CLKOS_TRIM_DELAY", &s("CLKOS_TRIM_DELAY", "0"), cfg)?;
        let clkop_used = matches!(p.pin("CLKOP"), Some(PrimPin::Out(Some(_))));
        for (k, d) in [("A", "DIVA"), ("B", "DIVB"), ("C", "DIVC"), ("D", "DIVD")] {
            let def = if clkop_used { d } else { "REFCLK" };
            w.enumv(&format!("OUTDIVIDER_MUX{k}"), &s(&format!("OUTDIVIDER_MUX{k}"), def), cfg)?;
        }
        w.word("PLL_LOCK_MODE", int("PLL_LOCK_MODE", 0), 3, cfg)?;
        for k in ["STDBY_ENABLE", "REFIN_RESET", "SYNC_ENABLE", "DPHASE_SOURCE", "PLLRST_ENA", "INTFB_WAKE"] {
            w.enumv(k, &s(k, "DISABLED"), cfg)?;
        }
        w.enumv("INT_LOCK_STICKY", &s("INT_LOCK_STICKY", "ENABLED"), cfg)?;
        for (k, bits, def) in [
            ("KVCO", 3, 0),
            ("LPF_CAPACITOR", 2, 0),
            ("LPF_RESISTOR", 7, 0),
            ("ICP_CURRENT", 5, 0),
            ("FREQ_LOCK_ACCURACY", 2, 0),
            ("MFG_GMC_GAIN", 3, 0),
            ("MFG_GMC_TEST", 4, 14),
            ("MFG1_TEST", 3, 0),
            ("MFG2_TEST", 3, 0),
            ("MFG_FORCE_VFILTER", 1, 0),
            ("MFG_ICP_TEST", 1, 0),
            ("MFG_EN_UP", 1, 0),
            ("MFG_FLOAT_ICP", 1, 0),
            ("MFG_GMC_PRESET", 1, 0),
            ("MFG_LF_PRESET", 1, 0),
            ("MFG_GMC_RESET", 1, 0),
            ("MFG_LF_RESET", 1, 0),
            ("MFG_LF_RESGRND", 1, 0),
            ("MFG_GMCREF_SEL", 2, 0),
            ("MFG_ENABLE_FILTEROPAMP", 1, 0),
        ] {
            w.word(k, attr_int(p, k, def)?, bits, cfg)?;
        }
    }
    Ok(())
}

/// The frequencies a PLL's settings give, for the report: (f_pfd, f_vco,
/// outputs), MHz, from `f_in` MHz; `None` for an unknown feedback path.
pub fn frequencies(p: &Prim, f_in: f64) -> Option<(f64, f64, Vec<(String, f64)>)> {
    let int = |k: &str, d: i64| p.param_int(k).unwrap_or(d);
    let path = p.param_str("FEEDBK_PATH").unwrap_or_else(|| "CLKOP".to_owned());
    let fb_out = match path.as_str() {
        "CLKOP" | "INT_OP" => "CLKOP",
        "CLKOS" | "INT_OS" => "CLKOS",
        "CLKOS2" | "INT_OS2" => "CLKOS2",
        "CLKOS3" | "INT_OS3" => "CLKOS3",
        _ => return None,
    };
    #[expect(clippy::cast_precision_loss, reason = "small integer dividers")]
    let div = |k: &str| int(k, 8).max(1) as f64;
    #[expect(clippy::cast_precision_loss, reason = "small integer dividers")]
    let (ci, cf) = (int("CLKI_DIV", 1).max(1) as f64, int("CLKFB_DIV", 1).max(1) as f64);
    let pfd = f_in / ci;
    let vco = pfd * cf * div(&format!("{fb_out}_DIV"));
    let outs = ["CLKOP", "CLKOS", "CLKOS2", "CLKOS3"]
        .iter()
        .filter(|o| matches!(p.pin(o), Some(PrimPin::Out(Some(_)))))
        .map(|o| ((*o).to_owned(), vco / div(&format!("{o}_DIV"))))
        .collect();
    Some((pfd, vco, outs))
}
