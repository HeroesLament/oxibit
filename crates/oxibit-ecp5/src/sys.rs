//! Configuration-block primitives: JTAGG, OSCG, GSR / SGSR, USRMCLK,
//! DTR, SEDGA (and PUR, which has no bel).
//!
//! Each has one bel. Its user-side pins reach the fabric through fixed
//! CIB connections and are routed like any other bel pin; OSCG's output
//! is a global wire (`G_JOSC_OSC`) and clocks through the global network.
//! Settings follow nextpnr's `write_bitstream`, by tile type:
//!
//! - JTAGG: `JTAG.ER1` / `JTAG.ER2` (EFB0), ENABLED unless the instance
//!   says otherwise. The dedicated TCK / TMS / TDI / TDO are left alone.
//! - OSCG: `OSC.DIV` (EFB0 and EFB1; DIV 128 is written as 127, the
//!   largest option), `OSC.MODE OSCG` (EFB1). 310 MHz / DIV nominal.
//! - GSR: `GSR.GSRMODE ACTIVE_LOW` (EFB0), `GSR.SYNCMODE` (VIQ_BUF):
//!   ASYNC for GSR, SYNC for SGSR (synchronous to its CLK). Every slice,
//!   IOLOGIC and EBR register with GSR enabled (the slice default) takes
//!   its REGSET value while GSR is low.
//! - USRMCLK: `CCLK.MODE USRMCLK` (EFB3); USRMCLKI / USRMCLKTS drive the
//!   CCLK pad (bel pins PADDO / PADDT).
//! - DTR: `DTR.MODE DTR`.
//! - SEDGA: `SED.CLK_FREQ` from SED_CLK_FREQ, `SED.CHECKALWAYS` (EFB2).
//! - PUR: power-up reset is a simulation construct; synthesis drops it.
//!
//! Synthesis treats these as cut points: GSR's effect on the registers is
//! not in the random check.

use std::collections::{BTreeMap, HashMap};

use crate::bels::{BelId, Bels};
use crate::config::TileConfig;
use crate::db::DeviceDb;
use crate::graph::Graph;
use crate::netlist::{Cell, Netlist};
use crate::{Result, err};

/// Kept primitive types this module places.
pub const TYPES: &[&str] = &["JTAGG", "OSCG", "GSR", "SGSR", "USRMCLK", "DTR", "SEDGA"];

/// Port declarations, for synthesis (JTAGG's are in `synth::PRIMITIVES`).
pub const PRIMITIVES: &str = "
module OSCG(output OSC);
endmodule
module GSR(input GSR);
endmodule
module SGSR(input GSR, CLK);
endmodule
module PUR(input PUR);
endmodule
module USRMCLK(input USRMCLKI, USRMCLKTS);
endmodule
module DTR(input STARTPULSE, output DTROUT7, DTROUT6, DTROUT5, DTROUT4, DTROUT3,
  DTROUT2, DTROUT1, DTROUT0);
endmodule
module SEDGA(input SEDENABLE, SEDSTART, SEDFRCERR, output SEDDONE, SEDINPROG, SEDERR);
endmodule
";

pub fn is_sys(ty: &str) -> bool {
    TYPES.contains(&ty)
}

/// The bel type of a primitive.
fn bel_type(ty: &str) -> &str {
    match ty {
        "SGSR" => "GSR",
        t => t,
    }
}

/// The bel pin of a primitive pin (USRMCLK's are the CCLK pad's).
pub fn bel_pin<'a>(ty: &str, pin: &'a str) -> &'a str {
    match (ty, pin) {
        ("USRMCLK", "USRMCLKI") => "PADDO",
        ("USRMCLK", "USRMCLKTS") => "PADDT",
        _ => pin,
    }
}

/// The bel of every configuration primitive (at most one of each).
pub fn assign(bels: &Bels, nl: &Netlist) -> Result<HashMap<usize, BelId>> {
    let mut out = HashMap::new();
    for (i, c) in nl.cells.iter().enumerate() {
        let Cell::Prim(p) = c else { continue };
        if !is_sys(&p.ty) {
            continue;
        }
        let ty = bel_type(&p.ty);
        let b = (0..bels.bels.len())
            .map(|k| BelId::try_from(k).expect("fits"))
            .find(|&b| bels.template(b).ty == ty)
            .ok_or_else(|| err!("{} {}: the device has no {ty} bel", p.ty, p.name))?;
        if out.values().any(|&o| o == b) {
            return Err(err!("{} {}: only one {ty} per device", p.ty, p.name));
        }
        out.insert(i, b);
    }
    Ok(out)
}

/// Set `setting` to `v` in every tile of type `ty`.
fn set(
    g: &Graph,
    db: &DeviceDb,
    ty: &str,
    setting: &str,
    v: &str,
    who: &str,
    cfg: &mut BTreeMap<String, TileConfig>,
) -> Result<()> {
    let mut n = 0;
    for t in g.tiles.iter().filter(|t| t.ty == ty) {
        let Some(e) = db.tile_bits(&t.ty)?.enums.get(setting) else {
            continue;
        };
        // `_NONE_` names the enum and sets nothing, so its default is not
        // written either (`CCLK.MODE`: its default clears an OSC.DIV bit).
        if v != "_NONE_" && !e.options.contains_key(v) {
            return Err(err!("{who}: {setting} has no option {v}"));
        }
        cfg.entry(t.name.clone())
            .or_default()
            .enums
            .push((setting.to_owned(), v.to_owned()));
        n += 1;
    }
    if n == 0 {
        return Err(err!("{who}: no {ty} tile has {setting}"));
    }
    Ok(())
}

/// Write the settings of every placed configuration primitive.
pub fn configure(
    g: &Graph,
    db: &DeviceDb,
    nl: &Netlist,
    at: &HashMap<usize, BelId>,
    cfg: &mut BTreeMap<String, TileConfig>,
) -> Result<()> {
    let mut cells: Vec<usize> = at.keys().copied().collect();
    cells.sort_unstable();
    for i in cells {
        let Cell::Prim(p) = &nl.cells[i] else { continue };
        let who = format!("{} {}", p.ty, p.name);
        let s = |name: &str, default: &str| p.param_str(name).unwrap_or_else(|| default.to_owned());
        match p.ty.as_str() {
            "JTAGG" => {
                for er in ["ER1", "ER2"] {
                    set(g, db, "EFB0_PICB0", &format!("JTAG.{er}"), &s(er, "ENABLED"), &who, cfg)?;
                }
            }
            "OSCG" => {
                let div = p.param_int("DIV").unwrap_or(128);
                if !(2..=128).contains(&div) {
                    return Err(err!("{who}: DIV {div} is not 2..128"));
                }
                let d = div.min(127).to_string();
                set(g, db, "EFB0_PICB0", "OSC.DIV", &d, &who, cfg)?;
                set(g, db, "EFB1_PICB1", "OSC.DIV", &d, &who, cfg)?;
                set(g, db, "EFB1_PICB1", "OSC.MODE", "OSCG", &who, cfg)?;
                // OSC.DIV's fourth EFB1 bit is CCLK.MODE's: the CCLK.MODE
                // default (NONE, the bit clear) must not be written over it
                // (nextpnr writes `CCLK.MODE _NONE_` for this). Silicon:
                // DIV 8 ran at DIV 79's rate, DIV 26 at about DIV 116's.
                set(g, db, "EFB1_PICB1", "CCLK.MODE", "_NONE_", &who, cfg)?;
            }
            "GSR" | "SGSR" => {
                set(g, db, "EFB0_PICB0", "GSR.GSRMODE", "ACTIVE_LOW", &who, cfg)?;
                let sync = if p.ty == "SGSR" { "SYNC" } else { "ASYNC" };
                set(g, db, "VIQ_BUF", "GSR.SYNCMODE", sync, &who, cfg)?;
            }
            "USRMCLK" => set(g, db, "EFB3_PICB1", "CCLK.MODE", "USRMCLK", &who, cfg)?,
            "DTR" => set(g, db, "DTR", "DTR.MODE", "DTR", &who, cfg)?,
            "SEDGA" => {
                let f = s("SED_CLK_FREQ", "2.4");
                set(g, db, "EFB2_PICB0", "SED.CLK_FREQ", &f, &who, cfg)?;
                let always = s("CHECKALWAYS", "DISABLED");
                set(g, db, "EFB2_PICB0", "SED.CHECKALWAYS", &always, &who, cfg)?;
                // SEDGA has no SEDEXCLK: the Trellis fuzzer (105-sedga) wrote
                // YES only with a net on SEDEXCLK. Left YES, SED never ran
                // (clk_probe, p21: in-progress and done stayed 0).
                set(g, db, "EFB2_PICB0", "SED.SEDEXCLK_USED", "NO", &who, cfg)?;
            }
            _ => {}
        }
    }
    Ok(())
}

/// LPF `SYSCONFIG` settings, as nextpnr writes them: the configuration
/// port and wake-up enums into their EFB tiles, `CONFIG_IOVOLTAGE` as bank
/// 8's VCCIO (over the pads' own), the rest (MCCLK_FREQ, COMPRESS_CONFIG,
/// ...) as `.sysconfig` for the bitstream writer.
pub fn sysconfig(
    g: &Graph,
    db: &DeviceDb,
    settings: &BTreeMap<String, String>,
    cc: &mut crate::config::ChipConfig,
) -> Result<()> {
    for (k, v) in settings {
        let who = format!("SYSCONFIG {k}");
        let name = format!("SYSCONFIG.{k}");
        let tiles: &[&str] = match k.as_str() {
            "SLAVE_SPI_PORT" | "DONE_EX" => &["EFB0_PICB0", "EFB2_PICB0"],
            "SLAVE_PARALLEL_PORT" | "BACKGROUND_RECONFIG" | "WAKE_UP" => &["EFB0_PICB0"],
            "MASTER_SPI_PORT" => &["EFB1_PICB1"],
            "TRANSFR" => &["EFB0_PICB0", "EFB1_PICB1"],
            "CONFIG_IOVOLTAGE" => {
                // "3.3" -> "3V3".
                let vcc = v.replacen('.', "V", 1);
                let mut n = 0;
                for t in g.tiles.iter().filter(|t| t.ty.starts_with("BANKREF8")) {
                    let bits = db.tile_bits(&t.ty)?;
                    if !bits.enums.get("BANK.VCCIO").is_some_and(|e| e.options.contains_key(&vcc)) {
                        return Err(err!("{who}: bank 8 has no VCCIO {vcc}"));
                    }
                    let tc = cc.tiles.entry(t.name.clone()).or_default();
                    tc.enums.retain(|(e, _)| e != "BANK.VCCIO");
                    tc.enums.push(("BANK.VCCIO".to_owned(), vcc.clone()));
                    n += 1;
                }
                if n == 0 {
                    return Err(err!("{who}: no BANKREF8 tile"));
                }
                continue;
            }
            _ => {
                cc.sysconfig.insert(k.clone(), v.clone());
                continue;
            }
        };
        for ty in tiles {
            set(g, db, ty, &name, v, &who, &mut cc.tiles)?;
        }
    }
    Ok(())
}
