//! Instantiated clock primitives: DCCA with a clock enable, DCSC (global
//! network), ECLKSYNCB, ECLKBRIDGECS, CLKDIVF (edge clocks).
//!
//! The flow puts every clock through a DCC of its own choosing (`clock`);
//! a DCCA whose CE is tied high dissolves into that. A gated DCCA and a
//! DCSC are kept: each gets a bel here, its inputs route to the bel's pins
//! as clock sinks of their nets and its output is the source of its own
//! clock net. Their pins (global wires at the chip origin) are kept out of
//! every other net's search.
//!
//! - DCCA (CLKI, CE, CLKO): a DCC bel `<s>DCC<n>`; `DCC_<s><n>.MODE DCCA`
//!   in the tile that has it (`write_dcc`), CE routed from the fabric.
//! - DCSC (CLK0, CLK1, SEL0, SEL1, MODESEL, DCSOUT): DCS0 / DCS1 in the
//!   CMUX tiles; `DCS<n>.DCSMODE` (POS unless DCSMODE says otherwise).
//!   DCSOUT enters the centre muxes as `G_DCS<n>`.
//! - ECLKSYNCB (ECLKI, STOP, ECLKO): `ECLKSYNC<n>_BK<b>`, the edge clock
//!   of bank b; `.MODE ECLKSYNCB` in the ECLK_L / ECLK_R tile when STOP is
//!   connected (`write_bitstream`). A clock to an IOLOGIC ECLK with no
//!   ECLKSYNCB of its own passes a free one with no setting (`clock`).
//! - ECLKBRIDGECS (CLK0, CLK1, SEL, ECSOUT): `.MODE ECLKBRIDGECS`.
//! - CLKDIVF (CLKI, RST, ALIGNWD, CDIVX): `CLKDIV_<L|R><n>.DIV` (2.0 or
//!   3.5) and `.GSR` (DISABLED unless GSR says otherwise); CLKI from a
//!   bank ECLK, CDIVX into the global network (`G_J<L|R>CDIVX<n>`).
//!
//! Synthesis treats them as cut points: the random check does not model
//! the gating or the selection.

use std::collections::{BTreeMap, HashMap, HashSet};

use crate::bels::{BelId, Bels};
use crate::config::TileConfig;
use crate::db::DeviceDb;
use crate::graph::{Graph, Wire};
use crate::netlist::{Cell, Netlist, Prim};
use crate::{Result, err};

/// Port declarations, for synthesis.
pub const PRIMITIVES: &str = "
module DCSC(input CLK1, CLK0, SEL1, SEL0, MODESEL, output DCSOUT);
endmodule
module ECLKSYNCB(input ECLKI, STOP, output ECLKO);
endmodule
module ECLKBRIDGECS(input CLK0, CLK1, SEL, output ECSOUT);
endmodule
module CLKDIVF(input CLKI, RST, ALIGNWD, output CDIVX);
endmodule
module TRELLIS_ECLKBUF(input ECLKI, output ECLKO);
endmodule
";

/// Their clock inputs.
pub const CLOCK_INPUTS: &[(&str, &str)] = &[
    ("DCCA", "CLKI"),
    ("DCSC", "CLK0"),
    ("DCSC", "CLK1"),
    ("ECLKSYNCB", "ECLKI"),
    ("ECLKBRIDGECS", "CLK0"),
    ("ECLKBRIDGECS", "CLK1"),
    ("CLKDIVF", "CLKI"),
];

/// The order bels are chosen in: a primitive's driver gets its bel first.
const ORDER: &[&str] = &["ECLKBRIDGECS", "ECLKSYNCB", "CLKDIVF", "DCCA", "DCSC"];

/// The clock input `assign` checks a bel by.
pub fn clock_in(ty: &str) -> &'static str {
    match ty {
        "DCSC" | "ECLKBRIDGECS" => "CLK0",
        "ECLKSYNCB" => "ECLKI",
        _ => "CLKI",
    }
}

/// The clock output.
pub fn clock_out(ty: &str) -> &'static str {
    match ty {
        "DCSC" => "DCSOUT",
        "ECLKSYNCB" => "ECLKO",
        "ECLKBRIDGECS" => "ECSOUT",
        "CLKDIVF" => "CDIVX",
        _ => "CLKO",
    }
}

/// Whether an instance is a kept clock primitive (a DCCA only when gated).
pub fn is_clk(p: &Prim) -> bool {
    ORDER.contains(&p.ty.as_str())
}

/// The bel type a primitive takes.
fn bel_type(ty: &str) -> &str {
    ty
}

/// Bels for every kept clock primitive: a free bel of its type, in bel
/// order, preferring one `reach` accepts (it sees the bels chosen so far:
/// the drivers, by `ORDER`). ECLKSYNCB never takes a bridge's sync
/// (`BRGECLKSYNC`).
pub fn assign(
    bels: &Bels,
    nl: &Netlist,
    reach: impl Fn(&Prim, BelId, &HashMap<usize, BelId>) -> bool,
) -> Result<HashMap<usize, BelId>> {
    let mut out: HashMap<usize, BelId> = HashMap::new();
    let mut used: HashSet<BelId> = HashSet::new();
    let mut cells: Vec<(usize, &Prim)> = nl
        .cells
        .iter()
        .enumerate()
        .filter_map(|(i, c)| match c {
            Cell::Prim(p) if is_clk(p) => Some((i, &**p)),
            _ => None,
        })
        .collect();
    cells.sort_by_key(|&(i, p)| (ORDER.iter().position(|&t| t == p.ty).unwrap_or(9), i));
    // The nets an ECLKBRIDGECS drives: an ECLKSYNCB on one is the bridge's
    // own sync (BRGECLKSYNC, fixed to ECSOUT); any other ECLKSYNCB is not.
    let bridged: HashSet<u32> = nl
        .cells
        .iter()
        .filter_map(|c| match c {
            Cell::Prim(p) if p.ty == "ECLKBRIDGECS" => match p.pin("ECSOUT") {
                Some(crate::netlist::PrimPin::Out(Some(n))) => Some(n),
                _ => None,
            },
            _ => None,
        })
        .collect();
    for (i, p) in cells {
        let ty = bel_type(&p.ty);
        let brg = p.ty == "ECLKSYNCB"
            && matches!(p.pin("ECLKI"), Some(crate::netlist::PrimPin::In(Some(n))) if bridged.contains(&n));
        let free: Vec<BelId> = (0..bels.bels.len())
            .map(|k| BelId::try_from(k).expect("fits"))
            .filter(|&b| {
                let t = bels.template(b);
                t.ty == ty && !used.contains(&b) && t.name.starts_with("BRG") == brg
            })
            .collect();
        let b = free
            .iter()
            .copied()
            .find(|&b| reach(p, b, &out))
            .or_else(|| free.first().copied())
            .ok_or_else(|| err!("{} {}: no free {ty} bel", p.ty, p.name))?;
        used.insert(b);
        out.insert(i, b);
    }
    Ok(out)
}

/// Every pin wire of the assigned bels: kept out of other nets' searches.
pub fn pin_wires(bels: &Bels, at: &HashMap<usize, BelId>) -> HashSet<Wire> {
    at.values()
        .flat_map(|&b| bels.pins(b).map(|(_, w)| w).collect::<Vec<_>>())
        .collect()
}

/// Write the settings of every placed clock primitive.
pub fn configure(
    g: &Graph,
    db: &DeviceDb,
    bels: &Bels,
    nl: &Netlist,
    at: &HashMap<usize, BelId>,
    tiles: &mut BTreeMap<String, TileConfig>,
) -> Result<()> {
    let mut cells: Vec<usize> = at.keys().copied().collect();
    cells.sort_unstable();
    for i in cells {
        let Cell::Prim(p) = &nl.cells[i] else { continue };
        let bel = at[&i];
        let name = bels.template(bel).name.clone();
        let (setting, value) = match p.ty.as_str() {
            "DCCA" => {
                // <s>DCC<n> -> DCC_<s><n>.MODE
                let side = &name[..1];
                let n = name.trim_start_matches(|c: char| !c.is_ascii_digit());
                (format!("DCC_{side}{n}.MODE"), "DCCA".to_owned())
            }
            "ECLKSYNCB" | "ECLKBRIDGECS" => {
                // Only with STOP / SEL connected (`write_bitstream`).
                let ctl = if p.ty == "ECLKSYNCB" { "STOP" } else { "SEL" };
                if !matches!(p.pin(ctl), Some(crate::netlist::PrimPin::In(Some(_)))) {
                    continue;
                }
                (format!("{name}.MODE"), p.ty.clone())
            }
            "CLKDIVF" => {
                let side = if bels.bels[bel as usize].x > 5 { 'R' } else { 'L' };
                let n = name.trim_start_matches(|c: char| !c.is_ascii_digit());
                let div = p.param_str("DIV").unwrap_or_else(|| "2.0".to_owned());
                let gsr = p.param_str("GSR").unwrap_or_else(|| "DISABLED".to_owned());
                let t = side_tile(g, side)?;
                for (k, v) in [("DIV", div), ("GSR", gsr)] {
                    set_enum(db, t, &format!("CLKDIV_{side}{n}.{k}"), &v, &p.name, tiles)?;
                }
                continue;
            }
            "DCSC" => {
                let m = p
                    .param_str("DCSMODE")
                    .or_else(|| p.attrs.get("DCSMODE").cloned())
                    .unwrap_or_else(|| "POS".to_owned());
                (format!("{name}.DCSMODE"), m)
            }
            _ => continue,
        };
        let t = g
            .tiles
            .iter()
            .find(|t| db.tile_bits(&t.ty).is_ok_and(|b| b.enums.contains_key(&setting)))
            .ok_or_else(|| err!("{} {}: no tile has {setting}", p.ty, p.name))?;
        let ok = db.tile_bits(&t.ty)?.enums[&setting].options.contains_key(&value);
        if !ok {
            return Err(err!("{} {}: {setting} has no option {value}", p.ty, p.name));
        }
        tiles
            .entry(t.name.clone())
            .or_default()
            .enums
            .push((setting, value));
        // A DCCA's CE unconnected (or tied): high.
        if p.ty == "DCCA" && !matches!(p.pin("CE"), Some(crate::netlist::PrimPin::In(Some(_)))) {
            let w = bels.pin_wire(bel, "CE").ok_or_else(|| err!("{name}: no CE pin"))?;
            crate::io::tie(g, db, w, crate::chip::Level::High, tiles)?;
        }
    }
    Ok(())
}

/// The ECLK tile of one side.
fn side_tile(g: &Graph, side: char) -> Result<&crate::graph::TileLoc> {
    let ty = format!("ECLK_{side}");
    g.tiles
        .iter()
        .find(|t| t.ty == ty)
        .ok_or_else(|| err!("no {ty} tile"))
}

/// Set an enum in one tile, checking the option.
fn set_enum(
    db: &DeviceDb,
    t: &crate::graph::TileLoc,
    setting: &str,
    v: &str,
    who: &str,
    tiles: &mut BTreeMap<String, TileConfig>,
) -> Result<()> {
    let bits = db.tile_bits(&t.ty)?;
    let e = bits
        .enums
        .get(setting)
        .ok_or_else(|| err!("{who}: {} has no {setting}", t.name))?;
    if !e.options.contains_key(v) {
        return Err(err!("{who}: {setting} has no option {v}"));
    }
    tiles
        .entry(t.name.clone())
        .or_default()
        .enums
        .push((setting.to_owned(), v.to_owned()));
    Ok(())
}
