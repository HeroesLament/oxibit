//! Clock-network enables that follow from the routing: the FASM lines
//! nextpnr-himbaechel's xilinx `write_clocking` writes from the pips a
//! design uses (its second and third tile loops).
//!
//! Per tile, from the source wires of the pips used there:
//!
//! - `HCLK_L`/`HCLK_R` (and `_BOT_UTURN`): a `HCLK_CK_*` source that is a
//!   horizontal or regional clock buffer (`BUFHCLK`, `BUFRCLK`) enables it,
//!   `TILE.ENABLE_BUFFER.<wire>`, and marks that buffer used in the row;
//! - `CLK_HROW*`: a `CLK_HROW_R_CK_GCLK<n>` source, `TILE.<wire>_ACTIVE`,
//!   and marks `GCLK<n>` used; a `CLK_HROW_CK_IN*` source (not the
//!   `HROW_CK_INT` ones), `TILE.<wire>_ACTIVE`;
//! - `HCLK_CMT*`: a `HCLK_CMT_CCIO*` source, `_ACTIVE` and `_USED`; a
//!   `HCLK_CMT_CK_*` buffer source, `_USED`, and marks the buffer used in
//!   the row.
//!
//! Then every `CLK_BUFG_REBUF` enables each used GCLK above and below
//! (`GCLK<n>_ENABLE_ABOVE`, `_ENABLE_BELOW`), and every `HCLK_CMT*` marks
//! each buffer used in its row (`HCLK_CMT_CK_<buffer>_USED`). A row is the
//! tile's grid y.

use std::collections::{BTreeMap, BTreeSet};

use crate::lutinit::Locator;

/// A pip a design uses: (tile, destination wire, source wire).
pub type UsedPip = (String, String, String);

/// Where a regional buffer's name starts in a clock wire (`BUFHCLK`,
/// `BUFRCLK`), if it is one.
fn buffer_at(wire: &str) -> Option<usize> {
    wire.find("BUFHCLK").or_else(|| wire.find("BUFRCLK"))
}

fn is_hclk(tile_type: &str) -> bool {
    matches!(tile_type, "HCLK_L" | "HCLK_R" | "HCLK_L_BOT_UTURN" | "HCLK_R_BOT_UTURN")
}

/// The clock enables `pips` need.
pub fn clock_enables(pips: impl IntoIterator<Item = UsedPip>, loc: &Locator) -> BTreeSet<String> {
    let mut out = BTreeSet::new();
    let mut gclks: BTreeSet<String> = BTreeSet::new();
    let mut by_row: BTreeMap<i64, BTreeSet<String>> = BTreeMap::new();
    for (tile, _dst, src) in pips {
        let Some(tt) = loc.tile_type(&tile) else { continue };
        let row = loc.grid(&tile).map(|p| p.1).unwrap_or(-1);
        if is_hclk(tt) {
            if src.starts_with("HCLK_CK_") {
                if let Some(at) = buffer_at(&src) {
                    out.insert(format!("{tile}.ENABLE_BUFFER.{src}"));
                    by_row.entry(row).or_default().insert(src[at..].to_string());
                }
            }
        } else if tt.starts_with("CLK_HROW") {
            if src.starts_with("CLK_HROW_R_CK_GCLK") {
                out.insert(format!("{tile}.{src}_ACTIVE"));
                if let Some(at) = src.find("GCLK") {
                    gclks.insert(src[at..].to_string());
                }
            } else if src.starts_with("CLK_HROW_CK_IN") && !src.contains("HROW_CK_INT") {
                out.insert(format!("{tile}.{src}_ACTIVE"));
            }
        } else if tt.starts_with("HCLK_CMT") {
            if src.starts_with("HCLK_CMT_CCIO") {
                out.insert(format!("{tile}.{src}_ACTIVE"));
                out.insert(format!("{tile}.{src}_USED"));
            } else if src.starts_with("HCLK_CMT_CK_") {
                if let Some(at) = buffer_at(&src) {
                    out.insert(format!("{tile}.{src}_USED"));
                    by_row.entry(row).or_default().insert(src[at..].to_string());
                }
            }
        }
    }
    for (tile, tt) in loc.tiles() {
        if tt == "CLK_BUFG_REBUF" {
            for g in &gclks {
                out.insert(format!("{tile}.{g}_ENABLE_ABOVE"));
                out.insert(format!("{tile}.{g}_ENABLE_BELOW"));
            }
        } else if tt.starts_with("HCLK_CMT") {
            let row = loc.grid(tile).map(|p| p.1).unwrap_or(-1);
            for b in by_row.get(&row).into_iter().flatten() {
                out.insert(format!("{tile}.HCLK_CMT_CK_{b}_USED"));
            }
        }
    }
    out
}

/// Whether a FASM line is a clock enable `clock_enables` writes (and so
/// follows the routing, not the cells).
pub fn is_clock_enable(line: &str, loc: &Locator) -> bool {
    let f = line.split('#').next().unwrap_or("").split('=').next().unwrap_or("").trim();
    let parts: Vec<&str> = f.split('.').collect();
    let Some(tt) = parts.first().and_then(|t| loc.tile_type(t)) else { return false };
    match parts[..] {
        [_, "ENABLE_BUFFER", w] => is_hclk(tt) && w.starts_with("HCLK_CK_"),
        [_, w] if tt.starts_with("CLK_HROW") => {
            w.ends_with("_ACTIVE") && (w.starts_with("CLK_HROW_R_CK_GCLK") || w.starts_with("CLK_HROW_CK_IN"))
        }
        [_, w] if tt.starts_with("HCLK_CMT") => {
            (w.ends_with("_ACTIVE") || w.ends_with("_USED")) && (w.starts_with("HCLK_CMT_CCIO") || w.starts_with("HCLK_CMT_CK_"))
        }
        [_, w] if tt == "CLK_BUFG_REBUF" => w.starts_with("GCLK") && (w.ends_with("_ENABLE_ABOVE") || w.ends_with("_ENABLE_BELOW")),
        _ => false,
    }
}

/// The pips of a FASM, for `clock_enables`: three-part lines
/// `TILE.DST.SRC` of a known tile that are not enables.
pub fn fasm_pips(fasm: &str, loc: &Locator) -> Vec<UsedPip> {
    let mut out = Vec::new();
    for l in fasm.lines() {
        let f = l.split('#').next().unwrap_or("").trim();
        if f.contains('=') || f.contains('[') {
            continue;
        }
        let parts: Vec<&str> = f.split('.').collect();
        if let [t, d, s] = parts[..] {
            if d != "ENABLE_BUFFER" && loc.tile_type(t).is_some() {
                out.push((t.to_string(), d.to_string(), s.to_string()));
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn buffers() {
        assert_eq!(buffer_at("HCLK_CK_BUFHCLK9"), Some(8));
        assert_eq!(buffer_at("HCLK_CMT_CK_BUFRCLK2"), Some(12));
        assert_eq!(buffer_at("HCLK_CK_OUTL0"), None);
    }
}
