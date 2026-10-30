//! A FASM under construction, every line with the writer that owns it.
//!
//! Writers add lines; a feature written twice with different values, or two
//! options of one routing bel (`PRECYINIT.C0` and `PRECYINIT.CIN`), is a
//! conflict, recorded and not written. The same line from the same writer
//! twice is one line. `Owner::Base` is a line taken from another tool's
//! FASM (the cell configuration oxibit does not write yet): the count of
//! those is what is left to own.

use std::collections::BTreeMap;

/// Who wrote a line.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Owner {
    /// Taken as it is from the design's FASM.
    Base,
    /// A pip of a routed net.
    Route,
    /// An `always` pseudo-pip on a default constant path (no bits; a clock
    /// buffer's relay names it).
    Always,
    /// A LUT INIT from the cells (`lutinit`).
    LutInit,
    /// A slice routing-bel setting from the site paths (`placed`).
    SiteMux,
    /// Slice FF, control set, LUT mode and carry-in (`cellcfg`).
    SliceCfg,
    /// A DSP48E1 input tie-off (`cellcfg::dsp_ties`).
    DspTie,
    /// A clock-network enable the routing needs (`clocking`).
    Clocking,
    /// A BUFGCTRL cell's settings (`cellcfg::bufg_config`).
    Bufg,
    /// A DSP48E1 cell's settings (`cellcfg::dsp_config`).
    Dsp,
}

impl Owner {
    pub fn name(self) -> &'static str {
        match self {
            Owner::Base => "base",
            Owner::Route => "route",
            Owner::Always => "always",
            Owner::LutInit => "lut_init",
            Owner::SiteMux => "site_mux",
            Owner::SliceCfg => "slice_cfg",
            Owner::DspTie => "dsp_tie",
            Owner::Clocking => "clocking",
            Owner::Bufg => "bufg",
            Owner::Dsp => "dsp",
        }
    }
}

/// Two writers disagree on a feature or a routing bel.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Conflict {
    /// The feature, or the routing bel (`TILE.SITE.PRECYINIT`).
    pub key: String,
    pub kept: (Owner, String),
    pub refused: (Owner, String),
}

#[derive(Debug, Default, Clone)]
pub struct FasmOut {
    /// Feature (the line up to `=`) -> (owner, whole line).
    lines: BTreeMap<String, (Owner, String)>,
    /// Routing bel -> (owner, feature of the option set).
    options: BTreeMap<String, (Owner, String)>,
    conflicts: Vec<Conflict>,
}

/// The feature of a line: up to `=` and `#`, trimmed.
pub fn feature(line: &str) -> &str {
    line.split('#').next().unwrap_or("").split('=').next().unwrap_or("").trim()
}

/// The routing bel a slice line selects an option of (`T.S.PRECYINIT`,
/// `T.S.AFFMUX`, `T.S.A5FFMUX`, `T.S.AOUTMUX`), if it is one.
fn option_of(feature: &str) -> Option<String> {
    let p: Vec<&str> = feature.split('.').collect();
    let [t, s, bel, _opt] = p[..] else { return None };
    if !t.starts_with("CLBL") || !s.starts_with("SLICE") {
        return None;
    }
    let letter = bel.len() > 1 && "ABCD".contains(&bel[..1]);
    let rest = if letter { &bel[1..] } else { bel };
    let exclusive = (letter && matches!(rest, "FFMUX" | "5FFMUX" | "OUTMUX")) || bel == "PRECYINIT";
    exclusive.then(|| format!("{t}.{s}.{bel}"))
}

impl FasmOut {
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a line for `owner`. Blank lines and comments are ignored.
    pub fn add(&mut self, owner: Owner, line: &str) {
        let line = line.trim();
        let f = feature(line);
        if f.is_empty() {
            return;
        }
        if let Some((o, l)) = self.lines.get(f) {
            if l != line {
                self.conflicts.push(Conflict { key: f.to_string(), kept: (*o, l.clone()), refused: (owner, line.to_string()) });
            }
            return;
        }
        if let Some(bel) = option_of(f) {
            if let Some((o, other)) = self.options.get(&bel) {
                if other != f {
                    self.conflicts.push(Conflict { key: bel, kept: (*o, other.clone()), refused: (owner, line.to_string()) });
                    return;
                }
            } else {
                self.options.insert(bel, (owner, f.to_string()));
            }
        }
        self.lines.insert(f.to_string(), (owner, line.to_string()));
    }

    pub fn conflicts(&self) -> &[Conflict] {
        &self.conflicts
    }

    /// Lines per owner.
    pub fn counts(&self) -> BTreeMap<Owner, usize> {
        let mut c = BTreeMap::new();
        for (o, _) in self.lines.values() {
            *c.entry(*o).or_default() += 1;
        }
        c
    }

    /// The lines of one owner.
    pub fn of(&self, owner: Owner) -> impl Iterator<Item = &str> {
        self.lines.values().filter(move |(o, _)| *o == owner).map(|(_, l)| l.as_str())
    }

    /// The FASM, one line per feature, in feature order.
    pub fn text(&self) -> String {
        let mut out = String::new();
        for (_, l) in self.lines.values() {
            out.push_str(l);
            out.push('\n');
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn owners_and_conflicts() {
        let mut f = FasmOut::new();
        f.add(Owner::Route, "INT_L_X1Y1.IMUX_L4.GFAN0");
        f.add(Owner::Route, "INT_L_X1Y1.IMUX_L4.GFAN0");
        f.add(Owner::LutInit, "CLBLL_L_X2Y3.SLICEL_X0.ALUT.INIT[63:0] = 64'h00000000000000FF");
        f.add(Owner::Base, "CLBLL_L_X2Y3.SLICEL_X0.ALUT.INIT[63:0] = 64'h0000000000000001");
        f.add(Owner::SiteMux, "CLBLL_L_X2Y3.SLICEL_X0.PRECYINIT.C0");
        f.add(Owner::SliceCfg, "CLBLL_L_X2Y3.SLICEL_X0.PRECYINIT.CIN");
        f.add(Owner::SliceCfg, "CLBLL_L_X2Y3.SLICEL_X0.PRECYINIT.C0");
        f.add(Owner::Base, "# comment");
        assert_eq!(f.conflicts().len(), 2);
        assert_eq!(f.conflicts()[0].key, "CLBLL_L_X2Y3.SLICEL_X0.ALUT.INIT[63:0]");
        assert_eq!(f.conflicts()[1].key, "CLBLL_L_X2Y3.SLICEL_X0.PRECYINIT");
        assert_eq!(f.counts().get(&Owner::Route), Some(&1));
        assert_eq!(
            f.text(),
            "CLBLL_L_X2Y3.SLICEL_X0.ALUT.INIT[63:0] = 64'h00000000000000FF\nCLBLL_L_X2Y3.SLICEL_X0.PRECYINIT.C0\nINT_L_X1Y1.IMUX_L4.GFAN0\n"
        );
    }

    #[test]
    fn options() {
        assert_eq!(option_of("CLBLL_L_X2Y3.SLICEL_X0.AFFMUX.O6").as_deref(), Some("CLBLL_L_X2Y3.SLICEL_X0.AFFMUX"));
        assert_eq!(option_of("CLBLL_L_X2Y3.SLICEL_X0.PRECYINIT.AX").as_deref(), Some("CLBLL_L_X2Y3.SLICEL_X0.PRECYINIT"));
        assert_eq!(option_of("CLBLL_L_X2Y3.SLICEL_X0.AFF.ZINI"), None);
        assert_eq!(option_of("CLBLL_L_X2Y3.SLICEL_X0.CARRY4.ACY0"), None);
    }
}
