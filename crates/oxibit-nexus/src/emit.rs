//! FASM emission for a placed and routed Nexus design.
//!
//! Besides cell configuration and routed pips, a working Nexus bitstream
//! needs settings for things the design does not use, the same ones
//! nextpnr's Nexus FASM writer emits:
//! - unused input pins whose bel ties them high by leaving them floating
//!   ("CIBMUX" pins) get their CIB mux pointed at an undriven wire, so they
//!   really float high instead of following whatever the mux selects;
//! - every unused PIO gets `BASE_TYPE.NONE`;
//! - DSP primitives are put in their default mode even when unused;
//! - each 3.3 V capable bank gets its VCCIO.

use std::collections::{BTreeMap, HashSet};
use std::fmt::Write;

use crate::chip::Chip;
use crate::graph::{Arc, ArcKind, Graph, NodeId, PlacedBel};

pub struct Emit<'c, 'd> {
    pub chip: &'c Chip<'d>,
    pub g: &'c Graph,
    pub out: String,
    /// Nodes carrying a signal; a CIBMUX may not select one of these.
    pub used: HashSet<NodeId>,
    /// Bel groups (`R<y>C<x>_<bel>`) of IO bels the design uses.
    pub used_io: HashSet<String>,
}

impl<'c, 'd> Emit<'c, 'd> {
    pub fn new(chip: &'c Chip<'d>, g: &'c Graph) -> Self {
        let mut e = Emit { chip, g, out: String::new(), used: HashSet::new(), used_io: HashSet::new() };
        e.attr("oxide.device", &chip.db.device);
        let variant = chip.variant.clone();
        e.attr("oxide.device_variant", &variant);
        e.out.push('\n');
        e
    }

    pub fn attr(&mut self, k: &str, v: &str) {
        writeln!(self.out, "{{ {k}=\"{v}\" }}").ok();
    }

    pub fn line(&mut self, s: impl AsRef<str>) {
        self.out.push_str(s.as_ref());
        self.out.push('\n');
    }

    pub fn comment(&mut self, s: impl AsRef<str>) {
        writeln!(self.out, "# {}", s.as_ref()).ok();
    }

    /// Mark a route's nodes used and write its configurable pips.
    pub fn route(&mut self, arcs: &[Arc]) {
        for a in arcs {
            self.used.insert(a.from);
            self.used.insert(a.to);
            if a.kind == ArcKind::Pip {
                let l = self.g.fasm_pip(self.chip, a);
                self.line(l);
            } else {
                let l = format!("# fixed {} -> {}", self.g.node_name(a.from), self.g.node_name(a.to));
                self.line(l);
            }
        }
    }

    /// The CIB mux output feeding a bel input pin: walk uphill (first
    /// arc each step, up to 10 steps) to a node named `JCIBMUXOUT*`.
    pub fn find_cibmux(&self, pin: NodeId) -> Option<NodeId> {
        let mut cur = pin;
        for _ in 0..10 {
            if self.g.name(self.g.nodes[cur as usize].name).starts_with("JCIBMUXOUT") {
                return Some(cur);
            }
            cur = self.g.uphill(cur).next()?.from;
        }
        None
    }

    /// Leave an unused CIBMUX input pin floating high: select an undriven
    /// input on its CIB mux. Returns false if the pin has no CIB mux.
    pub fn float_pin(&mut self, pin: NodeId) -> bool {
        let Some(mux) = self.find_cibmux(pin) else { return false };
        let pick = self.g.uphill(mux).find(|a| !self.used.contains(&a.from)).copied();
        match pick {
            Some(a) => {
                self.used.insert(mux);
                if a.kind == ArcKind::Pip {
                    let l = self.g.fasm_pip(self.chip, &a);
                    self.line(l);
                }
                true
            }
            None => false,
        }
    }

    /// Float each named pin of a bel that the design leaves unconnected.
    pub fn float_pins(&mut self, bel: &PlacedBel, pins: &[&str]) {
        for p in pins {
            if let Some(n) = bel.pin(p) {
                if !self.used.contains(&n) {
                    self.float_pin(n);
                }
            }
        }
    }

    /// Bank VCCIO: `vcco` maps bank -> millivolts for banks with outputs.
    /// Banks 3-5 are the 1.8 V HP banks and take no VCC setting here.
    pub fn bank_cfg(&mut self, vcco: &BTreeMap<i32, u32>) {
        for b in [0, 1, 2, 6, 7] {
            if self.chip.db.device == "LIFCL-17" && b != 0 && b != 1 {
                continue;
            }
            let mv = vcco.get(&b).copied().unwrap_or(3300);
            self.line(format!("GLOBAL.BANK{b}.VCC.{}V{}", mv / 1000, (mv / 100) % 10));
        }
        self.out.push('\n');
    }

    /// `BASE_TYPE.NONE` for unused PIOs and default modes for unused DSPs.
    /// `used_cells` are bel groups occupied by the design.
    pub fn unused(&mut self, used_cells: &HashSet<String>) {
        self.comment("unused bels");
        let dsp_default: &[(&str, &[&str])] = &[
            ("MULT9_CORE", &["GSR.ENABLED", "MODE.NONE", "RSTAMUX.RSTA", "RSTPMUX.RSTP"]),
            ("PREADD9_CORE", &["GSR.ENABLED", "MODE.NONE", "RSTBMUX.RSTB", "RSTCLMUX.RSTCL"]),
            ("REG18_CORE", &["GSR.ENABLED", "MODE.NONE", "RSTPMUX.RSTP"]),
            ("ACC54_CORE", &["ACCUBYPS.BYPASS", "MODE.NONE"]),
        ];
        let mut lines = Vec::new();
        for b in &self.g.bels {
            let group = b.group();
            let pre = format!("{group}.{}", b.bel.name);
            match b.bel.beltype.as_str() {
                "SEIO33_CORE" if !self.used_io.contains(&group) => lines.push(format!("{pre}.BASE_TYPE.NONE")),
                "SEIO18_CORE" if !self.used_io.contains(&group) => {
                    lines.push(format!("{pre}.SEIO18.BASE_TYPE.NONE"))
                }
                t => {
                    if let Some((_, bits)) = dsp_default.iter().find(|(d, _)| *d == t) {
                        if !used_cells.contains(&group) {
                            lines.extend(bits.iter().map(|s| format!("{pre}.{s}")));
                        }
                    }
                }
            }
        }
        for l in lines {
            self.line(l);
        }
    }
}
