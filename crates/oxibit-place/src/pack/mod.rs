//! Packing: primitives into bel-sized cells with bel-pin ports.
//!
//! The passes follow nextpnr-xilinx's packer (ISC, Myrtle Shah and
//! contributors) closely enough that its output is a reference: the same
//! cell types, port names, `X_ORIG_*` attributes, feed-through LUTs and
//! constant ties, so `examples/pack_compare` can hold the two side by side.
//! Where oxibit decides differently (on purpose, for routability) the pass
//! says so.
//!
//! Relative placement (LUT with its flop, carry chains, wide-mux trees) is
//! not part of the netlist; the passes record it in `Packer::clusters` for
//! the placer.

use std::collections::BTreeMap;

use serde_json::{Value, json};

use crate::netlist::{CellId, Dir, NetId, Netlist};

pub mod bram;
pub mod carry;
pub mod clocking;
pub mod constants;
pub mod dram;
pub mod dsp;
pub mod ffs;
pub mod io;
pub mod luts;
pub mod muxf;
pub mod srl;

/// A port and parameter rewrite from one primitive to a packed cell type
/// (nextpnr's `XFormRule`).
#[derive(Debug, Clone, Default)]
pub struct Rule {
    pub new_type: String,
    /// Port renames; a port not listed keeps its name with `[` `]` removed.
    pub port: BTreeMap<String, String>,
    /// One logical port onto several bel pins (or none).
    pub multi: BTreeMap<String, Vec<String>>,
    /// Parameters copied to a new name.
    pub param: BTreeMap<String, String>,
    pub set_attrs: Vec<(String, Value)>,
    pub set_params: Vec<(String, Value)>,
}

impl Rule {
    pub fn to(new_type: &str) -> Rule {
        Rule {
            new_type: new_type.to_string(),
            ..Rule::default()
        }
    }

    pub fn port(mut self, from: &str, to: &str) -> Rule {
        self.port.insert(from.to_string(), to.to_string());
        self
    }

    pub fn attr(mut self, k: &str, v: Value) -> Rule {
        self.set_attrs.push((k.to_string(), v));
        self
    }

    pub fn param(mut self, k: &str, v: Value) -> Rule {
        self.set_params.push((k.to_string(), v));
        self
    }
}

/// Where a cell sits relative to its cluster's root: tile offset and bel
/// slot. The root's own slot may be absolute.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Offset {
    pub dx: i32,
    pub dy: i32,
    /// Bel slot (nextpnr's z: `letter << 4 | bel`), relative to the root's
    /// unless `abs`.
    pub z: i32,
    pub abs: bool,
}

/// Bel slots within a slice (nextpnr-xilinx's `LogicBelTypeZ`): low nibble
/// the function, high nibble the letter (A = 0 .. D = 3); `z = letter << 4 |
/// function`.
pub mod slot {
    pub const LUT6: i32 = 0x0;
    pub const LUT5: i32 = 0x1;
    pub const FF: i32 = 0x2;
    pub const FF2: i32 = 0x3;
    pub const F7MUX: i32 = 0x7;
    pub const F8MUX: i32 = 0x8;
    pub const CARRY4: i32 = 0xF;

    pub fn at(letter: i32, function: i32) -> i32 {
        letter << 4 | function
    }
}

/// A group of cells placed together: a root and members at offsets.
#[derive(Debug, Clone, Default)]
pub struct Cluster {
    pub root_abs_z: Option<i32>,
    pub members: Vec<(CellId, Offset)>,
}

pub struct Packer {
    pub nl: Netlist,
    pub autoidx: usize,
    /// Cluster root -> its cluster.
    pub clusters: BTreeMap<CellId, Cluster>,
    /// Cell -> the root of the cluster it is in (roots map to themselves).
    pub cluster_of: BTreeMap<CellId, CellId>,
    /// Counts per pass, for the report.
    pub stats: BTreeMap<String, usize>,
    /// I/O cells (buffers, delays) -> the IOB site they belong to.
    pub io_site: BTreeMap<CellId, String>,
}

impl Packer {
    pub fn new(nl: Netlist) -> Packer {
        Packer {
            nl,
            autoidx: 0,
            clusters: BTreeMap::new(),
            cluster_of: BTreeMap::new(),
            stats: BTreeMap::new(),
            io_site: BTreeMap::new(),
        }
    }

    pub fn count(&mut self, k: &str, n: usize) {
        *self.stats.entry(k.to_string()).or_default() += n;
    }

    /// Apply `rule` to `c` (nextpnr's `xform_cell`).
    pub fn xform(&mut self, c: CellId, rule: &Rule) {
        let old_ty = self.nl.cell(c).ty.clone();
        {
            let cell = self.nl.cell_mut(c);
            cell.attrs.insert("X_ORIG_TYPE".into(), json!(old_ty));
            cell.ty = rule.new_type.clone();
        }
        let ports: Vec<String> = self.nl.cell(c).ports.keys().cloned().collect();
        for p in ports {
            if let Some(targets) = rule.multi.get(&p) {
                let old = self.nl.cell(c).ports[&p].clone();
                self.nl.disconnect(c, &p);
                self.nl.cell_mut(c).ports.remove(&p);
                for t in targets {
                    self.nl.add_port(c, t, old.dir);
                    if let Some(n) = old.net {
                        self.nl.connect(c, t, n);
                    }
                    self.nl.cell_mut(c).attrs.insert(format!("X_ORIG_PORT_{t}"), json!(p));
                }
            } else {
                let new = rule
                    .port
                    .get(&p)
                    .cloned()
                    .unwrap_or_else(|| p.chars().filter(|&ch| ch != '[' && ch != ']').collect());
                self.nl.rename_port(c, &p, &new);
                self.nl.cell_mut(c).attrs.insert(format!("X_ORIG_PORT_{new}"), json!(p));
            }
        }
        let cell = self.nl.cell_mut(c);
        for (from, to) in &rule.param {
            if let Some(v) = cell.params.get(from).cloned() {
                cell.params.insert(to.clone(), v);
            }
        }
        for (k, v) in &rule.set_attrs {
            cell.attrs.insert(k.clone(), v.clone());
        }
        for (k, v) in &rule.set_params {
            cell.params.insert(k.clone(), v.clone());
        }
    }

    /// Apply the rule for each cell whose type has one.
    pub fn generic_xform(&mut self, rules: &BTreeMap<String, Rule>) -> usize {
        let mut n = 0;
        for c in self.nl.cell_ids() {
            if let Some(r) = rules.get(&self.nl.cell(c).ty).cloned() {
                self.xform(c, &r);
                n += 1;
            }
        }
        n
    }

    /// A `LUT<k>` cell named `name` reading `inputs`, driving `output`.
    pub fn create_lut(&mut self, name: &str, inputs: &[NetId], output: Option<NetId>, init: Value) -> CellId {
        let c = self.nl.add_cell(name, &format!("LUT{}", inputs.len()));
        for (i, &n) in inputs.iter().enumerate() {
            let p = format!("I{i}");
            self.nl.add_port(c, &p, Dir::Input);
            self.nl.connect(c, &p, n);
        }
        self.nl.add_port(c, "O", Dir::Output);
        if let Some(o) = output {
            self.nl.connect(c, "O", o);
        }
        self.nl.cell_mut(c).params.insert("INIT".into(), init);
        c
    }

    /// Put a pass-through LUT1 between `net` and the given users
    /// (nextpnr's `feed_through_lut`): `{net}$LUT{n}` driving `{net}$legal{n}`.
    pub fn feed_through_lut(&mut self, net: NetId, users: &[(CellId, String)]) -> CellId {
        let base = self.nl.net(net).name.clone();
        self.autoidx += 1;
        let feed = self.nl.add_net_unique(&format!("{base}$legal{}", self.autoidx));
        self.autoidx += 1;
        let name = format!("{base}$LUT{}", self.autoidx);
        let lut = self.create_lut(&name, &[net], Some(feed), constants::prop32(2));
        for (c, p) in users {
            self.nl.connect(*c, p, feed);
        }
        lut
    }

    /// Tie `port` of `c` to a constant (nextpnr's `tie_port`): with `inv`, a
    /// zero becomes VCC and `IS_<port>_INVERTED`.
    pub fn tie_port(&mut self, c: CellId, port: &str, value: bool, inv: bool) {
        self.nl.add_port(c, port, Dir::Input);
        let n = if value || inv { self.nl.vcc } else { self.nl.gnd };
        self.nl.connect(c, port, n);
        if !value && inv {
            self.nl.cell_mut(c).params.insert(format!("IS_{port}_INVERTED"), json!(1));
        }
    }

    /// Make `root` a cluster root (absolute slot `abs_z` if given).
    pub fn make_root(&mut self, root: CellId, abs_z: Option<i32>) {
        let cl = self.clusters.entry(root).or_default();
        if abs_z.is_some() {
            cl.root_abs_z = abs_z;
        }
        self.cluster_of.insert(root, root);
    }

    /// Add `c` to `root`'s cluster at `off`.
    pub fn add_member(&mut self, root: CellId, c: CellId, off: Offset) {
        self.make_root(root, None);
        self.clusters.get_mut(&root).expect("root").members.push((c, off));
        self.cluster_of.insert(c, root);
    }

    /// In some cluster, as root or member.
    pub fn is_constrained(&self, c: CellId) -> bool {
        self.cluster_of.contains_key(&c)
    }
}

/// Run the passes implemented so far, in nextpnr's order. Not yet: other
/// IOLOGIC (IDDR, SERDES, ODELAY), PLL/MMCM, user MUXF trees, SRL cascades.
pub fn pack(nl: Netlist, dev: &crate::device::Device, props: &crate::xdc::PortProps) -> Result<Packer, String> {
    let mut p = Packer::new(nl);
    constants::pack_constants(&mut p);
    luts::pack_inverters(&mut p);
    io::pack_io(&mut p, dev, props)?;
    clocking::prepare_clocking(&mut p);
    constants::pack_constants_with(&mut p, false);
    io::pack_iologic(&mut p, dev)?;
    io::pack_idelayctrl(&mut p, dev)?;
    clocking::pack_gbs(&mut p);
    carry::pack_carries(&mut p);
    srl::pack_srls(&mut p);
    luts::pack_luts(&mut p);
    dram::pack_dram(&mut p);
    bram::pack_bram(&mut p);
    dsp::pack_dsps(&mut p);
    ffs::pack_ffs(&mut p);
    muxf::finalise_muxfs(&mut p);
    ffs::pack_lutffs(&mut p);
    p.nl.sweep_nets();
    Ok(p)
}
