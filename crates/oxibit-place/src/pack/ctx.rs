//! The read-only context slice packing scores against, and the score.
//!
//! Slice packing works on atoms: the units the passes before it already
//! fixed and that fit one slice letter. A free LUT, a free flop, or a LUT
//! with the flop `pack_lutffs` paired it with. Carry chains, wide-mux trees,
//! distributed RAM and shift registers are macros with their own relative
//! placement and are not atoms.
//!
//! `merging_score` is the best-choice clustering score (Alpert, Kahng, Nam,
//! Reda, Villarrubia, "A semi-persistent clustering technique for VLSI
//! circuit placement", ISPD 2005): the weight of the nets two objects
//! share, each net weighing `1 / pins`, over the area of the merged object.
//! The physical term is ours: a linear falloff with distance up to a
//! threshold, beyond which a pair is not a candidate at all. The constants
//! in `ScoreParams::default` are starting values, not tuned ones.
//!
//! The score ranks; it never decides legality. That stays with the
//! capacity table and `ffs::Ctrl` (`slices::fits`).

use std::cmp::Reverse;
use std::collections::{BTreeMap, HashMap, HashSet};

use super::ffs::{Ctrl, ctrl};
use super::{Packer, slot};
use crate::netlist::{CellId, NetId, Netlist};

pub type AtomId = u32;

/// One letter's worth of logic, moved as a unit.
#[derive(Debug, Clone)]
pub struct Atom {
    /// A LUT atom's first cell is the LUT on the letter's 6LUT.
    pub cells: Vec<CellId>,
    /// 6LUT slots it needs: 1 if it holds any LUT, 2 for the lone leaf of
    /// an F7 mux whose other input is tied to ground (that letter's LUT
    /// has to stay empty).
    pub luts: u8,
    /// FF slots it needs (the letter's FF, not its 5FF): 1 for a lone flop
    /// or the flop paired with the LUT on the 6LUT.
    pub ffs: u8,
    /// The second LUT of a fractured letter, on the 5LUT. Its own flop, if
    /// it has one, rides on the 5FF.
    pub five: Option<CellId>,
    /// FF slots of its slice that it puts out of reach of bypass-fed flops:
    /// a wide mux takes its select on a bypass pin (AX, CX for F7; BX for
    /// F8), so the FF of that letter is lost unless the leaf LUT there
    /// has a flop of its own. Counts against the slice's FF slots.
    pub blocks_ff: u8,
    /// The control set its flop needs, if it has one.
    pub ctrl: Option<Ctrl>,
}

/// What one slice holds, in atoms' terms: a LUT and a flop per letter. A
/// flop beside a LUT that does not drive it takes the bypass pin, so the
/// two counts are independent. LUT fracturing (two LUTs in one letter) is
/// not modelled yet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SliceCap {
    pub luts: u8,
    pub ffs: u8,
}

impl Default for SliceCap {
    fn default() -> SliceCap {
        SliceCap { luts: 4, ffs: 4 }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct ScoreParams {
    /// Nets with more pins than this carry no connectivity weight.
    pub max_fanout: u32,
    /// Weight of the physical term at distance zero, in units of one
    /// two-pin net's worth of sharing (0.5).
    pub phys_weight: f64,
    /// Neighbor-distance threshold, in position units (Manhattan).
    pub neighbor_dist: f64,
    /// Put two LUTs in one letter (6LUT + 5LUT) when they share at least
    /// this many inputs; `None` for one LUT a letter. Ours: 1, the least
    /// sharing that saves an input pin. Tuned by us only in that sense: it
    /// has not been swept (the one design measured fractures 261 letters
    /// at 1; nextpnr's placement of it has 714).
    pub fracture: Option<u32>,
}

impl Default for ScoreParams {
    fn default() -> ScoreParams {
        ScoreParams {
            max_fanout: 64,
            phys_weight: 0.1,
            neighbor_dist: 4.0,
            fracture: Some(1),
        }
    }
}

/// A net that connects at least two atoms and carries weight.
#[derive(Debug, Clone)]
pub struct ScoredNet {
    pub id: NetId,
    pub pins: u32,
    pub weight: f64,
    /// The atoms on it, ascending, each once.
    pub atoms: Vec<AtomId>,
    /// Has pins on cells that are not atoms: no packing absorbs it.
    pub open: bool,
}

pub struct PackCtx<'a> {
    pub nl: &'a Netlist,
    pub atoms: Vec<Atom>,
    pub atom_of: HashMap<CellId, AtomId>,
    pub nets: Vec<ScoredNet>,
    /// Per atom, indices into `nets`, ascending.
    pub atom_nets: Vec<Vec<u32>>,
    /// Pairs of atoms that must share a slice: the two leaf LUTs of an F7
    /// mux, which the mux reaches only inside its own slice. Lower atom
    /// first.
    pub ties: Vec<(AtomId, AtomId)>,
    /// Per atom position, from a coarse spread or an earlier placement.
    pub pos: Option<Vec<(f64, f64)>>,
    pub cap: SliceCap,
    pub params: ScoreParams,
}

fn free(p: &Packer, c: CellId) -> bool {
    let cell = p.nl.cell(c);
    cell.bel.is_none() && !cell.attrs.contains_key("BEL")
}

fn plain_lut(p: &Packer, c: CellId) -> bool {
    let cell = p.nl.cell(c);
    cell.ty == "SLICE_LUTX"
        && !cell.attrs.contains_key("X_LUT_AS_DRAM")
        && !cell.attrs.contains_key("X_LUT_AS_SRL")
        && cell.net("CLK").is_none()
}

/// The atom `c` starts, if it starts one: a cluster is entered at its root.
fn atom_at(p: &Packer, c: CellId) -> Option<Atom> {
    let ty = p.nl.cell(c).ty.as_str();
    match p.cluster_of.get(&c) {
        None if plain_lut(p, c) && free(p, c) => Some(Atom {
            cells: vec![c],
            luts: 1,
            ffs: 0,
            five: None,
            blocks_ff: 0,
            ctrl: None,
        }),
        None if ty == "SLICE_FFX" && free(p, c) => Some(Atom {
            cells: vec![c],
            luts: 0,
            ffs: 1,
            five: None,
            blocks_ff: 0,
            ctrl: Some(ctrl(p, c)),
        }),
        Some(&root) if root == c => {
            // Only the LUT + its flop shape; anything else is a macro.
            let cl = &p.clusters[&root];
            let [(ff, off)] = cl.members[..] else { return None };
            let pair = cl.root_abs_z.is_none()
                && plain_lut(p, root)
                && free(p, root)
                && p.nl.cell(ff).ty == "SLICE_FFX"
                && free(p, ff)
                && (off.dx, off.dy, off.abs) == (0, 0, false)
                && off.z == slot::FF - slot::LUT6;
            pair.then(|| Atom {
                cells: vec![root, ff],
                luts: 1,
                ffs: 1,
                five: None,
                blocks_ff: 0,
                ctrl: Some(ctrl(p, ff)),
            })
        }
        _ => None,
    }
}

/// The input nets of a LUT (pins A1..A6), each once.
fn lut_inputs(p: &Packer, lut: CellId) -> Vec<NetId> {
    let cell = p.nl.cell(lut);
    let mut v: Vec<NetId> = (1..=6).filter_map(|k| cell.net(&format!("A{k}"))).collect();
    v.sort_unstable();
    v.dedup();
    v
}

/// Fracture letters: pair up LUT atoms that can share one, most shared
/// inputs first. Then, of those left, neighbours whose inputs are disjoint
/// but fit five pins together (a LUT2 and a LUT3): one drives the other, or
/// both drive one cell. Neighbours only, so a letter holds related logic.
///
/// A letter's 6LUT and 5LUT share pins A1..A5 (A6 is tied high), so the
/// two need five distinct inputs or fewer between them. The 5LUT leaves
/// the slice on the letter's MUX output, which its 5FF also uses: so the
/// LUT that goes low is one with no flop, or one whose flop is its only
/// user (then nothing but the 5FF's output leaves). Flops of both must
/// agree on the control set. The leaves of an F7 mux are left whole, and
/// so is any LUT with an input `place::legalise` could not lay out again
/// (no `X_ORIG_PORT` marker).
fn fracture(p: &Packer, atoms: Vec<Atom>, min_shared: u32) -> Vec<Atom> {
    /// Nets on more candidate LUTs than this pair nothing.
    const FANOUT: usize = 64;
    let nl = &p.nl;
    let mut leaves: HashSet<CellId> = HashSet::new();
    for m in nl.cells_of("SELMUX2_1") {
        for port in ["0", "1"] {
            leaves.extend(nl.cell(m).net(port).and_then(|n| nl.driver(n)).map(|(d, _)| d));
        }
    }
    // Per atom, its LUT's inputs if the LUT may share a letter.
    let inputs: Vec<Option<Vec<NetId>>> = atoms
        .iter()
        .map(|a| {
            let lut = *a.cells.first().filter(|_| a.luts == 1)?;
            let cell = nl.cell(lut);
            let marked = (1..=6).all(|k| cell.net(&format!("A{k}")).is_none() || cell.attrs.contains_key(&format!("X_ORIG_PORT_A{k}")));
            let ins = lut_inputs(p, lut);
            (marked && ins.len() <= 5 && !leaves.contains(&lut)).then_some(ins)
        })
        .collect();
    let may_go_low = |a: &Atom| a.ffs == 0 || nl.cell(a.cells[0]).net("O6").is_some_and(|n| nl.net(n).users.len() == 1);

    let mut on: BTreeMap<NetId, Vec<u32>> = BTreeMap::new();
    for (i, ins) in inputs.iter().enumerate() {
        for &n in ins.iter().flatten() {
            on.entry(n).or_default().push(i as u32);
        }
    }
    let mut shared: BTreeMap<(u32, u32), u32> = BTreeMap::new();
    for v in on.values().filter(|v| v.len() <= FANOUT) {
        for (k, &a) in v.iter().enumerate() {
            for &b in &v[k + 1..] {
                *shared.entry((a, b)).or_default() += 1;
            }
        }
    }
    let width = |i: u32| inputs[i as usize].as_ref().map_or(0, Vec::len) as u32;
    let mut pairs: Vec<(u32, u32, u32, u32)> = shared
        .into_iter()
        .filter_map(|((a, b), s)| {
            let (x, y) = (&atoms[a as usize], &atoms[b as usize]);
            let union = width(a) + width(b) - s;
            let ctrl_ok = match (&x.ctrl, &y.ctrl) {
                (Some(c), Some(d)) => c == d,
                _ => true,
            };
            (s >= min_shared && union <= 5 && ctrl_ok && (may_go_low(x) || may_go_low(y))).then_some((s, union, a, b))
        })
        .collect();
    pairs.sort_by_key(|&(s, union, a, b)| (Reverse(s), union, a, b));
    let mut mate: Vec<Option<u32>> = vec![None; atoms.len()];
    for (_, _, a, b) in pairs {
        if mate[a as usize].is_none() && mate[b as usize].is_none() {
            mate[a as usize] = Some(b);
            mate[b as usize] = Some(a);
        }
    }

    // Neighbours with disjoint inputs, smallest union first.
    let lut_of: HashMap<CellId, u32> = (0..atoms.len() as u32).filter(|&i| inputs[i as usize].is_some()).map(|i| (atoms[i as usize].cells[0], i)).collect();
    let mut near: Vec<(u32, u32, u32)> = Vec::new();
    let pairable = |a: u32, b: u32, mate: &[Option<u32>]| {
        let (x, y) = (&atoms[a as usize], &atoms[b as usize]);
        let ctrl_ok = match (&x.ctrl, &y.ctrl) {
            (Some(c), Some(d)) => c == d,
            _ => true,
        };
        a != b && mate[a as usize].is_none() && mate[b as usize].is_none() && width(a) + width(b) <= 5 && ctrl_ok && (may_go_low(x) || may_go_low(y))
    };
    for (&lut, &a) in &lut_of {
        if mate[a as usize].is_some() || width(a) > 4 {
            continue;
        }
        let Some(o) = nl.cell(lut).net("O6") else { continue };
        let users = &nl.net(o).users;
        if users.len() > FANOUT {
            continue;
        }
        for (u, _) in users {
            // `a` drives LUT `u`.
            if let Some(&b) = lut_of.get(u)
                && pairable(a, b, &mate)
            {
                near.push((width(a) + width(b), a.min(b), a.max(b)));
            }
            // `a` and LUT `b` both drive cell `u`.
            for port in nl.cell(*u).ports.values() {
                let Some(n) = port.net else { continue };
                if n == o {
                    continue;
                }
                if let Some((d, _)) = nl.driver(n)
                    && let Some(&b) = lut_of.get(&d)
                    && pairable(a, b, &mate)
                {
                    near.push((width(a) + width(b), a.min(b), a.max(b)));
                }
            }
        }
    }
    near.sort_unstable();
    near.dedup();
    for (_, a, b) in near {
        if pairable(a, b, &mate) {
            mate[a as usize] = Some(b);
            mate[b as usize] = Some(a);
        }
    }

    // A fractured atom stands where the lower of its two stood.
    let low: Vec<bool> = atoms.iter().map(may_go_low).collect();
    let mut atoms: Vec<Option<Atom>> = atoms.into_iter().map(Some).collect();
    let mut out: Vec<Atom> = Vec::new();
    for i in 0..atoms.len() {
        let Some(a) = atoms[i].take() else { continue };
        let Some(j) = mate[i] else {
            out.push(a);
            continue;
        };
        let b = atoms[j as usize].take().expect("a mate is taken once");
        // Low goes a LUT without a flop if there is one.
        let b_low = b.ffs == 0 || (a.ffs != 0 && low[j as usize]);
        let (six, five) = if b_low { (a, b) } else { (b, a) };
        out.push(Atom {
            luts: 1,
            ffs: six.ffs,
            five: Some(five.cells[0]),
            blocks_ff: 0,
            ctrl: six.ctrl.or(five.ctrl),
            cells: six.cells.into_iter().chain(five.cells).collect(),
        });
    }
    out
}

impl<'a> PackCtx<'a> {
    pub fn new(p: &'a Packer, params: ScoreParams) -> PackCtx<'a> {
        let nl = &p.nl;
        let mut atoms: Vec<Atom> = Vec::new();
        let mut atom_of: HashMap<CellId, AtomId> = HashMap::new();
        for c in nl.cell_ids() {
            if atom_of.contains_key(&c) {
                continue;
            }
            if let Some(a) = atom_at(p, c) {
                for &m in &a.cells {
                    atom_of.insert(m, atoms.len() as AtomId);
                }
                atoms.push(a);
            }
        }

        if let Some(min_shared) = params.fracture {
            atoms = fracture(p, atoms, min_shared);
            atom_of.clear();
            for (i, a) in atoms.iter().enumerate() {
                for &c in &a.cells {
                    atom_of.insert(c, i as AtomId);
                }
            }
        }

        // Net -> the atoms on it, one entry per pin, ascending by atom.
        let mut on: BTreeMap<NetId, Vec<AtomId>> = BTreeMap::new();
        for (i, a) in atoms.iter().enumerate() {
            for &c in &a.cells {
                for port in nl.cell(c).ports.values() {
                    if let Some(n) = port.net
                        && !nl.is_const(n)
                    {
                        on.entry(n).or_default().push(i as AtomId);
                    }
                }
            }
        }
        let mut nets: Vec<ScoredNet> = Vec::new();
        let mut atom_nets: Vec<Vec<u32>> = vec![Vec::new(); atoms.len()];
        for (id, mut v) in on {
            let atom_pins = v.len();
            v.dedup();
            if v.len() < 2 {
                continue;
            }
            let net = nl.net(id);
            let pins = net.users.len() + usize::from(net.driver.is_some());
            let clock = net.users.iter().any(|(_, port)| port == "CK" || port == "CLK");
            if clock || pins as u32 > params.max_fanout {
                continue;
            }
            let idx = nets.len() as u32;
            for &a in &v {
                atom_nets[a as usize].push(idx);
            }
            nets.push(ScoredNet {
                id,
                pins: pins as u32,
                weight: 1.0 / pins as f64,
                atoms: v,
                open: atom_pins < pins,
            });
        }
        let mut ties: Vec<(AtomId, AtomId)> = Vec::new();
        for m in nl.cells_of("SELMUX2_1") {
            let leaf = |port: &str| nl.cell(m).net(port).and_then(|n| nl.driver(n)).and_then(|(d, _)| atom_of.get(&d).copied());
            if let (Some(a), Some(b)) = (leaf("0"), leaf("1"))
                && a != b
            {
                ties.push((a.min(b), a.max(b)));
            }
            if let Some(a) = leaf("1")
                && atoms[a as usize].ffs == 0
            {
                atoms[a as usize].blocks_ff += 1;
            }
            // An input tied to ground needs no LUT: an unused 6LUT reads 0.
            // The letter must stay empty for that, so the other leaf's atom
            // answers for two LUT slots; and when the empty letter is the
            // lower one, its FF is behind the mux select all the same.
            for (tied, other) in [("1", "0"), ("0", "1")] {
                if nl.cell(m).net(tied) == Some(nl.gnd)
                    && let Some(a) = leaf(other)
                {
                    atoms[a as usize].luts = (atoms[a as usize].luts + 1).min(4);
                    if tied == "1" {
                        atoms[a as usize].blocks_ff += 1;
                    }
                }
            }
            // An F8 mux reads two F7 muxes of its own slice: tying one leaf
            // of each ties all four. Its select is BX, so the leaf on
            // letter B (the `0` leaf of the F7 on its `1` input) loses its
            // FF to bypass flops as the leaves on A and C do.
            let f7 = |port: &str| nl.cell(m).net(port).and_then(|n| nl.driver(n)).map(|(d, _)| d).filter(|&d| nl.cell(d).ty == "SELMUX2_1");
            if let (Some(hi), Some(lo)) = (f7("1"), f7("0")) {
                let leaf_of = |mux: CellId, port: &str| nl.cell(mux).net(port).and_then(|n| nl.driver(n)).and_then(|(d, _)| atom_of.get(&d).copied());
                let any_leaf = |mux: CellId| leaf_of(mux, "1").or(leaf_of(mux, "0"));
                if let (Some(a), Some(b)) = (any_leaf(hi), any_leaf(lo))
                    && a != b
                {
                    ties.push((a.min(b), a.max(b)));
                }
                if let Some(b) = leaf_of(hi, "0")
                    && atoms[b as usize].ffs == 0
                {
                    atoms[b as usize].blocks_ff += 1;
                }
                // An F7 with both inputs grounded has no leaf to place it:
                // it takes its half of the slice empty. The other F7's leaf
                // answers for those two LUT slots, and for the FFs behind
                // the selects there (AX and BX on the upper half, CX on the
                // lower).
                let empty = |mux: CellId| ["0", "1"].iter().all(|port| nl.cell(mux).net(port) == Some(nl.gnd));
                for (bare, leafy, lost) in [(hi, lo, 2), (lo, hi, 1)] {
                    if empty(bare)
                        && let Some(a) = any_leaf(leafy)
                    {
                        atoms[a as usize].luts = (atoms[a as usize].luts + 2).min(4);
                        atoms[a as usize].blocks_ff += lost;
                    }
                }
            }
        }
        PackCtx {
            nl,
            atoms,
            ties,
            atom_of,
            nets,
            atom_nets,
            pos: None,
            cap: SliceCap::default(),
            params,
        }
    }

    /// With a position per atom (`atoms` order).
    pub fn with_positions(mut self, pos: Vec<(f64, f64)>) -> PackCtx<'a> {
        assert_eq!(pos.len(), self.atoms.len(), "one position per atom");
        self.pos = Some(pos);
        self
    }

    /// Weight of the scored nets both sorted net lists hold.
    pub fn shared_weight(&self, a: &[u32], b: &[u32]) -> f64 {
        let (mut i, mut j, mut w) = (0, 0, 0.0);
        while i < a.len() && j < b.len() {
            match a[i].cmp(&b[j]) {
                std::cmp::Ordering::Less => i += 1,
                std::cmp::Ordering::Greater => j += 1,
                std::cmp::Ordering::Equal => {
                    w += self.nets[a[i] as usize].weight;
                    i += 1;
                    j += 1;
                }
            }
        }
        w
    }
}

pub fn manhattan(a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - b.0).abs() + (a.1 - b.1).abs()
}

/// The score from its parts: shared net weight, merged area in letters, and
/// the distance between the two if positions are known. Negative infinity
/// for a pair further apart than the threshold.
pub fn score_of(shared: f64, area: u32, dist: Option<f64>, p: &ScoreParams) -> f64 {
    let phys = match dist {
        None => 0.0,
        Some(d) if d > p.neighbor_dist => return f64::NEG_INFINITY,
        Some(d) => 0.5 * p.phys_weight * (1.0 - d / p.neighbor_dist),
    };
    (shared + phys) / f64::from(area.max(1))
}

/// How good it is to put atoms `a` and `b` in one slice. Pure; legality is
/// not its business (`slices::fits`).
pub fn merging_score(a: AtomId, b: AtomId, ctx: &PackCtx) -> f64 {
    if a == b {
        return f64::NEG_INFINITY;
    }
    let (x, y) = (&ctx.atoms[a as usize], &ctx.atoms[b as usize]);
    let shared = ctx.shared_weight(&ctx.atom_nets[a as usize], &ctx.atom_nets[b as usize]);
    let area = u32::from(x.luts.max(x.ffs)) + u32::from(y.luts.max(y.ffs));
    let dist = ctx.pos.as_ref().map(|p| manhattan(p[a as usize], p[b as usize]));
    score_of(shared, area, dist, &ctx.params)
}

#[cfg(test)]
mod tests {
    use serde_json::json;

    use super::*;
    use crate::netlist::Dir;
    use crate::pack::Offset;

    /// A LUT as synthesis leaves one: inputs marked with their origin.
    fn lut(p: &mut Packer, name: &str, ins: &[NetId], out: NetId) -> CellId {
        let c = p.nl.add_cell(name, "SLICE_LUTX");
        for (i, &n) in ins.iter().enumerate() {
            p.nl.connect(c, &format!("A{}", i + 1), n);
            p.nl.cell_mut(c).attrs.insert(format!("X_ORIG_PORT_A{}", i + 1), json!(format!("I{i}")));
        }
        p.nl.add_port(c, "O6", Dir::Output);
        p.nl.connect(c, "O6", out);
        c
    }

    /// A flop on `d`, paired with the LUT driving it as `pack_lutffs` does.
    fn flop_of(p: &mut Packer, name: &str, lut: CellId, d: NetId, clk: NetId) -> CellId {
        let c = p.nl.add_cell(name, "SLICE_FFX");
        p.nl.connect(c, "D", d);
        p.nl.connect(c, "CK", clk);
        p.add_member(
            lut,
            c,
            Offset {
                dx: 0,
                dy: 0,
                z: slot::FF - slot::LUT6,
                abs: false,
            },
        );
        c
    }

    fn nets(p: &mut Packer, names: &[&str]) -> Vec<NetId> {
        names.iter().map(|n| p.nl.add_net(n)).collect()
    }

    #[test]
    fn luts_sharing_inputs_fracture_within_five_pins() {
        let mut p = Packer::new(Netlist::new("t"));
        let n = nets(&mut p, &["a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "o0", "o1", "o2", "o3"]);
        // Two inputs shared, four between them: one letter.
        let l0 = lut(&mut p, "l0", &[n[0], n[1], n[2]], n[10]);
        let l1 = lut(&mut p, "l1", &[n[1], n[2], n[3]], n[11]);
        // One input shared, but six between them: two letters.
        let l2 = lut(&mut p, "l2", &[n[4], n[5], n[6], n[7], n[8]], n[12]);
        let l3 = lut(&mut p, "l3", &[n[4], n[9]], n[13]);

        let ctx = PackCtx::new(&p, ScoreParams::default());
        assert_eq!(ctx.atoms.len(), 3);
        let a = &ctx.atoms[ctx.atom_of[&l0] as usize];
        assert_eq!((a.cells.clone(), a.five, a.luts, a.ffs), (vec![l0, l1], Some(l1), 1, 0));
        assert_eq!(ctx.atom_of[&l1], ctx.atom_of[&l0]);
        assert_ne!(ctx.atom_of[&l2], ctx.atom_of[&l3]);

        let whole = PackCtx::new(
            &p,
            ScoreParams {
                fracture: None,
                ..ScoreParams::default()
            },
        );
        assert_eq!(whole.atoms.len(), 4);
        assert!(whole.atoms.iter().all(|a| a.five.is_none()));
    }

    #[test]
    fn only_a_lut_whose_flop_is_its_sole_user_goes_low_with_a_flop() {
        let mut p = Packer::new(Netlist::new("t"));
        let n = nets(&mut p, &["a", "b", "clk", "o0", "o1", "o9"]);
        // l0's output also feeds l9, so it cannot leave by a 5FF alone.
        let l0 = lut(&mut p, "l0", &[n[0], n[1]], n[3]);
        let f0 = flop_of(&mut p, "f0", l0, n[3], n[2]);
        let l1 = lut(&mut p, "l1", &[n[0], n[1]], n[4]);
        let f1 = flop_of(&mut p, "f1", l1, n[4], n[2]);
        let l9 = lut(&mut p, "l9", &[n[3]], n[5]);

        let ctx = PackCtx::new(&p, ScoreParams::default());
        let a = &ctx.atoms[ctx.atom_of[&l0] as usize];
        // l0 keeps the 6LUT and the FF; l1 and its flop ride below.
        assert_eq!((a.cells.clone(), a.five, a.luts, a.ffs), (vec![l0, f0, l1, f1], Some(l1), 1, 1));
        assert_ne!(ctx.atom_of[&l9], ctx.atom_of[&l0]);

        // Give l1's output a second user too: neither may go low.
        let extra = p.nl.add_net("o8");
        lut(&mut p, "l8", &[n[4]], extra);
        let ctx = PackCtx::new(&p, ScoreParams::default());
        assert_ne!(ctx.atom_of[&l0], ctx.atom_of[&l1]);
    }
}
