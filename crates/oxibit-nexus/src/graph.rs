//! Routing graph of a Nexus device, built from the bit database.
//!
//! Every tile's pips and fixed connections name wires relative to the
//! tile. A wire becomes a node by resolving its prefix (as prjoxide's
//! `Neighbour::parse_wire` and `LocationGrid::neighbour_tile` do):
//! - none: this location;
//! - `N<n>`/`S<n>`/`E<n>`/`W<n>` combinations: a relative location (a
//!   location off the grid leaves the wire local, prefix kept);
//! - `G:` global (location 0,0); `G:VCC` is the constant-one source;
//! - `BRANCH:` the branch column feeding this column; `BRANCH_L:`/`BRANCH_R:`
//!   the branch two columns to that side; `SPINE:` and `HROW:` the global
//!   spine and row origins; `DQSG:` the DQS group column of this column.
//!
//! Nodes are (x, y, name) after resolution; wires that resolve to the same
//! triple are one node. Pips keep their tile and tile-local wire names for
//! FASM (`<tile>.PIP.<to>.<from>`, `:` written `__`). Fixed connections
//! are arcs with no configuration.

use std::collections::HashMap;

use crate::bels::{Bel, tile_bels};
use crate::chip::Chip;
use crate::{Result, err};

pub type NodeId = u32;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Node {
    pub x: i32,
    pub y: i32,
    pub name: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ArcKind {
    /// Configurable pip, written to FASM.
    Pip,
    /// Hard wired; costs nothing in the bitstream.
    Fixed,
}

#[derive(Clone, Copy, Debug)]
pub struct Arc {
    pub from: NodeId,
    pub to: NodeId,
    pub kind: ArcKind,
    /// Index into `chip.tiles`.
    pub tile: u32,
    /// Tile-local names (interned), as the FASM pip feature spells them.
    pub to_wire: u32,
    pub from_wire: u32,
}

/// A bel placed in the grid, with its pins resolved to nodes.
#[derive(Clone, Debug)]
pub struct PlacedBel {
    pub bel: Bel,
    /// Index into `chip.tiles` of the tile the bel was found in.
    pub tile: u32,
    /// Nominal location: tile location plus the bel offset.
    pub x: i32,
    pub y: i32,
    pub pin_nodes: Vec<Option<NodeId>>,
}

impl PlacedBel {
    /// `R<y>C<x>_<name>`, the bel's tilegroup and FASM prefix.
    pub fn group(&self) -> String {
        format!("R{}C{}_{}", self.y, self.x, self.bel.name)
    }
    pub fn pin(&self, name: &str) -> Option<NodeId> {
        let i = self.bel.pins.iter().position(|p| p.name == name)?;
        self.pin_nodes[i]
    }
}

pub struct Graph {
    pub width: i32,
    pub height: i32,
    names: Vec<String>,
    name_ix: HashMap<String, u32>,
    pub nodes: Vec<Node>,
    node_ix: HashMap<Node, NodeId>,
    pub arcs: Vec<Arc>,
    /// CSR: arcs leaving node n are `arcs[down_idx[down_off[n]..down_off[n+1]]]`.
    down_off: Vec<u32>,
    down_idx: Vec<u32>,
    up_off: Vec<u32>,
    up_idx: Vec<u32>,
    pub vcc: NodeId,
    pub bels: Vec<PlacedBel>,
    bel_ix: HashMap<String, usize>,
    /// Electrical node of each node: nodes joined by fixed connections
    /// are one wire, so they are occupied and reserved together.
    pub group: Vec<NodeId>,
}

/// How a tile-local wire name resolves.
enum Resolved {
    At(i32, i32, String),
    Vcc,
}

impl Graph {
    fn intern(&mut self, s: &str) -> u32 {
        if let Some(&i) = self.name_ix.get(s) {
            return i;
        }
        let i = self.names.len() as u32;
        self.names.push(s.to_string());
        self.name_ix.insert(s.to_string(), i);
        i
    }

    pub fn name(&self, id: u32) -> &str {
        &self.names[id as usize]
    }

    fn node(&mut self, x: i32, y: i32, name: &str) -> NodeId {
        let name = self.intern(name);
        let n = Node { x, y, name };
        if let Some(&i) = self.node_ix.get(&n) {
            return i;
        }
        let i = self.nodes.len() as NodeId;
        self.nodes.push(n);
        self.node_ix.insert(n, i);
        i
    }

    /// The node of a wire as named in a tile at (x, y), if it exists.
    pub fn find(&self, chip: &Chip, x: i32, y: i32, wire: &str) -> Option<NodeId> {
        match resolve(chip, x, y, wire, self.width, self.height) {
            Resolved::Vcc => Some(self.vcc),
            Resolved::At(nx, ny, n) => {
                let name = *self.name_ix.get(&n)?;
                self.node_ix.get(&Node { x: nx, y: ny, name }).copied()
            }
        }
    }

    pub fn node_name(&self, n: NodeId) -> String {
        let nd = self.nodes[n as usize];
        format!("R{}C{}_{}", nd.y, nd.x, self.name(nd.name))
    }

    pub fn downhill(&self, n: NodeId) -> impl Iterator<Item = &Arc> + '_ {
        let (a, b) = (self.down_off[n as usize] as usize, self.down_off[n as usize + 1] as usize);
        self.down_idx[a..b].iter().map(|&i| &self.arcs[i as usize])
    }

    pub fn uphill(&self, n: NodeId) -> impl Iterator<Item = &Arc> + '_ {
        let (a, b) = (self.up_off[n as usize] as usize, self.up_off[n as usize + 1] as usize);
        self.up_idx[a..b].iter().map(|&i| &self.arcs[i as usize])
    }

    pub fn bel(&self, group: &str) -> Option<&PlacedBel> {
        self.bel_ix.get(group).map(|&i| &self.bels[i])
    }

    pub fn bel_index(&self, group: &str) -> Option<usize> {
        self.bel_ix.get(group).copied()
    }

    /// FASM line for a pip arc.
    pub fn fasm_pip(&self, chip: &Chip, a: &Arc) -> String {
        let t = &chip.tiles[a.tile as usize].name;
        format!(
            "{}.PIP.{}.{}",
            t.replace(':', "__"),
            self.name(a.to_wire).replace(':', "__"),
            self.name(a.from_wire).replace(':', "__")
        )
    }

    pub fn build(chip: &Chip) -> Result<Graph> {
        let db = chip.db;
        let width = db.data.max_col as i32 + 1;
        let height = db.data.max_row as i32 + 1;
        let mut g = Graph {
            width,
            height,
            names: Vec::new(),
            name_ix: HashMap::new(),
            nodes: Vec::new(),
            node_ix: HashMap::new(),
            arcs: Vec::new(),
            down_off: Vec::new(),
            down_idx: Vec::new(),
            up_off: Vec::new(),
            up_idx: Vec::new(),
            vcc: 0,
            bels: Vec::new(),
            bel_ix: HashMap::new(),
            group: Vec::new(),
        };
        g.vcc = g.node(0, 0, "G:VCC");
        let wire_node = |g: &mut Graph, x: i32, y: i32, w: &str| -> NodeId {
            match resolve(chip, x, y, w, width, height) {
                Resolved::Vcc => g.vcc,
                Resolved::At(nx, ny, n) => g.node(nx, ny, &n),
            }
        };
        for (ti, t) in chip.tiles.iter().enumerate() {
            let tb = db.tile_bits(&t.tiletype);
            let (x, y) = (t.x as i32, t.y as i32);
            for (kind, to, from) in tb
                .pips
                .iter()
                .flat_map(|(to, ps)| ps.iter().map(move |p| (ArcKind::Pip, to, &p.from_wire)))
                .chain(tb.conns.iter().flat_map(|(to, cs)| cs.iter().map(move |c| (ArcKind::Fixed, to, &c.from_wire))))
            {
                let tn = wire_node(&mut g, x, y, to);
                let fnode = wire_node(&mut g, x, y, from);
                let to_wire = g.intern(to);
                let from_wire = g.intern(from);
                g.arcs.push(Arc { from: fnode, to: tn, kind, tile: ti as u32, to_wire, from_wire });
            }
            for bel in tile_bels(&t.tiletype, tb) {
                let pin_nodes = bel.pins.iter().map(|p| Some(wire_node(&mut g, x, y, &p.wire))).collect();
                let pb = PlacedBel { tile: ti as u32, x: x + bel.rel_x, y: y + bel.rel_y, bel, pin_nodes };
                let key = pb.group();
                if g.bel_ix.insert(key.clone(), g.bels.len()).is_some() {
                    return Err(err!("bel {key} defined twice"));
                }
                g.bels.push(pb);
            }
        }
        let n = g.nodes.len();
        let csr = |key: &dyn Fn(&Arc) -> NodeId| -> (Vec<u32>, Vec<u32>) {
            let mut off = vec![0u32; n + 1];
            for a in &g.arcs {
                off[key(a) as usize + 1] += 1;
            }
            for i in 0..n {
                off[i + 1] += off[i];
            }
            let mut fill = off.clone();
            let mut idx = vec![0u32; g.arcs.len()];
            for (i, a) in g.arcs.iter().enumerate() {
                let k = key(a) as usize;
                idx[fill[k] as usize] = i as u32;
                fill[k] += 1;
            }
            (off, idx)
        };
        let (down_off, down_idx) = csr(&|a| a.from);
        let (up_off, up_idx) = csr(&|a| a.to);
        g.down_off = down_off;
        g.down_idx = down_idx;
        g.up_off = up_off;
        g.up_idx = up_idx;
        // Union-find over fixed connections.
        let mut parent: Vec<NodeId> = (0..n as NodeId).collect();
        fn find(p: &mut [NodeId], mut x: NodeId) -> NodeId {
            while p[x as usize] != x {
                p[x as usize] = p[p[x as usize] as usize];
                x = p[x as usize];
            }
            x
        }
        for a in &g.arcs {
            if a.kind == ArcKind::Fixed {
                let (ra, rb) = (find(&mut parent, a.from), find(&mut parent, a.to));
                if ra != rb {
                    parent[ra.max(rb) as usize] = ra.min(rb);
                }
            }
        }
        g.group = (0..n as NodeId).map(|x| find(&mut parent, x)).collect();
        Ok(g)
    }
}

/// Split `N2E1:FOO` into (dx, dy, "FOO"); None for no or a non-positional prefix.
fn parse_rel(prefix: &str) -> Option<(i32, i32)> {
    let mut dx = 0;
    let mut dy = 0;
    let b = prefix.as_bytes();
    let mut i = 0;
    if b.is_empty() {
        return None;
    }
    while i < b.len() {
        let d = b[i];
        let start = i + 1;
        let mut j = start;
        while j < b.len() && b[j].is_ascii_digit() {
            j += 1;
        }
        if j == start {
            return None;
        }
        let v: i32 = prefix[start..j].parse().ok()?;
        match d {
            b'N' => dy = -v,
            b'S' => dy = v,
            b'E' => dx = v,
            b'W' => dx = -v,
            _ => return None,
        }
        i = j;
    }
    Some((dx, dy))
}

fn resolve(chip: &Chip, x: i32, y: i32, wire: &str, w: i32, h: i32) -> Resolved {
    let Some(colon) = wire.find(':') else {
        return Resolved::At(x, y, wire.to_string());
    };
    let (prefix, base) = (&wire[..colon], &wire[colon + 1..]);
    let glb = &chip.db.globals;
    let local = || Resolved::At(x, y, wire.to_string());
    let at = |p: Option<(usize, usize)>| match p {
        Some((nx, ny)) => Resolved::At(nx as i32, ny as i32, base.to_string()),
        None => Resolved::At(x, y, wire.to_string()),
    };
    match prefix {
        "G" if base == "VCC" => Resolved::Vcc,
        "G" => Resolved::At(0, 0, base.to_string()),
        "BRANCH" => at(glb.branch_sink_to_origin(x as usize).map(|c| (c, y as usize))),
        "BRANCH_L" | "BRANCH_R" => {
            let c = x + if prefix == "BRANCH_R" { 2 } else { -2 };
            if c < 0 {
                return local();
            }
            at(glb.branch_sink_to_origin(c as usize).map(|c| (c, y as usize)))
        }
        "SPINE" => at(glb.spine_sink_to_origin(x as usize, y as usize)),
        "HROW" => at(glb.hrow_sink_to_origin(x as usize, y as usize)),
        "DQSG" => {
            let col = chip
                .db
                .iodb
                .pads
                .iter()
                .filter(|p| p.offset == x && p.dqs.len() > 1)
                .map(|p| p.dqs[1])
                .last();
            at(col.filter(|&c| c >= 0).map(|c| (c as usize, y as usize)))
        }
        _ => match parse_rel(prefix) {
            Some((dx, dy)) => {
                let (nx, ny) = (x + dx, y + dy);
                if nx >= 0 && ny >= 0 && nx < w && ny < h {
                    Resolved::At(nx, ny, base.to_string())
                } else {
                    local()
                }
            }
            None => local(),
        },
    }
}

#[cfg(test)]
mod tests {
    use super::parse_rel;

    #[test]
    fn relative_prefixes_parse() {
        assert_eq!(parse_rel("N1"), Some((0, -1)));
        assert_eq!(parse_rel("S2E1"), Some((1, 2)));
        assert_eq!(parse_rel("N12W3"), Some((-3, -12)));
        assert_eq!(parse_rel("BANK"), None);
        assert_eq!(parse_rel(""), None);
    }
}
