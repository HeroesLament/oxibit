//! The ECP5 routing graph, as libtrellis's `Chip::get_routing_graph_ecp5`
//! (`TileBitDatabase::add_routing`, `RoutingGraph::globalise_net_ecp5`)
//! defines it, held per tile type instead of per arc.
//!
//! Every tile's muxes and fixed connections name wires relative to the
//! tile, which sits at the first `R<row>C<col>` in its name. A wire name
//! becomes a wire of the chip:
//! - a `25K_`/`45K_`/`85K_` prefix keeps the wire only on that density,
//!   stripped;
//! - from column 69 on, `PCSA` in a name reads `PCSB` (the two PCS share
//!   tile databases);
//! - `G_`, `L_`, `R_`: a global wire. `G_` wires other than `VPTX`, `HPBX`
//!   and `HPRX` ones sit at the nominal location (0, 0), the rest at their
//!   tile;
//! - otherwise an optional `N<n>`/`S<n>` then `E<n>`/`W<n>` prefix and `_`
//!   move the wire to a relative tile (north is row - n); with no prefix the
//!   wire is the tile's own. A wire off the grid is dropped, with every arc
//!   that touches it.
//!
//! A chip wire is (x = col, y = row, name). The graph stores, per tile type,
//! its pips with both ends classified once, and a reverse index from each
//! wire name to the (type, local wire, placement) that produce it; the arcs
//! of a wire are generated from those on demand. An LFE5U-45F's explicit
//! graph is 2.0M wires and 14.8M arcs; this one is the tile types plus a
//! grid. `explicit_arcs` enumerates the full arc list for checking.
//!
//! Arcs keep their tile and the tile type's pip index, so a routed arc is
//! written back as `arc: <sink> <source>` in that tile's configuration;
//! fixed connections have no bits.

use std::collections::{BTreeMap, HashMap};

use crate::db::DeviceDb;
use crate::{Result, err};

pub type NameId = u32;
pub type TileId = u32;
/// A tile type, or a PCS variant of one (see the module docs).
pub type TypeId = u32;

/// A wire of the chip.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Wire {
    pub x: i16,
    pub y: i16,
    pub name: NameId,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PipKind {
    /// Set by the mux bits of the tile.
    Configurable,
    /// Always connected (`.fixed_conn`).
    Fixed,
}

/// A tile type's arc, in tile-local wire names.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Pip {
    pub sink: String,
    pub source: String,
    pub kind: PipKind,
}

/// One arc of the chip: a pip of a tile.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Arc {
    pub from: Wire,
    pub to: Wire,
    pub tile: TileId,
    /// Index into the tile type's pip list.
    pub pip: u32,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TileLoc {
    pub name: String,
    /// The database tile type.
    pub ty: String,
    pub routing: TypeId,
    pub x: i16,
    pub y: i16,
}

/// Where a tile-local wire lands relative to its tile.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Place {
    /// Not on this density.
    Absent,
    /// A global at (0, 0).
    Origin(NameId),
    /// At the tile offset by (dx, dy).
    Offset { dx: i16, dy: i16, name: NameId },
}

/// A tile type's routing: its pips, the placement of each local wire, and
/// the pips by local source and sink wire.
#[derive(Debug, Clone)]
pub struct TypeRouting {
    pub ty: String,
    pub pips: Vec<Pip>,
    places: Vec<Place>,
    /// (sink, source) local wire of each pip.
    ends: Vec<(u32, u32)>,
    by_source: Vec<Vec<u32>>,
    by_sink: Vec<Vec<u32>>,
}

/// Which PCS a tile's `PCSA` wire names refer to.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Pcs {
    /// As written.
    A,
    /// Read as `PCSB` (columns 69 and up).
    B,
}

/// Which end of an arc a wire is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Direction {
    /// Arcs whose source is the wire.
    Downhill,
    /// Arcs whose sink is the wire.
    Uphill,
}

/// Which local wire of which tile type produces a wire name.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Producer {
    routing: TypeId,
    local: u32,
    place: Place,
}

#[derive(Debug)]
pub struct Graph {
    /// Columns and rows of the grid (max + 1).
    pub width: i16,
    pub height: i16,
    pub names: Vec<String>,
    name_ids: HashMap<String, NameId>,
    pub types: Vec<TypeRouting>,
    pub tiles: Vec<TileLoc>,
    /// Tiles by grid cell, `y * width + x`.
    grid: Vec<Vec<TileId>>,
    /// Tiles by routing type.
    of_type: Vec<Vec<TileId>>,
    /// By wire name.
    producers: Vec<Vec<Producer>>,
}

/// The density prefix of a device: `25K_` for the 12F and 25F.
fn chip_prefix(device: &str) -> Option<&'static str> {
    if device.contains("25F") || device.contains("12F") {
        Some("25K_")
    } else if device.contains("45F") {
        Some("45K_")
    } else if device.contains("85F") {
        Some("85K_")
    } else {
        None
    }
}

/// `^([NS]\d+)?([EW]\d+)?_(.*)`: the relative offset and the rest.
fn split_relative(s: &str) -> Option<(i16, i16, &str)> {
    let bytes = s.as_bytes();
    let mut i = 0;
    let num = |i: &mut usize| -> Option<i16> {
        let start = *i;
        while bytes.get(*i).is_some_and(u8::is_ascii_digit) {
            *i += 1;
        }
        if start == *i {
            None
        } else {
            s[start..*i].parse().ok()
        }
    };
    let (mut dx, mut dy) = (0, 0);
    if let Some(&c @ (b'N' | b'S')) = bytes.first() {
        i += 1;
        let n = num(&mut i)?;
        dy = if c == b'N' { -n } else { n };
    }
    if let Some(&c @ (b'E' | b'W')) = bytes.get(i) {
        i += 1;
        let n = num(&mut i)?;
        dx = if c == b'W' { -n } else { n };
    }
    (bytes.get(i) == Some(&b'_')).then(|| (dx, dy, &s[i + 1..]))
}

/// A classified name: `None` when absent on this density, else
/// (dx, dy, name, at_origin).
fn classify(name: &str, prefix: Option<&str>) -> Option<(i16, i16, String, bool)> {
    let mut stripped = name;
    for p in ["25K_", "45K_", "85K_"] {
        if let Some(rest) = name.strip_prefix(p) {
            if Some(p) != prefix {
                return None;
            }
            stripped = rest;
        }
    }
    if ["G_", "L_", "R_"].iter().any(|p| stripped.starts_with(p)) {
        let at_origin = stripped.starts_with("G_")
            && !["VPTX", "HPBX", "HPRX"]
                .iter()
                .any(|k| stripped.contains(k));
        return Some((0, 0, stripped.to_owned(), at_origin));
    }
    Some(match split_relative(stripped) {
        Some((dx, dy, rest)) => (dx, dy, rest.to_owned(), false),
        None => (0, 0, stripped.to_owned(), false),
    })
}

/// The first `R<row>C<col>` in a tile name.
pub fn tile_row_col(name: &str) -> Option<(i16, i16)> {
    let b = name.as_bytes();
    let digits = |from: usize| {
        let mut e = from;
        while b.get(e).is_some_and(u8::is_ascii_digit) {
            e += 1;
        }
        (e > from).then(|| (name[from..e].parse::<i16>().ok(), e))
    };
    for i in 0..b.len() {
        if b[i] != b'R' {
            continue;
        }
        let Some((Some(row), e)) = digits(i + 1) else {
            continue;
        };
        if b.get(e) != Some(&b'C') {
            continue;
        }
        if let Some((Some(col), _)) = digits(e + 1) {
            return Some((row, col));
        }
    }
    None
}

fn pips_of(bits: &crate::db::TileBits) -> Vec<Pip> {
    let mut list = Vec::new();
    for (sink, mux) in &bits.muxes {
        for source in mux.arcs.keys() {
            list.push(Pip {
                sink: sink.clone(),
                source: source.clone(),
                kind: PipKind::Configurable,
            });
        }
    }
    for (sink, source) in &bits.fixed_conns {
        list.push(Pip {
            sink: sink.clone(),
            source: source.clone(),
            kind: PipKind::Fixed,
        });
    }
    list
}

impl Graph {
    fn name_id(&mut self, s: &str) -> NameId {
        if let Some(&id) = self.name_ids.get(s) {
            return id;
        }
        let id = NameId::try_from(self.names.len()).expect("name count fits u32");
        self.names.push(s.to_owned());
        self.name_ids.insert(s.to_owned(), id);
        id
    }

    /// A routing type for `ty`, with `PCSA` read as `PCSB` for `Pcs::B`.
    fn add_type(&mut self, ty: &str, pips: Vec<Pip>, prefix: Option<&str>, pcs: Pcs) -> TypeId {
        let mut locals: HashMap<String, u32> = HashMap::new();
        let mut places = Vec::new();
        let mut local = |g: &mut Graph, raw: &str| -> u32 {
            if let Some(&l) = locals.get(raw) {
                return l;
            }
            let name = if pcs == Pcs::B && raw.contains("PCSA") {
                raw.replacen("PCSA", "PCSB", 1)
            } else {
                raw.to_owned()
            };
            let place = match classify(&name, prefix) {
                None => Place::Absent,
                Some((_, _, n, true)) => Place::Origin(g.name_id(&n)),
                Some((dx, dy, n, false)) => Place::Offset {
                    dx,
                    dy,
                    name: g.name_id(&n),
                },
            };
            let l = u32::try_from(places.len()).expect("local wire count fits u32");
            places.push(place);
            locals.insert(raw.to_owned(), l);
            l
        };
        let ends: Vec<(u32, u32)> = pips
            .iter()
            .map(|p| (local(self, &p.sink), local(self, &p.source)))
            .collect();
        let mut by_source = vec![Vec::new(); places.len()];
        let mut by_sink = vec![Vec::new(); places.len()];
        for (i, &(snk, src)) in ends.iter().enumerate() {
            let i = u32::try_from(i).expect("pip count fits u32");
            by_sink[snk as usize].push(i);
            by_source[src as usize].push(i);
        }
        let id = TypeId::try_from(self.types.len()).expect("type count fits u32");
        for (l, &place) in places.iter().enumerate() {
            let name = match place {
                Place::Absent => continue,
                Place::Origin(n) | Place::Offset { name: n, .. } => n,
            };
            if self.producers.len() <= name as usize {
                self.producers.resize(name as usize + 1, Vec::new());
            }
            self.producers[name as usize].push(Producer {
                routing: id,
                local: u32::try_from(l).expect("fits"),
                place,
            });
        }
        self.types.push(TypeRouting {
            ty: ty.to_owned(),
            pips,
            places,
            ends,
            by_source,
            by_sink,
        });
        self.of_type.push(Vec::new());
        id
    }

    pub fn build(db: &DeviceDb) -> Result<Graph> {
        let width = i16::try_from(db.info.max_col + 1).map_err(|_| err!("max_col out of range"))?;
        let height =
            i16::try_from(db.info.max_row + 1).map_err(|_| err!("max_row out of range"))?;
        let prefix = chip_prefix(&db.info.name);
        let mut g = Graph {
            width,
            height,
            names: Vec::new(),
            name_ids: HashMap::new(),
            types: Vec::new(),
            tiles: Vec::with_capacity(db.tiles.len()),
            grid: vec![
                Vec::new();
                usize::try_from(width).expect("positive")
                    * usize::try_from(height).expect("positive")
            ],
            of_type: Vec::new(),
            producers: Vec::new(),
        };
        let mut type_ids: BTreeMap<(String, Pcs), TypeId> = BTreeMap::new();
        for (name, t) in &db.tiles {
            let (row, col) = tile_row_col(name)
                .ok_or_else(|| err!("tile {name}: no R<row>C<col> in its name"))?;
            if col >= width || row >= height {
                return Err(err!(
                    "tile {name} at R{row}C{col} is off the {width}x{height} grid"
                ));
            }
            let bits = db.tile_bits(&t.ty)?;
            let has_pcsa = bits
                .muxes
                .iter()
                .any(|(s, m)| s.contains("PCSA") || m.arcs.keys().any(|k| k.contains("PCSA")))
                || bits
                    .fixed_conns
                    .iter()
                    .any(|(a, b)| a.contains("PCSA") || b.contains("PCSA"));
            let pcs = if col >= 69 && has_pcsa {
                Pcs::B
            } else {
                Pcs::A
            };
            let key = (t.ty.clone(), pcs);
            let routing = if let Some(&id) = type_ids.get(&key) {
                id
            } else {
                let id = g.add_type(&t.ty, pips_of(bits), prefix, pcs);
                type_ids.insert(key, id);
                id
            };
            let tile = TileId::try_from(g.tiles.len()).expect("tile count fits u32");
            g.tiles.push(TileLoc {
                name: name.clone(),
                ty: t.ty.clone(),
                routing,
                x: col,
                y: row,
            });
            let cell = g.cell(col, row);
            g.grid[cell].push(tile);
            g.of_type[routing as usize].push(tile);
        }
        Ok(g)
    }

    fn cell(&self, x: i16, y: i16) -> usize {
        usize::try_from(y).expect("on grid") * usize::try_from(self.width).expect("positive")
            + usize::try_from(x).expect("on grid")
    }

    fn on_grid(&self, x: i16, y: i16) -> bool {
        (0..self.width).contains(&x) && (0..self.height).contains(&y)
    }

    /// The id of a wire name, added if new (a bel pin's wire need not be
    /// in any pip).
    pub fn intern(&mut self, name: &str) -> NameId {
        self.name_id(name)
    }

    pub fn name_id_of(&self, name: &str) -> Option<NameId> {
        self.name_ids.get(name).copied()
    }

    pub fn wire_name(&self, w: Wire) -> String {
        format!("R{}C{}_{}", w.y, w.x, self.names[w.name as usize])
    }

    pub fn pip(&self, a: &Arc) -> &Pip {
        &self.types[self.tiles[a.tile as usize].routing as usize].pips[a.pip as usize]
    }

    /// The chip wire a tile's local wire is, if present and on the grid.
    fn local_wire(&self, tile: &TileLoc, local: u32) -> Option<Wire> {
        match self.types[tile.routing as usize].places[local as usize] {
            Place::Absent => None,
            Place::Origin(name) => Some(Wire { x: 0, y: 0, name }),
            Place::Offset { dx, dy, name } => {
                let (x, y) = (tile.x + dx, tile.y + dy);
                self.on_grid(x, y).then_some(Wire { x, y, name })
            }
        }
    }

    /// The tiles whose local wire `p` is `w`.
    fn tiles_producing<'a>(
        &'a self,
        w: Wire,
        p: &'a Producer,
    ) -> Box<dyn Iterator<Item = TileId> + 'a> {
        match p.place {
            Place::Absent => Box::new(std::iter::empty()),
            Place::Origin(_) => {
                if (w.x, w.y) == (0, 0) {
                    Box::new(self.of_type[p.routing as usize].iter().copied())
                } else {
                    Box::new(std::iter::empty())
                }
            }
            Place::Offset { dx, dy, .. } => {
                let (x, y) = (w.x - dx, w.y - dy);
                if !self.on_grid(x, y) {
                    return Box::new(std::iter::empty());
                }
                Box::new(
                    self.grid[self.cell(x, y)]
                        .iter()
                        .copied()
                        .filter(move |&t| self.tiles[t as usize].routing == p.routing),
                )
            }
        }
    }

    fn arcs_of(&self, w: Wire, dir: Direction) -> Vec<Arc> {
        let mut out = Vec::new();
        let Some(producers) = self.producers.get(w.name as usize) else {
            return out;
        };
        for p in producers {
            let ty = &self.types[p.routing as usize];
            let pips = if dir == Direction::Downhill {
                &ty.by_source[p.local as usize]
            } else {
                &ty.by_sink[p.local as usize]
            };
            if pips.is_empty() {
                continue;
            }
            for tile in self.tiles_producing(w, p) {
                let tl = &self.tiles[tile as usize];
                for &pip in pips {
                    let (snk, src) = ty.ends[pip as usize];
                    let other = if dir == Direction::Downhill { snk } else { src };
                    let Some(o) = self.local_wire(tl, other) else {
                        continue;
                    };
                    let (from, to) = if dir == Direction::Downhill {
                        (w, o)
                    } else {
                        (o, w)
                    };
                    out.push(Arc {
                        from,
                        to,
                        tile,
                        pip,
                    });
                }
            }
        }
        out
    }

    /// Arcs leaving `w`.
    pub fn downhill(&self, w: Wire) -> Vec<Arc> {
        self.arcs_of(w, Direction::Downhill)
    }

    /// Arcs into `w`.
    pub fn uphill(&self, w: Wire) -> Vec<Arc> {
        self.arcs_of(w, Direction::Uphill)
    }

    /// Every arc of the chip, tile by tile: libtrellis's arc list. For
    /// checking; an 85F has tens of millions.
    pub fn explicit_arcs(&self) -> impl Iterator<Item = Arc> + '_ {
        self.tiles.iter().enumerate().flat_map(move |(t, tl)| {
            let tile = TileId::try_from(t).expect("tile count fits u32");
            let ty = &self.types[tl.routing as usize];
            ty.ends
                .iter()
                .enumerate()
                .filter_map(move |(i, &(snk, src))| {
                    let to = self.local_wire(tl, snk)?;
                    let from = self.local_wire(tl, src)?;
                    Some(Arc {
                        from,
                        to,
                        tile,
                        pip: u32::try_from(i).expect("pip count fits u32"),
                    })
                })
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wire_names() {
        assert_eq!(split_relative("N1_V02S0501"), Some((0, -1, "V02S0501")));
        assert_eq!(split_relative("S3E2_X"), Some((2, 3, "X")));
        assert_eq!(split_relative("W1_H02E0701"), Some((-1, 0, "H02E0701")));
        assert_eq!(split_relative("H02E0501"), None);
        assert_eq!(split_relative("F5_A"), None);
        assert_eq!(classify("45K_X", Some("85K_")), None);
        assert_eq!(
            classify("85K_N1_X", Some("85K_")),
            Some((0, -1, "X".into(), false))
        );
        assert_eq!(
            classify("G_HPBX0000", None),
            Some((0, 0, "G_HPBX0000".into(), false))
        );
        assert_eq!(classify("G_VCC", None), Some((0, 0, "G_VCC".into(), true)));
        assert_eq!(tile_row_col("CIB_R10C100:CIB_DSP"), Some((10, 100)));
        assert_eq!(tile_row_col("R2C3:PLC2"), Some((2, 3)));
    }
}
