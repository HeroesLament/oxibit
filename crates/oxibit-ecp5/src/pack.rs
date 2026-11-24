//! Packing the ECP5 cell netlist into PLC2 tiles.
//!
//! A slice is two lanes (LUT K<i> with flop REG<i>, i = 0, 1) or one
//! CCU2C with its two flops. A lane's flop takes D from its own LUT (or
//! carry half) through DI when that LUT drives it; otherwise from M.
//!
//! Legality (Trellis PLC2):
//! - a slice's flops share one clock, set/reset and enable (`CLK<z>`,
//!   `LSR<z>`, `CE<z>`);
//! - a tile has two clocks (`CLK0`/`CLK1`, each with its own `CLKMUX`
//!   inversion) and two set/resets (`LSR0`/`LSR1`, each with `LSRMUX` and
//!   `SRMODE`) that its slices pick from (`MUXCLK<z>`, `MUXLSR<z>`); so at
//!   most two distinct clocks and two distinct set/resets per tile, and one
//!   polarity (and mode) per net;
//! - a carry chain runs slice A -> D in a tile and on into slice A of the
//!   next tile east (`FCO` -> `E1_HFIE0000` -> `FCI`), so a chain is a run
//!   of consecutive tiles, starting at slice A.
//!
//! V1 packs at tile granularity: each tile is a placement unit; a chain's
//! tiles are a macro and hold only the chain and its flops.

use std::collections::HashMap;

use crate::netlist::{Cell, CellId, ControlSet, Lsr, NetId, Netlist, Polarity};
use crate::{Result, err};

/// A LUT and/or a flop sharing a lane.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Lane {
    pub lut: Option<CellId>,
    pub ff: Option<CellId>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SliceKind {
    Logic([Lane; 2]),
    /// A CCU2C; each half's flop (`ff[j]`) takes D from that half's sum.
    Carry { ccu2: CellId, ff: [Option<CellId>; 2] },
    /// Half of a distributed RAM (TRELLIS_DPR16X4S `ram`): slice A holds
    /// its LUTs 0 and 1, slice B LUTs 2 and 3 (`half` 0 or 1).
    Dpram { ram: CellId, half: usize },
    /// Slice C of a distributed RAM tile: the write port (WAD, DI).
    Ramw { ram: CellId },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Slice {
    pub kind: SliceKind,
    /// The flops' control set, when the slice has flops.
    pub ctrl: Option<ControlSet>,
}

/// One PLC2 tile's contents: slice z = 0..4 (A..D).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Tile {
    pub slices: [Option<Slice>; 4],
}

/// Where a cell went.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Loc {
    pub tile: usize,
    pub z: usize,
    /// Lane (logic) or half (carry).
    pub lane: usize,
}

#[derive(Clone, Debug, Default)]
pub struct Packed {
    pub tiles: Vec<Tile>,
    /// Each chain's tiles, west to east.
    pub chains: Vec<Vec<usize>>,
    /// Per cell, its place (LUTs, CCU2Cs, flops).
    pub loc: Vec<Option<Loc>>,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct PackStats {
    pub tiles: usize,
    pub chain_tiles: usize,
    pub slices: usize,
    pub lut_ff_pairs: usize,
    pub carry_ffs: usize,
    pub lone_ffs: usize,
}

/// A tile's control resources as slices fill it.
#[derive(Clone, Debug, Default)]
struct TileCtrl {
    clks: Vec<(NetId, Polarity)>,
    lsrs: Vec<Lsr>,
    slices: usize,
}

impl TileCtrl {
    /// Whether a slice with `c` fits, and the resulting state.
    fn with(&self, c: Option<&ControlSet>) -> Option<TileCtrl> {
        if self.slices >= 4 {
            return None;
        }
        let mut t = self.clone();
        t.slices += 1;
        if let Some(c) = c {
            let clk = (c.clk, c.clk_polarity);
            if !t.clks.contains(&clk) {
                if t.clks.iter().any(|x| x.0 == c.clk) || t.clks.len() == 2 {
                    return None;
                }
                t.clks.push(clk);
            }
            if let Some(l) = c.lsr {
                if !t.lsrs.contains(&l) {
                    if t.lsrs.iter().any(|x| x.net == l.net) || t.lsrs.len() == 2 {
                        return None;
                    }
                    t.lsrs.push(l);
                }
            }
        }
        Some(t)
    }
}

fn ff_ctrl(nl: &Netlist, ff: CellId) -> ControlSet {
    match &nl.cells[ff as usize] {
        Cell::Ff { ctrl, .. } => *ctrl,
        _ => unreachable!("not a flop"),
    }
}

fn ff_d(nl: &Netlist, ff: CellId) -> NetId {
    match &nl.cells[ff as usize] {
        Cell::Ff { d, .. } => *d,
        _ => unreachable!("not a flop"),
    }
}

/// Pack `nl`.
pub fn pack(nl: &Netlist) -> Result<(Packed, PackStats)> {
    let drivers = nl.drivers();
    let mut stats = PackStats::default();
    let mut used_ff = vec![false; nl.cells.len()];
    let mut used_lut = vec![false; nl.cells.len()];
    // Flops by D net.
    let mut ffs_by_d: HashMap<NetId, Vec<CellId>> = HashMap::new();
    for (i, c) in nl.cells.iter().enumerate() {
        if let Cell::Ff { d, .. } = c {
            ffs_by_d.entry(*d).or_default().push(CellId::try_from(i).expect("fits"));
        }
    }
    let mut packed = Packed {
        loc: vec![None; nl.cells.len()],
        ..Packed::default()
    };

    // Carry chains, head first.
    let mut next_of: HashMap<CellId, CellId> = HashMap::new();
    for (i, c) in nl.cells.iter().enumerate() {
        if let Cell::Ccu2 { cin: Some(p), .. } = c {
            next_of.insert(*p, CellId::try_from(i).expect("fits"));
        }
    }
    for (i, c) in nl.cells.iter().enumerate() {
        let Cell::Ccu2 { cin: None, .. } = c else {
            continue;
        };
        let mut chain = vec![CellId::try_from(i).expect("fits")];
        while let Some(&n) = next_of.get(chain.last().expect("nonempty")) {
            chain.push(n);
        }
        let n_tiles = chain.len().div_ceil(4);
        let first = packed.tiles.len();
        let mut ctrls: Vec<TileCtrl> = vec![TileCtrl::default(); n_tiles];
        for _ in 0..n_tiles {
            packed.tiles.push(Tile::default());
        }
        for (k, &cc) in chain.iter().enumerate() {
            let (t, z) = (k / 4, k % 4);
            let Cell::Ccu2 { halves, .. } = &nl.cells[cc as usize] else {
                return Err(err!("chain link {cc} is not a CCU2C"));
            };
            // Flops on the sums, sharing one control set.
            let mut ff = [None, None];
            let mut ctrl: Option<ControlSet> = None;
            for (j, h) in halves.iter().enumerate() {
                let Some(s) = h.s else { continue };
                let Some(cands) = ffs_by_d.get(&s) else {
                    continue;
                };
                for &f in cands {
                    if used_ff[f as usize] {
                        continue;
                    }
                    let fc = ff_ctrl(nl, f);
                    if ctrl.is_some_and(|c| c != fc) {
                        continue;
                    }
                    // The tile's control resources, with this slice's set.
                    let probe = TileCtrl {
                        slices: 0,
                        ..ctrls[t].clone()
                    };
                    if probe.with(Some(&fc)).is_none() {
                        continue;
                    }
                    ff[j] = Some(f);
                    ctrl = Some(fc);
                    used_ff[f as usize] = true;
                    break;
                }
            }
            ctrls[t] = ctrls[t]
                .with(ctrl.as_ref())
                .ok_or_else(|| err!("carry slice does not fit its tile"))?;
            packed.tiles[first + t].slices[z] = Some(Slice {
                kind: SliceKind::Carry { ccu2: cc, ff },
                ctrl,
            });
            packed.loc[cc as usize] = Some(Loc {
                tile: first + t,
                z,
                lane: 0,
            });
            for (j, f) in ff.iter().enumerate() {
                if let Some(f) = f {
                    packed.loc[*f as usize] = Some(Loc {
                        tile: first + t,
                        z,
                        lane: j,
                    });
                    stats.carry_ffs += 1;
                }
            }
            stats.slices += 1;
        }
        packed.chains.push((first..first + n_tiles).collect());
        stats.chain_tiles += n_tiles;
    }

    // Lanes: each remaining flop with the LUT driving its D (when that LUT
    // is free), then the remaining LUTs.
    let mut ff_lanes: Vec<(ControlSet, Lane)> = Vec::new();
    for (i, c) in nl.cells.iter().enumerate() {
        let Cell::Ff { .. } = c else { continue };
        if used_ff[i] {
            continue;
        }
        let f = CellId::try_from(i).expect("fits");
        used_ff[i] = true;
        let d = ff_d(nl, f);
        let lut = drivers[d as usize].filter(|&dc| {
            matches!(nl.cells[dc as usize], Cell::Lut { .. }) && !used_lut[dc as usize]
        });
        if let Some(l) = lut {
            used_lut[l as usize] = true;
            stats.lut_ff_pairs += 1;
        } else {
            stats.lone_ffs += 1;
        }
        ff_lanes.push((ff_ctrl(nl, f), Lane { lut, ff: Some(f) }));
    }
    let mut lone_luts: Vec<CellId> = Vec::new();
    for (i, c) in nl.cells.iter().enumerate() {
        if matches!(c, Cell::Lut { .. }) && !used_lut[i] {
            lone_luts.push(CellId::try_from(i).expect("fits"));
        }
    }

    // Slices: flop lanes two at a time per control set (in first-seen
    // order), then lone LUTs into free lanes and new slices.
    let mut order: Vec<ControlSet> = Vec::new();
    let mut by_ctrl: HashMap<ControlSet, Vec<Lane>> = HashMap::new();
    for (c, l) in ff_lanes {
        if !by_ctrl.contains_key(&c) {
            order.push(c);
        }
        by_ctrl.entry(c).or_default().push(l);
    }
    let mut slices: Vec<Slice> = Vec::new();
    for c in order {
        for pair in by_ctrl[&c].chunks(2) {
            let lanes = [pair[0], pair.get(1).copied().unwrap_or_default()];
            slices.push(Slice {
                kind: SliceKind::Logic(lanes),
                ctrl: Some(c),
            });
        }
    }
    let mut luts = lone_luts.into_iter();
    for s in &mut slices {
        if let SliceKind::Logic(lanes) = &mut s.kind {
            for lane in lanes.iter_mut() {
                if lane.lut.is_none() {
                    // A flop lane fed from M can still hold an unrelated LUT.
                    if let Some(l) = luts.next() {
                        lane.lut = Some(l);
                    }
                }
            }
        }
    }
    let rest: Vec<CellId> = luts.collect();
    for pair in rest.chunks(2) {
        slices.push(Slice {
            kind: SliceKind::Logic([
                Lane {
                    lut: Some(pair[0]),
                    ff: None,
                },
                Lane {
                    lut: pair.get(1).copied(),
                    ff: None,
                },
            ]),
            ctrl: None,
        });
    }

    // Tiles: first fit, trying the newest open tile first.
    let mut open: Vec<(usize, TileCtrl)> = Vec::new();
    for s in slices {
        let mut placed = false;
        for (t, st) in open.iter_mut().rev() {
            if let Some(next) = st.with(s.ctrl.as_ref()) {
                let z = st.slices;
                *st = next;
                put(&mut packed, *t, z, s);
                placed = true;
                break;
            }
        }
        if !placed {
            let t = packed.tiles.len();
            packed.tiles.push(Tile::default());
            let st = TileCtrl::default()
                .with(s.ctrl.as_ref())
                .ok_or_else(|| err!("a slice does not fit an empty tile"))?;
            put(&mut packed, t, 0, s);
            open.push((t, st));
        }
        stats.slices += 1;
        open.retain(|(_, st)| st.slices < 4);
    }
    // Distributed RAMs: one tile each, slices A and B in DPRAM mode and
    // slice C as the write port (slice D stays empty, since the RAM
    // holds the tile's CLK1 and LSR1).
    for (i, c) in nl.cells.iter().enumerate() {
        let Cell::Prim(p) = c else { continue };
        if p.ty != crate::soft::LUTRAM {
            continue;
        }
        let ram = CellId::try_from(i).expect("fits");
        let t = packed.tiles.len();
        let mut tile = Tile::default();
        for half in 0..2 {
            tile.slices[half] = Some(Slice {
                kind: SliceKind::Dpram { ram, half },
                ctrl: None,
            });
        }
        tile.slices[2] = Some(Slice {
            kind: SliceKind::Ramw { ram },
            ctrl: None,
        });
        packed.tiles.push(tile);
        packed.loc[i] = Some(Loc { tile: t, z: 0, lane: 0 });
        stats.slices += 3;
    }
    stats.tiles = packed.tiles.len();
    Ok((packed, stats))
}

fn put(packed: &mut Packed, t: usize, z: usize, s: Slice) {
    packed.tiles[t].slices[z] = Some(s);
    if let SliceKind::Logic(lanes) = s.kind {
        for (i, l) in lanes.iter().enumerate() {
            for c in [l.lut, l.ff].into_iter().flatten() {
                packed.loc[c as usize] = Some(Loc {
                    tile: t,
                    z,
                    lane: i,
                });
            }
        }
    }
}
