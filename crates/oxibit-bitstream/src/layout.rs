//! The configuration frame address space of one part.
//!
//! Read from prjxray's `part.yaml` (the `global_clock_regions` form: half ->
//! row -> block type -> column -> frame count). [`FrameLayout::next`] is a
//! line-for-line port of prjxray's `Part::GetNextFrameAddress` chain, because
//! the order it defines decides where `xc7frames2bit` inserts the zero frames
//! that separate rows, and output has to match it word for word.

use std::collections::BTreeMap;
use std::ops::Bound::{Excluded, Unbounded};

pub const CLB_IO_CLK: u32 = 0;
pub const BLOCK_RAM: u32 = 1;
pub const CFG_CLB: u32 = 2;

/// A FAR value: block type [25:23], bottom half [22], row [21:17],
/// column [16:7], minor [6:0].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct FrameAddress(pub u32);

impl FrameAddress {
    pub fn new(block_type: u32, bottom: bool, row: u32, column: u32, minor: u32) -> Self {
        FrameAddress(
            (block_type & 0x7) << 23
                | (bottom as u32) << 22
                | (row & 0x1F) << 17
                | (column & 0x3FF) << 7
                | (minor & 0x7F),
        )
    }
    pub fn block_type(self) -> u32 {
        (self.0 >> 23) & 0x7
    }
    pub fn bottom(self) -> bool {
        (self.0 >> 22) & 1 == 1
    }
    pub fn row(self) -> u32 {
        (self.0 >> 17) & 0x1F
    }
    pub fn column(self) -> u32 {
        (self.0 >> 7) & 0x3FF
    }
    pub fn minor(self) -> u32 {
        self.0 & 0x7F
    }
}

/// column -> frame count
type Bus = BTreeMap<u32, u32>;
/// block type -> bus
type Row = BTreeMap<u32, Bus>;
/// row -> row
type Region = BTreeMap<u32, Row>;

#[derive(Debug, Clone, Default)]
pub struct FrameLayout {
    pub idcode: u32,
    top: Region,
    bottom: Region,
}

impl FrameLayout {
    /// Parse prjxray's `part.yaml`. Only the shape prjxray writes is
    /// accepted; anything else is an error rather than a guess.
    pub fn from_part_yaml(text: &str) -> Result<Self, String> {
        let mut layout = FrameLayout::default();
        let mut bottom: Option<bool> = None;
        let mut row: Option<u32> = None;
        let mut bus: Option<u32> = None;
        let mut column: Option<u32> = None;

        for (n, raw) in text.lines().enumerate() {
            let line = raw.trim();
            let err = |what: &str| format!("part.yaml line {}: {what}: {raw}", n + 1);

            if let Some(v) = line.strip_prefix("idcode:") {
                let v = v.trim();
                layout.idcode = u32::from_str_radix(v.trim_start_matches("0x"), 16).map_err(|_| err("bad idcode"))?;
            } else if line.starts_with("top:") {
                bottom = Some(false);
            } else if line.starts_with("bottom:") {
                bottom = Some(true);
            } else if line.contains("!<xilinx/xc7series/row>") {
                row = Some(leading_number(line).ok_or_else(|| err("row key"))?);
            } else if line.contains("!<xilinx/xc7series/configuration_bus>") {
                let name = line.split(':').next().unwrap_or("");
                bus = Some(match name {
                    "CLB_IO_CLK" => CLB_IO_CLK,
                    "BLOCK_RAM" => BLOCK_RAM,
                    "CFG_CLB" => CFG_CLB,
                    _ => return Err(err("block type")),
                });
            } else if line.contains("!<xilinx/xc7series/configuration_column>") {
                column = Some(leading_number(line).ok_or_else(|| err("column key"))?);
            } else if let Some(v) = line.strip_prefix("frame_count:") {
                let count: u32 = v.trim().parse().map_err(|_| err("frame_count"))?;
                let (b, r, t, c) = match (bottom, row, bus, column) {
                    (Some(b), Some(r), Some(t), Some(c)) => (b, r, t, c),
                    _ => return Err(err("frame_count outside a column")),
                };
                let region = if b { &mut layout.bottom } else { &mut layout.top };
                region.entry(r).or_default().entry(t).or_default().insert(c, count);
            }
        }

        if layout.top.is_empty() && layout.bottom.is_empty() {
            return Err("part.yaml: no frames found".into());
        }
        Ok(layout)
    }

    fn region(&self, bottom: bool) -> &Region {
        if bottom { &self.bottom } else { &self.top }
    }

    pub fn is_valid(&self, a: FrameAddress) -> bool {
        region_valid(self.region(a.bottom()), a)
    }

    /// prjxray `Part::GetNextFrameAddress`.
    pub fn next(&self, a: FrameAddress) -> Option<FrameAddress> {
        if let Some(n) = region_next(self.region(a.bottom()), a) {
            return Some(n);
        }
        if !a.bottom() {
            let n = FrameAddress::new(a.block_type(), true, 0, 0, 0);
            if region_valid(&self.bottom, n) {
                return Some(n);
            }
        }
        if a.block_type() < BLOCK_RAM {
            let n = FrameAddress::new(BLOCK_RAM, false, 0, 0, 0);
            if self.is_valid(n) {
                return Some(n);
            }
        }
        if a.block_type() < CFG_CLB {
            let n = FrameAddress::new(CFG_CLB, false, 0, 0, 0);
            if self.is_valid(n) {
                return Some(n);
            }
        }
        None
    }

    /// Every address reachable from FAR 0, in device order: prjxray's
    /// `Frames::addMissingFrames` walk.
    pub fn all(&self) -> Vec<FrameAddress> {
        let mut out = Vec::new();
        let mut cur = Some(FrameAddress(0));
        while let Some(a) = cur {
            out.push(a);
            cur = self.next(a);
        }
        out
    }
}

fn leading_number(line: &str) -> Option<u32> {
    line.split(':').next()?.trim().parse().ok()
}

fn region_valid(region: &Region, a: FrameAddress) -> bool {
    region
        .get(&a.row())
        .and_then(|row| row.get(&a.block_type()))
        .and_then(|bus| bus.get(&a.column()))
        .is_some_and(|&frames| a.minor() < frames)
}

/// `GlobalClockRegion::GetNextFrameAddress`
fn region_next(region: &Region, a: FrameAddress) -> Option<FrameAddress> {
    let row = region.get(&a.row())?;
    if let Some(n) = row_next(row, a) {
        return Some(n);
    }
    let (&next_row, _) = region.range((Excluded(a.row()), Unbounded)).next()?;
    let n = FrameAddress::new(a.block_type(), a.bottom(), next_row, 0, 0);
    region_valid(region, n).then_some(n)
}

/// `Row::GetNextFrameAddress` -> `ConfigurationBus::GetNextFrameAddress`
fn row_next(row: &Row, a: FrameAddress) -> Option<FrameAddress> {
    let bus = row.get(&a.block_type())?;
    let &frames = bus.get(&a.column())?;
    // ConfigurationColumn: the next minor, if this one is valid and not last.
    if a.minor() < frames && a.minor() + 1 < frames {
        return Some(FrameAddress(a.0 + 1));
    }
    let (&next_col, &next_frames) = bus.range((Excluded(a.column()), Unbounded)).next()?;
    let n = FrameAddress::new(a.block_type(), a.bottom(), a.row(), next_col, 0);
    (0 < next_frames).then_some(n)
}
