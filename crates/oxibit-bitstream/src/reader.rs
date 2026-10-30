//! Configuration words -> frames: the inverse of
//! [`writer::full_configuration`](crate::writer::full_configuration), for a
//! full-configuration bitstream (one FAR, one FDRI write of every frame in
//! device order, as `xc7frames2bit`, `oxibit fasm2bit` and an uncompressed
//! Vivado `write_bitstream` lay it out).
//!
//! Frame addresses follow [`FrameLayout::next`] from the FAR written before
//! FDRI. Where the walk changes block type, half or row, the data carries
//! two pad frames the device's address counter steps through; they are
//! skipped, and counted.

use std::collections::BTreeMap;

use crate::FRAME_WORDS;
use crate::layout::{FrameAddress, FrameLayout};
use crate::packet::{Item, Opcode, Reg, cmd, walk};
use crate::writer::Frame;

/// Frames read from a bitstream.
#[derive(Debug, Default)]
pub struct Frames {
    pub frames: BTreeMap<FrameAddress, Frame>,
    /// Pad frames skipped between rows (and after the last).
    pub pad_frames: usize,
    /// Pad frames with a nonzero word.
    pub nonzero_pads: usize,
}

/// Read every frame the words' FDRI writes carry. An error for what this
/// reader does not follow: a multiple-frame write (MFWR), FDRI before any
/// FAR, data that ends inside a frame, or data past the last frame other
/// than pad frames.
pub fn read_frames(words: &[u32], layout: &FrameLayout) -> Result<Frames, String> {
    let mut out = Frames::default();
    let mut far: Option<FrameAddress> = None;
    for item in walk(words).map_err(|e| format!("{e:?}"))? {
        let Item::Packet(p) = item else { continue };
        if p.opcode != Opcode::Write {
            continue;
        }
        match p.reg {
            Reg::FAR => far = p.data.last().map(|&a| FrameAddress(a)),
            Reg::CMD if p.data.contains(&cmd::MFW) => return Err(format!("word {}: multiple-frame write", p.index)),
            Reg::FDRI if !p.data.is_empty() => {
                let start = far.ok_or_else(|| format!("word {}: FDRI before FAR", p.index))?;
                fdri(p.data, start, layout, &mut out).map_err(|e| format!("FDRI at word {}: {e}", p.index))?;
            }
            _ => {}
        }
    }
    Ok(out)
}

fn fdri(data: &[u32], start: FrameAddress, layout: &FrameLayout, out: &mut Frames) -> Result<(), String> {
    let mut chunks = data.chunks(FRAME_WORDS);
    let pad = |chunks: &mut std::slice::Chunks<'_, u32>, out: &mut Frames| -> Result<(), String> {
        for _ in 0..2 {
            let c = chunks.next().ok_or("data ends inside the pad frames")?;
            out.pad_frames += 1;
            out.nonzero_pads += c.iter().any(|&w| w != 0) as usize;
        }
        Ok(())
    };
    let mut addr = Some(start);
    while let Some(a) = addr {
        let Some(c) = chunks.next() else { return Ok(()) };
        if c.len() != FRAME_WORDS {
            return Err(format!("{} words left at {:#010X}", c.len(), a.0));
        }
        if !layout.is_valid(a) {
            return Err(format!("frame address {:#010X} is not in the part", a.0));
        }
        let mut f = [0u32; FRAME_WORDS];
        f.copy_from_slice(c);
        out.frames.insert(a, f);
        addr = layout.next(a);
        match addr {
            Some(n) if n.block_type() != a.block_type() || n.bottom() != a.bottom() || n.row() != a.row() => pad(&mut chunks, out)?,
            Some(_) => {}
            None => pad(&mut chunks, out)?,
        }
    }
    let rest: usize = chunks.map(<[u32]>::len).sum();
    if rest > 0 {
        return Err(format!("{rest} words past the last frame"));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::writer::full_configuration;

    const PART: &str = "idcode: 0x3727093
global_clock_regions:
  top: !<xilinx/xc7series/global_clock_region>
    rows:
      0: !<xilinx/xc7series/row>
        configuration_buses:
          CLB_IO_CLK: !<xilinx/xc7series/configuration_bus>
            configuration_columns:
              0: !<xilinx/xc7series/configuration_column>
                frame_count: 3
              1: !<xilinx/xc7series/configuration_column>
                frame_count: 2
      1: !<xilinx/xc7series/row>
        configuration_buses:
          CLB_IO_CLK: !<xilinx/xc7series/configuration_bus>
            configuration_columns:
              0: !<xilinx/xc7series/configuration_column>
                frame_count: 2
          BLOCK_RAM: !<xilinx/xc7series/configuration_bus>
            configuration_columns:
              0: !<xilinx/xc7series/configuration_column>
                frame_count: 2
";

    #[test]
    fn reads_back_what_the_writer_wrote() {
        let layout = FrameLayout::from_part_yaml(PART).unwrap();
        let mut frames = BTreeMap::new();
        for (i, a) in layout.all().into_iter().enumerate() {
            let mut f = [0u32; FRAME_WORDS];
            f[0] = 0x100 + i as u32;
            f[100] = a.0;
            frames.insert(a, f);
        }
        let words = full_configuration(&frames, &layout);
        let got = read_frames(&words, &layout).unwrap();
        assert_eq!(got.frames.len(), frames.len());
        for (a, f) in &frames {
            let g = &got.frames[a];
            // Word 50 carries the ECC the writer computes.
            assert_eq!((g[0], g[100]), (f[0], f[100]), "{:#x}", a.0);
        }
        assert_eq!(got.nonzero_pads, 0);
        assert!(got.pad_frames >= 4);
    }
}
