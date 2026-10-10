//! Frames -> full-configuration bitstream words, in exactly the packet
//! sequence prjxray's `xc7frames2bit` writes for Series-7
//! (`Configuration<Series7>::createConfigurationPackage`), so the output
//! can be checked word for word against it.

use std::collections::BTreeMap;

use crate::FRAME_WORDS;
use crate::ecc::update_ecc;
use crate::layout::{FrameAddress, FrameLayout};

pub type Frame = [u32; FRAME_WORDS];

// Configuration registers (UG470 table 5-23).
const FAR: u32 = 0x01;
const FDRI: u32 = 0x02;
const CMD: u32 = 0x04;
const CTL0: u32 = 0x05;
const MASK: u32 = 0x06;
const COR0: u32 = 0x09;
const IDCODE: u32 = 0x0C;
const COR1: u32 = 0x0E;
const WBSTAR: u32 = 0x10;
const TIMER: u32 = 0x11;
const UNKNOWN_13: u32 = 0x13;
const CTL1: u32 = 0x18;

// CMD values.
const C_NOP: u32 = 0x0;
const C_WCFG: u32 = 0x1;
const C_LFRM: u32 = 0x3;
const C_START: u32 = 0x5;
const C_RCRC: u32 = 0x7;
const C_SWITCH: u32 = 0x9;
const C_GRESTORE: u32 = 0xA;
const C_DESYNC: u32 = 0xD;

/// COR0 as xc7frames2bit sets it: done-in pipeline stage, DONE released in
/// phase 4, no DCI/MMCM stall, GTS in phase 5, GWE in phase 6.
const COR0_VALUE: u32 = (1 << 25) | (3 << 12) | (7 << 9) | (7 << 6) | (4 << 3) | 5;

/// Bus-width detection and sync, as xc7frames2bit writes them.
const PREAMBLE: [u32; 13] = [
    0xFFFF_FFFF, 0xFFFF_FFFF, 0xFFFF_FFFF, 0xFFFF_FFFF, 0xFFFF_FFFF, 0xFFFF_FFFF, 0xFFFF_FFFF, 0xFFFF_FFFF,
    0x0000_00BB, 0x1122_0044, 0xFFFF_FFFF, 0xFFFF_FFFF, 0xAA99_5566,
];

/// Two zero frames separate block-type / half / row changes in FDRI data.
const SEPARATOR_WORDS: usize = 2 * FRAME_WORDS;

struct Out(Vec<u32>);

impl Out {
    fn nop(&mut self) {
        self.0.push(0x2000_0000);
    }
    fn nops(&mut self, n: usize) {
        for _ in 0..n {
            self.nop();
        }
    }
    fn write(&mut self, reg: u32, value: u32) {
        self.0.push(type1_write(reg, 1));
        self.0.push(value);
    }
    fn cmd(&mut self, c: u32) {
        self.write(CMD, c);
    }
}

fn type1_write(reg: u32, len: u32) -> u32 {
    (1 << 29) | (2 << 27) | ((reg & 0x3FFF) << 13) | (len & 0x7FF)
}

fn type2_write(len: u32) -> u32 {
    (2 << 29) | (2 << 27) | (len & 0x07FF_FFFF)
}

/// Build the configuration words for `frames` (missing frames are added as
/// zero frames; every frame gets its ECC recomputed).
pub fn full_configuration(frames: &BTreeMap<FrameAddress, Frame>, layout: &FrameLayout) -> Vec<u32> {
    // Frames::readFrames (ECC on every frame given) + addMissingFrames.
    let mut all: BTreeMap<FrameAddress, Frame> = frames.clone();
    for f in all.values_mut() {
        update_ecc(f);
    }
    for a in layout.all() {
        all.entry(a).or_insert([0; FRAME_WORDS]);
    }

    // Configuration::createType2ConfigurationPacketData
    let mut data: Vec<u32> = Vec::with_capacity(all.len() * FRAME_WORDS + 64 * SEPARATOR_WORDS);
    for (&addr, frame) in &all {
        data.extend_from_slice(frame);
        if let Some(n) = layout.next(addr) {
            if n.block_type() != addr.block_type() || n.bottom() != addr.bottom() || n.row() != addr.row() {
                data.extend(std::iter::repeat_n(0, SEPARATOR_WORDS));
            }
        }
    }
    data.extend(std::iter::repeat_n(0, SEPARATOR_WORDS));

    // Configuration<Series7>::createConfigurationPackage
    let mut o = Out(Vec::with_capacity(data.len() + 1024));
    o.0.extend_from_slice(&PREAMBLE);
    o.nop();
    o.write(TIMER, 0);
    o.write(WBSTAR, 0);
    o.cmd(C_NOP);
    o.nop();
    o.cmd(C_RCRC);
    o.nops(2);
    o.write(UNKNOWN_13, 0);
    o.write(COR0, COR0_VALUE);
    o.write(COR1, 0);
    o.write(IDCODE, layout.idcode);
    o.cmd(C_SWITCH);
    o.nop();
    o.write(MASK, 0x401);
    o.write(CTL0, 0x501);
    o.write(MASK, 0);
    o.write(CTL1, 0);
    o.nops(8);
    o.write(FAR, 0);
    o.cmd(C_WCFG);
    o.nop();
    o.0.push(type1_write(FDRI, 0));
    o.0.push(type2_write(data.len() as u32));
    o.0.extend_from_slice(&data);
    o.cmd(C_RCRC);
    o.nops(2);
    o.cmd(C_GRESTORE);
    o.nop();
    o.cmd(C_LFRM);
    o.nops(100);
    o.cmd(C_START);
    o.nop();
    o.write(FAR, 0x03BE_0000);
    o.write(MASK, 0x501);
    o.write(CTL0, 0x501);
    o.cmd(C_RCRC);
    o.nops(2);
    o.cmd(C_DESYNC);
    o.nops(400);
    o.0
}
