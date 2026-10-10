//! Configuration packets (UG470, "Configuration Packets").
//!
//! After the sync word the stream is a sequence of packets. The header word:
//!
//! ```text
//! Type 1:  [31:29]=001  [28:27] opcode  [26:13] register  [12:11] reserved  [10:0] word count
//! Type 2:  [31:29]=010  [28:27] opcode  [26:0] word count          (register of the last type 1)
//! ```
//!
//! A write of DESYNC to CMD ends the synced region; whatever follows is
//! padding until the next sync word, if any (MultiBoot images carry several).

use serde::Serialize;

use crate::{Error, SYNC};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Opcode {
    Nop,
    Read,
    Write,
    Reserved,
}

impl Opcode {
    fn from_bits(bits: u32) -> Self {
        match bits & 0b11 {
            0 => Opcode::Nop,
            1 => Opcode::Read,
            2 => Opcode::Write,
            _ => Opcode::Reserved,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum PacketType {
    Type1,
    Type2,
}

/// A configuration register address. Kept raw so unknown addresses survive a
/// round trip; [`Reg::name`] knows the 7-series set (UG470 table 5-23).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Reg(pub u16);

impl Reg {
    pub const CRC: Reg = Reg(0x00);
    pub const FAR: Reg = Reg(0x01);
    pub const FDRI: Reg = Reg(0x02);
    pub const FDRO: Reg = Reg(0x03);
    pub const CMD: Reg = Reg(0x04);
    pub const CTL0: Reg = Reg(0x05);
    pub const MASK: Reg = Reg(0x06);
    pub const STAT: Reg = Reg(0x07);
    pub const LOUT: Reg = Reg(0x08);
    pub const COR0: Reg = Reg(0x09);
    pub const MFWR: Reg = Reg(0x0A);
    pub const CBC: Reg = Reg(0x0B);
    pub const IDCODE: Reg = Reg(0x0C);
    pub const AXSS: Reg = Reg(0x0D);
    pub const COR1: Reg = Reg(0x0E);
    pub const WBSTAR: Reg = Reg(0x10);
    pub const TIMER: Reg = Reg(0x11);
    pub const RBCRC_SW: Reg = Reg(0x13);
    pub const BOOTSTS: Reg = Reg(0x16);
    pub const CTL1: Reg = Reg(0x18);
    pub const BSPI: Reg = Reg(0x1F);

    pub fn name(self) -> Option<&'static str> {
        Some(match self {
            Reg::CRC => "CRC",
            Reg::FAR => "FAR",
            Reg::FDRI => "FDRI",
            Reg::FDRO => "FDRO",
            Reg::CMD => "CMD",
            Reg::CTL0 => "CTL0",
            Reg::MASK => "MASK",
            Reg::STAT => "STAT",
            Reg::LOUT => "LOUT",
            Reg::COR0 => "COR0",
            Reg::MFWR => "MFWR",
            Reg::CBC => "CBC",
            Reg::IDCODE => "IDCODE",
            Reg::AXSS => "AXSS",
            Reg::COR1 => "COR1",
            Reg::WBSTAR => "WBSTAR",
            Reg::TIMER => "TIMER",
            Reg::RBCRC_SW => "RBCRC_SW",
            Reg::BOOTSTS => "BOOTSTS",
            Reg::CTL1 => "CTL1",
            Reg::BSPI => "BSPI",
            _ => return None,
        })
    }
}

impl std::fmt::Display for Reg {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.name() {
            Some(n) => f.write_str(n),
            None => write!(f, "REG_0x{:02X}", self.0),
        }
    }
}

impl Serialize for Reg {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        s.collect_str(self)
    }
}

/// CMD register values (UG470 table 5-25).
pub mod cmd {
    pub const NULL: u32 = 0x00;
    pub const WCFG: u32 = 0x01;
    pub const MFW: u32 = 0x02;
    pub const LFRM: u32 = 0x03;
    pub const RCFG: u32 = 0x04;
    pub const START: u32 = 0x05;
    pub const RCAP: u32 = 0x06;
    pub const RCRC: u32 = 0x07;
    pub const AGHIGH: u32 = 0x08;
    pub const SWITCH: u32 = 0x09;
    pub const GRESTORE: u32 = 0x0A;
    pub const SHUTDOWN: u32 = 0x0B;
    pub const GCAPTURE: u32 = 0x0C;
    pub const DESYNC: u32 = 0x0D;
    pub const IPROG: u32 = 0x0F;
    pub const CRCC: u32 = 0x10;
    pub const LTIMER: u32 = 0x11;
    pub const BSPI_READ: u32 = 0x12;
    pub const FALL_EDGE: u32 = 0x13;

    pub fn name(value: u32) -> Option<&'static str> {
        Some(match value {
            NULL => "NULL",
            WCFG => "WCFG",
            MFW => "MFW",
            LFRM => "LFRM",
            RCFG => "RCFG",
            START => "START",
            RCAP => "RCAP",
            RCRC => "RCRC",
            AGHIGH => "AGHIGH",
            SWITCH => "SWITCH",
            GRESTORE => "GRESTORE",
            SHUTDOWN => "SHUTDOWN",
            GCAPTURE => "GCAPTURE",
            DESYNC => "DESYNC",
            IPROG => "IPROG",
            CRCC => "CRCC",
            LTIMER => "LTIMER",
            BSPI_READ => "BSPI_READ",
            FALL_EDGE => "FALL_EDGE",
            _ => return None,
        })
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Packet<'a> {
    /// Word index of the header.
    pub index: usize,
    pub header: u32,
    pub kind: PacketType,
    pub opcode: Opcode,
    pub reg: Reg,
    pub data: &'a [u32],
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Item<'a> {
    /// Words outside a synced region: dummy pads and the bus-width pattern
    /// before a sync, NOP padding after a DESYNC.
    Unsynced { index: usize, words: &'a [u32] },
    Sync { index: usize },
    Packet(Packet<'a>),
}

/// Walk the whole word stream into packets.
pub fn walk(words: &[u32]) -> Result<Vec<Item<'_>>, Error> {
    let mut items = Vec::new();
    let mut i = 0;

    while i < words.len() {
        // Unsynced: everything up to the next sync word is padding.
        let Some(offset) = words[i..].iter().position(|&w| w == SYNC) else {
            items.push(Item::Unsynced { index: i, words: &words[i..] });
            break;
        };
        if offset > 0 {
            items.push(Item::Unsynced { index: i, words: &words[i..i + offset] });
        }
        i += offset;
        items.push(Item::Sync { index: i });
        i += 1;

        // Synced: packets until a DESYNC command or the end of the data.
        let mut last_reg: Option<Reg> = None;
        while i < words.len() {
            let header = words[i];
            let (kind, reg, count) = match header >> 29 {
                0b001 => {
                    let reg = Reg(((header >> 13) & 0x3FFF) as u16);
                    last_reg = Some(reg);
                    (PacketType::Type1, reg, (header & 0x7FF) as usize)
                }
                0b010 => {
                    let reg = last_reg.ok_or(Error::OrphanType2 { index: i, header })?;
                    (PacketType::Type2, reg, (header & 0x07FF_FFFF) as usize)
                }
                _ => return Err(Error::BadPacketType { index: i, header }),
            };
            let opcode = Opcode::from_bits(header >> 27);

            let start = i + 1;
            let have = words.len() - start;
            if count > have {
                return Err(Error::Truncated { index: i, header, need: count, have });
            }
            let data = &words[start..start + count];
            items.push(Item::Packet(Packet { index: i, header, kind, opcode, reg, data }));
            i = start + count;

            let desync = opcode == Opcode::Write && reg == Reg::CMD && data.contains(&cmd::DESYNC);
            if desync {
                break;
            }
        }
    }

    Ok(items)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn walks_type1_type2_and_desync() {
        let words = [
            0xFFFF_FFFF,
            SYNC,
            0x2000_0000,              // NOP
            0x3000_8001, cmd::RCRC,   // write CMD, 1 word
            0x3000_4000,              // write FDRI, 0 words
            0x5000_0002, 0xA, 0xB,    // type 2, 2 words -> FDRI
            0x3000_8001, cmd::DESYNC, // write CMD DESYNC
            0x2000_0000, 0x2000_0000, // trailing NOPs
        ];
        let items = walk(&words).unwrap();
        let packets: Vec<_> = items
            .iter()
            .filter_map(|it| if let Item::Packet(p) = it { Some(p) } else { None })
            .collect();
        assert_eq!(packets.len(), 5);
        assert_eq!(packets[1].reg, Reg::CMD);
        assert_eq!(packets[3].kind, PacketType::Type2);
        assert_eq!(packets[3].reg, Reg::FDRI);
        assert_eq!(packets[3].data, &[0xA, 0xB]);
        assert!(matches!(items.last(), Some(Item::Unsynced { words, .. }) if words.len() == 2));
    }

    #[test]
    fn truncated_and_orphan() {
        assert!(matches!(walk(&[SYNC, 0x3000_4005, 1]), Err(Error::Truncated { .. })));
        assert!(matches!(walk(&[SYNC, 0x5000_0001, 1]), Err(Error::OrphanType2 { .. })));
    }
}
