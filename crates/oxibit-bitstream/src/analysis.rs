//! Everything checkable about a bitstream without a device.

use serde::Serialize;
use sha2::{Digest, Sha256};

use crate::container::{BitHeader, Bitstream, Container};
use crate::crc::icap_crc;
use crate::packet::{self, Item, Opcode, Reg, cmd};
use crate::part::{self, Die};
use crate::{Error, FRAME_WORDS};

#[derive(Debug, Clone, Serialize)]
pub struct Report {
    pub container: Container,
    pub header: Option<BitHeader>,
    /// Configuration words, including pre-sync padding.
    pub words: usize,
    /// SHA-256 of the configuration content: the words from the first sync
    /// word through the end of the last packet, big-endian. Padding before the
    /// sync and after DESYNC is excluded, so one design hashes the same in a
    /// `.bit`, a raw or swapped `.bin`, and a bootgen partition.
    /// SHA-256 of the configuration words (first sync to the last packet);
    /// `None` when the caller skipped it (`analyze_with(.., false)`).
    pub config_sha256: Option<String>,
    /// Word index of every sync word.
    pub syncs: Vec<usize>,
    pub idcode: Option<u32>,
    /// Die named by the IDCODE the stream writes.
    pub die: Option<Die>,
    /// Die named by the `.bit` header's part record.
    pub header_die: Option<Die>,
    /// Words written to FDRI (frame data, including pad frames).
    pub fdri_words: usize,
    pub commands: Vec<String>,
    pub crc: CrcSummary,
    pub desync: bool,
    /// Vivado marks partial bitstreams with `PARTIAL=TRUE` in record `a`.
    pub partial_hint: bool,
    /// Anything that would make a device reject this, or configure the wrong thing.
    pub problems: Vec<String>,
    /// Oddities worth a look that do not by themselves mean failure.
    pub warnings: Vec<String>,
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct CrcSummary {
    /// Writes to the CRC register (each is a check the device performs).
    pub checks: usize,
    pub failures: Vec<CrcFailure>,
}

#[derive(Debug, Clone, Serialize)]
pub struct CrcFailure {
    /// Word index of the CRC packet header.
    pub index: usize,
    pub written: u32,
    pub computed: u32,
}

pub fn analyze(bs: &Bitstream) -> Result<Report, Error> {
    analyze_with(bs, true)
}

/// [`analyze`], optionally without the content hash: on a Cortex-A9 the
/// SHA-256 of a 4 MB bitstream costs more than every check combined, and a
/// caller that already knows what it is loading (a test farm that hashed it
/// on the host) does not need it recomputed.
pub fn analyze_with(bs: &Bitstream, hash: bool) -> Result<Report, Error> {
    let items = packet::walk(&bs.words)?;

    let mut syncs = Vec::new();
    let mut idcode = None;
    let mut fdri_words = 0;
    let mut commands = Vec::new();
    let mut desync = false;
    let mut problems = Vec::new();
    let mut warnings = Vec::new();

    let mut content_end = 0;
    for item in &items {
        if let Item::Packet(p) = item {
            content_end = content_end.max(p.index + 1 + p.data.len());
        }
        match item {
            Item::Sync { index } => syncs.push(*index),
            Item::Packet(p) if p.opcode == Opcode::Write => {
                if p.reg == Reg::IDCODE {
                    if let Some(&w) = p.data.first() {
                        if idcode.is_some_and(|prev| prev != w) {
                            problems.push(format!("conflicting IDCODE writes at word {}", p.index));
                        }
                        idcode = Some(w);
                    }
                }
                if p.reg == Reg::FDRI {
                    fdri_words += p.data.len();
                }
                if p.reg == Reg::CMD {
                    for &w in p.data {
                        commands.push(cmd::name(w).map(str::to_owned).unwrap_or(format!("0x{w:X}")));
                        desync |= w == cmd::DESYNC;
                    }
                }
            }
            Item::Packet(p) if p.opcode == Opcode::Reserved => {
                warnings.push(format!("reserved opcode at word {} (0x{:08X})", p.index, p.header));
            }
            _ => {}
        }
    }

    let crc = check_crc(&items);

    let die = idcode.and_then(part::by_idcode).copied();
    let header_die = bs.header.as_ref().and_then(|h| h.part.as_deref()).and_then(part::by_part).copied();

    match (idcode, die) {
        (None, _) => problems.push("no IDCODE write: the device will not check it is the right part".into()),
        (Some(id), None) => warnings.push(format!("IDCODE 0x{id:08X} is not in the die table")),
        _ => {}
    }
    if let (Some(d), Some(h)) = (die, header_die) {
        if d.name != h.name {
            problems.push(format!("header part is {} but the stream's IDCODE is {}", h.name, d.name));
        }
    }
    if let Some(part) = bs.header.as_ref().and_then(|h| h.part.as_deref()) {
        if header_die.is_none() {
            warnings.push(format!("header part {part:?} is not in the die table"));
        }
    }
    for f in &crc.failures {
        problems.push(format!(
            "CRC check at word {}: stream writes 0x{:08X}, computed 0x{:08X}",
            f.index, f.written, f.computed
        ));
    }
    if !desync {
        problems.push("no DESYNC command: the configuration logic is left listening".into());
    }
    if !commands.iter().any(|c| c == "START") {
        warnings.push("no START command: the device will not run the startup sequence".into());
    }
    if fdri_words == 0 {
        problems.push("no frame data (FDRI) written".into());
    } else if fdri_words % FRAME_WORDS != 0 {
        warnings.push(format!(
            "FDRI carries {fdri_words} words, not a whole number of {FRAME_WORDS}-word frames"
        ));
    }

    let partial_hint = bs
        .header
        .as_ref()
        .and_then(|h| h.design.as_deref())
        .is_some_and(|d| d.to_ascii_uppercase().contains("PARTIAL=TRUE"));

    let config_sha256 = hash.then(|| content_sha256(&bs.words, syncs.first().copied(), content_end));

    Ok(Report {
        container: bs.container,
        header: bs.header.clone(),
        words: bs.words.len(),
        config_sha256,
        syncs,
        idcode,
        die,
        header_die,
        fdri_words,
        commands,
        crc,
        desync,
        partial_hint,
        problems,
        warnings,
    })
}

/// Recompute the device's CRC and compare it at every write to the CRC
/// register.
///
/// Model: the CRC starts at zero at each sync word; every written word to any
/// register other than CRC is folded in with that register's address; a CMD
/// write of RCRC resets it to zero; a write to CRC is compared against the
/// running value and then resets it to zero.
fn check_crc(items: &[Item<'_>]) -> CrcSummary {
    let mut summary = CrcSummary::default();
    let mut crc = 0u32;

    for item in items {
        match item {
            Item::Sync { .. } => crc = 0,
            Item::Packet(p) if p.opcode == Opcode::Write => {
                for &w in p.data {
                    if p.reg == Reg::CRC {
                        summary.checks += 1;
                        if w != crc {
                            summary.failures.push(CrcFailure { index: p.index, written: w, computed: crc });
                        }
                        crc = 0;
                    } else {
                        crc = icap_crc(p.reg.0 as u32, w, crc);
                        if p.reg == Reg::CMD && w == cmd::RCRC {
                            crc = 0;
                        }
                    }
                }
            }
            _ => {}
        }
    }
    summary
}

fn content_sha256(words: &[u32], start: Option<usize>, end: usize) -> String {
    let start = start.unwrap_or(0);
    // One contiguous buffer, one update: per-word updates cost more than
    // the compression itself on a Cortex-A9.
    let span = &words[start..end.max(start)];
    let mut bytes = Vec::with_capacity(span.len() * 4);
    for w in span {
        bytes.extend_from_slice(&w.to_be_bytes());
    }
    hex(&Sha256::digest(&bytes))
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}
