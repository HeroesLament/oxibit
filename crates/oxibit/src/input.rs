//! Reading bitstreams from files, whatever container they are in.

use std::io::Read;
use std::path::Path;

use oxibit_bitstream::Bitstream;
use oxibit_bitstream::zynq_boot::{self, BootImage, Destination};

pub type Result<T> = std::result::Result<T, String>;

/// Read a file, transparently gunzipping `.gz` content.
pub fn read_file(path: &Path) -> Result<Vec<u8>> {
    let raw = std::fs::read(path).map_err(|e| format!("{}: {e}", path.display()))?;
    if !raw.starts_with(&[0x1F, 0x8B]) {
        return Ok(raw);
    }
    let mut out = Vec::new();
    flate2::read::GzDecoder::new(&raw[..])
        .read_to_end(&mut out)
        .map_err(|e| format!("{}: gunzip: {e}", path.display()))?;
    Ok(out)
}

/// One bitstream found in a file, with a label saying where it came from.
pub struct Found {
    pub label: String,
    pub bitstream: Bitstream,
}

/// Every bitstream in a file: the file itself, or the PL partitions of a
/// Zynq boot image.
pub fn load(path: &Path) -> Result<(Option<BootImage>, Vec<Found>)> {
    let data = read_file(path)?;
    let at = |e: &dyn std::fmt::Display| format!("{}: {e}", path.display());

    if !zynq_boot::is_boot_image(&data) {
        let bitstream = Bitstream::parse(&data).map_err(|e| at(&e))?;
        return Ok((None, vec![Found { label: path.display().to_string(), bitstream }]));
    }

    let boot = zynq_boot::parse(&data).map_err(|e| at(&e))?;
    let mut found = Vec::new();
    for p in boot.partitions.iter().filter(|p| p.destination == Destination::Pl) {
        let label = format!("{}#{}:{}", path.display(), p.index, p.name.as_deref().unwrap_or("?"));
        if p.encrypted {
            return Err(format!("{label}: partition is encrypted"));
        }
        let bitstream = Bitstream::parse(p.data(&data)).map_err(|e| format!("{label}: {e}"))?;
        found.push(Found { label, bitstream });
    }
    Ok((Some(boot), found))
}


/// Exactly one bitstream from `path`.
#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
pub fn load_one(path: &Path) -> Result<Found> {
    let (_, mut found) = load(path)?;
    match found.len() {
        1 => Ok(found.remove(0)),
        n => Err(format!("{}: expected one bitstream, found {n}", path.display())),
    }
}
