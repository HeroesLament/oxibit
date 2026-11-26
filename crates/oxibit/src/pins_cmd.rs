//! `oxibit pins`: the ports a board's constraints locate, and where.
//!
//! A design's top-level port binds to a pin by name, so these names are what
//! a design may call its ports. Reads the same constraints `build` would:
//! the board's built-in file, or `--constraints` (`.lpf` or `.xdc`).

use std::collections::BTreeMap;
use std::path::Path;
use std::process::ExitCode;

use oxibit_ecp5::lpf::{Lpf, base_name};

use crate::input::Result;

/// One located port bit.
struct Pin {
    site: String,
    io: String,
    pull: String,
    drive: String,
}

/// A scalar port, or a vector under its base name (`led[7:0]`), bits high
/// to low.
struct Row {
    name: String,
    sites: Vec<String>,
    io: String,
    pull: String,
    drive: String,
}

/// Where the constraints come from.
pub enum Source<'a> {
    /// A board's built-in constraints (text and format) and the board's name.
    BuiltIn { board: &'a str, text: &'a str, format: Format },
    /// A `.lpf`, `.pdc` or `.xdc` file.
    File(&'a Path),
}

/// A constraint file format: ECP5 LPF, Nexus PDC, or 7-series XDC.
#[derive(Clone, Copy)]
pub enum Format {
    Lpf,
    Pdc,
    Xdc,
}

impl Format {
    fn of_path(path: &Path) -> Format {
        match path.extension().and_then(|e| e.to_str()).map(str::to_ascii_lowercase).as_deref() {
            Some("xdc") => Format::Xdc,
            Some("pdc") => Format::Pdc,
            _ => Format::Lpf,
        }
    }

    fn parse(self, text: &str) -> Result<BTreeMap<String, Pin>> {
        match self {
            Format::Lpf => from_lpf(text),
            Format::Pdc => from_pdc(text),
            Format::Xdc => Ok(from_xdc(text)),
        }
    }
}

pub fn pins(source: &Source<'_>, filter: Option<&str>, json: bool) -> Result<ExitCode> {
    let (pins, origin) = match source {
        Source::BuiltIn { board, text, format } => (format.parse(text)?, format!("{board}, built in")),
        Source::File(path) => {
            let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
            let pins = Format::of_path(path).parse(&text)?;
            (pins, path.display().to_string())
        }
    };

    let needle = filter.map(str::to_ascii_lowercase);
    let rows: Vec<Row> = rows(pins)
        .into_iter()
        .filter(|r| needle.as_ref().is_none_or(|n| r.name.to_ascii_lowercase().contains(n)))
        .collect();

    if json {
        let out: Vec<serde_json::Value> = rows
            .iter()
            .map(|r| serde_json::json!({"port": r.name, "sites": r.sites, "io": r.io, "pull": r.pull, "drive": r.drive}))
            .collect();
        println!("{}", serde_json::Value::Array(out));
        return Ok(ExitCode::SUCCESS);
    }

    let width = rows.iter().map(|r| r.name.len()).max().unwrap_or(0).max("PORT".len());
    println!("{:width$}  {:10}  {:5}  {:5}  PINS", "PORT", "IO", "PULL", "DRIVE");
    for r in &rows {
        println!("{:width$}  {:10}  {:5}  {:5}  {}", r.name, r.io, r.pull, r.drive, r.sites.join(" "));
    }
    let bits: usize = rows.iter().map(|r| r.sites.len()).sum();
    println!("{} ports, {bits} pins ({origin})", rows.len());
    Ok(ExitCode::SUCCESS)
}

fn or_dash(v: Option<&String>) -> String {
    v.cloned().unwrap_or_else(|| "-".to_owned())
}

fn from_lpf(text: &str) -> Result<BTreeMap<String, Pin>> {
    let lpf = Lpf::parse(text).map_err(|e| e.to_string())?;
    let mut pins = BTreeMap::new();
    for (name, prefs) in &lpf.ports {
        let Some(site) = &prefs.site else { continue };
        let pin = Pin {
            site: site.clone(),
            io: lpf.io_type(name),
            pull: or_dash(prefs.iobuf.get("PULLMODE")),
            drive: or_dash(prefs.iobuf.get("DRIVE")),
        };
        pins.insert(name.clone(), pin);
    }
    Ok(pins)
}

/// A Nexus PDC locates ports only; it carries no IO type, pull or drive.
fn from_pdc(text: &str) -> Result<BTreeMap<String, Pin>> {
    let sites = oxibit_nexus::flow::parse_pdc(text).map_err(|e| e.to_string())?;
    let dash = || "-".to_owned();
    Ok(sites.into_iter().map(|(port, site)| (port, Pin { site, io: dash(), pull: dash(), drive: dash() })).collect())
}

fn from_xdc(text: &str) -> BTreeMap<String, Pin> {
    let mut pins = BTreeMap::new();
    for (name, props) in oxibit_place::xdc::parse(text) {
        let Some(site) = props.get("PACKAGE_PIN") else { continue };
        let pin = Pin {
            site: site.clone(),
            io: or_dash(props.get("IOSTANDARD")),
            pull: or_dash(props.get("PULLTYPE")),
            drive: or_dash(props.get("DRIVE")),
        };
        pins.insert(name, pin);
    }
    pins
}

/// The bit index of `led[3]`.
fn index(port: &str) -> Option<u32> {
    port.strip_suffix(']')?.rsplit_once('[')?.1.parse().ok()
}

/// One value if every bit agrees, else `mixed`.
fn agreed<'a>(mut values: impl Iterator<Item = &'a str>) -> String {
    let first = values.next().unwrap_or("-");
    if values.all(|v| v == first) { first.to_owned() } else { "mixed".to_owned() }
}

/// Group bits into vectors. A vector whose indices do not run without a gap
/// stays as one row per bit.
fn rows(pins: BTreeMap<String, Pin>) -> Vec<Row> {
    let mut groups: BTreeMap<String, Vec<(Option<u32>, String, Pin)>> = BTreeMap::new();
    for (name, pin) in pins {
        groups.entry(base_name(&name).to_owned()).or_default().push((index(&name), name, pin));
    }

    let mut out = Vec::new();
    for (base, mut bits) in groups {
        bits.sort_by_key(|b| std::cmp::Reverse(b.0));
        let hi = bits.first().and_then(|b| b.0);
        let lo = bits.last().and_then(|b| b.0);
        let run = match (hi, lo) {
            (Some(hi), Some(lo)) => bits.len() > 1 && (hi - lo) as usize + 1 == bits.len(),
            _ => false,
        };
        if run {
            out.push(Row {
                name: format!("{base}[{}:{}]", hi.unwrap_or(0), lo.unwrap_or(0)),
                sites: bits.iter().map(|b| b.2.site.clone()).collect(),
                io: agreed(bits.iter().map(|b| b.2.io.as_str())),
                pull: agreed(bits.iter().map(|b| b.2.pull.as_str())),
                drive: agreed(bits.iter().map(|b| b.2.drive.as_str())),
            });
        } else {
            for (_, name, pin) in bits.into_iter().rev() {
                out.push(Row { name, sites: vec![pin.site], io: pin.io, pull: pin.pull, drive: pin.drive });
            }
        }
    }
    out
}
