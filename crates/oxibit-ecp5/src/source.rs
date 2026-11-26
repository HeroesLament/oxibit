//! Where the ECP5 chip database comes from.
//!
//! [`Source::Text`] reads a Project Trellis checkout (`devices.json`,
//! `ECP5/tiledata`, per-device `tilegrid.json`, `globals.json`, `iodb.json`),
//! and is the default. [`Source::Builtin`] uses the data compiled into
//! `oxibit-arch-ecp5`, so no external files are needed. Both produce the same
//! [`DeviceDb`] and the same `globals.json` / `iodb.json` values; the readers
//! downstream do not know which one they got.

use std::fs;
use std::path::{Path, PathBuf};

use oxibit_arch::lit::DeviceLit;
use serde_json::{Map, Value, json};

use crate::{Result, err};

/// The chip database in use.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Source {
    /// A Project Trellis database directory.
    Text(PathBuf),
    /// The data checked in as `oxibit-arch-ecp5`.
    Builtin,
}

impl Source {
    /// A Trellis database directory.
    pub fn text(root: impl Into<PathBuf>) -> Source {
        Source::Text(root.into())
    }

    /// For messages: the directory, or `built-in`.
    pub fn describe(&self) -> String {
        match self {
            Source::Text(p) => p.display().to_string(),
            Source::Builtin => "built-in".to_owned(),
        }
    }

    /// `ECP5/<device>/iodb.json`, parsed or synthesised.
    pub fn iodb(&self, device: &str) -> Result<Value> {
        match self {
            Source::Text(root) => read_json(&root.join("ECP5").join(device).join("iodb.json")),
            Source::Builtin => Ok(iodb_value(builtin(device)?)),
        }
    }

    /// `ECP5/<device>/globals.json`, parsed or synthesised.
    pub fn globals(&self, device: &str) -> Result<Value> {
        match self {
            Source::Text(root) => read_json(&root.join("ECP5").join(device).join("globals.json")),
            Source::Builtin => Ok(globals_value(builtin(device)?)),
        }
    }
}

/// A built-in device by name.
pub(crate) fn builtin(device: &str) -> Result<&'static DeviceLit> {
    oxibit_arch_ecp5::device(device).ok_or_else(|| {
        let known: Vec<&str> = oxibit_arch_ecp5::devices::ALL.iter().map(|d| d.name).collect();
        err!("device {device} is not built in (built in: {})", known.join(", "))
    })
}

fn read_json(path: &Path) -> Result<Value> {
    let text = fs::read_to_string(path).map_err(|e| err!("{}: {e}", path.display()))?;
    serde_json::from_str(&text).map_err(|e| err!("{}: {e}", path.display()))
}

/// The `iodb.json` of a device.
pub fn iodb_value(d: &DeviceLit) -> Value {
    let mut packages = Map::new();
    for p in d.packages {
        let mut pins = Map::new();
        for pin in p.pins {
            pins.insert(pin.name.to_owned(), json!({"col": pin.col, "row": pin.row, "pio": pin.pio}));
        }
        packages.insert(p.name.to_owned(), Value::Object(pins));
    }
    let meta: Vec<Value> = d
        .pio_meta
        .iter()
        .map(|m| {
            let mut o = Map::new();
            o.insert("bank".into(), json!(m.bank));
            o.insert("col".into(), json!(m.col));
            o.insert("row".into(), json!(m.row));
            o.insert("pio".into(), json!(m.pio));
            if let Some(v) = m.dqs {
                o.insert("dqs".into(), json!(v));
            }
            if let Some(v) = m.function {
                o.insert("function".into(), json!(v));
            }
            Value::Object(o)
        })
        .collect();
    json!({"packages": packages, "pio_metadata": meta})
}

/// The `globals.json` of a device.
pub fn globals_value(d: &DeviceLit) -> Value {
    let mut quadrants = Map::new();
    for q in d.quadrants {
        quadrants.insert(q.name.to_owned(), json!({"x0": q.x0, "y0": q.y0, "x1": q.x1, "y1": q.y1}));
    }
    let mut taps = Map::new();
    for t in d.taps {
        taps.insert(t.name.to_owned(), json!({"lx0": t.lx0, "lx1": t.lx1, "rx0": t.rx0, "rx1": t.rx1}));
    }
    let mut spines = Map::new();
    for s in d.spines {
        spines.insert(s.name.to_owned(), json!({"x": s.x, "y": s.y}));
    }
    json!({"quadrants": quadrants, "taps": taps, "spines": spines})
}
