//! Per-chip calibration file (`docs/timing.md`).
//!
//! One JSON file per chip. Every section is optional, so a file grows as
//! measurements are added; the schema number changes only when a field's
//! meaning does, and a reader refuses a schema it does not know.

use std::fs;
use std::path::Path;

use serde::{Deserialize, Serialize};

/// Schema of the calibration file.
pub const SCHEMA: u32 = 1;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Calibration {
    pub schema: u32,
    /// Farm name of the unit the measurements came from.
    pub unit: String,
    /// Device DNA, hex, once the probe designs read it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dna: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub taps: Option<Taps>,
}

/// MMCM phase-tap spacing, as fractions of one 1/8-VCO phase step.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Taps {
    /// Capture output (CLKOUT1): position error of tap `j`, summing to 0.
    /// What a sweep point at step `k` is corrected by: `capture[k mod 8] * step_ps`.
    pub capture: [f64; 8],
    /// Launch output (CLKOUT0), relative to tap 0 (`launch[0] = 0`).
    pub launch: [f64; 8],
    /// Designs the fit used.
    pub designs: Vec<String>,
    pub edges: usize,
    pub points: usize,
    /// Residual sum of squares of the fit.
    pub sse: f64,
    /// Median fitted edge width, ps (below ~10 ps the spacing is only bracketed).
    pub sigma_median: f64,
}

impl Calibration {
    pub fn new(unit: &str) -> Self {
        Calibration { schema: SCHEMA, unit: unit.to_string(), dna: None, taps: None }
    }

    pub fn load(path: &Path) -> Result<Self, String> {
        let text = fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
        let c: Calibration = serde_json::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))?;
        if c.schema != SCHEMA {
            return Err(format!("{}: calibration schema {}, this oxibit reads {SCHEMA}", path.display(), c.schema));
        }
        Ok(c)
    }

    /// The file at `path` if it exists (it must be this unit's), else a new one.
    pub fn load_or_new(path: &Path, unit: &str) -> Result<Self, String> {
        if !path.exists() {
            return Ok(Self::new(unit));
        }
        let c = Self::load(path)?;
        if c.unit != unit {
            return Err(format!("{} belongs to unit {}, not {unit}", path.display(), c.unit));
        }
        Ok(c)
    }

    pub fn save(&self, path: &Path) -> Result<(), String> {
        let text = serde_json::to_string_pretty(self).map_err(|e| e.to_string())?;
        fs::write(path, text + "\n").map_err(|e| format!("{}: {e}", path.display()))
    }

    /// Capture-tap corrections for edge fits; nominal without a tap section.
    pub fn capture_taps(&self) -> [f64; 8] {
        self.taps.as_ref().map_or([0.0; 8], |t| t.capture)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_and_schema_check() {
        let dir = std::env::temp_dir().join(format!("oxt-calib-{}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        let path = dir.join("dr-b.timing.json");
        let mut c = Calibration::new("dr-b");
        c.taps = Some(Taps {
            capture: [0.03, 0.02, -0.03, -0.1, -0.01, 0.07, -0.05, 0.07],
            launch: [0.0; 8],
            designs: vec!["t3s5".into()],
            edges: 30,
            points: 2376,
            sse: 11.5,
            sigma_median: 15.0,
        });
        c.save(&path).unwrap();
        assert_eq!(Calibration::load(&path).unwrap(), c);
        assert_eq!(Calibration::load_or_new(&path, "dr-b").unwrap().capture_taps()[3], -0.1);
        assert!(Calibration::load_or_new(&path, "dr-a").is_err());
        fs::write(&path, r#"{"schema":2,"unit":"dr-b"}"#).unwrap();
        assert!(Calibration::load(&path).unwrap_err().contains("schema 2"));
        assert_eq!(Calibration::load_or_new(&dir.join("none.json"), "dr-c").unwrap(), Calibration::new("dr-c"));
        fs::remove_dir_all(&dir).unwrap();
    }
}
