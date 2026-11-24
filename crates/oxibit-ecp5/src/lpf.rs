//! Lattice preference files (`.lpf`): pin locations and I/O settings.
//!
//! Statements end at `;`; `#` and `//` start comments. Read: `LOCATE COMP
//! "<port>" SITE "<ball>"` and `IOBUF PORT "<port>" <KEY>=<VALUE> ...`
//! (IO_TYPE, DRIVE, PULLMODE, SLEWRATE, OPENDRAIN, HYSTERESIS). Everything
//! else (FREQUENCY, BLOCK, SYSCONFIG, ...) is kept as a count of ignored
//! statements. Port names are as the design has them: `led[3]`.

use std::collections::BTreeMap;

use crate::{Result, err};

/// One port's preferences.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct PortPrefs {
    pub site: Option<String>,
    /// IOBUF keys, upper case, as written.
    pub iobuf: BTreeMap<String, String>,
}

#[derive(Clone, Debug, Default)]
pub struct Lpf {
    pub ports: BTreeMap<String, PortPrefs>,
    /// `SYSCONFIG KEY=VALUE ...` settings (nextpnr's keys).
    pub sysconfig: BTreeMap<String, String>,
    pub ignored: usize,
}

/// The SYSCONFIG keys nextpnr accepts.
pub const SYSCONFIG_KEYS: &[&str] = &[
    "SLAVE_SPI_PORT", "MASTER_SPI_PORT", "SLAVE_PARALLEL_PORT", "BACKGROUND_RECONFIG", "DONE_EX", "DONE_OD",
    "DONE_PULL", "MCCLK_FREQ", "TRANSFR", "CONFIG_IOVOLTAGE", "CONFIG_SECURE", "WAKE_UP", "COMPRESS_CONFIG",
    "CONFIG_MODE", "INBUF",
];

/// Strip comments, keeping quoted text.
fn strip(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for line in text.lines() {
        let mut in_q = false;
        let b = line.as_bytes();
        let mut end = line.len();
        let mut i = 0;
        while i < b.len() {
            match b[i] {
                b'"' => in_q = !in_q,
                b'#' if !in_q => {
                    end = i;
                    break;
                }
                b'/' if !in_q && b.get(i + 1) == Some(&b'/') => {
                    end = i;
                    break;
                }
                _ => {}
            }
            i += 1;
        }
        out.push_str(&line[..end]);
        out.push('\n');
    }
    out
}

/// Split a statement into words, a quoted string being one word (quotes
/// removed).
fn words(stmt: &str) -> Vec<String> {
    let mut v = Vec::new();
    let mut cur = String::new();
    let mut in_q = false;
    for ch in stmt.chars() {
        match ch {
            '"' => {
                in_q = !in_q;
                if !in_q {
                    v.push(std::mem::take(&mut cur));
                }
            }
            c if c.is_whitespace() && !in_q => {
                if !cur.is_empty() {
                    v.push(std::mem::take(&mut cur));
                }
            }
            c => cur.push(c),
        }
    }
    if !cur.is_empty() {
        v.push(cur);
    }
    v
}

impl Lpf {
    pub fn parse(text: &str) -> Result<Lpf> {
        let mut lpf = Lpf::default();
        for stmt in strip(text).split(';') {
            let w = words(stmt);
            if w.is_empty() {
                continue;
            }
            let up: Vec<String> = w.iter().map(|s| s.to_ascii_uppercase()).collect();
            match up.first().map(String::as_str) {
                Some("LOCATE") => {
                    // LOCATE COMP "name" SITE "ball"
                    if up.len() < 5 || up[1] != "COMP" || up[3] != "SITE" {
                        return Err(err!("lpf: bad LOCATE: {}", stmt.trim()));
                    }
                    lpf.ports.entry(w[2].clone()).or_default().site = Some(w[4].clone());
                }
                Some("IOBUF") => {
                    // IOBUF PORT "name" KEY=VALUE ...
                    if up.len() < 3 || up[1] != "PORT" {
                        return Err(err!("lpf: bad IOBUF: {}", stmt.trim()));
                    }
                    let p = lpf.ports.entry(w[2].clone()).or_default();
                    for kv in &w[3..] {
                        let (k, v) = kv
                            .split_once('=')
                            .ok_or_else(|| err!("lpf: bad IOBUF setting {kv}"))?;
                        p.iobuf.insert(k.trim().to_ascii_uppercase(), v.trim().to_owned());
                    }
                }
                Some("SYSCONFIG") => {
                    for kv in &w[1..] {
                        let (k, v) = kv
                            .split_once('=')
                            .ok_or_else(|| err!("lpf: bad SYSCONFIG setting {kv} (KEY=VALUE)"))?;
                        let k = k.trim().to_ascii_uppercase();
                        if !SYSCONFIG_KEYS.contains(&k.as_str()) {
                            return Err(err!("lpf: unexpected SYSCONFIG key {k}"));
                        }
                        lpf.sysconfig.insert(k, v.trim().to_owned());
                    }
                }
                _ => lpf.ignored += 1,
            }
        }
        Ok(lpf)
    }

    /// The I/O standard of a port (`IO_TYPE`, default LVCMOS33).
    /// Preferences for a netlist port. A one-bit vector `btn[0:0]` and a
    /// scalar `btn` both reach the netlist as `btn`, so fall back to `btn[0]`.
    #[must_use]
    pub fn port(&self, name: &str) -> Option<&PortPrefs> {
        self.ports
            .get(name)
            .or_else(|| self.ports.get(&format!("{name}[0]")))
    }

    pub fn io_type(&self, port: &str) -> String {
        self.port(port)
            .and_then(|p| p.iobuf.get("IO_TYPE"))
            .cloned()
            .unwrap_or_else(|| "LVCMOS33".to_owned())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ulx3s_lines() {
        let t = r#"BLOCK RESETPATHS;
LOCATE COMP "clk_25mhz" SITE "G2";
IOBUF  PORT "clk_25mhz" PULLMODE=NONE IO_TYPE=LVCMOS33;
FREQUENCY PORT "clk_25mhz" 25 MHZ;
LOCATE COMP "led[0]" SITE "B2"; # LED
IOBUF  PORT "led[0]" PULLMODE=NONE IO_TYPE=LVCMOS33 DRIVE=4;
"#;
        let l = Lpf::parse(t).unwrap();
        assert_eq!(l.ports["clk_25mhz"].site.as_deref(), Some("G2"));
        assert_eq!(l.ports["led[0]"].iobuf["DRIVE"], "4");
        assert_eq!(l.io_type("led[0]"), "LVCMOS33");
        assert_eq!(l.ignored, 2);
    }
}
