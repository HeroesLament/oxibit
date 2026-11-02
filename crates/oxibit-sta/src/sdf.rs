//! Cell timing from prjxray's SDF files (`timings/*.sdf` in a family's
//! database): per cell type, the delay arcs (`IOPATH`) and the timing
//! checks (`SETUP`, `HOLD`, `RECOVERY`, `REMOVAL`) against a clock edge.
//!
//! prjxray writes SDF 3.0 with `TIMESCALE 1ns` and every value as
//! `(min::max)`, one triple per transition (rise, then fall). A [`Delay`]
//! keeps both corners in ps: `min` the smaller of the two transitions'
//! minimums, `max` the larger of their maximums, the value a setup
//! analysis at the slow corner uses.

use std::collections::BTreeMap;
use std::path::Path;

/// Both corners of one value, ps. Checks may be negative.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Delay {
    pub min: i32,
    pub max: i32,
}

/// A combinational or clock-to-output arc.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Arc {
    pub from: String,
    pub to: String,
    pub delay: Delay,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum CheckKind {
    Setup,
    Hold,
    Recovery,
    Removal,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Edge {
    Rise,
    Fall,
}

/// A data pin checked against a clock edge.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Check {
    pub kind: CheckKind,
    pub pin: String,
    pub clock: String,
    pub edge: Edge,
    pub value: Delay,
}

/// One `CELL` block: a cell type at an instance path (`SLICEL`,
/// `SLICEL/A6LUT`), with its arcs and checks.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Cell {
    pub celltype: String,
    pub instance: String,
    pub arcs: Vec<Arc>,
    pub checks: Vec<Check>,
}

impl Cell {
    /// The arc from `from` to `to`, if the cell has one.
    pub fn arc(&self, from: &str, to: &str) -> Option<Delay> {
        self.arcs
            .iter()
            .find(|a| a.from == from && a.to == to)
            .map(|a| a.delay)
    }

    /// The check of `kind` on `pin`, if the cell has one.
    pub fn check(&self, kind: CheckKind, pin: &str) -> Option<&Check> {
        self.checks.iter().find(|c| c.kind == kind && c.pin == pin)
    }
}

/// Every cell of every SDF file read, by (file stem, cell type, instance).
#[derive(Debug, Default)]
pub struct Library {
    cells: BTreeMap<(String, String, String), Cell>,
}

impl Library {
    /// Every `*.sdf` in `dir` (a database's `timings`).
    pub fn load_dir(dir: &Path) -> Result<Library, String> {
        let mut lib = Library::default();
        let rd = std::fs::read_dir(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
        let mut paths: Vec<_> = rd
            .filter_map(|e| e.ok().map(|e| e.path()))
            .filter(|p| p.extension().is_some_and(|x| x == "sdf"))
            .collect();
        paths.sort();
        for p in paths {
            let stem = p
                .file_stem()
                .and_then(|s| s.to_str())
                .unwrap_or_default()
                .to_string();
            let text = std::fs::read_to_string(&p).map_err(|e| format!("{}: {e}", p.display()))?;
            for c in parse(&text).map_err(|e| format!("{}: {e}", p.display()))? {
                lib.cells
                    .insert((stem.clone(), c.celltype.clone(), c.instance.clone()), c);
            }
        }
        Ok(lib)
    }

    pub fn len(&self) -> usize {
        self.cells.len()
    }

    pub fn is_empty(&self) -> bool {
        self.cells.is_empty()
    }

    /// The cell `celltype` at `instance` in file `file` (stem, `slicel`).
    pub fn get(&self, file: &str, celltype: &str, instance: &str) -> Option<&Cell> {
        self.cells
            .get(&(file.to_string(), celltype.to_string(), instance.to_string()))
    }

    pub fn cells(&self) -> impl Iterator<Item = (&(String, String, String), &Cell)> {
        self.cells.iter()
    }
}

#[derive(Debug, Clone, PartialEq)]
enum Sx {
    Atom(String),
    List(Vec<Sx>),
}

impl Sx {
    fn head(&self) -> Option<&str> {
        match self {
            Sx::List(v) => match v.first() {
                Some(Sx::Atom(a)) => Some(a.as_str()),
                _ => None,
            },
            Sx::Atom(_) => None,
        }
    }
    fn items(&self) -> &[Sx] {
        match self {
            Sx::List(v) => v,
            Sx::Atom(_) => &[],
        }
    }
    fn atom(&self) -> Option<&str> {
        match self {
            Sx::Atom(a) => Some(a.as_str()),
            Sx::List(_) => None,
        }
    }
}

fn read(text: &str) -> Result<Sx, String> {
    let mut stack: Vec<Vec<Sx>> = vec![Vec::new()];
    let mut chars = text.char_indices().peekable();
    while let Some((i, ch)) = chars.next() {
        match ch {
            '(' => stack.push(Vec::new()),
            ')' => {
                let done = stack
                    .pop()
                    .ok_or_else(|| format!("unbalanced ')' at byte {i}"))?;
                stack
                    .last_mut()
                    .ok_or_else(|| format!("unbalanced ')' at byte {i}"))?
                    .push(Sx::List(done));
            }
            '"' => {
                let mut s = String::new();
                loop {
                    match chars.next() {
                        Some((_, '"')) => break,
                        Some((_, c)) => s.push(c),
                        None => return Err("unterminated string".into()),
                    }
                }
                stack.last_mut().expect("stack").push(Sx::Atom(s));
            }
            c if c.is_whitespace() => {}
            _ => {
                let mut s = String::from(ch);
                while let Some(&(_, c)) = chars.peek() {
                    if c.is_whitespace() || c == '(' || c == ')' || c == '"' {
                        break;
                    }
                    s.push(c);
                    chars.next();
                }
                stack.last_mut().expect("stack").push(Sx::Atom(s));
            }
        }
    }
    if stack.len() != 1 {
        return Err(format!("{} unclosed '('", stack.len() - 1));
    }
    let mut top = stack.pop().expect("top");
    if top.len() != 1 {
        return Err(format!("{} top-level forms, expected 1", top.len()));
    }
    Ok(top.pop().expect("one"))
}

/// `(min::max)` or `(min:typ:max)` -> (min, max) ps, with `scale` ps per
/// unit. A `()` value is absent.
fn triple(sx: &Sx, scale: f64) -> Result<Option<(i32, i32)>, String> {
    let items = sx.items();
    let Some(text) = items.first().and_then(Sx::atom) else {
        return Ok(None);
    };
    let parts: Vec<&str> = text.split(':').collect();
    let num = |s: &str| -> Result<Option<f64>, String> {
        if s.is_empty() {
            Ok(None)
        } else {
            s.parse::<f64>()
                .map(Some)
                .map_err(|_| format!("bad value {text}"))
        }
    };
    let (lo, hi) = match parts.as_slice() {
        [v] => (num(v)?, num(v)?),
        [a, _, c] => (num(a)?, num(c)?),
        _ => return Err(format!("bad value {text}")),
    };
    let lo = lo.or(hi);
    let hi = hi.or(lo);
    Ok(match (lo, hi) {
        (Some(a), Some(b)) => Some(((a * scale).round() as i32, (b * scale).round() as i32)),
        _ => None,
    })
}

/// Every transition's triple, folded to one corner pair.
fn delay(values: &[Sx], scale: f64) -> Result<Delay, String> {
    let mut d: Option<Delay> = None;
    for v in values {
        if let Some((lo, hi)) = triple(v, scale)? {
            d = Some(match d {
                None => Delay { min: lo, max: hi },
                Some(o) => Delay {
                    min: o.min.min(lo),
                    max: o.max.max(hi),
                },
            });
        }
    }
    d.ok_or_else(|| "no value".to_string())
}

/// ps per SDF time unit, from `(TIMESCALE 1ns)`-style text.
fn scale_of(ts: &str) -> Result<f64, String> {
    let t = ts.trim();
    let split = t.find(|c: char| c.is_alphabetic()).unwrap_or(t.len());
    let (n, unit) = t.split_at(split);
    let n: f64 = if n.trim().is_empty() {
        1.0
    } else {
        n.trim()
            .parse()
            .map_err(|_| format!("bad TIMESCALE {ts}"))?
    };
    let per = match unit.trim() {
        "ps" => 1.0,
        "ns" => 1000.0,
        "us" => 1_000_000.0,
        u => return Err(format!("TIMESCALE unit {u}")),
    };
    Ok(n * per)
}

/// The cells of one SDF file.
pub fn parse(text: &str) -> Result<Vec<Cell>, String> {
    let top = read(text)?;
    if top.head() != Some("DELAYFILE") {
        return Err("not a DELAYFILE".into());
    }
    let mut scale = 1000.0;
    let mut cells = Vec::new();
    for form in &top.items()[1..] {
        match form.head() {
            Some("TIMESCALE") => {
                let t: Vec<&str> = form.items()[1..].iter().filter_map(Sx::atom).collect();
                scale = scale_of(&t.join(""))?;
            }
            Some("CELL") => cells.push(cell(form, scale)?),
            _ => {}
        }
    }
    Ok(cells)
}

fn cell(form: &Sx, scale: f64) -> Result<Cell, String> {
    let mut c = Cell::default();
    for f in &form.items()[1..] {
        match f.head() {
            Some("CELLTYPE") => {
                c.celltype = f
                    .items()
                    .get(1)
                    .and_then(Sx::atom)
                    .unwrap_or_default()
                    .to_string()
            }
            Some("INSTANCE") => {
                c.instance = f
                    .items()
                    .get(1)
                    .and_then(Sx::atom)
                    .unwrap_or_default()
                    .to_string()
            }
            Some("DELAY") => {
                for abs in &f.items()[1..] {
                    for p in &abs.items()[1..] {
                        if p.head() == Some("IOPATH") {
                            let it = p.items();
                            let from =
                                it.get(1).and_then(Sx::atom).ok_or("IOPATH without input")?;
                            let to = it
                                .get(2)
                                .and_then(Sx::atom)
                                .ok_or("IOPATH without output")?;
                            c.arcs.push(Arc {
                                from: from.into(),
                                to: to.into(),
                                delay: delay(&it[3..], scale)?,
                            });
                        }
                    }
                }
            }
            Some("TIMINGCHECK") => {
                for t in &f.items()[1..] {
                    let kind = match t.head() {
                        Some("SETUP") => CheckKind::Setup,
                        Some("HOLD") => CheckKind::Hold,
                        Some("RECOVERY") => CheckKind::Recovery,
                        Some("REMOVAL") => CheckKind::Removal,
                        _ => continue,
                    };
                    let it = t.items();
                    let pin = it.get(1).and_then(Sx::atom).ok_or("check without pin")?;
                    let (edge, clock) = match it.get(2) {
                        Some(e @ Sx::List(_)) => {
                            let edge = match e.head() {
                                Some("posedge") => Edge::Rise,
                                Some("negedge") => Edge::Fall,
                                h => return Err(format!("check edge {h:?}")),
                            };
                            (
                                edge,
                                e.items()
                                    .get(1)
                                    .and_then(Sx::atom)
                                    .ok_or("check without clock")?,
                            )
                        }
                        Some(Sx::Atom(a)) => (Edge::Rise, a.as_str()),
                        None => return Err("check without clock".into()),
                    };
                    c.checks.push(Check {
                        kind,
                        pin: pin.into(),
                        clock: clock.into(),
                        edge,
                        value: delay(&it[3..], scale)?,
                    });
                }
            }
            _ => {}
        }
    }
    Ok(c)
}

#[cfg(test)]
mod tests {
    use super::*;

    const SNIPPET: &str = r#"(DELAYFILE
    (SDFVERSION "3.0")
    (TIMESCALE 1ns)
    (CELL
        (CELLTYPE "FF_INIT")
        (INSTANCE SLICEL)
        (DELAY
            (ABSOLUTE
                (IOPATH CLK Q (0.099::0.124)(0.244::0.303))
            )
        )
        (TIMINGCHECK
                (HOLD CE (posedge CLK) (-0.011::-0.009))
                (SETUP DIN (posedge CLK) (-0.057::-0.046))
        )
    )
    (CELL
        (CELLTYPE "LUT6")
        (INSTANCE SLICEL/A6LUT)
        (DELAY (ABSOLUTE (IOPATH A1 O6 (0.045::0.056)(0.1::0.124))))
    )
)"#;

    #[test]
    fn reads_arcs_and_checks() {
        let cells = parse(SNIPPET).unwrap();
        assert_eq!(cells.len(), 2);
        let ff = &cells[0];
        assert_eq!(
            (ff.celltype.as_str(), ff.instance.as_str()),
            ("FF_INIT", "SLICEL")
        );
        assert_eq!(ff.arc("CLK", "Q"), Some(Delay { min: 99, max: 303 }));
        let su = ff.check(CheckKind::Setup, "DIN").unwrap();
        assert_eq!(
            (su.clock.as_str(), su.edge, su.value),
            ("CLK", Edge::Rise, Delay { min: -57, max: -46 })
        );
        assert_eq!(
            ff.check(CheckKind::Hold, "CE").unwrap().value,
            Delay { min: -11, max: -9 }
        );
        assert_eq!(cells[1].arc("A1", "O6"), Some(Delay { min: 45, max: 124 }));
    }

    #[test]
    fn rejects_broken_text() {
        assert!(parse("(DELAYFILE (CELL").is_err());
        assert!(parse("(NOTSDF)").is_err());
    }
}
