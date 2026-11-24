//! Pad (PIO) and I/O bank settings, as nextpnr's `write_io` and
//! `init_io_banks` write them.
//!
//! Each pad gets `BASE_TYPE` `<DIR>_<IO_TYPE>`, then the LPF `IOBUF`
//! settings nextpnr applies: `HYSTERESIS` (ON by default on single-ended,
//! unreferenced inputs), `SLEWRATE`, `PULLMODE`, `DIFFRESISTOR`, `CLAMP`,
//! `DRIVE` (LVCMOS33 and LVCMOS33D only, as in Trellis), `TERMINATION`
//! (by VCCIO), `OPENDRAIN`.
//!
//! A differential pad is the A (or C) PIO of a pair. On the left and right
//! edges the partner (B, D) gets `PULLMODE NONE` and the pad too; a
//! `DRIVE` or `OPENDRAIN` goes on both. On the top edge the pair is
//! pseudo-differential and output only: the partner PIO's tiles (one
//! column east) get the same `PIOA.BASE_TYPE`, and a `DRIVE` lands on
//! `PIOA` and the partner's `PIOB`.
//!
//! Banks: an output, a bidir or a referenced input sets its bank's VCCIO
//! (from the I/O standard; 1V35 banks are written 1V2, as nextpnr does).
//! LVDS sets `DIFF_REF` and `LVDSO`, a differential input `DIFF_REF`, a
//! referenced input `DIFF_REF` and `VREF`, in every `BANKREF<n>` tile.

use std::collections::{BTreeMap, HashMap};
use std::path::Path;

use crate::config::TileConfig;
use crate::db::DeviceDb;
use crate::graph::Graph;
use crate::io::{PioSite, pio_enum, site_enum};
use crate::{Result, err};

/// One pad as the design uses it.
pub struct PadUse<'a> {
    pub name: &'a str,
    /// `INPUT`, `OUTPUT` or `BIDIR`.
    pub dir: &'static str,
    pub site: PioSite,
    /// The LPF `IOBUF` settings of the port.
    pub prefs: Option<&'a BTreeMap<String, String>>,
    /// The pad buffer's own attributes (PULLMODE of an IBPU), under `prefs`.
    pub attrs: Option<&'a BTreeMap<String, String>>,
}

/// The bank VCCIO an I/O standard needs (nextpnr `get_vccio`).
pub fn vccio(io: &str) -> Result<&'static str> {
    Ok(match io {
        "LVTTL33" | "LVCMOS33" | "LVCMOS33D" | "LVPECL33" | "LVPECL33E" => "3V3",
        "LVCMOS25" | "LVCMOS25D" | "LVDS" | "SLVS" | "SUBLVDS" | "LVDS25E" | "MLVDS25" | "MLVDS25E"
        | "BLVDS25" | "BLVDS25E" => "2V5",
        "LVCMOS18" | "LVCMOS18D" | "SSTL18_I" | "SSTL18_II" | "SSTL18D_I" | "SSTL18D_II" => "1V8",
        "LVCMOS15" | "LVCMOS15D" | "SSTL15_I" | "SSTL15_II" | "SSTL15D_I" | "SSTL15D_II" => "1V5",
        "SSTL135_I" | "SSTL135_II" | "SSTL135D_I" | "SSTL135D_II" => "1V35",
        "LVCMOS12" | "LVCMOS12D" | "HSUL12" | "HSUL12D" => "1V2",
        _ => return Err(err!("unknown IO_TYPE {io}")),
    })
}

/// A true or pseudo differential standard (nextpnr `is_differential`).
pub fn differential(io: &str) -> bool {
    matches!(
        io,
        "LVCMOS33D" | "LVCMOS25D" | "LVCMOS15D" | "LVCMOS12D" | "LVPECL33" | "LVDS" | "MLVDS25"
            | "BLVDS25" | "SLVS" | "SUBLVDS" | "LVCMOS18D" | "SSTL18D_I" | "SSTL18D_II" | "SSTL15D_I"
            | "SSTL15D_II" | "SSTL135D_I" | "SSTL135D_II" | "HSUL12D"
    )
}

/// A standard with a VREF input threshold (nextpnr `is_referenced`).
pub fn referenced(io: &str) -> bool {
    matches!(
        io,
        "SSTL18_I" | "SSTL18_II" | "SSTL18D_I" | "SSTL18D_II" | "SSTL15_I" | "SSTL15_II" | "SSTL15D_I"
            | "SSTL15D_II" | "SSTL135_I" | "SSTL135_II" | "SSTL135D_I" | "SSTL135D_II" | "HSUL12"
            | "HSUL12D"
    )
}

/// Each PIO's bank (`iodb.json` `pio_metadata`).
fn load_banks(root: &Path, device: &str) -> Result<HashMap<(i16, i16, char), u32>> {
    let path = root.join("ECP5").join(device).join("iodb.json");
    let text = std::fs::read_to_string(&path).map_err(|e| err!("{}: {e}", path.display()))?;
    let v: serde_json::Value = serde_json::from_str(&text).map_err(|e| err!("{}: {e}", path.display()))?;
    let mut out = HashMap::new();
    for p in v["pio_metadata"].as_array().ok_or(err!("iodb.json: pio_metadata"))? {
        let (Some(bank), Some(col), Some(row), Some(pio)) =
            (p["bank"].as_u64(), p["col"].as_i64(), p["row"].as_i64(), p["pio"].as_str().and_then(|s| s.chars().next()))
        else {
            continue;
        };
        let (Ok(bank), Ok(col), Ok(row)) = (u32::try_from(bank), i16::try_from(col), i16::try_from(row)) else {
            continue;
        };
        out.insert((row, col, pio), bank);
    }
    Ok(out)
}

/// The other PIO of a differential pair.
fn partner(p: char) -> Option<char> {
    match p {
        'A' => Some('B'),
        'C' => Some('D'),
        _ => None,
    }
}

#[derive(Default)]
struct Bank {
    vcc: Option<(&'static str, String)>,
    lvds: bool,
    diff: bool,
    vref: bool,
}

/// Write every pad's PIO settings and the banks' settings.
pub fn configure(
    g: &Graph,
    db: &DeviceDb,
    root: &Path,
    device: &str,
    pads: &[PadUse<'_>],
    tiles: &mut BTreeMap<String, TileConfig>,
) -> Result<()> {
    let bank_of = load_banks(root, device)?;
    let mut banks: BTreeMap<u32, Bank> = BTreeMap::new();
    for p in pads {
        let get = |k: &str| {
            p.prefs
                .and_then(|m| m.get(k))
                .or_else(|| p.attrs.and_then(|m| m.get(k)))
                .map(String::as_str)
        };
        let io = get("IO_TYPE").unwrap_or("LVCMOS33");
        let (diff, refd) = (differential(io), referenced(io));
        let site = p.site;
        let top = site.row == 0;
        let base = format!("{}_{io}", p.dir);
        let at = |e| err!("{} ({}{}): {e}", p.name, site.pio, site.col);
        pio_enum(g, db, site, "BASE_TYPE", &base, tiles).map_err(at)?;
        if diff {
            if top {
                if p.dir != "OUTPUT" {
                    return Err(err!("{}: pseudo-differential {io} on the top edge must be an output", p.name));
                }
                if site.pio != 'A' {
                    return Err(err!("{}: pseudo-differential {io} must be on the A pad of its pair", p.name));
                }
                let comp = PioSite { pio: 'B', ..site };
                site_enum(g, db, comp, "PIOA.BASE_TYPE", &base, tiles).map_err(at)?;
            } else {
                let other = partner(site.pio)
                    .ok_or_else(|| err!("{}: a differential pad must be the A or C PIO of a pair", p.name))?;
                pio_enum(g, db, PioSite { pio: other, ..site }, "PULLMODE", "NONE", tiles).map_err(at)?;
                pio_enum(g, db, site, "PULLMODE", "NONE", tiles).map_err(at)?;
            }
        } else if refd {
            pio_enum(g, db, site, "PULLMODE", "NONE", tiles).map_err(at)?;
        }
        if matches!(p.dir, "INPUT" | "BIDIR") && !diff && !refd {
            pio_enum(g, db, site, "HYSTERESIS", get("HYSTERESIS").unwrap_or("ON"), tiles).map_err(at)?;
        }
        if let Some(v) = get("SLEWRATE").filter(|_| !refd) {
            pio_enum(g, db, site, "SLEWRATE", v, tiles).map_err(at)?;
        }
        for key in ["PULLMODE", "DIFFRESISTOR", "CLAMP"] {
            if let Some(v) = get(key) {
                pio_enum(g, db, site, key, v, tiles).map_err(at)?;
            }
        }
        if let Some(v) = get("DRIVE") {
            match io {
                "LVCMOS33" => {
                    pio_enum(g, db, site, "DRIVE", v, tiles).map_err(at)?;
                }
                "LVCMOS33D" if top => {
                    pio_enum(g, db, site, "DRIVE", v, tiles).map_err(at)?;
                    site_enum(g, db, PioSite { pio: 'B', ..site }, "PIOB.DRIVE", v, tiles).map_err(at)?;
                }
                "LVCMOS33D" => {
                    let other = partner(site.pio).ok_or_else(|| err!("{}: DRIVE on a differential pad off A / C", p.name))?;
                    pio_enum(g, db, site, "DRIVE", v, tiles).map_err(at)?;
                    pio_enum(g, db, PioSite { pio: other, ..site }, "DRIVE", v, tiles).map_err(at)?;
                }
                // Trellis limitation (nextpnr warns): DRIVE only on 3V3.
                _ => {}
            }
        }
        if let Some(v) = get("TERMINATION") {
            let key = match vccio(io)? {
                "1V8" => "TERMINATION_1V8",
                "1V5" => "TERMINATION_1V5",
                "1V35" => "TERMINATION_1V35",
                other => return Err(err!("{}: TERMINATION is not supported at VCCIO {other}", p.name)),
            };
            pio_enum(g, db, site, key, v, tiles).map_err(at)?;
        }
        if let Some(v) = get("OPENDRAIN") {
            pio_enum(g, db, site, "OPENDRAIN", v, tiles).map_err(at)?;
            if diff {
                let other = partner(site.pio).ok_or_else(|| err!("{}: OPENDRAIN on a differential pad off A / C", p.name))?;
                pio_enum(g, db, PioSite { pio: other, ..site }, "OPENDRAIN", v, tiles).map_err(at)?;
            }
        }

        // The bank.
        let bank = *bank_of
            .get(&(site.row, site.col, site.pio))
            .ok_or_else(|| err!("{}: no bank for R{}C{} PIO{}", p.name, site.row, site.col, site.pio))?;
        let b = banks.entry(bank).or_default();
        if p.dir != "INPUT" || refd {
            let v = vccio(io)?;
            match &b.vcc {
                Some((have, who)) if *have != v => {
                    return Err(err!("{}: VCCIO {v} on bank {bank}, where {who} needs {have}", p.name));
                }
                Some(_) => {}
                None => b.vcc = Some((v, p.name.to_owned())),
            }
        }
        if io == "LVDS" {
            b.lvds = true;
        }
        let input = matches!(p.dir, "INPUT" | "BIDIR");
        if input && diff {
            b.diff = true;
        }
        if input && refd {
            b.vref = true;
        }
    }
    for t in &g.tiles {
        let Some(rest) = t.ty.strip_prefix("BANKREF") else {
            continue;
        };
        let digits: String = rest.chars().take_while(char::is_ascii_digit).collect();
        let Ok(bank) = digits.parse::<u32>() else {
            continue;
        };
        let Some(b) = banks.get(&bank) else {
            continue;
        };
        let bits = db.tile_bits(&t.ty)?;
        let mut set = |name: &str, value: &str| {
            if bits.enums.get(name).is_some_and(|e| e.options.contains_key(value)) {
                tiles.entry(t.name.clone()).or_default().enums.push((name.to_owned(), value.to_owned()));
            }
        };
        if let Some((v, _)) = &b.vcc {
            set("BANK.VCCIO", if *v == "1V35" { "1V2" } else { v });
        }
        if b.lvds || b.diff || b.vref {
            set("BANK.DIFF_REF", "ON");
        }
        if b.lvds {
            set("BANK.LVDSO", "ON");
        }
        if b.vref {
            set("BANK.VREF", "ON");
        }
    }
    // nextpnr init_io_banks: a bank with referenced inputs takes its
    // threshold from its VREF1_<bank> pad, written as a dummy output of the
    // bank's referenced standard.
    let vref_banks: Vec<(u32, Option<&'static str>)> =
        banks.iter().filter(|(_, b)| b.vref).map(|(&n, b)| (n, b.vcc.as_ref().map(|v| v.0))).collect();
    if !vref_banks.is_empty() {
        let vref_pad = load_vref(root, device)?;
        for (bank, vcc) in vref_banks {
            let site = *vref_pad.get(&bank).ok_or_else(|| err!("bank {bank}: no VREF1_{bank} pad"))?;
            if let Some(p) = pads.iter().find(|p| p.site == site) {
                return Err(err!(
                    "{}: on R{}C{} PIO{}, the VREF pad of bank {bank}, which its referenced inputs need",
                    p.name, site.row, site.col, site.pio
                ));
            }
            let iotype = match vcc {
                Some("1V2") => "HSUL12",
                Some("1V35" | "1V5" | "1V8") => "SSTL18_I",
                other => {
                    return Err(err!("bank {bank}: referenced inputs at VCCIO {}", other.unwrap_or("unset")));
                }
            };
            pio_enum(g, db, site, "BASE_TYPE", &format!("OUTPUT_{iotype}"), tiles)?;
            pio_enum(g, db, site, "PULLMODE", "NONE", tiles)?;
        }
    }
    Ok(())
}

/// Each bank's VREF1_<bank> pad (iodb.json `function`).
fn load_vref(root: &Path, device: &str) -> Result<HashMap<u32, PioSite>> {
    let path = root.join("ECP5").join(device).join("iodb.json");
    let text = std::fs::read_to_string(&path).map_err(|e| err!("{}: {e}", path.display()))?;
    let v: serde_json::Value = serde_json::from_str(&text).map_err(|e| err!("{}: {e}", path.display()))?;
    let mut out = HashMap::new();
    for p in v["pio_metadata"].as_array().ok_or(err!("iodb.json: pio_metadata"))? {
        let Some(bank) = p["function"].as_str().and_then(|f| f.strip_prefix("VREF1_")).and_then(|b| b.parse::<u32>().ok())
        else {
            continue;
        };
        let (Some(col), Some(row), Some(pio)) =
            (p["col"].as_i64(), p["row"].as_i64(), p["pio"].as_str().and_then(|s| s.chars().next()))
        else {
            continue;
        };
        let (Ok(col), Ok(row)) = (i16::try_from(col), i16::try_from(row)) else { continue };
        out.insert(bank, PioSite { row, col, pio });
    }
    Ok(out)
}

/// The pad buffers synthesis takes directly.
pub const CANONICAL: &[&str] = &["IB", "OB", "OBZ", "BB"];

/// The other Lattice pad buffers, written as the canonical ones before
/// lowering (the Lattice `cells_io.vh` mapping onto `TRELLIS_IO`): the PU
/// and PD variants set `PULLMODE` (under the LPF's), `ILVDS` / `OLVDS` /
/// `OBCO` keep the true pad (`A`, `Z`, `OT`) and drop the complement
/// (`AN`, `ZN`, `OC`), whose port is the pair's other pad and is no
/// longer a design port, and `TRELLIS_IO` goes by `DIR`.
pub const VARIANTS: &str = "
module IBPU(input I, output O);
endmodule
module IBPD(input I, output O);
endmodule
module OBZPU(input I, T, output O);
endmodule
module OBZPD(input I, T, output O);
endmodule
module BBPU(input I, T, output O, inout B);
endmodule
module BBPD(input I, T, output O, inout B);
endmodule
module ILVDS(input A, AN, output Z);
endmodule
module OLVDS(input A, output Z, ZN);
endmodule
module OBCO(input I, output OT, OC);
endmodule
module TRELLIS_IO(input I, T, output O, inout B);
endmodule
";

/// Rewrite the pad buffer variants of `m` as IB / OB / OBZ / BB.
pub fn canonicalize(m: &mut oxibit_hdl::netlist::Module) -> Result<()> {
    use oxibit_hdl::netlist::{Bit, Cell, Param, SigSpec};
    let mut complement: Vec<Bit> = Vec::new();
    for c in &mut m.cells {
        let Cell::Instance { ty, name, params, conns, attrs } = c else {
            continue;
        };
        let pull = |a: &mut BTreeMap<String, String>, v: &str| {
            a.entry("PULLMODE".to_owned()).or_insert_with(|| v.to_owned());
        };
        let mut rename = |conns: &mut BTreeMap<String, SigSpec>, from: &str, to: &str| {
            if let Some(s) = conns.remove(from) {
                conns.insert(to.to_owned(), s);
            }
        };
        let mut drop_pin = |conns: &mut BTreeMap<String, SigSpec>, pin: &str| {
            if let Some(s) = conns.remove(pin) {
                complement.extend(s.0);
            }
        };
        let new_ty = match ty.as_str() {
            "IBPU" | "IBPD" => {
                pull(attrs, if ty == "IBPU" { "UP" } else { "DOWN" });
                "IB"
            }
            "OBZPU" | "OBZPD" => {
                pull(attrs, if ty == "OBZPU" { "UP" } else { "DOWN" });
                "OBZ"
            }
            "BBPU" | "BBPD" => {
                pull(attrs, if ty == "BBPU" { "UP" } else { "DOWN" });
                "BB"
            }
            "ILVDS" => {
                drop_pin(conns, "AN");
                rename(conns, "A", "I");
                rename(conns, "Z", "O");
                "IB"
            }
            "OLVDS" => {
                drop_pin(conns, "ZN");
                rename(conns, "A", "I");
                rename(conns, "Z", "O");
                "OB"
            }
            "OBCO" => {
                drop_pin(conns, "OC");
                rename(conns, "OT", "O");
                "OB"
            }
            "TRELLIS_IO" => {
                let dir = match params.get("DIR") {
                    Some(Param::Str(s)) => s.clone(),
                    None => "INPUT".to_owned(),
                    Some(other) => return Err(err!("TRELLIS_IO {name}: DIR {other:?}")),
                };
                params.remove("DIR");
                match dir.as_str() {
                    "INPUT" => {
                        conns.remove("I");
                        conns.remove("T");
                        rename(conns, "B", "I");
                        "IB"
                    }
                    "OUTPUT" => {
                        rename(conns, "B", "O");
                        let t_used = conns.get("T").is_some_and(|s| s.0.iter().any(|b| !matches!(b, Bit::Zero | Bit::X | Bit::Z)));
                        if t_used {
                            "OBZ"
                        } else {
                            conns.remove("T");
                            "OB"
                        }
                    }
                    "BIDIR" => "BB",
                    d => return Err(err!("TRELLIS_IO {name}: DIR {d} is not INPUT, OUTPUT or BIDIR")),
                }
            }
            _ => continue,
        };
        *ty = new_ty.to_owned();
    }
    // A complement pad's port is the pair's other pad: not a design port.
    // It must be the whole port.
    let mut by_sig: BTreeMap<u32, Vec<u32>> = BTreeMap::new();
    for b in &complement {
        if let Bit::Sig(id, i) = b {
            by_sig.entry(id.0).or_default().push(*i);
        }
    }
    for (id, mut bits) in by_sig {
        let s = &mut m.signals[id as usize];
        if s.port.is_none() {
            continue;
        }
        bits.sort_unstable();
        bits.dedup();
        if bits.len() != s.width as usize {
            return Err(err!(
                "port {}: some of its bits are differential complements and some are not",
                s.name
            ));
        }
        s.port = None;
    }
    Ok(())
}

/// Open drain in logic. On LFE5U-85F silicon the `OPENDRAIN ON` bits
/// (fuzzed by Trellis on OUTPUT pads only) leave a BIDIR pad driving both
/// levels (pad_probe y2), so a pad buffer on a port the LPF marks
/// `OPENDRAIN=ON` is rewritten to drive only low: OB(I) becomes OBZ(I 0,
/// T I); OBZ and BB get I 0 and T = T | I. The OPENDRAIN bits are still
/// written, as nextpnr writes them.
pub fn open_drain(m: &mut oxibit_hdl::netlist::Module, ports: &std::collections::BTreeSet<String>) -> Result<usize> {
    use oxibit_hdl::netlist::{Bit, Cell, Op, SigSpec, Signal};
    if ports.is_empty() {
        return Ok(0);
    }
    // The port name of a bit: `name` or `name[i]`.
    let port_name = |m: &oxibit_hdl::netlist::Module, b: &Bit| -> Option<String> {
        let Bit::Sig(id, i) = b else { return None };
        let s = &m.signals[id.0 as usize];
        s.port?;
        Some(if s.width == 1 { s.name.clone() } else { format!("{}[{i}]", s.name) })
    };
    let mut ors: Vec<(SigSpec, SigSpec, SigSpec)> = Vec::new();
    let mut n = 0;
    let mut fresh = 0;
    let cells = std::mem::take(&mut m.cells);
    let mut out = Vec::with_capacity(cells.len());
    for mut c in cells {
        if let Cell::Instance { ty, conns, .. } = &mut c {
            let pad_pin = match ty.as_str() {
                "OB" | "OBZ" => "O",
                "BB" => "B",
                _ => "",
            };
            let on = !pad_pin.is_empty()
                && conns
                    .get(pad_pin)
                    .and_then(|s| s.0.first())
                    .and_then(|b| port_name(m, b))
                    .is_some_and(|p| ports.contains(&p));
            if on {
                let i = conns.remove("I").unwrap_or(SigSpec(vec![Bit::Zero]));
                let t = match ty.as_str() {
                    "OB" => i.clone(),
                    _ => {
                        let t = conns.remove("T").unwrap_or(SigSpec(vec![Bit::Zero]));
                        let w = m.add_signal(Signal {
                            name: format!("$oxibit_od{fresh}"),
                            width: 1,
                            signed: false,
                            port: None,
                            init: None,
                        });
                        fresh += 1;
                        let y = SigSpec(vec![Bit::Sig(w, 0)]);
                        ors.push((t, i, y.clone()));
                        y
                    }
                };
                conns.insert("I".to_owned(), SigSpec(vec![Bit::Zero]));
                conns.insert("T".to_owned(), t);
                if ty == "OB" {
                    *ty = "OBZ".to_owned();
                }
                n += 1;
            }
        }
        out.push(c);
    }
    for (a, b, y) in ors {
        out.push(Cell::Binary { op: Op::Or, a, b, signed: false, y });
    }
    m.cells = out;
    Ok(n)
}
