//! DP16KD block RAMs in the flow: bel choice, pins and tie-offs, and the
//! configuration.
//!
//! A DP16KD's settings (`EBR<z>.*`) sit in the MIB_EBR tile of its bel and
//! the two east of it (EBR0 at `MIB_EBR0` uses `MIB_EBR0`/`MIB_EBR1`, EBR1
//! at `MIB_EBR2` uses `MIB_EBR2..4`, and so on, as the prjtrellis
//! `041-ebr_config` fuzzer lists them); each setting is written into every
//! one of those tiles that has it. The initial contents are addressed by
//! the block's write ID: `EBR<z>.WID` in the configuration and the
//! `.bram_init` index (the bitstream's `LSC_EBR_ADDRESS`) are the same
//! number.
//!
//! Port A writes and port B reads.
//!
//! Unused inputs are tied the way nextpnr's `write_bram` ties them, fuse
//! for fuse, since that is the pattern known to work on silicon. Address
//! and data pins sit on CIB data wires (A..D), whose constant muxes offer
//! both levels, and are tied to the level wanted. Every control pin (CLK,
//! CE, OCE, WE, RST, CS) is tied "high" through [`tie_high`]: `1` on a CE
//! wire, `0` on an LSR or CLK wire (the only option those offer, which
//! nextpnr writes for a high tie). A control pin that must be low (an
//! unused clock, write enable or reset) then has its own `<PIN>MUX INV`
//! set. The chip selects are all tied high and `CSDECODE` is 111, so the
//! block is always selected. In x18 the two lane enables go on
//! `ADA0`/`ADA1` (byte enables) when they differ; when they are one
//! signal it drives `WEA` and the byte enables are tied high.
//!
//! Port B is never configured x9. On silicon (LFE5U-85F) a port B set
//! to `DATA_WIDTH_B` 9 behaves as x18: reads drop the word address LSB
//! and writes need the x18 byte enables. The database's only "B not 18"
//! fuse (in the tile two east, e.g. `MIB_EBR4` F1B0 for EBR1) is set and
//! is not enough; port A at 9 works for reads and writes. So an x9
//! memory reads port B at x18 (`Ebr::read_width`) and synthesis picks
//! the 9-bit word in the fabric.
//!
//! Word settings are MSB first in the database (the fuzzer wrote
//! `0b<bit0><bit1>..`), so `WID` and `CSDECODE` are written bit reversed,
//! as nextpnr does; the `.bram_init` index is the plain WID.

use std::collections::{BTreeMap, HashMap};

use crate::bels::{BelId, Bels};
use crate::chip::Level;
use crate::config::{ChipConfig, TileConfig};
use crate::db::DeviceDb;
use crate::graph::Graph;
use crate::io::{tie, tie_high};
use crate::netlist::{Cell, Ebr, EbrWidth, NetId, Netlist, Polarity, Prim, PrimPin};
use oxibit_hdl::netlist::Param;
use crate::{Result, err};


/// Port declarations of the instantiable block RAMs, for synthesis.
pub const PRIMITIVES: &str = "
module DP16KD(input DIA17, DIA16, DIA15, DIA14, DIA13, DIA12, DIA11, DIA10, DIA9, DIA8, DIA7, DIA6, 
DIA5, DIA4, DIA3, DIA2, DIA1, DIA0, ADA13, ADA12, ADA11, ADA10, ADA9, ADA8, ADA7, ADA6, ADA5, ADA4, 
ADA3, ADA2, ADA1, ADA0, CEA, OCEA, CLKA, WEA, CSA2, CSA1, CSA0, RSTA, DIB17, DIB16, DIB15, DIB14, 
DIB13, DIB12, DIB11, DIB10, DIB9, DIB8, DIB7, DIB6, DIB5, DIB4, DIB3, DIB2, DIB1, DIB0, ADB13, 
ADB12, ADB11, ADB10, ADB9, ADB8, ADB7, ADB6, ADB5, ADB4, ADB3, ADB2, ADB1, ADB0, CEB, OCEB, CLKB, 
WEB, CSB2, CSB1, CSB0, RSTB,
  output DOA17, DOA16, DOA15, DOA14, DOA13, DOA12, DOA11, DOA10, DOA9, DOA8, DOA7, DOA6, DOA5, 
DOA4, DOA3, DOA2, DOA1, DOA0, DOB17, DOB16, DOB15, DOB14, DOB13, DOB12, DOB11, DOB10, DOB9, DOB8, 
DOB7, DOB6, DOB5, DOB4, DOB3, DOB2, DOB1, DOB0);
endmodule
module PDPW16KD(input DI35, DI34, DI33, DI32, DI31, DI30, DI29, DI28, DI27, DI26, DI25, DI24, DI23, 
DI22, DI21, DI20, DI19, DI18, DI17, DI16, DI15, DI14, DI13, DI12, DI11, DI10, DI9, DI8, DI7, DI6, 
DI5, DI4, DI3, DI2, DI1, DI0, ADW8, ADW7, ADW6, ADW5, ADW4, ADW3, ADW2, ADW1, ADW0, BE3, BE2, BE1, 
BE0, CEW, CLKW, CSW2, CSW1, CSW0, ADR13, ADR12, ADR11, ADR10, ADR9, ADR8, ADR7, ADR6, ADR5, ADR4, 
ADR3, ADR2, ADR1, ADR0, CER, OCER, CLKR, CSR2, CSR1, CSR0, RST,
  output DO35, DO34, DO33, DO32, DO31, DO30, DO29, DO28, DO27, DO26, DO25, DO24, DO23, DO22, DO21, 
DO20, DO19, DO18, DO17, DO16, DO15, DO14, DO13, DO12, DO11, DO10, DO9, DO8, DO7, DO6, DO5, DO4, 
DO3, DO2, DO1, DO0);
endmodule
";

/// The instantiable block RAM types. A PDPW16KD becomes a DP16KD in
/// synthesis (nextpnr's `pack_ebr` renaming), with `DATA_WIDTH_A` 36 to
/// mark it.
pub const TYPES: &[&str] = &["DP16KD", "PDPW16KD"];

pub fn is_bram(ty: &str) -> bool {
    TYPES.contains(&ty)
}

pub fn is_clock_input(ty: &str, pin: &str) -> bool {
    is_bram(ty) && matches!(pin, "CLKA" | "CLKB" | "CLKW" | "CLKR")
}

/// A control input: on a CIB CE, LSR or CLK wire. A constant on one is its
/// `<PIN>MUX` (the pin itself for 1, INV for 0) over a high tie.
pub fn control(pin: &str) -> bool {
    ["CLK", "CE", "OCE", "WE", "RST", "CS"].iter().any(|p| pin.starts_with(p))
}

/// The DP16KD pins a PDPW16KD pin becomes (nextpnr `pack_ebr`).
pub fn pdp_pins(pin: &str) -> Vec<String> {
    let bus = |p: &str| pin.strip_prefix(p).and_then(|n| n.parse::<usize>().ok());
    let one = |s: String| vec![s];
    if let Some(i) = bus("BE") {
        return one(format!("ADA{i}"));
    }
    if let Some(i) = bus("ADW") {
        return one(format!("ADA{}", i + 5));
    }
    if let Some(i) = bus("ADR") {
        return one(format!("ADB{i}"));
    }
    if let Some(i) = bus("CSW") {
        return one(format!("CSA{i}"));
    }
    if let Some(i) = bus("CSR") {
        return one(format!("CSB{i}"));
    }
    if let Some(i) = bus("DI") {
        return one(if i < 18 { format!("DIA{i}") } else { format!("DIB{}", i - 18) });
    }
    if let Some(i) = bus("DO") {
        return one(if i < 18 { format!("DOB{i}") } else { format!("DOA{}", i - 18) });
    }
    match pin {
        "CLKW" => one("CLKA".into()),
        "CLKR" => one("CLKB".into()),
        "CEW" => one("CEA".into()),
        "CER" => one("CEB".into()),
        "OCER" => one("OCEB".into()),
        "RST" => vec!["RSTA".into(), "RSTB".into()],
        _ => one(pin.to_owned()),
    }
}


/// The PDPW16KD pin a renamed DP16KD pin came from (the inverse of
/// [`pdp_pins`]), for checks against the design.
pub fn pdp_pin(pin: &str) -> String {
    let bus = |p: &str| pin.strip_prefix(p).and_then(|n| n.parse::<usize>().ok());
    if let Some(i) = bus("ADA") {
        return if i < 4 { format!("BE{i}") } else { format!("ADW{}", i - 5) };
    }
    if let Some(i) = bus("ADB") {
        return format!("ADR{i}");
    }
    if let Some(i) = bus("CSA") {
        return format!("CSW{i}");
    }
    if let Some(i) = bus("CSB") {
        return format!("CSR{i}");
    }
    if let Some(i) = bus("DIA") {
        return format!("DI{i}");
    }
    if let Some(i) = bus("DIB") {
        return format!("DI{}", i + 18);
    }
    if let Some(i) = bus("DOB") {
        return format!("DO{i}");
    }
    if let Some(i) = bus("DOA") {
        return format!("DO{}", i + 18);
    }
    match pin {
        "CLKA" => "CLKW",
        "CLKB" => "CLKR",
        "CEA" => "CEW",
        "CEB" => "CER",
        "OCEB" => "OCER",
        "RSTA" | "RSTB" => "RST",
        p => p,
    }
    .to_owned()
}

/// The attribute marking a DP16KD that was a PDPW16KD in the design.
pub const FROM_PDP: &str = "oxibit.from_pdpw16kd";
/// A PDPW16KD's parameters as a DP16KD's (nextpnr `pack_ebr`).
pub fn pdp_params(params: &mut BTreeMap<String, Param>) {
    params.insert("DATA_WIDTH_A".into(), Param::Int(36));
    params.remove("DATA_WIDTH_W");
    let mut rename = |o: &str, n: &str| {
        if let Some(v) = params.remove(o) {
            params.insert(n.to_owned(), v);
        }
    };
    rename("CLKWMUX", "CLKAMUX");
    rename("CLKRMUX", "CLKBMUX");
    rename("CSDECODE_W", "CSDECODE_A");
    rename("CSDECODE_R", "CSDECODE_B");
    rename("DATA_WIDTH_R", "DATA_WIDTH_B");
    for (k, from, to) in [("CLKAMUX", "CLKW", "CLKA"), ("CLKBMUX", "CLKR", "CLKB")] {
        if matches!(params.get(k), Some(Param::Str(s)) if s == from) {
            params.insert(k.to_owned(), Param::Str(to.to_owned()));
        }
    }
    let reg = match params.remove("REGMODE") {
        Some(v) => v,
        None => Param::Str("NOREG".into()),
    };
    params.insert("REGMODE_A".into(), reg.clone());
    params.insert("REGMODE_B".into(), reg);
}

/// A parameter's bits, LSB first, `n` of them: a Verilog literal or a
/// `0x` / `0b` string.
fn param_bits(p: &Prim, name: &str, n: usize) -> Result<Vec<bool>> {
    let mut v = vec![false; n];
    match p.params.get(name) {
        None => {}
        Some(Param::Bits(b)) => {
            for (i, x) in b.iter().take(n).enumerate() {
                v[i] = *x == Some(true);
            }
        }
        Some(Param::Int(i)) => {
            for (k, x) in v.iter_mut().enumerate().take(63) {
                *x = (i >> k) & 1 == 1;
            }
        }
        Some(Param::Str(s)) => {
            let s = s.replace('_', "");
            let (digits, radix) = if let Some(h) = s.strip_prefix("0x") {
                (h.to_owned(), 16)
            } else if let Some(b) = s.strip_prefix("0b") {
                (b.to_owned(), 2)
            } else {
                return Err(err!("{} {}: {name} {s:?} is not 0x.. or 0b..", p.ty, p.name));
            };
            let per = if radix == 16 { 4 } else { 1 };
            for (k, c) in digits.chars().rev().enumerate() {
                let d = c.to_digit(radix).ok_or_else(|| err!("{} {}: bad digit in {name}", p.ty, p.name))?;
                for j in 0..per {
                    if let Some(x) = v.get_mut(k * per + j) {
                        *x = (d >> j) & 1 == 1;
                    }
                }
            }
        }
    }
    Ok(v)
}

/// An instantiated DP16KD (or a renamed PDPW16KD): nextpnr's `write_bram`,
/// setting for setting and tie for tie.
fn configure_prim(
    g: &Graph,
    db: &DeviceDb,
    bels: &Bels,
    b: BelId,
    p: &Prim,
    wid: usize,
    cc: &mut ChipConfig,
) -> Result<()> {
    let bel = bels.bels[b as usize];
    let name = bels.template(b).name.clone();
    let loc = (bel.x, bel.y, name.as_str());
    let s = |k: &str, d: &str| p.param_str(k).unwrap_or_else(|| d.to_owned());
    let pdp = p.param_int("DATA_WIDTH_A") == Some(36);
    let tiles = &mut cc.tiles;
    if pdp {
        set_enum(g, db, loc, "MODE", "PDPW16KD", tiles)?;
        set_enum(g, db, loc, "PDPW16KD.DATA_WIDTH_R", &s("DATA_WIDTH_B", "36"), tiles)?;
    } else {
        set_enum(g, db, loc, "MODE", "DP16KD", tiles)?;
        set_enum(g, db, loc, "DP16KD.DATA_WIDTH_A", &s("DATA_WIDTH_A", "18"), tiles)?;
        set_enum(g, db, loc, "DP16KD.DATA_WIDTH_B", &s("DATA_WIDTH_B", "18"), tiles)?;
        set_enum(g, db, loc, "DP16KD.WRITEMODE_A", &s("WRITEMODE_A", "NORMAL"), tiles)?;
        set_enum(g, db, loc, "DP16KD.WRITEMODE_B", &s("WRITEMODE_B", "NORMAL"), tiles)?;
    }
    set_enum(g, db, loc, "REGMODE_A", &s("REGMODE_A", "NOREG"), tiles)?;
    set_enum(g, db, loc, "REGMODE_B", &s("REGMODE_B", "NOREG"), tiles)?;
    set_enum(g, db, loc, "RESETMODE", &s("RESETMODE", "SYNC"), tiles)?;
    set_enum(g, db, loc, "ASYNC_RESET_RELEASE", &s("ASYNC_RESET_RELEASE", "SYNC"), tiles)?;
    set_enum(g, db, loc, "GSR", &s("GSR", "DISABLED"), tiles)?;
    let wid_bits: Vec<bool> = (0..9).map(|k| (wid >> (8 - k)) & 1 == 1).collect();
    set_word(g, db, loc, "WID", &wid_bits, tiles)?;

    // Ties, and the mux each unused control pin needs for its level.
    let mut mux: BTreeMap<String, String> = BTreeMap::new();
    for (k, _) in p.params.iter().filter(|(k, _)| k.ends_with("MUX")) {
        if let Some(v) = p.param_str(k) {
            mux.insert(k.clone(), v);
        }
    }
    let ins: Vec<String> = bels
        .pins(b)
        .filter(|(pp, _)| pp.dir == crate::bels::PinDir::In)
        .map(|(pp, _)| pp.name.clone())
        .collect();
    for pin in ins {
        if pdp && matches!(pin.as_str(), "WEA" | "WEB" | "ADA4") {
            continue;
        }
        if matches!(p.pin(&pin), Some(PrimPin::In(Some(_)))) {
            continue;
        }
        let wire = bels.pin_wire(b, &pin).ok_or_else(|| err!("{name} has no pin {pin}"))?;
        let at = |er| err!("{} {} ({name} R{}C{}) {pin}: {er}", p.ty, p.name, bel.y, bel.x);
        let key = format!("{pin}MUX");
        if control(&pin) {
            tie_high(g, db, wire, tiles).map_err(at)?;
            let low = matches!(pin.as_str(), "CLKA" | "CLKB" | "WEA" | "WEB" | "RSTA" | "RSTB") || pin.starts_with("CS");
            mux.entry(key).or_insert_with(|| if low { "INV".to_owned() } else { pin.clone() });
        } else {
            let high = mux.get(&key).is_some_and(|v| v == "1");
            tie(g, db, wire, if high { Level::High } else { Level::Low }, tiles).map_err(at)?;
        }
    }
    let m = |k: &str, d: &str| mux.get(k).cloned().unwrap_or_else(|| d.to_owned());
    for (port, dflt) in [("CLKA", "CLKA"), ("CLKB", "CLKB"), ("RSTA", "RSTA"), ("RSTB", "RSTB"), ("CEA", "CEA"), ("CEB", "CEB"), ("OCEA", "OCEA"), ("OCEB", "OCEB")] {
        set_enum(g, db, loc, &format!("{port}MUX"), &m(&format!("{port}MUX"), dflt), tiles)?;
    }
    if !pdp {
        set_enum(g, db, loc, "WEAMUX", &m("WEAMUX", "WEA"), tiles)?;
        set_enum(g, db, loc, "WEBMUX", &m("WEBMUX", "WEB"), tiles)?;
    }
    // CSDECODE, with the bit flipped where a chip select is inverted, then
    // MSB first as the database has it.
    for (port, param) in [("CSA", "CSDECODE_A"), ("CSB", "CSDECODE_B")] {
        let text = s(param, "0b000");
        let digits = text.strip_prefix("0b").unwrap_or(&text);
        let mut bits: Vec<bool> = (0..3).map(|i| digits.chars().rev().nth(i) == Some('1')).collect();
        for (i, x) in bits.iter_mut().enumerate() {
            if m(&format!("{port}{i}MUX"), "") == "INV" {
                *x = !*x;
            }
        }
        bits.reverse();
        set_word(g, db, loc, param, &bits, tiles)?;
    }
    // INITVAL_00..3F: sixteen 18-bit words each, two padding bits apart.
    let mut init = vec![0u16; 2048];
    for i in 0..0x40 {
        let v = param_bits(p, &format!("INITVAL_{i:02X}"), 320)?;
        for j in 0..16 {
            for k in 0..18 {
                if v[20 * j + k] {
                    init[i * 32 + j * 2 + k / 9] |= 1 << (k % 9);
                }
            }
        }
    }
    let wid = u16::try_from(wid).map_err(|_| err!("too many block RAMs"))?;
    cc.bram_data.insert(wid, init);
    Ok(())
}
/// How a DP16KD pin is used.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Use {
    /// Driven by the net.
    Sink(NetId),
    /// A clock input on the net.
    Clock(NetId),
    /// Drives the net.
    Source(NetId),
    /// An address or data pin tied to a constant.
    Tie(Level),
    /// A control pin tied to a constant: high through the CIB, low by
    /// also setting the pin's own inverter.
    Control(Level),
}

/// Every pin of `e` the design uses or ties, by pin name.
pub fn pin_uses(e: &Ebr) -> Vec<(String, Use)> {
    let mut v: Vec<(String, Use)> = Vec::new();
    let addr = |v: &mut Vec<(String, Use)>,
                port: char,
                lsb: usize,
                nets: &[NetId],
                low: &[(usize, Level)]| {
        let mut driven = [false; 14];
        for &(k, level) in low {
            v.push((format!("AD{port}{k}"), Use::Tie(level)));
            driven[k] = true;
        }
        for (k, &n) in nets.iter().enumerate() {
            v.push((format!("AD{port}{}", k + lsb), Use::Sink(n)));
            driven[k + lsb] = true;
        }
        for (k, d) in driven.iter().enumerate() {
            if !d {
                v.push((format!("AD{port}{k}"), Use::Tie(Level::Low)));
            }
        }
    };
    let cs = |v: &mut Vec<(String, Use)>, port: char| {
        for k in 0..3 {
            v.push((format!("CS{port}{k}"), Use::Control(Level::High)));
        }
    };
    // Port B: read.
    let r = &e.read;
    v.push(("CLKB".to_owned(), Use::Clock(r.clk)));
    addr(&mut v, 'B', e.read_width.addr_lsb(), &r.addr, &[]);
    for (j, d) in r.data.iter().enumerate() {
        if let Some(d) = d {
            v.push((format!("DOB{j}"), Use::Source(*d)));
        }
    }
    v.push(("CEB".to_owned(), Use::Control(Level::High)));
    v.push(("OCEB".to_owned(), Use::Control(Level::High)));
    v.push(("RSTB".to_owned(), Use::Control(Level::Low)));
    v.push(("WEB".to_owned(), Use::Control(Level::Low)));
    cs(&mut v, 'B');
    // Port A: write.
    match &e.write {
        Some(w) => {
            v.push(("CLKA".to_owned(), Use::Clock(w.clk)));
            let same = w.we[1].is_none() || w.we[0] == w.we[1];
            let we0 = w.we[0];
            match (e.width, same) {
                (EbrWidth::X9, _) | (EbrWidth::X18, true) => {
                    v.push((
                        "WEA".to_owned(),
                        we0.map_or(Use::Control(Level::Low), Use::Sink),
                    ));
                    let be: &[(usize, Level)] = if e.width == EbrWidth::X18 {
                        &[(0, Level::High), (1, Level::High)]
                    } else {
                        &[]
                    };
                    addr(&mut v, 'A', e.width.addr_lsb(), &w.addr, be);
                }
                (EbrWidth::X18, false) => {
                    v.push(("WEA".to_owned(), Use::Control(Level::High)));
                    let mut tmp = Vec::new();
                    addr(&mut tmp, 'A', e.width.addr_lsb(), &w.addr, &[]);
                    for (pin, u) in tmp {
                        let be = match pin.as_str() {
                            "ADA0" => w.we[0],
                            "ADA1" => w.we[1],
                            _ => {
                                v.push((pin, u));
                                continue;
                            }
                        };
                        v.push((pin, be.map_or(Use::Tie(Level::Low), Use::Sink)));
                    }
                }
            }
            for (j, d) in w.data.iter().enumerate() {
                if let Some(d) = d {
                    v.push((format!("DIA{j}"), Use::Sink(*d)));
                }
            }
            v.push(("CEA".to_owned(), Use::Control(Level::High)));
            v.push(("OCEA".to_owned(), Use::Control(Level::High)));
        }
        None => {
            v.push(("CLKA".to_owned(), Use::Control(Level::Low)));
            v.push(("WEA".to_owned(), Use::Control(Level::Low)));
            v.push(("CEA".to_owned(), Use::Control(Level::High)));
            v.push(("OCEA".to_owned(), Use::Control(Level::High)));
        }
    }
    v.push(("RSTA".to_owned(), Use::Control(Level::Low)));
    cs(&mut v, 'A');
    v
}

/// A DP16KD bel for every block RAM cell, nearest `near` (row, col) first.
pub fn assign(bels: &Bels, nl: &Netlist, near: (f64, f64)) -> Result<HashMap<usize, BelId>> {
    let mut free: Vec<BelId> = (0..bels.bels.len())
        .map(|i| BelId::try_from(i).expect("fits"))
        .filter(|&b| bels.template(b).ty == "DP16KD")
        .collect();
    let d = |b: BelId| {
        let bel = bels.bels[b as usize];
        let dy = f64::from(bel.y) - near.0;
        let dx = f64::from(bel.x) - near.1;
        dx * dx + dy * dy
    };
    free.sort_by(|&a, &b| d(a).total_cmp(&d(b)).then(a.cmp(&b)));
    let mut out = HashMap::new();
    let mut it = free.into_iter();
    for (i, c) in nl.cells.iter().enumerate() {
        if matches!(c, Cell::Ebr(_)) || matches!(c, Cell::Prim(p) if is_bram(&p.ty)) {
            let b = it
                .next()
                .ok_or_else(|| err!("more block RAMs than the device has"))?;
            out.insert(i, b);
        }
    }
    Ok(out)
}

/// The tiles holding `EBR<z>.*` for a bel at (x, y).
fn holds(bx: i16, by: i16, x: i16, y: i16) -> bool {
    y == by && (bx..=bx + 2).contains(&x)
}

fn set_enum(
    g: &Graph,
    db: &DeviceDb,
    bel: (i16, i16, &str),
    name: &str,
    value: &str,
    tiles: &mut BTreeMap<String, TileConfig>,
) -> Result<()> {
    let full = format!("{}.{name}", bel.2);
    let mut n = 0;
    for t in g.tiles.iter().filter(|t| holds(bel.0, bel.1, t.x, t.y)) {
        let Some(e) = db.tile_bits(&t.ty)?.enums.get(&full) else {
            continue;
        };
        if !e.options.contains_key(value) {
            return Err(err!("{}: {full} has no option {value}", t.name));
        }
        tiles
            .entry(t.name.clone())
            .or_default()
            .enums
            .push((full.clone(), value.to_owned()));
        n += 1;
    }
    if n == 0 {
        return Err(err!("no tile at R{}C{} has {full}", bel.1, bel.0));
    }
    Ok(())
}

fn set_word(
    g: &Graph,
    db: &DeviceDb,
    bel: (i16, i16, &str),
    name: &str,
    value: &[bool],
    tiles: &mut BTreeMap<String, TileConfig>,
) -> Result<()> {
    let full = format!("{}.{name}", bel.2);
    let mut n = 0;
    for t in g.tiles.iter().filter(|t| holds(bel.0, bel.1, t.x, t.y)) {
        let Some(w) = db.tile_bits(&t.ty)?.words.get(&full) else {
            continue;
        };
        if w.bits.len() != value.len() {
            return Err(err!(
                "{}: {full} is {} bits, not {}",
                t.name,
                w.bits.len(),
                value.len()
            ));
        }
        tiles
            .entry(t.name.clone())
            .or_default()
            .words
            .push((full.clone(), value.to_vec()));
        n += 1;
    }
    if n == 0 {
        return Err(err!("no tile at R{}C{} has {full}", bel.1, bel.0));
    }
    Ok(())
}

/// Write every block RAM's settings, ties and initial contents.
pub fn configure(
    g: &Graph,
    db: &DeviceDb,
    bels: &Bels,
    nl: &Netlist,
    at: &HashMap<usize, BelId>,
    cc: &mut ChipConfig,
) -> Result<()> {
    let mut cells: Vec<(usize, BelId)> = at.iter().map(|(&c, &b)| (c, b)).collect();
    cells.sort_unstable();
    for (wid, (c, b)) in cells.into_iter().enumerate() {
        let e = match &nl.cells[c] {
            Cell::Ebr(e) => e,
            Cell::Prim(p) if is_bram(&p.ty) => {
                configure_prim(g, db, bels, b, p, wid, cc)?;
                continue;
            }
            _ => return Err(err!("cell {c} is not a block RAM")),
        };
        let bel = bels.bels[b as usize];
        let name = bels.template(b).name.clone();
        let loc = (bel.x, bel.y, name.as_str());
        let wname = |w: EbrWidth| match w {
            EbrWidth::X9 => "9",
            EbrWidth::X18 => "18",
        };
        let tiles = &mut cc.tiles;
        set_enum(g, db, loc, "MODE", "DP16KD", tiles)?;
        set_enum(g, db, loc, "DP16KD.DATA_WIDTH_A", wname(e.width), tiles)?;
        if e.read_width == EbrWidth::X9 {
            return Err(err!("{name}: port B cannot be x9 (it behaves as x18)"));
        }
        set_enum(g, db, loc, "DP16KD.DATA_WIDTH_B", wname(e.read_width), tiles)?;
        set_enum(g, db, loc, "DP16KD.WRITEMODE_A", "NORMAL", tiles)?;
        set_enum(g, db, loc, "DP16KD.WRITEMODE_B", "NORMAL", tiles)?;
        set_enum(g, db, loc, "REGMODE_A", "NOREG", tiles)?;
        set_enum(g, db, loc, "REGMODE_B", "NOREG", tiles)?;
        set_enum(g, db, loc, "RESETMODE", "SYNC", tiles)?;
        let wid_u16 = u16::try_from(wid).map_err(|_| err!("too many block RAMs"))?;
        let wid_bits: Vec<bool> = (0..9).map(|k| (wid >> (8 - k)) & 1 == 1).collect();
        set_word(g, db, loc, "WID", &wid_bits, tiles)?;
        if e.read.clk_polarity == Polarity::Neg {
            set_enum(g, db, loc, "CLKBMUX", "INV", tiles)?;
        }
        if e.write.as_ref().is_some_and(|w| w.clk_polarity == Polarity::Neg) {
            set_enum(g, db, loc, "CLKAMUX", "INV", tiles)?;
        }
        for (pin, u) in pin_uses(e) {
            let wire = || {
                bels.pin_wire(b, &pin)
                    .ok_or_else(|| err!("{name} has no pin {pin}"))
            };
            let at = |er| err!("{name} R{}C{} {pin}: {er}", bel.y, bel.x);
            match u {
                Use::Tie(level) => {
                    tie(g, db, wire()?, level, tiles).map_err(at)?;
                }
                Use::Control(level) => {
                    tie_high(g, db, wire()?, tiles).map_err(at)?;
                    if level == Level::Low {
                        if pin.starts_with("CS") {
                            return Err(at(err!("a chip select cannot be tied low")));
                        }
                        set_enum(g, db, loc, &format!("{pin}MUX"), "INV", tiles)?;
                    }
                }
                _ => {}
            }
        }
        set_word(g, db, loc, "CSDECODE_A", &[true; 3], tiles)?;
        set_word(g, db, loc, "CSDECODE_B", &[true; 3], tiles)?;
        // Always written, zeros included: a block left out keeps whatever
        // it powered up with.
        cc.bram_data.insert(wid_u16, e.init.clone());
    }
    Ok(())
}
