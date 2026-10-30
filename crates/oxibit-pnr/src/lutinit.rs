//! LUT INIT under input permutation.
//!
//! The router takes a LUT input on any pin of its group (`Net::pin_groups`),
//! not necessarily the bel pin the placer gave it. The LUT's truth table
//! must then read that input from the new pin: an INIT written for the
//! placer's pins (nextpnr's FASM of a placed, unrouted design) is rewritten
//! here by permuting its address bits.
//!
//! Pins are numbered 1..=6 (A1..A6), address bit `k - 1` of the INIT. A move
//! `(from, to)` takes the signal the placer put on `from` to `to`; several
//! `from` pins of one signal may collapse onto one `to`. Pins no move names
//! keep their place: constant ties, A6 of a fractured LUT, pins of a memory
//! slice, and unused pins (which the INIT does not depend on).

use std::collections::{BTreeMap, HashMap, HashSet};

use oxibit_fasm::db::Database;
use oxibit_fasm::parse::parse_line;
use serde_json::Value;

use crate::placed::{self, Import};
use crate::router::{self, Hop};

type R<T> = Result<T, String>;

/// One signal into one LUT: the bel pins the placer put it on, and the pin
/// the router took.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Move {
    pub from: Vec<u8>,
    pub to: u8,
}

/// A LUT: (tile, FASM site, letter).
pub type LutKey = (String, String, char);

/// Moves per LUT.
pub type Moves = BTreeMap<LutKey, Vec<Move>>;

/// INIT per LUT.
pub type Inits = BTreeMap<LutKey, u64>;

/// The INIT that computes the same function as `init` once every move is
/// made: new address `a'` reads old address `a`, where `a` takes each moved
/// pin's bit from its `to` pin in `a'` and every other pin's bit from itself.
pub fn permute(init: u64, moves: &[Move]) -> R<u64> {
    let mut src = [0u8; 6]; // old pin index -> new pin index it reads
    for (i, s) in src.iter_mut().enumerate() {
        *s = i as u8;
    }
    let mut from_seen: HashSet<u8> = HashSet::new();
    let mut to_seen: HashSet<u8> = HashSet::new();
    for m in moves {
        if !(1..=6).contains(&m.to) || !to_seen.insert(m.to) {
            return Err(format!("LUT pin A{} taken twice or out of range", m.to));
        }
        for &f in &m.from {
            if !(1..=6).contains(&f) || !from_seen.insert(f) {
                return Err(format!("LUT pin A{f} moved twice or out of range"));
            }
            src[(f - 1) as usize] = m.to - 1;
        }
    }
    let mut out = 0u64;
    for a2 in 0..64u64 {
        let mut a = 0u64;
        for (i, &s) in src.iter().enumerate() {
            a |= (a2 >> s & 1) << i;
        }
        out |= (init >> a & 1) << a2;
    }
    Ok(out)
}

/// The moves the router made: for each pin group of each routed net, the
/// pin it reached against the pins the placer used. A group the router left
/// unreached is an error: its LUT would read nothing.
pub fn moves(imp: &Import, routes: &[Vec<Hop>]) -> R<Moves> {
    let mut out: Moves = BTreeMap::new();
    for (i, hops) in routes.iter().take(imp.nets.len()).enumerate() {
        let net = &imp.nets[i];
        for ((grp, slot), pin) in net
            .pin_groups
            .iter()
            .zip(&imp.group_slots[i])
            .zip(router::chosen_pins(net, hops))
        {
            let Some(pin) = pin else {
                return Err(format!(
                    "{}: LUT {}/{} {} input unrouted",
                    imp.names[i], slot.tile, slot.site, slot.letter
                ));
            };
            let idx = grp
                .iter()
                .position(|&p| p == pin)
                .ok_or("chosen pin outside its group")?;
            let to = slot.pins[idx];
            out.entry((slot.tile.clone(), slot.site.clone(), slot.letter))
                .or_default()
                .push(Move {
                    from: slot.from.clone(),
                    to,
                });
        }
    }
    Ok(out)
}

/// `TILE.SITE.XLUT.INIT` split into (tile, site, letter).
pub fn lut_key(feature: &str) -> Option<LutKey> {
    let mut parts = feature.split('.');
    let (tile, site, bel, init) = (parts.next()?, parts.next()?, parts.next()?, parts.next()?);
    if parts.next().is_some() || init != "INIT" || bel.len() != 4 || !bel.ends_with("LUT") {
        return None;
    }
    let letter = bel.chars().next().filter(|c| "ABCD".contains(*c))?;
    Some((tile.to_string(), site.to_string(), letter))
}

/// Rewrite the LUT INIT lines of `fasm` for `moves`. Every other line is
/// kept as it is. A LUT's INIT may come split over several lines; it is
/// written back as one `[63:0]` line where its first line was. A LUT with
/// moves and no INIT line computes 0 and stays 0.
///
/// Returns the FASM and the number of LUTs rewritten.
pub fn rewrite_fasm(fasm: &str, moves: &Moves) -> R<(String, usize)> {
    let inits = fasm_inits(fasm)?;
    let mut out = String::with_capacity(fasm.len());
    let mut written: HashSet<LutKey> = HashSet::new();
    let mut rewritten = 0usize;
    for line in fasm.lines() {
        let key = match parse_line(line)? {
            Some(f) => lut_key(&f.feature),
            None => None,
        };
        let Some(key) = key else {
            out.push_str(line);
            out.push('\n');
            continue;
        };
        if !written.insert(key.clone()) {
            continue; // the rest of a split INIT
        }
        let init = inits[&key];
        let new = match moves.get(&key) {
            Some(m) => {
                rewritten += 1;
                permute(init, m)?
            }
            None => init,
        };
        let (tile, site, letter) = &key;
        out.push_str(&format!(
            "{tile}.{site}.{letter}LUT.INIT[63:0] = 64'h{new:016X}\n"
        ));
    }
    Ok((out, rewritten))
}

/// The INIT of every LUT a FASM sets, by (tile, site, letter), split
/// lines joined.
pub fn fasm_inits(fasm: &str) -> R<Inits> {
    let mut inits: Inits = BTreeMap::new();
    for line in fasm.lines() {
        let Some(f) = parse_line(line)? else { continue };
        let Some(key) = lut_key(&f.feature) else {
            continue;
        };
        let v = inits.entry(key).or_default();
        let start = f.start.unwrap_or(0);
        for (k, &b) in f.value.iter().enumerate() {
            let a = start as usize + k;
            if a >= 64 {
                return Err(format!("INIT address {a} out of range: {line}"));
            }
            if b {
                *v |= 1 << a;
            }
        }
    }
    Ok(inits)
}

/// Where a placed cell sits: tile, tile type, the site as nextpnr names it
/// relative to its tile (`SLICE_X1Y0`), and the bel (`A6LUT`).
pub type Located = (String, String, String, String);

/// Resolves nextpnr bel names to `Located`, from the tilegrid: grid
/// coordinates (`X14Y142/SLICE_X0Y0.A6LUT`) or the absolute site name of
/// older builds (`SLICE_X43Y98/A6LUT`).
pub struct Locator {
    at: HashMap<(i64, i64), (String, String)>,
    tile_type: HashMap<String, String>,
    pos: HashMap<String, (i64, i64)>,
    sites: HashMap<String, (String, String)>,
}

impl Locator {
    #[oxibit_prof::hot(name = "Locator::new")]
    pub fn new(db: &Database) -> R<Self> {
        let path = db.root.join(&db.fabric).join("tilegrid.json");
        let text = std::fs::read_to_string(&path).map_err(|e| format!("{}: {e}", path.display()))?;
        let grid: Value = serde_json::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))?;
        let mut at = HashMap::new();
        let mut tile_type = HashMap::new();
        let mut pos = HashMap::new();
        for (name, t) in grid.as_object().ok_or("tilegrid is not an object")? {
            let tt = t["type"].as_str().unwrap_or("").to_string();
            let xy = (t["grid_x"].as_i64().unwrap_or(-1), t["grid_y"].as_i64().unwrap_or(-1));
            at.insert(xy, (name.clone(), tt.clone()));
            tile_type.insert(name.clone(), tt);
            pos.insert(name.clone(), xy);
        }
        Ok(Locator { at, tile_type, pos, sites: placed::site_index(db) })
    }

    /// A tile's type.
    pub fn tile_type(&self, tile: &str) -> Option<&str> {
        self.tile_type.get(tile).map(String::as_str)
    }

    /// The tile at a grid position.
    pub fn tile_at(&self, x: i64, y: i64) -> Option<&str> {
        self.at.get(&(x, y)).map(|(n, _)| n.as_str())
    }

    /// A tile's grid position (x, y).
    pub fn grid(&self, tile: &str) -> Option<(i64, i64)> {
        self.pos.get(tile).copied()
    }

    /// Every tile: (name, type).
    pub fn tiles(&self) -> impl Iterator<Item = (&str, &str)> {
        self.tile_type.iter().map(|(n, t)| (n.as_str(), t.as_str()))
    }

    pub fn locate(&self, bel: &str) -> Option<Located> {
        let (head, rest) = bel.split_once('/')?;
        if let Some((x, y)) = head.strip_prefix('X').and_then(|t| t.split_once('Y')) {
            if let (Ok(x), Ok(y)) = (x.parse::<i64>(), y.parse::<i64>()) {
                let (tile, tt) = self.at.get(&(x, y))?.clone();
                let (site, b) = rest.split_once('.')?;
                return Some((tile, tt, site.to_string(), b.to_string()));
            }
        }
        let (tile, rel) = self.sites.get(head)?.clone();
        let tt = self.tile_type.get(&tile)?.clone();
        Some((tile, tt, rel, rest.to_string()))
    }
}

/// Which pin a one-input identity INIT passes to O6: a LUT doing no logic,
/// only carrying a route (nextpnr routes through unused LUTs this way).
pub fn passes(init: u64) -> Option<u8> {
    (1..=6u8).find(|&k| (0..64u64).all(|a| (init >> a & 1) == (a >> (k - 1) & 1)))
}

/// What `apply` did.
#[derive(Debug, Default, Clone, Copy)]
pub struct Applied {
    /// LUT INITs written from the cells.
    pub written: usize,
    /// Of those, LUTs with an input moved.
    pub moved: usize,
    /// INIT lines dropped: LUTs with no cell that only passed a route.
    pub routing_dropped: usize,
    /// INIT lines kept as they were: LUTs with no cell that compute
    /// something.
    pub cellless_kept: usize,
}

/// Put the cells' LUT INITs (`cell_inits`), with `moves` made, into
/// `fasm` in place of its own. The FASM's other lines are kept; a LUT the
/// cells do not hold keeps its INIT unless it only passes a route (the
/// routing that used it is gone).
pub fn apply(fasm: &str, cells: &Inits, moves: &Moves) -> R<(String, Applied)> {
    let mut stats = Applied::default();
    let (fin, moved) = final_inits(cells, moves)?;
    stats.moved = moved;
    let theirs = fasm_inits(fasm)?;
    let mut out = String::with_capacity(fasm.len());
    let mut done: HashSet<LutKey> = HashSet::new();
    let put = |out: &mut String, k: &LutKey, v: u64| {
        out.push_str(&format!("{}.{}.{}LUT.INIT[63:0] = 64'h{v:016X}\n", k.0, k.1, k.2));
    };
    for line in fasm.lines() {
        let key = match parse_line(line)? {
            Some(f) => lut_key(&f.feature),
            None => None,
        };
        let Some(key) = key else {
            out.push_str(line);
            out.push('\n');
            continue;
        };
        if !done.insert(key.clone()) {
            continue;
        }
        match fin.get(&key) {
            Some(&v) => {
                stats.written += 1;
                if v != 0 {
                    put(&mut out, &key, v);
                }
            }
            None if passes(theirs[&key]).is_some() => stats.routing_dropped += 1,
            None => {
                stats.cellless_kept += 1;
                put(&mut out, &key, theirs[&key]);
            }
        }
    }
    for (k, &v) in &fin {
        if !done.contains(k) {
            stats.written += 1;
            if v != 0 {
                put(&mut out, k, v);
            }
        }
    }
    Ok((out, stats))
}

/// The cells' INITs with `moves` made, and how many LUTs had a pin move.
pub fn final_inits(cells: &Inits, moves: &Moves) -> R<(Inits, usize)> {
    let mut moved = 0;
    let mut fin: Inits = BTreeMap::new();
    for (k, &init) in cells {
        let v = match moves.get(k) {
            Some(m) if m.iter().any(|m| m.from != [m.to]) => {
                moved += 1;
                permute(init, m)?
            }
            Some(m) => permute(init, m)?,
            None => init,
        };
        fin.insert(k.clone(), v);
    }
    Ok((fin, moved))
}

/// The FASM line of a LUT's INIT; none for 0 (nothing to set).
pub fn init_line(key: &LutKey, init: u64) -> Option<String> {
    (init != 0).then(|| format!("{}.{}.{}LUT.INIT[63:0] = 64'h{init:016X}", key.0, key.1, key.2))
}

/// The FASM name of a slice: in a CLBLM the first slice is the SLICEM.
pub fn slice_name(tile_type: &str, np_site: &str) -> Option<String> {
    let x = np_site.strip_prefix("SLICE_X")?.split('Y').next()?;
    match (tile_type.starts_with("CLBLM"), x) {
        (true, "0") => Some("SLICEM_X0".into()),
        (_, "0") => Some("SLICEL_X0".into()),
        (_, "1") => Some("SLICEL_X1".into()),
        _ => None,
    }
}

/// A LUT-type cell's logical inputs, address bit order (nextpnr-xilinx
/// `get_inputs`).
fn lut_inputs(orig_type: &str) -> Option<Vec<String>> {
    let n = |pre: &str, k: usize| (0..k).map(|i| format!("{pre}{i}")).collect::<Vec<_>>();
    Some(match orig_type {
        t if t.starts_with("LUT") => n("I", t[3..].parse().ok()?),
        "RAMD64E" | "RAMS64E" => n("RADR", 6),
        "RAMD32" | "RAMS32" => n("RADR", 5),
        "SRL16E" => n("A", 4),
        "SRLC32E" => (0..5).map(|i| format!("A[{i}]")).collect(),
        _ => return None,
    })
}

/// A cell's INIT parameter, LSB first: yosys writes a binary string with
/// the MSB first, or a number.
fn param_bits(v: &Value) -> Option<Vec<bool>> {
    match v {
        Value::String(s) if s.chars().all(|c| c == '0' || c == '1' || c == 'x') => {
            Some(s.chars().rev().map(|c| c == '1').collect())
        }
        Value::Number(n) => {
            let n = n.as_u64()?;
            Some((0..64).map(|i| n >> i & 1 == 1).collect())
        }
        _ => None,
    }
}

/// The INIT nextpnr writes for each LUT the placed design uses, by (tile,
/// FASM site, letter), from each LUT cell's logical INIT and the bel pins
/// its logical inputs were placed on (`X_ORIG_PORT_A1..A6`, several
/// inputs on one pin space-separated): physical address bit k reads every
/// logical input placed on A(k+1). A fractured LUT's 6LUT fills the upper
/// half and its 5LUT the lower; a LUT alone fills all 64 bits. As
/// nextpnr-xilinx `get_lut_init`, an input name it cannot place reads
/// logical input 0 and an address past the INIT reads 0.
///
/// `locate` resolves a cell's `NEXTPNR_BEL`. Returns the INITs and the LUT
/// cells it could not place, by reason.
#[oxibit_prof::hot]
pub fn cell_inits(
    placed: &Value,
    locate: &dyn Fn(&str) -> Option<Located>,
) -> R<(Inits, BTreeMap<String, usize>)> {
    let mods = placed["modules"].as_object().ok_or("no modules")?;
    let top = mods
        .values()
        .find(|m| m["attributes"]["top"].is_string() || m["attributes"]["top"].is_number())
        .or_else(|| mods.values().next())
        .ok_or("no top module")?;
    let cells = top["cells"].as_object().ok_or("no cells")?;

    // (tile, site, letter) -> (6LUT cell, 5LUT cell)
    let mut slots: BTreeMap<LutKey, (Option<&Value>, Option<&Value>)> = BTreeMap::new();
    let mut skipped: BTreeMap<String, usize> = BTreeMap::new();
    for c in cells.values() {
        let attrs = &c["attributes"];
        let Some(orig) = attrs["X_ORIG_TYPE"].as_str() else {
            continue;
        };
        if lut_inputs(orig).is_none() {
            continue;
        }
        let Some(bel) = attrs["NEXTPNR_BEL"].as_str() else {
            *skipped.entry("unplaced".into()).or_default() += 1;
            continue;
        };
        let Some((tile, tile_type, np_site, bel)) = locate(bel) else {
            *skipped.entry("bel_name".into()).or_default() += 1;
            continue;
        };
        let (Some(site), Some(letter), true) = (
            slice_name(&tile_type, &np_site),
            bel.chars().next(),
            bel.len() == 5 && bel.ends_with("LUT"),
        ) else {
            *skipped.entry(format!("bel {bel}")).or_default() += 1;
            continue;
        };
        let e = slots.entry((tile, site, letter)).or_default();
        match &bel[1..2] {
            "6" => e.0 = Some(c),
            "5" => e.1 = Some(c),
            _ => *skipped.entry(format!("bel {bel}")).or_default() += 1,
        }
    }

    let mut out = BTreeMap::new();
    for (key, (lut6, lut5)) in slots {
        let mut bits = 0u64;
        for (i, lut) in [lut6, lut5].into_iter().enumerate() {
            let Some(lut) = lut else { continue };
            let attrs = &lut["attributes"];
            let inputs =
                lut_inputs(attrs["X_ORIG_TYPE"].as_str().unwrap_or("")).unwrap_or_default();
            let log_to_bit: HashMap<&str, usize> = inputs
                .iter()
                .enumerate()
                .map(|(j, s)| (s.as_str(), j))
                .collect();
            let phys_to_log: Vec<Vec<&str>> = (1..=6)
                .map(|j| {
                    attrs[format!("X_ORIG_PORT_A{j}")]
                        .as_str()
                        .map(|s| s.split(' ').collect())
                        .unwrap_or_default()
                })
                .collect();
            let (lo, hi) = match (lut6.is_some() && lut5.is_some(), i) {
                (true, 0) => (32, 64),
                (true, _) => (0, 32),
                (false, _) => (0, 64),
            };
            let init = lut["parameters"]
                .get("INIT")
                .and_then(param_bits)
                .unwrap_or_default();
            for j in lo..hi {
                let mut log_index = 0usize;
                for (k, logs) in phys_to_log.iter().enumerate() {
                    if j >> k & 1 == 1 {
                        for l in logs {
                            log_index |= 1 << log_to_bit.get(l).copied().unwrap_or(0);
                        }
                    }
                }
                if init.get(log_index).copied().unwrap_or(false) {
                    bits |= 1 << j;
                }
            }
        }
        out.insert(key, bits);
    }
    Ok((out, skipped))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// INIT of f over pins (1-based), the other pins ignored.
    fn table(pins: &[u8], f: impl Fn(&[bool]) -> bool) -> u64 {
        let mut init = 0u64;
        for a in 0..64u64 {
            let ins: Vec<bool> = pins.iter().map(|&p| a >> (p - 1) & 1 == 1).collect();
            if f(&ins) {
                init |= 1 << a;
            }
        }
        init
    }

    fn mv(from: &[u8], to: u8) -> Move {
        Move {
            from: from.to_vec(),
            to,
        }
    }

    #[test]
    fn identity() {
        let init = 0x0123_4567_89AB_CDEF;
        assert_eq!(permute(init, &[]).unwrap(), init);
        assert_eq!(permute(init, &[mv(&[3], 3)]).unwrap(), init);
    }

    #[test]
    fn swap_two_inputs() {
        // f = A1 & !A2: after swapping 1 and 2 it reads A2 & !A1.
        let f = |x: &[bool]| x[0] && !x[1];
        let init = table(&[1, 2], f);
        let got = permute(init, &[mv(&[1], 2), mv(&[2], 1)]).unwrap();
        assert_eq!(got, table(&[2, 1], f));
    }

    #[test]
    fn move_to_unused_pin() {
        // A three-input LUT on A1..A3; A2's signal moves to A5.
        let f = |x: &[bool]| (x[0] ^ x[1]) | x[2];
        let init = table(&[1, 2, 3], f);
        let got = permute(init, &[mv(&[2], 5)]).unwrap();
        assert_eq!(got, table(&[1, 5, 3], f));
    }

    #[test]
    fn rotate_five() {
        let f = |x: &[bool]| (x[0] && x[1]) ^ (x[2] || !x[3]) ^ x[4];
        let init = table(&[1, 2, 3, 4, 5], f);
        let moves = [
            mv(&[1], 2),
            mv(&[2], 3),
            mv(&[3], 4),
            mv(&[4], 5),
            mv(&[5], 1),
        ];
        assert_eq!(permute(init, &moves).unwrap(), table(&[2, 3, 4, 5, 1], f));
    }

    #[test]
    fn fractured_halves_follow() {
        // O6 (A6 = 1) and O5 (the low half) over the same A1..A5: both
        // halves move together and A6 stays the half select.
        let o6 = table(&[1, 2, 3], |x| x[0] & x[1] & x[2]);
        let o5 = table(&[1, 2], |x| x[0] | x[1]);
        let init = (o6 & 0xFFFF_FFFF_0000_0000) | (o5 & 0x0000_0000_FFFF_FFFF);
        let got = permute(init, &[mv(&[1], 4), mv(&[3], 1)]).unwrap();
        let o6n = table(&[4, 2, 1], |x| x[0] & x[1] & x[2]);
        let o5n = table(&[4, 2], |x| x[0] | x[1]);
        assert_eq!(
            got,
            (o6n & 0xFFFF_FFFF_0000_0000) | (o5n & 0x0000_0000_FFFF_FFFF)
        );
    }

    #[test]
    fn one_signal_on_two_pins_collapses() {
        // The placer put net n on A1 and A3: f = A1 & A3 & A2 is n & A2.
        let init = table(&[1, 2, 3], |x| x[0] & x[1] & x[2]);
        let got = permute(init, &[mv(&[1, 3], 4)]).unwrap();
        assert_eq!(got, table(&[4, 2], |x| x[0] & x[1]));
    }

    #[test]
    fn rejects_clashes() {
        assert!(permute(0, &[mv(&[1], 2), mv(&[3], 2)]).is_err());
        assert!(permute(0, &[mv(&[1], 2), mv(&[1], 3)]).is_err());
        assert!(permute(0, &[mv(&[7], 2)]).is_err());
    }

    #[test]
    fn rewrite_lines() {
        let init = table(&[1, 2], |x| x[0] & !x[1]);
        let fasm = format!(
            "CLBLL_L_X2Y1.SLICEL_X0.AFF.ZINI\nCLBLL_L_X2Y1.SLICEL_X0.ALUT.INIT[31:0] = 32'h{:08X}\nCLBLL_L_X2Y1.SLICEL_X0.ALUT.INIT[63:32] = 32'h{:08X}\nCLBLL_L_X2Y1.SLICEL_X0.BLUT.INIT[63:0] = 64'h00000000000000FF\n",
            init & 0xFFFF_FFFF,
            init >> 32
        );
        let mut moves: Moves = BTreeMap::new();
        moves.insert(
            ("CLBLL_L_X2Y1".into(), "SLICEL_X0".into(), 'A'),
            vec![mv(&[1], 2), mv(&[2], 1)],
        );
        let (out, n) = rewrite_fasm(&fasm, &moves).unwrap();
        assert_eq!(n, 1);
        let want = table(&[2, 1], |x| x[0] & !x[1]);
        assert_eq!(
            out,
            format!(
                "CLBLL_L_X2Y1.SLICEL_X0.AFF.ZINI\nCLBLL_L_X2Y1.SLICEL_X0.ALUT.INIT[63:0] = 64'h{want:016X}\nCLBLL_L_X2Y1.SLICEL_X0.BLUT.INIT[63:0] = 64'h00000000000000FF\n"
            )
        );
    }

    #[test]
    fn apply_moves_and_drops_routing() {
        let key = |l: char| ("T".to_string(), "SLICEL_X0".to_string(), l);
        let f = |x: &[bool]| x[0] && !x[1];
        let mut cells: Inits = BTreeMap::new();
        cells.insert(key('A'), table(&[1, 2], f));
        let mut moves: Moves = BTreeMap::new();
        moves.insert(key('A'), vec![mv(&[1], 3), mv(&[2], 2)]);
        // nextpnr's FASM: A with its own INIT, B a routing LUT passing A4,
        // C a LUT with no cell that computes something.
        let fasm = "T.SLICEL_X0.ALUT.INIT[63:0] = 64'h0000000000000002\nT.SLICEL_X0.BLUT.INIT[63:0] = 64'hFF00FF00FF00FF00\nT.SLICEL_X0.CLUT.INIT[63:0] = 64'h0000000000000008\nT.SLICEL_X0.AFF.ZINI\n";
        let (out, st) = apply(fasm, &cells, &moves).unwrap();
        assert_eq!((st.written, st.moved, st.routing_dropped, st.cellless_kept), (1, 1, 1, 1));
        let want = table(&[3, 2], f);
        assert_eq!(out, format!("T.SLICEL_X0.ALUT.INIT[63:0] = 64'h{want:016X}\nT.SLICEL_X0.CLUT.INIT[63:0] = 64'h0000000000000008\nT.SLICEL_X0.AFF.ZINI\n"));
        assert_eq!(passes(0xFF00FF00FF00FF00), Some(4));
        assert_eq!(passes(0x0000000000000008), None);
    }

    fn locate(bel: &str) -> Option<Located> {
        let (site, bel) = bel.split_once('/')?;
        let tile_type = if site == "SLICE_X0Y0" {
            "CLBLM_R"
        } else {
            "CLBLL_L"
        };
        Some(("T".into(), tile_type.into(), site.into(), bel.into()))
    }

    #[test]
    fn cell_init_from_placement() {
        // LUT2 I0 & !I1 with I0 on A4 and I1 on A1; in a CLBLM's SLICEM.
        let v = serde_json::json!({"modules": {"top": {"attributes": {"top": 1}, "cells": {
            "a": {"type": "SLICE_LUTX", "attributes": {"X_ORIG_TYPE": "LUT2", "NEXTPNR_BEL": "SLICE_X0Y0/B6LUT",
                  "X_ORIG_PORT_A4": "I0", "X_ORIG_PORT_A1": "I1"}, "parameters": {"INIT": "0010"}}
        }}}});
        let (inits, skipped) = cell_inits(&v, &locate).unwrap();
        assert!(skipped.is_empty());
        assert_eq!(
            inits[&("T".into(), "SLICEM_X0".into(), 'B')],
            table(&[4, 1], |x| x[0] && !x[1])
        );
    }

    #[test]
    fn cell_init_fractured() {
        // 6LUT: I0 ^ I1 on A1, A2 (upper half); 5LUT: I0 | I1 on A3, A2.
        let v = serde_json::json!({"modules": {"top": {"attributes": {"top": 1}, "cells": {
            "x": {"type": "SLICE_LUTX", "attributes": {"X_ORIG_TYPE": "LUT2", "NEXTPNR_BEL": "SLICE_X1Y0/C6LUT",
                  "X_ORIG_PORT_A1": "I0", "X_ORIG_PORT_A2": "I1"}, "parameters": {"INIT": "0110"}},
            "o": {"type": "SLICE_LUTX", "attributes": {"X_ORIG_TYPE": "LUT2", "NEXTPNR_BEL": "SLICE_X1Y0/C5LUT",
                  "X_ORIG_PORT_A3": "I0", "X_ORIG_PORT_A2": "I1"}, "parameters": {"INIT": "1110"}}
        }}}});
        let (inits, _) = cell_inits(&v, &locate).unwrap();
        let want = (table(&[1, 2], |x| x[0] ^ x[1]) & 0xFFFF_FFFF_0000_0000)
            | (table(&[3, 2], |x| x[0] | x[1]) & 0xFFFF_FFFF);
        assert_eq!(inits[&("T".into(), "SLICEL_X1".into(), 'C')], want);
    }
}
