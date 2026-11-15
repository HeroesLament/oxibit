//! The FASM subset prjoxide reads.
//!
//! The first `.`-separated part of a feature is the tile (or tilegroup, or
//! `IP_<region>`), with `__` standing for `:`. Then:
//! - `PIP.<to>.<from>`: a pip (last one for a `<to>` wins);
//! - `UNKNOWN.<frame>.<bit>`: a raw bit;
//! - `<name>[hi:lo] = <value>` or `<name>[i]`: bits of a word;
//! - anything else: an enum, `<name>.<option>`, the name being every part
//!   but the last.
//!
//! `{ key = value, ... }` lines are attributes (`oxide.device`,
//! `oxide.device_variant`, `oxide.meta`).

use std::collections::BTreeMap;

use crate::{Result, err};

/// An unsigned value of any width, bit 0 first.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Value(Vec<bool>);

impl Value {
    pub fn from_u64(mut v: u64) -> Value {
        let mut b = Vec::new();
        while v != 0 {
            b.push(v & 1 == 1);
            v >>= 1;
        }
        Value(b)
    }

    pub fn from_bits(bits: &[bool]) -> Value {
        Value(bits.to_vec())
    }

    pub fn bit(&self, i: usize) -> bool {
        self.0.get(i).copied().unwrap_or(false)
    }

    pub fn set_bit(&mut self, i: usize, v: bool) {
        if i >= self.0.len() {
            if !v {
                return;
            }
            self.0.resize(i + 1, false);
        }
        self.0[i] = v;
    }

    /// Position of the highest set bit plus one; 0 for zero.
    pub fn bits(&self) -> usize {
        self.0.iter().rposition(|&b| b).map_or(0, |p| p + 1)
    }

    /// Digits in `radix` (2, 8, 10 or 16), `_` ignored.
    pub fn parse_radix(digits: &str, radix: u32) -> Option<Value> {
        // Little-endian base 2^32 limbs; multiply-add per digit.
        let mut limbs: Vec<u32> = Vec::new();
        let mut any = false;
        for c in digits.chars() {
            if c == '_' {
                continue;
            }
            let d = c.to_digit(radix)?;
            any = true;
            let mut carry = d as u64;
            for l in limbs.iter_mut() {
                let v = (*l as u64) * radix as u64 + carry;
                *l = v as u32;
                carry = v >> 32;
            }
            if carry != 0 {
                limbs.push(carry as u32);
            }
        }
        if !any {
            return None;
        }
        let mut bits = Vec::with_capacity(limbs.len() * 32);
        for l in limbs {
            for i in 0..32 {
                bits.push((l >> i) & 1 == 1);
            }
        }
        Some(Value(bits))
    }
}

#[derive(Default, Debug)]
pub struct FasmTile {
    pub pips: BTreeMap<String, String>,
    pub enums: BTreeMap<String, String>,
    pub words: BTreeMap<String, Value>,
    pub unknowns: Vec<(usize, usize)>,
}

#[derive(Default, Debug)]
pub struct ParsedFasm {
    pub attrs: Vec<(String, String)>,
    pub tiles: BTreeMap<String, FasmTile>,
}

struct Cur<'a> {
    s: &'a str,
    line: usize,
}

impl<'a> Cur<'a> {
    fn take(&mut self, keep: impl Fn(char) -> bool) -> &'a str {
        let end = self.s.find(|c: char| !keep(c)).unwrap_or(self.s.len());
        let (t, rest) = self.s.split_at(end);
        self.s = rest;
        t
    }
    fn ident(&mut self) -> &'a str {
        self.take(|c| c.is_ascii_alphanumeric() || c == '_')
    }
    fn attr_key(&mut self) -> &'a str {
        self.take(|c| c.is_ascii_alphanumeric() || c == '_' || c == '.')
    }
    fn ws(&mut self) {
        self.take(char::is_whitespace);
    }
    fn eat(&mut self, tok: &str) -> bool {
        match self.s.strip_prefix(tok) {
            Some(rest) => {
                self.s = rest;
                true
            }
            None => false,
        }
    }
    fn expect(&mut self, tok: &str) -> Result<()> {
        if self.eat(tok) { Ok(()) } else { Err(err!("line {}: expected {tok}", self.line)) }
    }
    fn int(&mut self) -> Result<u64> {
        let line = self.line;
        let t = self.take(|c| c.is_ascii_digit());
        t.parse().map_err(|_| err!("line {line}: expected a number"))
    }
    fn value(&mut self) -> Result<Value> {
        self.ws();
        let n = self.int()?;
        self.ws();
        if !self.eat("'") {
            return Ok(Value::from_u64(n));
        }
        self.ws();
        let radix = match self.s.chars().next() {
            Some('b') => 2,
            Some('o') => 8,
            Some('d') => 10,
            Some('h') => 16,
            other => return Err(err!("line {}: unsupported base {other:?}", self.line)),
        };
        self.s = &self.s[1..];
        let digits = self.ident();
        Value::parse_radix(digits, radix)
            .ok_or_else(|| err!("line {}: bad digits {digits:?}", self.line))
    }
    fn attr_value(&mut self) -> Result<String> {
        if self.eat("\"") {
            let end = self.s.find('"').ok_or_else(|| err!("line {}: unterminated string", self.line))?;
            let v = &self.s[..end];
            self.s = &self.s[end + 1..];
            Ok(v.to_string())
        } else {
            Ok(self.attr_key().to_string())
        }
    }
}

impl ParsedFasm {
    pub fn attr(&self, key: &str) -> Option<&str> {
        self.attrs.iter().find(|(k, _)| k == key).map(|(_, v)| v.as_str())
    }

    pub fn parse(text: &str) -> Result<ParsedFasm> {
        let mut p = ParsedFasm::default();
        for (i, l) in text.lines().enumerate() {
            let body = &l[..l.find('#').unwrap_or(l.len())];
            let body = body.trim_start();
            if body.trim().is_empty() {
                continue;
            }
            let mut c = Cur { s: body, line: i + 1 };
            if c.eat("{") {
                c.ws();
                while !c.eat("}") {
                    let key = c.attr_key().to_string();
                    c.ws();
                    c.expect("=")?;
                    c.ws();
                    let v = c.attr_value()?;
                    p.attrs.push((key, v));
                    c.ws();
                    if !c.eat(",") {
                        c.expect("}")?;
                        break;
                    }
                    c.ws();
                }
                continue;
            }
            let tile = c.ident().replace("__", ":");
            let t = p.tiles.entry(tile).or_default();
            c.expect(".")?;
            if c.eat("PIP.") {
                let to = c.ident().replace("__", ":");
                c.expect(".")?;
                let from = c.ident().replace("__", ":");
                t.pips.insert(to, from);
            } else if c.eat("UNKNOWN.") {
                let f = c.int()? as usize;
                c.expect(".")?;
                let b = c.int()? as usize;
                t.unknowns.push((f, b));
            } else {
                let mut parts = Vec::new();
                loop {
                    parts.push(c.ident());
                    if !c.eat(".") {
                        break;
                    }
                }
                c.ws();
                if c.eat("[") {
                    c.ws();
                    let key = parts.join(".");
                    let hi = c.int()?;
                    c.ws();
                    let mut lo = hi;
                    if c.eat(":") {
                        c.ws();
                        lo = c.int()?;
                        c.ws();
                    }
                    if hi < lo {
                        return Err(err!("line {}: [{hi}:{lo}] is reversed", i + 1));
                    }
                    c.expect("]")?;
                    c.ws();
                    let v = if c.eat("=") { c.value()? } else { Value::from_u64(1) };
                    let dest = t.words.entry(key).or_default();
                    for k in 0..=(hi - lo) {
                        dest.set_bit((lo + k) as usize, v.bit(k as usize));
                    }
                } else {
                    let (opt, name) = parts.split_last().ok_or_else(|| err!("line {}: empty feature", i + 1))?;
                    t.enums.insert(name.join("."), opt.to_string());
                }
            }
        }
        Ok(p)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn features_of_each_kind_parse() {
        let f = ParsedFasm::parse(
            "{ oxide.device=\"LIFCL-40\" }\n\
             R2C3__PLC.PIP.E1__H02E0001.JF0 # c\n\
             R2C3__PLC.SLICEA.K0.INIT[15:0] = 16'hBEEF\n\
             R2C3__PLC.SLICEA.REG0.SD.0\n\
             R2C3__PLC.UNKNOWN.4.5\n\
             R2C3__PLC.W[3]\n",
        )
        .unwrap();
        assert_eq!(f.attr("oxide.device"), Some("LIFCL-40"));
        let t = &f.tiles["R2C3:PLC"];
        assert_eq!(t.pips["E1:H02E0001"], "JF0");
        assert_eq!(t.enums["SLICEA.REG0.SD"], "0");
        assert_eq!(t.unknowns, vec![(4, 5)]);
        let w = &t.words["SLICEA.K0.INIT"];
        assert_eq!(w.bits(), 16);
        assert!(w.bit(0) && w.bit(1) && w.bit(2) && w.bit(3) && !w.bit(4));
        assert!(t.words["W"].bit(3) && t.words["W"].bits() == 4);
    }

    #[test]
    fn a_wide_decimal_value_is_exact() {
        let v = Value::parse_radix("18446744073709551617", 10).unwrap(); // 2^64 + 1
        assert!(v.bit(0) && v.bit(64) && v.bits() == 65);
        assert!((1..64).all(|i| !v.bit(i)));
    }
}
