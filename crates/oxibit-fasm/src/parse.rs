//! FASM lines.
//!
//! ```text
//! TILE.SITE.FEATURE                      # set, value 1
//! TILE.SITE.FEATURE[7]                   # one address
//! TILE.SITE.INIT[63:0] = 64'b0101...     # a range, value LSB at the low address
//! TILE.SITE.FEATURE = 1'b0               # value 0: nothing is set
//! { annotation = "x" }                   # annotations are ignored
//! ```
//!
//! [`SetFeature::addresses`] is `fasm.canonical_features`: the addresses
//! whose bit is set, address 0 standing for "no address".

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SetFeature {
    /// Full dotted name, tile first, without the address.
    pub feature: String,
    /// `[start]` or `[end:start]`.
    pub start: Option<u32>,
    pub end: Option<u32>,
    /// Value bits, LSB first. A feature with no `= value` is `[true]`.
    pub value: Vec<bool>,
}

impl SetFeature {
    pub fn is_zero(&self) -> bool {
        !self.value.iter().any(|&b| b)
    }

    /// The addresses to enable, as `fasm.canonical_features` yields them.
    pub fn addresses(&self) -> Vec<u32> {
        if self.is_zero() {
            return vec![];
        }
        match (self.start, self.end) {
            (None, _) => vec![0],
            (Some(s), None) => vec![s],
            (Some(s), Some(e)) => (s..=e)
                .filter(|a| self.value.get((a - s) as usize).copied().unwrap_or(false))
                .collect(),
        }
    }
}

/// Parse one line. `Ok(None)` for blank, comment-only and annotation-only lines.
pub fn parse_line(raw: &str) -> Result<Option<SetFeature>, String> {
    let line = strip_annotations_and_comment(raw);
    let line = line.trim();
    if line.is_empty() {
        return Ok(None);
    }

    let (lhs, value) = match line.split_once('=') {
        Some((l, v)) => (l.trim(), Some(parse_value(v.trim()).map_err(|e| format!("{e}: {raw}"))?)),
        None => (line, None),
    };

    let (feature, start, end) = match lhs.find('[') {
        Some(i) => {
            let close = lhs.rfind(']').filter(|&c| c > i).ok_or_else(|| format!("unclosed address: {raw}"))?;
            let inner = &lhs[i + 1..close];
            let num = |s: &str| s.trim().parse::<u32>().map_err(|_| format!("bad address: {raw}"));
            match inner.split_once(':') {
                Some((hi, lo)) => (&lhs[..i], Some(num(lo)?), Some(num(hi)?)),
                None => (&lhs[..i], Some(num(inner)?), None),
            }
        }
        None => (lhs, None, None),
    };

    if feature.is_empty() || feature.contains(char::is_whitespace) {
        return Err(format!("bad feature: {raw}"));
    }
    if let (Some(s), Some(e)) = (start, end) {
        if e < s {
            return Err(format!("reversed range: {raw}"));
        }
    }

    Ok(Some(SetFeature { feature: feature.to_string(), start, end, value: value.unwrap_or_else(|| vec![true]) }))
}

/// Drop `{ ... }` annotation blocks (which may contain '#' inside quotes)
/// and anything after a '#' outside them.
fn strip_annotations_and_comment(raw: &str) -> String {
    let mut out = String::with_capacity(raw.len());
    let mut in_braces = false;
    let mut in_quotes = false;
    for c in raw.chars() {
        match (in_braces, in_quotes, c) {
            (true, _, '"') => in_quotes = !in_quotes,
            (true, false, '}') => in_braces = false,
            (true, _, _) => {}
            (false, _, '{') => in_braces = true,
            (false, _, '#') => break,
            (false, _, c) => out.push(c),
        }
    }
    out
}

/// Verilog-style constants: `N'bxxx`, `N'hxx`, `N'oxx`, `N'dxx`, `'b..`, or plain decimal.
fn parse_value(v: &str) -> Result<Vec<bool>, String> {
    let v: String = v.chars().filter(|&c| c != '_').collect();
    let (width, radix, digits) = match v.find('\'') {
        Some(i) => {
            let width = if i == 0 { None } else { Some(v[..i].parse::<usize>().map_err(|_| "bad width")?) };
            let rest = &v[i + 1..];
            let mut cs = rest.chars();
            let radix = match cs.next().map(|c| c.to_ascii_lowercase()) {
                Some('b') => 2,
                Some('o') => 8,
                Some('h') => 16,
                Some('d') => 10,
                _ => return Err("bad radix".into()),
            };
            (width, radix, cs.as_str().to_string())
        }
        None => (None, 10, v.clone()),
    };
    if digits.is_empty() {
        return Err("empty value".into());
    }

    let mut bits: Vec<bool> = match radix {
        2 | 8 | 16 => {
            let per = match radix {
                2 => 1,
                8 => 3,
                _ => 4,
            };
            let mut bits = Vec::with_capacity(digits.len() * per);
            for c in digits.chars().rev() {
                let d = c.to_digit(radix).ok_or("bad digit")?;
                for k in 0..per {
                    bits.push((d >> k) & 1 == 1);
                }
            }
            bits
        }
        _ => {
            let n: u128 = digits.parse().map_err(|_| "bad decimal")?;
            (0..128).map(|k| (n >> k) & 1 == 1).collect()
        }
    };
    if let Some(w) = width {
        bits.truncate(w);
    }
    Ok(bits)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plain_and_ranges() {
        let f = parse_line("T.S.F").unwrap().unwrap();
        assert_eq!(f.addresses(), vec![0]);
        let f = parse_line("T.S.INIT[3:0] = 4'b1010 # c").unwrap().unwrap();
        assert_eq!((f.start, f.end), (Some(0), Some(3)));
        assert_eq!(f.addresses(), vec![1, 3]);
        let f = parse_line("T.S.F[5]").unwrap().unwrap();
        assert_eq!(f.addresses(), vec![5]);
        assert!(parse_line("T.S.F = 1'b0").unwrap().unwrap().addresses().is_empty());
        assert_eq!(parse_line("# only").unwrap(), None);
        assert_eq!(parse_line("{ a = \"#x\" }").unwrap(), None);
        let f = parse_line("T.S.V[11:4] = 8'hA5").unwrap().unwrap();
        assert_eq!(f.addresses(), vec![4, 6, 9, 11]);
    }
}
