//! Binary AIGER ("aig") reader.
//!
//! Variables: 0 is constant false, then inputs, latches, AND gates, in
//! file order (topological for the ANDs). A literal is `2 * var + negated`.
//! Latches are kept as extra inputs: a checker that treats them as free is
//! conservative (it can only report more open outputs, never a false proof).

use std::io::{BufRead, Read};

#[derive(Debug, Default)]
pub struct Aig {
    pub inputs: u32,
    pub latches: u32,
    /// Fanin literals of AND variable `inputs + latches + 1 + i`.
    pub ands: Vec<(u32, u32)>,
    pub outputs: Vec<u32>,
}

impl Aig {
    pub fn num_vars(&self) -> usize {
        1 + self.inputs as usize + self.latches as usize + self.ands.len()
    }
    /// First AND variable.
    pub fn first_and(&self) -> u32 {
        1 + self.inputs + self.latches
    }
    pub fn fanins(&self, v: u32) -> Option<(u32, u32)> {
        let f = self.first_and();
        if v >= f { Some(self.ands[(v - f) as usize]) } else { None }
    }
}

fn varint(r: &mut impl Read) -> Result<u32, String> {
    let (mut x, mut shift) = (0u32, 0);
    loop {
        let mut b = [0u8];
        r.read_exact(&mut b).map_err(|e| format!("aiger: {e}"))?;
        x |= u32::from(b[0] & 0x7f) << shift;
        if b[0] & 0x80 == 0 {
            return Ok(x);
        }
        shift += 7;
    }
}

pub fn read(r: impl Read) -> Result<Aig, String> {
    let mut r = std::io::BufReader::new(r);
    let mut line = String::new();
    let mut next = |r: &mut std::io::BufReader<_>| -> Result<String, String> {
        line.clear();
        r.read_line(&mut line).map_err(|e| format!("aiger: {e}"))?;
        Ok(line.trim().to_string())
    };
    let head = next(&mut r)?;
    let f: Vec<&str> = head.split_whitespace().collect();
    if f.first() != Some(&"aig") || f.len() < 6 {
        return Err(format!("aiger: not a binary AIGER header: {head}"));
    }
    let n: Vec<u32> = f[1..]
        .iter()
        .map(|s| s.parse().map_err(|_| format!("aiger: bad header {head}")))
        .collect::<Result<_, _>>()?;
    let (m, i, l, o, a) = (n[0], n[1], n[2], n[3], n[4]);
    if m != i + l + a {
        return Err(format!("aiger: M != I + L + A in {head}"));
    }
    let extra: u32 = n.get(5..).map(|s| s.iter().sum()).unwrap_or(0);
    for _ in 0..l {
        next(&mut r)?;
    }
    let mut outputs = Vec::with_capacity(o as usize);
    for _ in 0..o {
        outputs.push(
            next(&mut r)?
                .split_whitespace()
                .next()
                .unwrap_or("0")
                .parse()
                .map_err(|_| "aiger: bad output")?,
        );
    }
    for _ in 0..extra {
        next(&mut r)?;
    }
    let mut ands = Vec::with_capacity(a as usize);
    for k in 0..a {
        let lhs = 2 * (i + l + 1 + k);
        let d0 = varint(&mut r)?;
        let d1 = varint(&mut r)?;
        let r0 = lhs - d0;
        let r1 = r0 - d1;
        ands.push((r0, r1));
    }
    Ok(Aig {
        inputs: i,
        latches: l,
        ands,
        outputs,
    })
}

/// Output names from a yosys `write_aiger -map` file: index -> "name[bit]".
pub fn output_names(map: &str) -> std::collections::HashMap<usize, String> {
    let mut out = std::collections::HashMap::new();
    for line in map.lines() {
        let f: Vec<&str> = line.split_whitespace().collect();
        if f.len() == 4 && f[0] == "output" {
            if let Ok(i) = f[1].parse() {
                out.insert(i, if f[2] == "0" { f[3].to_string() } else { format!("{}[{}]", f[3], f[2]) });
            }
        }
    }
    out
}
