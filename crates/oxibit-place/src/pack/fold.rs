//! Constant folding the router would otherwise pay for. Not in
//! nextpnr-xilinx.
//!
//! A LUT input tied to a constant is routed from a VCC_WIRE or GND_WIRE
//! tie. VCC reaches most inputs through the interconnect default and costs
//! nothing; GND takes GFAN and bypass wires the signal nets need. On the
//! PHY, 11,593 LUT inputs were tied to GND: the feed-through LUT1s that
//! carry a constant to a CARRY4's DI or S (4,482), the RAM32M read and
//! write addresses of 16-deep memories (2,728), and synthesised LUTs with
//! constant inputs (1,021).
//!
//! * `fold_luts`: a LUT input on a constant goes into the INIT (the
//!   cofactor) and off the cell. A LUT left with no inputs keeps `I0`
//!   unconnected and an INIT of all zeros or all ones.
//! * `dram_addr_high`: a RAM32M or RAM64M address bit tied low on every
//!   port moves to VCC when the INITs are uniform (all zeros or all ones),
//!   so no contents move.

use serde_json::Value;

use super::Packer;

fn init_bits(v: Option<&Value>, k: usize) -> Option<Vec<bool>> {
    let s = match v? {
        Value::String(s) => s.trim().to_string(),
        Value::Number(n) => format!("{:b}", n.as_u64()?),
        _ => return None,
    };
    if !s.chars().all(|c| c == '0' || c == '1') {
        return None;
    }
    let lsb_first: Vec<bool> = s.chars().rev().map(|c| c == '1').collect();
    Some((0..1usize << k).map(|i| lsb_first.get(i).copied().unwrap_or(false)).collect())
}

fn bits_string(bits: &[bool]) -> Value {
    Value::String(bits.iter().rev().map(|&b| if b { '1' } else { '0' }).collect())
}

/// The truth table with inputs `consts` (index, value) fixed, over the
/// inputs left (returned, in order).
fn cofactor(mut bits: Vec<bool>, k: usize, consts: &[(usize, bool)]) -> (Vec<bool>, Vec<usize>) {
    let mut live: Vec<usize> = (0..k).collect();
    for &(j, v) in consts.iter().rev() {
        let pos = live.iter().position(|&x| x == j).expect("live input");
        let mut next = vec![false; 1 << (live.len() - 1)];
        for (m, slot) in next.iter_mut().enumerate() {
            let lo = m & ((1 << pos) - 1);
            let hi = (m >> pos) << (pos + 1);
            *slot = bits[hi | (usize::from(v) << pos) | lo];
        }
        bits = next;
        live.remove(pos);
    }
    (bits, live)
}

/// Whether the table over `k` inputs reads input `j` at all.
fn reads(bits: &[bool], k: usize, j: usize) -> bool {
    (0..1usize << k).any(|i| i >> j & 1 == 0 && bits[i] != bits[i | 1 << j])
}

/// Fold constant inputs of `LUT1`..`LUT6` cells into their INIT, and take
/// off inputs the INIT does not read (synthesis leaves some: on the PHY a
/// select whose two arms agree, routed to four LUTs that ignore it).
pub fn fold_luts(p: &mut Packer) {
    let (gnd, vcc) = (p.nl.gnd, p.nl.vcc);
    let (mut inputs, mut cells, mut emptied, mut dead) = (0usize, 0usize, 0usize, 0usize);
    for c in p.nl.cell_ids() {
        let ty = p.nl.cell(c).ty.clone();
        let Some(k) = ty.strip_prefix("LUT").and_then(|k| k.parse::<usize>().ok()).filter(|k| (1..=6).contains(k)) else { continue };
        let mut consts: Vec<(usize, bool)> = (0..k)
            .filter_map(|j| match p.nl.cell(c).net(&format!("I{j}")) {
                Some(n) if n == gnd => Some((j, false)),
                Some(n) if n == vcc => Some((j, true)),
                _ => None,
            })
            .collect();
        let Some(table) = init_bits(p.nl.cell(c).params.get("INIT"), k) else { continue };
        let n_const = consts.len();
        // An input the table never reads folds like a constant (either value).
        for j in 0..k {
            if !consts.iter().any(|&(i, _)| i == j) && !reads(&table, k, j) {
                consts.push((j, false));
            }
        }
        if consts.is_empty() {
            continue;
        }
        dead += consts.len() - n_const;
        consts.sort_unstable();
        let (mut bits, live) = cofactor(table, k, &consts);
        // Rewire: the live inputs move down to I0..; the rest come off.
        let nets: Vec<_> = live.iter().map(|&j| p.nl.cell(c).net(&format!("I{j}"))).collect();
        for j in 0..k {
            p.nl.disconnect(c, &format!("I{j}"));
        }
        let k2 = live.len();
        for (i, n) in nets.into_iter().enumerate() {
            if let Some(n) = n {
                p.nl.connect(c, &format!("I{i}"), n);
            }
        }
        if k2 == 0 {
            // No inputs left: a LUT1 whose output ignores its (open) input.
            let v = bits[0];
            bits = vec![v, v];
            p.nl.cell_mut(c).ty = "LUT1".into();
            emptied += 1;
        } else {
            p.nl.cell_mut(c).ty = format!("LUT{k2}");
        }
        for j in k2.max(1)..k {
            p.nl.remove_port(c, &format!("I{j}"));
        }
        p.nl.cell_mut(c).params.insert("INIT".into(), bits_string(&bits));
        inputs += n_const;
        cells += 1;
    }
    p.count("fold.lut_inputs", inputs);
    p.count("fold.luts", cells);
    p.count("fold.luts_constant", emptied);
    p.count("fold.lut_unread_inputs", dead);
}

/// Tied-low RAM32M/RAM64M address bits to VCC where no contents move.
pub fn dram_addr_high(p: &mut Packer) {
    let (gnd, vcc) = (p.nl.gnd, p.nl.vcc);
    let mut moved = 0usize;
    for c in p.nl.cell_ids() {
        let ty = p.nl.cell(c).ty.clone();
        let width = match ty.as_str() {
            "RAM32M" => 5,
            "RAM64M" => 6,
            _ => continue,
        };
        let uniform = ["INIT_A", "INIT_B", "INIT_C", "INIT_D"].iter().all(|k| match p.nl.cell(c).params.get(*k) {
            None => true,
            Some(Value::String(s)) => {
                let t = s.trim();
                t.chars().all(|ch| ch == '0') || t.chars().all(|ch| ch == '1')
            }
            Some(Value::Number(n)) => n.as_u64() == Some(0),
            _ => false,
        });
        if !uniform {
            continue;
        }
        for i in 0..width {
            let ports: Vec<String> = ["ADDRA", "ADDRB", "ADDRC", "ADDRD"].iter().map(|a| format!("{a}[{i}]")).collect();
            if ports.iter().all(|q| p.nl.cell(c).net(q) == Some(gnd)) {
                for q in &ports {
                    p.nl.connect(c, q, vcc);
                }
                moved += 1;
            }
        }
    }
    p.count("fold.dram_addr_high", moved);
}

#[cfg(test)]
mod tests {
    use super::cofactor;

    /// Every fold agrees with the full table on every input.
    #[test]
    fn cofactor_matches_table() {
        let mut seed = 0x9e3779b97f4a7c15u64;
        let mut rnd = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            seed
        };
        for _ in 0..2000 {
            let k = 1 + (rnd() % 6) as usize;
            let bits: Vec<bool> = (0..1usize << k).map(|_| rnd() & 1 == 1).collect();
            let mut consts: Vec<(usize, bool)> = Vec::new();
            for j in 0..k {
                if rnd() % 3 == 0 {
                    consts.push((j, rnd() & 1 == 1));
                }
            }
            let (folded, live) = cofactor(bits.clone(), k, &consts);
            for (i, &j) in live.iter().enumerate() {
                // A kept input the full table never reads stays unread.
                if !super::reads(&bits, k, j) {
                    assert!(!super::reads(&folded, live.len(), i));
                }
            }
            assert_eq!(folded.len(), 1 << live.len());
            for full in 0..1usize << k {
                if consts.iter().any(|&(j, v)| (full >> j & 1 == 1) != v) {
                    continue;
                }
                let m = live.iter().enumerate().fold(0, |m, (i, &j)| m | ((full >> j & 1) << i));
                assert_eq!(folded[m], bits[full]);
            }
        }
    }
}
