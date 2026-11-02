//! And-inverter graph with structural hashing.
//!
//! Node 0 is constant false. A literal is `node << 1 | complement`. Nodes
//! are created in topological order: an AND node's fanins always have
//! smaller indices, so a forward walk over `nodes` is a valid evaluation
//! order.

use std::collections::HashMap;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Lit(pub u32);

impl Lit {
    pub const FALSE: Lit = Lit(0);
    pub const TRUE: Lit = Lit(1);
    pub fn node(self) -> u32 {
        self.0 >> 1
    }
    pub fn compl(self) -> bool {
        self.0 & 1 == 1
    }
    pub fn not(self) -> Lit {
        Lit(self.0 ^ 1)
    }
    pub fn not_if(self, c: bool) -> Lit {
        Lit(self.0 ^ u32::from(c))
    }
    pub fn positive(self) -> Lit {
        Lit(self.0 & !1)
    }
    pub fn is_const(self) -> bool {
        self.0 < 2
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Node {
    Const,
    /// A primary input (cut point): its index in `Aig::inputs`.
    Input(u32),
    And(Lit, Lit),
}

/// A carry chain (CARRY4 cells) computing `a + b + cin` outside the AIG:
/// `s[i] = a[i] ^ b[i]` and `di[i]` are AIG outputs (LUT-mapped), `o` and
/// `co` are AIG inputs.
#[derive(Debug, Clone)]
pub struct Chain {
    pub s: Vec<Lit>,
    pub di: Vec<Lit>,
    pub cin: Lit,
    pub o: Vec<Lit>,
    pub co: Lit,
}

#[derive(Debug, Default)]
pub struct Aig {
    pub nodes: Vec<Node>,
    /// Node index of each primary input.
    pub inputs: Vec<u32>,
    strash: HashMap<(Lit, Lit), u32>,
    /// Adders at least this wide become carry chains (0: never).
    pub carry_min: usize,
    /// LUT inputs of the target (sizes comparator chunks).
    pub lut_k: usize,
    pub chains: Vec<Chain>,
}

impl Aig {
    pub fn new() -> Self {
        Aig {
            nodes: vec![Node::Const],
            inputs: Vec::new(),
            strash: HashMap::new(),
            carry_min: 0,
            lut_k: 6,
            chains: Vec::new(),
        }
    }

    pub fn input(&mut self) -> Lit {
        let n = self.nodes.len() as u32;
        self.nodes.push(Node::Input(self.inputs.len() as u32));
        self.inputs.push(n);
        Lit(n << 1)
    }

    pub fn and(&mut self, a: Lit, b: Lit) -> Lit {
        let (a, b) = if a <= b { (a, b) } else { (b, a) };
        if a == Lit::FALSE || a == b.not() {
            return Lit::FALSE;
        }
        if a == Lit::TRUE || a == b {
            return b;
        }
        if let Some(&n) = self.strash.get(&(a, b)) {
            return Lit(n << 1);
        }
        let n = self.nodes.len() as u32;
        self.nodes.push(Node::And(a, b));
        self.strash.insert((a, b), n);
        Lit(n << 1)
    }

    pub fn or(&mut self, a: Lit, b: Lit) -> Lit {
        self.and(a.not(), b.not()).not()
    }

    pub fn xor(&mut self, a: Lit, b: Lit) -> Lit {
        if a.is_const() {
            return b.not_if(a == Lit::TRUE);
        }
        if b.is_const() {
            return a.not_if(b == Lit::TRUE);
        }
        let x = self.and(a, b.not());
        let y = self.and(a.not(), b);
        self.or(x, y)
    }

    /// `s ? b : a`.
    pub fn mux(&mut self, s: Lit, a: Lit, b: Lit) -> Lit {
        if s == Lit::TRUE || a == b {
            return b;
        }
        if s == Lit::FALSE {
            return a;
        }
        let x = self.and(s, b);
        let y = self.and(s.not(), a);
        self.or(x, y)
    }

    pub fn and_all(&mut self, v: &[Lit]) -> Lit {
        // Balanced, so depth is log2.
        match v.len() {
            0 => Lit::TRUE,
            1 => v[0],
            n => {
                let l = self.and_all(&v[..n / 2]);
                let r = self.and_all(&v[n / 2..]);
                self.and(l, r)
            }
        }
    }

    pub fn or_all(&mut self, v: &[Lit]) -> Lit {
        let inv: Vec<Lit> = v.iter().map(|l| l.not()).collect();
        self.and_all(&inv).not()
    }

    pub fn xor_all(&mut self, v: &[Lit]) -> Lit {
        match v.len() {
            0 => Lit::FALSE,
            1 => v[0],
            n => {
                let l = self.xor_all(&v[..n / 2]);
                let r = self.xor_all(&v[n / 2..]);
                self.xor(l, r)
            }
        }
    }

    pub fn ands(&self) -> usize {
        self.nodes.iter().filter(|n| matches!(n, Node::And(..))).count()
    }

    /// Evaluate every node for 64 parallel input patterns (one u64 per
    /// input). Used by tests and the random-simulation checks.
    pub fn simulate(&self, inputs: &[u64]) -> Vec<u64> {
        let mut v = vec![0u64; self.nodes.len()];
        for (i, n) in self.nodes.iter().enumerate() {
            v[i] = match *n {
                Node::Const => 0,
                Node::Input(k) => inputs[k as usize],
                Node::And(a, b) => lit_val(&v, a) & lit_val(&v, b),
            };
        }
        v
    }
}

pub fn lit_val(v: &[u64], l: Lit) -> u64 {
    let x = v[l.node() as usize];
    if l.compl() { !x } else { x }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn strash_and_simplify() {
        let mut g = Aig::new();
        let a = g.input();
        let b = g.input();
        assert_eq!(g.and(a, b), g.and(b, a));
        assert_eq!(g.and(a, a.not()), Lit::FALSE);
        assert_eq!(g.and(a, Lit::TRUE), a);
        let x = g.xor(a, b);
        let v = g.simulate(&[0b1100, 0b1010]);
        assert_eq!(lit_val(&v, x) & 0xf, 0b0110);
        let m = g.mux(a, b, Lit::TRUE);
        let v = g.simulate(&[0b1100, 0b1010]);
        assert_eq!(lit_val(&v, m) & 0xf, 0b1110);
    }
}
