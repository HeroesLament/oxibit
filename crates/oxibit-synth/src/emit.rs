//! Build the mapped netlist as a `Module` of xc7 primitive instances
//! (LUT1..LUT6, FDRE/FDSE/FDCE/FDPE), with memories and the design's own
//! primitive instances passed through. `oxibit_hdl::yosys_json::write`
//! then writes it like any other module.

use std::collections::{BTreeMap, HashMap};

use oxibit_hdl::netlist::{Bit, Cell, Dir, MemRead, MemWrite, Memory, Module, Param, SigSpec, Signal};
use oxibit_prof::hot;

use crate::aig::{Lit, Node};
use crate::lower::{FfKind, Lowered};
use crate::map::Mapping;

#[derive(Debug, Default, Clone)]
pub struct Stats {
    pub luts: [usize; 7],
    pub inverters: usize,
    pub carry4: usize,
    pub ffs: BTreeMap<&'static str, usize>,
    pub named: usize,
}

struct Emit<'a> {
    lw: &'a Lowered,
    mp: &'a Mapping,
    out: Module,
    net: HashMap<Lit, Bit>,
    stats: Stats,
}

impl<'a> Emit<'a> {
    fn fresh(&mut self) -> Bit {
        let n = self.out.signals.len();
        let id = self.out.add_signal(Signal {
            name: format!("$n{n}"),
            width: 1,
            signed: false,
            port: None,
            init: None,
        });
        Bit::Sig(id, 0)
    }

    fn lut(&mut self, leaves: &[Bit], tt: u64, o: Bit) {
        let k = leaves.len();
        let bits: Vec<Option<bool>> = (0..1usize << k).map(|i| Some((tt >> i) & 1 == 1)).collect();
        let mut conns = BTreeMap::new();
        for (i, b) in leaves.iter().enumerate() {
            conns.insert(format!("I{i}"), SigSpec(vec![*b]));
        }
        conns.insert("O".into(), SigSpec(vec![o]));
        self.stats.luts[k] += 1;
        let name = format!("oxsyn_lut{}", self.out.cells.len());
        let params = BTreeMap::from([("INIT".to_string(), Param::Bits(bits))]);
        self.out.cells.push(Cell::Instance {
            ty: format!("LUT{k}"),
            name,
            params,
            conns,
        });
    }

    /// The net carrying `l`, adding an inverting LUT for a complemented
    /// literal the first time it is asked for.
    fn of(&mut self, l: Lit) -> Bit {
        if l == Lit::FALSE {
            return Bit::Zero;
        }
        if l == Lit::TRUE {
            return Bit::One;
        }
        if let Some(b) = self.net.get(&l) {
            return *b;
        }
        let p = l.positive();
        let o = self.fresh();
        let lw = self.lw;
        let mp = self.mp;
        match (lw.aig.nodes[p.node() as usize], mp.lut_of[p.node() as usize]) {
            (_, Some(i)) => {
                let lut = &mp.luts[i as usize];
                let leaves: Vec<Bit> = lut.leaves.iter().map(|n| self.net[&Lit(n << 1)]).collect();
                let tt = !lut.tt;
                self.lut(&leaves, tt, o);
            }
            (Node::Input(_), None) => {
                let a = self.of(p);
                self.stats.inverters += 1;
                self.lut(&[a], 0b01, o);
            }
            (n, None) => panic!("literal {l:?} ({n:?}) is not in the cover"),
        }
        self.net.insert(l, o);
        o
    }

    fn spec(&mut self, v: &[Lit]) -> SigSpec {
        SigSpec(v.iter().map(|l| self.of(*l)).collect())
    }
}

#[hot]
pub fn emit(lw: &Lowered, mp: &Mapping) -> (Module, Stats) {
    let mut e = Emit {
        lw,
        mp,
        out: Module::new(&lw.name),
        net: HashMap::new(),
        stats: Stats::default(),
    };
    // Ports. Input bits are the nets of their AIG inputs.
    let mut outs = Vec::new();
    for (name, dir, signed, bits) in &lw.ports {
        let id = e.out.add_signal(Signal {
            name: name.clone(),
            width: bits.len() as u32,
            signed: *signed,
            port: Some(*dir),
            init: None,
        });
        match dir {
            Dir::Output => outs.push((id, bits.clone())),
            _ => {
                for (i, l) in bits.iter().enumerate() {
                    if !l.is_const() && !l.compl() {
                        e.net.entry(*l).or_insert(Bit::Sig(id, i as u32));
                    }
                }
            }
        }
    }
    // Every other AIG input (flop, memory and instance outputs) and every
    // LUT root gets a net up front.
    for &n in &lw.aig.inputs {
        let l = Lit(n << 1);
        if !e.net.contains_key(&l) && !lw.dead.contains(&n) {
            let b = e.fresh();
            e.net.insert(l, b);
        }
    }
    for lut in &mp.luts {
        let b = e.fresh();
        e.net.insert(Lit(lut.root << 1), b);
    }
    for i in 0..mp.luts.len() {
        let lut = &mp.luts[i];
        let leaves: Vec<Bit> = lut.leaves.iter().map(|n| e.net[&Lit(n << 1)]).collect();
        let o = e.net[&Lit(lut.root << 1)];
        e.lut(&leaves, lut.tt, o);
    }
    for (id, bits) in outs {
        let s = e.spec(&bits);
        e.out.connects.push((SigSpec::of(id, bits.len() as u32), s));
    }
    // Flops.
    for (k, f) in lw.ffs.iter().enumerate() {
        let (ty, srp, inv) = match f.kind {
            FfKind::Fdre => ("FDRE", "R", "IS_R_INVERTED"),
            FfKind::Fdse => ("FDSE", "S", "IS_S_INVERTED"),
            FfKind::Fdce => ("FDCE", "CLR", "IS_CLR_INVERTED"),
            FfKind::Fdpe => ("FDPE", "PRE", "IS_PRE_INVERTED"),
        };
        *e.stats.ffs.entry(ty).or_default() += 1;
        let mut conns = BTreeMap::new();
        conns.insert("C".to_string(), SigSpec(vec![e.of(f.clk)]));
        conns.insert("CE".to_string(), SigSpec(vec![e.of(f.ce)]));
        conns.insert("D".to_string(), SigSpec(vec![e.of(f.d)]));
        conns.insert(srp.to_string(), SigSpec(vec![e.of(f.sr)]));
        conns.insert("Q".to_string(), SigSpec(vec![e.net[&f.q]]));
        let mut params = BTreeMap::from([("INIT".to_string(), Param::Bits(vec![Some(f.init)]))]);
        if f.clk_inv {
            params.insert("IS_C_INVERTED".into(), Param::Bits(vec![Some(true)]));
        }
        if f.sr_inv {
            params.insert(inv.into(), Param::Bits(vec![Some(true)]));
        }
        let name = format!("oxsyn_ff{k}${}", f.name);
        e.out.cells.push(Cell::Instance {
            ty: ty.into(),
            name,
            params,
            conns,
        });
    }
    // Carry chains: one CARRY4 per 4 bits; the first takes a constant
    // carry-in on CYINIT, a signal on CI.
    for (ci, ch) in lw.aig.chains.iter().enumerate() {
        let n = ch.s.len();
        let mut carry = Bit::Zero;
        for (k, lo) in (0..n).step_by(4).enumerate() {
            let w = (n - lo).min(4);
            let mut s = Vec::with_capacity(4);
            let mut di = Vec::with_capacity(4);
            let mut o = Vec::with_capacity(4);
            let mut co = Vec::with_capacity(4);
            for j in 0..4 {
                if j < w {
                    s.push(e.of(ch.s[lo + j]));
                    di.push(e.of(ch.di[lo + j]));
                    o.push(e.net[&ch.o[lo + j]]);
                } else {
                    s.push(Bit::Zero);
                    di.push(Bit::Zero);
                    o.push(e.fresh());
                }
                co.push(if lo + j == n - 1 { e.net[&ch.co] } else { e.fresh() });
            }
            let (ci_b, cyinit) = if k == 0 {
                if ch.cin.is_const() {
                    (Bit::Zero, e.of(ch.cin))
                } else {
                    (e.of(ch.cin), Bit::Zero)
                }
            } else {
                (carry, Bit::Zero)
            };
            carry = co[3];
            let conns = BTreeMap::from([
                ("CI".to_string(), SigSpec(vec![ci_b])),
                ("CYINIT".to_string(), SigSpec(vec![cyinit])),
                ("DI".to_string(), SigSpec(di)),
                ("S".to_string(), SigSpec(s)),
                ("O".to_string(), SigSpec(o)),
                ("CO".to_string(), SigSpec(co)),
            ]);
            e.stats.carry4 += 1;
            e.out.cells.push(Cell::Instance {
                ty: "CARRY4".into(),
                name: format!("oxsyn_carry{ci}_{k}"),
                params: BTreeMap::new(),
                conns,
            });
        }
    }
    for i in &lw.insts {
        let conns = i.conns.iter().map(|(p, v)| (p.clone(), e.spec(v))).collect();
        e.out.cells.push(Cell::Instance {
            ty: i.ty.clone(),
            name: i.name.clone(),
            params: i.params.clone(),
            conns,
        });
    }
    for m in &lw.mems {
        let reads =
            m.rd.iter()
                .map(|(c, a, d)| MemRead {
                    clk: c.map(|(c, p)| (SigSpec(vec![e.of(c)]), p)),
                    addr: e.spec(a),
                    data: e.spec(d),
                })
                .collect();
        let writes =
            m.wr.iter()
                .map(|((c, p), a, d, en)| MemWrite {
                    clk: (SigSpec(vec![e.of(*c)]), *p),
                    addr: e.spec(a),
                    data: e.spec(d),
                    en: e.spec(en),
                })
                .collect();
        e.out.cells.push(Cell::Mem(Memory {
            reads,
            writes,
            ..m.mem.clone()
        }));
    }
    // Names for signals whose every bit is an existing net.
    let ports: std::collections::HashSet<&str> = lw.ports.iter().map(|p| p.0.as_str()).collect();
    for (name, signed, bits) in &lw.named {
        if ports.contains(name.as_str()) {
            continue;
        }
        let mut v = Vec::with_capacity(bits.len());
        for b in bits {
            match b {
                Some(l) if l.is_const() => v.push(if *l == Lit::TRUE { Bit::One } else { Bit::Zero }),
                Some(l) => match e.net.get(l) {
                    Some(n) => v.push(*n),
                    None => break,
                },
                None => break,
            }
        }
        if v.len() != bits.len() {
            continue;
        }
        let id = e.out.add_signal(Signal {
            name: name.clone(),
            width: v.len() as u32,
            signed: *signed,
            port: None,
            init: None,
        });
        e.out.connects.push((SigSpec::of(id, v.len() as u32), SigSpec(v)));
        e.stats.named += 1;
    }
    (e.out, e.stats)
}
