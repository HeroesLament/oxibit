//! Write a `Module` as yosys JSON (`write_json` format), so yosys can read
//! it back next to its own `read_verilog; proc` result and prove the two
//! equivalent. Continuous connections are resolved into shared bits, as
//! yosys's own JSON writer does.

use serde_json::{Map, Value, json};

use oxibit_prof::{hot, hot_block};

use crate::netlist::{Bit, Cell, Dir, Module, Param, SigSpec};

/// Union-find over every signal bit (flat index), with path halving; a
/// set may carry the constant it is tied to.
struct Alias {
    offs: Vec<usize>,
    parent: Vec<u32>,
    konst: Vec<Option<Bit>>,
}

impl Alias {
    fn new(m: &Module) -> Self {
        let mut offs = Vec::with_capacity(m.signals.len() + 1);
        let mut n = 0usize;
        for s in &m.signals {
            offs.push(n);
            n += s.width as usize;
        }
        offs.push(n);
        Alias { offs, parent: (0..n as u32).collect(), konst: vec![None; n] }
    }
    fn idx(&self, b: Bit) -> Option<usize> {
        match b {
            Bit::Sig(id, i) => Some(self.offs[id.0 as usize] + i as usize),
            _ => None,
        }
    }
    fn find(&mut self, mut x: usize) -> usize {
        while self.parent[x] as usize != x {
            let g = self.parent[self.parent[x] as usize];
            self.parent[x] = g;
            x = g as usize;
        }
        x
    }
    fn union(&mut self, a: Bit, b: Bit) {
        match (self.idx(a), self.idx(b)) {
            (Some(x), Some(y)) => {
                let (rx, ry) = (self.find(x), self.find(y));
                if rx != ry {
                    // Two different constants would be a contradiction yosys
                    // reports; keep the first.
                    let k = self.konst[rx].or(self.konst[ry]);
                    self.parent[ry] = rx as u32;
                    self.konst[rx] = k;
                }
            }
            (Some(x), None) | (None, Some(x)) => {
                let c = if self.idx(a).is_none() { a } else { b };
                let r = self.find(x);
                if self.konst[r].is_none() {
                    self.konst[r] = Some(c);
                }
            }
            (None, None) => {}
        }
    }
}

/// An object from owned values (`json!` would serialize each value again).
fn obj<const N: usize>(kv: [(&str, Value); N]) -> Value {
    Value::Object(kv.into_iter().map(|(k, v)| (k.to_string(), v)).collect())
}

fn konst(b: Bit) -> Value {
    match b {
        Bit::Zero => json!("0"),
        Bit::One => json!("1"),
        Bit::Z => json!("z"),
        _ => json!("x"),
    }
}

fn bin(v: u64, width: u32) -> Value {
    json!((0..width).rev().map(|i| if (v >> i) & 1 == 1 { '1' } else { '0' }).collect::<String>())
}

fn param(p: &Param) -> Value {
    match p {
        Param::Int(i) => bin(*i as u64, 32),
        Param::Bits(b) => json!(b.iter().rev().map(|x| match x { Some(true) => '1', Some(false) => '0', None => 'x' }).collect::<String>()),
        Param::Str(s) => json!(s),
    }
}

#[hot]
pub fn write(m: &Module) -> Value {
    // Resolve connections.
    let mut alias = hot_block!("json.alias", {
        let mut alias = Alias::new(m);
        for (l, r) in &m.connects {
            for (a, b) in l.0.iter().zip(&r.0) {
                alias.union(*a, *b);
            }
        }
        alias
    });
    // Number every set; a set tied to a constant is that constant.
    let nets = hot_block!("json.number", {
    let n = *alias.offs.last().unwrap();
    let mut root_num: Vec<u64> = vec![0; n];
    let mut next = 2u64;
    let mut nets: Vec<Value> = Vec::with_capacity(n);
    for x in 0..n {
        let r = alias.find(x);
        nets.push(match alias.konst[r] {
            Some(c) => konst(c),
            None => {
                if root_num[r] == 0 {
                    root_num[r] = next;
                    next += 1;
                }
                json!(root_num[r])
            }
        });
    }
    nets
    });
    let offs = alias.offs.clone();
    let bitval = |b: Bit| -> Value {
        match b {
            Bit::Sig(id, i) => nets[offs[id.0 as usize] + i as usize].clone(),
            c => konst(c),
        }
    };
    let spec = |s: &SigSpec| -> Value { Value::Array(s.0.iter().map(|b| bitval(*b)).collect()) };

    let (ports, netnames) = hot_block!("json.netnames", {
    let mut ports = Map::new();
    let mut netnames = Map::new();
    for (si, s) in m.signals.iter().enumerate() {
        let bits = spec(&SigSpec::of(crate::netlist::SigId(si as u32), s.width));
        if let Some(d) = s.port {
            let dir = match d {
                Dir::Input => "input",
                Dir::Output => "output",
                Dir::Inout => "inout",
            };
            ports.insert(s.name.clone(), obj([("direction", json!(dir)), ("bits", bits.clone())]));
        }
        let mut attrs = Map::new();
        if let Some(init) = &s.init {
            attrs.insert("init".into(), param(&Param::Bits(init.clone())));
        }
        netnames.insert(
            s.name.clone(),
            obj([
                ("hide_name", json!(u8::from(s.name.starts_with('$')))),
                ("bits", bits),
                ("attributes", Value::Object(attrs)),
                ("signed", json!(u8::from(s.signed))),
            ]),
        );
    }
    (ports, netnames)
    });

    let cells = hot_block!("json.cells", {
    let mut cells = Map::new();
    for (ci, c) in m.cells.iter().enumerate() {
        let (ty, params, dirs, conns): (String, Map<String, Value>, Vec<(&str, &str)>, Vec<(&str, &SigSpec)>) = match c {
            Cell::Unary { op, a, a_signed, y } => (
                op.yosys().into(),
                [("A_SIGNED", bin(u64::from(*a_signed), 32)), ("A_WIDTH", bin(a.width() as u64, 32)), ("Y_WIDTH", bin(y.width() as u64, 32))]
                    .into_iter()
                    .map(|(k, v)| (k.to_string(), v))
                    .collect(),
                vec![("A", "input"), ("Y", "output")],
                vec![("A", a), ("Y", y)],
            ),
            Cell::Binary { op, a, b, signed, y } => (
                op.yosys().into(),
                [
                    ("A_SIGNED", bin(u64::from(*signed && *op != crate::netlist::Op::Shiftx), 32)),
                    ("B_SIGNED", bin(u64::from(*signed), 32)),
                    ("A_WIDTH", bin(a.width() as u64, 32)),
                    ("B_WIDTH", bin(b.width() as u64, 32)),
                    ("Y_WIDTH", bin(y.width() as u64, 32)),
                ]
                .into_iter()
                .map(|(k, v)| (k.to_string(), v))
                .collect(),
                vec![("A", "input"), ("B", "input"), ("Y", "output")],
                vec![("A", a), ("B", b), ("Y", y)],
            ),
            Cell::Mux { a, b, s, y } => (
                "$mux".into(),
                [("WIDTH".to_string(), bin(y.width() as u64, 32))].into_iter().collect(),
                vec![("A", "input"), ("B", "input"), ("S", "input"), ("Y", "output")],
                vec![("A", a), ("B", b), ("S", s), ("Y", y)],
            ),
            Cell::Dff { clk, posedge, d, q } => (
                "$dff".into(),
                [("CLK_POLARITY".to_string(), bin(u64::from(*posedge), 1)), ("WIDTH".to_string(), bin(q.width() as u64, 32))].into_iter().collect(),
                vec![("CLK", "input"), ("D", "input"), ("Q", "output")],
                vec![("CLK", clk), ("D", d), ("Q", q)],
            ),
            Cell::Adff { clk, posedge, arst, arst_high, value, d, q } => (
                "$adff".into(),
                [
                    ("CLK_POLARITY".to_string(), bin(u64::from(*posedge), 1)),
                    ("ARST_POLARITY".to_string(), bin(u64::from(*arst_high), 1)),
                    ("ARST_VALUE".to_string(), bits_str(value.iter().map(|b| Some(*b)).collect::<Vec<_>>().into_iter())),
                    ("WIDTH".to_string(), bin(q.width() as u64, 32)),
                ]
                .into_iter()
                .collect(),
                vec![("CLK", "input"), ("ARST", "input"), ("D", "input"), ("Q", "output")],
                vec![("CLK", clk), ("ARST", arst), ("D", d), ("Q", q)],
            ),
            Cell::Instance { ty, name, params, conns } => {
                let p = params.iter().map(|(k, v)| (k.clone(), param(v))).collect();
                let mut cell = Map::new();
                cell.insert("hide_name".into(), json!(0));
                cell.insert("type".into(), json!(ty));
                cell.insert("parameters".into(), Value::Object(p));
                cell.insert("attributes".into(), json!({}));
                cell.insert("connections".into(), Value::Object(conns.iter().map(|(k, v)| (k.clone(), spec(v))).collect()));
                let _ = ci;
                cells.insert(name.clone(), Value::Object(cell));
                continue;
            }
            Cell::Mem(mem) => {
                cells.insert(format!("$mem{ci}"), mem_v2(mem, &spec));
                continue;
            }
        };
        cells.insert(
            format!("$c{ci}"),
            obj([
                ("hide_name", json!(1)),
                ("type", json!(ty)),
                ("parameters", Value::Object(params)),
                ("attributes", json!({})),
                ("port_directions", Value::Object(dirs.iter().map(|(p, d)| (p.to_string(), json!(d))).collect())),
                ("connections", Value::Object(conns.iter().map(|(p, s)| (p.to_string(), spec(s))).collect())),
            ]),
        );
    }
    cells
    });
    let module = obj([
        ("attributes", json!({"top": "00000000000000000000000000000001"})),
        ("ports", Value::Object(ports)),
        ("cells", Value::Object(cells)),
        ("netnames", Value::Object(netnames)),
    ]);
    let mut mods = Map::new();
    mods.insert(m.name.clone(), module);
    obj([("creator", json!("oxibit-hdl")), ("modules", Value::Object(mods))])
}

fn bits_str(b: impl DoubleEndedIterator<Item = Option<bool>>) -> Value {
    json!(b.rev().map(|x| match x { Some(true) => '1', Some(false) => '0', None => 'x' }).collect::<String>())
}

fn cat(parts: impl Iterator<Item = Vec<Value>>) -> Value {
    Value::Array(parts.flatten().collect())
}

fn arr(v: &Value) -> Vec<Value> {
    v.as_array().cloned().unwrap_or_default()
}

/// A memory as one `$mem_v2` cell, the form `memory_collect` produces.
fn mem_v2(m: &crate::netlist::Memory, spec: &dyn Fn(&SigSpec) -> Value) -> Value {
    let abits = m.reads.iter().map(|r| r.addr.width()).chain(m.writes.iter().map(|w| w.addr.width())).max().unwrap_or(1).max(1);
    let addr = |a: &SigSpec| arr(&spec(&a.extend(abits, false)));
    let (nr, nw) = (m.reads.len(), m.writes.len());
    let flag = |f: &dyn Fn(usize) -> bool, n: usize| bits_str((0..n).map(|i| Some(f(i))).collect::<Vec<_>>().into_iter());
    let mut init = Vec::with_capacity((m.size * m.width) as usize);
    for i in 0..m.size {
        match m.init.get(&i) {
            Some(w) => init.extend((0..m.width as usize).map(|b| w.get(b).copied().flatten())),
            None => init.extend(std::iter::repeat_n(None, m.width as usize)),
        }
    }
    let xw = |n: usize| bits_str(std::iter::repeat_n(None, n * m.width as usize).collect::<Vec<_>>().into_iter());
    let one = |v: &str, n: usize| Value::Array(std::iter::repeat_n(json!(v), n).collect());
    let mut prio = Vec::new();
    for i in 0..nw {
        for j in 0..nw {
            prio.push(Some(j < i));
        }
    }
    let params: Map<String, Value> = [
        ("MEMID", json!(format!("\\{}", m.name))),
        ("SIZE", bin(m.size as u64, 32)),
        ("OFFSET", bin(m.offset as u64, 32)),
        ("ABITS", bin(abits as u64, 32)),
        ("WIDTH", bin(m.width as u64, 32)),
        ("INIT", bits_str(init.into_iter())),
        ("RD_PORTS", bin(nr as u64, 32)),
        ("RD_WIDE_CONTINUATION", flag(&|_| false, nr.max(1))),
        ("RD_CLK_ENABLE", flag(&|i| m.reads.get(i).is_some_and(|r| r.clk.is_some()), nr.max(1))),
        ("RD_CLK_POLARITY", flag(&|i| m.reads.get(i).and_then(|r| r.clk.as_ref()).is_some_and(|c| c.1), nr.max(1))),
        ("RD_TRANSPARENCY_MASK", flag(&|_| false, (nr * nw).max(1))),
        ("RD_COLLISION_X_MASK", flag(&|_| false, (nr * nw).max(1))),
        ("RD_CE_OVER_SRST", flag(&|_| false, nr.max(1))),
        ("RD_INIT_VALUE", xw(nr.max(1))),
        ("RD_ARST_VALUE", xw(nr.max(1))),
        ("RD_SRST_VALUE", xw(nr.max(1))),
        ("WR_PORTS", bin(nw as u64, 32)),
        ("WR_WIDE_CONTINUATION", flag(&|_| false, nw.max(1))),
        ("WR_CLK_ENABLE", flag(&|i| i < nw, nw.max(1))),
        ("WR_CLK_POLARITY", flag(&|i| m.writes.get(i).is_some_and(|w| w.clk.1), nw.max(1))),
        ("WR_PRIORITY_MASK", if nw == 0 { json!("0") } else { bits_str(prio.into_iter()) }),
    ]
    .into_iter()
    .map(|(k, v)| (k.to_string(), v))
    .collect();
    let rd_clk = cat(m.reads.iter().map(|r| match &r.clk { Some((c, _)) => arr(&spec(c)), None => vec![json!("x")] }));
    let conns = obj([
        ("RD_CLK", rd_clk),
        ("RD_EN", one("1", nr)),
        ("RD_ARST", one("0", nr)),
        ("RD_SRST", one("0", nr)),
        ("RD_ADDR", cat(m.reads.iter().map(|r| addr(&r.addr)))),
        ("RD_DATA", cat(m.reads.iter().map(|r| arr(&spec(&r.data))))),
        ("WR_CLK", cat(m.writes.iter().map(|w| arr(&spec(&w.clk.0))))),
        ("WR_EN", cat(m.writes.iter().map(|w| arr(&spec(&w.en))))),
        ("WR_ADDR", cat(m.writes.iter().map(|w| addr(&w.addr)))),
        ("WR_DATA", cat(m.writes.iter().map(|w| arr(&spec(&w.data))))),
    ]);
    let dirs: Map<String, Value> = ["RD_CLK", "RD_EN", "RD_ARST", "RD_SRST", "RD_ADDR", "WR_CLK", "WR_EN", "WR_ADDR", "WR_DATA"]
        .iter()
        .map(|p| (p.to_string(), json!("input")))
        .chain([("RD_DATA".to_string(), json!("output"))])
        .collect();
    let attrs: Map<String, Value> = m.attrs.iter().map(|(k, v)| (k.clone(), json!(v))).collect();
    obj([
        ("hide_name", json!(1)),
        ("type", json!("$mem_v2")),
        ("parameters", Value::Object(params)),
        ("attributes", Value::Object(attrs)),
        ("port_directions", Value::Object(dirs)),
        ("connections", conns),
    ])
}
