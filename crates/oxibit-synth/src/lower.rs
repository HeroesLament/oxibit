//! Lower a word-level `Module` to an AIG plus the sequential and
//! black-box elements around it.
//!
//! Cut points (AIG inputs): module inputs, flop outputs, memory read data,
//! primitive instance outputs. Everything combinational between them is
//! bit-blasted with yosys's cell semantics (widths, signedness, `$shiftx`
//! offsets). Flop enables and synchronous resets are recognised on the
//! word-level mux chain feeding D, so they land on CE / R / S pins instead
//! of in LUTs.

use std::collections::{BTreeMap, HashMap};

use oxibit_hdl::netlist::{Bit, Cell, Dir, Module, Op, Param, SigId, SigSpec};
use oxibit_prof::hot;

use crate::aig::{Aig, Lit};
use crate::lib_ports::PortDirs;

pub type R<T> = Result<T, String>;

/// A flop bit, ready to become an FDRE / FDSE / FDCE / FDPE.
#[derive(Debug, Clone)]
pub struct Ff {
    pub q: Lit,
    pub d: Lit,
    pub ce: Lit,
    pub clk: Lit,
    pub clk_inv: bool,
    pub kind: FfKind,
    /// Reset / set / clear / preset input (FALSE when none).
    pub sr: Lit,
    /// Async reset is active low (sets IS_CLR/PRE_INVERTED).
    pub sr_inv: bool,
    pub init: bool,
    /// Name of the register signal and bit, for the instance name.
    pub name: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FfKind {
    /// Sync reset to 0.
    Fdre,
    /// Sync set to 1.
    Fdse,
    /// Async clear.
    Fdce,
    /// Async preset.
    Fdpe,
}

#[derive(Debug, Clone)]
pub struct InstL {
    pub ty: String,
    pub name: String,
    pub params: BTreeMap<String, Param>,
    pub conns: Vec<(String, Vec<Lit>)>,
}

#[derive(Debug, Clone)]
pub struct MemL {
    pub mem: oxibit_hdl::netlist::Memory,
    pub rd: Vec<(Option<(Lit, bool)>, Vec<Lit>, Vec<Lit>)>,
    pub wr: Vec<((Lit, bool), Vec<Lit>, Vec<Lit>, Vec<Lit>)>,
}

#[derive(Debug, Default)]
pub struct Lowered {
    pub aig: Aig,
    pub name: String,
    /// Module ports: name, direction, signedness, bits.
    pub ports: Vec<(String, Dir, bool, Vec<Lit>)>,
    pub ffs: Vec<Ff>,
    pub insts: Vec<InstL>,
    pub mems: Vec<MemL>,
    /// Every named signal's bits, where computed: for net names.
    pub named: Vec<(String, bool, Vec<Option<Lit>>)>,
    /// Bits nothing drives (tied to 0).
    pub undriven: usize,
    /// AIG nodes (ANDs and chain outputs) added per word-level cell type.
    pub ands_by_op: BTreeMap<&'static str, usize>,
    /// Carry-chain bits added per word-level cell type.
    pub carry_by_op: BTreeMap<&'static str, usize>,
    /// AIG input nodes whose driver `sweep` removed.
    pub dead: std::collections::HashSet<u32>,
}

impl Lowered {
    /// Every literal something outside the AIG consumes.
    pub fn outputs(&self) -> Vec<Lit> {
        let mut v = Vec::new();
        for (_, d, _, bits) in &self.ports {
            if *d == Dir::Output {
                v.extend(bits);
            }
        }
        for c in &self.aig.chains {
            v.extend(&c.s);
            v.extend(&c.di);
            v.push(c.cin);
        }
        for f in &self.ffs {
            v.extend([f.d, f.ce, f.clk, f.sr]);
        }
        for i in &self.insts {
            for (_, b) in &i.conns {
                v.extend(b);
            }
        }
        for m in &self.mems {
            for (c, a, _) in &m.rd {
                if let Some((c, _)) = c {
                    v.push(*c);
                }
                v.extend(a);
            }
            for ((c, _), a, d, e) in &m.wr {
                v.push(*c);
                v.extend(a);
                v.extend(d);
                v.extend(e);
            }
        }
        v.retain(|l| !l.is_const());
        v
    }
}

impl Lowered {
    /// Drop flops and carry chains nothing observable depends on (module
    /// outputs, primitive instance inputs, memory ports). Returns the
    /// numbers removed; the AIG inputs they drove are recorded in `dead`.
    #[hot]
    pub fn sweep(&mut self) -> (usize, usize) {
        use crate::aig::Node;
        #[derive(Clone, Copy)]
        enum Owner {
            None,
            Ff(u32),
            /// Chain, position (`o` index; the length for `co`).
            Chain(u32, u32),
        }
        let n = self.aig.nodes.len();
        let mut owner = vec![Owner::None; n];
        for (i, f) in self.ffs.iter().enumerate() {
            owner[f.q.node() as usize] = Owner::Ff(i as u32);
        }
        for (i, c) in self.aig.chains.iter().enumerate() {
            for (p, l) in c.o.iter().chain([&c.co]).enumerate() {
                owner[l.node() as usize] = Owner::Chain(i as u32, p as u32);
            }
        }
        let mut mark = vec![false; n];
        let mut ff_live = vec![false; self.ffs.len()];
        // Chain positions something reads: a carry only flows upward, so
        // positions above the highest read sum bit (carry out unread) go.
        let mut need = vec![0usize; self.aig.chains.len()];
        let mut stack: Vec<u32> = Vec::new();
        // Roots: everything but flop and chain inputs.
        let mut roots: Vec<Lit> = Vec::new();
        for (_, d, _, bits) in &self.ports {
            if *d == Dir::Output {
                roots.extend(bits);
            }
        }
        for i in &self.insts {
            for (_, b) in &i.conns {
                roots.extend(b);
            }
        }
        for m in &self.mems {
            for (c, a, _) in &m.rd {
                roots.extend(c.map(|c| c.0));
                roots.extend(a);
            }
            for ((c, _), a, d, e) in &m.wr {
                roots.push(*c);
                roots.extend(a.iter().chain(d).chain(e));
            }
        }
        stack.extend(roots.iter().map(|l| l.node()));
        while let Some(k) = stack.pop() {
            let k = k as usize;
            if mark[k] {
                continue;
            }
            mark[k] = true;
            match self.aig.nodes[k] {
                Node::And(a, b) => {
                    stack.push(a.node());
                    stack.push(b.node());
                }
                Node::Input(_) => match owner[k] {
                    Owner::Ff(i) if !ff_live[i as usize] => {
                        ff_live[i as usize] = true;
                        let f = &self.ffs[i as usize];
                        stack.extend([f.d, f.ce, f.clk, f.sr].map(|l| l.node()));
                    }
                    Owner::Chain(i, p) => {
                        let c = &self.aig.chains[i as usize];
                        let n = c.s.len();
                        let want = if p as usize == n { n } else { p as usize + 1 };
                        let have = need[i as usize];
                        if want > have {
                            if have == 0 {
                                stack.push(c.cin.node());
                            }
                            stack.extend(c.s[have..want].iter().chain(&c.di[have..want]).map(|l| l.node()));
                            need[i as usize] = want;
                        }
                    }
                    _ => {}
                },
                Node::Const => {}
            }
        }
        let (nf, nc) = (self.ffs.len(), self.aig.chains.len());
        let mut i = 0;
        self.ffs.retain(|f| {
            let keep = ff_live[i];
            if !keep {
                self.dead.insert(f.q.node());
            }
            i += 1;
            keep
        });
        let mut i = 0;
        let dead = &mut self.dead;
        self.aig.chains.retain_mut(|c| {
            let keep = need[i] > 0;
            if !keep {
                dead.extend(c.o.iter().chain([&c.co]).map(|l| l.node()));
            } else if need[i] < c.s.len() {
                let k = need[i];
                dead.extend(c.o[k..].iter().chain([&c.co]).map(|l| l.node()));
                c.s.truncate(k);
                c.di.truncate(k);
                c.o.truncate(k);
                c.co = Lit::FALSE;
            }
            i += 1;
            keep
        });
        (nf - self.ffs.len(), nc - self.aig.chains.len())
    }
}

#[derive(Clone, Copy, PartialEq)]
enum Drv {
    None,
    Input,
    /// Output bit `k` of cell `c` (for memories: read-data bit index
    /// across all read ports).
    Cell(u32, u32),
}

#[derive(Clone, Copy, PartialEq)]
enum CBit {
    Net(usize),
    Const(bool),
}

struct Lower<'a> {
    m: &'a Module,
    dirs: &'a PortDirs,
    offs: Vec<usize>,
    parent: Vec<u32>,
    konst: Vec<Option<bool>>,
    drv: Vec<Drv>,
    lit: Vec<Option<Lit>>,
    /// Cell evaluation state: 0 = not yet, 1 = in progress, 2 = done.
    state: Vec<u8>,
    g: Aig,
    undriven: usize,
    ands_by_op: BTreeMap<&'static str, usize>,
    carry_by_op: BTreeMap<&'static str, usize>,
}

impl<'a> Lower<'a> {
    fn idx(&self, id: SigId, i: u32) -> usize {
        self.offs[id.0 as usize] + i as usize
    }
    fn find(&mut self, mut x: usize) -> usize {
        while self.parent[x] as usize != x {
            let g = self.parent[self.parent[x] as usize];
            self.parent[x] = g;
            x = g as usize;
        }
        x
    }
    fn canon(&mut self, b: Bit) -> CBit {
        match b {
            Bit::Sig(id, i) => {
                let x = self.idx(id, i);
                let r = self.find(x);
                match self.konst[r] {
                    Some(v) => CBit::Const(v),
                    None => CBit::Net(r),
                }
            }
            Bit::One => CBit::Const(true),
            // x and z become 0.
            _ => CBit::Const(false),
        }
    }

    fn inst_dir(&self, ty: &str, port: &str) -> R<Dir> {
        self.dirs
            .get(ty)
            .and_then(|p| p.get(port))
            .copied()
            .ok_or_else(|| format!("no port direction for {ty}.{port} (pass the primitive library)"))
    }

    fn new(m: &'a Module, dirs: &'a PortDirs) -> R<Self> {
        let mut offs = Vec::with_capacity(m.signals.len() + 1);
        let mut n = 0usize;
        for s in &m.signals {
            offs.push(n);
            n += s.width as usize;
        }
        offs.push(n);
        let mut l = Lower {
            m,
            dirs,
            offs,
            parent: (0..n as u32).collect(),
            konst: vec![None; n],
            drv: vec![Drv::None; n],
            lit: vec![None; n],
            state: vec![0; m.cells.len()],
            g: Aig::new(),
            undriven: 0,
            ands_by_op: BTreeMap::new(),
            carry_by_op: BTreeMap::new(),
        };
        for (lhs, rhs) in &m.connects {
            for (a, b) in lhs.0.iter().zip(&rhs.0) {
                let ca = match a {
                    Bit::Sig(id, i) => Some(l.idx(*id, *i)),
                    _ => None,
                };
                let cb = match b {
                    Bit::Sig(id, i) => Some(l.idx(*id, *i)),
                    _ => None,
                };
                match (ca, cb) {
                    (Some(x), Some(y)) => {
                        let (rx, ry) = (l.find(x), l.find(y));
                        if rx != ry {
                            let k = l.konst[rx].or(l.konst[ry]);
                            l.parent[ry] = rx as u32;
                            l.konst[rx] = k;
                        }
                    }
                    (Some(x), None) | (None, Some(x)) => {
                        let c = if ca.is_none() { *a } else { *b };
                        let r = l.find(x);
                        if l.konst[r].is_none() {
                            l.konst[r] = Some(c == Bit::One);
                        }
                    }
                    _ => {}
                }
            }
        }
        // Drivers.
        for (si, s) in m.signals.iter().enumerate() {
            if matches!(s.port, Some(Dir::Input) | Some(Dir::Inout)) {
                for i in 0..s.width {
                    if let CBit::Net(r) = l.canon(Bit::Sig(SigId(si as u32), i)) {
                        l.drv[r] = Drv::Input;
                    }
                }
            }
        }
        for (ci, c) in m.cells.iter().enumerate() {
            let outs: Vec<Bit> = match c {
                Cell::Unary { y, .. } | Cell::Binary { y, .. } | Cell::Mux { y, .. } => y.0.clone(),
                Cell::Dff { q, .. } | Cell::Adff { q, .. } => q.0.clone(),
                Cell::Mem(mem) => mem.reads.iter().flat_map(|r| r.data.0.iter().copied()).collect(),
                Cell::Instance { ty, conns, .. } => {
                    let mut v = Vec::new();
                    for (p, s) in conns {
                        if matches!(l.inst_dir(ty, p)?, Dir::Output | Dir::Inout) {
                            v.extend(s.0.iter().copied());
                        }
                    }
                    v
                }
            };
            for (k, b) in outs.into_iter().enumerate() {
                if let CBit::Net(r) = l.canon(b) {
                    if l.drv[r] == Drv::None {
                        l.drv[r] = Drv::Cell(ci as u32, k as u32);
                    }
                }
            }
        }
        Ok(l)
    }

    fn bit(&mut self, b: Bit) -> R<Lit> {
        match self.canon(b) {
            CBit::Const(v) => Ok(if v { Lit::TRUE } else { Lit::FALSE }),
            CBit::Net(r) => self.net(r),
        }
    }

    fn spec(&mut self, s: &SigSpec) -> R<Vec<Lit>> {
        s.0.iter().map(|b| self.bit(*b)).collect()
    }

    fn net(&mut self, r: usize) -> R<Lit> {
        if let Some(l) = self.lit[r] {
            return Ok(l);
        }
        let l = match self.drv[r] {
            Drv::None => {
                self.undriven += 1;
                Lit::FALSE
            }
            Drv::Input => self.g.input(),
            Drv::Cell(c, _) => {
                let ci = c as usize;
                match &self.m.cells[ci] {
                    Cell::Unary { .. } | Cell::Binary { .. } | Cell::Mux { .. } => {
                        if self.state[ci] == 1 {
                            return Err(format!("combinational loop through cell {ci}"));
                        }
                        self.state[ci] = 1;
                        self.eval(ci)?;
                        self.state[ci] = 2;
                        return self.lit[r].ok_or_else(|| format!("cell {ci} did not drive its output"));
                    }
                    // Sequential and black-box outputs are cut points.
                    _ => self.g.input(),
                }
            }
        };
        self.lit[r] = Some(l);
        Ok(l)
    }

    fn set_out(&mut self, y: &SigSpec, v: &[Lit]) {
        for (b, l) in y.0.iter().zip(v) {
            if let CBit::Net(r) = self.canon(*b) {
                if self.lit[r].is_none() {
                    self.lit[r] = Some(*l);
                }
            }
        }
    }

    fn eval(&mut self, ci: usize) -> R<()> {
        let m = self.m;
        // Count the AND nodes each cell adds by itself (inputs are
        // evaluated first, so they are not charged to it).
        match &m.cells[ci] {
            Cell::Unary { a, .. } => {
                self.spec(a)?;
            }
            Cell::Binary { a, b, .. } | Cell::Mux { a, b, .. } => {
                self.spec(a)?;
                self.spec(b)?;
            }
            _ => {}
        }
        if let Cell::Mux { s, .. } = &m.cells[ci] {
            self.bit(s.0[0])?;
        }
        let before = self.g.nodes.len();
        let chains_before = self.g.chains.len();
        self.eval_own(ci)?;
        let kind = match &m.cells[ci] {
            Cell::Unary { op, .. } | Cell::Binary { op, .. } => op.yosys(),
            _ => "$mux",
        };
        *self.ands_by_op.entry(kind).or_default() += self.g.nodes.len() - before;
        let bits: usize = self.g.chains[chains_before..].iter().map(|c| c.s.len()).sum();
        if bits > 0 {
            *self.carry_by_op.entry(kind).or_default() += bits;
        }
        Ok(())
    }

    fn eval_own(&mut self, ci: usize) -> R<()> {
        let m = self.m;
        match &m.cells[ci] {
            Cell::Unary { op, a, a_signed, y } => {
                let av = self.spec(a)?;
                let yw = y.width() as usize;
                let v = unary(&mut self.g, *op, &av, *a_signed, yw);
                self.set_out(y, &v);
            }
            Cell::Binary { op, a, b, signed, y } => {
                let av = self.spec(a)?;
                let bv = self.spec(b)?;
                let yw = y.width() as usize;
                let v = binary(&mut self.g, *op, &av, &bv, *signed, yw);
                self.set_out(y, &v);
            }
            Cell::Mux { a, b, s, y } => {
                let av = self.spec(a)?;
                let bv = self.spec(b)?;
                let sv = self.bit(s.0[0])?;
                let v: Vec<Lit> = av.iter().zip(&bv).map(|(x, z)| self.g.mux(sv, *x, *z)).collect();
                self.set_out(y, &v);
            }
            _ => unreachable!(),
        }
        Ok(())
    }

    /// The mux driving canonical bit `r`, as (a, b, s) bits.
    fn mux_of(&mut self, r: usize) -> Option<(Bit, Bit, Bit)> {
        let Drv::Cell(c, k) = self.drv[r] else {
            return None;
        };
        match &self.m.cells[c as usize] {
            Cell::Mux { a, b, s, .. } => Some((a.0[k as usize], b.0[k as usize], s.0[0])),
            _ => None,
        }
    }

    /// Peel enables (and, for `sync`, one synchronous reset) off the D
    /// input of a flop bit whose output is canonical bit `q`.
    fn ff_bit(&mut self, q: usize, d: Bit, sync: bool) -> R<(Lit, Lit, Option<(Lit, bool)>)> {
        enum Layer {
            En(Bit, bool),
            Rst(Bit, bool, bool),
        }
        let mut layers = Vec::new();
        let mut cur = d;
        let mut have_rst = false;
        let mut hold = false;
        loop {
            let CBit::Net(r) = self.canon(cur) else { break };
            if r == q {
                hold = true;
                break;
            }
            let Some((a, b, s)) = self.mux_of(r) else {
                break;
            };
            let (ca, cb) = (self.canon(a), self.canon(b));
            if ca == CBit::Net(q) {
                layers.push(Layer::En(s, true));
                cur = b;
            } else if cb == CBit::Net(q) {
                layers.push(Layer::En(s, false));
                cur = a;
            } else if sync && !have_rst {
                if let CBit::Const(v) = cb {
                    layers.push(Layer::Rst(s, true, v));
                    have_rst = true;
                    cur = a;
                } else if let CBit::Const(v) = ca {
                    layers.push(Layer::Rst(s, false, v));
                    have_rst = true;
                    cur = b;
                } else {
                    break;
                }
            } else {
                break;
            }
        }
        // Hold arms deeper in the mux tree (q fed back under some
        // condition): drop them from D and fold their conditions into CE.
        let (dl, inner_hold) = if hold {
            (Lit::FALSE, Lit::TRUE)
        } else {
            let mut memo = HashMap::new();
            match self.hold_split(q, cur, &mut memo, 0)? {
                (Some(d), h) => (d, h),
                (None, _) => (Lit::FALSE, Lit::TRUE),
            }
        };
        let mut ens: Vec<Lit> = Vec::new();
        let mut rst = None;
        for layer in &layers {
            match *layer {
                Layer::En(s, pol) => {
                    let sl = self.bit(s)?;
                    ens.push(sl.not_if(!pol));
                }
                Layer::Rst(s, pol, v) => {
                    let sl = self.bit(s)?.not_if(!pol);
                    let outer = self.g.and_all(&ens);
                    rst = Some((self.g.and(sl, outer), v));
                }
            }
        }
        // The inner hold sits below every layer: it gates CE only.
        ens.push(inner_hold.not());
        let mut ce = self.g.and_all(&ens);
        if hold {
            ce = Lit::FALSE;
        }
        Ok((dl, ce, rst))
    }

    /// Split the mux tree at `x` into (value with arms equal to `q`
    /// removed, condition under which the tree selects `q`). `None`: every
    /// path ends in `q`. Subtrees that cannot reach `q` are left as they are.
    fn hold_split(&mut self, q: usize, x: Bit, memo: &mut HashMap<usize, (Option<Lit>, Lit)>, depth: u32) -> R<(Option<Lit>, Lit)> {
        let CBit::Net(r) = self.canon(x) else {
            return Ok((Some(self.bit(x)?), Lit::FALSE));
        };
        if r == q {
            return Ok((None, Lit::TRUE));
        }
        if let Some(v) = memo.get(&r) {
            return Ok(*v);
        }
        let res = match self.mux_of(r) {
            Some((a, b, s)) if depth < 64 && self.reaches(q, r, &mut HashMap::new(), 0) => {
                let (da, ha) = self.hold_split(q, a, memo, depth + 1)?;
                let (db, hb) = self.hold_split(q, b, memo, depth + 1)?;
                let sl = self.bit(s)?;
                let d = match (da, db) {
                    (None, None) => None,
                    (None, Some(v)) | (Some(v), None) => Some(v),
                    (Some(va), Some(vb)) => Some(self.g.mux(sl, va, vb)),
                };
                (d, self.g.mux(sl, ha, hb))
            }
            _ => (Some(self.net(r)?), Lit::FALSE),
        };
        memo.insert(r, res);
        Ok(res)
    }

    /// Does the mux tree at canonical bit `r` have `q` as a leaf?
    fn reaches(&mut self, q: usize, r: usize, memo: &mut HashMap<usize, bool>, depth: u32) -> bool {
        if r == q {
            return true;
        }
        if depth >= 64 {
            return false;
        }
        if let Some(v) = memo.get(&r) {
            return *v;
        }
        let v = match self.mux_of(r) {
            Some((a, b, _)) => {
                let ra = matches!(self.canon(a), CBit::Net(x) if self.reaches(q, x, memo, depth + 1));
                ra || matches!(self.canon(b), CBit::Net(x) if self.reaches(q, x, memo, depth + 1))
            }
            None => false,
        };
        memo.insert(r, v);
        v
    }
}

/// Zero- or sign-extend / truncate.
fn ext(v: &[Lit], w: usize, signed: bool) -> Vec<Lit> {
    let mut o: Vec<Lit> = v.iter().take(w).copied().collect();
    let fill = if signed { v.last().copied().unwrap_or(Lit::FALSE) } else { Lit::FALSE };
    o.resize(w, fill);
    o
}

/// One ripple position in the AIG: (sum, carry out).
fn ripple(g: &mut Aig, x: Lit, y: Lit, c: Lit) -> (Lit, Lit) {
    let t = g.xor(x, y);
    let s = g.xor(t, c);
    let p = g.and(x, y);
    let q = g.and(t, c);
    (s, g.or(p, q))
}

/// `a + b + c` over equal widths: (sum, carry out of the top bit).
///
/// Only the positions that need a carry chain get one (yosys `wreduce` and
/// `alumacc` trimming, done per bit):
/// - low positions ripple in the AIG while the carry into them is constant
///   and one operand is (the carry folds or becomes that operand bit);
/// - when both operands are extensions of their bit k (equal literals from
///   k up; sign or zero extension, or constants), the carry out of k is the
///   carry out of the top, and every sum bit above k is
///   `a[k] ^ b[k] ^ carry`;
/// - top positions where both operands are constant ripple in the AIG
///   (each folds to generate, kill or propagate).
fn add(g: &mut Aig, a: &[Lit], b: &[Lit], mut c: Lit) -> (Vec<Lit>, Lit) {
    let n = a.len();
    let mut out = Vec::with_capacity(n);
    let mut i = 0;
    while i < n && c.is_const() && (a[i].is_const() || b[i].is_const()) {
        let (s, c2) = ripple(g, a[i], b[i], c);
        out.push(s);
        c = c2;
        i += 1;
    }
    if i == n {
        return (out, c);
    }
    // Extension region: a[m] == a[k] and b[m] == b[k] for all m >= k.
    let mut k = n - 1;
    while k > i && a[k - 1] == a[n - 1] && b[k - 1] == b[n - 1] {
        k -= 1;
    }
    let e = k + 1;
    // Constant top of [i, e).
    let mut j = e;
    while j > i && a[j - 1].is_const() && b[j - 1].is_const() {
        j -= 1;
    }
    if g.carry_min > 0 && j - i >= g.carry_min {
        let s: Vec<Lit> = (i..j).map(|m| g.xor(a[m], b[m])).collect();
        // DI is the carry when a == b: either operand; prefer a constant.
        let di: Vec<Lit> = (i..j).map(|m| if b[m].is_const() { b[m] } else { a[m] }).collect();
        let o: Vec<Lit> = (i..j).map(|_| g.input()).collect();
        let co = g.input();
        g.chains.push(crate::aig::Chain {
            s,
            di,
            cin: c,
            o: o.clone(),
            co,
        });
        out.extend(o);
        c = co;
    } else {
        for m in i..j {
            let (s, c2) = ripple(g, a[m], b[m], c);
            out.push(s);
            c = c2;
        }
    }
    for m in j..e {
        let (s, c2) = ripple(g, a[m], b[m], c);
        out.push(s);
        c = c2;
    }
    if e < n {
        let t = g.xor(a[k], b[k]);
        let s = g.xor(t, c);
        out.resize(n, s);
    }
    (out, c)
}

/// `a < b`, unsigned, same width, rippled in the AIG.
fn ult_aig(g: &mut Aig, a: &[Lit], b: &[Lit]) -> Lit {
    let mut c = Lit::FALSE;
    for (x, y) in a.iter().zip(b) {
        let lt = g.and(x.not(), *y);
        let eq = g.xor(*x, *y).not();
        let keep = g.and(eq, c);
        c = g.or(lt, keep);
    }
    c
}

/// `a < b`, unsigned, same width.
///
/// On a carry chain each position compares a chunk of bits instead of one
/// (yosys `cmp2lcu`): S = chunk equal (propagate the lower result), DI =
/// chunk less (decides when not equal), so CO = S ? CI : DI. A chunk takes
/// as many bits as keep it within one 6-input LUT for S and one for DI:
/// three bit pairs, or six bits against a constant. Constant chunks at
/// either end fold into the AIG.
fn ult(g: &mut Aig, a: &[Lit], b: &[Lit]) -> Lit {
    if g.carry_min == 0 || a.len() < g.carry_min {
        return ult_aig(g, a, b);
    }
    // Chunks, LSB first: (S, DI).
    let mut chunks: Vec<(Lit, Lit)> = Vec::new();
    let mut i = 0;
    while i < a.len() {
        let mut j = i;
        let mut vars = 0;
        while j < a.len() {
            let v = usize::from(!a[j].is_const()) + usize::from(!b[j].is_const());
            if vars + v > 6 {
                break;
            }
            vars += v;
            j += 1;
        }
        let j = j.max(i + 1);
        let x: Vec<Lit> = (i..j).map(|k| g.xor(a[k], b[k]).not()).collect();
        let eq = g.and_all(&x);
        let lt = ult_aig(g, &a[i..j], &b[i..j]);
        chunks.push((eq, lt));
        i = j;
    }
    let mut c = Lit::FALSE;
    let mut lo = 0;
    while lo < chunks.len() && chunks[lo].0.is_const() && chunks[lo].1.is_const() {
        c = g.mux(chunks[lo].0, chunks[lo].1, c);
        lo += 1;
    }
    let mut hi = chunks.len();
    while hi > lo && chunks[hi - 1].0.is_const() && chunks[hi - 1].1.is_const() {
        hi -= 1;
    }
    if hi - lo >= 2 {
        let s: Vec<Lit> = chunks[lo..hi].iter().map(|p| p.0).collect();
        let di: Vec<Lit> = chunks[lo..hi].iter().map(|p| p.1).collect();
        let o: Vec<Lit> = (lo..hi).map(|_| g.input()).collect();
        let co = g.input();
        g.chains.push(crate::aig::Chain { s, di, cin: c, o, co });
        c = co;
    } else {
        for &(eq, lt) in &chunks[lo..hi] {
            c = g.mux(eq, lt, c);
        }
    }
    for &(eq, lt) in &chunks[hi..] {
        c = g.mux(eq, lt, c);
    }
    c
}

fn lt(g: &mut Aig, a: &[Lit], b: &[Lit], signed: bool) -> Lit {
    if signed && !a.is_empty() {
        let mut a2 = a.to_vec();
        let mut b2 = b.to_vec();
        let n = a2.len() - 1;
        a2[n] = a2[n].not();
        b2[n] = b2[n].not();
        ult(g, &a2, &b2)
    } else {
        ult(g, a, b)
    }
}

fn eq(g: &mut Aig, a: &[Lit], b: &[Lit]) -> Lit {
    let x: Vec<Lit> = a.iter().zip(b).map(|(p, q)| g.xor(*p, *q).not()).collect();
    g.and_all(&x)
}

fn bool1(l: Lit, yw: usize) -> Vec<Lit> {
    let mut v = vec![Lit::FALSE; yw];
    if yw > 0 {
        v[0] = l;
    }
    v
}

/// Barrel shift of `v` by unsigned amount `amt`; `left` shifts toward the
/// MSB. Vacated bits get `fill`.
fn shift(g: &mut Aig, v: &[Lit], amt: &[Lit], left: bool, fill: Lit) -> Vec<Lit> {
    let w = v.len();
    let mut cur = v.to_vec();
    for (k, s) in amt.iter().enumerate() {
        if *s == Lit::FALSE {
            continue;
        }
        let step = if k >= 63 { usize::MAX } else { 1usize << k };
        let next: Vec<Lit> = (0..w)
            .map(|i| {
                if step >= w {
                    fill
                } else if left {
                    if i >= step { cur[i - step] } else { fill }
                } else if i + step < w {
                    cur[i + step]
                } else {
                    fill
                }
            })
            .collect();
        cur = cur.iter().zip(&next).map(|(a, b)| g.mux(*s, *a, *b)).collect();
    }
    cur
}

fn unary(g: &mut Aig, op: Op, a: &[Lit], signed: bool, yw: usize) -> Vec<Lit> {
    match op {
        Op::Not => ext(a, yw, signed).iter().map(|l| l.not()).collect(),
        Op::Neg => {
            let x: Vec<Lit> = ext(a, yw, signed).iter().map(|l| l.not()).collect();
            let z = vec![Lit::FALSE; yw];
            add(g, &x, &z, Lit::TRUE).0
        }
        Op::ReduceAnd => {
            let r = g.and_all(a);
            bool1(r, yw)
        }
        Op::ReduceOr => {
            let r = g.or_all(a);
            bool1(r, yw)
        }
        Op::ReduceXor => {
            let r = g.xor_all(a);
            bool1(r, yw)
        }
        Op::LogicNot => {
            let r = g.or_all(a).not();
            bool1(r, yw)
        }
        _ => unreachable!("{op:?} is not unary"),
    }
}

fn binary(g: &mut Aig, op: Op, a: &[Lit], b: &[Lit], signed: bool, yw: usize) -> Vec<Lit> {
    use Op::*;
    match op {
        And | Or | Xor | Xnor => {
            let (x, y) = (ext(a, yw, signed), ext(b, yw, signed));
            x.iter()
                .zip(&y)
                .map(|(p, q)| match op {
                    And => g.and(*p, *q),
                    Or => g.or(*p, *q),
                    Xor => g.xor(*p, *q),
                    _ => g.xor(*p, *q).not(),
                })
                .collect()
        }
        Add => add(g, &ext(a, yw, signed), &ext(b, yw, signed), Lit::FALSE).0,
        Sub => {
            let nb: Vec<Lit> = ext(b, yw, signed).iter().map(|l| l.not()).collect();
            add(g, &ext(a, yw, signed), &nb, Lit::TRUE).0
        }
        Mul => {
            let (x, y) = (ext(a, yw, signed), ext(b, yw, signed));
            let mut acc = vec![Lit::FALSE; yw];
            for (i, bi) in y.iter().enumerate() {
                if *bi == Lit::FALSE {
                    continue;
                }
                let mut pp = vec![Lit::FALSE; yw];
                for j in 0..yw - i {
                    pp[i + j] = g.and(x[j], *bi);
                }
                acc = add(g, &acc, &pp, Lit::FALSE).0;
            }
            acc
        }
        Eq | Ne | Lt | Le | Gt | Ge => {
            let w = a.len().max(b.len());
            let (x, y) = (ext(a, w, signed), ext(b, w, signed));
            let r = match op {
                Eq => eq(g, &x, &y),
                Ne => eq(g, &x, &y).not(),
                Lt => lt(g, &x, &y, signed),
                Ge => lt(g, &x, &y, signed).not(),
                Gt => lt(g, &y, &x, signed),
                _ => lt(g, &y, &x, signed).not(),
            };
            bool1(r, yw)
        }
        LogicAnd | LogicOr => {
            let p = g.or_all(a);
            let q = g.or_all(b);
            let r = if op == LogicAnd { g.and(p, q) } else { g.or(p, q) };
            bool1(r, yw)
        }
        Shl | Shr | Sshr => {
            // A extends to max(A, Y) by its own signedness; B is unsigned.
            let w = a.len().max(yw);
            let x = ext(a, w, signed);
            let fill = if op == Sshr && signed {
                x.last().copied().unwrap_or(Lit::FALSE)
            } else {
                Lit::FALSE
            };
            let mut v = shift(g, &x, b, op == Shl, fill);
            v.truncate(yw);
            v
        }
        Shiftx => {
            // Y[i] = A[i + B]; B signed when `signed`; out of range is x (0).
            let w = a.len().max(yw);
            let x = ext(a, w, false);
            let mut v = if signed && !b.is_empty() {
                let neg = b[b.len() - 1];
                let pos = shift(g, &x, &b[..b.len() - 1], false, Lit::FALSE);
                let nb: Vec<Lit> = b.iter().map(|l| l.not()).collect();
                let z = vec![Lit::FALSE; b.len()];
                let mag = add(g, &nb, &z, Lit::TRUE).0;
                let negv = shift(g, &x, &mag, true, Lit::FALSE);
                pos.iter().zip(&negv).map(|(p, n)| g.mux(neg, *p, *n)).collect()
            } else {
                shift(g, &x, b, false, Lit::FALSE)
            };
            v.truncate(yw);
            v
        }
        _ => unreachable!("{op:?} is not binary"),
    }
}

#[hot]
pub fn lower(m: &Module, dirs: &PortDirs, carry_min: usize) -> R<Lowered> {
    let mut l = Lower::new(m, dirs)?;
    l.g.carry_min = carry_min;
    let mut out = Lowered {
        name: m.name.clone(),
        ..Default::default()
    };
    // Ports.
    for (si, s) in m.signals.iter().enumerate() {
        if let Some(d) = s.port {
            let bits = l.spec(&SigSpec::of(SigId(si as u32), s.width))?;
            out.ports.push((s.name.clone(), d, s.signed, bits));
        }
    }
    for (ci, c) in m.cells.iter().enumerate() {
        match c {
            Cell::Dff { clk, posedge, d, q } | Cell::Adff { clk, posedge, d, q, .. } => {
                let clk_l = l.bit(clk.0[0])?;
                let (arst, arst_high, value) = match c {
                    Cell::Adff { arst, arst_high, value, .. } => (Some(arst.0[0]), *arst_high, value.clone()),
                    _ => (None, true, Vec::new()),
                };
                let arst_l = match arst {
                    Some(b) => Some(l.bit(b)?),
                    None => None,
                };
                for i in 0..q.width() as usize {
                    let qb = q.0[i];
                    let CBit::Net(qr) = l.canon(qb) else {
                        return Err(format!("flop {ci} output bit {i} is tied to a constant"));
                    };
                    let ql = l.net(qr)?;
                    let (dl, ce, rst) = l.ff_bit(qr, d.0[i], arst.is_none())?;
                    let (name, init) = match qb {
                        Bit::Sig(id, k) => {
                            let s = m.signal(id);
                            let init = s.init.as_ref().and_then(|v| v.get(k as usize).copied().flatten());
                            (format!("{}[{k}]", s.name), init)
                        }
                        _ => (format!("$ff{ci}[{i}]"), None),
                    };
                    let (kind, sr, sr_inv, rv) = match (arst_l, rst) {
                        (Some(a), _) => {
                            let v = value.get(i).copied().unwrap_or(false);
                            (if v { FfKind::Fdpe } else { FfKind::Fdce }, a, !arst_high, v)
                        }
                        (None, Some((r, v))) => (if v { FfKind::Fdse } else { FfKind::Fdre }, r, false, v),
                        (None, None) => (FfKind::Fdre, Lit::FALSE, false, false),
                    };
                    out.ffs.push(Ff {
                        q: ql,
                        d: dl,
                        ce,
                        clk: clk_l,
                        clk_inv: !posedge,
                        kind,
                        sr,
                        sr_inv,
                        init: init.unwrap_or(rv),
                        name,
                    });
                }
            }
            Cell::Instance { ty, name, params, conns } => {
                let mut cv = Vec::new();
                for (p, s) in conns {
                    let bits = match l.inst_dir(ty, p)? {
                        Dir::Input => l.spec(s)?,
                        _ => {
                            // Outputs (and inouts) are cut points this
                            // instance drives, unless something else does.
                            let mut v = Vec::new();
                            for b in &s.0 {
                                v.push(l.bit(*b)?);
                            }
                            v
                        }
                    };
                    cv.push((p.clone(), bits));
                }
                out.insts.push(InstL {
                    ty: ty.clone(),
                    name: name.clone(),
                    params: params.clone(),
                    conns: cv,
                });
            }
            Cell::Mem(mem) => {
                let mut rd = Vec::new();
                for r in &mem.reads {
                    let clk = match &r.clk {
                        Some((c, p)) => Some((l.bit(c.0[0])?, *p)),
                        None => None,
                    };
                    rd.push((clk, l.spec(&r.addr)?, l.spec(&r.data)?));
                }
                let mut wr = Vec::new();
                for w in &mem.writes {
                    wr.push(((l.bit(w.clk.0.0[0])?, w.clk.1), l.spec(&w.addr)?, l.spec(&w.data)?, l.spec(&w.en)?));
                }
                out.mems.push(MemL { mem: mem.clone(), rd, wr });
            }
            _ => {}
        }
    }
    // Names: whatever got computed.
    for (si, s) in m.signals.iter().enumerate() {
        if s.name.starts_with('$') {
            continue;
        }
        let bits: Vec<Option<Lit>> = (0..s.width)
            .map(|i| match l.canon(Bit::Sig(SigId(si as u32), i)) {
                CBit::Const(v) => Some(if v { Lit::TRUE } else { Lit::FALSE }),
                CBit::Net(r) => l.lit[r],
            })
            .collect();
        out.named.push((s.name.clone(), s.signed, bits));
    }
    out.undriven = l.undriven;
    out.ands_by_op = std::mem::take(&mut l.ands_by_op);
    out.carry_by_op = std::mem::take(&mut l.carry_by_op);
    out.aig = l.g;
    Ok(out)
}
