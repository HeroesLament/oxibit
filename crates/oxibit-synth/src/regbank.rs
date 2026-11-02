//! Register banks to memories.
//!
//! A barrel processor keeps per-context state as banks of registers:
//! `x0..x3`, each loaded with one shared value when its context writes and
//! otherwise holding, and read as `(s == 0) * x0 | (s == 1) * x1 | ...`
//! (the `onehot` pass has turned the sums into ORs by then). That is a
//! small memory, and as flops it costs a flop per bit per context, a
//! select tree per read, and a bypass pin per flop.
//!
//! A bank here is a set of word registers on one clock edge where
//! - each register's D is a mux tree whose leaves are its own Q (hold) or
//!   one value shared by the whole bank (load), and no tree mux has another
//!   reader;
//! - every read of every register is a term `(s == k) * q` (or `q & {s ==
//!   k}`) of a one-hot selection over one word `s`, built of ORs of such
//!   terms with no other reader inside, where every selection names every
//!   register exactly once, with the same key everywhere, and the keys
//!   cover every value of `s`.
//!
//! It becomes a `$mem_v2` of `2^|s|` words, register `k` at word `k`
//! (its initial value the word's): each register's load is a write port at
//! the constant address `k`, its enable the tree's load condition, and each
//! selection is an asynchronous read port at `s`. Writes to distinct
//! constant words never collide, so the ports' order does not matter.
//! `memports` then merges the write ports when SAT proves the enables
//! exclusive (one context writes per cycle), or turns the memory back into
//! word registers when it cannot; `bram` maps what is left to LUTRAM.

use std::collections::{BTreeMap, HashMap, HashSet};

use oxibit_hdl::netlist::{Bit, Cell, MemRead, MemWrite, Memory, Module, Op, SigId, SigSpec};
use oxibit_prof::hot;

use crate::dsp::{Canon, K};

#[derive(Debug, Default, Clone)]
pub struct RegBankStats {
    /// Banks turned into memories, their registers, bits and read ports.
    pub banks: usize,
    pub registers: usize,
    pub bits: u64,
    pub reads: usize,
    /// Registers whose next state is hold-or-load, and selection terms seen.
    pub parsed: usize,
    pub terms: usize,
    /// Candidate banks (registers sharing clock and load value) left as
    /// flops, by reason.
    pub left: BTreeMap<&'static str, usize>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
enum Kb {
    Net(usize),
    Zero,
    One,
    Other,
}

fn kb(k: K) -> Kb {
    match k {
        K::Net(n) => Kb::Net(n),
        K::Const(Bit::Zero) => Kb::Zero,
        K::Const(Bit::One) => Kb::One,
        K::Const(_) => Kb::Other,
    }
}

/// A node of a register's next-state cone (a DAG: EHDL emits a mux per
/// `if` for every register of an `on` block, shared between arms).
enum Node {
    Hold,
    Load,
    Mux { s: Bit, a: usize, b: usize },
}

/// A next-state cone being built.
#[derive(Default)]
struct Cone {
    nodes: Vec<Node>,
    memo: HashMap<Vec<Kb>, usize>,
    /// Mux cells in the cone, and how often the cone (or the flop) reads each.
    cells: Vec<usize>,
    refs: HashMap<usize, u32>,
    /// Reads of Q from the cone.
    holds: u32,
    load: Option<Vec<Kb>>,
    data: Option<SigSpec>,
    /// Whether Q is reachable through muxes from a value.
    reach: HashMap<Vec<Kb>, bool>,
}

struct Reg {
    cell: usize,
    q: SigSpec,
    qk: Vec<Kb>,
    clk: SigSpec,
    clk_k: Kb,
    posedge: bool,
    data: SigSpec,
    data_k: Vec<Kb>,
    nodes: Vec<Node>,
    root: usize,
    tree_cells: Vec<usize>,
    /// Q leaves in the tree (each is one read of every Q bit).
    holds: u32,
}

/// A selection term `(s == key) * q`.
struct Term {
    cell: usize,
    reg: usize,
    sel: SigSpec,
    sel_k: Vec<Kb>,
    key: u64,
}

struct Ctx<'a> {
    m: &'a Module,
    cn: Canon,
    /// Canonical net -> driving cell.
    driver: HashMap<usize, usize>,
    /// Reads of each canonical net (cell inputs and ports).
    uses: HashMap<usize, u32>,
}

impl Ctx<'_> {
    fn k(&mut self, b: Bit) -> Kb {
        kb(self.cn.k(b))
    }

    fn ks(&mut self, s: &SigSpec) -> Vec<Kb> {
        s.0.iter().map(|b| self.k(*b)).collect()
    }

    /// The cell driving every bit of `bits`, in order, if one does.
    fn driven_by(&mut self, bits: &[Kb]) -> Option<usize> {
        let Kb::Net(n0) = *bits.first()? else { return None };
        let ci = *self.driver.get(&n0)?;
        let m = self.m;
        let y = match &m.cells[ci] {
            Cell::Mux { y, .. } | Cell::Binary { y, .. } => y.clone(),
            _ => return None,
        };
        (self.ks(&y) == bits).then_some(ci)
    }

    fn used_once(&mut self, ci: usize) -> bool {
        let m = self.m;
        let y = match &m.cells[ci] {
            Cell::Mux { y, .. } | Cell::Binary { y, .. } => y.clone(),
            _ => return false,
        };
        y.0.iter().all(|b| match self.k(*b) {
            Kb::Net(n) => self.uses.get(&n).copied().unwrap_or(0) <= 1,
            _ => true,
        })
    }

    /// The node for `bits` in a register's next-state cone: Q (hold), a
    /// mux over further nodes, or the load value, of which a register has
    /// one.
    fn cone(&mut self, bits: Vec<Kb>, src: &SigSpec, qk: &[Kb], c: &mut Cone, depth: u32) -> Option<usize> {
        if bits == qk {
            c.holds += 1;
            c.nodes.push(Node::Hold);
            return Some(c.nodes.len() - 1);
        }
        let mux = self.driven_by(&bits).filter(|&ci| matches!(&self.m.cells[ci], Cell::Mux { s, .. } if s.0.len() == 1));
        // a mux Q is not reachable from belongs to the load value
        let mux = mux.filter(|_| self.reaches(&bits, qk, &mut c.reach, 0));
        if let Some(&ix) = c.memo.get(&bits) {
            if let Some(ci) = mux {
                *c.refs.entry(ci).or_default() += 1;
            }
            return Some(ix);
        }
        let m = self.m;
        let ix = match mux {
            Some(ci) if depth < 512 => {
                let Cell::Mux { a, b, s, .. } = &m.cells[ci] else { unreachable!("filtered to muxes") };
                *c.refs.entry(ci).or_default() += 1;
                c.cells.push(ci);
                let (ak, bk) = (self.ks(a), self.ks(b));
                let ia = self.cone(ak, a, qk, c, depth + 1)?;
                let ib = self.cone(bk, b, qk, c, depth + 1)?;
                if ia == ib {
                    ia
                } else {
                    c.nodes.push(Node::Mux { s: s.0[0], a: ia, b: ib });
                    c.nodes.len() - 1
                }
            }
            _ => {
                match &c.load {
                    Some(l) if *l != bits => return None,
                    Some(_) => {}
                    None => {
                        c.load = Some(bits.clone());
                        c.data = Some(src.clone());
                    }
                }
                c.nodes.push(Node::Load);
                c.nodes.len() - 1
            }
        };
        c.memo.insert(bits, ix);
        Some(ix)
    }

    /// Whether Q is reachable from `bits` through muxes.
    fn reaches(&mut self, bits: &[Kb], qk: &[Kb], memo: &mut HashMap<Vec<Kb>, bool>, depth: u32) -> bool {
        if bits == qk {
            return true;
        }
        if let Some(&r) = memo.get(bits) {
            return r;
        }
        memo.insert(bits.to_vec(), false);
        let m = self.m;
        let r = depth < 512
            && match self.driven_by(bits).map(|ci| &m.cells[ci]) {
                Some(Cell::Mux { a, b, s, .. }) if s.0.len() == 1 => {
                    let (ak, bk) = (self.ks(a), self.ks(b));
                    self.reaches(&ak, qk, memo, depth + 1) || self.reaches(&bk, qk, memo, depth + 1)
                }
                _ => false,
            };
        memo.insert(bits.to_vec(), r);
        r
    }

    /// Reads of a cell's output, if every bit is read the same number of times.
    fn reads_of(&mut self, ci: usize) -> Option<u32> {
        let m = self.m;
        let y = match &m.cells[ci] {
            Cell::Mux { y, .. } | Cell::Binary { y, .. } => y,
            _ => return None,
        };
        let mut n: Option<u32> = None;
        for b in &y.0 {
            let Kb::Net(x) = self.k(*b) else { return None };
            let u = self.uses.get(&x).copied().unwrap_or(0);
            match n {
                None => n = Some(u),
                Some(v) if v != u => return None,
                _ => {}
            }
        }
        n
    }

    /// `s == k` driving `net`: the word, its canonical bits and the key
    /// (`None` when the constant cannot fit the word).
    fn compare(&mut self, net: usize) -> Option<(SigSpec, Vec<Kb>, Option<u64>)> {
        let ci = *self.driver.get(&net)?;
        let m = self.m;
        let Cell::Binary { op: Op::Eq, a, b, signed: false, y } = &m.cells[ci] else {
            return None;
        };
        let (a, b, y) = (a.clone(), b.clone(), y.clone());
        if self.k(*y.0.first()?) != Kb::Net(net) {
            return None;
        }
        let konst = |me: &mut Self, s: &SigSpec| -> Option<Vec<bool>> {
            s.0.iter()
                .map(|b| match me.k(*b) {
                    Kb::Zero => Some(false),
                    Kb::One => Some(true),
                    _ => None,
                })
                .collect()
        };
        let (x, key) = match (konst(self, &a), konst(self, &b)) {
            (None, Some(k)) => (a, k),
            (Some(k), None) => (b, k),
            _ => return None,
        };
        // a zero-extended word compares as its live bits
        let full = self.ks(&x);
        let w = full.iter().rposition(|b| *b != Kb::Zero).map_or(0, |i| i + 1);
        let (x, xk) = (x.slice(0, w as u32), full[..w].to_vec());
        let fits = !key.iter().skip(w).any(|v| *v) && w <= 16;
        let v = key.iter().take(w).enumerate().fold(0u64, |acc, (i, b)| acc | (u64::from(*b) << i));
        Some((x, xk, fits.then_some(v)))
    }
}

// Selections: OR trees of terms, each internal node and term read once.
// node -> (terms under it), or None when it is not a pure selection.
fn collect(cx: &mut Ctx, ci: usize, term_of: &HashMap<usize, usize>, memo: &mut HashMap<usize, Option<Vec<usize>>>) -> Option<Vec<usize>> {
    if let Some(r) = memo.get(&ci) {
        return r.clone();
    }
    memo.insert(ci, None);
    let r = if let Some(&ti) = term_of.get(&ci) {
        Some(vec![ti])
    } else if let Cell::Binary { op: Op::Or, a, b, signed: false, .. } = &{ let m = cx.m; m }.cells[ci] {
        let (a, b) = (a.clone(), b.clone());
        let mut out = Vec::new();
        let mut ok = true;
        for s in [a, b] {
            let mut seen = HashSet::new();
            for bit in &s.0 {
                match cx.k(*bit) {
                    Kb::Zero => {}
                    Kb::Net(n) => {
                        let Some(&d) = cx.driver.get(&n) else {
                            ok = false;
                            break;
                        };
                        if !seen.insert(d) {
                            continue;
                        }
                        match collect(cx, d, term_of, memo) {
                            Some(v) if cx.used_once(d) => out.extend(v),
                            _ => {
                                ok = false;
                                break;
                            }
                        }
                    }
                    _ => {
                        ok = false;
                        break;
                    }
                }
            }
        }
        ok.then_some(out)
    } else {
        None
    };
    memo.insert(ci, r.clone());
    r
}

struct Plan {
    regs: Vec<usize>,
    addr_w: u32,
    key_of: HashMap<usize, u64>,
    roots: Vec<usize>,
    drop: Vec<usize>,
}

/// Banks of registers to memories.
#[hot]
pub fn convert(m: &mut Module) -> RegBankStats {
    let mut st = RegBankStats::default();
    let mr: &Module = m;
    let mut cx = Ctx {
        m: mr,
        cn: Canon::new(mr),
        driver: HashMap::new(),
        uses: HashMap::new(),
    };
    for (ci, c) in mr.cells.iter().enumerate() {
        let y = match c {
            Cell::Mux { y, .. } | Cell::Binary { y, .. } | Cell::Unary { y, .. } => y,
            _ => continue,
        };
        for b in &y.0 {
            if let K::Net(n) = cx.cn.k(*b) {
                cx.driver.entry(n).or_insert(ci);
            }
        }
    }
    {
        let count = |s: &SigSpec, cx: &mut Ctx| {
            for b in &s.0 {
                if let Kb::Net(n) = cx.k(*b) {
                    *cx.uses.entry(n).or_default() += 1;
                }
            }
        };
        for c in &mr.cells {
            match c {
                Cell::Unary { a, .. } => count(a, &mut cx),
                Cell::Binary { a, b, .. } => {
                    count(a, &mut cx);
                    count(b, &mut cx);
                }
                Cell::Mux { a, b, s, .. } => {
                    count(a, &mut cx);
                    count(b, &mut cx);
                    count(s, &mut cx);
                }
                Cell::Dff { clk, d, .. } => {
                    count(clk, &mut cx);
                    count(d, &mut cx);
                }
                Cell::Adff { clk, arst, d, .. } => {
                    count(clk, &mut cx);
                    count(arst, &mut cx);
                    count(d, &mut cx);
                }
                Cell::Mem(mem) => {
                    for r in &mem.reads {
                        count(&r.addr, &mut cx);
                        if let Some((c, _)) = &r.clk {
                            count(c, &mut cx);
                        }
                    }
                    for w in &mem.writes {
                        count(&w.clk.0, &mut cx);
                        count(&w.addr, &mut cx);
                        count(&w.data, &mut cx);
                        count(&w.en, &mut cx);
                    }
                }
                Cell::Instance { conns, .. } => conns.values().for_each(|s| count(s, &mut cx)),
            }
        }
        for (i, s) in mr.signals.iter().enumerate() {
            if s.port.is_some() {
                count(&SigSpec::of(SigId(i as u32), s.width), &mut cx);
            }
        }
    }

    // Registers that hold or load one value.
    let mut regs: Vec<Reg> = Vec::new();
    for (ci, c) in mr.cells.iter().enumerate() {
        let Cell::Dff { clk, posedge, d, q } = c else { continue };
        let qk = cx.ks(q);
        if qk.iter().any(|b| !matches!(b, Kb::Net(_))) || qk.iter().collect::<HashSet<_>>().len() != qk.len() {
            continue;
        }
        let mut c = Cone::default();
        let dk = cx.ks(d);
        let Some(root) = cx.cone(dk, d, &qk, &mut c, 0) else {
            continue;
        };
        let (Some(data_k), Some(data)) = (c.load.take(), c.data.take()) else { continue };
        if c.holds == 0 {
            continue;
        }
        // Every cone mux is read by the cone (or the flop) only.
        c.cells.sort_unstable();
        c.cells.dedup();
        let private = c.cells.iter().all(|&ci| cx.reads_of(ci) == c.refs.get(&ci).copied());
        if !private {
            *st.left.entry("next-state logic shared").or_default() += 1;
            continue;
        }
        st.parsed += 1;
        let clk_k = cx.k(clk.0[0]);
        regs.push(Reg {
            cell: ci,
            q: q.clone(),
            qk,
            clk: clk.clone(),
            clk_k,
            posedge: *posedge,
            data,
            data_k,
            nodes: c.nodes,
            root,
            tree_cells: c.cells,
            holds: c.holds,
        });
    }
    // Candidate banks: same clock, edge and load value.
    let mut banks: BTreeMap<(Kb, bool, Vec<Kb>), Vec<usize>> = BTreeMap::new();
    for (ri, r) in regs.iter().enumerate() {
        banks.entry((r.clk_k, r.posedge, r.data_k.clone())).or_default().push(ri);
    }
    banks.retain(|_, v| v.len() >= 2);
    let reg_of: HashMap<Vec<Kb>, usize> = banks.values().flatten().map(|&ri| (regs[ri].qk.clone(), ri)).collect();

    // Selection terms over bank registers.
    let mut terms: Vec<Term> = Vec::new();
    for (ci, c) in mr.cells.iter().enumerate() {
        let Cell::Binary { op: op @ (Op::Mul | Op::And), a, b, signed: false, .. } = c else { continue };
        for (sel_side, val) in [(a, b), (b, a)] {
            let sk = cx.ks(sel_side);
            let Some(Kb::Net(sn)) = sk.first().copied() else { continue };
            let ok = match op {
                Op::Mul => sk[1..].iter().all(|b| *b == Kb::Zero),
                _ => sk.iter().all(|b| *b == Kb::Net(sn)),
            };
            if !ok {
                continue;
            }
            let Some((sel, sel_k, Some(key))) = cx.compare(sn) else { continue };
            let vk = cx.ks(val);
            let n = vk.iter().rposition(|b| *b != Kb::Zero).map_or(0, |i| i + 1);
            if let Some(&ri) = reg_of.get(&vk[..n]) {
                terms.push(Term { cell: ci, reg: ri, sel, sel_k, key });
                break;
            }
        }
    }
    st.terms = terms.len();
    if std::env::var_os("OXIBIT_REGBANK_DEBUG").is_some() {
        let name = |q: &SigSpec| match q.0[0] {
            Bit::Sig(id, i) => format!("{}[{i}]", mr.signal(id).name),
            _ => "?".into(),
        };
        for (key, v) in banks.iter().take(12) {
            eprintln!("regbank debug: bank of {} (data {} bits): {}", v.len(), key.2.len(), v.iter().map(|&ri| name(&regs[ri].q)).collect::<Vec<_>>().join(" "));
        }
        let (mut muls, mut sel_ok, mut cmp_ok) = (0, 0, 0);
        let mut vals: Vec<String> = Vec::new();
        for c in &mr.cells {
            let Cell::Binary { op: op @ (Op::Mul | Op::And), a, b, signed, .. } = c else { continue };
            muls += 1;
            for (sel_side, val) in [(a, b), (b, a)] {
                let sk = cx.ks(sel_side);
                let Some(Kb::Net(sn)) = sk.first().copied() else { continue };
                let ok = match op {
                    Op::Mul => sk[1..].iter().all(|b| *b == Kb::Zero),
                    _ => sk.iter().all(|b| *b == Kb::Net(sn)),
                };
                if !ok {
                    continue;
                }
                sel_ok += 1;
                let why_cmp = match cx.driver.get(&sn).map(|&d| &mr.cells[d]) {
                    None => "no driver".to_string(),
                    Some(Cell::Binary { op, signed, a, b, .. }) => format!("{op:?} signed={signed} wa={} wb={}", a.0.len(), b.0.len()),
                    Some(Cell::Unary { op, .. }) => format!("{op:?}"),
                    Some(Cell::Mux { .. }) => "mux".into(),
                    Some(_) => "other".into(),
                };
                let vk = cx.ks(val);
                let nn = vk.iter().rposition(|b| *b != Kb::Zero).map_or(0, |i| i + 1);
                let is_bank = reg_of.contains_key(&vk[..nn]);
                if is_bank && vals.len() < 12 {
                    vals.push(format!("bank term {}: cmp {:?}", name(val), cx.compare(sn).map(|(x, _, k)| (x.0.len(), k))));
                }
                if false && vals.len() < 12 && cx.compare(sn).is_none() {
                    vals.push(format!("cmp fail on {}: {why_cmp}", name(val)));
                }
                if let Some((_, _, Some(_))) = cx.compare(sn) {
                    cmp_ok += 1;
                    if vals.len() < 6 {
                        vals.push(format!("{} signed={signed} w={}", name(val), val.0.len()));
                    }
                }
            }
        }
        eprintln!("regbank debug: mul/and {muls}, select shape {sel_ok}, compare {cmp_ok}, examples {vals:?}");
    }
    let term_of: HashMap<usize, usize> = terms.iter().enumerate().map(|(ti, t)| (t.cell, ti)).collect();
    let mut memo: HashMap<usize, Option<Vec<usize>>> = HashMap::new();
    // Roots: OR nodes (or a lone term) over terms, not inside another such node.
    let mut inner: HashSet<usize> = HashSet::new();
    let mut site_terms: HashMap<usize, Vec<usize>> = HashMap::new();
    for (ci, c) in mr.cells.iter().enumerate() {
        if !matches!(c, Cell::Binary { op: Op::Or, .. }) {
            continue;
        }
        if let Some(ts) = collect(&mut cx, ci, &term_of, &mut memo)
            && ts.len() >= 2
        {
            site_terms.insert(ci, ts);
        }
    }
    for (&ci, _) in &site_terms {
        if let Cell::Binary { a, b, .. } = &mr.cells[ci] {
            for s in [a, b] {
                for bit in &s.0 {
                    if let Kb::Net(n) = cx.k(*bit)
                        && let Some(&d) = cx.driver.get(&n)
                    {
                        inner.insert(d);
                    }
                }
            }
        }
    }
    site_terms.retain(|ci, _| !inner.contains(ci));
    // Sites by bank.
    let mut sites_of: HashMap<usize, Vec<usize>> = HashMap::new(); // first reg -> roots
    let bank_of: HashMap<usize, usize> = banks.values().flat_map(|v| v.iter().map(move |&ri| (ri, v[0]))).collect();
    for (&root, ts) in &site_terms {
        if let Some(&b) = bank_of.get(&terms[ts[0]].reg) {
            sites_of.entry(b).or_default().push(root);
        }
    }

    // Check each bank and plan its memory.
    let mut plans: Vec<Plan> = Vec::new();
    'bank: for members in banks.values() {
        let first = members[0];
        let left = |why: &'static str, st: &mut RegBankStats| {
            *st.left.entry(why).or_default() += 1;
        };
        let Some(roots) = sites_of.get(&first) else {
            left("no selection read", &mut st);
            continue;
        };
        let mut roots = roots.clone();
        roots.sort_unstable();
        let mut key_of: HashMap<usize, u64> = HashMap::new();
        // each selection is a read port with its own address word; the
        // words are the same width
        let mut sel_w: Option<usize> = None;
        let mut reads_of: HashMap<usize, u32> = HashMap::new();
        let mut drop: Vec<usize> = Vec::new();
        for &root in &roots {
            let ts = &site_terms[&root];
            let mut seen_regs = HashSet::new();
            let mut sel_k: Option<Vec<Kb>> = None;
            for &ti in ts {
                let t = &terms[ti];
                if !members.contains(&t.reg) {
                    left("selection mixes banks", &mut st);
                    continue 'bank;
                }
                if !seen_regs.insert(t.reg) {
                    left("register selected twice", &mut st);
                    continue 'bank;
                }
                match &sel_k {
                    None => sel_k = Some(t.sel_k.clone()),
                    Some(s) if *s != t.sel_k => {
                        left("a selection over two words", &mut st);
                        continue 'bank;
                    }
                    _ => {}
                }
                match sel_w {
                    None => sel_w = Some(t.sel_k.len()),
                    Some(w) if w != t.sel_k.len() => {
                        left("selections of different widths", &mut st);
                        continue 'bank;
                    }
                    _ => {}
                }
                match key_of.get(&t.reg) {
                    None => {
                        key_of.insert(t.reg, t.key);
                    }
                    Some(&k) if k != t.key => {
                        left("register under different keys", &mut st);
                        continue 'bank;
                    }
                    _ => {}
                }
                *reads_of.entry(t.reg).or_default() += 1;
                drop.push(t.cell);
            }
            if seen_regs.len() != members.len() {
                left("selection misses a register", &mut st);
                continue 'bank;
            }
            // the root and the ORs under it
            let mut stack = vec![root];
            while let Some(ci) = stack.pop() {
                if term_of.contains_key(&ci) {
                    continue;
                }
                drop.push(ci);
                if let Cell::Binary { a, b, .. } = &mr.cells[ci] {
                    for s in [a, b] {
                        for bit in &s.0 {
                            if let Kb::Net(n) = cx.k(*bit)
                                && let Some(&d) = cx.driver.get(&n)
                                && !drop.contains(&d)
                                && !stack.contains(&d)
                            {
                                stack.push(d);
                            }
                        }
                    }
                }
            }
        }
        let w = sel_w.unwrap_or(0) as u32;
        let keys: HashSet<u64> = key_of.values().copied().collect();
        if w == 0 || w > 8 || members.len() != 1usize << w || keys.len() != members.len() {
            left("keys do not cover the select word", &mut st);
            continue;
        }
        // No other reader of any Q bit.
        for &ri in members {
            let r = &regs[ri];
            let want = r.holds + reads_of.get(&ri).copied().unwrap_or(0);
            let qk = r.qk.clone();
            if qk.iter().any(|b| match b {
                Kb::Net(n) => cx.uses.get(n).copied().unwrap_or(0) != want,
                _ => true,
            }) {
                left("register read elsewhere", &mut st);
                continue 'bank;
            }
        }
        plans.push(Plan {
            regs: members.clone(),
            addr_w: w,
            key_of,
            roots,
            drop,
        });
    }
    // Root outputs and selects, captured before the module changes.
    let root_y: HashMap<usize, SigSpec> = plans
        .iter()
        .flat_map(|p| p.roots.iter())
        .map(|&ci| match &mr.cells[ci] {
            Cell::Binary { y, .. } => (ci, y.clone()),
            _ => unreachable!("roots are ORs"),
        })
        .collect();
    let root_sel: HashMap<usize, SigSpec> = plans
        .iter()
        .flat_map(|p| p.roots.iter())
        .map(|&r| (r, terms[site_terms[&r][0]].sel.clone()))
        .collect();
    drop(cx);

    // Rewrite.
    let mut gone: HashSet<usize> = HashSet::new();
    let mut add: Vec<Cell> = Vec::new();
    for p in &plans {
        let r0 = &regs[p.regs[0]];
        let width = r0.q.width();
        let size = 1u32 << p.addr_w;
        let mut init: BTreeMap<u32, Vec<Option<bool>>> = BTreeMap::new();
        let mut writes = Vec::new();
        for &ri in &p.regs {
            let r = &regs[ri];
            let k = p.key_of[&ri] as u32;
            let word: Vec<Option<bool>> = r
                .q
                .0
                .iter()
                .map(|b| match b {
                    Bit::Sig(id, i) => m.signal(*id).init.as_ref().and_then(|v| v.get(*i as usize).copied().flatten()),
                    _ => None,
                })
                .collect();
            if word.iter().any(|b| b.is_some()) {
                init.insert(k, word);
            }
            let mut memo = HashMap::new();
            let en = enable(m, &mut add, &r.nodes, r.root, &mut memo);
            writes.push(MemWrite {
                clk: (r.clk.clone(), r.posedge),
                addr: SigSpec::constant(u64::from(k), p.addr_w),
                data: r.data.clone(),
                en: SigSpec(vec![en; width as usize]),
            });
            gone.insert(r.cell);
            gone.extend(r.tree_cells.iter().copied());
        }
        let mut reads = Vec::new();
        for &root in &p.roots {
            let rd = m.wire(width);
            reads.push(MemRead {
                clk: None,
                addr: root_sel[&root].clone(),
                data: rd.clone(),
            });
            let y = &root_y[&root];
            m.connects.push((y.clone(), rd.extend(y.width(), false)));
        }
        gone.extend(p.drop.iter().copied());
        let name = m.signal(match r0.q.0[0] {
            Bit::Sig(id, _) => id,
            _ => unreachable!("register Q bits are signals"),
        })
        .name
        .clone();
        st.banks += 1;
        st.registers += p.regs.len();
        st.bits += u64::from(width) * p.regs.len() as u64;
        st.reads += reads.len();
        add.push(Cell::Mem(Memory {
            name: format!("regbank${name}"),
            width,
            size,
            offset: 0,
            init,
            attrs: BTreeMap::new(),
            reads,
            writes,
        }));
    }
    let mut i = 0;
    m.cells.retain(|_| {
        let keep = !gone.contains(&i);
        i += 1;
        keep
    });
    m.cells.extend(add);
    st
}

/// The load condition of a next-state cone node, as a bit.
fn enable(m: &mut Module, out: &mut Vec<Cell>, nodes: &[Node], ix: usize, memo: &mut HashMap<usize, Bit>) -> Bit {
    if let Some(&b) = memo.get(&ix) {
        return b;
    }
    let r = match &nodes[ix] {
        Node::Hold => Bit::Zero,
        Node::Load => Bit::One,
        Node::Mux { s, a, b } => {
            let (ea, eb) = (enable(m, out, nodes, *a, memo), enable(m, out, nodes, *b, memo));
            match (ea, eb) {
                (x, y) if x == y => x,
                (Bit::Zero, Bit::One) => *s,
                _ => {
                    let y = m.wire(1);
                    out.push(Cell::Mux {
                        a: SigSpec(vec![ea]),
                        b: SigSpec(vec![eb]),
                        s: SigSpec(vec![*s]),
                        y: y.clone(),
                    });
                    y.0[0]
                }
            }
        }
    };
    memo.insert(ix, r);
    r
}

/// Banks of per-context registers to memories.
pub struct RegBank;

impl crate::stage::Pass for RegBank {
    const NAME: &'static str = "regbank";
    const EMITS: &'static [crate::caps::Mode] = &[];
    type Stats = RegBankStats;
    fn enabled(p: &crate::policy::PassSet) -> bool {
        p.regbank
    }
    fn json(s: &RegBankStats) -> serde_json::Value {
        serde_json::json!({ "banks": s.banks, "registers": s.registers, "bits": s.bits, "reads": s.reads, "parsed": s.parsed, "terms": s.terms, "left": s.left })
    }
}

impl crate::stage::RtlPass for RegBank {
    fn run(&self, m: &mut crate::stage::Rtl) -> RegBankStats {
        convert(&mut m.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxibit_hdl::netlist::Signal;

    fn sig(m: &mut Module, name: &str, w: u32) -> SigSpec {
        let id = m.add_signal(Signal { name: name.into(), width: w, signed: false, port: None, init: Some(vec![Some(false); w as usize]) });
        SigSpec::of(id, w)
    }

    fn port(m: &mut Module, name: &str, w: u32, dir: oxibit_hdl::netlist::Dir) -> SigSpec {
        let id = m.add_signal(Signal { name: name.into(), width: w, signed: false, port: Some(dir), init: None });
        SigSpec::of(id, w)
    }

    fn bin(m: &mut Module, op: Op, a: SigSpec, b: SigSpec, w: u32) -> SigSpec {
        let y = m.wire(w);
        m.cells.push(Cell::Binary { op, a, b, signed: false, y: y.clone() });
        y
    }

    fn mux(m: &mut Module, a: SigSpec, b: SigSpec, s: SigSpec) -> SigSpec {
        let y = m.wire(a.width());
        m.cells.push(Cell::Mux { a, b, s, y: y.clone() });
        y
    }

    /// Four 8-bit registers `x0..x3`: `if we && ws == k then xk = data`,
    /// read as `OR_k (rs == k) * xk` into an output; `extra` adds a second
    /// reader of x2.
    fn bank(extra: bool) -> Module {
        use oxibit_hdl::netlist::Dir;
        let mut m = Module::new("t");
        let clk = port(&mut m, "clk", 1, Dir::Input);
        let we = port(&mut m, "we", 1, Dir::Input);
        let ws = port(&mut m, "ws", 2, Dir::Input);
        let rs = port(&mut m, "rs", 2, Dir::Input);
        let data = port(&mut m, "data", 8, Dir::Input);
        let out = port(&mut m, "out", 9, Dir::Output);
        let mut acc: Option<SigSpec> = None;
        for k in 0..4u64 {
            let q = sig(&mut m, &format!("x{k}"), 8);
            let hit = bin(&mut m, Op::Eq, ws.clone(), SigSpec::constant(k, 2), 1);
            let inner = mux(&mut m, q.clone(), data.clone(), hit);
            let d = mux(&mut m, q.clone(), inner, we.clone());
            m.cells.push(Cell::Dff { clk: clk.clone(), posedge: true, d, q: q.clone() });
            let e = bin(&mut m, Op::Eq, rs.clone(), SigSpec::constant(k, 2), 1);
            let t = bin(&mut m, Op::Mul, e.extend(9, false), q.clone(), 9);
            acc = Some(match acc {
                None => t,
                Some(a) => bin(&mut m, Op::Or, a, t, 9),
            });
            if extra && k == 2 {
                let o2 = port(&mut m, "peek", 8, Dir::Output);
                m.connects.push((o2, q.clone()));
            }
        }
        m.connects.push((out, acc.expect("terms")));
        m
    }

    #[test]
    fn a_bank_becomes_a_memory() {
        let mut m = bank(false);
        let st = convert(&mut m);
        assert_eq!((st.banks, st.registers, st.bits, st.reads), (1, 4, 32, 1));
        assert!(!m.cells.iter().any(|c| matches!(c, Cell::Dff { .. })));
        let mem = m.cells.iter().find_map(|c| if let Cell::Mem(x) = c { Some(x) } else { None }).expect("memory");
        assert_eq!((mem.size, mem.width, mem.writes.len(), mem.reads.len()), (4, 8, 4, 1));
        assert_eq!(mem.init.len(), 4);
    }

    #[test]
    fn another_reader_keeps_the_flops() {
        let mut m = bank(true);
        let st = convert(&mut m);
        assert_eq!(st.banks, 0);
        assert_eq!(st.left.get("register read elsewhere"), Some(&1));
    }
}
