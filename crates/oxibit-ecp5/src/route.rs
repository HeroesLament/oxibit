//! Negotiated-congestion routing (PathFinder) on the ECP5 routing graph.
//!
//! Each net is a source wire and sink wires. A net's tree grows one sink at
//! a time by A* from every wire already in the tree; a wire costs
//! `(1 + history) * (1 + present * others)`, `others` being how many other
//! nets hold it. After every pass, wires held by more than one net raise
//! their history and the present factor grows, until nothing is shared.
//!
//! Wires joined by fixed connections are one electrical node, so a net
//! holds the whole fixed-connected group of every wire in its tree; two
//! nets touching one group conflict even on different wires of it.
//!
//! Signal nets never use the global clock network (`G_`, `L_`, `R_` wires
//! and the `HPBX`/`VPTX`/`HPRX`/`VPSX`/`HSBX` spines and branches); clock
//! nets may.

use std::cmp::Reverse;
use std::collections::{BTreeSet, BinaryHeap, HashMap, HashSet};

use crate::graph::{Arc, Graph, PipKind, Wire};
use crate::{Result, err};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NetKind {
    Signal,
    Clock,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Net {
    pub name: String,
    pub source: Wire,
    pub sinks: Vec<Wire>,
    pub kind: NetKind,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Options {
    pub max_passes: usize,
    /// Present-congestion factor of the first pass and its growth per pass.
    pub present: f64,
    pub present_growth: f64,
    /// History added to a wire per pass it is overused.
    pub history: f64,
    /// Tiles beyond a net's bounding box the search may leave it by.
    pub margin: i16,
}

impl Default for Options {
    fn default() -> Options {
        // Tuned-by-us starting values, not swept: PathFinder's usual shape
        // (present factor from ~0.5 growing ~1.5x, history ~1 per pass).
        Options {
            max_passes: 50,
            present: 0.5,
            present_growth: 1.5,
            history: 1.0,
            margin: 8,
        }
    }
}

/// The routed arcs of every net, in net order; each list is a tree from the
/// net's source to all its sinks.
#[derive(Clone, Debug, Default)]
pub struct Routes {
    pub nets: Vec<Vec<Arc>>,
    pub passes: usize,
}

/// Whether a wire belongs to the global clock network.
pub fn is_global(g: &Graph, w: Wire) -> bool {
    let name = &g.names[w.name as usize];
    ["G_", "L_", "R_"].iter().any(|p| name.starts_with(p))
        || ["HPBX", "VPTX", "HPRX", "VPSX", "HSBX"]
            .iter()
            .any(|k| name.contains(k))
}

/// The wires joined to `w` by fixed connections, `w` included.
fn fixed_group(g: &Graph, w: Wire, cache: &mut HashMap<Wire, Vec<Wire>>) -> Vec<Wire> {
    if let Some(v) = cache.get(&w) {
        return v.clone();
    }
    let mut seen = BTreeSet::from([w]);
    let mut stack = vec![w];
    while let Some(x) = stack.pop() {
        for a in g.downhill(x).into_iter().chain(g.uphill(x)) {
            if g.pip(&a).kind != PipKind::Fixed {
                continue;
            }
            for y in [a.from, a.to] {
                if seen.insert(y) {
                    stack.push(y);
                }
            }
        }
    }
    let v: Vec<Wire> = seen.into_iter().collect();
    for &m in &v {
        cache.insert(m, v.clone());
    }
    v
}

#[derive(Clone, Copy, Debug)]
struct Bbox {
    x0: i16,
    x1: i16,
    y0: i16,
    y1: i16,
}

impl Bbox {
    fn of(net: &Net, margin: i16) -> Bbox {
        let ws = std::iter::once(net.source).chain(net.sinks.iter().copied());
        let (mut x0, mut x1, mut y0, mut y1) = (i16::MAX, i16::MIN, i16::MAX, i16::MIN);
        for w in ws {
            x0 = x0.min(w.x);
            x1 = x1.max(w.x);
            y0 = y0.min(w.y);
            y1 = y1.max(w.y);
        }
        Bbox {
            x0: x0 - margin,
            x1: x1 + margin,
            y0: y0 - margin,
            y1: y1 + margin,
        }
    }

    fn contains(self, w: Wire) -> bool {
        (self.x0..=self.x1).contains(&w.x) && (self.y0..=self.y1).contains(&w.y)
    }
}

/// Router state across passes. Wires are tracked by fixed-connection
/// group (one electrical node): users, history and reservations per group.
struct State<'g> {
    g: &'g Graph,
    opts: Options,
    /// Group of each wire seen so far, and each group's wires.
    gid: HashMap<Wire, u32>,
    members: Vec<Vec<Wire>>,
    /// Nets holding each group.
    users: Vec<Vec<u32>>,
    history: Vec<f64>,
    present: f64,
    /// Groups no net here may use (other trees, e.g. clocks).
    reserved: HashSet<u32>,
    /// Groups each net holds.
    held: Vec<Vec<u32>>,
    /// Per wire name: unknown, local, global.
    global: Vec<u8>,
}

/// A* order key: f-cost in fixed point, then insertion order.
type Key = Reverse<(u64, u64)>;

fn fixed_point(c: f64) -> u64 {
    // Costs are small positive reals; 1e-3 resolution is enough to order them.
    #[expect(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "non-negative, bounded cost"
    )]
    let v = (c * 1000.0).round() as u64;
    v
}

impl<'g> State<'g> {
    fn new(g: &'g Graph, opts: Options, nets: usize) -> State<'g> {
        State {
            g,
            opts,
            gid: HashMap::new(),
            members: Vec::new(),
            users: Vec::new(),
            history: Vec::new(),
            present: opts.present,
            reserved: HashSet::new(),
            held: vec![Vec::new(); nets],
            global: vec![0; g.names.len()],
        }
    }

    /// The group of `w`, found on first use.
    fn group(&mut self, w: Wire) -> u32 {
        if let Some(&id) = self.gid.get(&w) {
            return id;
        }
        let mut cache = HashMap::new();
        let v = fixed_group(self.g, w, &mut cache);
        let id = u32::try_from(self.members.len()).expect("group count fits u32");
        for &m in &v {
            self.gid.insert(m, id);
        }
        self.members.push(v);
        self.users.push(Vec::new());
        self.history.push(0.0);
        id
    }

    fn is_global(&mut self, w: Wire) -> bool {
        let k = w.name as usize;
        if self.global[k] == 0 {
            self.global[k] = if is_global(self.g, w) { 2 } else { 1 };
        }
        self.global[k] == 2
    }

    fn cost(&mut self, w: Wire, net: u32) -> f64 {
        let id = self.group(w) as usize;
        let others = self.users[id].iter().filter(|&&n| n != net).count();
        #[expect(clippy::cast_precision_loss, reason = "a handful of nets")]
        let o = others as f64;
        (1.0 + self.history[id]) * (1.0 + self.present * o)
    }

    fn hold(&mut self, w: Wire, net: u32) {
        let id = self.group(w);
        let u = &mut self.users[id as usize];
        if !u.contains(&net) {
            u.push(net);
            self.held[net as usize].push(id);
        }
    }

    fn release(&mut self, net: u32) {
        for id in std::mem::take(&mut self.held[net as usize]) {
            self.users[id as usize].retain(|&n| n != net);
        }
    }

    /// A* from the tree to `sink`; returns the arcs from a tree wire to it.
    fn search(
        &mut self,
        net: u32,
        n: &Net,
        tree: &HashSet<Wire>,
        order: &[Wire],
        sink: Wire,
        bbox: Bbox,
    ) -> Result<Vec<Arc>> {
        let h = |w: Wire| {
            // Not admissible (the longest wires span 6 tiles) but close:
            // most hops are 1 or 2 tiles, and a tighter estimate keeps
            // the search from flooding the bounding box.
            f64::from((w.x - sink.x).abs() + (w.y - sink.y).abs()) / 3.0
        };
        let mut best: HashMap<Wire, f64> = HashMap::new();
        let mut parent: HashMap<Wire, Arc> = HashMap::new();
        let mut heap: BinaryHeap<(Key, Wire)> = BinaryHeap::new();
        let mut tick = 0u64;
        // Seed in insertion order so ties break the same way every run.
        for &w in order {
            best.insert(w, 0.0);
            heap.push((Reverse((fixed_point(h(w)), tick)), w));
            tick += 1;
        }
        let mut done: HashSet<Wire> = HashSet::new();
        while let Some((_, w)) = heap.pop() {
            if w == sink {
                let mut path = Vec::new();
                let mut x = sink;
                while !tree.contains(&x) {
                    let a = parent[&x];
                    path.push(a);
                    x = a.from;
                }
                path.reverse();
                return Ok(path);
            }
            if !done.insert(w) {
                continue;
            }
            let gw = best[&w];
            for a in self.g.downhill(w) {
                let v = a.to;
                if !bbox.contains(v) || (n.kind == NetKind::Signal && self.is_global(v)) {
                    continue;
                }
                let gv = self.group(v);
                if self.reserved.contains(&gv) {
                    continue;
                }
                let c = gw + self.cost(v, net);
                if best.get(&v).is_none_or(|&b| c < b) {
                    best.insert(v, c);
                    parent.insert(v, a);
                    heap.push((Reverse((fixed_point(c + h(v)), tick)), v));
                    tick += 1;
                }
            }
        }
        Err(err!(
            "net {}: no path to {}",
            n.name,
            self.g.wire_name(sink)
        ))
    }
}

/// Route all nets. Fails if a sink is unreachable or congestion persists
/// after `max_passes`.
pub fn route(g: &Graph, nets: &[Net], opts: Options) -> Result<Routes> {
    route_with(g, nets, opts, &HashSet::new())
}

/// `route`, keeping off `reserved` wires (and everything fixed-connected
/// to them): trees routed before, such as clocks.
pub fn route_with(
    g: &Graph,
    nets: &[Net],
    opts: Options,
    reserved: &HashSet<Wire>,
) -> Result<Routes> {
    let mut st = State::new(g, opts, nets.len());
    let mut reserved: Vec<Wire> = reserved.iter().copied().collect();
    reserved.sort();
    for &w in &reserved {
        let id = st.group(w);
        st.reserved.insert(id);
    }
    let mut routes = vec![Vec::new(); nets.len()];
    // Nets to (re)route this pass: all at first, then those on overused
    // groups.
    let mut dirty = vec![true; nets.len()];
    for pass in 1..=opts.max_passes {
        for (i, n) in nets.iter().enumerate() {
            if !dirty[i] {
                continue;
            }
            let ni = u32::try_from(i).expect("net count fits u32");
            st.release(ni);
            let bbox = Bbox::of(n, opts.margin);
            let mut tree = HashSet::from([n.source]);
            let mut order = vec![n.source];
            st.hold(n.source, ni);
            let mut arcs = Vec::new();
            let mut sinks = n.sinks.clone();
            sinks.sort_by_key(|s| (s.x - n.source.x).abs() + (s.y - n.source.y).abs());
            for sink in sinks {
                if tree.contains(&sink) {
                    continue;
                }
                for a in st.search(ni, n, &tree, &order, sink, bbox)? {
                    if tree.insert(a.to) {
                        order.push(a.to);
                    }
                    st.hold(a.to, ni);
                    arcs.push(a);
                }
            }
            routes[i] = arcs;
        }
        let overused: Vec<usize> = (0..st.users.len()).filter(|&k| st.users[k].len() > 1).collect();
        if overused.is_empty() {
            return Ok(Routes {
                nets: routes,
                passes: pass,
            });
        }
        dirty.fill(false);
        for k in overused {
            st.history[k] += st.opts.history;
            for &n in &st.users[k] {
                dirty[n as usize] = true;
            }
        }
        st.present *= st.opts.present_growth;
    }
    Err(err!("congestion remains after {} passes", opts.max_passes))
}

/// Check routed trees: every sink reached from the source through the net's
/// own arcs, and no electrical node (fixed group) held by two nets.
pub fn check(g: &Graph, nets: &[Net], routes: &Routes) -> Result<()> {
    let mut groups = HashMap::new();
    let mut owner: HashMap<Wire, usize> = HashMap::new();
    for (i, (n, arcs)) in nets.iter().zip(&routes.nets).enumerate() {
        let mut reached = HashSet::from([n.source]);
        let mut changed = true;
        while changed {
            changed = false;
            for a in arcs {
                if reached.contains(&a.from) && reached.insert(a.to) {
                    changed = true;
                }
            }
        }
        for s in &n.sinks {
            if !reached.contains(s) {
                return Err(err!("net {}: sink {} not reached", n.name, g.wire_name(*s)));
            }
        }
        for &w in &reached {
            for m in fixed_group(g, w, &mut groups) {
                if let Some(&o) = owner.get(&m) {
                    if o != i {
                        return Err(err!(
                            "nets {} and {} share {}",
                            nets[o].name,
                            n.name,
                            g.wire_name(m)
                        ));
                    }
                }
                owner.insert(m, i);
            }
        }
    }
    Ok(())
}
