//! Replication: copies of a cell whose sinks lie far apart, each copy
//! driving the sinks near it.
//!
//! A placer puts a cell in one spot. A LUT or flop that drives sinks all
//! over the device lands between them and is far from each, and no net
//! weight shortens that: the cell is one thing serving many places. A copy
//! per group of sinks is. `plan` picks the cells and groups from a routed
//! design's timed arcs (`sta`'s STA_ARCS); `apply` makes the copies in a
//! netlist before it is packed. The plan is a text file, so the same plan
//! and netlist give the same result, and no plan gives the netlist as read.

use std::collections::BTreeMap;

use crate::netlist::{CellId, Dir, Netlist};

/// Which cells `plan` copies, and how far.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlanOpts {
    /// Fewest sinks a cell must drive.
    pub min_fanout: usize,
    /// Least mean routed delay to its sinks, ps.
    pub min_mean_ps: f64,
    /// A group of sinks no wider or taller than this, in tiles, is left whole.
    pub span: i64,
    /// Most groups per cell (the original keeps one).
    pub max_groups: usize,
    /// Most copies over the whole design.
    pub budget: usize,
}

impl Default for PlanOpts {
    /// Ours, not swept.
    fn default() -> PlanOpts {
        PlanOpts { min_fanout: 8, min_mean_ps: 1500.0, span: 24, max_groups: 8, budget: 2000 }
    }
}

/// What a plan holds.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct PlanStats {
    /// LUT and flop drivers seen.
    pub drivers: usize,
    /// Of those, with the fan-out and delay to qualify.
    pub candidates: usize,
    /// Cells given at least one copy.
    pub copied: usize,
    pub copies: usize,
    pub sinks_moved: usize,
}

#[derive(Debug, Clone)]
struct Sink {
    dx: i64,
    dy: i64,
    name: String,
}

/// Halve `sinks` along their wider axis until each group fits `span` or
/// `allowed` groups are used.
fn split(mut sinks: Vec<Sink>, span: i64, allowed: usize, out: &mut Vec<Vec<Sink>>) {
    let ext = |f: fn(&Sink) -> i64| sinks.iter().map(f).max().unwrap_or(0) - sinks.iter().map(f).min().unwrap_or(0);
    let (ex, ey) = (ext(|s| s.dx), ext(|s| s.dy));
    if allowed < 2 || sinks.len() < 2 || ex.max(ey) <= span {
        out.push(sinks);
        return;
    }
    if ex >= ey {
        sinks.sort_by(|a, b| (a.dx, a.dy, &a.name).cmp(&(b.dx, b.dy, &b.name)));
    } else {
        sinks.sort_by(|a, b| (a.dy, a.dx, &a.name).cmp(&(b.dy, b.dx, &b.name)));
    }
    let far = sinks.split_off(sinks.len() / 2);
    split(sinks, span, allowed / 2, out);
    split(far, span, allowed - allowed / 2, out);
}

/// A plan from STA_ARCS text: `driver <tab> copy <tab> sink` per sink that
/// moves to a copy (copies numbered from 1; a sink not listed stays).
/// Worst cells first, by total delay to their sinks, while `budget` lasts.
pub fn plan(arcs: &str, o: &PlanOpts) -> Result<(String, PlanStats), String> {
    let mut by: BTreeMap<&str, (Vec<Sink>, i64)> = BTreeMap::new();
    for (ln, line) in arcs.lines().enumerate() {
        let f: Vec<&str> = line.split('\t').collect();
        if f.len() < 8 {
            return Err(format!("line {}: {} columns; this needs sta's dx, dy and cell kinds (8)", ln + 1, f.len()));
        }
        let num = |s: &str| s.trim().parse::<i64>().map_err(|_| format!("line {}: {s} is not a number", ln + 1));
        if num(f[3])? < 0 || !matches!(f[6].trim(), "Lut" | "Ff") {
            continue;
        }
        let e = by.entry(f[0]).or_default();
        // One entry per sink cell, however many of its pins the net enters.
        if !e.0.iter().any(|s| s.name == f[1]) {
            e.0.push(Sink { dx: num(f[4])?, dy: num(f[5])?, name: f[1].to_string() });
            e.1 += num(f[2])?;
        }
    }
    let mut st = PlanStats { drivers: by.len(), ..Default::default() };
    let mut cands: Vec<(&str, Vec<Sink>, i64)> = by
        .into_iter()
        .filter(|(_, (s, ps))| s.len() >= o.min_fanout && *ps as f64 / s.len() as f64 >= o.min_mean_ps)
        .map(|(d, (s, ps))| (d, s, ps))
        .collect();
    st.candidates = cands.len();
    cands.sort_by(|a, b| b.2.cmp(&a.2).then(a.0.cmp(b.0)));

    let mut text = String::new();
    for (driver, sinks, _) in cands {
        let mut groups = Vec::new();
        split(sinks, o.span, o.max_groups, &mut groups);
        if groups.len() < 2 {
            continue;
        }
        if st.copies + groups.len() - 1 > o.budget {
            break;
        }
        // The group nearest the cell as placed keeps the original.
        let off = |g: &Vec<Sink>| (g.iter().map(|s| s.dx).sum::<i64>().abs() + g.iter().map(|s| s.dy).sum::<i64>().abs()) as f64 / g.len() as f64;
        let home = (0..groups.len()).min_by(|&a, &b| off(&groups[a]).total_cmp(&off(&groups[b])).then(a.cmp(&b))).unwrap_or(0);
        let mut k = 0;
        for (i, g) in groups.iter().enumerate() {
            if i == home {
                continue;
            }
            k += 1;
            for s in g {
                text.push_str(&format!("{driver}\t{k}\t{}\n", s.name));
                st.sinks_moved += 1;
            }
        }
        st.copied += 1;
        st.copies += k;
    }
    Ok((text, st))
}

/// What `apply` did.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ApplyStats {
    pub copies: usize,
    /// Sink pins moved to a copy.
    pub pins_moved: usize,
    /// Plan drivers not in the netlist, or without exactly one driven output.
    pub drivers_skipped: usize,
    /// Plan sinks not in the netlist or not on the driver's net.
    pub sinks_skipped: usize,
}

/// The cell `name`, or the cell a packed name came from: `x/DPR0` and
/// `x_0` are parts of `x`.
fn resolve(nl: &Netlist, name: &str) -> Option<CellId> {
    nl.cell_id(name).or_else(|| name.rsplit_once('/').and_then(|(base, _)| nl.cell_id(base))).or_else(|| {
        let (base, tail) = name.rsplit_once('_')?;
        (!tail.is_empty() && tail.bytes().all(|b| b.is_ascii_digit())).then(|| nl.cell_id(base)).flatten()
    })
}

/// Make the copies of `plan` in `nl`: each a cell `<driver>$rep<k>` with
/// the driver's type, parameters and inputs, driving the sinks listed for it.
pub fn apply(nl: &mut Netlist, plan: &str) -> Result<ApplyStats, String> {
    let mut groups: BTreeMap<(String, usize), Vec<&str>> = BTreeMap::new();
    for (ln, line) in plan.lines().enumerate().filter(|(_, l)| !l.trim().is_empty() && !l.starts_with('#')) {
        let f: Vec<&str> = line.split('\t').collect();
        let [driver, k, sink] = f[..] else { return Err(format!("plan line {}: expected driver, copy, sink", ln + 1)) };
        let k: usize = k.trim().parse().map_err(|_| format!("plan line {}: {k} is not a copy number", ln + 1))?;
        groups.entry((driver.to_string(), k)).or_default().push(sink);
    }
    let mut st = ApplyStats::default();
    for ((driver, k), sinks) in groups {
        let Some(d) = nl.cell_id(&driver) else {
            st.drivers_skipped += 1;
            continue;
        };
        let cell = nl.cell(d).clone();
        let outs: Vec<_> = cell.ports.iter().filter(|(_, p)| p.dir == Dir::Output && p.net.is_some()).collect();
        let [(oport, oref)] = outs[..] else {
            st.drivers_skipped += 1;
            continue;
        };
        let Some(onet) = oref.net else { continue };
        let name = format!("{driver}$rep{k}");
        let c = nl.add_cell(&name, &cell.ty);
        nl.cell_mut(c).params = cell.params.clone();
        nl.cell_mut(c).attrs = cell.attrs.clone();
        for (pname, port) in &cell.ports {
            nl.add_port(c, pname, port.dir);
            if let (Dir::Input, Some(net)) = (port.dir, port.net) {
                nl.connect(c, pname, net);
            }
        }
        let net = nl.add_net_unique(&format!("{name}$o"));
        nl.connect(c, oport, net);
        st.copies += 1;
        for sink in sinks {
            let pins: Vec<String> = resolve(nl, sink)
                .map(|s| nl.cell(s).ports.iter().filter(|(_, p)| p.dir != Dir::Output && p.net == Some(onet)).map(|(n, _)| n.clone()).collect())
                .unwrap_or_default();
            if pins.is_empty() {
                st.sinks_skipped += 1;
                continue;
            }
            let s = resolve(nl, sink).expect("resolved above");
            for pin in pins {
                nl.connect(s, &pin, net);
                st.pins_moved += 1;
            }
        }
    }
    Ok(st)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn arcs() -> String {
        // drv: four sinks near, four 60 tiles right. near: two sinks.
        let mut t = String::new();
        for i in 0..4 {
            t.push_str(&format!("drv\tnear{i}\t2000\t{}\t{i}\t0\tLut\tFf\n", i));
            t.push_str(&format!("drv\tfar{i}\t3000\t{}\t{}\t0\tLut\tFf\n", 60 + i, 60 + i));
        }
        t.push_str("small\ta\t5000\t50\t50\t0\tLut\tFf\nsmall\tb\t5000\t50\t-50\t0\tLut\tFf\n");
        t.push_str("carry\tx\t5000\t50\t50\t0\tCarry\tFf\n");
        t
    }

    #[test]
    fn a_plan_copies_a_spread_cell_for_its_far_sinks() {
        let o = PlanOpts { min_fanout: 8, min_mean_ps: 1500.0, span: 24, max_groups: 8, budget: 10 };
        let (text, st) = plan(&arcs(), &o).unwrap();
        assert_eq!(st, PlanStats { drivers: 2, candidates: 1, copied: 1, copies: 1, sinks_moved: 4 });
        assert_eq!(text, "drv\t1\tfar0\ndrv\t1\tfar1\ndrv\t1\tfar2\ndrv\t1\tfar3\n");
        // Same arcs, same plan; no budget, no copies.
        assert_eq!(plan(&arcs(), &o).unwrap().0, text);
        assert_eq!(plan(&arcs(), &PlanOpts { budget: 0, ..o }).unwrap().1.copies, 0);
    }

    #[test]
    fn applying_a_plan_moves_only_the_listed_sinks() {
        let mut nl = Netlist::new("top");
        let (a, y) = (nl.add_net("a"), nl.add_net("y"));
        let d = nl.add_cell("drv", "LUT1");
        nl.cell_mut(d).params.insert("INIT".into(), serde_json::json!("10"));
        nl.connect(d, "I0", a);
        nl.add_port(d, "O", Dir::Output);
        nl.connect(d, "O", y);
        for s in ["s0", "s1", "s2"] {
            let c = nl.add_cell(s, "FDRE");
            nl.connect(c, "D", y);
        }
        let st = apply(&mut nl, "# plan\ndrv\t1\ts1\ndrv\t1\ts2_0\ndrv\t1\tgone\nnone\t1\ts0\n").unwrap();
        assert_eq!(st, ApplyStats { copies: 1, pins_moved: 2, drivers_skipped: 1, sinks_skipped: 1 });
        let c = nl.cell_id("drv$rep1").unwrap();
        assert_eq!(nl.cell(c).ty, "LUT1");
        assert_eq!(nl.cell(c).params, nl.cell(d).params);
        assert_eq!(nl.cell(c).net("I0"), Some(a));
        let copy_net = nl.cell(c).net("O").unwrap();
        assert_ne!(copy_net, y);
        assert_eq!(nl.net(y).users.len(), 1, "s0 stays");
        assert_eq!(nl.net(copy_net).users.len(), 2, "s1 and s2 move");
        assert_eq!(nl.net(a).users.len(), 2, "both LUTs read the input");
    }
}
