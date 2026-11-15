//! `oxibit timing abs`: model-free path delays of two units and their
//! difference, from timing-observatory sweeps (oxibit-timing).
//! `oxibit timing taps`: MMCM phase-tap spacing from launch-tap sweeps.
//!
//! Text output, one line per design with `--verbose`, then the summary:
//!
//! ```text
//! t3h   ref  365.0/ 381.0   p0  448.0/ 467.0 (-19.0)  ...
//! dr-c - dr-b: n=55 mean -19.8 median -18.2 sd 18.4 min -68.5 max +27.5 ps
//! ```

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;
use std::process::ExitCode;

use oxibit_timing::absolute::{self, Edges};
use oxibit_timing::edge::{self, NO_TAPS};
use oxibit_timing::calib::{Calibration, Taps};
use oxibit_timing::{sweep, taps as tapfit};

use crate::input::Result;

pub struct AbsArgs<'a> {
    pub dir: &'a Path,
    pub a: &'a str,
    pub b: &'a str,
    pub tags: &'a [String],
    pub split: &'a [String],
    pub vote: &'a [String],
    /// UNIT=FILE calibration files.
    pub calib: &'a [String],
    pub verbose: bool,
    pub json: bool,
}

pub fn abs(args: AbsArgs) -> Result<ExitCode> {
    let units: BTreeSet<&str> = args.vote.iter().map(String::as_str).chain([args.a, args.b]).collect();
    let mut taps: BTreeMap<String, [f64; 8]> = BTreeMap::new();
    for spec in args.calib {
        let (unit, file) = spec.split_once('=').ok_or_else(|| format!("--calib {spec:?}: want UNIT=FILE"))?;
        // Calibrations of units not in this comparison are ignored, so one
        // set of --calib flags serves every pair.
        if !units.contains(unit) {
            continue;
        }
        taps.insert(unit.to_string(), Calibration::load(Path::new(file))?.capture_taps());
    }
    let mut deltas = Vec::new();
    let mut designs = Vec::new();
    for tag in args.tags {
        let split = args.split.contains(tag);
        let mut all: BTreeMap<String, Edges> = BTreeMap::new();
        for &unit in &units {
            let pts = sweep::load(args.dir, tag, unit, split)?;
            if pts.is_empty() {
                return Err(format!("no sweeps of {tag} on {unit} in {}", args.dir.display()));
            }
            let t = taps.get(unit).unwrap_or(&NO_TAPS);
            all.insert(unit.to_string(), absolute::edges(&edge::windows(&pts, t)));
        }
        let (da, db) = (absolute::absolute(&all, args.a)?, absolute::absolute(&all, args.b)?);
        let d: Vec<f64> = da.iter().zip(&db).map(|(x, y)| x - y).collect();
        deltas.extend_from_slice(&d);
        if args.verbose && !args.json {
            let slots: Vec<String> =
                (0..4).map(|i| format!("p{i} {:6.1}/{:6.1} ({:+5.1})", da[i + 1], db[i + 1], d[i + 1])).collect();
            println!("{tag:5} ref {:6.1}/{:6.1}   {}", da[0], db[0], slots.join("  "));
        }
        designs.push(serde_json::json!({"tag": tag, "a": da, "b": db, "delta": d}));
    }
    let s = absolute::stats(&deltas).ok_or("no designs")?;
    if args.json {
        let out = serde_json::json!({"a": args.a, "b": args.b, "designs": designs, "stats": s});
        println!("{out}");
    } else {
        println!(
            "{} - {}: n={} mean {:+.1} median {:+.1} sd {:.1} min {:+.1} max {:+.1} ps",
            args.a, args.b, s.n, s.mean, s.median, s.sd, s.min, s.max
        );
    }
    Ok(ExitCode::SUCCESS)
}

pub fn taps(dir: &Path, units: &[String], tags: &[String], split: &[String], iters: usize, write: Option<&Path>, json: bool) -> Result<ExitCode> {
    if write.is_some() && units.len() != 1 {
        return Err("--write: a calibration file is per chip; give exactly one unit".into());
    }
    let mut edges = Vec::new();
    for unit in units {
        for tag in tags {
            let pts = tapfit::load(dir, tag, unit, split.contains(tag))?;
            if pts.is_empty() {
                return Err(format!("no tap sweeps of {tag} on {unit} in {}", dir.display()));
            }
            edges.extend(tapfit::edges(&pts, &format!("{unit} {tag}")));
        }
    }
    if edges.is_empty() {
        return Err("no usable edges".into());
    }
    let r = tapfit::fit(&edges, iters);
    if let Some(path) = write {
        let mut c = Calibration::load_or_new(path, &units[0])?;
        let mut sg = r.sigmas.clone();
        sg.sort_by(f64::total_cmp);
        c.taps = Some(Taps {
            capture: r.a1,
            launch: r.a0,
            designs: tags.to_vec(),
            edges: r.edges,
            points: r.points,
            sse: r.sse,
            sigma_median: sg[sg.len() / 2],
        });
        c.save(path)?;
    }
    let mean0 = r.a0.iter().sum::<f64>() / 8.0;
    let a0c = r.a0.map(|x| x - mean0);
    let corr = tapfit::corr(&r.a1, &a0c);
    if json {
        println!("{}", serde_json::json!({"units": units, "tags": tags, "fit": r, "a0_centred": a0c, "corr": corr}));
        return Ok(ExitCode::SUCCESS);
    }
    println!("edges {} points {}", r.edges, r.points);
    println!("SSE {:.4} -> {:.4}", r.sse_start, r.sse);
    println!("tap   a1 CLKOUT1   a0 CLKOUT0(centred)   ps@125: a1   a0");
    for j in 0..8 {
        println!("{j}     {:+.4}      {:+.4}              {:+6.1} {:+6.1}", r.a1[j], a0c[j], r.a1[j] * 125.0, a0c[j] * 125.0);
    }
    println!("corr(a1, a0) {corr:.3}");
    let mut sg = r.sigmas.clone();
    sg.sort_by(f64::total_cmp);
    let n = sg.len();
    println!("edge sigma p10/med/p90 {} {} {}", sg[n / 10], sg[n / 2], sg[9 * n / 10]);
    Ok(ExitCode::SUCCESS)
}
