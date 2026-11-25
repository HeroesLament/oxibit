//! ECP5 from the command line: `oxibit build` (Verilog to a bitstream) and
//! `oxibit prog` (bitstream into the board's SRAM over JTAG).

use std::path::{Path, PathBuf};
use std::process::ExitCode;

use oxibit_ecp5::flow::{FlowOptions, run_with};
use oxibit_ecp5::{Chip, DeviceDb, Options, serialise};
use oxibit_report::{Event, Progress, Reporter};

use crate::input::Result;
use crate::report::ReportArgs;

fn start(r: &Reporter, cmd: &str) {
    r.emit(Event::Start { cmd: cmd.into(), version: env!("CARGO_PKG_VERSION").into(), args: std::env::args().skip(1).collect() });
}

fn done(r: &Reporter, ok: bool, stats: serde_json::Value, outputs: Vec<String>) {
    r.emit(Event::Done {
        ok,
        exit: if ok { 0 } else { 2 },
        cancelled: false,
        phases: r.phases(),
        peak_rss_mb: None,
        stats,
        check: serde_json::json!({}),
        outputs,
    });
    r.finish();
}

/// ULX3S pin preferences, used when no `--lpf` is given.
const ULX3S_LPF: &str = include_str!("../../oxibit-ecp5/designs/ulx3s.lpf");

/// OpenOCD setup for the ULX3S onboard FT231X (bit-banged JTAG). trst / srst
/// are assigned because the ft232r defaults put them on pins this map uses.
const ULX3S_OCD: &str = "\
adapter driver ft232r
adapter speed 1000
ft232r vid_pid 0x0403 0x6015
ft232r tck_num DSR
ft232r tms_num DCD
ft232r tdi_num RI
ft232r tdo_num CTS
ft232r trst_num RTS
ft232r srst_num DTR
transport select jtag
jtag newtap ecp5 tap -irlen 8 -expected-id 0x41113043
pld create ecp5.pld lattice -chain-position ecp5.tap
";

fn trellis_db() -> PathBuf {
    if let Some(p) = std::env::var_os("OXIBIT_TRELLIS_DB") {
        return PathBuf::from(p);
    }
    let home = std::env::var_os("HOME").map(PathBuf::from).unwrap_or_default();
    home.join("oss-cad-suite/share/trellis/database")
}

pub struct BuildArgs {
    pub input: PathBuf,
    pub lpf: Option<PathBuf>,
    pub device: Option<String>,
    pub package: Option<String>,
    pub top: Option<String>,
    pub out: Option<PathBuf>,
    pub load: bool,
}

/// Verilog in, compressed bitstream out (`<input stem>.bit` in the current
/// directory unless `--out` names it).
pub fn build(a: BuildArgs, rep: &ReportArgs) -> Result<ExitCode> {
    let r = rep.reporter(true)?;
    start(&r, "build");
    let stem = a.input.file_stem().and_then(|s| s.to_str()).ok_or("input has no file name")?.to_owned();
    let out = a.out.clone().unwrap_or_else(|| PathBuf::from(format!("{stem}.bit")));
    let src = std::fs::read_to_string(&a.input).map_err(|e| format!("{}: {e}", a.input.display()))?;
    let lpf = match &a.lpf {
        Some(p) => std::fs::read_to_string(p).map_err(|e| format!("{}: {e}", p.display()))?,
        None => ULX3S_LPF.to_owned(),
    };
    let mut opts = FlowOptions::default();
    if let Some(d) = a.device {
        opts.device = d;
    }
    if let Some(p) = a.package {
        opts.package = p;
    }
    opts.top = a.top;

    let db = trellis_db();
    let t = std::time::Instant::now();
    // Deep designs recurse in elaboration and lowering; give them room.
    let work = {
        let out = out.clone();
        let r = r.clone();
        move || -> Result<serde_json::Value> {
            let (cc, rep) = run_with(&db, &src, &lpf, &opts, &*r).map_err(|e| e.to_string())?;
            for w in &rep.synth.warnings {
                r.emit(Event::Warn { code: "synth".into(), msg: w.clone() });
            }
            r.begin("write");
            let ddb = DeviceDb::load(&db, &opts.device).map_err(|e| e.to_string())?;
            let chip = Chip::from_config(&ddb, &cc).map_err(|e| e.to_string())?;
            let over = Options { compress: true, ..Options::default() };
            let bits = serialise(&chip, &Options::from_sysconfig(&cc.sysconfig, &over)).map_err(|e| e.to_string())?;
            std::fs::write(&out, bits).map_err(|e| format!("{}: {e}", out.display()))?;
            r.end("write");
            Ok(serde_json::json!({
                "luts": rep.synth.luts, "ffs": rep.synth.ffs, "nets": rep.nets, "passes": rep.passes
            }))
        }
    };
    let res = std::thread::Builder::new()
        .stack_size(1 << 28)
        .spawn(work)
        .map_err(|e| e.to_string())?
        .join()
        .map_err(|_| "the flow panicked".to_owned())?;
    let stats = match res {
        Ok(s) => s,
        Err(e) => {
            done(&r, false, serde_json::json!({}), vec![]);
            return Err(e);
        }
    };
    if rep.text() {
        eprintln!(
            "{stem}: {} luts, {} ffs, {} nets routed in {} passes, {:.1} s",
            stats["luts"], stats["ffs"], stats["nets"], stats["passes"], t.elapsed().as_secs_f64()
        );
    }
    done(&r, true, stats, vec![out.display().to_string()]);
    if a.load { prog(&out, rep) } else { Ok(ExitCode::SUCCESS) }
}

/// Program a bitstream into the ULX3S SRAM (gone at power-off) with OpenOCD:
/// `$OPENOCD`, else `~/oss-cad-suite/bin/openocd`.
pub fn prog(bit: &Path, rep: &ReportArgs) -> Result<ExitCode> {
    let r = rep.reporter(true)?;
    start(&r, "prog");
    if !bit.is_file() {
        return Err(format!("{}: no such bitstream", bit.display()));
    }
    let exe = std::env::var_os("OPENOCD").map(PathBuf::from).unwrap_or_else(|| {
        let home = std::env::var_os("HOME").map(PathBuf::from).unwrap_or_default();
        home.join("oss-cad-suite/bin/openocd")
    });
    let cfg = std::env::temp_dir().join("oxibit-ulx3s-ocd.cfg");
    std::fs::write(&cfg, ULX3S_OCD).map_err(|e| format!("{}: {e}", cfg.display()))?;
    let abs = std::fs::canonicalize(bit).map_err(|e| format!("{}: {e}", bit.display()))?;
    let kb = std::fs::metadata(&abs).map(|m| m.len() / 1024).unwrap_or(0);
    let mut child = std::process::Command::new(&exe)
        .arg("-f")
        .arg(&cfg)
        .args(["-c", "init", "-c"])
        .arg(format!("pld load ecp5.pld {{{}}}", abs.display()))
        .args(["-c", "shutdown"])
        .stdin(std::process::Stdio::null())
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::piped())
        .spawn()
        .map_err(|e| format!("{}: {e}", exe.display()))?;
    // OpenOCD says nothing while it shifts the bitstream in; keep the status
    // line alive with the elapsed time until it exits.
    r.begin("load");
    let status = loop {
        match child.try_wait().map_err(|e| e.to_string())? {
            Some(s) => break s,
            None => {
                r.emit(Event::progress("load", serde_json::json!({"sram": format!("{kb} kB over JTAG")})));
                std::thread::sleep(std::time::Duration::from_millis(250));
            }
        }
    };
    r.end("load");
    let mut log = String::new();
    if let Some(mut e) = child.stderr.take() {
        use std::io::Read;
        let _ = e.read_to_string(&mut log);
    }
    let ok = status.success() && log.contains("loaded file");
    done(&r, ok, serde_json::json!({"kb": kb}), vec![]);
    if ok { Ok(ExitCode::SUCCESS) } else { Err(format!("openocd did not load {}:\n{log}", bit.display())) }
}
