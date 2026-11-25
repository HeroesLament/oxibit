//! CrossLink-NX from the command line: `oxibit build` for `LIFCL-*` parts
//! (Verilog to a bitstream, all in oxibit-nexus).

use std::path::PathBuf;
use std::process::ExitCode;

use oxibit_nexus::graph::Graph;
use oxibit_nexus::{Chip, DeviceDb, ParsedFasm, flow, serialise};

use crate::ecp5_cmd::{done, start};
use crate::input::Result;
use crate::report::ReportArgs;

/// The CrossLink-NX EVN board's pins (LIFCL-40-9BG400C), used when no
/// `--constraints` is given.
pub const EVN_PDC: &str = include_str!("../../oxibit-nexus/boards/evn.pdc");

/// The prjoxide database: `$OXIBIT_PRJOXIDE_DB`, else `~/src/prjoxide/database`.
fn prjoxide_db() -> PathBuf {
    if let Some(p) = std::env::var_os("OXIBIT_PRJOXIDE_DB") {
        return PathBuf::from(p);
    }
    let home = std::env::var_os("HOME").map(PathBuf::from).unwrap_or_default();
    home.join("src/prjoxide/database")
}

pub struct BuildArgs {
    pub input: PathBuf,
    pub pdc: Option<PathBuf>,
    pub device: String,
    pub package: Option<String>,
    pub out: Option<PathBuf>,
    pub load: bool,
}

/// OpenOCD setup for the EVN's onboard FT2232H (channel A is JTAG). The
/// idcode is the LIFCL-40; `-ignore-version` admits the ES silicon too.
const EVN_OCD: &str = "\
adapter driver ftdi
adapter speed 6000
ftdi vid_pid 0x0403 0x6010
ftdi channel 0
ftdi layout_init 0x0008 0x008b
transport select jtag
jtag newtap nx tap -irlen 8 -expected-id 0x110f1043 -ignore-version
";

/// The SRAM load as SVF, the sequence openFPGALoader uses (verified on the
/// EVN). OpenOCD's `lattice` driver does not know the CrossLink-NX idcode and
/// its Certus path stops on a status bit this part always raises in ISC mode,
/// so the instructions are played directly: PRELOAD 1C, ISC_ENABLE C6,
/// REFRESH 79, DEVICE_CTRL 7D (08 then 00), ISC_ERASE 0E, INIT_ADDRESS 46,
/// BITSTREAM_BURST 7A, READ_STATUS 3C, ISC_DISABLE 26, BYPASS FF. Waits are
/// clocks, not time (the engine only advances on TCK). The whole file goes
/// out, header included (starting at the FFFFBDB3 preamble aborts the load),
/// first byte first and MSB first; an SVF vector shifts its least
/// significant bit first, so the hex is the bytes reversed, each
/// bit-reversed. The status check wants DONE set, BUSY/FAIL/BSE error clear.
fn sram_svf(bit: &[u8]) -> String {
    let mut s = String::with_capacity(bit.len() * 2 + 1024);
    s.push_str("TRST OFF;\nENDIR IDLE;\nENDDR IDLE;\nSTATE RESET IDLE;\n");
    s.push_str("SIR 8 TDI (1C);\nSDR 362 TDI (3");
    s.push_str(&"F".repeat(90));
    s.push_str(
        ");\n\
         SIR 8 TDI (C6);\nSDR 8 TDI (00);\nRUNTEST 200000 TCK;\n\
         SIR 8 TDI (79);\nRUNTEST 2 TCK;\n\
         SIR 8 TDI (7D);\nSDR 8 TDI (08);\nRUNTEST 200000 TCK;\n\
         SIR 8 TDI (7D);\nSDR 8 TDI (00);\nRUNTEST 200000 TCK;\n\
         SIR 8 TDI (0E);\nSDR 8 TDI (00);\nRUNTEST 100 TCK;\nRUNTEST 1000000 TCK;\n\
         SIR 8 TDI (46);\nRUNTEST 200000 TCK;\n\
         SIR 8 TDI (7A);\nRUNTEST 2 TCK;\n",
    );
    s.push_str(&format!("SDR {} TDI (", bit.len() * 8));
    for b in bit.iter().rev() {
        s.push_str(&format!("{:02X}", b.reverse_bits()));
    }
    s.push_str(
        ");\nRUNTEST 200000 TCK;\n\
         SIR 8 TDI (3C);\nRUNTEST 200000 TCK;\n\
         SDR 64 TDI (0000000000000000) TDO (0000000000000100) MASK (000000000F003100);\n\
         SIR 8 TDI (FF);\nRUNTEST 2 TCK;\n\
         SIR 8 TDI (26);\nRUNTEST 2 TCK;\n\
         SIR 8 TDI (FF);\nRUNTEST 100 TCK;\nRUNTEST 1000000 TCK;\n",
    );
    s
}

/// Program a bitstream into the EVN's SRAM (gone at power-off) with OpenOCD:
/// `$OPENOCD`, else `~/oss-cad-suite/bin/openocd`.
pub fn prog(bit: &std::path::Path, rep: &ReportArgs) -> Result<ExitCode> {
    use oxibit_report::{Event, Progress};
    let r = rep.reporter(true)?;
    start(&r, "prog");
    if !bit.is_file() {
        return Err(format!("{}: no such bitstream", bit.display()));
    }
    let exe = std::env::var_os("OPENOCD").map(PathBuf::from).unwrap_or_else(|| {
        let home = std::env::var_os("HOME").map(PathBuf::from).unwrap_or_default();
        home.join("oss-cad-suite/bin/openocd")
    });
    let cfg = std::env::temp_dir().join("oxibit-evn-ocd.cfg");
    std::fs::write(&cfg, EVN_OCD).map_err(|e| format!("{}: {e}", cfg.display()))?;
    let abs = std::fs::canonicalize(bit).map_err(|e| format!("{}: {e}", bit.display()))?;
    let kb = std::fs::metadata(&abs).map(|m| m.len() / 1024).unwrap_or(0);
    let svf = std::env::temp_dir().join("oxibit-evn-load.svf");
    let raw = std::fs::read(&abs).map_err(|e| format!("{}: {e}", abs.display()))?;
    std::fs::write(&svf, sram_svf(&raw)).map_err(|e| format!("{}: {e}", svf.display()))?;
    let mut child = std::process::Command::new(&exe)
        .arg("-f")
        .arg(&cfg)
        .args(["-c", "init", "-c"])
        .arg(format!("svf -quiet {{{}}}", svf.display()))
        .args(["-c", "shutdown"])
        .stdin(std::process::Stdio::null())
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::piped())
        .spawn()
        .map_err(|e| format!("{}: {e}", exe.display()))?;
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
    let ok = status.success() && !log.contains("Error:") && log.contains("programmed successfully");
    done(&r, ok, serde_json::json!({"kb": kb}), vec![]);
    if ok { Ok(ExitCode::SUCCESS) } else { Err(format!("openocd did not load {}:\n{log}", bit.display())) }
}

/// Verilog in, bitstream out (`<input stem>.bit` in the current directory
/// unless `--out` names it); `--load` then programs it with [`prog`].
pub fn build(a: BuildArgs, rep: &ReportArgs) -> Result<ExitCode> {
    let r = rep.reporter(true)?;
    start(&r, "build");
    let stem = a.input.file_stem().and_then(|s| s.to_str()).ok_or("input has no file name")?.to_owned();
    let out = a.out.clone().unwrap_or_else(|| PathBuf::from(format!("{stem}.bit")));
    let src = std::fs::read_to_string(&a.input).map_err(|e| format!("{}: {e}", a.input.display()))?;
    let pdc = match &a.pdc {
        Some(p) => std::fs::read_to_string(p).map_err(|e| format!("{}: {e}", p.display()))?,
        None => EVN_PDC.to_owned(),
    };
    let device = a.device;
    let package = a.package.unwrap_or_else(|| "CABGA400".to_owned());

    let db = prjoxide_db();
    let t = std::time::Instant::now();
    // Deep designs recurse in elaboration and lowering; give them room.
    let work = {
        let out = out.clone();
        let r = r.clone();
        move || -> Result<serde_json::Value> {
            let pins = flow::parse_pdc(&pdc).map_err(|e| e.to_string())?;
            r.begin("device");
            let ddb = DeviceDb::open(&db, &device).map_err(|e| e.to_string())?;
            let chip = Chip::new(&ddb, "").map_err(|e| e.to_string())?;
            let g = Graph::build(&chip).map_err(|e| e.to_string())?;
            r.end("device");
            r.begin("flow");
            let fasm = flow::run(&chip, &g, &src, &package, &pins).map_err(|e| e.to_string())?;
            r.end("flow");
            r.begin("write");
            let parsed = ParsedFasm::parse(&fasm).map_err(|e| e.to_string())?;
            let packed = Chip::from_fasm(&ddb, &parsed).map_err(|e| e.to_string())?;
            let bits = serialise(&packed).map_err(|e| e.to_string())?;
            std::fs::write(&out, &bits).map_err(|e| format!("{}: {e}", out.display()))?;
            r.end("write");
            Ok(serde_json::json!({ "bytes": bits.len() }))
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
        eprintln!("{stem}: {} byte bitstream, {:.1} s", stats["bytes"], t.elapsed().as_secs_f64());
    }
    done(&r, true, stats, vec![out.display().to_string()]);
    if a.load { prog(&out, rep) } else { Ok(ExitCode::SUCCESS) }
}
