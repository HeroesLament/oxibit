//! Target-side commands: run on the Zynq, as root.

use std::path::Path;
use std::process::ExitCode;

use oxibit_guard::{Aperture, Error, Guard, LoadOptions, RunDir, Status, observe};
use oxibit_zynq7::{Config, Zynq7};
use serde_json::json;

use crate::input::{self, Result};

pub fn backend(base: u64, len: usize) -> Result<Zynq7> {
    Zynq7::open(Config::discover(base, len)).map_err(|e| format!("Zynq-7000 registers: {e}"))
}

pub fn guard(base: u64, len: usize) -> Result<Guard<Zynq7>> {
    Guard::open(backend(base, len)?, RunDir::from_env()).map_err(|e| e.to_string())
}

pub fn status(base: u64, len: usize, json: bool) -> Result<ExitCode> {
    let b = backend(base, len)?;
    let s = observe(&b, &RunDir::from_env()).map_err(|e| e.to_string())?;
    if json {
        println!("{}", serde_json::to_string(&s).map_err(|e| e.to_string())?);
    } else {
        print_status(&s);
    }
    Ok(ExitCode::SUCCESS)
}

pub fn load(file: &Path, expect: &[(usize, u32)], aperture: bool, base: u64, len: usize, json: bool) -> Result<ExitCode> {
    let found = input::load_one(file)?;
    let mut g = guard(base, len)?;
    let report = g.load(&found.bitstream, &found.label, LoadOptions { aperture, hash: true }).map_err(|e| e.to_string())?;

    // Post-load identity checks: design-specific words the caller knows.
    let mut checks = Vec::new();
    let mut all_ok = true;
    for &(offset, want) in expect {
        let got = g.access().and_then(|a| a.read32(offset).map_err(Error::from));
        let ok = matches!(got, Ok(v) if v == want);
        all_ok &= ok;
        checks.push(json!({
            "offset": format!("0x{offset:X}"),
            "want": format!("0x{want:08X}"),
            "got": match &got {
                Ok(v) => format!("0x{v:08X}"),
                Err(e) => format!("error: {e}"),
            },
            "ok": ok,
        }));
    }

    if json {
        let mut v = serde_json::to_value(&report).map_err(|e| e.to_string())?;
        v["expect"] = checks.into();
        println!("{v}");
    } else {
        let m = &report.loaded;
        println!("loaded    {}", m.source);
        println!("die       {}", m.die.as_deref().unwrap_or("?"));
        if let Some(d) = &m.design {
            println!("design    {d}");
        }
        println!("config    {}", m.config_sha256.as_deref().unwrap_or("(not hashed)"));
        println!(
            "took      {} ms (erase {} ms, stage {} ms, configure {} ms), {} bytes",
            report.program.ms,
            report.program.detail["erase_ms"],
            report.program.detail["staged_ms"],
            report.program.detail["configure_ms"],
            m.bytes
        );
        if !m.aperture {
            println!("aperture  none declared: access refused until the next load");
        }
        println!("ready     {}", if report.hardware.ready() { "yes" } else { "NO" });
        for c in &checks {
            println!(
                "expect    {} = {}  got {}  {}",
                c["offset"].as_str().unwrap_or("?"),
                c["want"].as_str().unwrap_or("?"),
                c["got"].as_str().unwrap_or("?"),
                if c["ok"] == true { "ok" } else { "MISMATCH" }
            );
        }
    }
    Ok(if all_ok { ExitCode::SUCCESS } else { ExitCode::from(1) })
}

pub fn clear(base: u64, len: usize, json: bool) -> Result<ExitCode> {
    let mut g = guard(base, len)?;
    let s = g.clear().map_err(|e| e.to_string())?;
    if json {
        println!("{}", serde_json::to_string(&s).map_err(|e| e.to_string())?);
    } else {
        print_status(&s);
    }
    Ok(ExitCode::SUCCESS)
}

pub fn peek(offset: usize, count: usize, repeat: bool, base: u64, len: usize) -> Result<ExitCode> {
    let g = guard(base, len)?;
    let a = g.access().map_err(|e| e.to_string())?;
    let words = if repeat { a.read_repeat(offset, count) } else { a.read_block(offset, count) }
        .map_err(|e| e.to_string())?;
    for (i, w) in words.iter().enumerate() {
        let at = if repeat { offset } else { offset + i * 4 };
        println!("0x{at:05X}  0x{w:08X}");
    }
    Ok(ExitCode::SUCCESS)
}

pub fn poke(offset: usize, values: &[u32], base: u64, len: usize) -> Result<ExitCode> {
    let g = guard(base, len)?;
    let a = g.access().map_err(|e| e.to_string())?;
    a.write_block(offset, values).map_err(|e| e.to_string())?;
    Ok(ExitCode::SUCCESS)
}

fn print_status(s: &Status) {
    let d = &s.hardware.detail;
    let text = |k: &str| d[k].as_str().unwrap_or("?").to_string();

    match (s.device_idcode, &s.device_die) {
        (Some(id), Some(die)) => println!("device      {die} (IDCODE 0x{id:08X})"),
        (Some(id), None) => println!("device      IDCODE 0x{id:08X} (unknown die)"),
        _ => println!("device      ?"),
    }
    if s.ready {
        println!("ready       yes");
    } else {
        println!("ready       no: {}", s.reason);
    }
    println!(
        "config      {}  FST_CFG_B {}  GTS_CFG_B {}  STATUS {}  (INT_STS {}, PCFG_DONE latch {})",
        if s.hardware.done { "configured" } else { "blank     " },
        d["fst_cfg_b"].as_bool().map(yes_no).unwrap_or("?"),
        d["gts_cfg_b"].as_bool().map(yes_no).unwrap_or("?"),
        text("status"),
        text("int_sts"),
        d["pcfg_done_latched"].as_bool().map(yes_no).unwrap_or("?"),
    );
    println!(
        "boundary    {}  LVL_SHFTR_EN {}  FPGA_RST_CTRL {}",
        if s.hardware.boundary_open { "open  " } else { "CLOSED" },
        text("lvl_shftr_en"),
        text("fpga_rst_ctrl")
    );
    if let Some(fclks) = d["fclk"].as_array() {
        let parts: Vec<String> = fclks
            .iter()
            .map(|f| match f["mhz"].as_f64() {
                Some(mhz) => format!("{}: {mhz} MHz", f["n"]),
                None => format!("{}: ?", f["n"]),
            })
            .collect();
        println!("fclk        {}", parts.join("  "));
    }
    println!("dma buffer  {}", d["dma_buffer"].as_str().unwrap_or("NONE (load unavailable)"));
    println!("aperture    {}", text("aperture"));
    match &s.loaded {
        Some(m) => {
            println!("loaded      {}", m.source);
            println!("            config {}  ({} ms, at uptime {} s)", m.config_sha256.as_deref().unwrap_or("(not hashed)"), m.load_ms, m.loaded_at_uptime_s);
            if !m.aperture {
                println!("            no slave on the aperture: access refused");
            }
        }
        None => println!("loaded      -"),
    }
    match s.lock_holder {
        Some(pid) => println!("lock        held by pid {pid}"),
        None => println!("lock        free"),
    }
    for n in &s.notes {
        println!("note        {n}");
    }
}

fn yes_no(b: bool) -> &'static str {
    if b { "yes" } else { "no" }
}
