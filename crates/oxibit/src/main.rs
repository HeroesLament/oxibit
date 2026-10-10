//! `oxibit`: Xilinx bitstream inspection, conversion and verification, and on
//! a Zynq-7000 target, loading and guarded PL access.

mod build;
mod route_cmd;
mod input;
#[cfg(target_os = "linux")]
mod device;
#[cfg(target_os = "linux")]
mod port;

use input::{Found, load};

use std::path::{Path, PathBuf};
use std::process::ExitCode;

use clap::{Parser, Subcommand, ValueEnum};
use oxibit_bitstream::packet::{self, Item, Reg, cmd};
use oxibit_bitstream::zynq_boot::{BootImage, Destination};
use oxibit_bitstream::{Bitstream, Report, analyze, part};

#[derive(Parser)]
#[command(name = "oxibit", version, about = "Xilinx bitstream inspection, conversion and verification")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Describe bitstreams: container, header, device, frames, commands, CRC.
    Inspect {
        files: Vec<PathBuf>,
        /// Also list every packet.
        #[arg(long)]
        packets: bool,
        /// One JSON report per line.
        #[arg(long)]
        json: bool,
    },
    /// Exit non-zero if any bitstream has a problem, or targets another die.
    Verify {
        files: Vec<PathBuf>,
        /// Die or part the bitstreams must target, e.g. xc7z020 or xc7z020clg400-1.
        #[arg(long)]
        part: Option<String>,
        #[arg(long)]
        json: bool,
    },
    /// Rewrite a bitstream in another container.
    Convert {
        input: PathBuf,
        output: PathBuf,
        #[arg(long, value_enum, default_value_t = Target::SwappedBin)]
        to: Target,
        /// Part for record b when writing a .bit from a header-less input.
        #[arg(long)]
        part: Option<String>,
        /// Design name for record a when writing a .bit from a header-less input.
        #[arg(long)]
        design: Option<String>,
    },
    /// Assemble FASM into a full-configuration .bit against a prjxray database
    /// (replaces prjxray's fasm2frames + xc7frames2bit; same configuration words).
    Fasm2bit {
        input: PathBuf,
        output: PathBuf,
        /// prjxray database family directory, e.g. ~/src/openxc7/prjxray-db/zynq7.
        #[arg(long)]
        db_root: PathBuf,
        /// Part, e.g. xc7z020clg400-1.
        #[arg(long)]
        part: String,
        /// Also write fasm2frames' .frm text here.
        #[arg(long)]
        frames: Option<PathBuf>,
    },
    /// Route a test net through chosen pips and print it as FASM (test harnesses).
    Route {
        #[arg(long)]
        db_root: PathBuf,
        #[arg(long)]
        part: String,
        /// Grid window x0,y0,x1,y1 to build the graph over (default: whole device).
        #[arg(long)]
        window: Option<String>,
        /// Source TILE/WIRE.
        #[arg(long)]
        from: Option<String>,
        /// Sink TILE/WIRE.
        #[arg(long)]
        to: Option<String>,
        /// Pip the route must take, TILE.DST.SRC; repeatable, in order.
        #[arg(long)]
        via: Vec<String>,
        /// FASM whose routing must be avoided; repeatable.
        #[arg(long)]
        avoid: Vec<PathBuf>,
        /// Report which pips a FASM file emits that the graph does not have as usable edges.
        #[arg(long)]
        check: Vec<PathBuf>,
    },
    /// Erlang port ({:packet, 4}): routes test chains through a base design with
    /// chosen nets stripped, and returns the .bit (test campaigns).
    RoutePort {
        #[arg(long)]
        db_root: PathBuf,
        #[arg(long)]
        part: String,
        /// Grid window x0,y0,x1,y1 to build the graph over (default: whole device).
        #[arg(long)]
        window: Option<String>,
        /// Base design FASM.
        #[arg(long)]
        base: PathBuf,
        /// TILE/WIRE source of a base net to strip; repeatable.
        #[arg(long)]
        strip: Vec<String>,
        /// TILE/WIRE root of the base clock net (BUFG output); enables FF relays.
        #[arg(long)]
        clock: Option<String>,
    },
    /// Erlang port ({:packet, 4}): FASM in, .bit out; the database is read once.
    BuildPort {
        #[arg(long)]
        db_root: PathBuf,
        #[arg(long)]
        part: String,
    },

    /// [target] Configuration, boundary, clocks and what is loaded. Never touches the PL.
    #[cfg(target_os = "linux")]
    Status {
        #[command(flatten)]
        aperture: ApertureArgs,
        #[arg(long)]
        json: bool,
    },
    /// [target] Load a bitstream (any container, or a BOOT.BIN's PL partition) into the PL.
    #[cfg(target_os = "linux")]
    Load {
        file: PathBuf,
        /// After loading, require the word at OFFSET to read VALUE, e.g. 0x0=0x45484431. Repeatable.
        #[arg(long = "expect", value_parser = parse_expect)]
        expect: Vec<(usize, u32)>,
        /// The design has no slave on the aperture (e.g. an EMIO/DDR-only
        /// design). Every aperture access is refused until the next load.
        #[arg(long, conflicts_with = "expect")]
        no_aperture: bool,
        #[command(flatten)]
        aperture: ApertureArgs,
        #[arg(long)]
        json: bool,
    },
    /// [target] Erase the PL and leave it unconfigured.
    #[cfg(target_os = "linux")]
    Clear {
        #[command(flatten)]
        aperture: ApertureArgs,
        #[arg(long)]
        json: bool,
    },
    /// [target] Read words from the PL aperture (refused unless the PL is ready).
    #[cfg(target_os = "linux")]
    Peek {
        #[arg(value_parser = parse_usize)]
        offset: usize,
        #[arg(default_value_t = 1)]
        count: usize,
        /// Read the same word COUNT times (FIFO / auto-advancing port).
        #[arg(long)]
        repeat: bool,
        #[command(flatten)]
        aperture: ApertureArgs,
    },
    /// [target] Write words to consecutive offsets in the PL aperture (refused unless ready).
    #[cfg(target_os = "linux")]
    Poke {
        #[arg(value_parser = parse_usize)]
        offset: usize,
        #[arg(value_parser = parse_u32, required = true)]
        values: Vec<u32>,
        #[command(flatten)]
        aperture: ApertureArgs,
    },
    /// [target] Serve the Erlang port protocol on stdin/stdout ({:packet, 4}).
    #[cfg(target_os = "linux")]
    Port {
        #[command(flatten)]
        aperture: ApertureArgs,
    },
    /// [target] Port protocol ops 0x01-0x06 over PS-side memory (DDR, SLCR,
    /// DevC, PS peripherals). Refuses ranges inside M_AXI_GP0/GP1.
    #[cfg(target_os = "linux")]
    Mem {
        #[arg(value_parser = parse_u64)]
        base: u64,
        #[arg(value_parser = parse_usize)]
        len: usize,
    },
}

/// The PL register window.
#[derive(clap::Args, Clone, Copy)]
#[allow(dead_code)]
struct ApertureArgs {
    /// Physical base of the PL aperture (M_AXI_GP0 is 0x40000000).
    #[arg(long, default_value = "0x40000000", value_parser = parse_u64)]
    base: u64,
    /// Length of the aperture in bytes.
    #[arg(long, default_value = "0x20000", value_parser = parse_usize)]
    len: usize,
}

fn parse_u64(s: &str) -> std::result::Result<u64, String> {
    let s = s.replace('_', "");
    match s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        Some(hex) => u64::from_str_radix(hex, 16),
        None => s.parse(),
    }
    .map_err(|e| format!("{s:?}: {e}"))
}

fn parse_usize(s: &str) -> std::result::Result<usize, String> {
    parse_u64(s).and_then(|v| usize::try_from(v).map_err(|e| e.to_string()))
}

fn parse_u32(s: &str) -> std::result::Result<u32, String> {
    parse_u64(s).and_then(|v| u32::try_from(v).map_err(|e| e.to_string()))
}

#[allow(dead_code)]
fn parse_expect(s: &str) -> std::result::Result<(usize, u32), String> {
    let (off, val) = s.split_once('=').ok_or_else(|| format!("{s:?}: expected OFFSET=VALUE"))?;
    Ok((parse_usize(off)?, parse_u32(val)?))
}

#[derive(Clone, Copy, ValueEnum)]
enum Target {
    /// Byte-swapped words, no header: Zynq-7000 PCAP / Linux fpga_manager.
    SwappedBin,
    /// Big-endian words, no header.
    RawBin,
    /// .bit with TLV header.
    Bit,
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    let result = match cli.command {
        Command::Inspect { files, packets, json } => inspect(&files, packets, json),
        Command::Verify { files, part, json } => verify(&files, part.as_deref(), json),
        Command::Convert { input, output, to, part, design } => {
            convert(&input, &output, to, part, design)
        }
        Command::Fasm2bit { input, output, db_root, part, frames } => {
            build::fasm2bit(&input, &output, &db_root, &part, frames.as_deref())
        }
        Command::BuildPort { db_root, part } => return build::serve(&db_root, &part),
        Command::RoutePort { db_root, part, window, base, strip, clock } => {
            return route_cmd::serve(route_cmd::PortArgs {
                db_root: &db_root,
                part: &part,
                window: window.as_deref(),
                base: &base,
                strip: &strip,
                clock: clock.as_deref(),
            });
        }
        Command::Route { db_root, part, window, from, to, via, avoid, check } => route_cmd::run(route_cmd::Args {
            db_root: &db_root,
            part: &part,
            window: window.as_deref(),
            from: from.as_deref(),
            to: to.as_deref(),
            via: &via,
            avoid: &avoid,
            check: &check,
        }),
        #[cfg(target_os = "linux")]
        Command::Status { aperture: a, json } => device::status(a.base, a.len, json),
        #[cfg(target_os = "linux")]
        Command::Load { file, expect, no_aperture, aperture: a, json } => {
            device::load(&file, &expect, !no_aperture, a.base, a.len, json)
        }
        #[cfg(target_os = "linux")]
        Command::Clear { aperture: a, json } => device::clear(a.base, a.len, json),
        #[cfg(target_os = "linux")]
        Command::Peek { offset, count, repeat, aperture: a } => device::peek(offset, count, repeat, a.base, a.len),
        #[cfg(target_os = "linux")]
        Command::Poke { offset, values, aperture: a } => device::poke(offset, &values, a.base, a.len),
        #[cfg(target_os = "linux")]
        Command::Port { aperture: a } => return port::serve(a.base, a.len),
        #[cfg(target_os = "linux")]
        Command::Mem { base, len } => return port::serve_mem(base, len),
    };
    match result {
        Ok(code) => code,
        Err(e) => {
            eprintln!("oxibit: {e}");
            ExitCode::from(2)
        }
    }
}

use input::Result;

fn inspect(files: &[PathBuf], packets: bool, json: bool) -> Result<ExitCode> {
    for path in files {
        let (boot, found) = load(path)?;
        if let (Some(boot), false) = (&boot, json) {
            print_boot(path, boot);
        }
        for f in found {
            let r = analyze(&f.bitstream).map_err(|e| format!("{}: {e}", f.label))?;
            if json {
                let mut v = serde_json::to_value(&r).map_err(|e| e.to_string())?;
                v["file"] = f.label.clone().into();
                println!("{v}");
            } else {
                print_report(&f.label, &r);
            }
            if packets {
                print_packets(&f.bitstream)?;
            }
        }
    }
    Ok(ExitCode::SUCCESS)
}

fn verify(files: &[PathBuf], want: Option<&str>, json: bool) -> Result<ExitCode> {
    let want_die = match want {
        Some(p) => Some(part::by_part(p).ok_or_else(|| format!("unknown part {p:?}"))?),
        None => None,
    };

    // (label, die, problems) for every bitstream, or for a file that failed to load.
    let mut results: Vec<(String, Option<&str>, Vec<String>)> = Vec::new();
    for path in files {
        match load(path) {
            Err(e) => results.push((path.display().to_string(), None, vec![e])),
            Ok((_, found)) if found.is_empty() => results.push((
                path.display().to_string(),
                None,
                vec!["boot image has no PL partition".into()],
            )),
            Ok((_, found)) => {
                for f in found {
                    let mut problems = Vec::new();
                    let mut die = None;
                    match analyze(&f.bitstream) {
                        Err(e) => problems.push(e.to_string()),
                        Ok(r) => {
                            die = r.die.map(|d| d.name);
                            problems.extend(r.problems);
                            if let Some(w) = want_die {
                                match r.die {
                                    Some(d) if d.name == w.name => {}
                                    Some(d) => problems.push(format!("targets {}, not {}", d.name, w.name)),
                                    None => problems.push(format!("cannot confirm it targets {}", w.name)),
                                }
                            }
                        }
                    }
                    results.push((f.label, die, problems));
                }
            }
        }
    }

    let failed = results.iter().filter(|(_, _, p)| !p.is_empty()).count();
    for (label, die, problems) in &results {
        if json {
            let v = serde_json::json!({
                "file": label,
                "ok": problems.is_empty(),
                "die": die,
                "problems": problems,
            });
            println!("{v}");
        } else if problems.is_empty() {
            println!("ok    {label}  {}", die.unwrap_or("?"));
        } else {
            println!("FAIL  {label}");
            for p in problems {
                println!("      {p}");
            }
        }
    }

    if !json {
        eprintln!("{} checked, {} failed", results.len(), failed);
    }
    Ok(if failed == 0 { ExitCode::SUCCESS } else { ExitCode::from(1) })
}

fn convert(
    input: &Path,
    output: &Path,
    to: Target,
    part: Option<String>,
    design: Option<String>,
) -> Result<ExitCode> {
    let (_, mut found) = load(input)?;
    if found.len() != 1 {
        return Err(format!("{}: expected one bitstream, found {}", input.display(), found.len()));
    }
    let Found { label, bitstream: bs } = found.remove(0);

    let bytes = match to {
        Target::SwappedBin => bs.to_swapped_bin(),
        Target::RawBin => bs.to_raw_bin(),
        Target::Bit => {
            let mut header = bs.header.clone().unwrap_or_default();
            if part.is_some() {
                header.part = part;
            }
            if design.is_some() {
                header.design = design;
            }
            if header.design.is_none() {
                header.design = Some(format!("{label};Generator=oxibit"));
            }
            bs.to_bit(&header).map_err(|e| e.to_string())?
        }
    };
    std::fs::write(output, &bytes).map_err(|e| format!("{}: {e}", output.display()))?;
    Ok(ExitCode::SUCCESS)
}

fn print_boot(path: &Path, boot: &BootImage) {
    println!("{}", path.display());
    println!(
        "  zynq boot image, header checksum {}",
        if boot.header_checksum_ok { "ok" } else { "BAD" }
    );
    for p in &boot.partitions {
        println!(
            "  #{}  {:<4} {:<20} offset 0x{:08X}  {:>9} bytes  load 0x{:08X}{}{}",
            p.index,
            match p.destination {
                Destination::Ps => "PS".to_string(),
                Destination::Pl => "PL".to_string(),
                Destination::None => "-".to_string(),
                Destination::Other(v) => format!("?{v}"),
            },
            p.name.as_deref().unwrap_or("?"),
            p.offset,
            p.length,
            p.load_address,
            if p.encrypted { "  encrypted" } else { "" },
            if p.checksum_ok { "" } else { "  HEADER CHECKSUM BAD" },
        );
    }
}

fn print_report(label: &str, r: &Report) {
    println!("{label}");
    println!("  container      {}", serde_json::to_value(r.container).unwrap().as_str().unwrap_or("?"));
    if let Some(h) = &r.header {
        if let Some(d) = &h.design {
            println!("  design         {d}");
        }
        if let Some(p) = &h.part {
            println!("  part           {p}");
        }
        if h.date.is_some() || h.time.is_some() {
            println!(
                "  built          {} {}",
                h.date.as_deref().unwrap_or(""),
                h.time.as_deref().unwrap_or("")
            );
        }
    }
    match (r.idcode, r.die) {
        (Some(id), Some(d)) => println!("  idcode         0x{id:08X}  {} {}", d.family, d.name),
        (Some(id), None) => println!("  idcode         0x{id:08X}  (unknown die)"),
        (None, _) => println!("  idcode         none"),
    }
    println!("  words          {}", r.words);
    println!(
        "  frame data     {} words = {} frames{}",
        r.fdri_words,
        r.fdri_words / oxibit_bitstream::FRAME_WORDS,
        match r.fdri_words % oxibit_bitstream::FRAME_WORDS {
            0 => String::new(),
            rem => format!(" + {rem}"),
        }
    );
    println!("  syncs          {:?}", r.syncs);
    println!("  commands       {}", r.commands.join(" "));
    println!("  crc            {} checks, {} failed", r.crc.checks, r.crc.failures.len());
    println!("  config sha256  {}", r.config_sha256.as_deref().unwrap_or("(not computed)"));
    if r.partial_hint {
        println!("  partial        yes (header)");
    }
    for w in &r.warnings {
        println!("  warning        {w}");
    }
    if r.problems.is_empty() {
        println!("  problems       none");
    } else {
        for p in &r.problems {
            println!("  PROBLEM        {p}");
        }
    }
}

fn print_packets(bs: &Bitstream) -> Result<()> {
    let items = packet::walk(&bs.words).map_err(|e| e.to_string())?;

    // Runs of NOPs are printed as one line.
    let mut nop_run: Option<(usize, usize)> = None;
    let flush = |run: &mut Option<(usize, usize)>| {
        if let Some((start, count)) = run.take() {
            if count == 1 {
                println!("  {start:>8}  T1 NOP");
            } else {
                println!("  {start:>8}  T1 NOP x{count}");
            }
        }
    };

    for item in items {
        if let Item::Packet(p) = &item {
            if p.opcode == packet::Opcode::Nop && p.data.is_empty() {
                match &mut nop_run {
                    Some((_, count)) => *count += 1,
                    None => nop_run = Some((p.index, 1)),
                }
                continue;
            }
        }
        flush(&mut nop_run);

        match item {
            Item::Unsynced { index, words } => {
                println!("  {index:>8}  unsynced  {} words", words.len());
            }
            Item::Sync { index } => println!("  {index:>8}  SYNC"),
            Item::Packet(p) => {
                let kind = match p.kind {
                    packet::PacketType::Type1 => "T1",
                    packet::PacketType::Type2 => "T2",
                };
                let op = format!("{:?}", p.opcode).to_uppercase();
                let detail = match (p.reg, p.data) {
                    (Reg::CMD, [w]) => cmd::name(*w).map(str::to_owned).unwrap_or(format!("0x{w:08X}")),
                    (_, []) => String::new(),
                    (_, [w]) => format!("0x{w:08X}"),
                    (_, d) => format!("{} words", d.len()),
                };
                println!("  {:>8}  {kind} {op:<5} {:<8} {detail}", p.index, p.reg.to_string());
            }
        }
    }
    flush(&mut nop_run);
    Ok(())
}
