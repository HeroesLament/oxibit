//! Prove every output of an AIGER miter constant 1.
//!
//! cargo run --release -p oxibit-cec --example cec -- MITER.aig MITER.map [THREADS] [SWEEP_BUDGET] [FINAL_BUDGET]
//!
//! stdout: "proven N failed N unproven N", then "FAIL name" / "OPEN name"
//! lines (the format of the ABC flow's result.txt). Progress goes through
//! oxibit-report (docs/events.md): human on stderr (a live status line on a
//! terminal, plain lines otherwise), the JSON Lines stream to the file named
//! by OXIBIT_EVENTS, and systemd status when NOTIFY_SOCKET is set.

use std::io::{BufWriter, IsTerminal};
use std::process::ExitCode;
use std::sync::atomic::AtomicBool;

use oxibit_cec::{aiger, sweep};
use oxibit_report::{Event, Human, HumanStyle, JsonLines, Progress, Reporter, Sink};

static CANCEL: AtomicBool = AtomicBool::new(false);

fn reporter() -> Reporter {
    let tty = std::io::stderr().is_terminal() && std::env::var_os("CI").is_none() && std::env::var("TERM").map_or(true, |t| t != "dumb");
    let style = if tty { HumanStyle::Live } else { HumanStyle::Plain };
    let mut sinks: Vec<Box<dyn Sink>> = vec![Box::new(Human::new(std::io::stderr(), style))];
    if let Some(p) = std::env::var_os("OXIBIT_EVENTS") {
        match std::fs::File::create(&p) {
            Ok(f) => sinks.push(Box::new(JsonLines(BufWriter::new(f)))),
            Err(e) => eprintln!("{}: {e}", p.to_string_lossy()),
        }
    }
    #[cfg(target_os = "linux")]
    if let Some(n) = oxibit_report::SdNotify::from_env() {
        sinks.push(Box::new(n));
    }
    Reporter::new(sinks, &CANCEL)
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 3 {
        eprintln!("usage: cec MITER.aig MITER.map [THREADS] [SWEEP_BUDGET] [FINAL_BUDGET]");
        return ExitCode::from(3);
    }
    let arg = |i: usize, d: u64| args.get(i).and_then(|s| s.parse().ok()).unwrap_or(d);
    let threads = arg(3, std::thread::available_parallelism().map(|n| n.get() as u64).unwrap_or(4)) as usize;
    let r = reporter();
    r.emit(Event::Start {
        cmd: "cec".into(),
        version: env!("CARGO_PKG_VERSION").into(),
        args: args[1..].to_vec(),
    });
    r.begin("read");
    let g = match std::fs::File::open(&args[1])
        .map_err(|e| format!("{}: {e}", args[1]))
        .and_then(aiger::read)
    {
        Ok(g) => g,
        Err(e) => {
            eprintln!("{e}");
            return ExitCode::from(3);
        }
    };
    let names = aiger::output_names(&std::fs::read_to_string(&args[2]).unwrap_or_default());
    r.end("read");
    let (verdict, st) = sweep::prove(&g, true, threads, arg(4, 1000), arg(5, 10_000), &r);
    let count = |v| verdict.iter().filter(|x| **x == v).count();
    let (p, f, o) = (count(sweep::Verdict::Proven), count(sweep::Verdict::Failed), count(sweep::Verdict::Open));
    println!("proven {p} failed {f} unproven {o}");
    for (i, v) in verdict.iter().enumerate() {
        let name = names.get(&i).cloned().unwrap_or_else(|| format!("out{i}"));
        match v {
            sweep::Verdict::Failed => println!("FAIL {name}"),
            sweep::Verdict::Open => println!("OPEN {name}"),
            sweep::Verdict::Proven => {}
        }
    }
    r.emit(Event::Done {
        ok: f == 0 && o == 0,
        exit: 0,
        cancelled: r.cancelled(),
        phases: r.phases(),
        peak_rss_mb: None,
        stats: serde_json::json!({"inputs": g.inputs, "latches": g.latches, "ands": g.ands.len(), "outputs": g.outputs.len(),
            "threads": threads, "sat_calls": st.sat_calls, "merges": st.merges}),
        check: serde_json::json!({"proven": p, "failed": f, "open": o}),
        outputs: vec![],
    });
    r.finish();
    ExitCode::SUCCESS
}
