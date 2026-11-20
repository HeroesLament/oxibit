//! Where a command's events go: the global `--progress`, `--events` and
//! `-q` options turned into `oxibit_report` sinks, plus the signal thread
//! for long commands.
//!
//! Commands that already print status lines (`fasm2bit`, `route`) keep
//! printing them unchanged unless `--progress json` or `none` is given; the
//! event stream is added beside them, not instead of them.

use std::fs::File;
use std::io::{BufWriter, IsTerminal};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use clap::{Args, ValueEnum};
use oxibit_report::{Human, HumanStyle, JsonLines, Reporter, Sink};

/// Set from SIGINT/SIGTERM; long commands poll it through `Progress::cancelled`.
pub static CANCEL: AtomicBool = AtomicBool::new(false);

#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum ProgressMode {
    /// A live status line on a terminal, plain lines otherwise.
    Auto,
    /// A live status line.
    Human,
    /// One line per event.
    Plain,
    /// JSON Lines on stderr instead of text.
    Json,
    /// Nothing on stderr but errors.
    None,
}

/// Global reporting options (docs/events.md).
#[derive(Debug, Clone, Args)]
pub struct ReportArgs {
    /// Progress on stderr.
    #[arg(long, global = true, value_enum, default_value_t = ProgressMode::Auto)]
    pub progress: ProgressMode,
    /// Also write the JSON Lines event stream here.
    #[arg(long, global = true)]
    pub events: Option<PathBuf>,
    /// Same as --progress none.
    #[arg(short, long, global = true)]
    pub quiet: bool,
}

impl ReportArgs {
    pub fn mode(&self) -> ProgressMode {
        if self.quiet { ProgressMode::None } else { self.progress }
    }

    /// The command's own status lines (the pre-events stderr text) stay on.
    pub fn text(&self) -> bool {
        !matches!(self.mode(), ProgressMode::Json | ProgressMode::None)
    }

    /// Sinks for this run. `human`: render events as text on stderr (for
    /// commands that have no status lines of their own).
    pub fn reporter(&self, human: bool) -> Result<Arc<Reporter>, String> {
        let mut sinks: Vec<Box<dyn Sink>> = Vec::new();
        let tty = std::io::stderr().is_terminal() && std::env::var_os("CI").is_none() && std::env::var("TERM").map_or(true, |t| t != "dumb");
        match self.mode() {
            ProgressMode::Json => sinks.push(Box::new(JsonLines(std::io::stderr()))),
            ProgressMode::None => {}
            _ if !human => {}
            ProgressMode::Auto if tty => sinks.push(Box::new(Human::new(std::io::stderr(), HumanStyle::Live))),
            ProgressMode::Human => sinks.push(Box::new(Human::new(std::io::stderr(), HumanStyle::Live))),
            ProgressMode::Auto | ProgressMode::Plain => sinks.push(Box::new(Human::new(std::io::stderr(), HumanStyle::Plain))),
        }
        if let Some(p) = &self.events {
            sinks.push(Box::new(JsonLines(BufWriter::new(open(p)?))));
        }
        #[cfg(target_os = "linux")]
        if let Some(n) = oxibit_report::SdNotify::from_env() {
            sinks.push(Box::new(n));
        }
        Ok(Arc::new(Reporter::new(sinks, &CANCEL)))
    }
}

fn open(p: &Path) -> Result<File, String> {
    File::create(p).map_err(|e| format!("{}: {e}", p.display()))
}

/// For long commands: take SIGINT, SIGTERM, SIGUSR1 (and SIGINFO, Ctrl-T,
/// where it exists) away from the default handlers and serve them on one
/// thread with `sigwait`. The first SIGINT/SIGTERM sets `CANCEL` (the run
/// stops at its next check and writes what it has); a second exits at once
/// with 130. SIGUSR1/SIGINFO print the current status line.
///
/// Call before starting any other thread, so every thread inherits the mask.
#[cfg(unix)]
pub fn serve_signals(r: Arc<Reporter>) {
    // SAFETY: sigset_t is a plain C type; all-zero is a valid value to hand to sigemptyset.
    let mut set: libc::sigset_t = unsafe { std::mem::zeroed() };
    // SAFETY: `set` is a valid, writable sigset_t.
    unsafe { libc::sigemptyset(&raw mut set) };
    for s in signals() {
        // SAFETY: `set` was initialised by sigemptyset; `s` is a valid signal number.
        unsafe { libc::sigaddset(&raw mut set, s) };
    }
    // SAFETY: `set` is initialised; a null old-mask pointer is permitted.
    if unsafe { libc::pthread_sigmask(libc::SIG_BLOCK, &raw const set, std::ptr::null_mut()) } != 0 {
        return;
    }
    // `set` is Copy: the thread waits on its own copy of the initialised mask.
    std::thread::spawn(move || loop {
        let mut sig: libc::c_int = 0;
        // SAFETY: `set` is an initialised mask of signals blocked in every thread; `sig` is writable.
        if unsafe { libc::sigwait(&raw const set, &raw mut sig) } != 0 {
            return;
        }
        if sig == libc::SIGINT || sig == libc::SIGTERM {
            if CANCEL.swap(true, Ordering::SeqCst) {
                eprintln!("\noxibit: interrupted again, exiting");
                // SAFETY: _exit ends the process at once without running destructors or atexit
                // handlers, which is the intent of a second interrupt.
                unsafe { libc::_exit(130) };
            }
            eprintln!("\noxibit: interrupted; stopping at the next check and writing what is done (again to exit now)");
        } else {
            let line = r.last_status().unwrap_or_else(|| "starting".into());
            eprintln!("\noxibit: {line}");
        }
    });
}

#[cfg(unix)]
fn signals() -> Vec<libc::c_int> {
    #[cfg_attr(
        not(any(target_os = "macos", target_os = "freebsd", target_os = "netbsd", target_os = "openbsd")),
        expect(unused_mut, reason = "mut is needed only where SIGINFO exists")
    )]
    let mut v = vec![libc::SIGINT, libc::SIGTERM, libc::SIGUSR1];
    #[cfg(any(target_os = "macos", target_os = "freebsd", target_os = "netbsd", target_os = "openbsd"))]
    v.push(libc::SIGINFO);
    v
}

#[cfg(not(unix))]
pub fn serve_signals(_: Arc<Reporter>) {}
