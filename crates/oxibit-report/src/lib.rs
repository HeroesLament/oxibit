//! Progress and result events for oxibit tools.
//!
//! Library code reports through `&dyn Progress` and never prints. The binary
//! decides where events go: a terminal (`Human`), JSON Lines (`JsonLines`),
//! port frames, systemd. Every sink sees the same `Event`s, so a terminal,
//! fabfarm and a tmux wait all read one schema (`docs/events.md`).
//!
//! Wire form of an event, one JSON object per line:
//!
//! ```text
//! {"v":1,"t_ms":3990,"ev":"progress","phase":"route","iter":5,"overused":329}
//! ```

use std::collections::BTreeMap;
use std::io::Write;
use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Instant;

use serde::Serialize;
use serde_json::{Map, Value};

/// Schema version carried by every event.
pub const SCHEMA: u32 = 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum PhaseState {
    Begin,
    End,
}

/// One event. Serialized flat with `"ev"` as the tag.
#[derive(Debug, Clone, Serialize)]
#[serde(tag = "ev", rename_all = "snake_case")]
pub enum Event {
    Start {
        cmd: String,
        version: String,
        args: Vec<String>,
    },
    Phase {
        phase: String,
        state: PhaseState,
        #[serde(skip_serializing_if = "Option::is_none")]
        ms: Option<u64>,
    },
    /// Phase-specific counters (`iter`, `overused`, ...), flattened.
    Progress {
        phase: String,
        #[serde(flatten)]
        data: Map<String, Value>,
    },
    Warn {
        code: String,
        msg: String,
    },
    Done {
        ok: bool,
        exit: i32,
        cancelled: bool,
        phases: BTreeMap<String, u64>,
        #[serde(skip_serializing_if = "Option::is_none")]
        peak_rss_mb: Option<u64>,
        stats: Value,
        check: Value,
        outputs: Vec<String>,
    },
}

impl Event {
    pub fn progress(phase: &str, data: Value) -> Event {
        let data = match data {
            Value::Object(m) => m,
            other => Map::from_iter([("value".to_string(), other)]),
        };
        Event::Progress { phase: phase.to_string(), data }
    }
}

/// An event as it goes on the wire: version and time since start, then the
/// event's own fields.
#[derive(Serialize)]
pub struct Envelope<'a> {
    pub v: u32,
    pub t_ms: u64,
    #[serde(flatten)]
    pub event: &'a Event,
}

/// What library code reports through.
pub trait Progress: Sync {
    fn emit(&self, event: Event);
    /// Polled between units of work; a cancelled run stops early and
    /// returns what it has.
    fn cancelled(&self) -> bool {
        false
    }
}

/// Reports nothing; never cancelled.
pub struct Silent;
impl Progress for Silent {
    fn emit(&self, _: Event) {}
}

/// Where a `Reporter` sends events.
pub trait Sink: Send {
    fn write(&mut self, t_ms: u64, event: &Event);
    fn finish(&mut self) {}
}

/// Fans events out to sinks, stamps them, times phases, and carries the
/// cancel flag.
pub struct Reporter {
    start: Instant,
    sinks: Mutex<Vec<Box<dyn Sink>>>,
    phase_start: Mutex<BTreeMap<String, u64>>,
    phase_ms: Mutex<BTreeMap<String, u64>>,
    cancel: &'static AtomicBool,
    status: Mutex<Option<String>>,
}

impl Reporter {
    /// `cancel` is usually set from a signal handler.
    pub fn new(sinks: Vec<Box<dyn Sink>>, cancel: &'static AtomicBool) -> Self {
        Reporter {
            start: Instant::now(),
            sinks: Mutex::new(sinks),
            phase_start: Mutex::new(BTreeMap::new()),
            phase_ms: Mutex::new(BTreeMap::new()),
            cancel,
            status: Mutex::new(None),
        }
    }

    pub fn elapsed_ms(&self) -> u64 {
        self.start.elapsed().as_millis() as u64
    }

    pub fn begin(&self, phase: &str) {
        self.emit(Event::Phase { phase: phase.into(), state: PhaseState::Begin, ms: None });
    }

    pub fn end(&self, phase: &str) {
        self.emit(Event::Phase { phase: phase.into(), state: PhaseState::End, ms: None });
    }

    /// Phase durations so far (for `Done`).
    pub fn phases(&self) -> BTreeMap<String, u64> {
        self.phase_ms.lock().unwrap().clone()
    }

    /// The latest one-line status (`status_line` of the last event that has
    /// one), for SIGINFO/SIGUSR1.
    pub fn last_status(&self) -> Option<String> {
        self.status.lock().unwrap().clone()
    }

    pub fn finish(&self) {
        for s in self.sinks.lock().unwrap().iter_mut() {
            s.finish();
        }
    }
}

impl Progress for Reporter {
    fn emit(&self, mut event: Event) {
        let t = self.elapsed_ms();
        if let Event::Phase { phase, state, ms } = &mut event {
            match state {
                PhaseState::Begin => {
                    self.phase_start.lock().unwrap().insert(phase.clone(), t);
                }
                PhaseState::End => {
                    let began = self.phase_start.lock().unwrap().remove(phase).unwrap_or(t);
                    *ms = Some(t - began);
                    *self.phase_ms.lock().unwrap().entry(phase.clone()).or_default() += t - began;
                }
            }
        }
        if let Some(line) = status_line(&event) {
            *self.status.lock().unwrap() = Some(format!("{line} [{:.1} s]", t as f64 / 1000.0));
        }
        for s in self.sinks.lock().unwrap().iter_mut() {
            s.write(t, &event);
        }
    }

    fn cancelled(&self) -> bool {
        self.cancel.load(Ordering::Relaxed)
    }
}

/// JSON Lines, flushed per event.
pub struct JsonLines<W: Write + Send>(pub W);

impl<W: Write + Send> Sink for JsonLines<W> {
    fn write(&mut self, t_ms: u64, event: &Event) {
        let env = Envelope { v: SCHEMA, t_ms, event };
        if let Ok(line) = serde_json::to_string(&env) {
            let _ = writeln!(self.0, "{line}");
            let _ = self.0.flush();
        }
    }
}

/// How `Human` draws.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HumanStyle {
    /// A terminal: progress rewrites one status line in place.
    Live,
    /// A pipe or log: one plain line per progress event.
    Plain,
}

/// Human-readable output on stderr (or any writer).
pub struct Human<W: Write + Send> {
    out: W,
    style: HumanStyle,
    /// A status line is on screen and must be cleared before other output.
    live_line: bool,
    cmd: String,
}

impl<W: Write + Send> Human<W> {
    pub fn new(out: W, style: HumanStyle) -> Self {
        Human { out, style, live_line: false, cmd: "oxibit".into() }
    }

    fn clear(&mut self) {
        if self.live_line {
            let _ = write!(self.out, "\r\x1b[2K");
            self.live_line = false;
        }
    }

    fn line(&mut self, text: &str) {
        self.clear();
        let _ = writeln!(self.out, "{text}");
        let _ = self.out.flush();
    }
}

/// `key value` pairs of a progress event, in the order the event lists them.
fn fields(data: &Map<String, Value>) -> String {
    data.iter()
        .map(|(k, v)| match v {
            Value::String(s) => format!("{k} {s}"),
            Value::Number(n) if n.is_f64() => format!("{k} {:.2}", n.as_f64().unwrap_or(0.0)),
            other => format!("{k} {other}"),
        })
        .collect::<Vec<_>>()
        .join(", ")
}

impl<W: Write + Send> Sink for Human<W> {
    fn write(&mut self, t_ms: u64, event: &Event) {
        let t = t_ms as f64 / 1000.0;
        match event {
            Event::Start { cmd, .. } => self.cmd = format!("oxibit {cmd}"),
            Event::Phase { phase, state: PhaseState::End, ms } => {
                let ms = ms.unwrap_or(0);
                let cmd = self.cmd.clone();
                self.line(&format!("{cmd}: {phase} {:.2} s", ms as f64 / 1000.0));
            }
            Event::Phase { .. } => {}
            Event::Progress { phase, data } => {
                let text = format!("{}: {phase}: {} [{t:.1} s]", self.cmd, fields(data));
                match self.style {
                    HumanStyle::Live => {
                        let _ = write!(self.out, "\r\x1b[2K{text}");
                        let _ = self.out.flush();
                        self.live_line = true;
                    }
                    HumanStyle::Plain => self.line(&text),
                }
            }
            Event::Warn { code, msg } => {
                let cmd = self.cmd.clone();
                self.line(&format!("{cmd}: warning [{code}]: {msg}"));
            }
            Event::Done { ok, cancelled, outputs, .. } => {
                let state = if *cancelled {
                    "cancelled"
                } else if *ok {
                    "done"
                } else {
                    "FAILED"
                };
                let cmd = self.cmd.clone();
                let out = if outputs.is_empty() { String::new() } else { format!(" -> {}", outputs.join(", ")) };
                self.line(&format!("{cmd}: {state} in {t:.2} s{out}"));
            }
        }
    }

    fn finish(&mut self) {
        self.clear();
        let _ = self.out.flush();
    }
}

/// systemd's notification socket (`NOTIFY_SOCKET`, set for units with
/// `Type=notify`): `READY=1` at start, `STATUS=` with the current phase or
/// progress (at most once a second), and the outcome at the end, so
/// `systemctl status UNIT` shows where a run is.
#[cfg(target_os = "linux")]
pub struct SdNotify {
    sock: std::os::unix::net::UnixDatagram,
    addr: std::os::unix::net::SocketAddr,
    last_ms: Option<u64>,
}

#[cfg(target_os = "linux")]
impl SdNotify {
    /// `None` when `NOTIFY_SOCKET` is unset or unusable.
    pub fn from_env() -> Option<Self> {
        use std::os::linux::net::SocketAddrExt;
        use std::os::unix::net::{SocketAddr, UnixDatagram};
        let path = std::env::var("NOTIFY_SOCKET").ok()?;
        let addr = match path.strip_prefix('@') {
            Some(name) => SocketAddr::from_abstract_name(name.as_bytes()).ok()?,
            None => SocketAddr::from_pathname(&path).ok()?,
        };
        Some(SdNotify { sock: UnixDatagram::unbound().ok()?, addr, last_ms: None })
    }

    fn send(&self, msg: &str) {
        let _ = self.sock.send_to_addr(msg.as_bytes(), &self.addr);
    }
}

#[cfg(target_os = "linux")]
impl Sink for SdNotify {
    fn write(&mut self, t_ms: u64, event: &Event) {
        match event {
            Event::Start { cmd, .. } => self.send(&format!("READY=1\nSTATUS=oxibit {cmd}: starting")),
            Event::Done { .. } => {
                if let Some(line) = status_line(event) {
                    self.send(&format!("STATUS={line}"));
                }
            }
            Event::Progress { .. } | Event::Phase { .. } => {
                // Phase changes always; progress at most once a second.
                let due = matches!(event, Event::Phase { .. }) || self.last_ms.is_none_or(|l| t_ms >= l + 1000);
                if let (true, Some(line)) = (due, status_line(event)) {
                    self.send(&format!("STATUS={line}"));
                    self.last_ms = Some(t_ms);
                }
            }
            Event::Warn { .. } => {}
        }
    }
}

/// The latest one-line status, for SIGINFO/SIGUSR1 and `sd_notify`.
pub fn status_line(event: &Event) -> Option<String> {
    match event {
        Event::Phase { phase, state: PhaseState::Begin, .. } => Some(format!("{phase}...")),
        Event::Progress { phase, data } => Some(format!("{phase}: {}", fields(data))),
        Event::Done { ok, cancelled, .. } => Some(if *cancelled { "cancelled".into() } else if *ok { "done".into() } else { "failed".into() }),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    /// The wire form is the contract (docs/events.md); pin it.
    #[test]
    fn wire_form() {
        let e = Event::progress("route", json!({"iter": 5, "overused": 329}));
        let s = serde_json::to_string(&Envelope { v: SCHEMA, t_ms: 3990, event: &e }).unwrap();
        assert_eq!(s, r#"{"v":1,"t_ms":3990,"ev":"progress","phase":"route","iter":5,"overused":329}"#);

        let e = Event::Phase { phase: "graph".into(), state: PhaseState::End, ms: Some(4690) };
        let s = serde_json::to_string(&Envelope { v: SCHEMA, t_ms: 4900, event: &e }).unwrap();
        assert_eq!(s, r#"{"v":1,"t_ms":4900,"ev":"phase","phase":"graph","state":"end","ms":4690}"#);

        let e = Event::Done {
            ok: true,
            exit: 0,
            cancelled: false,
            phases: BTreeMap::from([("route".to_string(), 26100)]),
            peak_rss_mb: None,
            stats: json!({}),
            check: json!({}),
            outputs: vec!["out.fasm".into()],
        };
        let s = serde_json::to_string(&Envelope { v: SCHEMA, t_ms: 40000, event: &e }).unwrap();
        assert_eq!(
            s,
            r#"{"v":1,"t_ms":40000,"ev":"done","ok":true,"exit":0,"cancelled":false,"phases":{"route":26100},"stats":{},"check":{},"outputs":["out.fasm"]}"#
        );
    }

    #[test]
    fn reporter_times_phases() {
        static C: AtomicBool = AtomicBool::new(false);
        let r = Reporter::new(vec![], &C);
        r.begin("graph");
        r.end("graph");
        assert!(r.phases().contains_key("graph"));
        assert!(!r.cancelled());
    }
}
