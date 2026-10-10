//! The load/access safety model for programmable logic, independent of any
//! particular chip.
//!
//! # The hazard
//!
//! On a Zynq-7000 a CPU read into the PL's AXI aperture with no configured
//! slave behind it never completes: the interconnect has no timeout, so the
//! core hangs, with no fault and no recovery short of a reset. Loading a
//! bitstream is exactly when that state exists, because the load starts by
//! erasing the running design (PROG_B) and a load that fails anywhere after
//! that leaves the PL blank.
//!
//! # The rules this crate enforces
//!
//! 1. **One owner.** A [`Guard`] holds an exclusive lock for its lifetime.
//!    Loading and access go through the same owner, so no second process can
//!    reconfigure the PL underneath an access, or access it mid-load.
//! 2. **Hardware decides.** Access is granted by [`Guard::access`], which
//!    re-reads the hardware's own state on every call ([`Hardware::ready`]):
//!    configuration done *and* the PS-PL boundary open. Nothing is cached, so
//!    nothing can go stale.
//! 3. **Check before touching.** [`Guard::load`] refuses a bitstream with any
//!    [`oxibit_bitstream::analyze`] problem, or whose IDCODE is not the
//!    device's, before the backend touches hardware.
//! 4. **Fail closed.** A backend must leave [`Hardware::ready`] false after any
//!    failed or interrupted load. The loaded-bitstream [`Marker`] is removed
//!    before programming starts and written only after hardware confirms.
//!
//! Within one process the borrow checker adds the last piece: the aperture is
//! borrowed from `&Guard`, while `load` and `clear` take `&mut Guard`, so no
//! aperture reference can be alive across a reconfiguration.

use std::fs::{self, File, OpenOptions};
use std::io::{self, Read, Seek, SeekFrom, Write};
use std::os::unix::io::AsRawFd;
use std::path::PathBuf;
use std::time::{SystemTime, UNIX_EPOCH};

use oxibit_bitstream::{Bitstream, analyze_with, part};
use serde::{Deserialize, Serialize};

/// Word access to a mapped PL aperture. Offsets are bytes from its base.
pub trait Aperture {
    fn base(&self) -> u64;
    fn len(&self) -> usize;
    fn is_empty(&self) -> bool {
        self.len() == 0
    }
    fn read32(&self, offset: usize) -> io::Result<u32>;
    fn write32(&self, offset: usize, value: u32) -> io::Result<()>;
    /// `count` consecutive words.
    fn read_block(&self, offset: usize, count: usize) -> io::Result<Vec<u32>>;
    /// The same word `count` times (a FIFO or auto-advancing read port).
    fn read_repeat(&self, offset: usize, count: usize) -> io::Result<Vec<u32>>;
    fn write_block(&self, offset: usize, values: &[u32]) -> io::Result<()>;
}

/// What the hardware says about the PL right now.
#[derive(Debug, Clone, Serialize)]
pub struct Hardware {
    /// The configuration engine reported completion.
    pub done: bool,
    /// The PS-PL boundary is open in both directions and the PL-side AXI
    /// resets are released.
    pub boundary_open: bool,
    /// Why `done` could not be determined, when it could not. `done` is then
    /// false: an unknown state is treated as unconfigured.
    pub unknown: Option<String>,
    /// Backend-specific raw register values, for status output.
    pub detail: serde_json::Value,
}

impl Hardware {
    /// Safe to access the aperture.
    pub fn ready(&self) -> bool {
        self.done && self.boundary_open
    }

    fn why_not(&self) -> String {
        if let Some(u) = &self.unknown {
            return format!("configuration state unknown: {u}");
        }
        match (self.done, self.boundary_open) {
            (false, _) => "PL not configured".into(),
            (true, false) => "PS-PL boundary closed or AXI resets asserted".into(),
            (true, true) => "ready".into(),
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct ProgramReport {
    pub ms: u128,
    pub bytes: usize,
    pub detail: serde_json::Value,
}

/// A chip's configuration port, boundary and aperture.
pub trait Backend {
    type Aperture: Aperture;

    /// The device IDCODE as the silicon reports it.
    fn idcode(&self) -> io::Result<u32>;

    /// Current hardware state. Must not touch the aperture.
    fn hardware(&self) -> io::Result<Hardware>;

    /// Program the PL with configuration words in canonical (big-endian)
    /// order. On any error `hardware().ready()` must be false afterwards.
    fn program(&mut self, words: &[u32]) -> Result<ProgramReport, String>;

    /// Erase the PL and leave it unconfigured.
    fn clear(&mut self) -> Result<(), String>;

    fn aperture(&self) -> &Self::Aperture;
}

#[derive(Debug, Clone, Serialize)]
#[serde(tag = "error", rename_all = "snake_case")]
pub enum Error {
    /// Another process holds the lock.
    Locked { pid: Option<u32>, lock: String },
    /// The bitstream failed analysis; nothing was touched.
    Rejected { problems: Vec<String> },
    /// The bitstream targets a different device; nothing was touched.
    WrongDevice { bitstream: String, device: String },
    /// Programming failed or did not leave the PL ready. The PL is blank.
    Program { reason: String },
    /// Aperture access refused.
    NotReady { reason: String },
    Io { reason: String },
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::Locked { pid: Some(pid), lock } => write!(f, "{lock} is held by pid {pid}"),
            Error::Locked { pid: None, lock } => write!(f, "{lock} is held by another process"),
            Error::Rejected { problems } => write!(f, "bitstream rejected: {}", problems.join("; ")),
            Error::WrongDevice { bitstream, device } => {
                write!(f, "bitstream is for {bitstream}, this device is {device}")
            }
            Error::Program { reason } => write!(f, "programming failed, PL is blank: {reason}"),
            Error::NotReady { reason } => write!(f, "PL access refused: {reason}"),
            Error::Io { reason } => write!(f, "{reason}"),
        }
    }
}

impl std::error::Error for Error {}

impl From<io::Error> for Error {
    fn from(e: io::Error) -> Self {
        Error::Io { reason: e.to_string() }
    }
}

/// What is loaded, as recorded by the guard that loaded it.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct Marker {
    pub source: String,
    /// `None` when the load skipped hashing ([`LoadOptions::hash`]).
    pub config_sha256: Option<String>,
    pub idcode: Option<u32>,
    pub die: Option<String>,
    pub design: Option<String>,
    pub built: Option<String>,
    pub bytes: usize,
    pub load_ms: u128,
    pub loaded_at_unix: u64,
    /// Seconds since boot at load time; unlike the wall clock, always right.
    #[serde(default)]
    pub loaded_at_uptime_s: u64,
    /// Whether the loaded design has a slave behind the aperture. A design
    /// without one hangs the CPU on access exactly as a blank PL does, and
    /// only the loader can know, so it is declared at load time.
    #[serde(default = "yes")]
    pub aperture: bool,
}

fn yes() -> bool {
    true
}

/// Per-load choices.
#[derive(Debug, Clone, Copy)]
pub struct LoadOptions {
    /// The design answers on the aperture. `false` makes [`Guard::access`]
    /// refuse until the next load.
    pub aperture: bool,
    /// Compute the configuration SHA-256 for the marker. Every check runs
    /// either way; this only decides whether the identity hash is recorded.
    pub hash: bool,
}

impl Default for LoadOptions {
    fn default() -> Self {
        LoadOptions { aperture: true, hash: true }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct Status {
    pub device_idcode: Option<u32>,
    pub device_die: Option<String>,
    pub ready: bool,
    pub reason: String,
    pub hardware: Hardware,
    pub loaded: Option<Marker>,
    pub lock_holder: Option<u32>,
    pub notes: Vec<String>,
}

#[derive(Debug, Clone, Serialize)]
pub struct LoadReport {
    pub loaded: Marker,
    pub program: ProgramReport,
    pub hardware: Hardware,
}

/// Where the lock and marker live. Must be tmpfs: both are only meaningful
/// until the next reset, which erases the PL.
#[derive(Debug, Clone)]
pub struct RunDir(pub PathBuf);

impl RunDir {
    /// `$OXIBIT_RUN_DIR`, else `/run/oxibit` if `/run` exists, else `/tmp/oxibit`.
    pub fn from_env() -> Self {
        if let Some(dir) = std::env::var_os("OXIBIT_RUN_DIR") {
            return RunDir(dir.into());
        }
        if std::path::Path::new("/run").is_dir() {
            RunDir("/run/oxibit".into())
        } else {
            RunDir("/tmp/oxibit".into())
        }
    }

    pub fn lock_path(&self) -> PathBuf {
        self.0.join("lock")
    }

    pub fn marker_path(&self) -> PathBuf {
        self.0.join("loaded.json")
    }

    fn read_marker(&self) -> Option<Marker> {
        let text = fs::read_to_string(self.marker_path()).ok()?;
        serde_json::from_str(&text).ok()
    }

    fn remove_marker(&self) -> io::Result<()> {
        match fs::remove_file(self.marker_path()) {
            Err(e) if e.kind() != io::ErrorKind::NotFound => Err(e),
            _ => Ok(()),
        }
    }

    fn write_marker(&self, m: &Marker) -> io::Result<()> {
        let tmp = self.0.join("loaded.json.tmp");
        fs::write(&tmp, serde_json::to_vec_pretty(m).map_err(io::Error::other)?)?;
        fs::rename(tmp, self.marker_path())
    }

    /// The pid holding the lock, if any. Does not take the lock.
    pub fn lock_holder(&self) -> Option<u32> {
        let mut f = File::open(self.lock_path()).ok()?;
        // SAFETY: flock on a valid fd.
        let free = unsafe { libc::flock(f.as_raw_fd(), libc::LOCK_SH | libc::LOCK_NB) } == 0;
        if free {
            // SAFETY: as above; release the probe lock at once.
            unsafe { libc::flock(f.as_raw_fd(), libc::LOCK_UN) };
            return None;
        }
        let mut s = String::new();
        f.read_to_string(&mut s).ok()?;
        s.trim().parse().ok()
    }
}

/// Exclusive owner of a PL region.
pub struct Guard<B: Backend> {
    backend: B,
    run: RunDir,
    /// What this guard (or a previous holder of the lock) loaded. Only a lock
    /// holder writes the marker, so while the lock is held this copy is
    /// authoritative and access checks need not re-read the file.
    loaded: Option<Marker>,
    _lock: File,
}

impl<B: Backend> Guard<B> {
    /// Take the lock. Fails with [`Error::Locked`] if another guard holds it.
    pub fn open(backend: B, run: RunDir) -> Result<Self, Error> {
        fs::create_dir_all(&run.0)?;
        let path = run.lock_path();
        let mut lock = OpenOptions::new().read(true).write(true).create(true).truncate(false).open(&path)?;

        // SAFETY: flock on a valid fd.
        if unsafe { libc::flock(lock.as_raw_fd(), libc::LOCK_EX | libc::LOCK_NB) } != 0 {
            let mut s = String::new();
            let _ = lock.read_to_string(&mut s);
            return Err(Error::Locked { pid: s.trim().parse().ok(), lock: path.display().to_string() });
        }

        lock.set_len(0)?;
        lock.seek(SeekFrom::Start(0))?;
        writeln!(lock, "{}", std::process::id())?;
        lock.flush()?;

        let loaded = run.read_marker();
        Ok(Guard { backend, run, loaded, _lock: lock })
    }

    pub fn backend(&self) -> &B {
        &self.backend
    }

    pub fn status(&self) -> Result<Status, Error> {
        observe(&self.backend, &self.run)
    }

    /// Check, program, confirm, record. See the crate docs for the order.
    pub fn load(&mut self, bitstream: &Bitstream, source: &str, options: LoadOptions) -> Result<LoadReport, Error> {
        let report = analyze_with(bitstream, options.hash).map_err(|e| Error::Rejected { problems: vec![e.to_string()] })?;
        if !report.problems.is_empty() {
            return Err(Error::Rejected { problems: report.problems });
        }

        let device = self.backend.idcode()?;
        let Some(stream_id) = report.idcode else {
            return Err(Error::Rejected { problems: vec!["no IDCODE in the bitstream".into()] });
        };
        if stream_id & 0x0FFF_FFFF != device & 0x0FFF_FFFF {
            return Err(Error::WrongDevice { bitstream: describe(stream_id), device: describe(device) });
        }

        // From here on the PL may be blank until hardware says otherwise.
        self.loaded = None;
        self.run.remove_marker()?;

        let program = self.backend.program(&bitstream.words).map_err(|reason| Error::Program { reason })?;

        let hardware = self.backend.hardware()?;
        if !hardware.ready() {
            return Err(Error::Program {
                reason: format!("loader reported success but {}", hardware.why_not()),
            });
        }

        let header = bitstream.header.as_ref();
        let loaded = Marker {
            source: source.to_string(),
            config_sha256: report.config_sha256,
            idcode: Some(stream_id),
            die: report.die.map(|d| d.name.to_string()),
            design: header.and_then(|h| h.design.clone()),
            built: header.map(|h| {
                format!("{} {}", h.date.as_deref().unwrap_or(""), h.time.as_deref().unwrap_or("")).trim().to_string()
            }),
            bytes: bitstream.words.len() * 4,
            load_ms: program.ms,
            loaded_at_unix: SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0),
            loaded_at_uptime_s: uptime_s(),
            aperture: options.aperture,
        };
        self.run.write_marker(&loaded)?;
        self.loaded = Some(loaded.clone());

        Ok(LoadReport { loaded, program, hardware })
    }

    /// Erase the PL.
    pub fn clear(&mut self) -> Result<Status, Error> {
        self.loaded = None;
        self.run.remove_marker()?;
        self.backend.clear().map_err(|reason| Error::Program { reason })?;
        self.status()
    }

    /// The aperture, if and only if hardware says it is safe right now.
    pub fn access(&self) -> Result<&B::Aperture, Error> {
        if let Some(m) = &self.loaded {
            if !m.aperture {
                return Err(Error::NotReady {
                    reason: format!("{} was loaded with no slave on the aperture", m.source),
                });
            }
        }
        let hw = self.backend.hardware()?;
        if hw.ready() {
            Ok(self.backend.aperture())
        } else {
            Err(Error::NotReady { reason: hw.why_not() })
        }
    }
}

/// Status without taking the lock. Reads configuration and boundary state
/// only; never touches the aperture.
pub fn observe<B: Backend>(backend: &B, run: &RunDir) -> Result<Status, Error> {
    let hardware = backend.hardware()?;
    let idcode = backend.idcode().ok();
    let mut notes = Vec::new();

    let loaded = match (run.read_marker(), hardware.done) {
        (Some(m), true) => Some(m),
        (Some(_), false) => {
            notes.push("marker present but PL not configured: ignored".into());
            None
        }
        (None, true) => {
            notes.push("PL configured by something other than oxibit: identity unknown".into());
            None
        }
        (None, false) => None,
    };

    let no_slave = loaded.as_ref().filter(|m| !m.aperture).map(|m| format!("{} was loaded with no slave on the aperture", m.source));

    Ok(Status {
        device_idcode: idcode,
        device_die: idcode.and_then(part::by_idcode).map(|d| d.name.to_string()),
        ready: hardware.ready() && no_slave.is_none(),
        reason: no_slave.unwrap_or_else(|| hardware.why_not()),
        hardware,
        loaded,
        lock_holder: run.lock_holder(),
        notes,
    })
}

fn uptime_s() -> u64 {
    fs::read_to_string("/proc/uptime")
        .ok()
        .and_then(|s| s.split('.').next().and_then(|n| n.parse().ok()))
        .unwrap_or(0)
}

fn describe(idcode: u32) -> String {
    match part::by_idcode(idcode) {
        Some(d) => format!("{} (0x{idcode:08X})", d.name),
        None => format!("0x{idcode:08X}"),
    }
}

#[cfg(test)]
mod tests;
