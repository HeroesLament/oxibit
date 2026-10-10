use std::cell::RefCell;

use oxibit_bitstream::packet::cmd;
use oxibit_bitstream::{Container, SYNC};

use super::*;

const Z020: u32 = 0x0372_7093;
const Z010: u32 = 0x0372_2093;

struct FakeAperture(RefCell<Vec<u32>>);

impl Aperture for FakeAperture {
    fn base(&self) -> u64 {
        0x4000_0000
    }
    fn len(&self) -> usize {
        self.0.borrow().len() * 4
    }
    fn read32(&self, offset: usize) -> io::Result<u32> {
        Ok(self.0.borrow()[offset / 4])
    }
    fn write32(&self, offset: usize, value: u32) -> io::Result<()> {
        self.0.borrow_mut()[offset / 4] = value;
        Ok(())
    }
    fn read_block(&self, offset: usize, count: usize) -> io::Result<Vec<u32>> {
        Ok(self.0.borrow()[offset / 4..offset / 4 + count].to_vec())
    }
    fn read_repeat(&self, offset: usize, count: usize) -> io::Result<Vec<u32>> {
        Ok(vec![self.0.borrow()[offset / 4]; count])
    }
    fn write_block(&self, offset: usize, values: &[u32]) -> io::Result<()> {
        self.0.borrow_mut()[offset / 4..offset / 4 + values.len()].copy_from_slice(values);
        Ok(())
    }
}

struct Fake {
    idcode: u32,
    done: bool,
    fail: bool,
    programs: usize,
    aperture: FakeAperture,
}

impl Fake {
    fn new(idcode: u32) -> Self {
        Fake { idcode, done: false, fail: false, programs: 0, aperture: FakeAperture(RefCell::new(vec![0; 16])) }
    }
}

impl Backend for Fake {
    type Aperture = FakeAperture;
    fn idcode(&self) -> io::Result<u32> {
        Ok(self.idcode)
    }
    fn hardware(&self) -> io::Result<Hardware> {
        Ok(Hardware { done: self.done, boundary_open: true, unknown: None, detail: serde_json::Value::Null })
    }
    fn program(&mut self, _words: &[u32]) -> Result<ProgramReport, String> {
        self.programs += 1;
        self.done = false; // PROG_B
        if self.fail {
            return Err("PCFG_DONE timeout".into());
        }
        self.done = true;
        Ok(ProgramReport { ms: 1, bytes: 0, detail: serde_json::Value::Null })
    }
    fn clear(&mut self) -> Result<(), String> {
        self.done = false;
        Ok(())
    }
    fn aperture(&self) -> &FakeAperture {
        &self.aperture
    }
}

/// Smallest stream `analyze` accepts: IDCODE, one frame, START, DESYNC.
fn bitstream(idcode: u32) -> Bitstream {
    let mut w = vec![0xFFFF_FFFF, SYNC, 0x3001_8001, idcode, 0x3000_4000, 0x5000_0000 | 101];
    w.extend(std::iter::repeat_n(0, 101));
    w.extend([0x3000_8001, cmd::START, 0x3000_8001, cmd::DESYNC]);
    Bitstream { container: Container::RawBin, header: None, words: w }
}

fn run_dir(name: &str) -> RunDir {
    let dir = std::env::temp_dir().join(format!("oxibit-guard-test-{}-{name}", std::process::id()));
    let _ = fs::remove_dir_all(&dir);
    RunDir(dir)
}

#[test]
fn access_refused_until_loaded_then_granted() {
    let run = run_dir("happy");
    let mut g = Guard::open(Fake::new(Z020), run.clone()).unwrap();
    assert!(matches!(g.access(), Err(Error::NotReady { .. })));

    let r = g.load(&bitstream(Z020), "t.bin", LoadOptions::default()).unwrap();
    assert_eq!(r.loaded.die.as_deref(), Some("xc7z020"));
    assert_eq!(run.read_marker().unwrap(), r.loaded);

    g.access().unwrap().write32(4, 7).unwrap();
    assert_eq!(g.access().unwrap().read32(4).unwrap(), 7);

    let s = g.clear().unwrap();
    assert!(!s.ready && s.loaded.is_none());
    assert!(matches!(g.access(), Err(Error::NotReady { .. })));
}

#[test]
fn rejects_before_touching_hardware() {
    let mut g = Guard::open(Fake::new(Z020), run_dir("reject")).unwrap();

    let mut no_desync = bitstream(Z020);
    no_desync.words.truncate(no_desync.words.len() - 2);
    assert!(matches!(g.load(&no_desync, "x", LoadOptions::default()), Err(Error::Rejected { .. })));

    assert!(matches!(g.load(&bitstream(Z010), "x", LoadOptions::default()), Err(Error::WrongDevice { .. })));
    assert_eq!(g.backend().programs, 0);
}

#[test]
fn failed_load_leaves_no_marker_and_no_access() {
    let run = run_dir("fail");
    let mut g = Guard::open(Fake::new(Z020), run.clone()).unwrap();
    g.load(&bitstream(Z020), "good", LoadOptions::default()).unwrap();
    assert!(run.read_marker().is_some());

    g.backend.fail = true;
    assert!(matches!(g.load(&bitstream(Z020), "bad", LoadOptions::default()), Err(Error::Program { .. })));
    assert!(run.read_marker().is_none());
    assert!(matches!(g.access(), Err(Error::NotReady { .. })));
}

#[test]
fn lock_is_exclusive_and_reports_holder() {
    let run = run_dir("lock");
    let g = Guard::open(Fake::new(Z020), run.clone()).unwrap();
    // flock is per open file description, so a second open in the same
    // process conflicts just as another process would.
    match Guard::open(Fake::new(Z020), run.clone()) {
        Err(Error::Locked { pid, .. }) => assert_eq!(pid, Some(std::process::id())),
        other => panic!("expected Locked, got {:?}", other.map(|_| ())),
    }
    assert_eq!(run.lock_holder(), Some(std::process::id()));
    drop(g);
    assert_eq!(run.lock_holder(), None);
    Guard::open(Fake::new(Z020), run).unwrap();
}

#[test]
fn observe_flags_foreign_configuration_and_stale_markers() {
    let run = run_dir("observe");
    fs::create_dir_all(&run.0).unwrap();
    let mut fake = Fake::new(Z020);
    fake.done = true;
    let s = observe(&fake, &run).unwrap();
    assert!(s.ready && s.loaded.is_none());
    assert!(s.notes[0].contains("other than oxibit"));

    run.write_marker(&Marker {
        source: "old".into(),
        config_sha256: None,
        idcode: None,
        die: None,
        design: None,
        built: None,
        bytes: 0,
        load_ms: 0,
        loaded_at_unix: 0,
        loaded_at_uptime_s: 0,
        aperture: true,
    })
    .unwrap();
    fake.done = false;
    let s = observe(&fake, &run).unwrap();
    assert!(!s.ready && s.loaded.is_none());
    assert!(s.notes[0].contains("ignored"));
}

#[test]
fn no_aperture_design_refuses_access_until_next_load() {
    let run = run_dir("noap");
    let mut g = Guard::open(Fake::new(Z020), run.clone()).unwrap();
    g.load(&bitstream(Z020), "hp_loop.bin", LoadOptions { aperture: false, hash: true }).unwrap();
    assert!(matches!(g.access(), Err(Error::NotReady { reason }) if reason.contains("no slave")));
    let s = g.status().unwrap();
    assert!(!s.ready && s.reason.contains("no slave"));

    // A new guard (port restart) picks the declaration up from the marker.
    drop(g);
    let mut g = Guard::open(Fake::new(Z020), run).unwrap();
    g.backend.done = true;
    assert!(matches!(g.access(), Err(Error::NotReady { .. })));

    g.load(&bitstream(Z020), "radio.bin", LoadOptions::default()).unwrap();
    assert!(g.access().is_ok());
}
