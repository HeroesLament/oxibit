//! Random, database-valid FASM for checking the bitstream writer against
//! `prjoxide pack` byte for byte. No design tools involved: features are
//! drawn straight from the bit database.
//!
//!   cargo run --release -p oxibit-nexus --example fuzz_fasm -- \
//!       [--db DIR] [--device LIFCL-40] [--tiles N] [--groups N] [--ip] SEED > x.fasm
//!
//! Covers pips, fixed connections, enums (BASE_ and other), words, unknown
//! bits, bel tilegroups, metadata and IP writes (EBR, PLL, LRAM, DPHY).

use std::collections::BTreeSet;
use std::fmt::Write;
use std::path::PathBuf;
use std::process::ExitCode;

use oxibit_nexus::db::TileBits;
use oxibit_nexus::{Chip, DeviceDb};

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        // xorshift64*
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    fn below(&mut self, n: usize) -> usize {
        (self.next() % n.max(1) as u64) as usize
    }
    fn pick<'a, T>(&mut self, v: &'a [T]) -> Option<&'a T> {
        if v.is_empty() { None } else { Some(&v[self.below(v.len())]) }
    }
}

/// Whether prjoxide's parser can carry the name through unchanged.
fn plain(s: &str) -> bool {
    !s.is_empty() && !s.contains("__") && s.chars().all(|c| c.is_ascii_alphanumeric() || c == '_' || c == ':' || c == '.')
}

fn f(s: &str) -> String {
    s.replace(':', "__")
}

fn word_value(rng: &mut Rng, n: usize) -> String {
    let bits: String = (0..n).map(|_| if rng.next() & 1 == 1 { '1' } else { '0' }).collect();
    format!("[{}:0] = {n}'b{bits}", n - 1)
}

fn enums_words(out: &mut String, rng: &mut Rng, prefix: &str, tb: &[&TileBits], k: usize) {
    let enums: BTreeSet<(&str, Vec<&str>)> = tb
        .iter()
        .flat_map(|t| t.enums.iter())
        .filter(|(n, _)| plain(n) && !n.contains(':'))
        .map(|(n, e)| (n.as_str(), e.options.keys().map(String::as_str).filter(|o| plain(o) && !o.contains('.') && !o.contains(':')).collect()))
        .collect();
    let enums: Vec<_> = enums.into_iter().filter(|(_, o)| !o.is_empty()).collect();
    for _ in 0..k {
        if let Some((n, opts)) = rng.pick(&enums) {
            let o = opts[rng.below(opts.len())];
            writeln!(out, "{prefix}.{n}.{o}").ok();
        }
    }
    let words: BTreeSet<(&str, usize)> = tb
        .iter()
        .flat_map(|t| t.words.iter())
        .filter(|(n, w)| plain(n) && !n.contains(':') && !w.bits.is_empty())
        .map(|(n, w)| (n.as_str(), w.bits.len()))
        .collect();
    let words: Vec<_> = words.into_iter().collect();
    for _ in 0..k.div_ceil(2) {
        if let Some(&(n, len)) = rng.pick(&words) {
            writeln!(out, "{prefix}.{n}{}", word_value(rng, len)).ok();
        }
    }
}

fn run() -> Result<(), String> {
    let mut db = std::env::var_os("OXIBIT_PRJOXIDE_DB")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from(std::env::var("HOME").unwrap_or_default()).join("src/prjoxide/database"));
    let mut device = "LIFCL-40".to_string();
    let (mut ntiles, mut ngroups, mut ip) = (40usize, 8usize, false);
    let mut seed = None;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        let mut val = |what: &str| args.next().ok_or(format!("{what} needs a value"));
        match a.as_str() {
            "--db" => db = PathBuf::from(val("--db")?),
            "--device" => device = val("--device")?,
            "--tiles" => ntiles = val("--tiles")?.parse().map_err(|_| "bad --tiles")?,
            "--groups" => ngroups = val("--groups")?.parse().map_err(|_| "bad --groups")?,
            "--ip" => ip = true,
            s => seed = Some(s.parse::<u64>().map_err(|_| format!("bad seed {s}"))?),
        }
    }
    let seed = seed.ok_or("usage: fuzz_fasm [--db DIR] [--device D] [--tiles N] [--groups N] [--ip] SEED")?;
    let ddb = DeviceDb::open(&db, &device).map_err(|e| e.to_string())?;
    let chip = Chip::new(&ddb, "").map_err(|e| e.to_string())?;
    let mut rng = Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1);
    let mut out = String::new();
    writeln!(out, "{{ oxide.device=\"{device}\" }}").ok();
    for m in 0..rng.below(3) {
        writeln!(out, "{{ oxide.meta=\"oxibit fuzz seed {seed} part {m}\" }}").ok();
    }

    let live: Vec<usize> = (0..chip.tiles.len())
        .filter(|&i| {
            let t = &chip.tiles[i];
            let tb = ddb.tile_bits(&t.tiletype);
            plain(&t.name) && (!tb.pips.is_empty() || !tb.enums.is_empty() || !tb.words.is_empty())
        })
        .collect();
    for _ in 0..ntiles {
        let Some(&i) = rng.pick(&live) else { break };
        let t = &chip.tiles[i];
        let tb = ddb.tile_bits(&t.tiletype);
        let tn = f(&t.name);
        let sinks: Vec<&String> = tb.pips.keys().filter(|k| plain(k)).collect();
        for _ in 0..rng.below(6) {
            let Some(to) = rng.pick(&sinks) else { break };
            let srcs: Vec<&str> = tb.pips[*to].iter().map(|p| p.from_wire.as_str()).filter(|s| plain(s)).collect();
            if let Some(from) = rng.pick(&srcs) {
                writeln!(out, "{tn}.PIP.{}.{}", f(to), f(from)).ok();
            }
        }
        let fixed: Vec<(&String, &str)> = tb
            .conns
            .iter()
            .filter(|(k, _)| plain(k))
            .flat_map(|(k, cs)| cs.iter().map(move |c| (k, c.from_wire.as_str())))
            .filter(|(_, s)| plain(s))
            .collect();
        if rng.below(4) == 0 {
            if let Some((to, from)) = rng.pick(&fixed) {
                writeln!(out, "{tn}.PIP.{}.{}", f(to), f(from)).ok();
            }
        }
        let k = rng.below(4) + 1;
        enums_words(&mut out, &mut rng, &tn, &[tb], k);
        if rng.below(8) == 0 {
            let (fr, b) = (rng.below(t.cram.frames), rng.below(t.cram.bits));
            writeln!(out, "{tn}.UNKNOWN.{fr}.{b}").ok();
        }
    }

    let mut groups: Vec<&String> = chip.tilegroups.keys().filter(|g| *g != "GLOBAL").collect();
    groups.sort();
    for _ in 0..ngroups {
        let Some(g) = rng.pick(&groups) else { break };
        let tbs: Vec<&TileBits> = chip.tilegroups[*g].iter().map(|&i| ddb.tile_bits(&chip.tiles[i].tiletype)).collect();
        enums_words(&mut out, &mut rng, g, &tbs, 2);
    }

    if ip {
        let regions: Vec<&String> = ddb.baseaddrs.keys().collect();
        let of = |p: &str| regions.iter().filter(|r| r.starts_with(p)).cloned().collect::<Vec<_>>();
        let ebr = ddb.ip_bits("EBR_INIT").map_err(|e| e.to_string())?;
        for _ in 0..rng.below(3) + 1 {
            let wid = rng.below(64);
            enums_words(&mut out, &mut rng, &format!("IP_EBR_WID{wid}"), &[ebr], 2);
        }
        let pll = ddb.ip_bits("PLL_CORE").map_err(|e| e.to_string())?;
        if let Some(r) = rng.pick(&of("PLL_")) {
            enums_words(&mut out, &mut rng, &format!("IP_{r}"), &[pll], 4);
        }
        let dphy = ddb.ip_bits("DPHY_CORE").map_err(|e| e.to_string())?;
        if rng.below(2) == 0 {
            if let Some(r) = rng.pick(&of("TDPHY_")) {
                enums_words(&mut out, &mut rng, &format!("IP_{r}"), &[dphy], 3);
            }
        }
        let lram = ddb.ip_bits("LRAM_INIT").map_err(|e| e.to_string())?;
        if rng.below(2) == 0 {
            if let (Some(r), Some(w)) = (rng.pick(&of("LRAM_")), lram.words.get("INITVAL_00")) {
                for _ in 0..rng.below(3) + 1 {
                    let n = rng.below(0x80);
                    writeln!(out, "IP_{r}.INITVAL_{n:02X}{}", word_value(&mut rng, w.bits.len())).ok();
                }
            }
        }
    }
    print!("{out}");
    Ok(())
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("fuzz_fasm: {e}");
            ExitCode::FAILURE
        }
    }
}
