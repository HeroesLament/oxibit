//! Random Trellis configurations for differential testing against `ecppack`.
//!
//!   cargo run --release -p oxibit-ecp5 --example fuzz_config -- \
//!       [--db <trellis/database>] [--tiles N] [--ebr N] DEVICE SEED out.config
//!
//! Picks N random tiles; in each, random mux arcs, random word values and
//! random enum options (`BASE_` ones included), from the tile type's own bit
//! database, so every setting is one `ecppack` accepts. Adds one tile group
//! over two tiles of a random type and N EBR initialisations of random
//! 9-bit words. Deterministic in SEED.

#![expect(
    clippy::print_stderr,
    reason = "a command-line example reports its errors on stderr"
)]
use std::collections::BTreeSet;
use std::fmt::Write as _;
use std::path::PathBuf;
use std::process::ExitCode;

use oxibit_ecp5::DeviceDb;

/// xorshift64*: enough randomness for test vectors, no dependency.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    fn below(&mut self, n: usize) -> usize {
        // Modulo bias is irrelevant for test vectors.
        usize::try_from(self.next() % u64::try_from(n.max(1)).expect("usize fits u64"))
            .expect("below n fits usize")
    }

    fn pick<'a, T>(&mut self, items: &'a [T]) -> Option<&'a T> {
        if items.is_empty() {
            None
        } else {
            Some(&items[self.below(items.len())])
        }
    }
}

fn default_db() -> PathBuf {
    if let Some(p) = std::env::var_os("OXIBIT_TRELLIS_DB") {
        return PathBuf::from(p);
    }
    let home = std::env::var_os("HOME")
        .map(PathBuf::from)
        .unwrap_or_default();
    home.join("oss-cad-suite/share/trellis/database")
}

fn bitvec(rng: &mut Rng, n: usize) -> String {
    (0..n)
        .map(|_| if rng.next() & 1 == 1 { '1' } else { '0' })
        .collect()
}

fn run() -> Result<(), String> {
    let mut db = default_db();
    let mut n_tiles = 40usize;
    let mut n_ebr = 2usize;
    let mut pos = Vec::new();
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        let mut val = |flag: &str| args.next().ok_or_else(|| format!("{flag} needs a value"));
        match a.as_str() {
            "--db" => db = PathBuf::from(val("--db")?),
            "--tiles" => {
                n_tiles = val("--tiles")?
                    .parse()
                    .map_err(|e| format!("--tiles: {e}"))?;
            }
            "--ebr" => n_ebr = val("--ebr")?.parse().map_err(|e| format!("--ebr: {e}"))?,
            _ => pos.push(a),
        }
    }
    let [device, seed, out] = pos.as_slice() else {
        return Err(
            "usage: fuzz_config [--db DIR] [--tiles N] [--ebr N] DEVICE SEED out.config".into(),
        );
    };
    let seed: u64 = seed.parse().map_err(|e| format!("seed: {e}"))?;
    let ddb = DeviceDb::load(&db, device).map_err(|e| e.to_string())?;
    let mut rng = Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1);
    let names: Vec<&String> = ddb.tiles.keys().collect();

    let mut cfg = format!(".device {device}\n.comment oxibit fuzz_config seed {seed}\n\n");
    let mut chosen = BTreeSet::new();
    for _ in 0..n_tiles {
        let Some(&name) = rng.pick(&names) else { break };
        chosen.insert(name.clone());
    }
    // IO tiles carry the `<site>.BASE_TYPE` enums (ECP5 has no top-level
    // `BASE_` enum, so libtrellis applies them in the ordinary enum phase);
    // a uniform pick almost never lands on an IO tile, so add some.
    let base_tiles: Vec<&String> = names
        .iter()
        .copied()
        .filter(|n| {
            ddb.tile_bits(&ddb.tiles[*n].ty)
                .is_ok_and(|b| b.enums.keys().any(|k| k.contains("BASE_")))
        })
        .collect();
    for _ in 0..n_tiles / 4 {
        let Some(&name) = rng.pick(&base_tiles) else {
            break;
        };
        chosen.insert(name.clone());
    }
    for name in &chosen {
        let ty = &ddb.tiles[name].ty;
        let bits = ddb.tile_bits(ty).map_err(|e| e.to_string())?;
        let _ = writeln!(cfg, ".tile {name}");
        let sinks: Vec<_> = bits.muxes.iter().collect();
        for _ in 0..rng.below(4) {
            if let Some((sink, mux)) = rng.pick(&sinks) {
                let sources: Vec<_> = mux.arcs.keys().collect();
                if let Some(src) = rng.pick(&sources) {
                    let _ = writeln!(cfg, "arc: {sink} {src}");
                }
            }
        }
        let words: Vec<_> = bits.words.iter().collect();
        for _ in 0..rng.below(3) {
            if let Some((name, w)) = rng.pick(&words) {
                let _ = writeln!(cfg, "word: {name} {}", bitvec(&mut rng, w.bits.len()));
            }
        }
        let enums: Vec<_> = bits.enums.iter().collect();
        for _ in 0..rng.below(3) {
            if let Some((name, e)) = rng.pick(&enums) {
                let opts: Vec<_> = e.options.keys().collect();
                if let Some(o) = rng.pick(&opts) {
                    let _ = writeln!(cfg, "enum: {name} {o}");
                }
            }
        }
        let base: Vec<_> = bits
            .enums
            .iter()
            .filter(|(k, _)| k.contains("BASE_"))
            .collect();
        if let Some((name, e)) = rng.pick(&base) {
            let opts: Vec<_> = e.options.keys().collect();
            if let Some(o) = rng.pick(&opts) {
                let _ = writeln!(cfg, "enum: {name} {o}");
            }
        }
        cfg.push('\n');
    }

    // A tile group: two tiles of one type, one of its enums and words.
    if let Some(&first) = rng.pick(&names) {
        let ty = &ddb.tiles[first].ty;
        let same: Vec<_> = names.iter().filter(|n| &ddb.tiles[**n].ty == ty).collect();
        let second = rng.pick(&same).map_or(first, |n| **n);
        let bits = ddb.tile_bits(ty).map_err(|e| e.to_string())?;
        let enums: Vec<_> = bits.enums.iter().collect();
        let words: Vec<_> = bits.words.iter().collect();
        let mut body = String::new();
        if let Some((name, e)) = rng.pick(&enums) {
            let opts: Vec<_> = e.options.keys().collect();
            if let Some(o) = rng.pick(&opts) {
                let _ = writeln!(body, "enum: {name} {o}");
            }
        }
        if let Some((name, w)) = rng.pick(&words) {
            let _ = writeln!(body, "word: {name} {}", bitvec(&mut rng, w.bits.len()));
        }
        if !body.is_empty() {
            let _ = writeln!(cfg, ".tile_group {first} {second}\n{body}");
        }
    }

    for _ in 0..n_ebr {
        let _ = writeln!(cfg, ".bram_init {}", rng.below(208));
        for i in 0..2048 {
            let _ = write!(
                cfg,
                "{:03x}{}",
                rng.next() & 0x1FF,
                if i % 16 == 15 { '\n' } else { ' ' }
            );
        }
        cfg.push('\n');
    }
    std::fs::write(out, cfg).map_err(|e| format!("{out}: {e}"))
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("fuzz_config: {e}");
            ExitCode::FAILURE
        }
    }
}
