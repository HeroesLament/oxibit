//! Write single-memory test cases for co-simulating the block RAM mapping.
//!
//! cargo run --release -p oxibit-synth --example memcases -- OUTDIR [N] [SEED]
//!
//! Each case OUTDIR/caseK/ holds ref.json (one `$mem_v2`, one read port,
//! synchronous for block RAM cases and asynchronous for distributed RAM
//! ones, one write port), dut.json (the same module after
//! `bram::map`) and case.txt ("width depth abits two_clocks sync_read"). Shapes cover
//! every block RAM shape the mapper can pick, plus random widths and depths;
//! contents are random with some words left uninitialised.

use std::collections::BTreeMap;
use std::process::ExitCode;

use oxibit_hdl::netlist::{Cell, Dir, MemRead, MemWrite, Memory, Module, SigSpec, Signal};
use oxibit_hdl::yosys_json;
use oxibit_synth::bram;

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
}

fn port(m: &mut Module, name: &str, width: u32, dir: Dir) -> SigSpec {
    let id = m.add_signal(Signal {
        name: name.into(),
        width,
        signed: false,
        port: Some(dir),
        init: None,
    });
    SigSpec::of(id, width)
}

fn case(width: u32, depth: u32, two_clocks: bool, sync: bool, rng: &mut Rng) -> (Module, u32) {
    let abits = (32 - (depth - 1).leading_zeros()).max(1);
    let mut m = Module::new("top");
    let rclk = port(&mut m, "rclk", 1, Dir::Input);
    let wclk = if two_clocks { port(&mut m, "wclk", 1, Dir::Input) } else { rclk.clone() };
    let raddr = port(&mut m, "raddr", abits, Dir::Input);
    let waddr = port(&mut m, "waddr", abits, Dir::Input);
    let wdata = port(&mut m, "wdata", width, Dir::Input);
    let we = port(&mut m, "we", 1, Dir::Input);
    let rdata = port(&mut m, "rdata", width, Dir::Output);
    let mut init = BTreeMap::new();
    for a in 0..depth {
        if rng.below(4) == 0 {
            continue;
        }
        init.insert(a, (0..width).map(|_| Some(rng.below(2) == 1)).collect());
    }
    let mem = Memory {
        name: "mem".into(),
        width,
        size: depth,
        offset: 0,
        init,
        attrs: BTreeMap::new(),
        reads: vec![MemRead {
            clk: if sync { Some((rclk, true)) } else { None },
            addr: raddr,
            data: rdata,
        }],
        writes: vec![MemWrite {
            clk: (wclk, true),
            addr: waddr,
            data: wdata,
            en: SigSpec(vec![we.0[0]; width as usize]),
        }],
    };
    m.cells.push(Cell::Mem(mem));
    (m, abits)
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    let Some(out) = args.get(1) else {
        eprintln!("usage: memcases OUTDIR [N] [SEED]");
        return ExitCode::from(3);
    };
    let n: usize = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(40);
    let mut rng = Rng(args.get(3).and_then(|s| s.parse().ok()).unwrap_or(0x9e37_79b9_7f4a_7c15));
    // One case per mapper shape first (width, depth at the shape's limit),
    // then random ones.
    // Asynchronous read (distributed RAM): (width, depth).
    let lut: Vec<(u32, u32)> = vec![
        (6, 32),
        (13, 32),
        (3, 64),
        (7, 64),
        (5, 128),
        (38, 4),
        (27, 16),
        (27, 64),
        (36, 128),
        (24, 128),
        (1, 2),
    ];
    let mut shapes: Vec<(u32, u32)> = vec![
        (1, 16384),
        (2, 8192),
        (4, 4096),
        (9, 2048),
        (18, 1024),
        (36, 512),
        (1, 32768),
        (2, 16384),
        (4, 8192),
        (9, 4096),
        (18, 2048),
        (36, 1024),
        (72, 512),
        (26, 1024),
        (64, 512),
        (26, 128),
        (216, 32),
        (89, 256),
        (32, 2048),
    ];
    while shapes.len() < n {
        let w = 1 + rng.below(96) as u32;
        let d = 2 + rng.below(4096) as u32;
        shapes.push((w, d));
    }
    let all = shapes.into_iter().take(n).map(|s| (s, true)).chain(lut.into_iter().map(|s| (s, false)));
    for (k, ((w, d), sync)) in all.enumerate() {
        let two = k % 3 == 2;
        let (m, abits) = case(w, d, two, sync, &mut rng);
        let mut dut = m.clone();
        let st = bram::map(&mut dut);
        let dir = format!("{out}/case{k:03}");
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(format!("{dir}/ref.json"), serde_json::to_string(&yosys_json::write(&m)).unwrap()).unwrap();
        std::fs::write(format!("{dir}/dut.json"), serde_json::to_string(&yosys_json::write(&dut)).unwrap()).unwrap();
        std::fs::write(
            format!("{dir}/case.txt"),
            format!("{w} {d} {abits} {} {}\n", u8::from(two), u8::from(sync)),
        )
        .unwrap();
        println!(
            "case{k:03} {w}x{d} sync={sync} two_clocks={two} ramb18={} ramb36={} ram32m={} ram64m={} ram128x1d={} left={:?}",
            st.ramb18, st.ramb36, st.ram32m, st.ram64m, st.ram128x1d, st.left
        );
    }
    ExitCode::SUCCESS
}
