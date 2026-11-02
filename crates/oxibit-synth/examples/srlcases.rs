//! Write flop-chain test cases for proving the SRLC32E mapping.
//!
//! cargo run --release -p oxibit-synth --example srlcases -- OUTDIR
//!
//! Each case OUTDIR/caseK/ holds ref.json (a chain of `oxsyn_` FDRE / FDSE
//! stages with random INIT, a CE port, optionally an inverted clock and a
//! tapped stage) and dut.json (the same module after `srl::map`).
//! models/srl_shapes.sh proves each pair equivalent with yosys.

use std::collections::BTreeMap;
use std::process::ExitCode;

use oxibit_hdl::netlist::{Bit, Cell, Dir, Module, Param, SigSpec, Signal};
use oxibit_hdl::yosys_json;
use oxibit_synth::srl;

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
}

fn port(m: &mut Module, name: &str, dir: Dir) -> Bit {
    let id = m.add_signal(Signal {
        name: name.into(),
        width: 1,
        signed: false,
        port: Some(dir),
        init: None,
    });
    SigSpec::of(id, 1).0[0]
}

fn case(n: usize, clk_inv: bool, tap: usize, rng: &mut Rng) -> Module {
    let mut m = Module::new("top");
    let clk = port(&mut m, "clk", Dir::Input);
    let ce = port(&mut m, "ce", Dir::Input);
    let din = port(&mut m, "d", Dir::Input);
    let q = port(&mut m, "q", Dir::Output);
    let t = port(&mut m, "t", Dir::Output);
    let w = m.wire(n as u32);
    let mut d = din;
    for k in 0..n {
        let o = if k == n - 1 { q } else { w.0[k] };
        let set = rng.next() % 4 == 0;
        let init = rng.next() % 2 == 1;
        let (ty, srp) = if set { ("FDSE", "S") } else { ("FDRE", "R") };
        let mut params = BTreeMap::from([("INIT".to_string(), Param::Bits(vec![Some(init)]))]);
        if clk_inv {
            params.insert("IS_C_INVERTED".into(), Param::Bits(vec![Some(true)]));
        }
        m.cells.push(Cell::Instance {
            ty: ty.into(),
            name: format!("oxsyn_ff{k}$r"),
            params,
            conns: BTreeMap::from([
                ("C".to_string(), SigSpec(vec![clk])),
                ("CE".to_string(), SigSpec(vec![ce])),
                (srp.to_string(), SigSpec(vec![Bit::Zero])),
                ("D".to_string(), SigSpec(vec![d])),
                ("Q".to_string(), SigSpec(vec![o])),
            ]),
        });
        d = o;
    }
    let tb = if tap > 0 { w.0[tap - 1] } else { Bit::Zero };
    m.connects.push((SigSpec(vec![t]), SigSpec(vec![tb])));
    m
}

fn main() -> ExitCode {
    let Some(out) = std::env::args().nth(1) else {
        eprintln!("usage: srlcases OUTDIR");
        return ExitCode::from(3);
    };
    let mut rng = Rng(0x2545_f491_4f6c_dd1d);
    let mut shapes: Vec<(usize, bool, usize)> = (2..=36).map(|n| (n, false, 0)).collect();
    shapes.extend([
        (63, false, 0),
        (64, false, 0),
        (65, false, 0),
        (66, false, 0),
        (67, false, 0),
        (100, false, 0),
    ]);
    shapes.extend([(5, true, 0), (40, true, 0), (12, false, 4), (40, false, 33), (40, true, 1)]);
    for (k, (n, inv, tap)) in shapes.into_iter().enumerate() {
        let m = case(n, inv, tap, &mut rng);
        let mut dut = m.clone();
        let st = srl::map(&mut dut);
        let dir = format!("{out}/case{k:03}");
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(format!("{dir}/ref.json"), serde_json::to_string(&yosys_json::write(&m)).unwrap()).unwrap();
        std::fs::write(format!("{dir}/dut.json"), serde_json::to_string(&yosys_json::write(&dut)).unwrap()).unwrap();
        std::fs::write(format!("{dir}/case.txt"), format!("{n} {} {tap}\n", u8::from(inv))).unwrap();
        println!("case{k:03} n={n} clk_inv={inv} tap={tap} srlc32e={} ffs={}", st.srlc32e, st.ffs);
    }
    ExitCode::SUCCESS
}
