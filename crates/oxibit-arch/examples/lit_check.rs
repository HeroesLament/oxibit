//! Compile a generated tile type as Rust and compare it with the blob.
//!
//! `cargo run --release -p oxibit-arch --example lit_check -- <blob>`

use oxibit_arch::lit::Bit;
use oxibit_arch::model::{Fuse, NO_DEFAULT};
use oxibit_arch::codec;

#[path = "generated/plc2.rs"]
mod plc2;

fn same(lit: &[Bit], raw: &[u32]) -> bool {
    lit.len() == raw.len()
        && lit.iter().zip(raw).all(|(b, &r)| {
            let x = Fuse::from_raw(r);
            x.frame() == usize::from(b.frame) && x.bit() == usize::from(b.bit) && x.inverted() == b.inverted
        })
}

fn main() -> Result<(), String> {
    let blob = std::env::args().nth(1).ok_or("usage: lit_check <blob>")?;
    let chip = codec::decode(&std::fs::read(&blob).map_err(|e| format!("{blob}: {e}"))?)?;
    let lit = &plc2::TILE;
    let ty = chip.tile_type(lit.name).ok_or("tile type not in the blob")?;
    let (t, s) = (&chip.tile_types[ty], &chip.strings);
    let mut bad = 0;

    bad += usize::from(lit.muxes.len() != t.muxes.keys.len());
    for (k, m) in lit.muxes.iter().enumerate() {
        bad += usize::from(s.get(t.muxes.keys[k]) != m.sink);
        let items: Vec<_> = t.muxes.items(k).collect();
        bad += usize::from(items.len() != m.arcs.len());
        for (a, (src, fuses)) in m.arcs.iter().zip(items) {
            bad += usize::from(s.get(src) != a.source || !same(a.bits, fuses));
        }
    }
    bad += usize::from(lit.enums.len() != t.enums.keys.len());
    for (k, e) in lit.enums.iter().enumerate() {
        let def = (t.enum_def[k] != NO_DEFAULT).then(|| s.get(t.enum_def[k]));
        bad += usize::from(s.get(t.enums.keys[k]) != e.name || def != e.default);
        let items: Vec<_> = t.enums.items(k).collect();
        bad += usize::from(items.len() != e.options.len());
        for ((name, bits), (o, fuses)) in e.options.iter().zip(items) {
            bad += usize::from(s.get(o) != *name || !same(bits, fuses));
        }
    }
    bad += usize::from(lit.words.len() != t.words.keys.len());
    for (k, w) in lit.words.iter().enumerate() {
        bad += usize::from(s.get(t.words.keys[k]) != w.name);
        let def = &t.word_def[t.word_def_start[k] as usize..t.word_def_start[k + 1] as usize];
        bad += usize::from(def.iter().map(|&b| b != 0).collect::<Vec<_>>() != w.default);
        let items: Vec<_> = t.words.items(k).collect();
        bad += usize::from(items.len() != w.bits.len());
        for (bits, (_, fuses)) in w.bits.iter().zip(items) {
            bad += usize::from(!same(bits, fuses));
        }
    }
    let fixed: Vec<(&str, &str)> = t.fixed.chunks_exact(2).map(|p| (s.get(p[0]), s.get(p[1]))).collect();
    bad += usize::from(fixed != lit.fixed);

    println!(
        "{}: {} muxes, {} enums, {} words, {} fixed",
        lit.name,
        lit.muxes.len(),
        lit.enums.len(),
        lit.words.len(),
        lit.fixed.len()
    );
    if bad == 0 {
        println!("literal form matches the blob");
        Ok(())
    } else {
        Err(format!("{bad} mismatches between the literal form and the blob"))
    }
}
