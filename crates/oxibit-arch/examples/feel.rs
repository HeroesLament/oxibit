//! Import a Trellis tiledata tree, write the blob, read it back, compare.
//!
//! `cargo run --release -p oxibit-arch --example feel -- <trellis database> [out.oxarch]`

use std::path::PathBuf;
use std::time::Instant;

use oxibit_arch::import::{enum_lists, from_tile_bits, mux_lists, read_tiledata};
use oxibit_arch::model::{Chip, Fuse, Groups};
use oxibit_arch::codec;
use oxibit_ecp5::db::BitGroup;

fn mismatches(chip: &Chip, g: &Groups, want: &[(&String, Vec<(&String, &BitGroup)>)]) -> usize {
    let mut bad = 0;
    for (key, items) in want {
        let Some(k) = g.find(&chip.strings, key) else {
            bad += 1;
            continue;
        };
        let got: Vec<_> = g.items(k).collect();
        if got.len() != items.len() {
            bad += 1;
            continue;
        }
        for ((name, bits), (gn, gf)) in items.iter().zip(got) {
            let w: Vec<u32> = bits.0.iter().filter_map(|b| Fuse::new(b.frame, b.bit, b.inv)).map(Fuse::raw).collect();
            if name.as_str() != chip.strings.get(gn) || w != gf {
                bad += 1;
            }
        }
    }
    bad
}

fn main() -> Result<(), String> {
    let mut args = std::env::args().skip(1);
    let root = PathBuf::from(args.next().ok_or("usage: feel <trellis database> [out]")?);
    let out = args.next().unwrap_or_else(|| "/tmp/ecp5.oxarch".to_owned());

    let t = Instant::now();
    let (tiles, text_bytes) = read_tiledata(&root, "ECP5")?;
    let t_text = t.elapsed();

    let t = Instant::now();
    let chip = from_tile_bits(&tiles)?;
    let t_build = t.elapsed();

    let blob = codec::encode(&chip)?;
    std::fs::write(&out, &blob).map_err(|e| e.to_string())?;

    let t = Instant::now();
    let bytes = std::fs::read(&out).map_err(|e| e.to_string())?;
    let back = codec::decode(&bytes)?;
    let t_load = t.elapsed();

    let mut bad = usize::from(back != chip);
    for (i, tb) in tiles.values().enumerate() {
        bad += mismatches(&back, &back.tile_types[i].muxes, &mux_lists(tb));
        bad += mismatches(&back, &back.tile_types[i].enums, &enum_lists(tb));
    }

    let arcs: usize = back.tile_types.iter().map(|t| t.muxes.item_name.len()).sum();
    let fuses: usize = back.tile_types.iter().map(|t| t.muxes.fuses.len() + t.enums.fuses.len()).sum();
    println!("tile types      {}", back.tile_types.len());
    println!("strings         {} ({} bytes)", back.strings.len(), back.strings.text().len());
    println!("mux arcs        {arcs}");
    println!("fuses           {fuses}");
    println!("text            {text_bytes} bytes, read and parse {t_text:?}");
    println!("build tables    {t_build:?}");
    println!("blob            {} bytes ({:.1}% of text), read and decode {t_load:?}", blob.len(), 100.0 * blob.len() as f64 / text_bytes as f64);
    if let Some(ty) = back.tile_type("PLC2") {
        let m = &back.tile_types[ty].muxes;
        if let Some(k) = m.find(&back.strings, "A1") {
            let n = m.items(k).count();
            let (src, f) = m.items(k).next().ok_or("empty")?;
            let f: Vec<String> = f.iter().map(|&r| Fuse::from_raw(r)).map(|f| format!("F{}B{}", f.frame(), f.bit())).collect();
            println!("PLC2 mux A1     {n} arcs, first {} {}", back.strings.get(src), f.join(" "));
        }
    }
    if bad == 0 {
        println!("round trip      ok");
        Ok(())
    } else {
        Err(format!("round trip      {bad} mismatches"))
    }
}
