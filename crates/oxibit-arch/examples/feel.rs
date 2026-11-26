//! Import a Trellis tiledata tree, write the blob, read it back, compare.
//!
//! `cargo run --release -p oxibit-arch --example feel -- <trellis database> [out.oxarch]`

use std::path::PathBuf;
use std::time::Instant;

use oxibit_arch::codec;
use oxibit_arch::import::{add_device, enum_lists, from_tile_bits, mux_lists, read_tiledata};
use oxibit_arch::model::{Chip, Fuse, Groups, TileType, NO_DEFAULT};
use oxibit_ecp5::db::{BitGroup, DeviceDb, TileBits};

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

fn extra(chip: &Chip, tt: &TileType, tb: &TileBits) -> usize {
    let s = &chip.strings;
    let mut bad = 0;
    for (k, e) in tb.enums.values().enumerate() {
        let got = tt.enum_def.get(k).copied().filter(|d| *d != NO_DEFAULT).map(|d| s.get(d));
        bad += usize::from(e.defval.as_deref() != got);
    }
    for (k, (name, w)) in tb.words.iter().enumerate() {
        let Some(key) = tt.words.find(s, name) else {
            bad += 1;
            continue;
        };
        let items: Vec<_> = tt.words.items(key).collect();
        if key != k || items.len() != w.bits.len() {
            bad += 1;
            continue;
        }
        for (n, (bits, (gn, gf))) in w.bits.iter().zip(items).enumerate() {
            let f: Vec<u32> = bits.0.iter().filter_map(|b| Fuse::new(b.frame, b.bit, b.inv)).map(Fuse::raw).collect();
            bad += usize::from(s.get(gn) != n.to_string() || f != gf);
        }
        let d = &tt.word_def[tt.word_def_start[k] as usize..tt.word_def_start[k + 1] as usize];
        let want: Vec<u8> = w.defval.iter().map(|b| u8::from(*b)).collect();
        bad += usize::from(d != want.as_slice());
    }
    let got: Vec<(&str, &str)> = tt.fixed.chunks_exact(2).map(|p| (s.get(p[0]), s.get(p[1]))).collect();
    let want: Vec<(&str, &str)> = tb.fixed_conns.iter().map(|(a, b)| (a.as_str(), b.as_str())).collect();
    bad + usize::from(got != want)
}

fn main() -> Result<(), String> {
    let mut args = std::env::args().skip(1);
    let root = PathBuf::from(args.next().ok_or("usage: feel <trellis database> [out]")?);
    let out = args.next().unwrap_or_else(|| "/tmp/ecp5.oxarch".to_owned());

    let t = Instant::now();
    let (tiles, text_bytes) = read_tiledata(&root, "ECP5")?;
    let t_text = t.elapsed();

    let t = Instant::now();
    let mut chip = from_tile_bits(&tiles)?;
    let devs: Vec<DeviceDb> = ["LFE5U-25F", "LFE5U-85F"]
        .iter()
        .map(|d| DeviceDb::load(&root, d).map_err(|e| e.to_string()))
        .collect::<Result<_, _>>()?;
    for db in &devs {
        add_device(&mut chip, db)?;
    }
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
        bad += extra(&back, &back.tile_types[i], tb);
    }
    let words: usize = back.tile_types.iter().map(|t| t.words.keys.len()).sum();
    let fixed: usize = back.tile_types.iter().map(|t| t.fixed.len() / 2).sum();
    println!("words {words}, enums {}, fixed connections {fixed}", back.tile_types.iter().map(|t| t.enums.keys.len()).sum::<usize>());

    for db in &devs {
        let Some(d) = back.device(&db.info.name).map(|i| &back.devices[i]) else {
            bad += 1;
            continue;
        };
        let i = &db.info;
        let geom = [i.idcode, i.frames as u32, i.bits_per_frame as u32, i.max_row as u32, i.max_col as u32];
        if geom != [d.idcode, d.frames, d.bits_per_frame, d.max_row, d.max_col] || d.tiles.len() != db.tiles.len() {
            bad += 1;
        }
        for (name, ti) in &db.tiles {
            let ok = d.tile(&back.strings, name).is_some_and(|t| {
                back.strings.get(back.tile_types[t.ty as usize].name) == ti.ty
                    && [t.start_frame, t.start_bit, t.frames, t.bits]
                        == [ti.start_frame as u32, ti.start_bit as u32, ti.frames as u32, ti.bits as u32]
            });
            bad += usize::from(!ok);
        }
        println!("device {:<10} {} tiles, idcode {:08x}, {} frames x {} bits", db.info.name, d.tiles.len(), d.idcode, d.frames, d.bits_per_frame);
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
