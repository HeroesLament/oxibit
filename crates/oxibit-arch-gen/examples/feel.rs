//! Import a Trellis tiledata tree, write the blob, read it back, compare.
//!
//! `cargo run --release -p oxibit-arch --example feel -- <trellis database> [out.oxarch]`

use std::path::{Path, PathBuf};
use std::time::Instant;

use serde_json::Value;

use oxibit_arch::codec;
use oxibit_arch_gen::import::{add_device, enum_lists, from_tile_bits, mux_lists, read_tiledata};
use oxibit_arch::model::{Chip, Device, Fuse, Groups, TileType, NONE, NO_DEFAULT};
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

fn json(p: &Path) -> Value {
    serde_json::from_str(&std::fs::read_to_string(p).unwrap_or_default()).unwrap_or(Value::Null)
}

fn n(v: &Value, k: &str) -> u64 {
    v[k].as_u64().unwrap_or(u64::MAX)
}

fn count(v: &Value) -> usize {
    v.as_object().map_or(usize::MAX, serde_json::Map::len)
}

/// Re-reads globals.json and iodb.json and compares them to the device.
fn device_data(chip: &Chip, d: &Device, dir: &Path) -> usize {
    let s = &chip.strings;
    let (g, io) = (json(&dir.join("globals.json")), json(&dir.join("iodb.json")));
    let mut bad = usize::from(count(&g["quadrants"]) != d.quadrants.len())
        + usize::from(count(&g["taps"]) != d.taps.len())
        + usize::from(count(&g["spines"]) != d.spines.len())
        + usize::from(count(&io["packages"]) != d.packages.len());
    for q in &d.quadrants {
        let v = &g["quadrants"][s.get(q.name)];
        bad += usize::from([q.x0, q.y0, q.x1, q.y1].map(u64::from) != [n(v, "x0"), n(v, "y0"), n(v, "x1"), n(v, "y1")]);
    }
    for t in &d.taps {
        let v = &g["taps"][s.get(t.name)];
        bad += usize::from([t.lx0, t.lx1, t.rx0, t.rx1].map(u64::from) != [n(v, "lx0"), n(v, "lx1"), n(v, "rx0"), n(v, "rx1")]);
    }
    for p in &d.spines {
        let v = &g["spines"][s.get(p.name)];
        bad += usize::from([p.x, p.y].map(u64::from) != [n(v, "x"), n(v, "y")]);
    }
    for p in &d.packages {
        let m = &io["packages"][s.get(p.name)];
        bad += usize::from(count(m) != p.pins.len());
        for pin in &p.pins {
            let v = &m[s.get(pin.name)];
            let same = [pin.col, pin.row].map(u64::from) == [n(v, "col"), n(v, "row")] && v["pio"].as_str() == Some(s.get(pin.pio));
            bad += usize::from(!same);
        }
    }
    let arr = io["pio_metadata"].as_array().map_or(&[][..], Vec::as_slice);
    bad += usize::from(arr.len() != d.pio_meta.len());
    for (m, v) in d.pio_meta.iter().zip(arr) {
        let opt = |id: u32| (id != NONE).then(|| s.get(id));
        let same = [m.bank, m.col, m.row].map(u64::from) == [n(v, "bank"), n(v, "col"), n(v, "row")]
            && v["pio"].as_str() == Some(s.get(m.pio))
            && v["dqs"].as_str() == opt(m.dqs)
            && v["function"].as_str() == opt(m.function);
        bad += usize::from(!same);
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
        bad += device_data(&back, d, &db.root.join(&i.family).join(&i.name));
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
