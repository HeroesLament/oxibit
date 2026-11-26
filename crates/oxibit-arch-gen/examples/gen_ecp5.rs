//! Generate `oxibit-arch-ecp5`'s tile type and device modules from a Trellis
//! database, or with `--check` compare what is checked in with a fresh
//! generation.
//!
//! `cargo run --release -p oxibit-arch-gen --example gen_ecp5 -- <trellis database> <crate dir> [--check]`

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use oxibit_arch::emit;
use oxibit_arch::model::Chip;
use oxibit_arch_gen::import::{add_device, from_tile_bits, read_tiledata};
use oxibit_ecp5::db::DeviceDb;

const DEVICES: [&str; 2] = ["LFE5U-25F", "LFE5U-85F"];
const GENERATED_DIRS: [&str; 2] = ["src/tiles", "src/devices"];

/// Every generated file: path relative to the crate, and its text.
fn generate(chip: &Chip) -> BTreeMap<String, String> {
    let mut out = BTreeMap::new();
    for (i, t) in chip.tile_types.iter().enumerate() {
        let name = emit::module_name(chip.strings.get(t.name));
        out.insert(format!("src/tiles/{name}.rs"), emit::tile_type_source(chip, i));
    }
    out.insert("src/tiles/mod.rs".to_owned(), emit::tiles_mod_source(chip));
    for (i, d) in chip.devices.iter().enumerate() {
        let name = emit::module_name(chip.strings.get(d.name));
        out.insert(format!("src/devices/{name}.rs"), emit::device_source(chip, i));
    }
    out.insert("src/devices/mod.rs".to_owned(), emit::devices_mod_source(chip));
    out
}

fn on_disk(dir: &Path) -> Vec<String> {
    let mut found = Vec::new();
    for sub in GENERATED_DIRS {
        if let Ok(rd) = std::fs::read_dir(dir.join(sub)) {
            for e in rd.flatten() {
                found.push(format!("{sub}/{}", e.file_name().to_string_lossy()));
            }
        }
    }
    found.sort();
    found
}

fn main() -> Result<(), String> {
    let mut args = std::env::args().skip(1);
    let root = PathBuf::from(args.next().ok_or("usage: gen_ecp5 <trellis database> <crate dir> [--check]")?);
    let dir = PathBuf::from(args.next().ok_or("usage: gen_ecp5 <trellis database> <crate dir> [--check]")?);
    let check = args.next().as_deref() == Some("--check");

    let (tiles, _) = read_tiledata(&root, "ECP5")?;
    let mut chip = from_tile_bits(&tiles)?;
    for name in DEVICES {
        let db = DeviceDb::load(&root, name).map_err(|e| e.to_string())?;
        add_device(&mut chip, &db)?;
    }
    let want = generate(&chip);

    if check {
        let mut bad = 0;
        for (path, text) in &want {
            if std::fs::read_to_string(dir.join(path)).ok().as_deref() != Some(text.as_str()) {
                println!("differs: {path}");
                bad += 1;
            }
        }
        for path in on_disk(&dir) {
            if !want.contains_key(&path) {
                println!("not generated: {path}");
                bad += 1;
            }
        }
        if bad > 0 {
            return Err(format!("{bad} files out of date; run without --check"));
        }
        println!("{} generated files are up to date", want.len());
        return Ok(());
    }

    for sub in GENERATED_DIRS {
        let p = dir.join(sub);
        if p.exists() {
            std::fs::remove_dir_all(&p).map_err(|e| format!("{}: {e}", p.display()))?;
        }
        std::fs::create_dir_all(&p).map_err(|e| format!("{}: {e}", p.display()))?;
    }
    let mut bytes = 0;
    for (path, text) in &want {
        std::fs::write(dir.join(path), text).map_err(|e| format!("{path}: {e}"))?;
        bytes += text.len();
    }
    println!("wrote {} files, {bytes} bytes, under {}", want.len(), dir.display());
    Ok(())
}
