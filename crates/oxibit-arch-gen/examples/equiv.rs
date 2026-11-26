//! Rebuild oxibit-ecp5's `DeviceDb` from a blob and compare it, field for
//! field, with `DeviceDb::load` on the Trellis text. `graph.rs` builds the
//! routing graph from a `DeviceDb`, so equal inputs give an identical graph.
//!
//! `cargo run --release -p oxibit-arch --example equiv -- <trellis database> <blob>`

use std::path::PathBuf;

use oxibit_arch::codec;
use oxibit_arch_gen::export::to_device_db;
use oxibit_ecp5::db::DeviceDb;

fn main() -> Result<(), String> {
    let mut args = std::env::args().skip(1);
    let root = PathBuf::from(args.next().ok_or("usage: equiv <trellis database> <blob>")?);
    let blob = args.next().ok_or("usage: equiv <trellis database> <blob>")?;
    let chip = codec::decode(&std::fs::read(&blob).map_err(|e| format!("{blob}: {e}"))?)?;
    let mut failed = false;
    for d in &chip.devices {
        let name = chip.strings.get(d.name).to_owned();
        let want = DeviceDb::load(&root, &name).map_err(|e| e.to_string())?;
        let i = chip.device(&name).ok_or("device missing")?;
        let got = to_device_db(&chip, i, root.clone())?;
        let (info, tiles, bits) = (got.info == want.info, got.tiles == want.tiles, got.bits == want.bits);
        println!(
            "{name:<10} info {}  tiles {} ({})  tile types {} ({})",
            if info { "equal" } else { "DIFFER" },
            if tiles { "equal" } else { "DIFFER" },
            want.tiles.len(),
            if bits { "equal" } else { "DIFFER" },
            want.bits.len(),
        );
        failed |= !(info && tiles && bits);
    }
    if failed {
        Err("rebuilt DeviceDb differs from DeviceDb::load".to_owned())
    } else {
        Ok(())
    }
}
