//! Write tile types from a blob as Rust source.
//!
//! `cargo run --release -p oxibit-arch --example emit_rs -- <blob> <outdir> [tile type ...]`

use std::path::PathBuf;

use oxibit_arch::{codec, emit};

fn main() -> Result<(), String> {
    let mut args = std::env::args().skip(1);
    let blob = args.next().ok_or("usage: emit_rs <blob> <outdir> [tile type ...]")?;
    let out = PathBuf::from(args.next().ok_or("usage: emit_rs <blob> <outdir> [tile type ...]")?);
    let mut names: Vec<String> = args.collect();
    if names.is_empty() {
        names.push("PLC2".to_owned());
    }
    let chip = codec::decode(&std::fs::read(&blob).map_err(|e| format!("{blob}: {e}"))?)?;
    std::fs::create_dir_all(&out).map_err(|e| e.to_string())?;
    for name in names {
        let ty = chip.tile_type(&name).ok_or_else(|| format!("no tile type {name}"))?;
        let src = emit::tile_type_source(&chip, ty);
        let path = out.join(format!("{}.rs", name.to_lowercase()));
        std::fs::write(&path, &src).map_err(|e| format!("{}: {e}", path.display()))?;
        println!("{}  {} bytes, {} lines", path.display(), src.len(), src.lines().count());
    }
    Ok(())
}
