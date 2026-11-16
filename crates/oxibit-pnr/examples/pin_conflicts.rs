//! Terminals two or more nets of a placed design claim (`placed::pin_conflicts`):
//! a source or fixed sink node no routing can give to both. Each is an
//! import fault or an illegal placement, and stays overused whatever the
//! router does.
//!
//! cargo run --release -p oxibit-pnr --example pin_conflicts -- \
//!     DB_ROOT PART META_DIR PLACED.json
//!
//! stdout: one JSON line: the count, the conflicts by wire kind (tile type
//! and wire with numbers dropped), and examples with the nets' names.

use std::collections::BTreeMap;
use std::path::Path;

use oxibit_fasm::db::Database;
use oxibit_fasm::route::{Graph, GraphOpts};
use oxibit_pnr::placed::{self, Meta, Perm};
use serde_json::json;

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().collect();
    let [_, db_root, part, meta_dir, design] = &args[..] else {
        return Err("usage: pin_conflicts DB_ROOT PART META_DIR PLACED.json".into());
    };
    let db = Database::open(Path::new(db_root), part)?;
    let g = Graph::build_with(&db, None, GraphOpts { defaults: true })?;
    let text = std::fs::read_to_string(design).map_err(|e| format!("{design}: {e}"))?;
    let placed_v: serde_json::Value = serde_json::from_str(&text).map_err(|e| format!("{design}: {e}"))?;
    let sites = placed::site_index(&db);
    let imp = placed::import(&g, &sites, &mut Meta::open(Path::new(meta_dir)), &placed_v, Perm::A5)?;
    let conflicts = placed::pin_conflicts(&imp.nets);

    // `TILE_X1Y2/WIRE_3` -> `TILE/WIRE_N`.
    let kind = |n: u32| -> String {
        let name = g.node_name(n);
        let (tile, wire) = name.split_once('/').unwrap_or((&name, ""));
        let tile = tile.split("_X").next().unwrap_or(tile);
        let wire: String = wire.chars().map(|c| if c.is_ascii_digit() { 'N' } else { c }).collect();
        format!("{tile}/{wire}")
    };
    let mut by_kind: BTreeMap<String, usize> = BTreeMap::new();
    for (n, _) in &conflicts {
        *by_kind.entry(kind(*n)).or_default() += 1;
    }
    let examples: Vec<serde_json::Value> = conflicts
        .iter()
        .take(12)
        .map(|(n, nets)| json!({"node": g.node_name(*n), "nets": nets.iter().map(|&i| imp.names.get(i).cloned().unwrap_or_default()).collect::<Vec<_>>()}))
        .collect();
    println!("{}", json!({"conflicts": conflicts.len(), "by_kind": by_kind, "examples": examples}));
    Ok(())
}
