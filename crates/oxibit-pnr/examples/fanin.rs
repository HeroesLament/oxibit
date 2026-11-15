//! What can drive a routing node: its in-edges, back a few levels, with
//! each edge's kind. For finding out why a sink cannot be reached.
//!
//! cargo run --release -p oxibit-pnr --example fanin -- DB_ROOT PART DEPTH NODE...
//!
//! NODE is `TILE/WIRE`. One line per edge, indented by level:
//! `<- SRC_NODE (kind, pip)`.

use std::path::Path;

use oxibit_fasm::db::Database;
use oxibit_fasm::route::{Graph, GraphOpts};

fn show(g: &Graph, n: u32, depth: u32, level: u32) {
    for (m, tile, pi) in g.in_edges(n) {
        let pad = "  ".repeat(level as usize + 1);
        println!("{pad}<- {} ({:?}, {})", g.node_name(m), g.edge_kind(tile, pi), g.edge_pip(tile, pi).name());
        if level + 1 < depth {
            show(g, m, depth, level + 1);
        }
    }
}

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().collect();
    let [_, db_root, part, depth, names @ ..] = &args[..] else {
        return Err("usage: fanin DB_ROOT PART DEPTH NODE...".into());
    };
    let depth: u32 = depth.parse().map_err(|e| format!("DEPTH: {e}"))?;
    let db = Database::open(Path::new(db_root), part)?;
    let g = Graph::build_with(&db, None, GraphOpts { defaults: true })?;
    for n in 0..g.nodes() as u32 {
        let name = g.node_name(n).to_string();
        if names.contains(&name) {
            println!("{name}");
            show(&g, n, depth, 0);
        }
    }
    Ok(())
}
