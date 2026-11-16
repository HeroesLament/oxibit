//! Where each PS wire meets the general interconnect: for a placer, which
//! otherwise has every PS pin at the PS site's one tile, tens of tiles from
//! where the pin's net has to arrive.
//!
//! cargo run --release -p oxibit-pnr --example ps_entries -- DB_ROOT PART OUT.JSON [TILE_PREFIX]
//!
//! TILE_PREFIX (default `PSS`) picks the nodes by the start of their name:
//! `BRAM_L_X22Y50` does the same for one block RAM tile, to see how far its
//! pins are from the tile.
//!
//! OUT.JSON maps each routing node in a PSS tile to `{"in": [x, y], "out":
//! [x, y]}`, either possibly null: the grid tile of the interconnect-side
//! node of the nearest interconnect pip, going against the edges (`in`,
//! for a PS input) or along them (`out`, for a PS output). Node names are
//! `TILE/WIRE`; a PS7 pin is the wire of that name on its PSS tile.

use std::collections::{BTreeMap, HashSet, VecDeque};
use std::path::Path;

use oxibit_fasm::db::Database;
use oxibit_fasm::route::{Graph, GraphOpts};
use serde_json::json;

/// Edges followed from a PS wire before giving up.
const DEPTH: u32 = 12;

/// The tile of the first interconnect pip met from `start`, breadth first,
/// against the edges (`back`) or along them.
fn walk(g: &Graph, start: u32, back: bool) -> Option<[i16; 2]> {
    let mut seen: HashSet<u32> = HashSet::from([start]);
    let mut queue: VecDeque<(u32, u32)> = VecDeque::from([(start, 0)]);
    while let Some((n, depth)) = queue.pop_front() {
        let mut next: Vec<u32> = Vec::new();
        let mut found = None;
        let mut step = |m: u32, tile: u32, pi: u32| {
            if found.is_none() && g.edge_pip(tile, pi).tile.starts_with("INT") {
                // The node on the far side of the pip: a PS wire's own
                // node runs the length of its PSS tile and sits at the
                // tile's anchor, not at the row it is entered on.
                found = Some(g.node_xy(m));
            }
            next.push(m);
        };
        if back {
            for (m, tile, pi) in g.in_edges(n) {
                step(m, tile, pi);
            }
        } else {
            for &(m, tile, pi) in g.out_edges(n) {
                step(m, tile, pi);
            }
        }
        if let Some((x, y)) = found {
            return Some([x, y]);
        }
        if depth < DEPTH {
            for m in next {
                if seen.insert(m) {
                    queue.push_back((m, depth + 1));
                }
            }
        }
    }
    None
}

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().collect();
    let (db_root, part, out, prefix) = match &args[..] {
        [_, a, b, c] => (a, b, c, "PSS"),
        [_, a, b, c, p] => (a, b, c, p.as_str()),
        _ => return Err("usage: ps_entries DB_ROOT PART OUT.JSON [TILE_PREFIX]".into()),
    };
    let db = Database::open(Path::new(db_root), part)?;
    let g = Graph::build_with(&db, None, GraphOpts::default())?;
    let mut map: BTreeMap<String, serde_json::Value> = BTreeMap::new();
    let (mut ins, mut outs) = (0usize, 0usize);
    for n in 0..g.nodes() as u32 {
        let name = g.node_name(n).to_string();
        if !name.starts_with(prefix) {
            continue;
        }
        let (i, o) = (walk(&g, n, true), walk(&g, n, false));
        ins += usize::from(i.is_some());
        outs += usize::from(o.is_some());
        if i.is_some() || o.is_some() {
            map.insert(name, json!({"in": i, "out": o}));
        }
    }
    std::fs::write(out, serde_json::to_string(&map).map_err(|e| e.to_string())?).map_err(|e| format!("{out}: {e}"))?;
    println!("{}", json!({"ps_wires": map.len(), "with_in": ins, "with_out": outs}));
    Ok(())
}
