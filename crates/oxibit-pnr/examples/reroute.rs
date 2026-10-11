//! `oxibit pnr route` without the CLI: route a design again on its own
//! placement, progress on stderr.
//!
//! cargo run --release -p oxibit-pnr --example reroute -- DB_ROOT PART IN.FASM OUT.FASM

use std::path::Path;
use std::sync::atomic::AtomicBool;

use oxibit_fasm::db::Database;
use oxibit_fasm::route::{Graph, GraphOpts};
use oxibit_pnr::{reroute, router::Opts};
use oxibit_report::{Human, HumanStyle, Reporter};

static CANCEL: AtomicBool = AtomicBool::new(false);

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().collect();
    let [_, db_root, part, input, output] = &args[..] else {
        return Err("usage: reroute DB_ROOT PART IN.FASM OUT.FASM".into());
    };
    let r = Reporter::new(vec![Box::new(Human::new(std::io::stderr(), HumanStyle::Plain))], &CANCEL);
    let db = Database::open(Path::new(db_root), part)?;
    let g = Graph::build_with(&db, None, GraphOpts { defaults: true })?;
    let fasm = std::fs::read_to_string(input).map_err(|e| format!("{input}: {e}"))?;
    let res = reroute::run(&g, &fasm, Opts::default(), &r);
    std::fs::write(output, &res.fasm).map_err(|e| format!("{output}: {e}"))?;
    eprintln!("{}\n{}", res.stats_json(), res.check_json());
    Ok(())
}
