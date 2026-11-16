use oxibit_fasm::db::Database;
use oxibit_fasm::route::{Graph, GraphOpts};

fn main() -> Result<(), String> {
    let db = Database::open("/Users/mac.w/src/openxc7/prjxray-db/zynq7".as_ref(), "xc7z020clg400-1")?;
    let g = Graph::build_with(&db, None, GraphOpts::default())?;
    for name in ["DSP_L_X14Y25/DSP_0_ACOUT0", "DSP_L_X14Y25/DSP_1_ACIN0", "DSP_L_X14Y25/DSP_ACOUT0", "DSP_L_X14Y15/DSP_0_ACIN0", "DSP_L_X14Y15/DSP_ACOUT0"] {
        let n = g.node(name)?;
        println!("{name} = node {n} (name: {})", g.node_name(n));
    }
    // grid coords of both tiles
    for name in ["DSP_L_X14Y25/DSP_ACOUT0", "DSP_L_X14Y15/DSP_0_ACIN0"] {
        let n = g.node(name)?;
        println!("{name} xy = {:?}", g.node_xy(n));
    }
    // BRKH neighbours?
    for t in ["DSP_L_X14Y20", "DSP_L_X14Y10"] {
        for w in ["DSP_0_ACIN0", "DSP_ACOUT0", "DSP_1_ACOUT0"] {
            if let Ok(n) = g.node(&format!("{t}/{w}")) { println!("{t}/{w} = node {n} (name: {})", g.node_name(n)); }
        }
    }
    Ok(())
}
