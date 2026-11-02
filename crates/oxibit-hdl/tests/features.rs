//! casez/casex, asynchronous reset, variable selects: elaborate and check
//! the cells. Equivalence against yosys is checked outside the repo.

use oxibit_hdl::{elab::Elab, netlist::Cell, parser};

fn elab(src: &str) -> Result<oxibit_hdl::netlist::Module, String> {
    let mods = parser::parse(src).map_err(|e| e.to_string())?;
    Elab::module(&mods[0]).map_err(|e| e.to_string())
}

#[test]
fn features_elaborate() {
    let m = elab(include_str!("features.v")).unwrap();
    let count = |f: &dyn Fn(&Cell) -> bool| m.cells.iter().filter(|c| f(c)).count();
    assert_eq!(count(&|c| matches!(c, Cell::Adff { .. })), 2);
    assert_eq!(count(&|c| matches!(c, Cell::Binary { op: oxibit_hdl::netlist::Op::Shiftx, .. })), 4);
}

#[test]
fn reset_edge_must_match_test() {
    let src = "module top(input clk, input rst_n, input d, output reg q);
      always @(posedge clk or negedge rst_n) if (rst_n) q <= 1'b0; else q <= d;
    endmodule";
    assert!(elab(src).unwrap_err().contains("does not match"));
}

#[test]
fn comb_latch_is_an_error() {
    let src = "module top(input s, input d, output reg q);
      always @(*) if (s) q = d;
    endmodule";
    assert!(elab(src).unwrap_err().contains("latch"));
}
