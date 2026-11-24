//! Net declarations with assignments, several declarators to one statement.

use oxibit_ecp5::sim::simulate_netlist;
use oxibit_ecp5::synth::{SynthOptions, elaborate, synth_lowered};

fn out(src: &str) -> u64 {
    let m = elaborate(src, None).expect("elaborate");
    let (nl, _, _) = synth_lowered(&m, SynthOptions::default()).expect("synth");
    let r = simulate_netlist(&nl, &[vec![]]).expect("sim");
    r[0].iter().enumerate().fold(0, |a, (i, &v)| a | ((v & 1) << i))
}

#[test]
fn wire_list_with_assignments() {
    let y = out(
        "module t(output [7:0] y);
           wire a = 1'b1, b = 1'b0, c = 1'b1;
           assign y = {5'h0, a, b, c};
         endmodule",
    );
    assert_eq!(y, 0b101, "y = {y:#b}");
}
