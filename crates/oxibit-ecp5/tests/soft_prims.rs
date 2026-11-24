//! Soft primitives against behavioural Verilog: both through synthesis,
//! the ECP5 netlists random-simulated with the same inputs.

use oxibit_ecp5::sim::{Rng, simulate_netlist};
use oxibit_ecp5::synth::{SynthOptions, elaborate, synth_lowered};

fn netlist(src: &str) -> oxibit_ecp5::netlist::Netlist {
    let m = elaborate(src, None).expect("elaborate");
    let (nl, rep, _) = synth_lowered(&m, SynthOptions::default()).expect("synth");
    assert!(
        !rep.warnings.iter().any(|w| w.contains("differs")),
        "warnings: {:?}",
        rep.warnings
    );
    nl
}

/// `prim` and `reference` give the same outputs for 300 cycles x 64 lanes.
fn same(prim: &str, reference: &str) {
    let (a, b) = (netlist(prim), netlist(reference));
    let names = |nl: &oxibit_ecp5::netlist::Netlist| -> Vec<(String, bool)> {
        nl.ports.iter().map(|p| (p.name.clone(), p.output)).collect()
    };
    assert_eq!(names(&a), names(&b), "port lists differ");
    let n_in = a.ports.iter().filter(|p| !p.output && !p.bidir).count();
    let mut rng = Rng::new(7);
    let inputs: Vec<Vec<u64>> = (0..300).map(|_| (0..n_in).map(|_| rng.next()).collect()).collect();
    let (x, y) = (simulate_netlist(&a, &inputs).unwrap(), simulate_netlist(&b, &inputs).unwrap());
    for (c, (p, q)) in x.iter().zip(&y).enumerate() {
        assert_eq!(p, q, "cycle {c}");
    }
}

#[test]
fn lut4_pfumx_l6mux() {
    same(
        "module t(input a, b, c, d, e, f, s, output z, w);
           wire l0, l1, m0;
           LUT4 #(.INIT(16'hCA35)) u0 (.A(a), .B(b), .C(c), .D(d), .Z(l0));
           LUT4 #(.INIT(16'h6996)) u1 (.A(b), .B(c), .C(d), .D(e), .Z(l1));
           PFUMX m (.ALUT(l0), .BLUT(l1), .C0(f), .Z(m0));
           L6MUX21 x (.D0(m0), .D1(a), .SD(s), .Z(z));
           LUT5 #(.INIT(32'hDEADBEEF)) u5 (.A(a), .B(b), .C(c), .D(d), .E(e), .Z(w));
         endmodule",
        "module t(input a, b, c, d, e, f, s, output z, w);
           wire [15:0] i0 = 16'hCA35, i1 = 16'h6996;
           wire [31:0] i5 = 32'hDEADBEEF;
           wire l0 = i0[{d, c, b, a}], l1 = i1[{e, d, c, b}];
           wire m0 = f ? l0 : l1;
           assign z = s ? a : m0;
           assign w = i5[{e, d, c, b, a}];
         endmodule",
    );
}

#[test]
fn gates_and_muxes() {
    same(
        "module t(input a, b, c, d, e, s1, s2, output [8:0] z);
           AND3 g0 (.A(a), .B(b), .C(c), .Z(z[0]));
           ND4 g1 (.A(a), .B(b), .C(c), .D(d), .Z(z[1]));
           OR5 g2 (.A(a), .B(b), .C(c), .D(d), .E(e), .Z(z[2]));
           NR2 g3 (.A(a), .B(b), .Z(z[3]));
           XOR3 g4 (.A(a), .B(b), .C(c), .Z(z[4]));
           XNOR2 g5 (.A(d), .B(e), .Z(z[5]));
           INV g6 (.A(e), .Z(z[6]));
           MUX41 g7 (.D0(a), .D1(b), .D2(c), .D3(d), .SD1(s1), .SD2(s2), .Z(z[7]));
           VHI g8 (.Z(z[8]));
         endmodule",
        "module t(input a, b, c, d, e, s1, s2, output [8:0] z);
           wire [3:0] dd = {d, c, b, a};
           assign z = {1'b1, dd[{s2, s1}], ~e, ~(d ^ e), a ^ b ^ c, ~(a | b),
                       a | b | c | d | e, ~(a & b & c & d), a & b & c};
         endmodule",
    );
}

#[test]
fn rom() {
    same(
        "module t(input [4:0] a, output z);
           ROM32X1A #(.INITVAL(32'h8BADF00D)) r (.AD0(a[0]), .AD1(a[1]), .AD2(a[2]), .AD3(a[3]), .AD4(a[4]), .DO0(z));
         endmodule",
        "module t(input [4:0] a, output z);
           wire [31:0] v = 32'h8BADF00D;
           assign z = v[a];
         endmodule",
    );
}

#[test]
fn flops() {
    same(
        "module t(input clk, d, e, r, output [5:0] q);
           FD1P3AX f0 (.D(d), .SP(e), .CK(clk), .Q(q[0]));
           FD1P3AY f1 (.D(d), .SP(e), .CK(clk), .Q(q[1]));
           FD1P3IX f2 (.D(d), .SP(e), .CK(clk), .CD(r), .Q(q[2]));
           FD1P3JX f3 (.D(d), .SP(e), .CK(clk), .PD(r), .Q(q[3]));
           FD1S3AX f4 (.D(d), .CK(clk), .Q(q[4]));
           FL1P3AZ f5 (.D0(d), .D1(r), .SD(e), .SP(1'b1), .CK(clk), .Q(q[5]));
         endmodule",
        "module t(input clk, d, e, r, output [5:0] q);
           reg q0 = 0, q1 = 1, q2 = 0, q3 = 1, q4 = 0, q5 = 0;
           always @(posedge clk) begin
             if (e) q0 <= d;
             if (e) q1 <= d;
             if (r) q2 <= 0; else if (e) q2 <= d;
             if (r) q3 <= 1; else if (e) q3 <= d;
             q4 <= d;
             q5 <= e ? r : d;
           end
           assign q = {q5, q4, q3, q2, q1, q0};
         endmodule",
    );
}

#[test]
fn async_flops() {
    same(
        "module t(input clk, d, e, r, output [1:0] q);
           FD1P3BX f0 (.D(d), .SP(e), .CK(clk), .PD(r), .Q(q[0]));
           FD1P3DX f1 (.D(d), .SP(e), .CK(clk), .CD(r), .Q(q[1]));
         endmodule",
        "module t(input clk, d, e, r, output [1:0] q);
           reg q0 = 1, q1 = 0;
           always @(posedge clk or posedge r) if (r) q0 <= 1; else if (e) q0 <= d;
           always @(posedge clk or posedge r) if (r) q1 <= 0; else if (e) q1 <= d;
           assign q = {q1, q0};
         endmodule",
    );
}

#[test]
fn ccu2c_adder() {
    // Four-bit adder from two CCU2Cs, as a synthesis tool emits it:
    // INIT 0x9668 (S = A ^ B ^ cin, the A & B nibble for the carry), carry from the LUT2 half (A & B).
    same(
        "module t(input [3:0] a, b, output [4:0] s);
           wire c1, c2;
           CCU2C #(.INIT0(16'h9668), .INIT1(16'h9668), .INJECT1_0(\"NO\"), .INJECT1_1(\"NO\"))
             u0 (.CIN(1'b0), .A0(a[0]), .B0(b[0]), .C0(1'b0), .D0(1'b1),
                 .A1(a[1]), .B1(b[1]), .C1(1'b0), .D1(1'b1), .S0(s[0]), .S1(s[1]), .COUT(c1));
           CCU2C #(.INIT0(16'h9668), .INIT1(16'h9668), .INJECT1_0(\"NO\"), .INJECT1_1(\"NO\"))
             u1 (.CIN(c1), .A0(a[2]), .B0(b[2]), .C0(1'b0), .D0(1'b1),
                 .A1(a[3]), .B1(b[3]), .C1(1'b0), .D1(1'b1), .S0(s[2]), .S1(s[3]), .COUT(c2));
           assign s[4] = c2;
         endmodule",
        "module t(input [3:0] a, b, output [4:0] s);
           assign s = a + b;
         endmodule",
    );
}

/// A 16 x 4 register file in plain Verilog: init `init`, write on `clk`
/// when `we`, asynchronous read at `rad`.
fn regfile(init: u64, write_addr: &str, read_addr: &str) -> String {
    let mut s = String::from("  reg [63:0] m = 64'h");
    s.push_str(&format!("{init:016X};\n  always @(posedge clk) if (we) begin\n"));
    for w in 0..16 {
        s.push_str(&format!("    if ({write_addr} == 4'd{w}) m[{}:{}] <= di;\n", 4 * w + 3, 4 * w));
    }
    s.push_str("  end\n  wire [15:0] b0, b1, b2, b3;\n");
    for w in 0..16 {
        for j in 0..4 {
            s.push_str(&format!("  assign b{j}[{w}] = m[{}];\n", 4 * w + j));
        }
    }
    s.push_str(&format!(
        "  assign q = {{b3[{read_addr}], b2[{read_addr}], b1[{read_addr}], b0[{read_addr}]}};\n"
    ));
    s
}

#[test]
fn distributed_ram() {
    // DPR16X4C with scalar pins, then TRELLIS_DPR16X4 (vector pins) and
    // SPR16X4C, each against the register file.
    let reference = |init, wa, ra| {
        format!(
            "module t(input clk, we, input [3:0] di, wad, rad, output [3:0] q);\n{}endmodule",
            regfile(init, wa, ra)
        )
    };
    same(
        "module t(input clk, we, input [3:0] di, wad, rad, output [3:0] q);
           DPR16X4C #(.INITVAL(\"0x0123456789ABCDEF\")) r (
             .DI0(di[0]), .DI1(di[1]), .DI2(di[2]), .DI3(di[3]),
             .WAD0(wad[0]), .WAD1(wad[1]), .WAD2(wad[2]), .WAD3(wad[3]), .WCK(clk), .WRE(we),
             .RAD0(rad[0]), .RAD1(rad[1]), .RAD2(rad[2]), .RAD3(rad[3]),
             .DO0(q[0]), .DO1(q[1]), .DO2(q[2]), .DO3(q[3]));
         endmodule",
        &reference(0x0123456789ABCDEF, "wad", "rad"),
    );
    same(
        "module t(input clk, we, input [3:0] di, wad, rad, output [3:0] q);
           TRELLIS_DPR16X4 #(.INITVAL(64'h5A5AC3C30F0F9696)) r (
             .DI(di), .WAD(wad), .WRE(we), .WCK(clk), .RAD(rad), .DO(q));
         endmodule",
        &reference(0x5A5AC3C30F0F9696, "wad", "rad"),
    );
    same(
        "module t(input clk, we, input [3:0] di, wad, rad, output [3:0] q);
           SPR16X4C #(.INITVAL(\"0xFEDCBA9876543210\")) r (
             .DI0(di[0]), .DI1(di[1]), .DI2(di[2]), .DI3(di[3]),
             .AD0(wad[0]), .AD1(wad[1]), .AD2(wad[2]), .AD3(wad[3]), .CK(clk), .WRE(we),
             .DO0(q[0]), .DO1(q[1]), .DO2(q[2]), .DO3(q[3]));
         endmodule",
        &reference(0xFEDCBA9876543210, "wad", "wad"),
    );
}

#[test]
fn inferred_distributed_ram() {
    // An asynchronously read 16 x 8 memory with two read ports becomes four
    // distributed RAMs; against the register-file form of the same thing.
    let inits: String = (0..16).map(|w| format!("m[{w}] = 8'd{}; ", w * 17)).collect();
    let src = format!(
        "module t(input clk, we, input [7:0] di, input [3:0] wad, ra, rb, output [7:0] qa, qb);
           reg [7:0] m [0:15];
           initial begin {inits}end
           always @(posedge clk) if (we) m[wad] <= di;
           assign qa = m[ra];
           assign qb = m[rb];
         endmodule"
    );
    let src = src.as_str();
    let m = elaborate(src, None).expect("elaborate");
    let (nl, _, _) = synth_lowered(&m, SynthOptions::default()).expect("synth");
    assert_eq!(nl.counts().get("LUTRAM"), Some(&4));
    let mut reference = String::from(
        "module t(input clk, we, input [7:0] di, input [3:0] wad, ra, rb, output [7:0] qa, qb);\n",
    );
    for w in 0..16 {
        reference.push_str(&format!("  reg [7:0] m{w} = 8'd{};\n", w * 17));
        reference.push_str(&format!("  always @(posedge clk) if (we && wad == 4'd{w}) m{w} <= di;\n"));
    }
    for (q, a) in [("qa", "ra"), ("qb", "rb")] {
        reference.push_str(&format!("  assign {q} = "));
        for w in 0..16 {
            reference.push_str(&format!("{a} == 4'd{w} ? m{w} : "));
        }
        reference.push_str("8'd0;\n");
    }
    reference.push_str("endmodule");
    same(src, &reference);
}
