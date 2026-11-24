// Soft-primitive probe for the ULX3S, read by jtag_oracle.exs --design soft.
//
// Everything runs on JTCK. ER1 (IR 0x32) is a 128-bit DR, LSB first.
//   capture: [31:0] magic 0x50F7C0DE, [63:32] x, [95:64] y,
//            [103:96] tag 0xB0, [111:104] capture count, [127:112] 0
//   update:  x <= DR[127:96]; the flops in y[31:24] take one step, from
//            the x before the update (their enables are gated by the step).
// y[23:0] are combinational in x through LUT4 / LUT5 / PFUMX / L6MUX21,
// gates, MUX41 / MUX81, ROM32X1A, two CCU2Cs and BUFBA / VHI / VLO.
module soft_probe (
  input clk_25mhz,
  output [7:0] led
);
  wire jtck, jtdi, jshift, jupdate, jrstn, jce1, jce2, jrti1, jrti2;
  wire zero = 1'b0;

  // TAP state sampled on the falling edge (see pll_jtag.v).
  reg ce1 = 0, shift = 0, upd = 0, tdi = 0;
  always @(negedge jtck) begin
    ce1 <= jce1;
    shift <= jshift;
    upd <= jupdate;
    tdi <= jtdi;
  end
  // JTDI is TDI as the TAP sampled it on the rising edge (it changes just
  // after that edge), so the bit a shift edge takes in is in `tdi` one
  // edge later: the last 32 bits shifted in collect in `rx` one edge
  // behind the shifts out of `sr`.
  reg shifted = 0;
  reg [31:0] rx = 0;
  always @(posedge jtck) begin
    shifted <= ce1 & shift;
    if (shifted) rx <= {tdi, rx[31:1]};
  end

  reg [127:0] sr = 0;
  reg [31:0] x = 0;
  reg [7:0] ncap = 0;
  reg armed = 0;
  wire step = upd & armed;
  wire [31:0] y;

  always @(posedge jtck) begin
    if (ce1) begin
      armed <= 1'b1;
      if (shift) sr <= {1'b0, sr[127:1]};
      else begin
        sr <= {16'h0000, ncap, 8'hB0, y, x, 32'h50F7C0DE};
        ncap <= ncap + 1;
      end
    end else if (step) begin
      armed <= 1'b0;
      x <= rx;
    end
  end

  // Combinational y[23:0].
  wire l0, l1;
  LUT4 #(.INIT(16'hCA35)) u_l0 (.A(x[0]), .B(x[1]), .C(x[2]), .D(x[3]), .Z(l0));
  LUT4 #(.INIT(16'h6996)) u_l1 (.A(x[4]), .B(x[5]), .C(x[6]), .D(x[7]), .Z(l1));
  assign y[0] = l0;
  assign y[1] = l1;
  LUT5 #(.INIT(32'hDEADBEEF)) u_l5 (.A(x[8]), .B(x[9]), .C(x[10]), .D(x[11]), .E(x[12]), .Z(y[2]));
  PFUMX u_pfu (.ALUT(l0), .BLUT(l1), .C0(x[13]), .Z(y[3]));
  L6MUX21 u_l6 (.D0(y[3]), .D1(y[2]), .SD(x[14]), .Z(y[4]));
  AND3 u_and (.A(x[0]), .B(x[1]), .C(x[2]), .Z(y[5]));
  ND4 u_nd (.A(x[3]), .B(x[4]), .C(x[5]), .D(x[6]), .Z(y[6]));
  OR5 u_or (.A(x[7]), .B(x[8]), .C(x[9]), .D(x[10]), .E(x[11]), .Z(y[7]));
  NR2 u_nr (.A(x[12]), .B(x[13]), .Z(y[8]));
  XOR3 u_xor (.A(x[14]), .B(x[15]), .C(x[16]), .Z(y[9]));
  XNOR2 u_xnor (.A(x[17]), .B(x[18]), .Z(y[10]));
  INV u_inv (.A(x[19]), .Z(y[11]));
  MUX41 u_m4 (.D0(x[20]), .D1(x[21]), .D2(x[22]), .D3(x[23]), .SD1(x[24]), .SD2(x[25]), .Z(y[12]));
  MUX81 u_m8 (.D0(x[0]), .D1(x[1]), .D2(x[2]), .D3(x[3]), .D4(x[4]), .D5(x[5]), .D6(x[6]), .D7(x[7]),
              .SD1(x[26]), .SD2(x[27]), .SD3(x[28]), .Z(y[13]));
  ROM32X1A #(.INITVAL(32'h8BADF00D)) u_rom (.AD0(x[27]), .AD1(x[28]), .AD2(x[29]), .AD3(x[30]), .AD4(x[31]), .DO0(y[14]));
  VHI u_hi (.Z(y[15]));
  // y[20:16] = x[3:0] + x[7:4] through two CCU2Cs.
  wire c1;
  CCU2C #(.INIT0(16'h9668), .INIT1(16'h9668), .INJECT1_0("NO"), .INJECT1_1("NO"))
    u_c0 (.CIN(zero), .A0(x[0]), .B0(x[4]), .C0(zero), .D0(y[15]), .A1(x[1]), .B1(x[5]), .C1(zero), .D1(y[15]),
          .S0(y[16]), .S1(y[17]), .COUT(c1));
  CCU2C #(.INIT0(16'h9668), .INIT1(16'h9668), .INJECT1_0("NO"), .INJECT1_1("NO"))
    u_c1 (.CIN(c1), .A0(x[2]), .B0(x[6]), .C0(zero), .D0(y[15]), .A1(x[3]), .B1(x[7]), .C1(zero), .D1(y[15]),
          .S0(y[18]), .S1(y[19]), .COUT(y[20]));
  XOR21 u_x21 (.A(x[0]), .B(x[1]), .C(x[2]), .D(x[3]), .E(x[4]), .F(x[5]), .G(x[6]), .H(x[7]), .I(x[8]),
               .J(x[9]), .K(x[10]), .L(x[11]), .M(x[12]), .N(x[13]), .O(x[14]), .P(x[15]), .Q(x[16]),
               .R(x[17]), .S(x[18]), .T(x[19]), .U(x[20]), .Z(y[21]));
  XOR11 u_x11 (.A(x[10]), .B(x[11]), .C(x[12]), .D(x[13]), .E(x[14]), .F(x[15]), .G(x[16]), .H(x[17]),
               .I(x[18]), .J(x[19]), .K(x[20]), .Z(y[22]));
  BUFBA u_buf (.A(x[31]), .Z(y[23]));

  // Flops y[31:24], one step per update.
  wire sp0, sp1, sp2, sp3, sp4, sp5, sp7;
  AND2 g0 (.A(x[1]), .B(step), .Z(sp0));
  AND2 g1 (.A(x[3]), .B(step), .Z(sp1));
  AND2 g2 (.A(x[5]), .B(step), .Z(sp2));
  AND2 g3 (.A(x[8]), .B(step), .Z(sp3));
  AND2 g4 (.A(x[11]), .B(step), .Z(sp4));
  AND2 g5 (.A(x[14]), .B(step), .Z(sp5));
  AND2 g7 (.A(x[20]), .B(step), .Z(sp7));
  FD1P3AX f0 (.D(x[0]), .SP(sp0), .CK(jtck), .Q(y[24]));
  FD1P3AY f1 (.D(x[2]), .SP(sp1), .CK(jtck), .Q(y[25]));
  FD1P3IX f2 (.D(x[4]), .SP(sp2), .CK(jtck), .CD(x[6]), .Q(y[26]));
  FD1P3JX f3 (.D(x[7]), .SP(sp3), .CK(jtck), .PD(x[9]), .Q(y[27]));
  FD1P3BX f4 (.D(x[10]), .SP(sp4), .CK(jtck), .PD(x[12]), .Q(y[28]));
  FD1P3DX f5 (.D(x[13]), .SP(sp5), .CK(jtck), .CD(x[15]), .Q(y[29]));
  FD1S3AX f6 (.D(x[16]), .CK(jtck), .Q(y[30]));
  FL1P3AZ f7 (.D0(x[17]), .D1(x[18]), .SD(x[19]), .SP(sp7), .CK(jtck), .Q(y[31]));

  JTAGG #(.ER1("ENABLED"), .ER2("ENABLED")) jtag (
    .JTDO1(sr[0]), .JTDO2(zero),
    .JTCK(jtck), .JTDI(jtdi), .JSHIFT(jshift), .JUPDATE(jupdate),
    .JRSTN(jrstn), .JCE1(jce1), .JCE2(jce2), .JRTI1(jrti1), .JRTI2(jrti2)
  );

  // Heartbeat so the board shows it is configured.
  reg [25:0] hb = 0;
  always @(posedge clk_25mhz) hb <= hb + 1;
  assign led = {hb[25], 7'b0};
endmodule
