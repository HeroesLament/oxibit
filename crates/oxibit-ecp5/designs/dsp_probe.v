// sysDSP probe for the ULX3S, read by jtag_oracle.exs --design dsp.
//
// Transport: an EIO register bus on ER1, from the local model. One 49-bit
// data register, LSB first: data[31:0], addr[47:32], write flag [48]. A
// scan with the flag set writes data to addr on Update-DR; a scan with it
// clear selects addr, and the next scan's capture returns that register.
//   0x0000 VERSION  {8'h00, 8'h05, "IO"}
//   0x0004 IN_W     187
//   0x0008 OUT_W    64
//   0x0010.. IN     {x[63:0], r4[23:0], po[8:0], p2[17:0], p1[35:0], p0[35:0]}
//   0x0100.. OUT    x[31:0], x[63:32]
//   m0: MULT18X18D, no registers: A x[17:0], B x[35:18], SIGNEDA x36,
//       SIGNEDB x37, P -> p0.
//   m1: the same operands with the A, B and output registers on JTCK
//       (CLK0, CE0 high): a few edges after an update p1 = p0.
//   m2: MULT9X9D (9x9 split mode), from the local model: A x[8:0],
//       B x[17:9], C x[26:18] preload; P -> p2 = A*B.
//   m3: PRADD9A (9-bit pre-adder), from the local model: PA x[8:0],
//       PB x[17:9], OPPRE zero; PO -> po = PA+PB (chain into m2's A
//       comes with the packing work; standalone here first).
//   m4: ALU24B (24-bit ALU split), from the local model: MA/MB open,
//       CIN x[23:0] with OPCINSEL high; R -> r4 = CIN.
module dsp_probe (
  input clk_25mhz,
  output [7:0] led
);
  wire jtck, jtdi, jshift, jupdate, jrstn, jce1, jce2, jrti1, jrti2;
  wire zero = 1'b0, one = 1'b1;

  reg ce1 = 0, shift = 0, upd = 0, tdi = 0;
  always @(negedge jtck) begin
    ce1 <= jce1;
    shift <= jshift;
    upd <= jupdate;
    tdi <= jtdi;
  end
  reg shifted = 0;
  reg [48:0] rx = 0;
  always @(posedge jtck) begin
    shifted <= ce1 & shift;
    if (shifted) rx <= {tdi, rx[48:1]};
  end

  reg [48:0] sr = 0;
  reg [15:0] addr = 0;
  reg [31:0] xl = 0, xh = 0;
  wire [63:0] x = {xh, xl};
  reg armed = 0;
  wire step = upd & armed;
  wire [35:0] p0, p1;
  wire [17:0] p2;
  wire [23:0] r4; wire [17:0] pa, pb;

  wire [191:0] pin = {5'h00, x, r4, 9'h000, p2, p1, p0};
  wire [31:0] rdata =
    addr == 16'h0000 ? 32'h0005494F :
    addr == 16'h0004 ? 32'd187 :
    addr == 16'h0008 ? 32'd64 :
    addr == 16'h0010 ? pin[31:0] :
    addr == 16'h0014 ? pin[63:32] :
    addr == 16'h0018 ? pin[95:64] :
    addr == 16'h001C ? pin[127:96] :
    addr == 16'h0020 ? pin[159:128] :
    addr == 16'h0024 ? pin[191:160] :
    addr == 16'h0100 ? xl :
    addr == 16'h0104 ? xh : 32'h00000000;

  always @(posedge jtck) begin
    if (ce1) begin
      armed <= 1'b1;
      if (shift) sr <= {1'b0, sr[48:1]};
      else sr <= {17'h00000, rdata};
    end else if (step) begin
      armed <= 1'b0;
      addr <= rx[47:32];
      if (rx[48] & (rx[47:32] == 16'h0100)) xl <= rx[31:0];
      if (rx[48] & (rx[47:32] == 16'h0104)) xh <= rx[31:0];
    end
  end

  wire [8:0] m9b = x[17:9];
  wire [8:0] m9c = x[26:18];
  wire [23:0] alu = x[47:24];
  wire [2:0] cin = x[51:49];

  MULT18X18D m0 (
    .A17(x[17]), .A16(x[16]), .A15(x[15]), .A14(x[14]), .A13(x[13]), .A12(x[12]),
    .A11(x[11]), .A10(x[10]), .A9(x[9]), .A8(x[8]), .A7(x[7]), .A6(x[6]),
    .A5(x[5]), .A4(x[4]), .A3(x[3]), .A2(x[2]), .A1(x[1]), .A0(x[0]),
    .B17(x[35]), .B16(x[34]), .B15(x[33]), .B14(x[32]), .B13(x[31]), .B12(x[30]),
    .B11(x[29]), .B10(x[28]), .B9(x[27]), .B8(x[26]), .B7(x[25]), .B6(x[24]),
    .B5(x[23]), .B4(x[22]), .B3(x[21]), .B2(x[20]), .B1(x[19]), .B0(x[18]),
    .SIGNEDA(x[36]), .SIGNEDB(x[37]), .SOURCEA(zero), .SOURCEB(zero),
    .P35(p0[35]), .P34(p0[34]), .P33(p0[33]), .P32(p0[32]), .P31(p0[31]), .P30(p0[30]),
    .P29(p0[29]), .P28(p0[28]), .P27(p0[27]), .P26(p0[26]), .P25(p0[25]), .P24(p0[24]),
    .P23(p0[23]), .P22(p0[22]), .P21(p0[21]), .P20(p0[20]), .P19(p0[19]), .P18(p0[18]),
    .P17(p0[17]), .P16(p0[16]), .P15(p0[15]), .P14(p0[14]), .P13(p0[13]), .P12(p0[12]),
    .P11(p0[11]), .P10(p0[10]), .P9(p0[9]), .P8(p0[8]), .P7(p0[7]), .P6(p0[6]),
    .P5(p0[5]), .P4(p0[4]), .P3(p0[3]), .P2(p0[2]), .P1(p0[1]), .P0(p0[0])
  );

  MULT18X18D #(.REG_INPUTA_CLK("CLK0"), .REG_INPUTB_CLK("CLK0"), .REG_OUTPUT_CLK("CLK0"))
  m1 (
    .A17(x[17]), .A16(x[16]), .A15(x[15]), .A14(x[14]), .A13(x[13]), .A12(x[12]),
    .A11(x[11]), .A10(x[10]), .A9(x[9]), .A8(x[8]), .A7(x[7]), .A6(x[6]),
    .A5(x[5]), .A4(x[4]), .A3(x[3]), .A2(x[2]), .A1(x[1]), .A0(x[0]),
    .B17(x[35]), .B16(x[34]), .B15(x[33]), .B14(x[32]), .B13(x[31]), .B12(x[30]),
    .B11(x[29]), .B10(x[28]), .B9(x[27]), .B8(x[26]), .B7(x[25]), .B6(x[24]),
    .B5(x[23]), .B4(x[22]), .B3(x[21]), .B2(x[20]), .B1(x[19]), .B0(x[18]),
    .SIGNEDA(x[36]), .SIGNEDB(x[37]), .SOURCEA(zero), .SOURCEB(zero),
    .CLK0(jtck), .CE0(one), .RST0(zero),
    .P35(p1[35]), .P34(p1[34]), .P33(p1[33]), .P32(p1[32]), .P31(p1[31]), .P30(p1[30]),
    .P29(p1[29]), .P28(p1[28]), .P27(p1[27]), .P26(p1[26]), .P25(p1[25]), .P24(p1[24]),
    .P23(p1[23]), .P22(p1[22]), .P21(p1[21]), .P20(p1[20]), .P19(p1[19]), .P18(p1[18]),
    .P17(p1[17]), .P16(p1[16]), .P15(p1[15]), .P14(p1[14]), .P13(p1[13]), .P12(p1[12]),
    .P11(p1[11]), .P10(p1[10]), .P9(p1[9]), .P8(p1[8]), .P7(p1[7]), .P6(p1[6]),
    .P5(p1[5]), .P4(p1[4]), .P3(p1[3]), .P2(p1[2]), .P1(p1[1]), .P0(p1[0])
  );

  // MULT9X9D, from the local model. Unregistered combinatorial split
  // multiplier: p2 = A*B (18-bit product of 9-bit operands).
  // Companion multipliers for the ALU24B, from the local model: the
  // ALU24B's MA/MB are dedicated wires from the paired multiplier sites
  // (z-3, z-2) of its block, so the pair must sit there. The products
  // are real: r4 = MA + MB. The sign pins ride x[63:60] (m5 A, m5 B,
  // m6 A, m6 B), low for unsigned operands.
  MULT9X9D m5 (
    .CE0(one), .CE1(one), .CE2(one), .CE3(one),
    .CLK0(jtck), .CLK1(jtck), .CLK2(jtck), .CLK3(jtck),
    .RST0(zero), .RST1(zero), .RST2(zero), .RST3(zero),
    .SIGNEDA(x[60]), .SIGNEDB(x[61]), .SOURCEA(zero), .SOURCEB(zero),
    .A8(x[8]), .A7(x[7]), .A6(x[6]), .A5(x[5]), .A4(x[4]), .A3(x[3]), .A2(x[2]), .A1(x[1]), .A0(x[0]),
    .B8(x[17]), .B7(x[16]), .B6(x[15]), .B5(x[14]), .B4(x[13]), .B3(x[12]), .B2(x[11]), .B1(x[10]), .B0(x[9]),
    .P17(pa[17]), .P16(pa[16]), .P15(pa[15]), .P14(pa[14]), .P13(pa[13]), .P12(pa[12]), .P11(pa[11]), .P10(pa[10]), .P9(pa[9]), .P8(pa[8]),
    .P7(pa[7]), .P6(pa[6]), .P5(pa[5]), .P4(pa[4]), .P3(pa[3]), .P2(pa[2]), .P1(pa[1]), .P0(pa[0])
  );
  MULT9X9D m6 (
    .CE0(one), .CE1(one), .CE2(one), .CE3(one),
    .CLK0(jtck), .CLK1(jtck), .CLK2(jtck), .CLK3(jtck),
    .RST0(zero), .RST1(zero), .RST2(zero), .RST3(zero),
    .SIGNEDA(x[62]), .SIGNEDB(x[63]), .SOURCEA(zero), .SOURCEB(zero),
    .A8(x[26]), .A7(x[25]), .A6(x[24]), .A5(x[23]), .A4(x[22]), .A3(x[21]), .A2(x[20]), .A1(x[19]), .A0(x[18]),
    .B8(x[35]), .B7(x[34]), .B6(x[33]), .B5(x[32]), .B4(x[31]), .B3(x[30]), .B2(x[29]), .B1(x[28]), .B0(x[27]),
    .P17(pb[17]), .P16(pb[16]), .P15(pb[15]), .P14(pb[14]), .P13(pb[13]), .P12(pb[12]), .P11(pb[11]), .P10(pb[10]), .P9(pb[9]), .P8(pb[8]),
    .P7(pb[7]), .P6(pb[6]), .P5(pb[5]), .P4(pb[4]), .P3(pb[3]), .P2(pb[2]), .P1(pb[1]), .P0(pb[0])
  );

  MULT9X9D m2 (
    .A8(x[8]), .A7(x[7]), .A6(x[6]), .A5(x[5]), .A4(x[4]), .A3(x[3]), .A2(x[2]), .A1(x[1]), .A0(x[0]),
    .B8(x[17]), .B7(x[16]), .B6(x[15]), .B5(x[14]), .B4(x[13]), .B3(x[12]), .B2(x[11]), .B1(x[10]), .B0(x[9]),
    .C8(x[8]), .C7(x[7]), .C6(x[6]), .C5(x[5]), .C4(x[4]), .C3(x[3]), .C2(x[2]), .C1(x[1]), .C0(x[0]),
    .SIGNEDA(x[54]), .SIGNEDB(x[55]), .SOURCEA(x[56]), .SOURCEB(x[57]),
    .P17(p2[17]), .P16(p2[16]), .P15(p2[15]), .P14(p2[14]), .P13(p2[13]), .P12(p2[12]),
    .P11(p2[11]), .P10(p2[10]), .P9(p2[9]), .P8(p2[8]), .P7(p2[7]), .P6(p2[6]),
    .P5(p2[5]), .P4(p2[4]), .P3(p2[3]), .P2(p2[2]), .P1(p2[1]), .P0(p2[0])
  );

  // PRADD9A, from the local model. Standalone pre-adder: po = PA+PB.
  PRADD9A m3 (
    .PA8(x[8]), .PA7(x[7]), .PA6(x[6]), .PA5(x[5]), .PA4(x[4]), .PA3(x[3]), .PA2(x[2]), .PA1(x[1]), .PA0(x[0]),
    .PB8(x[17]), .PB7(x[16]), .PB6(x[15]), .PB5(x[14]), .PB4(x[13]), .PB3(x[12]), .PB2(x[11]), .PB1(x[10]), .PB0(x[9]),
    .C8(x[8]), .C7(x[7]), .C6(x[6]), .C5(x[5]), .C4(x[4]), .C3(x[3]), .C2(x[2]), .C1(x[1]), .C0(x[0]),
    .SOURCEA(x[56]), .OPPRE(),
    .CE0(one), .CE1(one), .CE2(one), .CE3(one),
    .CLK0(jtck), .CLK1(jtck), .CLK2(jtck), .CLK3(jtck),
    .RST0(zero), .RST1(zero), .RST2(zero), .RST3(zero),
    .PO8(), .PO7(), .PO6(), .PO5(), .PO4(), .PO3(), .PO2(), .PO1(), .PO0()
  );

  // ALU24B, from the local model. MA/MB are the paired multiplier
  // products (dedicated wires from m5/m6) and CIN is grounded (the
  // cascade input; the pairing rule forbids fabric on it), so
  // r4 = MA + MB.
  ALU24B m4 (
    .CE0(one), .CE1(one), .CE2(one), .CE3(one),
    .CLK0(jtck), .CLK1(jtck), .CLK2(jtck), .CLK3(jtck),
    .RST0(zero), .RST1(zero), .RST2(zero), .RST3(zero),
    .SIGNEDIA(zero), .SIGNEDIB(zero),
    .MA17(pa[17]), .MA16(pa[16]), .MA15(pa[15]), .MA14(pa[14]), .MA13(pa[13]), .MA12(pa[12]),
    .MA11(pa[11]), .MA10(pa[10]), .MA9(pa[9]), .MA8(pa[8]), .MA7(pa[7]), .MA6(pa[6]),
    .MA5(pa[5]), .MA4(pa[4]), .MA3(pa[3]), .MA2(pa[2]), .MA1(pa[1]), .MA0(pa[0]),
    .MB17(pb[17]), .MB16(pb[16]), .MB15(pb[15]), .MB14(pb[14]), .MB13(pb[13]), .MB12(pb[12]),
    .MB11(pb[11]), .MB10(pb[10]), .MB9(pb[9]), .MB8(pb[8]), .MB7(pb[7]), .MB6(pb[6]),
    .MB5(pb[5]), .MB4(pb[4]), .MB3(pb[3]), .MB2(pb[2]), .MB1(pb[1]), .MB0(pb[0]),
    .CIN23(zero), .CIN22(zero), .CIN21(zero), .CIN20(zero), .CIN19(zero), .CIN18(zero),
    .CIN17(zero), .CIN16(zero), .CIN15(zero), .CIN14(zero), .CIN13(zero), .CIN12(zero),
    .CIN11(zero), .CIN10(zero), .CIN9(zero), .CIN8(zero), .CIN7(zero), .CIN6(zero),
    .CIN5(zero), .CIN4(zero), .CIN3(zero), .CIN2(zero), .CIN1(zero), .CIN0(zero),
    .CFB23(zero), .CFB22(zero), .CFB21(zero), .CFB20(zero), .CFB19(zero), .CFB18(zero),
    .CFB17(zero), .CFB16(zero), .CFB15(zero), .CFB14(zero), .CFB13(zero), .CFB12(zero),
    .CFB11(zero), .CFB10(zero), .CFB9(zero), .CFB8(zero), .CFB7(zero), .CFB6(zero),
    .CFB5(zero), .CFB4(zero), .CFB3(zero), .CFB2(zero), .CFB1(zero), .CFB0(zero),
    .OPADDNSUB(zero), .OPCINSEL(one),
    .R23(r4[23]), .R22(r4[22]), .R21(r4[21]), .R20(r4[20]), .R19(r4[19]), .R18(r4[18]),
    .R17(r4[17]), .R16(r4[16]), .R15(r4[15]), .R14(r4[14]), .R13(r4[13]), .R12(r4[12]),
    .R11(r4[11]), .R10(r4[10]), .R9(r4[9]), .R8(r4[8]), .R7(r4[7]), .R6(r4[6]),
    .R5(r4[5]), .R4(r4[4]), .R3(r4[3]), .R2(r4[2]), .R1(r4[1]), .R0(r4[0])
  );

  JTAGG #(.ER1("ENABLED"), .ER2("ENABLED")) jtag (
    .JTDO1(sr[0]), .JTDO2(zero),
    .JTCK(jtck), .JTDI(jtdi), .JSHIFT(jshift), .JUPDATE(jupdate),
    .JRSTN(jrstn), .JCE1(jce1), .JCE2(jce2), .JRTI1(jrti1), .JRTI2(jrti2)
  );

  reg [25:0] hb = 0;
  always @(posedge clk_25mhz) hb <= hb + 1;
  assign led = {hb[25], 7'b0};
endmodule
