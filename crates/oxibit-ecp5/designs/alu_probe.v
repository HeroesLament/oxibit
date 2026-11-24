// sysDSP ALU54B probe for the ULX3S, read by jtag_oracle.exs --design alu.
//
// 256-bit ER1 frame, LSB first: magic 0xA1A10000, x[63:0], r[53:0],
// 10'h0, tag 0xA5, ncap, then zeros. x is the last 64 bits shifted in,
// latched on Update-DR. All combinational:
//   m0: MULT18X18D, A x[17:0], B x[35:18], unsigned.
//   m1: MULT18X18D, A {9'h0, x[44:36]}, B {9'h0, x[53:45]}, unsigned.
//   alu: ALU54B, MA m0.P, MB m1.P, A {m0.ROB, m0.ROA}, B {m1.ROB, m1.ROA},
//        C x[53:0], OP {1'b0, x[63:54]}, R -> r.
module alu_probe (
  input clk_25mhz,
  output [7:0] led
);
  wire jtck, jtdi, jshift, jupdate, jrstn, jce1, jce2, jrti1, jrti2;
  wire zero = 1'b0;

  reg ce1 = 0, shift = 0, upd = 0, tdi = 0;
  always @(negedge jtck) begin
    ce1 <= jce1;
    shift <= jshift;
    upd <= jupdate;
    tdi <= jtdi;
  end
  reg shifted = 0;
  reg [63:0] rx = 0;
  always @(posedge jtck) begin
    shifted <= ce1 & shift;
    if (shifted) rx <= {tdi, rx[63:1]};
  end

  reg [255:0] sr = 0;
  reg [63:0] x = 0;
  reg [7:0] ncap = 0;
  reg armed = 0;
  wire step = upd & armed;
  wire [53:0] r;

  always @(posedge jtck) begin
    if (ce1) begin
      armed <= 1'b1;
      if (shift) sr <= {1'b0, sr[255:1]};
      else begin
        sr <= {80'h0, ncap, 8'hA5, 10'h0, r, x, 32'hA1A10000};
        ncap <= ncap + 1;
      end
    end else if (step) begin
      armed <= 1'b0;
      x <= rx;
    end
  end

  wire [35:0] p0, p1;
  wire [17:0] roa0, rob0, roa1, rob1;
  wire [17:0] a1 = {9'h0, x[44:36]};
  wire [17:0] b1 = {9'h0, x[53:45]};
  wire [10:0] op = {1'b0, x[63:54]};
  wire sp0, sp1;

  MULT18X18D m0 (
    .A17(x[17]), .A16(x[16]), .A15(x[15]), .A14(x[14]), .A13(x[13]), .A12(x[12]), .A11(x[11]), 
    .A10(x[10]), .A9(x[9]), .A8(x[8]), .A7(x[7]), .A6(x[6]), .A5(x[5]), .A4(x[4]), .A3(x[3]), 
    .A2(x[2]), .A1(x[1]), .A0(x[0]), .B17(x[35]), .B16(x[34]), .B15(x[33]), .B14(x[32]), 
    .B13(x[31]), .B12(x[30]), .B11(x[29]), .B10(x[28]), .B9(x[27]), .B8(x[26]), .B7(x[25]), 
    .B6(x[24]), .B5(x[23]), .B4(x[22]), .B3(x[21]), .B2(x[20]), .B1(x[19]), .B0(x[18]), 
    .SIGNEDA(zero), .SIGNEDB(zero), .SOURCEA(zero), .SOURCEB(zero), .SIGNEDP(sp0), .ROA17(roa0[17]), 
    .ROA16(roa0[16]), .ROA15(roa0[15]), .ROA14(roa0[14]), .ROA13(roa0[13]), .ROA12(roa0[12]), 
    .ROA11(roa0[11]), .ROA10(roa0[10]), .ROA9(roa0[9]), .ROA8(roa0[8]), .ROA7(roa0[7]), 
    .ROA6(roa0[6]), .ROA5(roa0[5]), .ROA4(roa0[4]), .ROA3(roa0[3]), .ROA2(roa0[2]), .ROA1(roa0[1]), 
    .ROA0(roa0[0]), .ROB17(rob0[17]), .ROB16(rob0[16]), .ROB15(rob0[15]), .ROB14(rob0[14]), 
    .ROB13(rob0[13]), .ROB12(rob0[12]), .ROB11(rob0[11]), .ROB10(rob0[10]), .ROB9(rob0[9]), 
    .ROB8(rob0[8]), .ROB7(rob0[7]), .ROB6(rob0[6]), .ROB5(rob0[5]), .ROB4(rob0[4]), .ROB3(rob0[3]), 
    .ROB2(rob0[2]), .ROB1(rob0[1]), .ROB0(rob0[0]), .P35(p0[35]), .P34(p0[34]), .P33(p0[33]), 
    .P32(p0[32]), .P31(p0[31]), .P30(p0[30]), .P29(p0[29]), .P28(p0[28]), .P27(p0[27]), 
    .P26(p0[26]), .P25(p0[25]), .P24(p0[24]), .P23(p0[23]), .P22(p0[22]), .P21(p0[21]), 
    .P20(p0[20]), .P19(p0[19]), .P18(p0[18]), .P17(p0[17]), .P16(p0[16]), .P15(p0[15]), 
    .P14(p0[14]), .P13(p0[13]), .P12(p0[12]), .P11(p0[11]), .P10(p0[10]), .P9(p0[9]), .P8(p0[8]), 
    .P7(p0[7]), .P6(p0[6]), .P5(p0[5]), .P4(p0[4]), .P3(p0[3]), .P2(p0[2]), .P1(p0[1]), .P0(p0[0])  );

  MULT18X18D m1 (
    .A17(a1[17]), .A16(a1[16]), .A15(a1[15]), .A14(a1[14]), .A13(a1[13]), .A12(a1[12]), 
    .A11(a1[11]), .A10(a1[10]), .A9(a1[9]), .A8(a1[8]), .A7(a1[7]), .A6(a1[6]), .A5(a1[5]), 
    .A4(a1[4]), .A3(a1[3]), .A2(a1[2]), .A1(a1[1]), .A0(a1[0]), .B17(b1[17]), .B16(b1[16]), 
    .B15(b1[15]), .B14(b1[14]), .B13(b1[13]), .B12(b1[12]), .B11(b1[11]), .B10(b1[10]), .B9(b1[9]), 
    .B8(b1[8]), .B7(b1[7]), .B6(b1[6]), .B5(b1[5]), .B4(b1[4]), .B3(b1[3]), .B2(b1[2]), .B1(b1[1]), 
    .B0(b1[0]), .SIGNEDA(zero), .SIGNEDB(zero), .SOURCEA(zero), .SOURCEB(zero), .SIGNEDP(sp1), .ROA17(roa1[17]), 
    .ROA16(roa1[16]), .ROA15(roa1[15]), .ROA14(roa1[14]), .ROA13(roa1[13]), .ROA12(roa1[12]), 
    .ROA11(roa1[11]), .ROA10(roa1[10]), .ROA9(roa1[9]), .ROA8(roa1[8]), .ROA7(roa1[7]), 
    .ROA6(roa1[6]), .ROA5(roa1[5]), .ROA4(roa1[4]), .ROA3(roa1[3]), .ROA2(roa1[2]), .ROA1(roa1[1]), 
    .ROA0(roa1[0]), .ROB17(rob1[17]), .ROB16(rob1[16]), .ROB15(rob1[15]), .ROB14(rob1[14]), 
    .ROB13(rob1[13]), .ROB12(rob1[12]), .ROB11(rob1[11]), .ROB10(rob1[10]), .ROB9(rob1[9]), 
    .ROB8(rob1[8]), .ROB7(rob1[7]), .ROB6(rob1[6]), .ROB5(rob1[5]), .ROB4(rob1[4]), .ROB3(rob1[3]), 
    .ROB2(rob1[2]), .ROB1(rob1[1]), .ROB0(rob1[0]), .P35(p1[35]), .P34(p1[34]), .P33(p1[33]), 
    .P32(p1[32]), .P31(p1[31]), .P30(p1[30]), .P29(p1[29]), .P28(p1[28]), .P27(p1[27]), 
    .P26(p1[26]), .P25(p1[25]), .P24(p1[24]), .P23(p1[23]), .P22(p1[22]), .P21(p1[21]), 
    .P20(p1[20]), .P19(p1[19]), .P18(p1[18]), .P17(p1[17]), .P16(p1[16]), .P15(p1[15]), 
    .P14(p1[14]), .P13(p1[13]), .P12(p1[12]), .P11(p1[11]), .P10(p1[10]), .P9(p1[9]), .P8(p1[8]), 
    .P7(p1[7]), .P6(p1[6]), .P5(p1[5]), .P4(p1[4]), .P3(p1[3]), .P2(p1[2]), .P1(p1[1]), .P0(p1[0])  );

  ALU54B alu (
    .A35(rob0[17]), .A34(rob0[16]), .A33(rob0[15]), .A32(rob0[14]), .A31(rob0[13]), .A30(rob0[12]), 
    .A29(rob0[11]), .A28(rob0[10]), .A27(rob0[9]), .A26(rob0[8]), .A25(rob0[7]), .A24(rob0[6]), 
    .A23(rob0[5]), .A22(rob0[4]), .A21(rob0[3]), .A20(rob0[2]), .A19(rob0[1]), .A18(rob0[0]), 
    .A17(roa0[17]), .A16(roa0[16]), .A15(roa0[15]), .A14(roa0[14]), .A13(roa0[13]), .A12(roa0[12]), 
    .A11(roa0[11]), .A10(roa0[10]), .A9(roa0[9]), .A8(roa0[8]), .A7(roa0[7]), .A6(roa0[6]), 
    .A5(roa0[5]), .A4(roa0[4]), .A3(roa0[3]), .A2(roa0[2]), .A1(roa0[1]), .A0(roa0[0]), 
    .B35(rob1[17]), .B34(rob1[16]), .B33(rob1[15]), .B32(rob1[14]), .B31(rob1[13]), .B30(rob1[12]), 
    .B29(rob1[11]), .B28(rob1[10]), .B27(rob1[9]), .B26(rob1[8]), .B25(rob1[7]), .B24(rob1[6]), 
    .B23(rob1[5]), .B22(rob1[4]), .B21(rob1[3]), .B20(rob1[2]), .B19(rob1[1]), .B18(rob1[0]), 
    .B17(roa1[17]), .B16(roa1[16]), .B15(roa1[15]), .B14(roa1[14]), .B13(roa1[13]), .B12(roa1[12]), 
    .B11(roa1[11]), .B10(roa1[10]), .B9(roa1[9]), .B8(roa1[8]), .B7(roa1[7]), .B6(roa1[6]), 
    .B5(roa1[5]), .B4(roa1[4]), .B3(roa1[3]), .B2(roa1[2]), .B1(roa1[1]), .B0(roa1[0]), 
    .MA35(p0[35]), .MA34(p0[34]), .MA33(p0[33]), .MA32(p0[32]), .MA31(p0[31]), .MA30(p0[30]), 
    .MA29(p0[29]), .MA28(p0[28]), .MA27(p0[27]), .MA26(p0[26]), .MA25(p0[25]), .MA24(p0[24]), 
    .MA23(p0[23]), .MA22(p0[22]), .MA21(p0[21]), .MA20(p0[20]), .MA19(p0[19]), .MA18(p0[18]), 
    .MA17(p0[17]), .MA16(p0[16]), .MA15(p0[15]), .MA14(p0[14]), .MA13(p0[13]), .MA12(p0[12]), 
    .MA11(p0[11]), .MA10(p0[10]), .MA9(p0[9]), .MA8(p0[8]), .MA7(p0[7]), .MA6(p0[6]), .MA5(p0[5]), 
    .MA4(p0[4]), .MA3(p0[3]), .MA2(p0[2]), .MA1(p0[1]), .MA0(p0[0]), .MB35(p1[35]), .MB34(p1[34]), 
    .MB33(p1[33]), .MB32(p1[32]), .MB31(p1[31]), .MB30(p1[30]), .MB29(p1[29]), .MB28(p1[28]), 
    .MB27(p1[27]), .MB26(p1[26]), .MB25(p1[25]), .MB24(p1[24]), .MB23(p1[23]), .MB22(p1[22]), 
    .MB21(p1[21]), .MB20(p1[20]), .MB19(p1[19]), .MB18(p1[18]), .MB17(p1[17]), .MB16(p1[16]), 
    .MB15(p1[15]), .MB14(p1[14]), .MB13(p1[13]), .MB12(p1[12]), .MB11(p1[11]), .MB10(p1[10]), 
    .MB9(p1[9]), .MB8(p1[8]), .MB7(p1[7]), .MB6(p1[6]), .MB5(p1[5]), .MB4(p1[4]), .MB3(p1[3]), 
    .MB2(p1[2]), .MB1(p1[1]), .MB0(p1[0]), .C53(x[53]), .C52(x[52]), .C51(x[51]), .C50(x[50]), 
    .C49(x[49]), .C48(x[48]), .C47(x[47]), .C46(x[46]), .C45(x[45]), .C44(x[44]), .C43(x[43]), .C42(x[42]), 
    .C41(x[41]), .C40(x[40]), .C39(x[39]), .C38(x[38]), .C37(x[37]), .C36(x[36]), .C35(x[35]), .C34(x[34]), 
    .C33(x[33]), .C32(x[32]), .C31(x[31]), .C30(x[30]), .C29(x[29]), .C28(x[28]), .C27(x[27]), .C26(x[26]), 
    .C25(x[25]), .C24(x[24]), .C23(x[23]), .C22(x[22]), .C21(x[21]), .C20(x[20]), .C19(x[19]), .C18(x[18]), 
    .C17(x[17]), .C16(x[16]), .C15(x[15]), .C14(x[14]), .C13(x[13]), .C12(x[12]), .C11(x[11]), .C10(x[10]), 
    .C9(x[9]), .C8(x[8]), .C7(x[7]), .C6(x[6]), .C5(x[5]), .C4(x[4]), .C3(x[3]), .C2(x[2]), 
    .C1(x[1]), .C0(x[0]), .OP10(op[10]), .OP9(op[9]), .OP8(op[8]), .OP7(op[7]), .OP6(op[6]), 
    .OP5(op[5]), .OP4(op[4]), .OP3(op[3]), .OP2(op[2]), .OP1(op[1]), .OP0(op[0]), .SIGNEDIA(sp0), 
    .SIGNEDIB(sp1), .SIGNEDCIN(zero), .R53(r[53]), .R52(r[52]), .R51(r[51]), .R50(r[50]), 
    .R49(r[49]), .R48(r[48]), .R47(r[47]), .R46(r[46]), .R45(r[45]), .R44(r[44]), .R43(r[43]), 
    .R42(r[42]), .R41(r[41]), .R40(r[40]), .R39(r[39]), .R38(r[38]), .R37(r[37]), .R36(r[36]), 
    .R35(r[35]), .R34(r[34]), .R33(r[33]), .R32(r[32]), .R31(r[31]), .R30(r[30]), .R29(r[29]), 
    .R28(r[28]), .R27(r[27]), .R26(r[26]), .R25(r[25]), .R24(r[24]), .R23(r[23]), .R22(r[22]), 
    .R21(r[21]), .R20(r[20]), .R19(r[19]), .R18(r[18]), .R17(r[17]), .R16(r[16]), .R15(r[15]), 
    .R14(r[14]), .R13(r[13]), .R12(r[12]), .R11(r[11]), .R10(r[10]), .R9(r[9]), .R8(r[8]), 
    .R7(r[7]), .R6(r[6]), .R5(r[5]), .R4(r[4]), .R3(r[3]), .R2(r[2]), .R1(r[1]), .R0(r[0])
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
