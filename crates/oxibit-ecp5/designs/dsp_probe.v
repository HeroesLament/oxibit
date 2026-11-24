// sysDSP probe for the ULX3S, read by jtag_oracle.exs --design dsp.
//
// 256-bit ER1 frame, LSB first: magic 0xD5D50000, x[31:0], x[63:32],
// p0[31:0], {24'h0, p1[35:32], p0[35:32]}, p1[31:0], 32'h0,
// {16'h0, ncap, tag 0xD5}. x is the last 64 bits shifted in, latched on
// Update-DR.
//   m0: MULT18X18D, no registers: A x[17:0], B x[35:18], SIGNEDA x36,
//       SIGNEDB x37, P -> p0.
//   m1: the same operands with the A, B and output registers on JTCK
//       (CLK0, CE0 high): a few edges after an update p1 = p0.
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
  wire [35:0] p0, p1;

  always @(posedge jtck) begin
    if (ce1) begin
      armed <= 1'b1;
      if (shift) sr <= {1'b0, sr[255:1]};
      else begin
        sr <= {16'h0000, ncap, 8'hD5, 32'h0, p1[31:0], 24'h0, p1[35:32], p0[35:32],
               p0[31:0], x, 32'hD5D50000};
        ncap <= ncap + 1;
      end
    end else if (step) begin
      armed <= 1'b0;
      x <= rx;
    end
  end

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

  JTAGG #(.ER1("ENABLED"), .ER2("ENABLED")) jtag (
    .JTDO1(sr[0]), .JTDO2(zero),
    .JTCK(jtck), .JTDI(jtdi), .JSHIFT(jshift), .JUPDATE(jupdate),
    .JRSTN(jrstn), .JCE1(jce1), .JCE2(jce2), .JRTI1(jrti1), .JRTI2(jrti2)
  );

  reg [25:0] hb = 0;
  always @(posedge clk_25mhz) hb <= hb + 1;
  assign led = {hb[25], 7'b0};
endmodule
