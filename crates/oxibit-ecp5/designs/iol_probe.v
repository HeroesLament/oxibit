// IO logic probe for the ULX3S, read by jtag_oracle.exs --design iol.
//
// Same JTAG frame as soft_probe.v (magic 0x10106000, tag 0xD0): x is
// written over TDI and latched on Update-DR; y is sampled at Capture-DR.
// Every IO register clocks on JTCK and takes its inputs from x, so a few
// JTCK edges after an update it shows x. Each pad is a loopback through
// its own BB: the IOLOGIC drives the pad and reads it back.
//
//   P1 gp18 (right, IOLOGIC A): OFS1P3DX out reg (D x0) and OFS1P3BX
//      tristate reg (D x1), both enabled by x20; IFS1P3DX reads the pad.
//      PULLMODE DOWN: tristated reads 0. y0 = in reg, y1 = pad in fabric.
//   P2 gn18 (right, B): ODDRX1F (D0 x2, D1 x3), IDDRX1F back: y2 Q0, y3 Q1.
//   P3 gp19 (right, A): fabric drives x4; DELAYG input delay, IFS1P3DX
//      after it (through DI): y4 = in reg, y5 = delay output in fabric.
//   P4 gn19 (right, B): DELAYF output delay from x5 (DEL_VALUE 20):
//      y6 = pad in fabric, y7 = CFLAG.
//   P5 gp20 (right, A): IFS1S1D latch, D = pad (x8), gate x9: y8.
//   P6 gn20 (right, B): IFS1P3IX, D = pad (x12), SP x11, sync CD x10: y9.
//   P7 gp22 (top, SIOLOGIC A): OFS1P3JX (D x13) and IFS1P3JX, sync PD x14
//      on both: y10 = in reg, y11 = pad in fabric.
//   P8 gn22 (top, SIOLOGIC B): ODDRX1F (D0 x16, D1 x17), IDDRX1F back:
//      y12 Q0, y13 Q1.
module iol_probe (
  input clk_25mhz,
  output [7:0] led,
  inout gp18, gn18, gp19, gn19, gp20, gn20, gp22, gn22
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
        sr <= {16'h0000, ncap, 8'hD0, y, x, 32'h10106000};
        ncap <= ncap + 1;
      end
    end else if (step) begin
      armed <= 1'b0;
      x <= rx;
    end
  end

  // P1: out + tristate registers, input register.
  wire p1_i, p1_t, p1_o;
  OFS1P3DX p1_oreg (.D(x[0]), .SP(x[20]), .SCLK(jtck), .CD(zero), .Q(p1_i));
  OFS1P3BX p1_treg (.D(x[1]), .SP(x[20]), .SCLK(jtck), .PD(zero), .Q(p1_t));
  BB p1_bb (.I(p1_i), .T(p1_t), .O(p1_o), .B(gp18));
  IFS1P3DX p1_ireg (.D(p1_o), .SP(one), .SCLK(jtck), .CD(zero), .Q(y[0]));
  assign y[1] = p1_o;

  // P2: DDR out and back.
  wire p2_i, p2_o;
  ODDRX1F p2_oddr (.D0(x[2]), .D1(x[3]), .SCLK(jtck), .RST(zero), .Q(p2_i));
  BB p2_bb (.I(p2_i), .T(zero), .O(p2_o), .B(gn18));
  IDDRX1F p2_iddr (.D(p2_o), .SCLK(jtck), .RST(zero), .Q0(y[2]), .Q1(y[3]));

  // P3: input delay, then the input register.
  wire p3_o, p3_z;
  BB p3_bb (.I(x[4]), .T(zero), .O(p3_o), .B(gp19));
  DELAYG #(.DEL_MODE("USER_DEFINED"), .DEL_VALUE(10)) p3_dly (.A(p3_o), .Z(p3_z));
  IFS1P3DX p3_ireg (.D(p3_z), .SP(one), .SCLK(jtck), .CD(zero), .Q(y[4]));
  assign y[5] = p3_z;

  // P4: output delay.
  wire p4_z, p4_o;
  DELAYF #(.DEL_VALUE(20)) p4_dly (.A(x[5]), .LOADN(zero), .MOVE(zero), .DIRECTION(zero),
    .Z(p4_z), .CFLAG(y[7]));
  BB p4_bb (.I(p4_z), .T(zero), .O(p4_o), .B(gn19));
  assign y[6] = p4_o;

  // P5: input latch.
  wire p5_o;
  BB p5_bb (.I(x[8]), .T(zero), .O(p5_o), .B(gp20));
  IFS1S1D p5_lat (.D(p5_o), .SCLK(x[9]), .CD(zero), .Q(y[8]));

  // P6: input register, enable and synchronous clear.
  wire p6_o;
  BB p6_bb (.I(x[12]), .T(zero), .O(p6_o), .B(gn20));
  IFS1P3IX p6_ireg (.D(p6_o), .SP(x[11]), .SCLK(jtck), .CD(x[10]), .Q(y[9]));

  // P7: top pad, synchronous preset on both registers.
  wire p7_i, p7_o;
  OFS1P3JX p7_oreg (.D(x[13]), .SP(one), .SCLK(jtck), .PD(x[14]), .Q(p7_i));
  BB p7_bb (.I(p7_i), .T(zero), .O(p7_o), .B(gp22));
  IFS1P3JX p7_ireg (.D(p7_o), .SP(one), .SCLK(jtck), .PD(x[14]), .Q(y[10]));
  assign y[11] = p7_o;

  // P8: top pad B, DDR out and back.
  wire p8_i, p8_o;
  ODDRX1F p8_oddr (.D0(x[16]), .D1(x[17]), .SCLK(jtck), .RST(zero), .Q(p8_i));
  BB p8_bb (.I(p8_i), .T(zero), .O(p8_o), .B(gn22));
  IDDRX1F p8_iddr (.D(p8_o), .SCLK(jtck), .RST(zero), .Q0(y[12]), .Q1(y[13]));

  assign y[31:14] = 18'h00000;

  JTAGG #(.ER1("ENABLED"), .ER2("ENABLED")) jtag (
    .JTDO1(sr[0]), .JTDO2(zero),
    .JTCK(jtck), .JTDI(jtdi), .JSHIFT(jshift), .JUPDATE(jupdate),
    .JRSTN(jrstn), .JCE1(jce1), .JCE2(jce2), .JRTI1(jrti1), .JRTI2(jrti2)
  );

  reg [25:0] hb = 0;
  always @(posedge clk_25mhz) hb <= hb + 1;
  assign led = {hb[25], 7'b0};
endmodule
