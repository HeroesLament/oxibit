// IO gearing probe for the ULX3S, read by jtag_oracle.exs --design gear.
//
// Same 128-bit JTAG frame as iol_probe.v (magic 0x6EA20000, tag 0xF0).
// The 25 MHz pad drives two ECLKSYNCBs (STOP x0); each edge clock feeds a
// CLKDIVF (DIV 2.0, DIV 3.5: CLKDIV<n> only takes bank ECLK<n>) and the
// IOLOGIC of one bank 7 pad, a loopback through its own BB:
//   gp9  (A2, R35C0 A): ODDRX2F D0..D3 = x1..x4, IDDRX2F back: y0..y3.
//   gp10 (C4, R14C0 A): ODDR71B D0..D6 = x5..x11, IDDR71B back: y4..y10.
// ALIGNWD of the two IDDRs is x13 (X2) and x12 (7:1).
module gear_probe (
  input clk_25mhz,
  output [7:0] led,
  inout gp9, gp10
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
        sr <= {16'h0000, ncap, 8'hF0, y, x, 32'h6EA20000};
        ncap <= ncap + 1;
      end
    end else if (step) begin
      armed <= 1'b0;
      x <= rx;
    end
  end

  // Edge clock and its two divided clocks.
  wire eclk2, eclk7, sclk2, sclk7;
  ECLKSYNCB sync2 (.ECLKI(clk_25mhz), .STOP(x[0]), .ECLKO(eclk2));
  ECLKSYNCB sync7 (.ECLKI(clk_25mhz), .STOP(x[0]), .ECLKO(eclk7));
  CLKDIVF #(.DIV("2.0")) div2 (.CLKI(eclk2), .RST(zero), .ALIGNWD(zero), .CDIVX(sclk2));
  CLKDIVF #(.DIV("3.5")) div7 (.CLKI(eclk7), .RST(zero), .ALIGNWD(zero), .CDIVX(sclk7));

  // X2 loopback.
  wire g9_i, g9_o;
  ODDRX2F o2 (.SCLK(sclk2), .ECLK(eclk2), .RST(zero), .D0(x[1]), .D1(x[2]), .D2(x[3]),
    .D3(x[4]), .Q(g9_i));
  BB g9_bb (.I(g9_i), .T(zero), .O(g9_o), .B(gp9));
  IDDRX2F i2 (.D(g9_o), .SCLK(sclk2), .ECLK(eclk2), .RST(zero), .ALIGNWD(x[13]),
    .Q0(y[0]), .Q1(y[1]), .Q2(y[2]), .Q3(y[3]));

  // 7:1 loopback.
  wire g10_i, g10_o;
  ODDR71B o7 (.SCLK(sclk7), .ECLK(eclk7), .RST(zero), .D0(x[5]), .D1(x[6]), .D2(x[7]),
    .D3(x[8]), .D4(x[9]), .D5(x[10]), .D6(x[11]), .Q(g10_i));
  BB g10_bb (.I(g10_i), .T(zero), .O(g10_o), .B(gp10));
  IDDR71B i7 (.D(g10_o), .SCLK(sclk7), .ECLK(eclk7), .RST(zero), .ALIGNWD(x[12]),
    .Q0(y[4]), .Q1(y[5]), .Q2(y[6]), .Q3(y[7]), .Q4(y[8]), .Q5(y[9]), .Q6(y[10]));

  assign y[31:11] = 21'h000000;

  JTAGG #(.ER1("ENABLED"), .ER2("ENABLED")) jtag (
    .JTDO1(sr[0]), .JTDO2(zero),
    .JTCK(jtck), .JTDI(jtdi), .JSHIFT(jshift), .JUPDATE(jupdate),
    .JRSTN(jrstn), .JCE1(jce1), .JCE2(jce2), .JRTI1(jrti1), .JRTI2(jrti2)
  );

  reg [25:0] hb = 0;
  always @(posedge clk_25mhz) hb <= hb + 1;
  assign led = {hb[25], 7'b0};
endmodule
