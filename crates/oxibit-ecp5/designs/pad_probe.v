// Pad buffer probe for the ULX3S, read by jtag_oracle.exs --design pad,
// with pad_probe.lpf (no PULLMODE but gp19's, so the buffers' own pull
// modes apply). Each pad reads itself back; nothing is wired to them.
//
// 256-bit ER1 frame, LSB first: magic 0xFADE0000, x[63:0], y[3:0],
// 28'h0, tag 0xFD, ncap. x is the last 64 bits shifted in, latched on
// Update-DR.
//   y0 gp18 BBPU: I x0, T x1. Released (T=1) it reads 1, driven x0.
//   y1 gn18 BBPD: I x2, T x3. Released it reads 0, driven x2.
//   y2 gp19 TRELLIS_IO BIDIR, LPF OPENDRAIN=ON PULLMODE=DOWN: I x4, T 0.
//      Open drain lets the pad go on a 1, so it reads 0 whatever x4 is.
//   y3 gn19 IBPU: reads 1.
module pad_probe (
  input clk_25mhz,
  output [7:0] led,
  inout gp18, gn18, gp19,
  input gn19
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
  wire [3:0] y;

  always @(posedge jtck) begin
    if (ce1) begin
      armed <= 1'b1;
      if (shift) sr <= {1'b0, sr[255:1]};
      else begin
        sr <= {136'h0, ncap, 8'hFD, 28'h0, y, x, 32'hFADE0000};
        ncap <= ncap + 1;
      end
    end else if (step) begin
      armed <= 1'b0;
      x <= rx;
    end
  end

  BBPU q0 (.I(x[0]), .T(x[1]), .O(y[0]), .B(gp18));
  BBPD q1 (.I(x[2]), .T(x[3]), .O(y[1]), .B(gn18));
  TRELLIS_IO #(.DIR("BIDIR")) q2 (.I(x[4]), .T(zero), .O(y[2]), .B(gp19));
  IBPU q3 (.I(gn19), .O(y[3]));

  JTAGG #(.ER1("ENABLED"), .ER2("ENABLED")) jtag (
    .JTDO1(sr[0]), .JTDO2(zero),
    .JTCK(jtck), .JTDI(jtdi), .JSHIFT(jshift), .JUPDATE(jupdate),
    .JRSTN(jrstn), .JCE1(jce1), .JCE2(jce2), .JRTI1(jrti1), .JRTI2(jrti2)
  );

  reg [25:0] hb = 0;
  always @(posedge clk_25mhz) hb <= hb + 1;
  assign led = {hb[25], 7'b0};
endmodule
