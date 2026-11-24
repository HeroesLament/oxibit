// Inferred-multiplier probe for the ULX3S, read by jtag_oracle.exs
// --design mul. No DSP primitive is instantiated: each `*` must become a
// MULT18X18D.
//
// 256-bit ER1 frame, LSB first: magic 0x3A170000, x[63:0], {3'h0, pu},
// {30'h0, ps}, {16'h0, pr}, 16'h0, tag 0xD5, ncap, 16'h0. x is the last
// 64 bits shifted in, latched on Update-DR.
//   pu = x[11:0] * x[28:12]                        12 x 17 unsigned, 29 bits
//   ps = $signed(x[45:30]) * $signed(x[63:46])     16 x 18 signed, 34 bits
//   pr <= x[7:0] * x[15:8] on JTCK                 8 x 8 into a fabric flop
module mul_probe (
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

  wire [28:0] pu = x[11:0] * x[28:12];
  wire signed [33:0] ps = $signed(x[45:30]) * $signed(x[63:46]);
  reg [15:0] pr = 0;
  always @(posedge jtck) pr <= x[7:0] * x[15:8];

  always @(posedge jtck) begin
    if (ce1) begin
      armed <= 1'b1;
      if (shift) sr <= {1'b0, sr[255:1]};
      else begin
        sr <= {16'h0, ncap, 8'hD5, 16'h0, pr, 30'h0, ps, 3'h0, pu, x, 32'h3A170000};
        ncap <= ncap + 1;
      end
    end else if (step) begin
      armed <= 1'b0;
      x <= rx;
    end
  end

  JTAGG #(.ER1("ENABLED"), .ER2("ENABLED")) jtag (
    .JTDO1(sr[0]), .JTDO2(zero),
    .JTCK(jtck), .JTDI(jtdi), .JSHIFT(jshift), .JUPDATE(jupdate),
    .JRSTN(jrstn), .JCE1(jce1), .JCE2(jce2), .JRTI1(jrti1), .JRTI2(jrti2)
  );

  reg [25:0] hb = 0;
  always @(posedge clk_25mhz) hb <= hb + 1;
  assign led = {hb[25], 7'b0};
endmodule
