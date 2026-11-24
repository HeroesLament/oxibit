// Wide inferred-multiplier probe for the ULX3S, read by jtag_oracle.exs
// --design mulw. No DSP primitive is instantiated: each `*` is split over
// 18-bit chunk MULT18X18Ds and adders.
//
// 256-bit ER1 frame, LSB first: magic 0x3A1D0000, x[63:0], pw[63:0],
// {10'h0, sw}, tag 0xD5, ncap, 16'h0. x is the last 64 bits shifted in,
// latched on Update-DR.
//   pw = x[31:0] * x[63:32]                        32 x 32 unsigned, 64 bits
//   sw = $signed(x[23:0]) * $signed(x[53:24])      24 x 30 signed, 54 bits
module mulw_probe (
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

  wire [63:0] pw = x[31:0] * x[63:32];
  wire signed [53:0] sw = $signed(x[23:0]) * $signed(x[53:24]);

  always @(posedge jtck) begin
    if (ce1) begin
      armed <= 1'b1;
      if (shift) sr <= {1'b0, sr[255:1]};
      else begin
        sr <= {16'h0, ncap, 8'hD5, 10'h0, sw, pw, x, 32'h3A1D0000};
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
