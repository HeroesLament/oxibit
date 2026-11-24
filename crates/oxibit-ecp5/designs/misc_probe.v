// Edge-clock bridge, OSHX2A and SGSR probe for the ULX3S, read by
// jtag_oracle.exs --design misc.
//
// 256-bit ER1 frame, LSB first: magic 0x415C0000, x, then gray-coded
// counters c25 (clk_25mhz) and cbr (the bridged system clock), 32'h0,
// y = {30'h0, gn20 high seen, gn20 low seen}, then {16'h0, ncap, tag 0xE1}.
// x is the last 32 bits shifted in, latched on Update-DR.
//   ECLKBRIDGECS: CLK0 clk_25mhz, CLK1 OSCG (DIV 26, about 12 MHz),
//     SEL x0; ECSOUT through the bridge ECLKSYNCB is the edge clock, its
//     CLKDIVF 2.0 the system clock that cbr counts: 12.5 MHz with SEL 0,
//     about 6 MHz with SEL 1.
//   OSHX2A on gn20: D0 x1, D1 x2 on that edge clock. The pad input is
//     sampled on JTCK (sticky between captures): 00 reads low only, 11
//     high only, 01 / 10 both.
//   SGSR: GSR ~x3 on clk_25mhz (every register with GSR enabled resets, x
//     included, so the pulse ends itself).
module misc_probe (
  input clk_25mhz,
  output [7:0] led,
  inout gn20
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

  reg [255:0] sr = 0;
  reg [31:0] x = 0;
  reg [7:0] ncap = 0;
  reg armed = 0;
  wire step = upd & armed;
  wire [31:0] g25, gbr, y;

  // The gn20 pad sampled on JTCK.
  wire pad_o;
  reg [1:0] pz_acc = 0;
  reg pad_s = 0;
  always @(posedge jtck) begin
    pad_s <= pad_o;
    if (ce1 & ~shift) pz_acc <= 0;
    else pz_acc <= pz_acc | {pad_s, ~pad_s};
  end

  always @(posedge jtck) begin
    if (ce1) begin
      armed <= 1'b1;
      if (shift) sr <= {1'b0, sr[255:1]};
      else begin
        sr <= {16'h0000, ncap, 8'hE1, y, 32'h0, gbr, g25, x, 32'h415C0000};
        ncap <= ncap + 1;
      end
    end else if (step) begin
      armed <= 1'b0;
      x <= rx;
    end
  end

  // The bridge.
  wire osc;
  OSCG #(.DIV(26)) oscg (.OSC(osc));
  wire ecs, beclk, bsclk;
  ECLKBRIDGECS bridge (.CLK0(clk_25mhz), .CLK1(osc), .SEL(x[0]), .ECSOUT(ecs));
  ECLKSYNCB bsync (.ECLKI(ecs), .STOP(zero), .ECLKO(beclk));
  CLKDIVF #(.DIV("2.0")) bdiv (.CLKI(beclk), .RST(zero), .ALIGNWD(zero), .CDIVX(bsclk));

  // OSHX2A on the bridged edge clock.
  wire q;
  OSHX2A osh (.D0(x[1]), .D1(x[2]), .SCLK(bsclk), .ECLK(beclk), .RST(zero), .Q(q));
  BB pad (.I(q), .T(zero), .O(pad_o), .B(gn20));
  assign y = {30'h0, pz_acc};

  SGSR sgsr (.GSR(~x[3]), .CLK(clk_25mhz));

  // Gray-coded counters.
  reg [31:0] b25 = 0, bbr = 0, r25 = 0, rbr = 0;
  always @(posedge clk_25mhz) b25 <= b25 + 1;
  always @(posedge bsclk) bbr <= bbr + 1;
  always @(posedge clk_25mhz) r25 <= b25 ^ (b25 >> 1);
  always @(posedge bsclk) rbr <= bbr ^ (bbr >> 1);
  assign g25 = r25, gbr = rbr;

  JTAGG #(.ER1("ENABLED"), .ER2("ENABLED")) jtag (
    .JTDO1(sr[0]), .JTDO2(zero),
    .JTCK(jtck), .JTDI(jtdi), .JSHIFT(jshift), .JUPDATE(jupdate),
    .JRSTN(jrstn), .JCE1(jce1), .JCE2(jce2), .JRTI1(jrti1), .JRTI2(jrti2)
  );

  reg [25:0] hb = 0;
  always @(posedge clk_25mhz) hb <= hb + 1;
  assign led = {hb[25], 7'b0};
endmodule
