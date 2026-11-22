module pll_jtag (
  input clk_25mhz,
  input [1:0] btn,
  output [7:0] led,
  inout gp0
);
  wire clk_48, clk_fast, locked;
  wire zero = 1'b0;
  (* ICP_CURRENT="12" *) (* LPF_RESISTOR="8" *) (* MFG_ENABLE_FILTEROPAMP="1" *) (* MFG_GMCREF_SEL="2" *)
  EHXPLLL #(
    .CLKI_DIV(25), .CLKFB_DIV(48), .CLKOP_DIV(9), .CLKOS_DIV(36),
    .CLKOS2_DIV(8), .CLKOS3_DIV(8),
    .CLKOP_ENABLE("ENABLED"), .CLKOS_ENABLE("ENABLED"),
    .CLKOS2_ENABLE("DISABLED"), .CLKOS3_ENABLE("DISABLED"),
    .CLKOP_CPHASE(8), .CLKOS_CPHASE(35), .CLKOS2_CPHASE(0), .CLKOS3_CPHASE(0),
    .CLKOP_FPHASE(0), .CLKOS_FPHASE(0), .CLKOS2_FPHASE(0), .CLKOS3_FPHASE(0),
    .FEEDBK_PATH("CLKOP"),
    .CLKOP_TRIM_POL("RISING"), .CLKOP_TRIM_DELAY(0),
    .CLKOS_TRIM_POL("RISING"), .CLKOS_TRIM_DELAY(0),
    .OUTDIVIDER_MUXA("DIVA"), .OUTDIVIDER_MUXB("DIVB"),
    .OUTDIVIDER_MUXC("DIVC"), .OUTDIVIDER_MUXD("DIVD"),
    .PLL_LOCK_MODE(0), .PLL_LOCK_DELAY(200),
    .STDBY_ENABLE("DISABLED"), .REFIN_RESET("DISABLED"), .SYNC_ENABLE("DISABLED"),
    .INT_LOCK_STICKY("ENABLED"), .DPHASE_SOURCE("ENABLED"),
    .PLLRST_ENA("DISABLED"), .INTFB_WAKE("DISABLED")
  ) pll (
    .CLKI(clk_25mhz), .CLKFB(clk_48),
    .PHASESEL1(zero), .PHASESEL0(zero), .PHASEDIR(zero), .PHASESTEP(zero),
    .PHASELOADREG(zero), .STDBY(zero), .PLLWAKESYNC(zero), .RST(zero),
    .ENCLKOP(zero), .ENCLKOS(zero), .ENCLKOS2(zero), .ENCLKOS3(zero),
    .CLKOP(clk_48), .CLKOS(clk_fast), .LOCK(locked)
  );

  reg [31:0] c25 = 0;
  reg [31:0] c48 = 0;
  always @(posedge clk_25mhz) c25 <= c25 + 1;
  always @(posedge clk_48) c48 <= c48 + 1;

  wire gp0_in;
  BB gp0_bb (
    .I(c25[22]),
    .T(~btn[1]),
    .B(gp0),
    .O(gp0_in)
  );

  assign led = {gp0_in, locked, c48[25:23], c25[25:23]};

  // JTAG oracle: ER1 (IR 0x32) is a 128-bit DR, LSB first.
  //   [31:0]    magic 0x0B1BEC55
  //   [63:32]   25 MHz counter, Gray coded
  //   [95:64]   48 MHz PLL counter, Gray coded
  //   [96]      PLL lock      [97]  gp0 pad in  [98] btn[1]  [99] btn[0]
  //   [103:100] 4'hA          [111:104] capture count (JTCK domain)
  //   [127:112] last value shifted in at [127:112], latched on Update-DR
  reg [31:0] g25 = 0;
  reg [31:0] g48 = 0;
  always @(posedge clk_25mhz) g25 <= c25 ^ (c25 >> 1);
  always @(posedge clk_48) g48 <= c48 ^ (c48 >> 1);

  wire jtck, jtdi, jshift, jupdate, jrstn, jce1, jce2, jrti1, jrti2;
  reg [127:0] sr = 0;
  reg [7:0] ncap = 0;
  reg [15:0] user = 0;
  reg armed = 0;
  wire [31:0] status = {user, ncap, 4'hA, ~btn[0], btn[1], gp0_in, locked};
  // The TAP state outputs change just after the JTCK rising edge and reach
  // the fabric by general routing, JTCK by a longer path: sampled on the
  // falling edge, the rising edge sees the state of the cycle just ended.
  // JTDI is the TDI pin, which the host changes on the falling edge.
  reg ce1 = 0, shift = 0, upd = 0;
  always @(negedge jtck) begin
    ce1 <= jce1;
    shift <= jshift;
    upd <= jupdate;
  end
  always @(posedge jtck) begin
    if (ce1) begin
      armed <= 1'b1;
      if (shift) sr <= {jtdi, sr[127:1]};
      else begin
        sr <= {status, g48, g25, 32'h0B1BEC55};
        ncap <= ncap + 1;
      end
    end else if (upd && armed) begin
      armed <= 1'b0;
      user <= sr[127:112];
    end
  end
  JTAGG #(.ER1("ENABLED"), .ER2("ENABLED")) jtag (
    .JTDO1(sr[0]), .JTDO2(zero),
    .JTCK(jtck), .JTDI(jtdi), .JSHIFT(jshift), .JUPDATE(jupdate),
    .JRSTN(jrstn), .JCE1(jce1), .JCE2(jce2), .JRTI1(jrti1), .JRTI2(jrti2)
  );
endmodule
