module pll_bb_top (
  clk_25mhz,
  btn1,
  led,
  gp0
);
  input clk_25mhz;
  input btn1;
  output [7:0] led;
  inout gp0;

  // Internal wires
  wire [7:0] _cat_6;
  wire [25:0] _add_5;
  wire _slice_4;
  wire [2:0] _slice_3;
  reg [25:0] c48_c;
  wire [25:0] _add_2;
  wire _slice_1;
  wire [2:0] _slice_0;
  reg [25:0] c25_c;
  wire pll_pll_clkintfb;
  wire pll_pll_refclk;
  wire pll_pll_intlock;
  wire pll_pll_clkos3;
  wire pll_pll_clkos2;
  wire pll_phase_step;
  wire pll_phase_dir;
  wire pll_zero;
  wire gp0_in;
  wire tap48;
  wire tap25;
  wire [2:0] t48;
  wire [2:0] t25;
  wire pll_locked;
  wire clk_12;
  (* keep *) wire clk_48;
  // Initial values
  initial c48_c = 26'd0;
  initial c25_c = 26'd0;

  // Combinational logic
  assign led = _cat_6;
  assign _cat_6 = {gp0_in, pll_locked, t48, t25};
  assign _add_5 = c48_c + 26'd1;
  assign tap48 = _slice_4;
  assign _slice_4 = c48_c[22:22];
  assign t48 = _slice_3;
  assign _slice_3 = c48_c[25:23];
  assign _add_2 = c25_c + 26'd1;
  assign tap25 = _slice_1;
  assign _slice_1 = c25_c[22:22];
  assign t25 = _slice_0;
  assign _slice_0 = c25_c[25:23];
  assign pll_phase_step = 1'd0;
  assign pll_phase_dir = 1'd0;
  assign pll_zero = 1'd0;

  // Sequential logic
  always @(posedge clk_25mhz) begin
    c25_c <= _add_2;
  end
  always @(posedge clk_48) begin
    c48_c <= _add_5;
  end
  // Module instances
  (* ICP_CURRENT="12" *) (* LPF_RESISTOR="8" *) (* MFG_ENABLE_FILTEROPAMP="1" *) (* MFG_GMCREF_SEL="2" *)
  EHXPLLL #(
    .CLKI_DIV(25),
    .CLKFB_DIV(48),
    .CLKOP_DIV(9),
    .CLKOS_DIV(36),
    .CLKOS2_DIV(8),
    .CLKOS3_DIV(8),
    .CLKOP_ENABLE("ENABLED"),
    .CLKOS_ENABLE("ENABLED"),
    .CLKOS2_ENABLE("DISABLED"),
    .CLKOS3_ENABLE("DISABLED"),
    .CLKOP_CPHASE(8),
    .CLKOS_CPHASE(35),
    .CLKOS2_CPHASE(0),
    .CLKOS3_CPHASE(0),
    .CLKOP_FPHASE(0),
    .CLKOS_FPHASE(0),
    .CLKOS2_FPHASE(0),
    .CLKOS3_FPHASE(0),
    .FEEDBK_PATH("CLKOP"),
    .CLKOP_TRIM_POL("RISING"),
    .CLKOP_TRIM_DELAY(0),
    .CLKOS_TRIM_POL("RISING"),
    .CLKOS_TRIM_DELAY(0),
    .OUTDIVIDER_MUXA("DIVA"),
    .OUTDIVIDER_MUXB("DIVB"),
    .OUTDIVIDER_MUXC("DIVC"),
    .OUTDIVIDER_MUXD("DIVD"),
    .PLL_LOCK_MODE(0),
    .PLL_LOCK_DELAY(200),
    .STDBY_ENABLE("DISABLED"),
    .REFIN_RESET("DISABLED"),
    .SYNC_ENABLE("DISABLED"),
    .INT_LOCK_STICKY("ENABLED"),
    .DPHASE_SOURCE("ENABLED"),
    .PLLRST_ENA("DISABLED"),
    .INTFB_WAKE("DISABLED")
  ) pll_pll_pll (
    .CLKI(clk_25mhz),
    .CLKFB(clk_48),
    .PHASESEL1(pll_zero),
    .PHASESEL0(pll_zero),
    .PHASEDIR(pll_phase_dir),
    .PHASESTEP(pll_phase_step),
    .PHASELOADREG(pll_zero),
    .STDBY(pll_zero),
    .PLLWAKESYNC(pll_zero),
    .RST(pll_zero),
    .ENCLKOP(pll_zero),
    .ENCLKOS(pll_zero),
    .ENCLKOS2(pll_zero),
    .ENCLKOS3(pll_zero),
    .CLKOP(clk_48),
    .CLKOS(clk_12),
    .CLKOS2(pll_pll_clkos2),
    .CLKOS3(pll_pll_clkos3),
    .LOCK(pll_locked),
    .INTLOCK(pll_pll_intlock),
    .REFCLK(pll_pll_refclk),
    .CLKINTFB(pll_pll_clkintfb)
  );
  // Tristate buffers
  BB gp0_bb (
    .I(tap25),
    .T(~btn1),
    .B(gp0),
    .O(gp0_in)
  );

endmodule
