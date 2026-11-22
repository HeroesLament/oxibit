// PLL + bidirectional pad test for the ULX3S, written the way EHDL emits
// ULX3S.PLL (Hw.ECP5.EHXPLLL) and a tristate (Hw.ECP5.BB).
//   led[2:0]  25 MHz counter, bits 25..23
//   led[5:3]  48 MHz PLL counter, bits 25..23 (about 1.9x faster)
//   led[6]    PLL lock
//   led[7]    gp0 read back through its BB: pulled up while released,
//             the 25 MHz counter's bit 22 while btn[1] is held
module pll_bb (
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

  reg [25:0] c25 = 0;
  reg [25:0] c48 = 0;
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
endmodule
