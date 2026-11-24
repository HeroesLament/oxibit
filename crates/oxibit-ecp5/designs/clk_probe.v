// Clock and configuration-block probe for the ULX3S, read by
// jtag_oracle.exs --design clk.
//
// 256-bit ER1 frame, LSB first: magic 0xC10C0000, x, then gray-coded
// counters c25 (clk_25mhz), cosc (OSCG), cgate (clk_25mhz through a DCCA
// gated by x0), cdcs (DCSC: CLK0 clk_25mhz, CLK1 OSCG, SEL0 x1, SEL1 x2,
// MODESEL x7), then y = {SED done/inprog/err, DTROUT}, then {16'h0, ncap,
// tag 0xE0}. x is the last 32 bits shifted in, latched on Update-DR.
//   x5: SEDGA SEDENABLE, x6: SEDSTART (SED clock 38.8 MHz). SEDGA and
//       OSCG share the oscillator; the OSCG rate is checked with both in.
//   x3: GSR low (every register with GSR enabled resets, x included, so
//       the pulse ends itself).
//   x4: DTR STARTPULSE.
module clk_probe (
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
  wire [31:0] g25, gosc, ggate, gdcs, y;

  always @(posedge jtck) begin
    if (ce1) begin
      armed <= 1'b1;
      if (shift) sr <= {1'b0, sr[255:1]};
      else begin
        sr <= {16'h0000, ncap, 8'hE0, y, gdcs, ggate, gosc, g25, x, 32'hC10C0000};
        ncap <= ncap + 1;
      end
    end else if (step) begin
      armed <= 1'b0;
      x <= rx;
    end
  end

  // Clocks.
  wire osc, gated, dcs;
  OSCG #(.DIV(26)) oscg (.OSC(osc));
  DCCA dcc (.CLKI(clk_25mhz), .CE(x[0]), .CLKO(gated));
  DCSC #(.DCSMODE("POS")) dcsc (.CLK0(clk_25mhz), .CLK1(osc), .SEL0(x[1]), .SEL1(x[2]),
    .MODESEL(x[7]), .DCSOUT(dcs));
  GSR gsr (.GSR(~x[3]));

  // A gray-coded counter per clock.
  reg [31:0] b25 = 0, bosc = 0, bgate = 0, bdcs = 0;
  always @(posedge clk_25mhz) b25 <= b25 + 1;
  always @(posedge osc) bosc <= bosc + 1;
  always @(posedge gated) bgate <= bgate + 1;
  always @(posedge dcs) bdcs <= bdcs + 1;
  reg [31:0] r25 = 0, rosc = 0, rgate = 0, rdcs = 0;
  always @(posedge clk_25mhz) r25 <= b25 ^ (b25 >> 1);
  always @(posedge osc) rosc <= bosc ^ (bosc >> 1);
  always @(posedge gated) rgate <= bgate ^ (bgate >> 1);
  always @(posedge dcs) rdcs <= bdcs ^ (bdcs >> 1);
  assign g25 = r25, gosc = rosc, ggate = rgate, gdcs = rdcs;

  // Die temperature and soft-error detection.
  wire [7:0] dtr;
  DTR dtr_i (.STARTPULSE(x[4]), .DTROUT7(dtr[7]), .DTROUT6(dtr[6]), .DTROUT5(dtr[5]),
    .DTROUT4(dtr[4]), .DTROUT3(dtr[3]), .DTROUT2(dtr[2]), .DTROUT1(dtr[1]), .DTROUT0(dtr[0]));
  wire sed_done, sed_inprog, sed_err;
  SEDGA #(.SED_CLK_FREQ("38.8"), .CHECKALWAYS("DISABLED")) sed (.SEDENABLE(x[5]),
    .SEDSTART(x[6]), .SEDFRCERR(zero), .SEDDONE(sed_done), .SEDINPROG(sed_inprog),
    .SEDERR(sed_err));
  assign y = {21'h0, sed_err, sed_inprog, sed_done, dtr};

  JTAGG #(.ER1("ENABLED"), .ER2("ENABLED")) jtag (
    .JTDO1(sr[0]), .JTDO2(zero),
    .JTCK(jtck), .JTDI(jtdi), .JSHIFT(jshift), .JUPDATE(jupdate),
    .JRSTN(jrstn), .JCE1(jce1), .JCE2(jce2), .JRTI1(jrti1), .JRTI2(jrti2)
  );

  reg [25:0] hb = 0;
  always @(posedge clk_25mhz) hb <= hb + 1;
  assign led = {hb[25], 7'b0};
endmodule
