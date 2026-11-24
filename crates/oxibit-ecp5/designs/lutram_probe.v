// Distributed-RAM probe for the ULX3S, read by jtag_oracle.exs --design lutram.
//
// Same JTAG frame as soft_probe.v (magic 0x1D7A3000, tag 0xC0): x is written
// over TDI and latched on Update-DR; the three RAMs write on the JTCK edge
// of that update, from the x before it. y[11:0] are the RAM outputs.
//   A: DPR16X4C, scalar pins: WAD x[3:0], DI x[7:4], WRE x[8], RAD x[12:9]
//   B: SPR16X4C: AD x[16:13], DI x[20:17], WRE x[21]
//   C: TRELLIS_DPR16X4, vector pins: WAD x[25:22], DI x[29:26], WRE x[30],
//      RAD x[3:0]
module lutram_probe (
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
  // JTDI is TDI as the TAP sampled it on the rising edge (it changes just
  // after that edge), so the bit a shift edge takes in is in `tdi` one
  // edge later: the last 32 bits shifted in collect in `rx` one edge
  // behind the shifts out of `sr`.
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
        sr <= {16'h0000, ncap, 8'hC0, y, x, 32'h1D7A3000};
        ncap <= ncap + 1;
      end
    end else if (step) begin
      armed <= 1'b0;
      x <= rx;
    end
  end

  wire we_a = x[8] & step, we_b = x[21] & step, we_c = x[30] & step;

  DPR16X4C #(.INITVAL("0x0123456789ABCDEF")) ram_a (
    .DI0(x[4]), .DI1(x[5]), .DI2(x[6]), .DI3(x[7]),
    .WAD0(x[0]), .WAD1(x[1]), .WAD2(x[2]), .WAD3(x[3]),
    .WCK(jtck), .WRE(we_a),
    .RAD0(x[9]), .RAD1(x[10]), .RAD2(x[11]), .RAD3(x[12]),
    .DO0(y[0]), .DO1(y[1]), .DO2(y[2]), .DO3(y[3])
  );
  SPR16X4C #(.INITVAL("0xFEDCBA9876543210")) ram_b (
    .DI0(x[17]), .DI1(x[18]), .DI2(x[19]), .DI3(x[20]),
    .AD0(x[13]), .AD1(x[14]), .AD2(x[15]), .AD3(x[16]),
    .CK(jtck), .WRE(we_b),
    .DO0(y[4]), .DO1(y[5]), .DO2(y[6]), .DO3(y[7])
  );
  TRELLIS_DPR16X4 #(.INITVAL(64'h5A5AC3C30F0F9696)) ram_c (
    .DI(x[29:26]), .WAD(x[25:22]), .WRE(we_c), .WCK(jtck),
    .RAD(x[3:0]), .DO(y[11:8])
  );
  assign y[31:12] = 20'h00000;

  JTAGG #(.ER1("ENABLED"), .ER2("ENABLED")) jtag (
    .JTDO1(sr[0]), .JTDO2(zero),
    .JTCK(jtck), .JTDI(jtdi), .JSHIFT(jshift), .JUPDATE(jupdate),
    .JRSTN(jrstn), .JCE1(jce1), .JCE2(jce2), .JRTI1(jrti1), .JRTI2(jrti2)
  );

  reg [25:0] hb = 0;
  always @(posedge clk_25mhz) hb <= hb + 1;
  assign led = {hb[25], 7'b0};
endmodule
