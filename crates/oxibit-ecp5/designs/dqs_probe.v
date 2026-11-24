// DDR (DQS group) probe for the ULX3S, read by jtag_oracle.exs --design dqs
// (dqsm_probe: the strobe tristate T0 / T1 from fabric logic; dqs_probe:
// T0 / T1 tied low, as in p27, so x24 burst mode has no effect there).
// One byte lane built as LiteDRAM's ECP5DDRPHY builds it (the open design
// proven on ECP5 silicon), with its ECP5DDRPHYInit sequence in fabric.
// Group R17 (bank 2): gp19 (R17C126 A) is its DQS pad, gp20 (R14C126 A) a
// DQ pad. Each loops back through its own BB, nothing is wired to them.
//
// 256-bit ER1 frame, LSB first: magic 0xD9500000, x[63:0], y[15:0],
// {12'h0, pz}, tag 0xD5, ncap. x is the last 64 bits shifted in, latched
// on Update-DR.
//
// Clocks (as LiteX's ECP5 DDR CRG): an EHXPLLL makes sys2x = 200 MHz from
// the 25 MHz pad; ECLKSYNCB (STOP = init stop) is the edge clock, CLKDIVF
// 2.0 (RST = init reset) the 100 MHz sys clock. DDRDLLA on the edge clock.
//
// Init (clk_25mhz, steps 8 cycles apart, from the rising edge of DLL LOCK
// or of x16): freeze the DLL, stop ECLK, reset CLKDIVF (the DQS-group RSTs
// stay low: LiteDRAM 1:2 resets them from sys reset, io_rst_init False),
// release reset, release stop, unfreeze, pause the DQSBUFM,
// update the DLL (UDDCNTLN low), end update, release pause.
//
// DQSBUFM as LiteDRAM, except the delays PLUS 0 (dqsm_probe has LiteDRAM's
// DQS_LI_DEL MINUS 1, DQS_LO_DEL MINUS 4): RDLOADN /
// WRLOADN 0, RDMOVE / WRMOVE 0, RDDIRECTION / WRDIRECTION 1 (with x[43:40] 0;
// RDLOADN x40, WRLOADN x41, RDDIRECTION ~x42, WRDIRECTION ~x43), READ0 / READ1
// the read pulse, READCLKSEL x[14:12].
//   gp19: ODDRX2DQSB D = 0,1,0,1 (x24 = 0) or 0,b,0,b with b high on two sys
//         cycles of 16 (x24 = 1: an idle-low burst, as a memory drives a
//         read), DQSW; TSHX2DQSA T0 = ~(oe | postamble),
//         T1 = ~(oe | preamble); oe always with x24 = 0, else a burst window
//         on sys frame cycles 7, 8 (preamble cycle 6, postamble cycle 9).
//   gp20: ODDRX2DQA D0..3 = x0..x3 (DQSW270), TSHX2DQA T = ~dq_oe (dq_oe
//         always), read back into IDDRX2DQA (no DELAYG, see below).
//   READ pulse (p35): x8 on sys frame cycles rds, rds + 1 (x45: also rds + 2,
//   rds + 3), rds = x[23:20]; PAUSE = init pause | x44 (the oracle raises x44
//   around READCLKSEL / rds changes, never in a measurement); x46 holds the
//   DQSBUFM RST through the init sequence. Scan word bits 126 / 127: live
//   dqs_rst / pause_all.
//   p36: x44 is synchronised to sclk; with x47 its release waits for frame
//   cycle x[51:48]. Scan word bits 144.. (16 each): DATAVALID cycles,
//   DATAVALID rises, BURSTDET rises, RDCFLAG rises, WRCFLAG rises (all Gray),
//   sclk cycles from the last PAUSE release to the first DATAVALID rise
//   (FFFF none); bits 240..255 PAUSE releases (Gray).
//   p37: x52 raises PAUSE for x[63:60] sclk cycles from frame cycle
//   x[59:56], on one frame in 2^x[55:53].
//   p38: RDMOVE = x17 (one step of the read-side DQS delay per pulse,
//   with RDLOADN x40 high; RDDIRECTION ~x42).
//   p39: WRMOVE = x18 (one step of the write-side DQSW270 delay, which
//   clocks DQ out, per pulse, with WRLOADN x41 high; WRDIRECTION ~x43).
//   p40: scan bits 224..239 hold the DQ word 1, 2, 3, 4 sclk cycles after
//   the last DATAVALID rise (4 bits each), in place of the release latency.
//
// y (results over a 2^18-cycle sys window, about 2.6 ms, then cleared):
//   y0..y3 Q0..Q3 on the last DATAVALID cycle, y4 QWL then, y5 DDRDLLA LOCK,
//   y6 DATAVALID seen, y7 BURSTDET seen, y8 RDCFLAG seen, y9 WRCFLAG seen,
//   y10..y13 Q0..Q3 seen high, y14 PLL LOCK, y15 init sequence done.
// pz: the DQS pad low, DQS high, DQ low, DQ high seen on JTCK samples since
//   the last capture (pads pulled up in dqs_probe.lpf: a released pad reads
//   high).
module dqs_probe (
  input clk_25mhz,
  output [7:0] led,
  inout gp19, gp20
);
  wire jtck, jtdi, jshift, jupdate, jrstn, jce1, jce2, jrti1, jrti2;
  wire zero = 1'b0, one = 1'b1;

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
  wire pause_all, dqs_rst; // p35: assigned below, read back in the scan word
  // p36: event counters (sclk domain, Gray coded) and the release latency.
  wire [15:0] g_dvc, g_dvr, g_bdr, g_rdc, g_wrc, lat_w;
  wire [15:0] g_pr;
  reg [7:0] ncap = 0;
  reg armed = 0;
  wire step = upd & armed;
  wire [15:0] y;
  wire dqs_i, dqs_o, dqs_t, dq_i, dq_o, dq_t;

  // Pad inputs sampled on JTCK, sticky between captures.
  reg [3:0] pz_acc = 0;
  reg [1:0] pads = 0;
  always @(posedge jtck) begin
    pads <= {dq_o, dqs_o};
    if (ce1 & ~shift) pz_acc <= 0;
    else pz_acc <= pz_acc | {pads[1], ~pads[1], pads[0], ~pads[0]};
  end

  always @(posedge jtck) begin
    if (ce1) begin
      armed <= 1'b1;
      if (shift) sr <= {1'b0, sr[255:1]};
      else begin
        sr <= {g_pr, lat_w, g_wrc, g_rdc, g_bdr, g_dvr, g_dvc, ncap, 8'hD5, {pause_all, dqs_rst, 10'h0}, pz_acc, y, x, 32'hD9500000};
        ncap <= ncap + 1;
      end
    end else if (step) begin
      armed <= 1'b0;
      x <= rx;
    end
  end

  // Clocks.
  wire clk200, pll_lock, eclk, sclk;
  EHXPLLL #(
    .CLKI_DIV(1), .CLKFB_DIV(8), .CLKOP_DIV(3), .CLKOS_DIV(1), .CLKOS2_DIV(1), .CLKOS3_DIV(1),
    .CLKOP_ENABLE("ENABLED"), .CLKOS_ENABLE("DISABLED"), .CLKOS2_ENABLE("DISABLED"),
    .CLKOS3_ENABLE("DISABLED"), .CLKOP_CPHASE(2), .CLKOS_CPHASE(0), .CLKOS2_CPHASE(0),
    .CLKOS3_CPHASE(0), .CLKOP_FPHASE(0), .CLKOS_FPHASE(0), .CLKOS2_FPHASE(0), .CLKOS3_FPHASE(0),
    .FEEDBK_PATH("CLKOP"), .CLKOP_TRIM_POL("RISING"), .CLKOP_TRIM_DELAY(0),
    .CLKOS_TRIM_POL("RISING"), .CLKOS_TRIM_DELAY(0), .OUTDIVIDER_MUXA("DIVA"),
    .OUTDIVIDER_MUXB("DIVB"), .OUTDIVIDER_MUXC("DIVC"), .OUTDIVIDER_MUXD("DIVD"),
    .PLL_LOCK_MODE(0), .PLL_LOCK_DELAY(200), .STDBY_ENABLE("DISABLED"),
    .REFIN_RESET("DISABLED"), .SYNC_ENABLE("DISABLED"), .INT_LOCK_STICKY("ENABLED"),
    .DPHASE_SOURCE("DISABLED"), .PLLRST_ENA("DISABLED"), .INTFB_WAKE("DISABLED")
  ) pll (
    .CLKI(clk_25mhz), .CLKFB(clk200),
    .PHASESEL1(zero), .PHASESEL0(zero), .PHASEDIR(zero), .PHASESTEP(zero),
    .PHASELOADREG(zero), .STDBY(zero), .PLLWAKESYNC(zero), .RST(zero),
    .ENCLKOP(zero), .ENCLKOS(zero), .ENCLKOS2(zero), .ENCLKOS3(zero),
    .CLKOP(clk200), .LOCK(pll_lock)
  );

  // The init sequence (ECP5DDRPHYInit) on clk_25mhz.
  reg [7:0] por = 0;
  wire init_rst = ~por[7];
  always @(posedge clk_25mhz) if (~por[7] & pll_lock) por <= por + 1;
  wire lock;
  reg [2:0] lock_s = 0, go_s = 0;
  always @(posedge clk_25mhz) begin
    lock_s <= {lock_s[1:0], lock};
    go_s <= {go_s[1:0], x[16]};
  end
  wire start = (lock_s[1] & ~lock_s[2]) | (go_s[1] & ~go_s[2]);
  reg [7:0] t = 8'hFF;
  reg freeze = 0, stop = 0, reset = 0, pause = 0, update = 0, done = 0;
  always @(posedge clk_25mhz) begin
    if (start) begin
      t <= 0;
      done <= 0;
    end else if (t != 8'hFF) t <= t + 1;
    case (t)
      8'd8: freeze <= 1;
      8'd16: stop <= 1;
      8'd24: reset <= 1;
      8'd32: reset <= 0;
      8'd40: stop <= 0;
      8'd48: freeze <= 0;
      8'd56: pause <= 1;
      8'd64: update <= 1;
      8'd72: update <= 0;
      8'd80: begin pause <= 0; done <= 1; end
      default: ;
    endcase
  end

  ECLKSYNCB sync (.ECLKI(clk200), .STOP(stop), .ECLKO(eclk));
  CLKDIVF #(.DIV("2.0")) div (.CLKI(eclk), .RST(reset), .ALIGNWD(zero), .CDIVX(sclk));
  wire ddrdel;
  // p35: DQSBUFM RST from the init reset (PLL lock plus POR), as LiteDRAM 1:2
  // resets it from sys reset. x46 also holds it through the init sequence to
  // the end of the pause release (t <= 88).
  assign dqs_rst = init_rst | (x[46] & (t <= 8'd88));
  DDRDLLA dll (.CLK(eclk), .RST(init_rst), .UDDCNTLN(~update), .FREEZE(freeze),
    .DDRDEL(ddrdel), .LOCK(lock));

  // sys frame: burst window and READ pulse.
  reg [3:0] fc = 0;
  reg rd = 0, oe = 1, pre = 0, post = 0, sb = 1;
  // p35: READ start phase, full 16 frame phases (host drives predicted point
  // first: burst start at pad (fc 6 + ODDRX_LAT) minus 3 SCLK lead, then +-4).
  localparam ODDRX_LAT = 1; // ODDRX2DQSB gearing latency in SCLK cycles
  wire [3:0] rds = x[23:20];
  always @(posedge sclk) begin
    fc <= fc + 1;
    oe <= ~x[24] | (fc == 4'd6) | (fc == 4'd7);
    pre <= x[24] & (fc == 4'd5);
    post <= x[24] & (fc == 4'd8);
    // Strobe data: 0,1,0,1 always (x24 = 0), or a burst of 8 on cycles 7, 8
    // with the strobe driven low around it (x24 = 1): the low preamble a
    // memory gives on a read, which the pulled-up loopback pad cannot.
    sb <= ~x[24] | (fc == 4'd6) | (fc == 4'd7);
    rd <= x[8] & ((fc == rds) | (fc == rds + 4'd1) |
      (x[45] & ((fc == rds + 4'd2) | (fc == rds + 4'd3)))); // 2 SCLK primary, 4 via x[45]
  end

  // p36: PAUSE = init pause | the oracle request x44, synchronised to sclk.
  // With x47 the release waits for frame cycle x[51:48]; else it follows the
  // request on the next cycle.
  reg [1:0] preq_s = 0;
  reg pause_q = 0;
  always @(posedge sclk) begin
    preq_s <= {preq_s[0], x[44]};
    if (preq_s[1]) pause_q <= 1'b1;
    else if (~x[47] | (fc == x[51:48])) pause_q <= 1'b0;
  end
  // p37: a PAUSE pulse before the burst: with x52, PAUSE is also raised for
  // x[63:60] sclk cycles from frame cycle x[59:56], on one frame in
  // 2^x[55:53] (the restart the detector fired on in p36).
  reg [6:0] fidx = 0;
  reg [3:0] fp_n = 0;
  reg fp = 0;
  reg fsel = 0;
  always @(*) begin
    case (x[55:53])
      3'd0: fsel = 1'b1;
      3'd1: fsel = fidx[0] == 1'b0;
      3'd2: fsel = fidx[1:0] == 2'd0;
      3'd3: fsel = fidx[2:0] == 3'd0;
      3'd4: fsel = fidx[3:0] == 4'd0;
      3'd5: fsel = fidx[4:0] == 5'd0;
      3'd6: fsel = fidx[5:0] == 6'd0;
      default: fsel = fidx == 7'd0;
    endcase
  end
  always @(posedge sclk) begin
    if (fc == 4'd15) fidx <= fidx + 1;
    if (x[52] & fsel & (fc == x[59:56]) & (x[63:60] != 4'd0)) begin
      fp <= 1'b1;
      fp_n <= x[63:60] - 4'd1;
    end else if (fp_n != 4'd0) begin
      fp_n <= fp_n - 4'd1;
    end else begin
      fp <= 1'b0;
    end
  end
  assign pause_all = pause | pause_q | fp;

  // Results.
  wire [3:0] q;
  wire qwl, dv, bd, rdc, wrc;
  reg [17:0] win = 0;
  reg [3:0] q_dv = 0, q_any = 0, q_dv_o = 0, q_any_o = 0;
  reg qwl_dv = 0, qwl_dv_o = 0;
  reg [3:0] seen = 0, seen_o = 0;
  always @(posedge sclk) begin
    win <= win + 1;
    if (win == 0) begin
      q_dv_o <= q_dv;
      qwl_dv_o <= qwl_dv;
      q_any_o <= q_any;
      seen_o <= seen;
      q_any <= q;
      seen <= {wrc, rdc, bd, dv};
    end else begin
      q_any <= q_any | q;
      seen <= seen | {wrc, rdc, bd, dv};
    end
    if (dv) begin
      q_dv <= q;
      qwl_dv <= qwl;
    end
  end
  assign y = {done, pll_lock, q_any_o, seen_o, lock, qwl_dv_o, q_dv_o};

  // p36: free-running event counts: DATAVALID cycles, DATAVALID / BURSTDET /
  // RDCFLAG / WRCFLAG rising edges, PAUSE releases; and the sclk cycles from
  // the last PAUSE release to the first DATAVALID rise after it (FFFF none).
  // Gray coded so a JTCK-domain sample is off by at most one count.
  reg [15:0] c_dvc = 0, c_dvr = 0, c_bdr = 0, c_rdc = 0, c_wrc = 0;
  reg [15:0] lat = 16'hFFFF, since = 16'hFFFF;
  reg [15:0] c_pr = 0;
  reg dv_d = 0, bd_d = 0, rdc_d = 0, wrc_d = 0, pa_d = 0, want_lat = 0;
  always @(posedge sclk) begin
    dv_d <= dv;
    bd_d <= bd;
    rdc_d <= rdc;
    wrc_d <= wrc;
    pa_d <= pause_all;
    if (dv) c_dvc <= c_dvc + 1;
    if (dv & ~dv_d) c_dvr <= c_dvr + 1;
    if (bd & ~bd_d) c_bdr <= c_bdr + 1;
    if (rdc & ~rdc_d) c_rdc <= c_rdc + 1;
    if (wrc & ~wrc_d) c_wrc <= c_wrc + 1;
    if (pa_d & ~pause_all) begin
      c_pr <= c_pr + 1;
      since <= 0;
      lat <= 16'hFFFF;
      want_lat <= 1'b1;
    end else begin
      if (since != 16'hFFFF) since <= since + 1;
      if (want_lat & dv & ~dv_d) begin
        lat <= since;
        want_lat <= 1'b0;
      end
    end
  end
  assign g_dvc = c_dvc ^ (c_dvc >> 1);
  assign g_dvr = c_dvr ^ (c_dvr >> 1);
  assign g_bdr = c_bdr ^ (c_bdr >> 1);
  assign g_rdc = c_rdc ^ (c_rdc >> 1);
  assign g_wrc = c_wrc ^ (c_wrc >> 1);
  assign g_pr = c_pr ^ (c_pr >> 1);
  // p40: the IDDRX2DQA word 1..4 sclk cycles after each DATAVALID rise
  // (bits 224..239 of the scan word, replacing the release latency).
  reg [3:0] dvp = 0;
  reg [3:0] q_p1 = 0, q_p2 = 0, q_p3 = 0, q_p4 = 0;
  always @(posedge sclk) begin
    dvp <= {dvp[2:0], dv & ~dv_d};
    if (dvp[0]) q_p1 <= q;
    if (dvp[1]) q_p2 <= q;
    if (dvp[2]) q_p3 <= q;
    if (dvp[3]) q_p4 <= q;
  end
  assign lat_w = {q_p4, q_p3, q_p2, q_p1};

  // The DQS group.
  wire dqsr90, dqsw, dqsw270;
  wire [2:0] rdp, wrp;
  DQSBUFM #(.DQS_LI_DEL_ADJ("PLUS"), .DQS_LI_DEL_VAL(0), .DQS_LO_DEL_ADJ("PLUS"),
    .DQS_LO_DEL_VAL(0)) dqsbuf (
    .DQSI(dqs_o), .READ1(rd), .READ0(rd), .READCLKSEL2(x[14]), .READCLKSEL1(x[13]),
    .READCLKSEL0(x[12]), .DDRDEL(ddrdel), .ECLK(eclk), .SCLK(sclk), .RST(dqs_rst), .PAUSE(pause_all),
    .RDLOADN(x[40]), .RDMOVE(x[17]), .RDDIRECTION(~x[42]),
    .WRLOADN(x[41]), .WRMOVE(x[18]), .WRDIRECTION(~x[43]),
    .DQSR90(dqsr90), .DQSW(dqsw), .DQSW270(dqsw270), .RDPNTR2(rdp[2]), .RDPNTR1(rdp[1]),
    .RDPNTR0(rdp[0]), .WRPNTR2(wrp[2]), .WRPNTR1(wrp[1]), .WRPNTR0(wrp[0]),
    .DATAVALID(dv), .BURSTDET(bd), .RDCFLAG(rdc), .WRCFLAG(wrc));

  // The strobe pad.
  ODDRX2DQSB dqs_out (.D0(zero), .D1(sb), .D2(zero), .D3(sb), .SCLK(sclk), .ECLK(eclk),
    .DQSW(dqsw), .RST(zero), .Q(dqs_i));
  TSHX2DQSA dqs_tri (.T0(zero), .T1(zero), .SCLK(sclk), .ECLK(eclk),
    .DQSW(dqsw), .RST(zero), .Q(dqs_t));
  BB dqs_bb (.I(dqs_i), .T(dqs_t), .O(dqs_o), .B(gp19));

  // The DQ pad.
  ODDRX2DQA dq_out (.D0(x[0]), .D1(x[1]), .D2(x[2]), .D3(x[3]), .DQSW270(dqsw270), .SCLK(sclk),
    .ECLK(eclk), .RST(zero), .Q(dq_i));
  TSHX2DQA dq_tri (.T0(zero), .T1(zero), .SCLK(sclk), .ECLK(eclk), .DQSW270(dqsw270),
    .RST(zero), .Q(dq_t));
  BB dq_bb (.I(dq_i), .T(dq_t), .O(dq_o), .B(gp20));
  // LiteDRAM puts a DELAYG (DQS_ALIGNED_X2) here; its output shares a CIB wire
  // with the pad input the fabric sampler needs, so the read takes the pad.
  IDDRX2DQA dq_in (.D(dq_o), .DQSR90(dqsr90), .SCLK(sclk), .ECLK(eclk), .RST(zero),
    .RDPNTR2(rdp[2]), .RDPNTR1(rdp[1]), .RDPNTR0(rdp[0]), .WRPNTR2(wrp[2]), .WRPNTR1(wrp[1]),
    .WRPNTR0(wrp[0]), .Q0(q[0]), .Q1(q[1]), .Q2(q[2]), .Q3(q[3]), .QWL(qwl));

  JTAGG #(.ER1("ENABLED"), .ER2("ENABLED")) jtag (
    .JTDO1(sr[0]), .JTDO2(zero),
    .JTCK(jtck), .JTDI(jtdi), .JSHIFT(jshift), .JUPDATE(jupdate),
    .JRSTN(jrstn), .JCE1(jce1), .JCE2(jce2), .JRTI1(jrti1), .JRTI2(jrti2)
  );

  reg [25:0] hb = 0;
  always @(posedge clk_25mhz) hb <= hb + 1;
  assign led = {hb[25], 6'b0, done};
endmodule
