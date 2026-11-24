// Block RAM probe for the ULX3S, read by jtag_oracle.exs --design bram.
//
// 256-bit ER1 frame, LSB first: magic 0xB0B00000, x[63:0], doa[17:0],
// dob[17:0], do[35:0], tag 0xB5, ncap. x is the last 64 bits shifted in,
// latched on Update-DR; both RAMs run on JTCK.
//   d0: DP16KD, both ports x18. Port A: word x[9:0], data x[37:20],
//       write x[38]. Port B: word x[19:10], read only. INITVAL_00 word j =
//       (0x2A5A5 + j * 0x0F0F1) & 0x3FFFF.
//   p0: PDPW16KD, x36. Write word x[48:40] with {x[37:20], x[37:20]} when
//       x[39]; read word x[57:49]. INITVAL_00 the same.
module bram_probe (
  input clk_25mhz,
  output [7:0] led
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
  reg [7:0] ncap = 0;
  reg armed = 0;
  wire step = upd & armed;
  wire [17:0] doa, dob;
  wire [35:0] dout;

  always @(posedge jtck) begin
    if (ce1) begin
      armed <= 1'b1;
      if (shift) sr <= {1'b0, sr[255:1]};
      else begin
        sr <= {72'h0, ncap, 8'hB5, dout, dob, doa, x, 32'hB0B00000};
        ncap <= ncap + 1;
      end
    end else if (step) begin
      armed <= 1'b0;
      x <= rx;
    end
  end

  wire [13:0] ada = {x[9:0], 2'b00, 2'b11};
  wire [13:0] adb = {x[19:10], 4'b0000};
  wire [17:0] di = x[37:20];
  wire [8:0] adw = x[48:40];
  wire [13:0] adr = {x[57:49], 5'b00000};
  wire [35:0] dw = {x[37:20], x[37:20]};

  DP16KD #(.DATA_WIDTH_A(18), .DATA_WIDTH_B(18), .INITVAL_00(320'h0C3C43D2D32E1E21F0F11000000F0F31E1E22D2D13C3C04B4B35A5A269691787808787396962A5A5)) d0 (
    .ADA13(ada[13]), .ADA12(ada[12]), .ADA11(ada[11]), .ADA10(ada[10]), .ADA9(ada[9]), 
    .ADA8(ada[8]), .ADA7(ada[7]), .ADA6(ada[6]), .ADA5(ada[5]), .ADA4(ada[4]), .ADA3(ada[3]), 
    .ADA2(ada[2]), .ADA1(ada[1]), .ADA0(ada[0]), .DIA17(di[17]), .DIA16(di[16]), .DIA15(di[15]), 
    .DIA14(di[14]), .DIA13(di[13]), .DIA12(di[12]), .DIA11(di[11]), .DIA10(di[10]), .DIA9(di[9]), 
    .DIA8(di[8]), .DIA7(di[7]), .DIA6(di[6]), .DIA5(di[5]), .DIA4(di[4]), .DIA3(di[3]), 
    .DIA2(di[2]), .DIA1(di[1]), .DIA0(di[0]), .WEA(x[38]), .CEA(one), .OCEA(one), .CLKA(jtck), 
    .RSTA(zero), .ADB13(adb[13]), .ADB12(adb[12]), .ADB11(adb[11]), .ADB10(adb[10]), .ADB9(adb[9]), 
    .ADB8(adb[8]), .ADB7(adb[7]), .ADB6(adb[6]), .ADB5(adb[5]), .ADB4(adb[4]), .ADB3(adb[3]), 
    .ADB2(adb[2]), .ADB1(adb[1]), .ADB0(adb[0]), .WEB(zero), .CEB(one), .OCEB(one), .CLKB(jtck), 
    .RSTB(zero), .DOA17(doa[17]), .DOA16(doa[16]), .DOA15(doa[15]), .DOA14(doa[14]), 
    .DOA13(doa[13]), .DOA12(doa[12]), .DOA11(doa[11]), .DOA10(doa[10]), .DOA9(doa[9]), 
    .DOA8(doa[8]), .DOA7(doa[7]), .DOA6(doa[6]), .DOA5(doa[5]), .DOA4(doa[4]), .DOA3(doa[3]), 
    .DOA2(doa[2]), .DOA1(doa[1]), .DOA0(doa[0]), .DOB17(dob[17]), .DOB16(dob[16]), .DOB15(dob[15]), 
    .DOB14(dob[14]), .DOB13(dob[13]), .DOB12(dob[12]), .DOB11(dob[11]), .DOB10(dob[10]), 
    .DOB9(dob[9]), .DOB8(dob[8]), .DOB7(dob[7]), .DOB6(dob[6]), .DOB5(dob[5]), .DOB4(dob[4]), 
    .DOB3(dob[3]), .DOB2(dob[2]), .DOB1(dob[1]), .DOB0(dob[0])  );

  PDPW16KD #(.DATA_WIDTH_W(36), .DATA_WIDTH_R(36), .INITVAL_00(320'h0C3C43D2D32E1E21F0F11000000F0F31E1E22D2D13C3C04B4B35A5A269691787808787396962A5A5)) p0 (
    .ADW8(adw[8]), .ADW7(adw[7]), .ADW6(adw[6]), .ADW5(adw[5]), .ADW4(adw[4]), .ADW3(adw[3]), 
    .ADW2(adw[2]), .ADW1(adw[1]), .ADW0(adw[0]), .DI35(dw[35]), .DI34(dw[34]), .DI33(dw[33]), 
    .DI32(dw[32]), .DI31(dw[31]), .DI30(dw[30]), .DI29(dw[29]), .DI28(dw[28]), .DI27(dw[27]), 
    .DI26(dw[26]), .DI25(dw[25]), .DI24(dw[24]), .DI23(dw[23]), .DI22(dw[22]), .DI21(dw[21]), 
    .DI20(dw[20]), .DI19(dw[19]), .DI18(dw[18]), .DI17(dw[17]), .DI16(dw[16]), .DI15(dw[15]), 
    .DI14(dw[14]), .DI13(dw[13]), .DI12(dw[12]), .DI11(dw[11]), .DI10(dw[10]), .DI9(dw[9]), 
    .DI8(dw[8]), .DI7(dw[7]), .DI6(dw[6]), .DI5(dw[5]), .DI4(dw[4]), .DI3(dw[3]), .DI2(dw[2]), 
    .DI1(dw[1]), .DI0(dw[0]), .BE3(one), .BE2(one), .BE1(one), .BE0(one), .CEW(x[39]), .CLKW(jtck), 
    .ADR13(adr[13]), .ADR12(adr[12]), .ADR11(adr[11]), .ADR10(adr[10]), .ADR9(adr[9]), 
    .ADR8(adr[8]), .ADR7(adr[7]), .ADR6(adr[6]), .ADR5(adr[5]), .ADR4(adr[4]), .ADR3(adr[3]), 
    .ADR2(adr[2]), .ADR1(adr[1]), .ADR0(adr[0]), .CER(one), .OCER(one), .CLKR(jtck), .RST(zero), 
    .DO35(dout[35]), .DO34(dout[34]), .DO33(dout[33]), .DO32(dout[32]), .DO31(dout[31]), 
    .DO30(dout[30]), .DO29(dout[29]), .DO28(dout[28]), .DO27(dout[27]), .DO26(dout[26]), 
    .DO25(dout[25]), .DO24(dout[24]), .DO23(dout[23]), .DO22(dout[22]), .DO21(dout[21]), 
    .DO20(dout[20]), .DO19(dout[19]), .DO18(dout[18]), .DO17(dout[17]), .DO16(dout[16]), 
    .DO15(dout[15]), .DO14(dout[14]), .DO13(dout[13]), .DO12(dout[12]), .DO11(dout[11]), 
    .DO10(dout[10]), .DO9(dout[9]), .DO8(dout[8]), .DO7(dout[7]), .DO6(dout[6]), .DO5(dout[5]), 
    .DO4(dout[4]), .DO3(dout[3]), .DO2(dout[2]), .DO1(dout[1]), .DO0(dout[0])  );

  JTAGG #(.ER1("ENABLED"), .ER2("ENABLED")) jtag (
    .JTDO1(sr[0]), .JTDO2(zero),
    .JTCK(jtck), .JTDI(jtdi), .JSHIFT(jshift), .JUPDATE(jupdate),
    .JRSTN(jrstn), .JCE1(jce1), .JCE2(jce2), .JRTI1(jrti1), .JRTI2(jrti2)
  );

  reg [25:0] hb = 0;
  always @(posedge clk_25mhz) hb <= hb + 1;
  assign led = {hb[25], 7'b0};
endmodule
