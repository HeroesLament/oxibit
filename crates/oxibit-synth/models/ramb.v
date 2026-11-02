// Behavioural RAMB36E1: the subset oxibit-synth emits (bram.rs).
// TDP and SDP, widths 1/2/4/9/18/36 (and SDP x72), no output
// registers, READ_FIRST, ENx and byte write enables, INIT/INITP contents.
// Written from UG473, not derived from yosys or Vivado models.
module RAMB36E1 (
    input CLKARDCLK,
    input CLKBWRCLK,
    input ENARDEN,
    input ENBWREN,
    input REGCEAREGCE,
    input REGCEB,
    input RSTRAMARSTRAM,
    input RSTRAMB,
    input RSTREGARSTREG,
    input RSTREGB,
    input [15:0] ADDRARDADDR,
    input [15:0] ADDRBWRADDR,
    input [31:0] DIADI,
    input [31:0] DIBDI,
    input [3:0] DIPADIP,
    input [3:0] DIPBDIP,
    input [3:0] WEA,
    input [7:0] WEBWE,
    output reg [31:0] DOADO,
    output reg [31:0] DOBDO,
    output reg [3:0] DOPADOP,
    output reg [3:0] DOPBDOP
);
    parameter RAM_MODE = "TDP";
    parameter WRITE_MODE_A = "READ_FIRST";
    parameter WRITE_MODE_B = "READ_FIRST";
    parameter RAM_EXTENSION_A = "NONE";
    parameter RAM_EXTENSION_B = "NONE";
    parameter integer READ_WIDTH_A = 0;
    parameter integer READ_WIDTH_B = 0;
    parameter integer WRITE_WIDTH_A = 0;
    parameter integer WRITE_WIDTH_B = 0;
    parameter integer DOA_REG = 0;
    parameter integer DOB_REG = 0;
    parameter [35:0] INIT_A = 0;
    parameter [35:0] INIT_B = 0;
    parameter [35:0] SRVAL_A = 0;
    parameter [35:0] SRVAL_B = 0;
    parameter [0:0] IS_CLKARDCLK_INVERTED = 1'b0;
    parameter [0:0] IS_CLKBWRCLK_INVERTED = 1'b0;
    parameter [255:0] INIT_00 = 256'h0;
    parameter [255:0] INIT_01 = 256'h0;
    parameter [255:0] INIT_02 = 256'h0;
    parameter [255:0] INIT_03 = 256'h0;
    parameter [255:0] INIT_04 = 256'h0;
    parameter [255:0] INIT_05 = 256'h0;
    parameter [255:0] INIT_06 = 256'h0;
    parameter [255:0] INIT_07 = 256'h0;
    parameter [255:0] INIT_08 = 256'h0;
    parameter [255:0] INIT_09 = 256'h0;
    parameter [255:0] INIT_0A = 256'h0;
    parameter [255:0] INIT_0B = 256'h0;
    parameter [255:0] INIT_0C = 256'h0;
    parameter [255:0] INIT_0D = 256'h0;
    parameter [255:0] INIT_0E = 256'h0;
    parameter [255:0] INIT_0F = 256'h0;
    parameter [255:0] INIT_10 = 256'h0;
    parameter [255:0] INIT_11 = 256'h0;
    parameter [255:0] INIT_12 = 256'h0;
    parameter [255:0] INIT_13 = 256'h0;
    parameter [255:0] INIT_14 = 256'h0;
    parameter [255:0] INIT_15 = 256'h0;
    parameter [255:0] INIT_16 = 256'h0;
    parameter [255:0] INIT_17 = 256'h0;
    parameter [255:0] INIT_18 = 256'h0;
    parameter [255:0] INIT_19 = 256'h0;
    parameter [255:0] INIT_1A = 256'h0;
    parameter [255:0] INIT_1B = 256'h0;
    parameter [255:0] INIT_1C = 256'h0;
    parameter [255:0] INIT_1D = 256'h0;
    parameter [255:0] INIT_1E = 256'h0;
    parameter [255:0] INIT_1F = 256'h0;
    parameter [255:0] INIT_20 = 256'h0;
    parameter [255:0] INIT_21 = 256'h0;
    parameter [255:0] INIT_22 = 256'h0;
    parameter [255:0] INIT_23 = 256'h0;
    parameter [255:0] INIT_24 = 256'h0;
    parameter [255:0] INIT_25 = 256'h0;
    parameter [255:0] INIT_26 = 256'h0;
    parameter [255:0] INIT_27 = 256'h0;
    parameter [255:0] INIT_28 = 256'h0;
    parameter [255:0] INIT_29 = 256'h0;
    parameter [255:0] INIT_2A = 256'h0;
    parameter [255:0] INIT_2B = 256'h0;
    parameter [255:0] INIT_2C = 256'h0;
    parameter [255:0] INIT_2D = 256'h0;
    parameter [255:0] INIT_2E = 256'h0;
    parameter [255:0] INIT_2F = 256'h0;
    parameter [255:0] INIT_30 = 256'h0;
    parameter [255:0] INIT_31 = 256'h0;
    parameter [255:0] INIT_32 = 256'h0;
    parameter [255:0] INIT_33 = 256'h0;
    parameter [255:0] INIT_34 = 256'h0;
    parameter [255:0] INIT_35 = 256'h0;
    parameter [255:0] INIT_36 = 256'h0;
    parameter [255:0] INIT_37 = 256'h0;
    parameter [255:0] INIT_38 = 256'h0;
    parameter [255:0] INIT_39 = 256'h0;
    parameter [255:0] INIT_3A = 256'h0;
    parameter [255:0] INIT_3B = 256'h0;
    parameter [255:0] INIT_3C = 256'h0;
    parameter [255:0] INIT_3D = 256'h0;
    parameter [255:0] INIT_3E = 256'h0;
    parameter [255:0] INIT_3F = 256'h0;
    parameter [255:0] INIT_40 = 256'h0;
    parameter [255:0] INIT_41 = 256'h0;
    parameter [255:0] INIT_42 = 256'h0;
    parameter [255:0] INIT_43 = 256'h0;
    parameter [255:0] INIT_44 = 256'h0;
    parameter [255:0] INIT_45 = 256'h0;
    parameter [255:0] INIT_46 = 256'h0;
    parameter [255:0] INIT_47 = 256'h0;
    parameter [255:0] INIT_48 = 256'h0;
    parameter [255:0] INIT_49 = 256'h0;
    parameter [255:0] INIT_4A = 256'h0;
    parameter [255:0] INIT_4B = 256'h0;
    parameter [255:0] INIT_4C = 256'h0;
    parameter [255:0] INIT_4D = 256'h0;
    parameter [255:0] INIT_4E = 256'h0;
    parameter [255:0] INIT_4F = 256'h0;
    parameter [255:0] INIT_50 = 256'h0;
    parameter [255:0] INIT_51 = 256'h0;
    parameter [255:0] INIT_52 = 256'h0;
    parameter [255:0] INIT_53 = 256'h0;
    parameter [255:0] INIT_54 = 256'h0;
    parameter [255:0] INIT_55 = 256'h0;
    parameter [255:0] INIT_56 = 256'h0;
    parameter [255:0] INIT_57 = 256'h0;
    parameter [255:0] INIT_58 = 256'h0;
    parameter [255:0] INIT_59 = 256'h0;
    parameter [255:0] INIT_5A = 256'h0;
    parameter [255:0] INIT_5B = 256'h0;
    parameter [255:0] INIT_5C = 256'h0;
    parameter [255:0] INIT_5D = 256'h0;
    parameter [255:0] INIT_5E = 256'h0;
    parameter [255:0] INIT_5F = 256'h0;
    parameter [255:0] INIT_60 = 256'h0;
    parameter [255:0] INIT_61 = 256'h0;
    parameter [255:0] INIT_62 = 256'h0;
    parameter [255:0] INIT_63 = 256'h0;
    parameter [255:0] INIT_64 = 256'h0;
    parameter [255:0] INIT_65 = 256'h0;
    parameter [255:0] INIT_66 = 256'h0;
    parameter [255:0] INIT_67 = 256'h0;
    parameter [255:0] INIT_68 = 256'h0;
    parameter [255:0] INIT_69 = 256'h0;
    parameter [255:0] INIT_6A = 256'h0;
    parameter [255:0] INIT_6B = 256'h0;
    parameter [255:0] INIT_6C = 256'h0;
    parameter [255:0] INIT_6D = 256'h0;
    parameter [255:0] INIT_6E = 256'h0;
    parameter [255:0] INIT_6F = 256'h0;
    parameter [255:0] INIT_70 = 256'h0;
    parameter [255:0] INIT_71 = 256'h0;
    parameter [255:0] INIT_72 = 256'h0;
    parameter [255:0] INIT_73 = 256'h0;
    parameter [255:0] INIT_74 = 256'h0;
    parameter [255:0] INIT_75 = 256'h0;
    parameter [255:0] INIT_76 = 256'h0;
    parameter [255:0] INIT_77 = 256'h0;
    parameter [255:0] INIT_78 = 256'h0;
    parameter [255:0] INIT_79 = 256'h0;
    parameter [255:0] INIT_7A = 256'h0;
    parameter [255:0] INIT_7B = 256'h0;
    parameter [255:0] INIT_7C = 256'h0;
    parameter [255:0] INIT_7D = 256'h0;
    parameter [255:0] INIT_7E = 256'h0;
    parameter [255:0] INIT_7F = 256'h0;
    parameter [255:0] INITP_00 = 256'h0;
    parameter [255:0] INITP_01 = 256'h0;
    parameter [255:0] INITP_02 = 256'h0;
    parameter [255:0] INITP_03 = 256'h0;
    parameter [255:0] INITP_04 = 256'h0;
    parameter [255:0] INITP_05 = 256'h0;
    parameter [255:0] INITP_06 = 256'h0;
    parameter [255:0] INITP_07 = 256'h0;
    parameter [255:0] INITP_08 = 256'h0;
    parameter [255:0] INITP_09 = 256'h0;
    parameter [255:0] INITP_0A = 256'h0;
    parameter [255:0] INITP_0B = 256'h0;
    parameter [255:0] INITP_0C = 256'h0;
    parameter [255:0] INITP_0D = 256'h0;
    parameter [255:0] INITP_0E = 256'h0;
    parameter [255:0] INITP_0F = 256'h0;

    reg data [0:32767];
    reg par [0:4095];
    integer i;
    initial begin
        for (i = 0; i < 256; i = i + 1) data[0 + i] = INIT_00[i];
        for (i = 0; i < 256; i = i + 1) data[256 + i] = INIT_01[i];
        for (i = 0; i < 256; i = i + 1) data[512 + i] = INIT_02[i];
        for (i = 0; i < 256; i = i + 1) data[768 + i] = INIT_03[i];
        for (i = 0; i < 256; i = i + 1) data[1024 + i] = INIT_04[i];
        for (i = 0; i < 256; i = i + 1) data[1280 + i] = INIT_05[i];
        for (i = 0; i < 256; i = i + 1) data[1536 + i] = INIT_06[i];
        for (i = 0; i < 256; i = i + 1) data[1792 + i] = INIT_07[i];
        for (i = 0; i < 256; i = i + 1) data[2048 + i] = INIT_08[i];
        for (i = 0; i < 256; i = i + 1) data[2304 + i] = INIT_09[i];
        for (i = 0; i < 256; i = i + 1) data[2560 + i] = INIT_0A[i];
        for (i = 0; i < 256; i = i + 1) data[2816 + i] = INIT_0B[i];
        for (i = 0; i < 256; i = i + 1) data[3072 + i] = INIT_0C[i];
        for (i = 0; i < 256; i = i + 1) data[3328 + i] = INIT_0D[i];
        for (i = 0; i < 256; i = i + 1) data[3584 + i] = INIT_0E[i];
        for (i = 0; i < 256; i = i + 1) data[3840 + i] = INIT_0F[i];
        for (i = 0; i < 256; i = i + 1) data[4096 + i] = INIT_10[i];
        for (i = 0; i < 256; i = i + 1) data[4352 + i] = INIT_11[i];
        for (i = 0; i < 256; i = i + 1) data[4608 + i] = INIT_12[i];
        for (i = 0; i < 256; i = i + 1) data[4864 + i] = INIT_13[i];
        for (i = 0; i < 256; i = i + 1) data[5120 + i] = INIT_14[i];
        for (i = 0; i < 256; i = i + 1) data[5376 + i] = INIT_15[i];
        for (i = 0; i < 256; i = i + 1) data[5632 + i] = INIT_16[i];
        for (i = 0; i < 256; i = i + 1) data[5888 + i] = INIT_17[i];
        for (i = 0; i < 256; i = i + 1) data[6144 + i] = INIT_18[i];
        for (i = 0; i < 256; i = i + 1) data[6400 + i] = INIT_19[i];
        for (i = 0; i < 256; i = i + 1) data[6656 + i] = INIT_1A[i];
        for (i = 0; i < 256; i = i + 1) data[6912 + i] = INIT_1B[i];
        for (i = 0; i < 256; i = i + 1) data[7168 + i] = INIT_1C[i];
        for (i = 0; i < 256; i = i + 1) data[7424 + i] = INIT_1D[i];
        for (i = 0; i < 256; i = i + 1) data[7680 + i] = INIT_1E[i];
        for (i = 0; i < 256; i = i + 1) data[7936 + i] = INIT_1F[i];
        for (i = 0; i < 256; i = i + 1) data[8192 + i] = INIT_20[i];
        for (i = 0; i < 256; i = i + 1) data[8448 + i] = INIT_21[i];
        for (i = 0; i < 256; i = i + 1) data[8704 + i] = INIT_22[i];
        for (i = 0; i < 256; i = i + 1) data[8960 + i] = INIT_23[i];
        for (i = 0; i < 256; i = i + 1) data[9216 + i] = INIT_24[i];
        for (i = 0; i < 256; i = i + 1) data[9472 + i] = INIT_25[i];
        for (i = 0; i < 256; i = i + 1) data[9728 + i] = INIT_26[i];
        for (i = 0; i < 256; i = i + 1) data[9984 + i] = INIT_27[i];
        for (i = 0; i < 256; i = i + 1) data[10240 + i] = INIT_28[i];
        for (i = 0; i < 256; i = i + 1) data[10496 + i] = INIT_29[i];
        for (i = 0; i < 256; i = i + 1) data[10752 + i] = INIT_2A[i];
        for (i = 0; i < 256; i = i + 1) data[11008 + i] = INIT_2B[i];
        for (i = 0; i < 256; i = i + 1) data[11264 + i] = INIT_2C[i];
        for (i = 0; i < 256; i = i + 1) data[11520 + i] = INIT_2D[i];
        for (i = 0; i < 256; i = i + 1) data[11776 + i] = INIT_2E[i];
        for (i = 0; i < 256; i = i + 1) data[12032 + i] = INIT_2F[i];
        for (i = 0; i < 256; i = i + 1) data[12288 + i] = INIT_30[i];
        for (i = 0; i < 256; i = i + 1) data[12544 + i] = INIT_31[i];
        for (i = 0; i < 256; i = i + 1) data[12800 + i] = INIT_32[i];
        for (i = 0; i < 256; i = i + 1) data[13056 + i] = INIT_33[i];
        for (i = 0; i < 256; i = i + 1) data[13312 + i] = INIT_34[i];
        for (i = 0; i < 256; i = i + 1) data[13568 + i] = INIT_35[i];
        for (i = 0; i < 256; i = i + 1) data[13824 + i] = INIT_36[i];
        for (i = 0; i < 256; i = i + 1) data[14080 + i] = INIT_37[i];
        for (i = 0; i < 256; i = i + 1) data[14336 + i] = INIT_38[i];
        for (i = 0; i < 256; i = i + 1) data[14592 + i] = INIT_39[i];
        for (i = 0; i < 256; i = i + 1) data[14848 + i] = INIT_3A[i];
        for (i = 0; i < 256; i = i + 1) data[15104 + i] = INIT_3B[i];
        for (i = 0; i < 256; i = i + 1) data[15360 + i] = INIT_3C[i];
        for (i = 0; i < 256; i = i + 1) data[15616 + i] = INIT_3D[i];
        for (i = 0; i < 256; i = i + 1) data[15872 + i] = INIT_3E[i];
        for (i = 0; i < 256; i = i + 1) data[16128 + i] = INIT_3F[i];
        for (i = 0; i < 256; i = i + 1) data[16384 + i] = INIT_40[i];
        for (i = 0; i < 256; i = i + 1) data[16640 + i] = INIT_41[i];
        for (i = 0; i < 256; i = i + 1) data[16896 + i] = INIT_42[i];
        for (i = 0; i < 256; i = i + 1) data[17152 + i] = INIT_43[i];
        for (i = 0; i < 256; i = i + 1) data[17408 + i] = INIT_44[i];
        for (i = 0; i < 256; i = i + 1) data[17664 + i] = INIT_45[i];
        for (i = 0; i < 256; i = i + 1) data[17920 + i] = INIT_46[i];
        for (i = 0; i < 256; i = i + 1) data[18176 + i] = INIT_47[i];
        for (i = 0; i < 256; i = i + 1) data[18432 + i] = INIT_48[i];
        for (i = 0; i < 256; i = i + 1) data[18688 + i] = INIT_49[i];
        for (i = 0; i < 256; i = i + 1) data[18944 + i] = INIT_4A[i];
        for (i = 0; i < 256; i = i + 1) data[19200 + i] = INIT_4B[i];
        for (i = 0; i < 256; i = i + 1) data[19456 + i] = INIT_4C[i];
        for (i = 0; i < 256; i = i + 1) data[19712 + i] = INIT_4D[i];
        for (i = 0; i < 256; i = i + 1) data[19968 + i] = INIT_4E[i];
        for (i = 0; i < 256; i = i + 1) data[20224 + i] = INIT_4F[i];
        for (i = 0; i < 256; i = i + 1) data[20480 + i] = INIT_50[i];
        for (i = 0; i < 256; i = i + 1) data[20736 + i] = INIT_51[i];
        for (i = 0; i < 256; i = i + 1) data[20992 + i] = INIT_52[i];
        for (i = 0; i < 256; i = i + 1) data[21248 + i] = INIT_53[i];
        for (i = 0; i < 256; i = i + 1) data[21504 + i] = INIT_54[i];
        for (i = 0; i < 256; i = i + 1) data[21760 + i] = INIT_55[i];
        for (i = 0; i < 256; i = i + 1) data[22016 + i] = INIT_56[i];
        for (i = 0; i < 256; i = i + 1) data[22272 + i] = INIT_57[i];
        for (i = 0; i < 256; i = i + 1) data[22528 + i] = INIT_58[i];
        for (i = 0; i < 256; i = i + 1) data[22784 + i] = INIT_59[i];
        for (i = 0; i < 256; i = i + 1) data[23040 + i] = INIT_5A[i];
        for (i = 0; i < 256; i = i + 1) data[23296 + i] = INIT_5B[i];
        for (i = 0; i < 256; i = i + 1) data[23552 + i] = INIT_5C[i];
        for (i = 0; i < 256; i = i + 1) data[23808 + i] = INIT_5D[i];
        for (i = 0; i < 256; i = i + 1) data[24064 + i] = INIT_5E[i];
        for (i = 0; i < 256; i = i + 1) data[24320 + i] = INIT_5F[i];
        for (i = 0; i < 256; i = i + 1) data[24576 + i] = INIT_60[i];
        for (i = 0; i < 256; i = i + 1) data[24832 + i] = INIT_61[i];
        for (i = 0; i < 256; i = i + 1) data[25088 + i] = INIT_62[i];
        for (i = 0; i < 256; i = i + 1) data[25344 + i] = INIT_63[i];
        for (i = 0; i < 256; i = i + 1) data[25600 + i] = INIT_64[i];
        for (i = 0; i < 256; i = i + 1) data[25856 + i] = INIT_65[i];
        for (i = 0; i < 256; i = i + 1) data[26112 + i] = INIT_66[i];
        for (i = 0; i < 256; i = i + 1) data[26368 + i] = INIT_67[i];
        for (i = 0; i < 256; i = i + 1) data[26624 + i] = INIT_68[i];
        for (i = 0; i < 256; i = i + 1) data[26880 + i] = INIT_69[i];
        for (i = 0; i < 256; i = i + 1) data[27136 + i] = INIT_6A[i];
        for (i = 0; i < 256; i = i + 1) data[27392 + i] = INIT_6B[i];
        for (i = 0; i < 256; i = i + 1) data[27648 + i] = INIT_6C[i];
        for (i = 0; i < 256; i = i + 1) data[27904 + i] = INIT_6D[i];
        for (i = 0; i < 256; i = i + 1) data[28160 + i] = INIT_6E[i];
        for (i = 0; i < 256; i = i + 1) data[28416 + i] = INIT_6F[i];
        for (i = 0; i < 256; i = i + 1) data[28672 + i] = INIT_70[i];
        for (i = 0; i < 256; i = i + 1) data[28928 + i] = INIT_71[i];
        for (i = 0; i < 256; i = i + 1) data[29184 + i] = INIT_72[i];
        for (i = 0; i < 256; i = i + 1) data[29440 + i] = INIT_73[i];
        for (i = 0; i < 256; i = i + 1) data[29696 + i] = INIT_74[i];
        for (i = 0; i < 256; i = i + 1) data[29952 + i] = INIT_75[i];
        for (i = 0; i < 256; i = i + 1) data[30208 + i] = INIT_76[i];
        for (i = 0; i < 256; i = i + 1) data[30464 + i] = INIT_77[i];
        for (i = 0; i < 256; i = i + 1) data[30720 + i] = INIT_78[i];
        for (i = 0; i < 256; i = i + 1) data[30976 + i] = INIT_79[i];
        for (i = 0; i < 256; i = i + 1) data[31232 + i] = INIT_7A[i];
        for (i = 0; i < 256; i = i + 1) data[31488 + i] = INIT_7B[i];
        for (i = 0; i < 256; i = i + 1) data[31744 + i] = INIT_7C[i];
        for (i = 0; i < 256; i = i + 1) data[32000 + i] = INIT_7D[i];
        for (i = 0; i < 256; i = i + 1) data[32256 + i] = INIT_7E[i];
        for (i = 0; i < 256; i = i + 1) data[32512 + i] = INIT_7F[i];
        for (i = 0; i < 256; i = i + 1) par[0 + i] = INITP_00[i];
        for (i = 0; i < 256; i = i + 1) par[256 + i] = INITP_01[i];
        for (i = 0; i < 256; i = i + 1) par[512 + i] = INITP_02[i];
        for (i = 0; i < 256; i = i + 1) par[768 + i] = INITP_03[i];
        for (i = 0; i < 256; i = i + 1) par[1024 + i] = INITP_04[i];
        for (i = 0; i < 256; i = i + 1) par[1280 + i] = INITP_05[i];
        for (i = 0; i < 256; i = i + 1) par[1536 + i] = INITP_06[i];
        for (i = 0; i < 256; i = i + 1) par[1792 + i] = INITP_07[i];
        for (i = 0; i < 256; i = i + 1) par[2048 + i] = INITP_08[i];
        for (i = 0; i < 256; i = i + 1) par[2304 + i] = INITP_09[i];
        for (i = 0; i < 256; i = i + 1) par[2560 + i] = INITP_0A[i];
        for (i = 0; i < 256; i = i + 1) par[2816 + i] = INITP_0B[i];
        for (i = 0; i < 256; i = i + 1) par[3072 + i] = INITP_0C[i];
        for (i = 0; i < 256; i = i + 1) par[3328 + i] = INITP_0D[i];
        for (i = 0; i < 256; i = i + 1) par[3584 + i] = INITP_0E[i];
        for (i = 0; i < 256; i = i + 1) par[3840 + i] = INITP_0F[i];
        DOADO = INIT_A; DOBDO = INIT_B; DOPADOP = 0; DOPBDOP = 0;
    end

    // Bits per entry on the data / parity planes, and the lowest address
    // bit used, for a port width.
    function integer dbits(input integer w); dbits = (w % 9 == 0) ? w / 9 * 8 : w; endfunction
    function integer pbits(input integer w); pbits = (w % 9 == 0) ? w / 9 : 0; endfunction
    function integer lsb(input integer w);
        lsb = w == 1 ? 0 : w == 2 ? 1 : w == 4 ? 2 : w == 9 ? 3 : w == 18 ? 4 : w == 36 ? 5 : 6;
    endfunction

    localparam SDP = RAM_MODE == "SDP";
    // In SDP the read port is A and the write port B, both 72 wide.
    localparam RWA = READ_WIDTH_A;
    localparam RWB = SDP ? 0 : READ_WIDTH_B;
    localparam WWA = SDP ? 0 : WRITE_WIDTH_A;
    localparam WWB = WRITE_WIDTH_B;
    wire clka = CLKARDCLK ^ IS_CLKARDCLK_INVERTED;
    wire clkb = CLKBWRCLK ^ IS_CLKBWRCLK_INVERTED;

    // Entry index of an address for a width.
    function integer entry(input [15:0] a, input integer w);
        entry = a[14:0] >> lsb(w);
    endfunction

    reg [63:0] rd_da, rd_db; reg [7:0] rd_pa, rd_pb; integer ea, eb, ka, kb;
    always @(posedge clka) if (ENARDEN && RWA != 0) begin
        ea = entry(ADDRARDADDR, RWA);
        rd_da = 0; rd_pa = 0;
        for (ka = 0; ka < dbits(RWA); ka = ka + 1) rd_da[ka] = data[ea * dbits(RWA) + ka];
        for (ka = 0; ka < pbits(RWA); ka = ka + 1) rd_pa[ka] = par[ea * pbits(RWA) + ka];
        DOADO <= rd_da[31:0]; DOPADOP <= rd_pa[3:0];
        if (SDP) begin DOBDO <= rd_da[63:32]; DOPBDOP <= rd_pa[7:4]; end
    end
    always @(posedge clkb) if (ENBWREN && RWB != 0) begin
        eb = entry(ADDRBWRADDR, RWB);
        rd_db = 0; rd_pb = 0;
        for (kb = 0; kb < dbits(RWB); kb = kb + 1) rd_db[kb] = data[eb * dbits(RWB) + kb];
        for (kb = 0; kb < pbits(RWB); kb = kb + 1) rd_pb[kb] = par[eb * pbits(RWB) + kb];
        DOBDO <= rd_db[31:0]; DOPBDOP <= rd_pb[3:0];
    end

    // Writes land after this edge's reads (non-blocking): READ_FIRST.
    // Byte lane b covers data bits 8b..8b+7 and parity bit b; widths under
    // 9 use WE[0].
    reg [63:0] wd_da, wd_db; reg [7:0] wd_pa, wd_pb; reg [7:0] wea, web; integer fa, fb, ja, jb;
    always @(posedge clka) if (ENARDEN && WWA != 0) begin
        fa = entry(ADDRARDADDR, WWA);
        wd_da = DIADI; wd_pa = DIPADIP; wea = WEA;
        for (ja = 0; ja < dbits(WWA); ja = ja + 1) if (wea[WWA < 9 ? 0 : ja / 8]) data[fa * dbits(WWA) + ja] <= wd_da[ja];
        for (ja = 0; ja < pbits(WWA); ja = ja + 1) if (wea[WWA < 9 ? 0 : ja]) par[fa * pbits(WWA) + ja] <= wd_pa[ja];
    end
    always @(posedge clkb) if (ENBWREN && WWB != 0) begin
        fb = entry(ADDRBWRADDR, WWB);
        wd_db = SDP ? {DIBDI, DIADI} : DIBDI; wd_pb = SDP ? {DIPBDIP, DIPADIP} : DIPBDIP; web = WEBWE;
        for (jb = 0; jb < dbits(WWB); jb = jb + 1) if (web[WWB < 9 ? 0 : jb / 8]) data[fb * dbits(WWB) + jb] <= wd_db[jb];
        for (jb = 0; jb < pbits(WWB); jb = jb + 1) if (web[WWB < 9 ? 0 : jb]) par[fb * pbits(WWB) + jb] <= wd_pb[jb];
    end
endmodule

// Behavioural RAMB18E1: the subset oxibit-synth emits (bram.rs).
// TDP and SDP, widths 1/2/4/9/18/36 (and SDP x36), no output
// registers, READ_FIRST, ENx and byte write enables, INIT/INITP contents.
// Written from UG473, not derived from yosys or Vivado models.
module RAMB18E1 (
    input CLKARDCLK,
    input CLKBWRCLK,
    input ENARDEN,
    input ENBWREN,
    input REGCEAREGCE,
    input REGCEB,
    input RSTRAMARSTRAM,
    input RSTRAMB,
    input RSTREGARSTREG,
    input RSTREGB,
    input [13:0] ADDRARDADDR,
    input [13:0] ADDRBWRADDR,
    input [15:0] DIADI,
    input [15:0] DIBDI,
    input [1:0] DIPADIP,
    input [1:0] DIPBDIP,
    input [1:0] WEA,
    input [3:0] WEBWE,
    output reg [15:0] DOADO,
    output reg [15:0] DOBDO,
    output reg [1:0] DOPADOP,
    output reg [1:0] DOPBDOP
);
    parameter RAM_MODE = "TDP";
    parameter WRITE_MODE_A = "READ_FIRST";
    parameter WRITE_MODE_B = "READ_FIRST";
    parameter RAM_EXTENSION_A = "NONE";
    parameter RAM_EXTENSION_B = "NONE";
    parameter integer READ_WIDTH_A = 0;
    parameter integer READ_WIDTH_B = 0;
    parameter integer WRITE_WIDTH_A = 0;
    parameter integer WRITE_WIDTH_B = 0;
    parameter integer DOA_REG = 0;
    parameter integer DOB_REG = 0;
    parameter [17:0] INIT_A = 0;
    parameter [17:0] INIT_B = 0;
    parameter [17:0] SRVAL_A = 0;
    parameter [17:0] SRVAL_B = 0;
    parameter [0:0] IS_CLKARDCLK_INVERTED = 1'b0;
    parameter [0:0] IS_CLKBWRCLK_INVERTED = 1'b0;
    parameter [255:0] INIT_00 = 256'h0;
    parameter [255:0] INIT_01 = 256'h0;
    parameter [255:0] INIT_02 = 256'h0;
    parameter [255:0] INIT_03 = 256'h0;
    parameter [255:0] INIT_04 = 256'h0;
    parameter [255:0] INIT_05 = 256'h0;
    parameter [255:0] INIT_06 = 256'h0;
    parameter [255:0] INIT_07 = 256'h0;
    parameter [255:0] INIT_08 = 256'h0;
    parameter [255:0] INIT_09 = 256'h0;
    parameter [255:0] INIT_0A = 256'h0;
    parameter [255:0] INIT_0B = 256'h0;
    parameter [255:0] INIT_0C = 256'h0;
    parameter [255:0] INIT_0D = 256'h0;
    parameter [255:0] INIT_0E = 256'h0;
    parameter [255:0] INIT_0F = 256'h0;
    parameter [255:0] INIT_10 = 256'h0;
    parameter [255:0] INIT_11 = 256'h0;
    parameter [255:0] INIT_12 = 256'h0;
    parameter [255:0] INIT_13 = 256'h0;
    parameter [255:0] INIT_14 = 256'h0;
    parameter [255:0] INIT_15 = 256'h0;
    parameter [255:0] INIT_16 = 256'h0;
    parameter [255:0] INIT_17 = 256'h0;
    parameter [255:0] INIT_18 = 256'h0;
    parameter [255:0] INIT_19 = 256'h0;
    parameter [255:0] INIT_1A = 256'h0;
    parameter [255:0] INIT_1B = 256'h0;
    parameter [255:0] INIT_1C = 256'h0;
    parameter [255:0] INIT_1D = 256'h0;
    parameter [255:0] INIT_1E = 256'h0;
    parameter [255:0] INIT_1F = 256'h0;
    parameter [255:0] INIT_20 = 256'h0;
    parameter [255:0] INIT_21 = 256'h0;
    parameter [255:0] INIT_22 = 256'h0;
    parameter [255:0] INIT_23 = 256'h0;
    parameter [255:0] INIT_24 = 256'h0;
    parameter [255:0] INIT_25 = 256'h0;
    parameter [255:0] INIT_26 = 256'h0;
    parameter [255:0] INIT_27 = 256'h0;
    parameter [255:0] INIT_28 = 256'h0;
    parameter [255:0] INIT_29 = 256'h0;
    parameter [255:0] INIT_2A = 256'h0;
    parameter [255:0] INIT_2B = 256'h0;
    parameter [255:0] INIT_2C = 256'h0;
    parameter [255:0] INIT_2D = 256'h0;
    parameter [255:0] INIT_2E = 256'h0;
    parameter [255:0] INIT_2F = 256'h0;
    parameter [255:0] INIT_30 = 256'h0;
    parameter [255:0] INIT_31 = 256'h0;
    parameter [255:0] INIT_32 = 256'h0;
    parameter [255:0] INIT_33 = 256'h0;
    parameter [255:0] INIT_34 = 256'h0;
    parameter [255:0] INIT_35 = 256'h0;
    parameter [255:0] INIT_36 = 256'h0;
    parameter [255:0] INIT_37 = 256'h0;
    parameter [255:0] INIT_38 = 256'h0;
    parameter [255:0] INIT_39 = 256'h0;
    parameter [255:0] INIT_3A = 256'h0;
    parameter [255:0] INIT_3B = 256'h0;
    parameter [255:0] INIT_3C = 256'h0;
    parameter [255:0] INIT_3D = 256'h0;
    parameter [255:0] INIT_3E = 256'h0;
    parameter [255:0] INIT_3F = 256'h0;
    parameter [255:0] INITP_00 = 256'h0;
    parameter [255:0] INITP_01 = 256'h0;
    parameter [255:0] INITP_02 = 256'h0;
    parameter [255:0] INITP_03 = 256'h0;
    parameter [255:0] INITP_04 = 256'h0;
    parameter [255:0] INITP_05 = 256'h0;
    parameter [255:0] INITP_06 = 256'h0;
    parameter [255:0] INITP_07 = 256'h0;

    reg data [0:16383];
    reg par [0:2047];
    integer i;
    initial begin
        for (i = 0; i < 256; i = i + 1) data[0 + i] = INIT_00[i];
        for (i = 0; i < 256; i = i + 1) data[256 + i] = INIT_01[i];
        for (i = 0; i < 256; i = i + 1) data[512 + i] = INIT_02[i];
        for (i = 0; i < 256; i = i + 1) data[768 + i] = INIT_03[i];
        for (i = 0; i < 256; i = i + 1) data[1024 + i] = INIT_04[i];
        for (i = 0; i < 256; i = i + 1) data[1280 + i] = INIT_05[i];
        for (i = 0; i < 256; i = i + 1) data[1536 + i] = INIT_06[i];
        for (i = 0; i < 256; i = i + 1) data[1792 + i] = INIT_07[i];
        for (i = 0; i < 256; i = i + 1) data[2048 + i] = INIT_08[i];
        for (i = 0; i < 256; i = i + 1) data[2304 + i] = INIT_09[i];
        for (i = 0; i < 256; i = i + 1) data[2560 + i] = INIT_0A[i];
        for (i = 0; i < 256; i = i + 1) data[2816 + i] = INIT_0B[i];
        for (i = 0; i < 256; i = i + 1) data[3072 + i] = INIT_0C[i];
        for (i = 0; i < 256; i = i + 1) data[3328 + i] = INIT_0D[i];
        for (i = 0; i < 256; i = i + 1) data[3584 + i] = INIT_0E[i];
        for (i = 0; i < 256; i = i + 1) data[3840 + i] = INIT_0F[i];
        for (i = 0; i < 256; i = i + 1) data[4096 + i] = INIT_10[i];
        for (i = 0; i < 256; i = i + 1) data[4352 + i] = INIT_11[i];
        for (i = 0; i < 256; i = i + 1) data[4608 + i] = INIT_12[i];
        for (i = 0; i < 256; i = i + 1) data[4864 + i] = INIT_13[i];
        for (i = 0; i < 256; i = i + 1) data[5120 + i] = INIT_14[i];
        for (i = 0; i < 256; i = i + 1) data[5376 + i] = INIT_15[i];
        for (i = 0; i < 256; i = i + 1) data[5632 + i] = INIT_16[i];
        for (i = 0; i < 256; i = i + 1) data[5888 + i] = INIT_17[i];
        for (i = 0; i < 256; i = i + 1) data[6144 + i] = INIT_18[i];
        for (i = 0; i < 256; i = i + 1) data[6400 + i] = INIT_19[i];
        for (i = 0; i < 256; i = i + 1) data[6656 + i] = INIT_1A[i];
        for (i = 0; i < 256; i = i + 1) data[6912 + i] = INIT_1B[i];
        for (i = 0; i < 256; i = i + 1) data[7168 + i] = INIT_1C[i];
        for (i = 0; i < 256; i = i + 1) data[7424 + i] = INIT_1D[i];
        for (i = 0; i < 256; i = i + 1) data[7680 + i] = INIT_1E[i];
        for (i = 0; i < 256; i = i + 1) data[7936 + i] = INIT_1F[i];
        for (i = 0; i < 256; i = i + 1) data[8192 + i] = INIT_20[i];
        for (i = 0; i < 256; i = i + 1) data[8448 + i] = INIT_21[i];
        for (i = 0; i < 256; i = i + 1) data[8704 + i] = INIT_22[i];
        for (i = 0; i < 256; i = i + 1) data[8960 + i] = INIT_23[i];
        for (i = 0; i < 256; i = i + 1) data[9216 + i] = INIT_24[i];
        for (i = 0; i < 256; i = i + 1) data[9472 + i] = INIT_25[i];
        for (i = 0; i < 256; i = i + 1) data[9728 + i] = INIT_26[i];
        for (i = 0; i < 256; i = i + 1) data[9984 + i] = INIT_27[i];
        for (i = 0; i < 256; i = i + 1) data[10240 + i] = INIT_28[i];
        for (i = 0; i < 256; i = i + 1) data[10496 + i] = INIT_29[i];
        for (i = 0; i < 256; i = i + 1) data[10752 + i] = INIT_2A[i];
        for (i = 0; i < 256; i = i + 1) data[11008 + i] = INIT_2B[i];
        for (i = 0; i < 256; i = i + 1) data[11264 + i] = INIT_2C[i];
        for (i = 0; i < 256; i = i + 1) data[11520 + i] = INIT_2D[i];
        for (i = 0; i < 256; i = i + 1) data[11776 + i] = INIT_2E[i];
        for (i = 0; i < 256; i = i + 1) data[12032 + i] = INIT_2F[i];
        for (i = 0; i < 256; i = i + 1) data[12288 + i] = INIT_30[i];
        for (i = 0; i < 256; i = i + 1) data[12544 + i] = INIT_31[i];
        for (i = 0; i < 256; i = i + 1) data[12800 + i] = INIT_32[i];
        for (i = 0; i < 256; i = i + 1) data[13056 + i] = INIT_33[i];
        for (i = 0; i < 256; i = i + 1) data[13312 + i] = INIT_34[i];
        for (i = 0; i < 256; i = i + 1) data[13568 + i] = INIT_35[i];
        for (i = 0; i < 256; i = i + 1) data[13824 + i] = INIT_36[i];
        for (i = 0; i < 256; i = i + 1) data[14080 + i] = INIT_37[i];
        for (i = 0; i < 256; i = i + 1) data[14336 + i] = INIT_38[i];
        for (i = 0; i < 256; i = i + 1) data[14592 + i] = INIT_39[i];
        for (i = 0; i < 256; i = i + 1) data[14848 + i] = INIT_3A[i];
        for (i = 0; i < 256; i = i + 1) data[15104 + i] = INIT_3B[i];
        for (i = 0; i < 256; i = i + 1) data[15360 + i] = INIT_3C[i];
        for (i = 0; i < 256; i = i + 1) data[15616 + i] = INIT_3D[i];
        for (i = 0; i < 256; i = i + 1) data[15872 + i] = INIT_3E[i];
        for (i = 0; i < 256; i = i + 1) data[16128 + i] = INIT_3F[i];
        for (i = 0; i < 256; i = i + 1) par[0 + i] = INITP_00[i];
        for (i = 0; i < 256; i = i + 1) par[256 + i] = INITP_01[i];
        for (i = 0; i < 256; i = i + 1) par[512 + i] = INITP_02[i];
        for (i = 0; i < 256; i = i + 1) par[768 + i] = INITP_03[i];
        for (i = 0; i < 256; i = i + 1) par[1024 + i] = INITP_04[i];
        for (i = 0; i < 256; i = i + 1) par[1280 + i] = INITP_05[i];
        for (i = 0; i < 256; i = i + 1) par[1536 + i] = INITP_06[i];
        for (i = 0; i < 256; i = i + 1) par[1792 + i] = INITP_07[i];
        DOADO = INIT_A; DOBDO = INIT_B; DOPADOP = 0; DOPBDOP = 0;
    end

    // Bits per entry on the data / parity planes, and the lowest address
    // bit used, for a port width.
    function integer dbits(input integer w); dbits = (w % 9 == 0) ? w / 9 * 8 : w; endfunction
    function integer pbits(input integer w); pbits = (w % 9 == 0) ? w / 9 : 0; endfunction
    function integer lsb(input integer w);
        lsb = w == 1 ? 0 : w == 2 ? 1 : w == 4 ? 2 : w == 9 ? 3 : w == 18 ? 4 : w == 36 ? 5 : 6;
    endfunction

    localparam SDP = RAM_MODE == "SDP";
    // In SDP the read port is A and the write port B, both 36 wide.
    localparam RWA = READ_WIDTH_A;
    localparam RWB = SDP ? 0 : READ_WIDTH_B;
    localparam WWA = SDP ? 0 : WRITE_WIDTH_A;
    localparam WWB = WRITE_WIDTH_B;
    wire clka = CLKARDCLK ^ IS_CLKARDCLK_INVERTED;
    wire clkb = CLKBWRCLK ^ IS_CLKBWRCLK_INVERTED;

    // Entry index of an address for a width.
    function integer entry(input [13:0] a, input integer w);
        entry = a[13:0] >> lsb(w);
    endfunction

    reg [31:0] rd_da, rd_db; reg [3:0] rd_pa, rd_pb; integer ea, eb, ka, kb;
    always @(posedge clka) if (ENARDEN && RWA != 0) begin
        ea = entry(ADDRARDADDR, RWA);
        rd_da = 0; rd_pa = 0;
        for (ka = 0; ka < dbits(RWA); ka = ka + 1) rd_da[ka] = data[ea * dbits(RWA) + ka];
        for (ka = 0; ka < pbits(RWA); ka = ka + 1) rd_pa[ka] = par[ea * pbits(RWA) + ka];
        DOADO <= rd_da[15:0]; DOPADOP <= rd_pa[1:0];
        if (SDP) begin DOBDO <= rd_da[31:16]; DOPBDOP <= rd_pa[3:2]; end
    end
    always @(posedge clkb) if (ENBWREN && RWB != 0) begin
        eb = entry(ADDRBWRADDR, RWB);
        rd_db = 0; rd_pb = 0;
        for (kb = 0; kb < dbits(RWB); kb = kb + 1) rd_db[kb] = data[eb * dbits(RWB) + kb];
        for (kb = 0; kb < pbits(RWB); kb = kb + 1) rd_pb[kb] = par[eb * pbits(RWB) + kb];
        DOBDO <= rd_db[15:0]; DOPBDOP <= rd_pb[1:0];
    end

    // Writes land after this edge's reads (non-blocking): READ_FIRST.
    // Byte lane b covers data bits 8b..8b+7 and parity bit b; widths under
    // 9 use WE[0].
    reg [31:0] wd_da, wd_db; reg [3:0] wd_pa, wd_pb; reg [7:0] wea, web; integer fa, fb, ja, jb;
    always @(posedge clka) if (ENARDEN && WWA != 0) begin
        fa = entry(ADDRARDADDR, WWA);
        wd_da = DIADI; wd_pa = DIPADIP; wea = WEA;
        for (ja = 0; ja < dbits(WWA); ja = ja + 1) if (wea[WWA < 9 ? 0 : ja / 8]) data[fa * dbits(WWA) + ja] <= wd_da[ja];
        for (ja = 0; ja < pbits(WWA); ja = ja + 1) if (wea[WWA < 9 ? 0 : ja]) par[fa * pbits(WWA) + ja] <= wd_pa[ja];
    end
    always @(posedge clkb) if (ENBWREN && WWB != 0) begin
        fb = entry(ADDRBWRADDR, WWB);
        wd_db = SDP ? {DIBDI, DIADI} : DIBDI; wd_pb = SDP ? {DIPBDIP, DIPADIP} : DIPBDIP; web = WEBWE;
        for (jb = 0; jb < dbits(WWB); jb = jb + 1) if (web[WWB < 9 ? 0 : jb / 8]) data[fb * dbits(WWB) + jb] <= wd_db[jb];
        for (jb = 0; jb < pbits(WWB); jb = jb + 1) if (web[WWB < 9 ? 0 : jb]) par[fb * pbits(WWB) + jb] <= wd_pb[jb];
    end
endmodule
