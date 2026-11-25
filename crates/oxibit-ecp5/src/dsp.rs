//! sysDSP: MULT18X18D and ALU54B.
//!
//! The 9x9 split multiplier (MULT9X9D), the 24-bit ALU split (ALU24B) and
//! the pre-adders (PRADD9A/PRADD18A) share these tiles; their fuse tables
//! are decoded below and pending silicon confirmation.
//!
//! A DSP block spans nine tiles (`MIB_DSP0..8`, with `MIB2_DSP*`); its
//! MULT18 bels sit at columns 0, 1, 4, 5 of the block and its ALU54 bels at
//! 3 and 7, so a bel at column x with index z belongs to the block
//! starting at x - z. An ALU54B takes the two MULT18X18Ds of its half (ALU
//! 3 with MULT 0 on MA and MULT 1 on MB, ALU 7 with 4 and 5), as nextpnr's
//! `pack_dsps` constrains them.
//!
//! Settings follow nextpnr's `write_mult18` / `write_alu54`, written into
//! every tile of the block that has them: the register and reset options
//! from the parameters, `MODE`, `CIBOUT_BYP ON` for an unregistered
//! MULT18, the block side's `CIBOUT ON`, and the CLK / CE / RST muxes set
//! to pass. Unconnected data inputs are tied low (`tieoff_dsp_ports`);
//! inputs synthesis tied to a constant are tied to it.
//!
//! Synthesis keeps them as black-box cells: the random check does not
//! model the arithmetic, the silicon probe does.

// DSP fuse/setting tables, from the local model.
//
// Decoded from the vendor device database: each (bel-type, setting)
// pair maps to fuse-group values. The "fuse bytes" are the MC1_MIBMEM
// indices of the bit(s) that encode that setting; the physical bitstream
// frame mapping for those groups is what the DSP fuzzing must confirm
// (only the MULT18/ALU54 groups are already mapped by Trellis).
//
// Setting spaces (all values verified present in the database):
//   ALU24:
//     0: CLK0, CLK1, CLK2, CLK3, OPADDNSUB, OPCINSEL
//     1: CE0, CE1, CE2, CE3, RST0, RST1, RST2, RST3
//     CLK0_DIV: DISABLED
//     CLK1_DIV: DISABLED
//     CLK2_DIV: DISABLED
//     CLK3_DIV: DISABLED
//     GSR: DISABLED
//     INV: CLK0, CLK1, CLK2, CLK3, OPADDNSUB, OPCINSEL
//     OUT2CIB: YES
//     OUT2CIN: YES
//     REG_INPUTCFB_CE: CE0, CE1, CE2
//     REG_INPUTCFB_CLK: CLK0, CLK1, CLK2, CLK3, NONE
//     REG_INPUTCFB_RST: RST0, RST1, RST2
//     REG_OPCODE_0_CE: CE0, CE1, CE2
//     REG_OPCODE_0_CLK: CLK0, CLK1, CLK2, NONE
//     REG_OPCODE_0_RST: RST0, RST1, RST2
//     REG_OPCODE_1_CE: CE0, CE1, CE2
//     REG_OPCODE_1_CLK: CLK0, CLK1, CLK2, NONE
//     REG_OPCODE_1_RST: RST0, RST1, RST2
//     REG_OUTPUT_CE: CE0, CE1, CE2
//     REG_OUTPUT_CLK: CLK0, CLK1, CLK2, NONE
//     REG_OUTPUT_RST: RST0, RST1, RST2
//     RESETMODE: ASYNC
//     SIG: CE0, CE1, CE2, CE3, RST0, RST1, RST2, RST3
//   ALU54:
//     0: CLK0, CLK1, CLK2, CLK3, OP0, OP1, OP10, OP2, OP3, OP4, OP5, OP6, OP7, OP8, OP9, OPADDNSUB, OPCINSEL
//     1: CE0, CE1, CE2, CE3, RST0, RST1, RST2, RST3
//     CLK0_DIV: DISABLED
//     CLK1_DIV: DISABLED
//     CLK2_DIV: DISABLED
//     CLK3_DIV: DISABLED
//     FLAG2CIB: YES
//     FORCE_ZERO_BARREL_SHIFT: DISABLED
//     GSR: DISABLED
//     INV: CLK0, CLK1, CLK2, CLK3, OP0, OP1, OP10, OP2, OP3, OP4, OP5, OP6, OP7, OP8, OP9, OPADDNSUB, OPCINSEL
//     LEGACY: ENABLED
//     MASKPAT_SOURCE: STATIC
//     MCPAT_SOURCE: STATIC
//     OUT2CIB: YES
//     OUT2CIN: YES
//     REG_FLAG_CE: CE0, CE1, CE2
//     REG_FLAG_CLK: CLK0, CLK1, CLK2, NONE
//     REG_FLAG_RST: RST0, RST1, RST2
//     REG_INPUTC0_CE: CE0, CE1, CE2
//     REG_INPUTC0_CLK: CLK0, CLK1, CLK2, NONE
//     REG_INPUTC0_RST: RST0, RST1, RST2
//     REG_INPUTC1_CE: CE0, CE1, CE2
//     REG_INPUTC1_CLK: CLK0, CLK1, CLK2, NONE
//     REG_INPUTC1_RST: RST0, RST1, RST2
//     REG_INPUTCFB_CLK: CLK0, CLK1, CLK2, CLK3, NONE
//     REG_OPCODEIN_0_CE: CE0, CE1, CE2
//     REG_OPCODEIN_0_CLK: CLK0, CLK1, CLK2, NONE
//     REG_OPCODEIN_0_RST: RST0, RST1, RST2
//     REG_OPCODEIN_1_CE: CE0, CE1, CE2
//     REG_OPCODEIN_1_CLK: CLK0, CLK1, CLK2, NONE
//     REG_OPCODEIN_1_RST: RST0, RST1, RST2
//     REG_OPCODEOP0_0_CE: CE0, CE1, CE2
//     REG_OPCODEOP0_0_CLK: CLK0, CLK1, CLK2, NONE
//     REG_OPCODEOP0_0_RST: RST0, RST1, RST2
//     REG_OPCODEOP0_1_CE: CE0, CE1, CE2
//     REG_OPCODEOP0_1_CLK: CLK0, CLK1, CLK2, NONE
//     REG_OPCODEOP0_1_RST: RST0, RST1, RST2
//     REG_OPCODEOP1_0_CLK: CLK0, CLK1, CLK2, NONE
//     REG_OPCODEOP1_1_CLK: CLK0, CLK1, CLK2, NONE
//     REG_OPCODE_0_CE: CE0, CE1, CE2
//     REG_OPCODE_0_CLK: CLK0, CLK1, CLK2, NONE
//     REG_OPCODE_0_RST: RST0, RST1, RST2
//     REG_OPCODE_1_CE: CE0, CE1, CE2
//     REG_OPCODE_1_CLK: CLK0, CLK1, CLK2, NONE
//     REG_OPCODE_1_RST: RST0, RST1, RST2
//     REG_OUTPUT0_CE: CE0, CE1, CE2
//     REG_OUTPUT0_CLK: CLK0, CLK1, CLK2, NONE
//     REG_OUTPUT0_RST: RST0, RST1, RST2
//     REG_OUTPUT1_CE: CE0, CE1, CE2
//     REG_OUTPUT1_CLK: CLK0, CLK1, CLK2, NONE
//     REG_OUTPUT1_RST: RST0, RST1, RST2
//     REG_OUTPUT_CE: CE0, CE1, CE2
//     REG_OUTPUT_CLK: CLK0, CLK1, CLK2, NONE
//     REG_OUTPUT_RST: RST0, RST1, RST2
//     RESETMODE: ASYNC
//     SIG: CE0, CE1, CE2, CE3, RST0, RST1, RST2, RST3
//   MULT18:
//     0: CLK0, CLK1, CLK2, CLK3, SIGNEDA, SIGNEDB, SOURCEA, SOURCEB
//     1: CE0, CE1, CE2, CE3, RST0, RST1, RST2, RST3, SOURCEA, SOURCEB
//     CAS_MATCH_REG: FALSE
//     CLK0_DIV: DISABLED
//     CLK1_DIV: DISABLED
//     CLK2_DIV: DISABLED
//     CLK3_DIV: DISABLED
//     GSR: DISABLED
//     HIGHSPEED_CLK: CLK0, CLK1, CLK2
//     INV: CLK0, CLK1, CLK2, CLK3, SIGNEDA, SIGNEDB, SOURCEA, SOURCEB
//     OUT2CIB: YES
//     REG_INPUTA_CE: CE0, CE1, CE2
//     REG_INPUTA_CLK: CLK0, CLK1, CLK2, NONE
//     REG_INPUTA_RST: RST0, RST1, RST2
//     REG_INPUTB_CE: CE0, CE1, CE2
//     REG_INPUTB_CLK: CLK0, CLK1, CLK2, NONE
//     REG_INPUTB_RST: RST0, RST1, RST2
//     REG_INPUTC_CLK: NONE
//     REG_OUTPUT_CE: CE0, CE1, CE2
//     REG_OUTPUT_CLK: CLK0, CLK1, CLK2, NONE
//     REG_OUTPUT_RST: RST0, RST1, RST2
//     REG_PIPELINE_CE: CE0, CE1, CE2
//     REG_PIPELINE_CLK: CLK0, CLK1, CLK2, NONE
//     REG_PIPELINE_RST: RST0, RST1, RST2
//     RESETMODE: ASYNC
//     SIG: CE0, CE1, CE2, CE3, RST0, RST1, RST2, RST3, SOURCEA, SOURCEB
//     SOURCEB_MODE: B_C_DYNAMIC, B_SHIFT, C_SHIFT, HIGHSPEED
//     SRO2CIB: NO, YES
//   MULT9:
//     0: CLK0, CLK1, CLK2, CLK3, SIGNEDA, SIGNEDB, SOURCEA, SOURCEB
//     1: CE0, CE1, CE2, CE3, RST0, RST1, RST2, RST3, SOURCEA, SOURCEB
//     CAS_MATCH_REG: FALSE
//     CLK0_DIV: DISABLED
//     CLK1_DIV: DISABLED
//     CLK2_DIV: DISABLED
//     CLK3_DIV: DISABLED
//     GSR: DISABLED
//     HIGHSPEED_CLK: CLK0, CLK1, CLK2
//     INV: CLK0, CLK1, CLK2, CLK3, SIGNEDA, SIGNEDB, SOURCEA, SOURCEB
//     OUT2CIB: YES
//     REG_INPUTA_CE: CE0, CE1, CE2
//     REG_INPUTA_CLK: CLK0, CLK1, CLK2, NONE
//     REG_INPUTA_RST: RST0, RST1, RST2
//     REG_INPUTB_CE: CE0, CE1, CE2
//     REG_INPUTB_CLK: CLK0, CLK1, CLK2, NONE
//     REG_INPUTB_RST: RST0, RST1, RST2
//     REG_INPUTC_CE: CE0, CE1, CE2
//     REG_INPUTC_CLK: CLK0, CLK1, CLK2, NONE
//     REG_INPUTC_RST: RST0, RST1, RST2
//     REG_OUTPUT_CE: CE0, CE1, CE2
//     REG_OUTPUT_CLK: CLK0, CLK1, CLK2, NONE
//     REG_OUTPUT_RST: RST0, RST1, RST2
//     REG_PIPELINE_CE: CE0, CE1, CE2
//     REG_PIPELINE_CLK: CLK0, CLK1, CLK2, NONE
//     REG_PIPELINE_RST: RST0, RST1, RST2
//     RESETMODE: ASYNC
//     SIG: CE0, CE1, CE2, CE3, RST0, RST1, RST2, RST3, SOURCEA, SOURCEB
//     SOURCEB_MODE: B_C_DYNAMIC, B_SHIFT, C_SHIFT, HIGHSPEED
//     SRO2CIB: NO
//   PRADD18:
//     0: CLK0, CLK1, CLK2, CLK3
//     1: CE0, CE1, CE2, CE3, PB0, PB1, PB10, PB11, PB12, PB13, PB14, PB15, PB16, PB17, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9, RST0, RST1, RST2, RST3, SOURCEA
//     CAS_MATCH_REG: FALSE
//     CLK0_DIV: DISABLED
//     CLK1_DIV: DISABLED
//     CLK2_DIV: DISABLED
//     CLK3_DIV: DISABLED
//     FB_MUX: SHIFT, SHIFT_BYPASS
//     GSR: DISABLED
//     HIGHSPEED_CLK: CLK0, CLK1, CLK2
//     INV: CLK0, CLK1, CLK2, CLK3, OPPRE, SOURCEA
//     REG_INPUTA_CE: CE0, CE1, CE2
//     REG_INPUTA_CLK: CLK0, CLK1, CLK2, NONE
//     REG_INPUTA_RST: RST0, RST1, RST2
//     REG_INPUTB_CE: CE0, CE1, CE2
//     REG_INPUTB_CLK: CLK0, CLK1, CLK2, NONE
//     REG_INPUTB_RST: RST0, RST1, RST2
//     REG_INPUTC_CLK: NONE
//     REG_OPPRE_CE: CE0, CE1, CE2
//     REG_OPPRE_CLK: CLK0, CLK1, CLK2, NONE
//     REG_OPPRE_RST: RST0, RST1, RST2
//     RESETMODE: ASYNC
//     SIG: CE0, CE1, CE2, CE3, PB0, PB1, PB10, PB11, PB12, PB13, PB14, PB15, PB16, PB17, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9, RST0, RST1, RST2, RST3, SOURCEA
//     SOURCEA_MODE: A_C_DYNAMIC, A_SHIFT, C_SHIFT, HIGHSPEED
//     SOURCEB_MODE: INTERNAL, PARALLEL, SHIFT
//     SYMMETRY_MODE: DIRECT
//   PRADD9:
//     0: CLK0, CLK1, CLK2, CLK3
//     1: CE0, CE1, CE2, CE3, PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, RST0, RST1, RST2, RST3, SOURCEA
//     CAS_MATCH_REG: FALSE
//     CLK0_DIV: DISABLED
//     CLK1_DIV: DISABLED
//     CLK2_DIV: DISABLED
//     CLK3_DIV: DISABLED
//     FB_MUX: SHIFT, SHIFT_BYPASS
//     GSR: DISABLED
//     HIGHSPEED_CLK: CLK0, CLK1, CLK2
//     INV: CLK0, CLK1, CLK2, CLK3, OPPRE, SOURCEA
//     REG_INPUTA_CE: CE0, CE1, CE2
//     REG_INPUTA_CLK: CLK0, CLK1, CLK2, NONE
//     REG_INPUTA_RST: RST0, RST1, RST2
//     REG_INPUTB_CE: CE0, CE1, CE2
//     REG_INPUTB_CLK: CLK0, CLK1, CLK2, NONE
//     REG_INPUTB_RST: RST0, RST1, RST2
//     REG_INPUTC_CE: CE0, CE1, CE2
//     REG_INPUTC_CLK: CLK0, CLK1, CLK2, NONE
//     REG_INPUTC_RST: RST0, RST1, RST2
//     REG_OPPRE_CE: CE0, CE1, CE2
//     REG_OPPRE_CLK: CLK0, CLK1, CLK2, NONE
//     REG_OPPRE_RST: RST0, RST1, RST2
//     RESETMODE: ASYNC
//     SIG: CE0, CE1, CE2, CE3, PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, RST0, RST1, RST2, RST3, SOURCEA
//     SOURCEA_MODE: A_C_DYNAMIC, A_SHIFT, C_SHIFT, HIGHSPEED
//     SOURCEB_MODE: INTERNAL, PARALLEL, SHIFT
//     SYMMETRY_MODE: DIRECT

use std::collections::{BTreeMap, HashMap, HashSet};

use crate::bels::{BelId, Bels};
use crate::chip::Level;
use crate::config::TileConfig;
use crate::db::DeviceDb;
use crate::graph::Graph;
use crate::netlist::{Cell, Netlist, Prim, PrimPin};
use crate::{Result, err};

/// Port declarations (from the Lattice library's port lists), for synthesis.
pub const PRIMITIVES: &str = "
module MULT18X18D(input A17, A16, A15, A14, A13, A12, A11, A10, A9, A8, A7, A6, A5, A4, A3, A2, 
A1, A0, B17, B16, B15, B14, B13, B12, B11, B10, B9, B8, B7, B6, B5, B4, B3, B2, B1, B0, C17, 
C16, C15, C14, C13, C12, C11, C10, C9, C8, C7, C6, C5, C4, C3, C2, C1, C0, SIGNEDA, SIGNEDB, 
SOURCEA, SOURCEB, CLK3, CLK2, CLK1, CLK0, CE3, CE2, CE1, CE0, RST3, RST2, RST1, RST0, SRIA17, 
SRIA16, SRIA15, SRIA14, SRIA13, SRIA12, SRIA11, SRIA10, SRIA9, SRIA8, SRIA7, SRIA6, SRIA5, 
SRIA4, SRIA3, SRIA2, SRIA1, SRIA0, SRIB17, SRIB16, SRIB15, SRIB14, SRIB13, SRIB12, SRIB11, 
SRIB10, SRIB9, SRIB8, SRIB7, SRIB6, SRIB5, SRIB4, SRIB3, SRIB2, SRIB1, SRIB0,
  output SROA17, SROA16, SROA15, SROA14, SROA13, SROA12, SROA11, SROA10, SROA9, SROA8, SROA7, 
SROA6, SROA5, SROA4, SROA3, SROA2, SROA1, SROA0, SROB17, SROB16, SROB15, SROB14, SROB13, 
SROB12, SROB11, SROB10, SROB9, SROB8, SROB7, SROB6, SROB5, SROB4, SROB3, SROB2, SROB1, SROB0, 
ROA17, ROA16, ROA15, ROA14, ROA13, ROA12, ROA11, ROA10, ROA9, ROA8, ROA7, ROA6, ROA5, ROA4, 
ROA3, ROA2, ROA1, ROA0, ROB17, ROB16, ROB15, ROB14, ROB13, ROB12, ROB11, ROB10, ROB9, ROB8, 
ROB7, ROB6, ROB5, ROB4, ROB3, ROB2, ROB1, ROB0, ROC17, ROC16, ROC15, ROC14, ROC13, ROC12, 
ROC11, ROC10, ROC9, ROC8, ROC7, ROC6, ROC5, ROC4, ROC3, ROC2, ROC1, ROC0, P35, P34, P33, P32, 
P31, P30, P29, P28, P27, P26, P25, P24, P23, P22, P21, P20, P19, P18, P17, P16, P15, P14, P13, 
P12, P11, P10, P9, P8, P7, P6, P5, P4, P3, P2, P1, P0, SIGNEDP);
endmodule
module ALU54B(input CE3, CE2, CE1, CE0, CLK3, CLK2, CLK1, CLK0, RST3, RST2, RST1, RST0, 
SIGNEDIA, SIGNEDIB, SIGNEDCIN, A35, A34, A33, A32, A31, A30, A29, A28, A27, A26, A25, A24, A23, 
A22, A21, A20, A19, A18, A17, A16, A15, A14, A13, A12, A11, A10, A9, A8, A7, A6, A5, A4, A3, 
A2, A1, A0, B35, B34, B33, B32, B31, B30, B29, B28, B27, B26, B25, B24, B23, B22, B21, B20, 
B19, B18, B17, B16, B15, B14, B13, B12, B11, B10, B9, B8, B7, B6, B5, B4, B3, B2, B1, B0, C53, 
C52, C51, C50, C49, C48, C47, C46, C45, C44, C43, C42, C41, C40, C39, C38, C37, C36, C35, C34, 
C33, C32, C31, C30, C29, C28, C27, C26, C25, C24, C23, C22, C21, C20, C19, C18, C17, C16, C15, 
C14, C13, C12, C11, C10, C9, C8, C7, C6, C5, C4, C3, C2, C1, C0, CFB53, CFB52, CFB51, CFB50, 
CFB49, CFB48, CFB47, CFB46, CFB45, CFB44, CFB43, CFB42, CFB41, CFB40, CFB39, CFB38, CFB37, 
CFB36, CFB35, CFB34, CFB33, CFB32, CFB31, CFB30, CFB29, CFB28, CFB27, CFB26, CFB25, CFB24, 
CFB23, CFB22, CFB21, CFB20, CFB19, CFB18, CFB17, CFB16, CFB15, CFB14, CFB13, CFB12, CFB11, 
CFB10, CFB9, CFB8, CFB7, CFB6, CFB5, CFB4, CFB3, CFB2, CFB1, CFB0, MA35, MA34, MA33, MA32, 
MA31, MA30, MA29, MA28, MA27, MA26, MA25, MA24, MA23, MA22, MA21, MA20, MA19, MA18, MA17, MA16, 
MA15, MA14, MA13, MA12, MA11, MA10, MA9, MA8, MA7, MA6, MA5, MA4, MA3, MA2, MA1, MA0, MB35, 
MB34, MB33, MB32, MB31, MB30, MB29, MB28, MB27, MB26, MB25, MB24, MB23, MB22, MB21, MB20, MB19, 
MB18, MB17, MB16, MB15, MB14, MB13, MB12, MB11, MB10, MB9, MB8, MB7, MB6, MB5, MB4, MB3, MB2, 
MB1, MB0, CIN53, CIN52, CIN51, CIN50, CIN49, CIN48, CIN47, CIN46, CIN45, CIN44, CIN43, CIN42, 
CIN41, CIN40, CIN39, CIN38, CIN37, CIN36, CIN35, CIN34, CIN33, CIN32, CIN31, CIN30, CIN29, 
CIN28, CIN27, CIN26, CIN25, CIN24, CIN23, CIN22, CIN21, CIN20, CIN19, CIN18, CIN17, CIN16, 
CIN15, CIN14, CIN13, CIN12, CIN11, CIN10, CIN9, CIN8, CIN7, CIN6, CIN5, CIN4, CIN3, CIN2, CIN1, 
CIN0, OP10, OP9, OP8, OP7, OP6, OP5, OP4, OP3, OP2, OP1, OP0,
  output R53, R52, R51, R50, R49, R48, R47, R46, R45, R44, R43, R42, R41, R40, R39, R38, R37, 
R36, R35, R34, R33, R32, R31, R30, R29, R28, R27, R26, R25, R24, R23, R22, R21, R20, R19, R18, 
R17, R16, R15, R14, R13, R12, R11, R10, R9, R8, R7, R6, R5, R4, R3, R2, R1, R0, CO53, CO52, 
CO51, CO50, CO49, CO48, CO47, CO46, CO45, CO44, CO43, CO42, CO41, CO40, CO39, CO38, CO37, CO36, 
CO35, CO34, CO33, CO32, CO31, CO30, CO29, CO28, CO27, CO26, CO25, CO24, CO23, CO22, CO21, CO20, 
CO19, CO18, CO17, CO16, CO15, CO14, CO13, CO12, CO11, CO10, CO9, CO8, CO7, CO6, CO5, CO4, CO3, 
CO2, CO1, CO0, EQZ, EQZM, EQOM, EQPAT, EQPATB, OVER, UNDER, OVERUNDER, SIGNEDR);
endmodule
module MULT9X9D(input A8, A7, A6, A5, A4, A3, A2, A1, A0, B8, B7, B6, B5, B4, B3, B2, B1, B0, \
C8, C7, C6, C5, C4, C3, C2, C1, C0, SIGNEDA, SIGNEDB, SIGNEDP, SOURCEA, SOURCEB, \
SRIA8, SRIA7, SRIA6, SRIA5, SRIA4, SRIA3, SRIA2, SRIA1, SRIA0, SRIB8, SRIB7, SRIB6, SRIB5, \
SRIB4, SRIB3, SRIB2, SRIB1, SRIB0, CLK3, CLK2, CLK1, CLK0, CE3, CE2, CE1, CE0, RST3, RST2, \
RST1, RST0, output SROA8, SROA7, SROA6, SROA5, SROA4, SROA3, SROA2, SROA1, SROA0, \
SROB8, SROB7, SROB6, SROB5, SROB4, SROB3, SROB2, SROB1, SROB0, \
ROC8, ROC7, ROC6, ROC5, ROC4, ROC3, ROC2, ROC1, ROC0, \
P17, P16, P15, P14, P13, P12, P11, P10, P9, P8, P7, P6, P5, P4, P3, P2, P1, P0);
endmodule
module PRADD9A(input PA8, PA7, PA6, PA5, PA4, PA3, PA2, PA1, PA0, \
PB8, PB7, PB6, PB5, PB4, PB3, PB2, PB1, PB0, \
C8, C7, C6, C5, C4, C3, C2, C1, C0, \
SRIA8, SRIA7, SRIA6, SRIA5, SRIA4, SRIA3, SRIA2, SRIA1, SRIA0, \
SRIB8, SRIB7, SRIB6, SRIB5, SRIB4, SRIB3, SRIB2, SRIB1, SRIB0, \
SOURCEA, OPPRE, CLK3, CLK2, CLK1, CLK0, CE3, CE2, CE1, CE0, RST3, RST2, RST1, RST0, \
output SROA8, SROA7, SROA6, SROA5, SROA4, SROA3, SROA2, SROA1, SROA0, \
SROB8, SROB7, SROB6, SROB5, SROB4, SROB3, SROB2, SROB1, SROB0, \
PO8, PO7, PO6, PO5, PO4, PO3, PO2, PO1, PO0);
endmodule
module ALU24B(input CE3, CE2, CE1, CE0, CLK3, CLK2, CLK1, CLK0, \
RST3, RST2, RST1, RST0, SIGNEDIA, SIGNEDIB, \
MA17, MA16, MA15, MA14, MA13, MA12, MA11, MA10, MA9, MA8, MA7, MA6, MA5, MA4, MA3, MA2, MA1, MA0, \
MB17, MB16, MB15, MB14, MB13, MB12, MB11, MB10, MB9, MB8, MB7, MB6, MB5, MB4, MB3, MB2, MB1, MB0, \
CIN23, CIN22, CIN21, CIN20, CIN19, CIN18, CIN17, CIN16, CIN15, CIN14, CIN13, CIN12, \
CIN11, CIN10, CIN9, CIN8, CIN7, CIN6, CIN5, CIN4, CIN3, CIN2, CIN1, CIN0, \
CFB23, CFB22, CFB21, CFB20, CFB19, CFB18, CFB17, CFB16, CFB15, CFB14, CFB13, CFB12, \
CFB11, CFB10, CFB9, CFB8, CFB7, CFB6, CFB5, CFB4, CFB3, CFB2, CFB1, CFB0, \
OPADDNSUB, OPCINSEL, \
output CO23, CO22, CO21, CO20, CO19, CO18, CO17, CO16, CO15, CO14, CO13, CO12, \
CO11, CO10, CO9, CO8, CO7, CO6, CO5, CO4, CO3, CO2, CO1, CO0, \
R23, R22, R21, R20, R19, R18, R17, R16, R15, R14, R13, R12, \
R11, R10, R9, R8, R7, R6, R5, R4, R3, R2, R1, R0);
endmodule
";

// Split-mode sysDSP primitives, from the local model. Their fuse bits are
// only partly known: the Trellis database maps MULT18/ALU54 settings, the
// vendor device database names the fuse groups for MULT9/PRADD9/ALU24
// (see data/dsp_fuses.json); the physical bit mapping is confirmed by the
// dsp_probe silicon fuzz.
pub const TYPES: &[&str] = &["MULT18X18D", "ALU54B", "MULT9X9D", "PRADD9A", "ALU24B"];

pub fn is_dsp(ty: &str) -> bool {
    TYPES.contains(&ty)
}

/// Inputs nextpnr feeds from a routed constant (`pack_constants`): their
/// CIB inputs are CE, LSR or CLK muxes, which have no low constant.
pub const ROUTED: [&str; 13] = ["CLK", "CE", "RST", "SRO", "SRI", "RO", "MA", "MB", "CFB", "CIN", "SOURCE", "SIGNED", "OP"];

/// Whether a constant on `pin` of a DSP is a routed constant net.
pub fn routed_const(pin: &str) -> bool {
    ROUTED.iter().any(|s| pin.starts_with(s))
}

/// ALU54B inputs that only the block's own wires reach (the MULT18X18D
/// products, register outputs and signs, the cascade): a constant on one
/// has no route, so it is left open.
pub fn dedicated(ty: &str, pin: &str) -> bool {
    let bus = |p: &str| pin.strip_prefix(p).is_some_and(|n| !n.is_empty() && n.bytes().all(|b| b.is_ascii_digit()));
    (ty == "ALU54B" || ty == "ALU24B")
        && (["MA", "MB", "A", "B", "CFB", "CIN"].iter().any(|p| bus(p))
            || matches!(pin, "SIGNEDIA" | "SIGNEDIB" | "SIGNEDCIN"))
}

/// Clock inputs.
pub fn is_clock_input(ty: &str, pin: &str) -> bool {
    is_dsp(ty) && matches!(pin, "CLK0" | "CLK1" | "CLK2" | "CLK3")
}

/// The bel index (`MULT18_<z>`, `ALU54_<z>`) and the block's first column.
fn block(bels: &Bels, b: BelId) -> (i16, i16, i16) {
    let t = bels.template(b);
    let z: i16 = t
        .name
        .rsplit('_')
        .next()
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    let bel = bels.bels[b as usize];
    (z, bel.x - z, bel.y)
}

/// Bels for every DSP primitive.
pub fn assign(bels: &Bels, nl: &Netlist) -> Result<HashMap<usize, BelId>> {
    let drivers = nl.drivers();
    let all = |ty: &str| -> Vec<BelId> {
        (0..bels.bels.len())
            .map(|k| BelId::try_from(k).expect("fits"))
            .filter(|&b| bels.template(b).ty == ty)
            .collect()
    };
    let mults = all("MULT18X18D");
    let alus = all("ALU54B");
    // Split-mode bels, from the local model. A split-mode bel is
    // site-exclusive with the 18-mode bel of its z: the fuses that select
    // the mode are shared, so only one of the pair can be configured.
    let mults9 = all("MULT9X9D");
    let pradds9 = all("PRADD9A");
    let alus24 = all("ALU24B");
    let site_used = |used: &HashSet<BelId>, bels: &Bels, b: BelId| -> bool {
        let (z, x0, y) = block(bels, b);
        used.iter().any(|&u| {
            let (uz, ux, uy) = block(bels, u);
            uz == z && ux == x0 && uy == y && u != b
        })
    };
    let mut out: HashMap<usize, BelId> = HashMap::new();
    let mut used: HashSet<BelId> = HashSet::new();
    // The MULT18X18D driving an ALU54B input bus (by its bit 0).
    let feeder = |p: &Prim, bus: &str| -> Option<usize> {
        let Some(PrimPin::In(Some(n))) = p.pin(&format!("{bus}0")) else { return None };
        let d = drivers[n as usize]? as usize;
        matches!(&nl.cells[d], Cell::Prim(q) if q.ty == "MULT18X18D").then_some(d)
    };
    // ALU54Bs first, with their multipliers.
    for (i, c) in nl.cells.iter().enumerate() {
        let Cell::Prim(p) = c else { continue };
        if p.ty != "ALU54B" {
            continue;
        }
        let ma = feeder(p, "MA");
        let mb = feeder(p, "MB");
        let b = alus
            .iter()
            .copied()
            .find(|&b| {
                if used.contains(&b) {
                    return false;
                }
                let (z, x0, y) = block(bels, b);
                let mult_free = |mz: i16| {
                    mults.iter().any(|&m| {
                        let (z2, x2, y2) = block(bels, m);
                        z2 == mz && x2 == x0 && y2 == y && !used.contains(&m)
                    })
                };
                mult_free(z - 3) && mult_free(z - 2)
            })
            .ok_or_else(|| err!("ALU54B {}: no free ALU54 with its two MULT18s", p.name))?;
        used.insert(b);
        out.insert(i, b);
        let (z, x0, y) = block(bels, b);
        for (m, mz) in [(ma, z - 3), (mb, z - 2)] {
            let Some(m) = m else { continue };
            if out.contains_key(&m) {
                return Err(err!("ALU54B {}: a MULT18X18D feeds two ALU54Bs", p.name));
            }
            let mb = mults
                .iter()
                .copied()
                .find(|&mm| block(bels, mm) == (mz, x0, y))
                .ok_or_else(|| err!("no MULT18_{mz} beside ALU54 {}", bels.template(b).name))?;
            used.insert(mb);
            out.insert(m, mb);
        }
    }
    for (i, c) in nl.cells.iter().enumerate() {
        let Cell::Prim(p) = c else { continue };
        if p.ty != "MULT18X18D" || out.contains_key(&i) {
            continue;
        }
        // Prefer a fresh block so the product exits never share wires
        // with another block's use, from the local model.
        let block_used = |used: &HashSet<BelId>, bels: &Bels, b: BelId| -> bool {
            // The block anchor is (x - z, y); z is the site within the
            // block, from the local model.
            let (_, x0, y) = block(bels, b);
            used.iter().any(|&u| {
                let (_, ux, uy) = block(bels, u);
                ux == x0 && uy == y
            })
        };
        let b = mults
            .iter()
            .copied()
            .find(|&b| !used.contains(&b) && !block_used(&used, bels, b))
            .or_else(|| mults.iter().copied().find(|b| !used.contains(b)))
            .ok_or_else(|| err!("MULT18X18D {}: no free MULT18", p.name))?;
        used.insert(b);
        out.insert(i, b);
    }
    // Split modes, from the local model: any free site (no chaining yet;
    // a PRADD driving a MULT arrives with the packing work).
    // A MULT9X9D that feeds an ALU24B's MA/MB is placed by the ALU24B's
    // own pass (pinned at the pair site), so it is skipped here.
    let feeds_alu24 = |nl: &Netlist, i: usize| -> bool {
        nl.cells.iter().any(|c| {
            let Cell::Prim(q) = c else { return false };
            if q.ty != "ALU24B" { return false }
            ["MA", "MB"].iter().any(|bus| {
                matches!(q.pin(&format!("{bus}0")), Some(PrimPin::In(Some(n)))
                    if drivers[n as usize] == Some(i as u32))
            })
        })
    };
    for (i, c) in nl.cells.iter().enumerate() {
        let Cell::Prim(p) = c else { continue };
        if p.ty == "MULT9X9D" && feeds_alu24(nl, i) {
            continue;
        }
        let pool: Vec<BelId> = match p.ty.as_str() {
            "MULT9X9D" => mults9.clone(),
            "PRADD9A" => pradds9.clone(),
            // From the local model: the split fuse tables below are those
            // of the low half of a block (sites 0..3), so an ALU24B and its
            // pair go there.
            "ALU24B" => alus24.iter().copied().filter(|&b| block(bels, b).0 < 4).collect(),
            _ => Vec::new(),
        };
        if pool.is_empty() {
            continue;
        }
        if out.contains_key(&i) {
            continue;
        }
        // Prefer a fresh block (none of its sites used): a split bel's
        // output exit shares wires with the 18-mode bels' products in the
        // same block, from the local model. Fall back to any free site.
        let block_used = |used: &HashSet<BelId>, bels: &Bels, b: BelId| -> bool {
            // The block anchor is (x - z, y): z is the site within the
            // block, from the local model.
            let (_, x0, y) = block(bels, b);
            used.iter().any(|&u| {
                let (_, ux, uy) = block(bels, u);
                ux == x0 && uy == y
            })
        };
        let free = |b: &BelId| !used.contains(b) && !site_used(&used, bels, *b);
        // From the local model: an ALU24B's MA/MB are dedicated wires from
        // the paired multiplier sites (z-3, z-2) of its block, so the
        // pair must be placeable there; netlist MULT9X9Ds are grabbed as
        // companions (products 0 with tied inputs, the ALU sees MA+MB=0).
        let pair_free = |b: &BelId| -> bool {
            if p.ty != "ALU24B" {
                return true;
            }
            let (z, x0, y) = block(bels, *b);
            [z - 3, z - 2].iter().all(|&mz| {
                mults9.iter().any(|&m| {
                    let (z2, x2, y2) = block(bels, m);
                    z2 == mz && x2 == x0 && y2 == y && !used.contains(&m)
                })
            })
        };
        let b = pool
            .iter()
            .copied()
            .find(|b| free(b) && !block_used(&used, bels, *b) && pair_free(b))
            .or_else(|| pool.iter().copied().find(|b| free(b) && pair_free(b)))
            .or_else(|| pool.iter().copied().find(|b| free(b)))
            .ok_or_else(|| err!("{} {}: no free {} site", p.ty, p.name, p.ty))?;
        if p.ty == "ALU24B" {
            // From the local model: the ALU24B's MA/MB are dedicated block
            // wires, so the driving MULT9X9Ds (feeder cells, below) are
            // pinned at the pair sites (z-3, z-2) of the ALU's block.
            let (z, x0, y) = block(bels, b);
            let feeder9 = |p: &Prim, bus: &str| -> Option<usize> {
                let Some(PrimPin::In(Some(n))) = p.pin(&format!("{bus}0")) else { return None };
                let d = drivers[n as usize]? as usize;
                matches!(&nl.cells[d], Cell::Prim(q) if q.ty == "MULT9X9D").then_some(d)
            };
            for (feeder, mz) in [(feeder9(p, "MA"), z - 3), (feeder9(p, "MB"), z - 2)] {
                if let Some(f) = feeder {
                    if out.contains_key(&f) {
                        return Err(err!("ALU24B {}: a MULT9X9D feeds two ALU24Bs", p.name));
                    }
                    let mb = mults9
                        .iter()
                        .copied()
                        .find(|&mm| block(bels, mm) == (mz, x0, y) && !used.contains(&mm))
                        .ok_or_else(|| err!("no MULT9 at z={mz} beside ALU24B {}", p.name))?;
                    used.insert(mb);
                    out.insert(f, mb);
                }
            }
        }
        used.insert(b);
        out.insert(i, b);
    }
    Ok(out)
}

/// A 56-bit pattern parameter (`0x...`), LSB first.
fn pattern(p: &Prim, name: &str) -> Result<Vec<bool>> {
    let s = p.param_str(name).unwrap_or_else(|| "0x00000000000000".to_owned());
    let hex = s.trim_start_matches("0x").trim_start_matches("0X");
    let v = u64::from_str_radix(if hex.is_empty() { "0" } else { hex }, 16)
        .map_err(|_| err!("{} {}: {name} {s} is not hex", p.ty, p.name))?;
    Ok((0..56).map(|i| (v >> i) & 1 == 1).collect())
}

/// Write every DSP primitive's settings and ties.
// Ties for an ALU24B-cluster cell. From the local model: the cluster's
// whole-tile image sets every input mux the block needs, so unconnected
// inputs take no tie; only netlist-tied pins (Some("1")/Some("0")) tie.
// Sign pins on the cluster's MULT9X9Ds are inverted under the image: pin
// low is signed, pin high unsigned, so a netlist-tied constant is flipped.
fn tie_cluster(
    g: &Graph,
    db: &DeviceDb,
    bels: &Bels,
    nl: &Netlist,
    i: usize,
    bel: BelId,
    tiles: &mut BTreeMap<String, TileConfig>,
) -> Result<()> {
    let Cell::Prim(p) = &nl.cells[i] else { return Ok(()) };
    let sign_pin = |p: &Prim, pin: &str| p.ty == "MULT9X9D" && (pin == "SIGNEDA" || pin == "SIGNEDB");
    for (pin, w) in bels
        .pins(bel)
        .filter(|(pp, _)| pp.dir == crate::bels::PinDir::In)
        .map(|(pp, w)| (pp.name.clone(), w))
        .collect::<Vec<_>>()
    {
        if matches!(p.pin(&pin), Some(PrimPin::In(Some(_)))) {
            continue;
        }
        let level = match p.attrs.get(&crate::iol::tie_attr(&pin)).map(String::as_str) {
            Some("1") => Level::High,
            Some("0") => Level::Low,
            _ => continue,
        };
        // Invert a tied constant sign pin: the cluster image inverts the
        // pin's sense (low = signed), from the local model.
        let level = if sign_pin(p, &pin)
            && matches!(p.pin(&pin), None)
        {
            match level {
                Level::High => Level::Low,
                Level::Low => Level::High,
            }
        } else {
            level
        };
        crate::io::tie(g, db, w, level, tiles)?;
    }
    Ok(())
}

pub fn configure(
    g: &Graph,
    db: &DeviceDb,
    bels: &Bels,
    nl: &Netlist,
    at: &HashMap<usize, BelId>,
    tiles: &mut BTreeMap<String, TileConfig>,
) -> Result<()> {
    let mut cells: Vec<usize> = at.keys().copied().collect();
    cells.sort_unstable();
    // From the local model: an ALU24B and the MULT9X9Ds at its pair sites
    // (z-3, z-2) are configured as one unit, by one bit set.
    let mut cluster: HashSet<usize> = HashSet::new();
    for (&i, &b) in at {
        let Cell::Prim(p) = &nl.cells[i] else { continue };
        if p.ty != "ALU24B" { continue }
        let (z, x0, y) = block(bels, b);
        cluster.insert(i);
        for (&j, &m) in at {
            let (mz, mx, my) = block(bels, m);
            if mx == x0 && my == y && (mz == z - 3 || mz == z - 2) {
                cluster.insert(j);
            }
        }
    }
    for i in cells {
        let Cell::Prim(p) = &nl.cells[i] else { continue };
        if !is_dsp(&p.ty) {
            continue;
        }
        let bel = at[&i];
        let (z, x0, y) = block(bels, bel);
        // From the local model: the ALU24B cluster (the ALU plus the two
        // MULT9X9Ds at its pair sites) is one whole-tile image; nothing
        // else is emitted into its tiles from these cells.
        if cluster.contains(&i) {
            if p.ty == "ALU24B" {
                // The four sign-pin inverters stay clear, so SIGNEDA / SIGNEDB
                // read true (low is unsigned), from the silicon fuzz:
                // MIB_DSP1 F104B0 and F105B0 (site 0 SIGNEDA and site 1 SIGNEDB,
                // not told apart), MIB_DSP2 F19B0 (site 0 SIGNEDB) and F6B0
                // (site 1 SIGNEDA).
                const ALU24_CLUSTER: &[(&str, &[(usize, usize)])] = &[
                    ("MIB_DSP0", &[(61, 0), (62, 0), (64, 0), (67, 0), (68, 0)]),
                    ("MIB2_DSP0", &[(4, 1), (5, 1), (15, 1), (16, 1), (23, 1), (24, 1), (28, 1), (30, 1),
                        (31, 1), (40, 1), (41, 1), (42, 1), (43, 1), (44, 1), (45, 1), (46, 1), (47, 1),
                        (48, 1), (60, 1), (61, 1), (75, 1), (76, 1), (79, 1), (83, 1), (84, 1), (86, 1),
                        (88, 1), (92, 1), (93, 1)]),
                    ("MIB_DSP1", &[(51, 0), (61, 0), (103, 0)]),
                    ("MIB2_DSP1", &[(14, 1), (28, 1), (31, 1), (32, 1), (33, 1), (56, 1), (57, 1), (58, 1),
                        (59, 1), (72, 0), (90, 0), (101, 0)]),
                    ("MIB_DSP2", &[(0, 0), (7, 0), (16, 0), (20, 0), (21, 0), (22, 0),
                        (26, 0), (27, 0), (28, 0), (33, 0), (34, 0), (35, 0), (36, 0), (42, 0), (54, 0),
                        (55, 0), (56, 0), (58, 0), (59, 0), (60, 0), (61, 0), (62, 0), (63, 0), (64, 0)]),
                    ("MIB2_DSP2", &[(0, 1), (1, 0), (1, 1), (2, 1), (3, 1), (14, 0), (24, 1), (25, 1),
                        (26, 1), (27, 1), (46, 1), (47, 1), (56, 1), (57, 1), (60, 1), (61, 1), (62, 1),
                        (63, 1), (68, 1), (69, 1), (70, 1), (71, 1), (82, 0), (83, 0), (84, 1), (85, 1),
                        (86, 0), (92, 1), (93, 1), (94, 1), (95, 1), (104, 1), (105, 1)]),
                    ("MIB2_DSP3", &[(10, 1), (11, 1), (22, 1), (23, 1), (28, 1), (30, 0), (34, 1), (76, 1),
                        (78, 1), (90, 1), (91, 1), (92, 1), (93, 1)]),
                    ("MIB_DSP4", &[(34, 0), (35, 0), (41, 0), (46, 0), (47, 0), (48, 0), (49, 0)]),
                    ("MIB2_DSP4", &[(0, 1), (2, 1), (3, 1), (4, 1), (6, 1), (7, 1), (8, 1), (9, 1), (16, 1),
                        (17, 1), (18, 1), (19, 1), (20, 1), (21, 1), (26, 1), (27, 1), (30, 1), (31, 1),
                        (34, 1), (35, 1)]),
                ];
                let block_tiles: Vec<&crate::graph::TileLoc> = g
                    .tiles
                    .iter()
                    .filter(|t| t.y == y && (x0..=x0 + 8).contains(&t.x))
                    .collect();
                for t in &block_tiles {
                    let Some((_, suffix)) = t.name.rsplit_once(':') else { continue };
                    let Some((_, bits)) = ALU24_CLUSTER.iter().find(|(s, _)| *s == suffix) else { continue };
                    tiles
                        .entry(t.name.clone())
                        .or_default()
                        .unknowns
                        .extend(bits.iter().copied());
                }
                // Ties still apply (see below); everything else is skipped.
                tie_cluster(g, db, bels, nl, i, bel, tiles)?;
            }
            continue;
        }
        let s = |k: &str, d: &str| p.param_str(k).unwrap_or_else(|| d.to_owned());
        let mut enums: Vec<(String, String)> = Vec::new();
        let mut words: Vec<(String, Vec<bool>)> = Vec::new();
        // Split modes, from the local model: the register-style settings
        // mirror the 18-mode sets (REG_INPUTA/B/C, REG_OPPRE for the
        // pre-adder, REG_OPCODE_x for the ALU split); only the values the
        // Trellis database knows are emitted, the rest wait on the silicon
        // fuzz for their bits.
        let (pre, regs): (String, &[(&str, &str)]) = if p.ty == "MULT9X9D" {
            // The site's settings live under the paired multiplier key the
            // device database maps (one fuse pair serves both sizes), from
            // the local model.
            (
                format!("MULT18_{z}"),
                &[
                    ("REG_INPUTA_CLK", "NONE"),
                    ("REG_INPUTA_CE", "CE0"),
                    ("REG_INPUTA_RST", "RST0"),
                    ("REG_INPUTB_CLK", "NONE"),
                    ("REG_INPUTB_CE", "CE0"),
                    ("REG_INPUTB_RST", "RST0"),
                    ("REG_INPUTC_CLK", "NONE"),
                    ("REG_INPUTC_CE", "CE0"),
                    ("REG_INPUTC_RST", "RST0"),
                    ("REG_PIPELINE_CLK", "NONE"),
                    ("REG_PIPELINE_CE", "CE0"),
                    ("REG_PIPELINE_RST", "RST0"),
                    ("REG_OUTPUT_CLK", "NONE"),
                    ("CLK0_DIV", "ENABLED"),
                    ("CLK1_DIV", "ENABLED"),
                    ("CLK2_DIV", "ENABLED"),
                    ("CLK3_DIV", "ENABLED"),
                    ("GSR", "ENABLED"),
                    ("RESETMODE", "SYNC"),
                ],
            )
        } else if p.ty == "PRADD9A" {
            (
                // The pre-adder's settings ride the shared multiplier key
                // the device database maps, from the local model.
                format!("MULT18_{z}"),
                &[
                    ("REG_INPUTA_CLK", "NONE"),
                    ("REG_INPUTA_CE", "CE0"),
                    ("REG_INPUTA_RST", "RST0"),
                    ("REG_INPUTB_CLK", "NONE"),
                    ("REG_INPUTB_CE", "CE0"),
                    ("REG_INPUTB_RST", "RST0"),
                    ("REG_INPUTC_CLK", "NONE"),
                    ("REG_INPUTC_CE", "CE0"),
                    ("REG_INPUTC_RST", "RST0"),
                    ("REG_OPPRE_CLK", "NONE"),
                    ("REG_OPPRE_CE", "CE0"),
                    ("REG_OPPRE_RST", "RST0"),
                    ("CLK0_DIV", "ENABLED"),
                    ("CLK1_DIV", "ENABLED"),
                    ("CLK2_DIV", "ENABLED"),
                    ("CLK3_DIV", "ENABLED"),
                    ("GSR", "ENABLED"),
                    ("RESETMODE", "SYNC"),
                    ("SOURCEA_MODE", "A_SHIFT"),
                    ("SOURCEB_MODE", "B_SHIFT"),
                ],
            )
        } else if p.ty == "ALU24B" {
            // The 24-bit split's settings ride the shared ALU key the
            // device database maps, from the local model.
            // The 24-bit split rides the shared ALU keys, from the local
            // model: the same key set the 54-bit mode uses, on the paired
            // site.
            (
                format!("ALU54_{z}"),
                &[
                    ("MODE", "ALU54B"),
                    ("REG_INPUTC0_CLK", "NONE"),
                    ("REG_INPUTC1_CLK", "NONE"),
                    ("REG_OPCODEOP0_0_CLK", "NONE"),
                    ("REG_OPCODEOP0_0_CE", "CE0"),
                    ("REG_OPCODEOP0_0_RST", "RST0"),
                    ("REG_OPCODEOP0_1_CLK", "NONE"),
                    ("REG_OPCODEOP0_1_CE", "CE0"),
                    ("REG_OPCODEOP0_1_RST", "RST0"),
                    ("REG_OPCODEOP1_0_CLK", "NONE"),
                    ("REG_OPCODEOP1_1_CLK", "NONE"),
                    ("REG_OPCODEIN_0_CLK", "NONE"),
                    ("REG_OPCODEIN_0_CE", "CE0"),
                    ("REG_OPCODEIN_0_RST", "RST0"),
                    ("REG_OPCODEIN_1_CLK", "NONE"),
                    ("REG_OPCODEIN_1_CE", "CE0"),
                    ("REG_OPCODEIN_1_RST", "RST0"),
                    ("REG_OUTPUT0_CLK", "NONE"),
                    ("REG_OUTPUT0_RST", "RST0"),
                    ("REG_OUTPUT1_CLK", "NONE"),
                    ("REG_OUTPUT1_RST", "RST0"),
                    ("REG_FLAG_CLK", "NONE"),
                    ("MCPAT_SOURCE", "STATIC"),
                    ("MASKPAT_SOURCE", "STATIC"),
                    ("CLK0_DIV", "ENABLED"),
                    ("CLK1_DIV", "ENABLED"),
                    ("CLK2_DIV", "ENABLED"),
                    ("CLK3_DIV", "ENABLED"),
                    ("GSR", "DISABLED"),
                    ("RESETMODE", "ASYNC"),
                    ("FORCE_ZERO_BARREL_SHIFT", "DISABLED"),
                    ("LEGACY", "DISABLED"),
                ],
            )
        } else if p.ty == "MULT18X18D" {
            (
                format!("MULT18_{z}"),
                &[
                    ("REG_INPUTA_CLK", "NONE"),
                    ("REG_INPUTA_CE", "CE0"),
                    ("REG_INPUTA_RST", "RST0"),
                    ("REG_INPUTB_CLK", "NONE"),
                    ("REG_INPUTB_CE", "CE0"),
                    ("REG_INPUTB_RST", "RST0"),
                    ("REG_INPUTC_CLK", "NONE"),
                    ("REG_PIPELINE_CLK", "NONE"),
                    ("REG_PIPELINE_CE", "CE0"),
                    ("REG_PIPELINE_RST", "RST0"),
                    ("REG_OUTPUT_CLK", "NONE"),
                    ("CLK0_DIV", "ENABLED"),
                    ("CLK1_DIV", "ENABLED"),
                    ("CLK2_DIV", "ENABLED"),
                    ("CLK3_DIV", "ENABLED"),
                    ("GSR", "ENABLED"),
                    ("SOURCEB_MODE", "B_SHIFT"),
                    ("RESETMODE", "SYNC"),
                ],
            )
        } else {
            (
                format!("ALU54_{z}"),
                &[
                    ("REG_INPUTC0_CLK", "NONE"),
                    ("REG_INPUTC1_CLK", "NONE"),
                    ("REG_OPCODEOP0_0_CLK", "NONE"),
                    ("REG_OPCODEOP0_0_CE", "CE0"),
                    ("REG_OPCODEOP0_0_RST", "RST0"),
                    ("REG_OPCODEOP1_0_CLK", "NONE"),
                    ("REG_OPCODEOP0_1_CLK", "NONE"),
                    ("REG_OPCODEOP1_1_CLK", "NONE"),
                    ("REG_OPCODEOP0_1_CE", "CE0"),
                    ("REG_OPCODEOP0_1_RST", "RST0"),
                    ("REG_OPCODEIN_0_CLK", "NONE"),
                    ("REG_OPCODEIN_0_CE", "CE0"),
                    ("REG_OPCODEIN_0_RST", "RST0"),
                    ("REG_OPCODEIN_1_CLK", "NONE"),
                    ("REG_OPCODEIN_1_CE", "CE0"),
                    ("REG_OPCODEIN_1_RST", "RST0"),
                    ("REG_OUTPUT0_CLK", "NONE"),
                    ("REG_OUTPUT1_CLK", "NONE"),
                    ("REG_FLAG_CLK", "NONE"),
                    ("MCPAT_SOURCE", "STATIC"),
                    ("MASKPAT_SOURCE", "STATIC"),
                    ("CLK0_DIV", "ENABLED"),
                    ("CLK1_DIV", "ENABLED"),
                    ("CLK2_DIV", "ENABLED"),
                    ("CLK3_DIV", "ENABLED"),
                    ("GSR", "ENABLED"),
                    ("RESETMODE", "SYNC"),
                    ("FORCE_ZERO_BARREL_SHIFT", "DISABLED"),
                    ("LEGACY", "DISABLED"),
                ],
            )
        };
        for &(k, d) in regs {
            enums.push((format!("{pre}.{k}"), s(k, d)));
        }
        if p.ty == "MULT18X18D" {
            if z == 0 || z == 4 {
                enums.push((format!("{pre}.REG_OUTPUT_RST"), s("REG_OUTPUT_RST", "RST0")));
            }
            enums.push((format!("{pre}.MODE"), "MULT18X18D".to_owned()));
            // Unregistered: the output register bypassed. The Trellis fuzzer
            // (076-dsp_cibo) set CIBOUT_BYP ON exactly when REG_OUTPUT_CLK
            // was NONE, so it is the register's bypass mux. nextpnr leaves it
            // off for a MULT feeding an ALU54B; on silicon that ALU then read
            // 0 for every opcode (MA/MB from an unclocked register).
            if s("REG_OUTPUT_CLK", "NONE") == "NONE" {
                enums.push((format!("{pre}.CIBOUT_BYP"), "ON".to_owned()));
            }
            for sig in ["CLK", "CE", "RST"] {
                for k in 0..4 {
                    enums.push((format!("{pre}.{sig}{k}MUX"), format!("{sig}{k}")));
                }
            }
        } else if p.ty == "MULT9X9D" {
            // From the local model: the 9x9 split's partner key keeps its
            // 18-mode OFF (MODE NONE, clearing the 18x18 mode fuse); the
            // split selector itself rides the unnamed fuse the device
            // database maps (one fuse flipping the paired sites together).
            // From the local model: the partner keeps full 18-mode enums;
            // the split itself rides the unnamed MC1_M9_S1 fuse bits.
            enums.push((format!("{pre}.MODE"), "MULT18X18D".to_owned()));
            for sig in ["CLK", "CE", "RST"] {
                for k in 0..4 {
                    enums.push((format!("{pre}.{sig}{k}MUX"), format!("{sig}{k}")));
                }
            }
        } else if p.ty == "PRADD9A" {
            // PRADD's MODE rides no Trellis key (split selector unknown,
            // found by the silicon fuzz).
            for sig in ["CLK", "CE", "RST"] {
                for k in 0..4 {
                    enums.push((format!("{pre}.{sig}{k}MUX"), format!("{sig}{k}")));
                }
            }
        } else if p.ty == "ALU24B" {
            // ALU24's MODE rides no Trellis key (split selector unknown,
            // found by the silicon fuzz).
        } else {
            for w in ["MASK01", "MCPAT", "MASKPAT", "RNDPAT"] {
                words.push((format!("{pre}.{w}"), pattern(p, w)?));
            }
            enums.push((format!("{pre}.MODE"), "ALU54B".to_owned()));
        }
        if p.ty == "ALU24B" {
            // From the local model: the split fuse sets the ALU24 site's
            // mode and its ALU54 partner's mode together; the ALU24 keys
            // are Trellis-known, the ALU54 partner key carries the
            // register settings emitted above.
            enums.push((format!("ALU24_{}.MODE", z - 1), "ALU24B".to_owned()));
        }
        if p.ty == "MULT9X9D" {
            // From the local model (076-dsp_cibo silicon lesson): an
            // unregistered product reads 0 unless the output register is
            // bypassed; the split's product exits through both sites.
            if s("REG_OUTPUT_CLK", "NONE") == "NONE" {
                enums.push((format!("{pre}.CIBOUT_BYP"), "ON".to_owned()));
                enums.push((format!("MULT18_{}.CIBOUT_BYP", z ^ 1), "ON".to_owned()));
            }
            // From the local model: the split fuse flips both sites of the
            // pair (MULT18_{z} and MULT18_{z^1}) together; the partner
            // carries the same 18-mode settings.
            let pre2 = format!("MULT18_{}", z ^ 1);
            for &(k, d) in regs {
                enums.push((format!("{pre2}.{k}"), s(k, d)));
            }
            enums.push((format!("{pre2}.MODE"), "MULT18X18D".to_owned()));
            for sig in ["CLK", "CE", "RST"] {
                for k in 0..4 {
                    enums.push((format!("{pre2}.{sig}{k}MUX"), format!("{sig}{k}")));
                }
            }
        }
        let side = if z < 4 { "DSP_LEFT" } else { "DSP_RIGHT" };
        enums.push((format!("{side}.CIBOUT"), "ON".to_owned()));
        // Into every tile of the block that has the setting.
        let block_tiles: Vec<&crate::graph::TileLoc> = g
            .tiles
            .iter()
            .filter(|t| t.y == y && (x0..=x0 + 8).contains(&t.x))
            .collect();
        for (k, v) in &enums {
            let mut n = 0;
            for t in &block_tiles {
                let Some(e) = db.tile_bits(&t.ty)?.enums.get(k) else { continue };
                if !e.options.contains_key(v.as_str()) {
                    return Err(err!("{} {}: {k} has no option {v}", p.ty, p.name));
                }
                tiles.entry(t.name.clone()).or_default().enums.push((k.clone(), v.clone()));
                n += 1;
            }
            // Split modes (MULT9X9D / PRADD9A / ALU24B, from the local
            // model) tolerate unresolved keys: their fuse tables are only
            // partly in the Trellis database, so a setting the placed tile
            // does not know is skipped here and its physical bit is found by
            // the silicon fuzz. The paired 18x18/ALU54 modes stay strict.
            if n == 0 {
                if matches!(p.ty.as_str(), "MULT9X9D" | "PRADD9A" | "ALU24B") {
                    continue;
                }
                return Err(err!("{} {}: no tile of its block has {k}", p.ty, p.name));
            }
        }
        // Split-mode selector fuses (from the local model): the physical
        // bits of the MC1_M9_S1 (9x9 split) and MC1_OPR_S1_0 (ALU24
        // split) fuse groups. They ride no Trellis key; a MULT9X9D or
        // ALU24B placed in the block flips its paired sites together, so
        // the bits go into every block tile whose suffix matches.
        if matches!(p.ty.as_str(), "MULT9X9D" | "ALU24B") {
            // (tile suffix, [(frame, bit) ...]) of the selector.
            let fuses: &[(&str, &[(usize, usize)])] = if p.ty == "MULT9X9D" {
                &[
                    ("MIB_DSP0", &[(67, 0), (68, 0)]),
                    ("MIB2_DSP0", &[(27, 1), (79, 1), (83, 1)]),
                    ("MIB2_DSP1", &[(64, 1), (65, 1), (78, 1), (79, 1)]),
                    ("MIB2_DSP2", &[(60, 1), (61, 1), (70, 1), (71, 1)]),
                    ("MIB_DSP2", &[(0, 0)]),
                    (
                        "MIB2_DSP4",
                        &[
                            (17, 1), (18, 1), (26, 1), (27, 1), (30, 1), (31, 1),
                            (34, 1), (35, 1), (36, 1), (37, 1), (40, 1), (41, 1),
                            (42, 1), (43, 1),
                        ],
                    ),
                ]
            } else {
                &[
                    ("MIB_DSP2", &[(10, 0), (12, 0), (25, 0), (39, 0), (48, 0), (54, 0)]),
                    ("MIB2_DSP3", &[(77, 1)]),
                    ("MIB_DSP4", &[(35, 0)]),
                ]
            };
            for t in &block_tiles {
                let Some((_, suffix)) = t.name.rsplit_once(':') else { continue };
                let Some((_, bits)) = fuses.iter().find(|(s, _)| *s == suffix) else { continue };
                tiles
                    .entry(t.name.clone())
                    .or_default()
                    .unknowns
                    .extend(bits.iter().copied());
            }
        }
        for (k, v) in &words {
            for t in &block_tiles {
                if db.tile_bits(&t.ty)?.words.contains_key(k) {
                    tiles.entry(t.name.clone()).or_default().words.push((k.clone(), v.clone()));
                }
            }
        }
        // Ties: inputs synthesis tied, and unconnected data inputs low.
        for (pin, w) in bels.pins(bel).filter(|(p, _)| p.dir == crate::bels::PinDir::In).map(|(pp, w)| (pp.name.clone(), w)).collect::<Vec<_>>() {
            if matches!(p.pin(&pin), Some(PrimPin::In(Some(_)))) {
                continue;
            }
            let level = match p.attrs.get(&crate::iol::tie_attr(&pin)).map(String::as_str) {
                Some("1") => Level::High,
                Some("0") => Level::Low,
                _ => {
                    if routed_const(&pin) || dedicated(&p.ty, &pin) {
                        continue;
                    }
                    Level::Low
                }
            };
            crate::io::tie(g, db, w, level, tiles)?;
        }
    }
    Ok(())
}

/// Multiplier inference, after yosys mul2dsp for ECP5 (DSP_A/B_MAXWIDTH 18,
/// MINWIDTH 2). The operands are first trimmed of their context extension
/// (zeros, or copies of the sign bit), as yosys wreduce does.
///
/// Operands of 2 to 18 bits with a result of at most 36 bits: one
/// MULT18X18D with no registers, operands extended to 18 bits (SIGNEDA /
/// SIGNEDB follow the cell), P the exact product, the result its low bits.
///
/// Wider operands: each is cut into 18-bit chunks from the bottom. The
/// chunks are unsigned except the top chunk of a signed operand, so each
/// chunk product a_i * b_j is one MULT18X18D with SIGNEDA / SIGNEDB set for
/// a top signed chunk, exact in 36 bits. The products are extended to the
/// result width (sign-extended when either chunk is signed), shifted by
/// 18 (i + j) and added. Products shifted past the result are not made; at
/// most `MAX_SPLIT` MULTs per multiply, else the multiply stays logic.
///
/// A multiply of two constants stays logic. Returns the MULTs made.
pub fn infer_mul(m: &mut oxibit_hdl::netlist::Module) -> usize {
    use oxibit_hdl::netlist::{Bit, Cell as HCell, Op, SigSpec};
    const MAX_SPLIT: usize = 16;
    let is_const = |s: &SigSpec| s.0.iter().all(|b| matches!(b, Bit::Zero | Bit::One));
    let trim = |s: &SigSpec, signed: bool| -> SigSpec {
        let mut v = s.0.clone();
        while v.len() > 1 {
            let top = v[v.len() - 1];
            let ext = if signed { top == v[v.len() - 2] } else { top == Bit::Zero };
            if !ext {
                break;
            }
            v.pop();
        }
        SigSpec(v)
    };
    let bit = |v: bool| SigSpec(vec![if v { Bit::One } else { Bit::Zero }]);
    // One MULT18X18D: 18-bit operands, the signs, P bits to `p` (<= 36).
    let mult = |name: String, a18: &SigSpec, b18: &SigSpec, sa: bool, sb: bool, p: &SigSpec| {
        let mut conns = BTreeMap::new();
        for i in 0..18 {
            conns.insert(format!("A{i}"), a18.slice(i, 1));
            conns.insert(format!("B{i}"), b18.slice(i, 1));
        }
        conns.insert("SIGNEDA".to_owned(), bit(sa));
        conns.insert("SIGNEDB".to_owned(), bit(sb));
        conns.insert("SOURCEA".to_owned(), bit(false));
        conns.insert("SOURCEB".to_owned(), bit(false));
        for i in 0..p.width() {
            conns.insert(format!("P{i}"), p.slice(i, 1));
        }
        HCell::Instance { ty: "MULT18X18D".to_owned(), name, params: BTreeMap::new(), conns, attrs: BTreeMap::new() }
    };

    let mut n = 0;
    let mut split: Vec<(usize, SigSpec, SigSpec, bool, SigSpec)> = Vec::new();
    for (ci, c) in m.cells.iter_mut().enumerate() {
        let HCell::Binary { op: Op::Mul, a, b, signed, y } = c else { continue };
        let (a, b) = (trim(a, *signed), trim(b, *signed));
        let (wa, wb, wy) = (a.width(), b.width(), y.width());
        if wa < 2 || wb < 2 || wy == 0 || (is_const(&a) && is_const(&b)) {
            continue;
        }
        if wa <= 18 && wb <= 18 && wy <= 36 {
            *c = mult(format!("$mul{n}"), &a.extend(18, *signed), &b.extend(18, *signed), *signed, *signed, y);
            n += 1;
            continue;
        }
        let chunks = |w: u32| w.div_ceil(18) as usize;
        let made = (0..chunks(wa))
            .flat_map(|i| (0..chunks(wb)).map(move |j| (i, j)))
            .filter(|&(i, j)| 18 * (i + j) < wy as usize)
            .count();
        if made <= MAX_SPLIT {
            split.push((ci, a, b, *signed, y.clone()));
        }
    }
    // The split multiplies: each cell becomes its chunk MULTs and adders.
    let mut out: Vec<(usize, Vec<HCell>)> = Vec::new();
    for (ci, a, b, signed, y) in split {
        let wy = y.width();
        let cut = |s: &SigSpec| -> Vec<(SigSpec, bool)> {
            let w = s.width();
            let k = w.div_ceil(18);
            (0..k)
                .map(|i| {
                    let lo = 18 * i;
                    let top = i == k - 1;
                    let sg = signed && top;
                    (s.slice(lo, (w - lo).min(18)).extend(18, sg), sg)
                })
                .collect()
        };
        let (ac, bc) = (cut(&a), cut(&b));
        let mut cells = Vec::new();
        let mut terms: Vec<SigSpec> = Vec::new();
        for (i, (ai, sa)) in ac.iter().enumerate() {
            for (j, (bj, sb)) in bc.iter().enumerate() {
                let sh = 18 * (i + j) as u32;
                if sh >= wy {
                    continue;
                }
                let pw = 36.min(wy - sh);
                let p = m.wire(pw);
                cells.push(mult(format!("$mul{n}"), ai, bj, *sa, *sb, &p));
                n += 1;
                // p << sh, extended to wy.
                let ext = p.extend(wy - sh, *sa || *sb);
                terms.push(SigSpec::constant(0, sh).concat(&ext));
            }
        }
        // Sum the terms; the last add drives y.
        let mut acc = terms[0].clone();
        for (k, t) in terms.iter().enumerate().skip(1) {
            let s = if k == terms.len() - 1 { y.clone() } else { m.wire(wy) };
            cells.push(HCell::Binary { op: Op::Add, a: acc, b: t.clone(), signed: false, y: s.clone() });
            acc = s;
        }
        if terms.len() == 1 {
            m.connects.push((y.clone(), acc));
        }
        out.push((ci, cells));
    }
    // Replace from the back so indices stay valid.
    for (ci, cells) in out.into_iter().rev() {
        m.cells.splice(ci..=ci, cells);
    }
    n
}
