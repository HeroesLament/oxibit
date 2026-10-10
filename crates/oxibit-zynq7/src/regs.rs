//! Register map. Offsets are from `0xF800_0000`, the base of the one mapping
//! that covers both the SLCR (+0) and DevC (+0x7000).
//!
//! DevC offsets, bits and the load sequence are transcribed from mainline
//! Linux `drivers/fpga/zynq-fpga.c` by way of nervezynq's `devcfg.rs`;
//! SLCR clock and ID offsets from UG585.

pub const SLCR_BASE: u64 = 0xF800_0000;
pub const MAP_LEN: usize = 0x8000;

// --- SLCR ------------------------------------------------------------------
pub const SLCR_LOCK: usize = 0x004;
pub const SLCR_UNLOCK: usize = 0x008;
/// Bit 0 set = locked.
pub const SLCR_LOCKSTA: usize = 0x00C;
pub const SLCR_LOCK_KEY: u32 = 0x0000_767B;
pub const SLCR_UNLOCK_KEY: u32 = 0xDF0D_767B;

pub const ARM_PLL_CTRL: usize = 0x100;
pub const DDR_PLL_CTRL: usize = 0x104;
pub const IO_PLL_CTRL: usize = 0x108;
/// 0x168 (mainline clkc.c: `clk_base + 0x68`). nervezynq's `devcfg.rs` used
/// 0x18C, which is FPGA1_THR_STA: its "enable the PCAP clock" write never
/// reached the PCAP clock, and worked only because ps7_init had enabled it.
pub const PCAP_CLK_CTRL: usize = 0x168;
/// FPGA0..3_CLK_CTRL. FPGA1 is 0x180, not 0x184 (that is FPGA0's THR_CNT row).
pub const FPGA_CLK_CTRL: [usize; 4] = [0x170, 0x180, 0x190, 0x1A0];
pub const FPGA_RST_CTRL: usize = 0x240;
pub const PSS_IDCODE: usize = 0x530;
pub const LVL_SHFTR_EN: usize = 0x900;

pub const LVL_SHFTR_DISABLE_ALL: u32 = 0x0;
pub const LVL_SHFTR_ENABLE_PS_TO_PL: u32 = 0xA;
pub const LVL_SHFTR_ENABLE_PL_TO_PS: u32 = 0xF;
pub const FPGA_RST_ALL: u32 = 0xF;
pub const FPGA_RST_NONE: u32 = 0x0;

// --- DevC ------------------------------------------------------------------
pub const DEVC: usize = 0x7000;
pub const CTRL: usize = DEVC; // + 0x00
pub const LOCK: usize = DEVC + 0x04;
pub const INT_STS: usize = DEVC + 0x0C;
pub const INT_MASK: usize = DEVC + 0x10;
pub const STATUS: usize = DEVC + 0x14;
pub const DMA_SRC_ADDR: usize = DEVC + 0x18;
pub const DMA_DST_ADDR: usize = DEVC + 0x1C;
pub const DMA_SRC_LEN: usize = DEVC + 0x20;
pub const DMA_DEST_LEN: usize = DEVC + 0x24;
pub const UNLOCK: usize = DEVC + 0x34;
pub const MCTRL: usize = DEVC + 0x80;

pub const CTRL_PCFG_PROG_B: u32 = 1 << 30;
pub const CTRL_PCAP_PR: u32 = 1 << 27;
pub const CTRL_PCAP_MODE: u32 = 1 << 26;
pub const MCTRL_PCAP_LPBK: u32 = 1 << 4;

pub const STATUS_DMA_Q_F: u32 = 1 << 31;
pub const STATUS_DMA_Q_E: u32 = 1 << 30;
pub const STATUS_PCFG_INIT: u32 = 1 << 4;
/// Level signals from the PL configuration logic (names from FreeBSD
/// `zy7_devcfg.c`). Both read 0 at boot and after PROG_B, and 1 once a
/// design has loaded and started (measured: STATUS 0x40000A30 blank,
/// 0x50000F30 configured). These, not INT_STS.PCFG_DONE, say whether the PL
/// is configured *now*: PCFG_DONE is a latched interrupt flag, and the
/// PROG_B pulse itself latches it.
pub const STATUS_PSS_FST_CFG_B: u32 = 1 << 10;
pub const STATUS_PSS_GTS_CFG_B: u32 = 1 << 8;
pub const STATUS_CONFIGURED: u32 = STATUS_PSS_FST_CFG_B | STATUS_PSS_GTS_CFG_B;

pub const IXR_D_P_DONE: u32 = 1 << 12;
pub const IXR_PCFG_DONE: u32 = 1 << 2;
pub const IXR_ERROR_FLAGS: u32 = 0x00F0_C860;
pub const IXR_ALL: u32 = 0xF8F7_F87F;

pub const DEVC_UNLOCK_KEY: u32 = 0x757B_DF0D;
pub const DMA_INVALID_ADDRESS: u32 = 0xFFFF_FFFF;
pub const DMA_SRC_LAST_TRANSFER: u32 = 1;
