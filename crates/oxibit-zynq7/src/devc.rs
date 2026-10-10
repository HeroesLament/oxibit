//! The Zynq-7000 backend: DevC/PCAP programming and the PS-PL boundary.

use std::io;
use std::time::{Duration, Instant};

use oxibit_guard::{Backend, Hardware, ProgramReport};
use serde_json::json;

use crate::aperture::ZynqAperture;
use crate::dt::{self, Region};
use crate::mem::{PhysMap, dsb};
use crate::regs::*;

/// Where things are. [`Config::discover`] fills it from the device tree.
#[derive(Debug, Clone)]
pub struct Config {
    /// PL register window, e.g. `0x4000_0000` for M_AXI_GP0.
    pub aperture_base: u64,
    pub aperture_len: usize,
    /// PCAP DMA staging buffer; must be `no-map` reserved memory.
    pub dma_buffer: Option<Region>,
    /// PS reference clock, for decoding FCLK frequencies.
    pub ps_clk_hz: Option<u32>,
}

impl Config {
    pub fn discover(aperture_base: u64, aperture_len: usize) -> Self {
        Config { aperture_base, aperture_len, dma_buffer: dt::pcap_buffer(), ps_clk_hz: dt::ps_clk_hz() }
    }
}

pub struct Zynq7 {
    regs: PhysMap,
    dma: Option<PhysMap>,
    aperture: ZynqAperture,
    config: Config,
}

impl Zynq7 {
    /// Map SLCR/DevC, the DMA buffer (if configured) and the aperture. No
    /// register is written and no bus access is made to the PL.
    pub fn open(config: Config) -> io::Result<Self> {
        let regs = PhysMap::open(SLCR_BASE, MAP_LEN)?;
        let dma = match config.dma_buffer {
            Some(r) => Some(PhysMap::open(r.base, r.size)?),
            None => None,
        };
        let aperture = ZynqAperture::open(config.aperture_base, config.aperture_len)?;
        Ok(Zynq7 { regs, dma, aperture, config })
    }

    pub fn config(&self) -> &Config {
        &self.config
    }

    /// FCLK n in Hz, decoded from its source PLL and two divisors.
    pub fn fclk_hz(&self, n: usize) -> Option<u64> {
        let ps = self.config.ps_clk_hz? as u64;
        let ctrl = self.regs.r32(FPGA_CLK_CTRL[n]);
        let pll_ctrl = match (ctrl >> 4) & 0x3 {
            0 | 1 => IO_PLL_CTRL,
            2 => ARM_PLL_CTRL,
            _ => DDR_PLL_CTRL,
        };
        let pll = self.regs.r32(pll_ctrl);
        // PLL_BYPASS_FORCE (bit 4) passes the reference straight through.
        let pll_hz = if pll & (1 << 4) != 0 { ps } else { ps * ((pll >> 12) & 0x7F) as u64 };
        let d0 = ((ctrl >> 8) & 0x3F).max(1) as u64;
        let d1 = ((ctrl >> 20) & 0x3F).max(1) as u64;
        Some(pll_hz / (d0 * d1))
    }

    fn fclks(&self) -> serde_json::Value {
        (0..4)
            .map(|n| {
                json!({
                    "n": n,
                    "ctrl": hex(self.regs.r32(FPGA_CLK_CTRL[n])),
                    "mhz": self.fclk_hz(n).map(|hz| (hz as f64 / 1e4).round() / 100.0),
                })
            })
            .collect()
    }

    /// PROG_B pulse, observing PCFG_INIT at each edge (UG585; zynq-fpga.c
    /// `zynq_fpga_ops_write_init`). Erases the PL. The caller has already
    /// cleared PCFG_DONE and closed the boundary.
    fn prog_b_pulse(&self) -> Result<(), String> {
        let r = &self.regs;
        let init = || r.r32(STATUS) & STATUS_PCFG_INIT != 0;
        let t = Duration::from_millis(100);

        r.w32(CTRL, r.r32(CTRL) | CTRL_PCFG_PROG_B);
        poll("PCFG_INIT high", t, init)?;
        r.w32(CTRL, r.r32(CTRL) & !CTRL_PCFG_PROG_B);
        poll("PCFG_INIT low", t, || !init())?;
        r.w32(CTRL, r.r32(CTRL) | CTRL_PCFG_PROG_B);
        poll("PCFG_INIT high again", t, init)
    }

    /// Enter the configuration state shared by `program` and `clear`:
    ///
    /// 1. clear the latched PCFG_DONE, so that from this instant until a load
    ///    completes, `hardware()` reports the PL unconfigured, even if this
    ///    process dies half way. (The kernel and `devcfg_load` clear it after
    ///    PROG_B, which leaves a window where a blank PL reads as configured.)
    /// 2. assert the PL AXI resets, drop both level-shifter directions, then
    ///    re-enable only PS-to-PL;
    /// 3. pulse PROG_B, which erases the PL.
    fn enter_config(&self) -> Result<(), String> {
        let r = &self.regs;
        // The kernel takes the PCAP clock around each operation, so it may be
        // gated; without it every DevC register reads as garbage.
        r.w32(PCAP_CLK_CTRL, r.r32(PCAP_CLK_CTRL) | 1);
        r.w32(UNLOCK, DEVC_UNLOCK_KEY);

        r.w32(INT_STS, IXR_ALL);

        r.w32(FPGA_RST_CTRL, FPGA_RST_ALL);
        r.w32(LVL_SHFTR_EN, LVL_SHFTR_DISABLE_ALL);
        r.w32(LVL_SHFTR_EN, LVL_SHFTR_ENABLE_PS_TO_PL);
        expect_reg(r, FPGA_RST_CTRL, 0xF, FPGA_RST_ALL, "FPGA_RST_CTRL")?;
        expect_reg(r, LVL_SHFTR_EN, 0xF, LVL_SHFTR_ENABLE_PS_TO_PL, "LVL_SHFTR_EN")?;

        self.prog_b_pulse()?;

        // The pulse itself latches interrupt flags, PCFG_DONE among them
        // (measured: INT_STS 0x5802000F straight after a clear). Clear them
        // again so nothing reads a blank PL as configured.
        r.w32(INT_STS, IXR_ALL);
        Ok(())
    }
}

impl Backend for Zynq7 {
    type Aperture = ZynqAperture;

    fn idcode(&self) -> io::Result<u32> {
        Ok(self.regs.r32(PSS_IDCODE))
    }

    fn hardware(&self) -> io::Result<Hardware> {
        let r = &self.regs;
        let pcap_clk = r.r32(PCAP_CLK_CTRL);
        let lvl = r.r32(LVL_SHFTR_EN);
        let rst = r.r32(FPGA_RST_CTRL);

        // With the PCAP clock gated, DevC reads are meaningless: report not
        // done rather than trust them.
        let (int_sts, status, ctrl) =
            if pcap_clk & 1 != 0 { (Some(r.r32(INT_STS)), Some(r.r32(STATUS)), Some(r.r32(CTRL))) } else { (None, None, None) };

        let done = status.is_some_and(|s| s & STATUS_CONFIGURED == STATUS_CONFIGURED);
        let boundary_open = lvl & 0xF == LVL_SHFTR_ENABLE_PL_TO_PS && rst & 0xF == FPGA_RST_NONE;

        Ok(Hardware {
            done,
            boundary_open,
            unknown: (pcap_clk & 1 == 0).then(|| "PCAP clock gated (PCAP_CLK_CTRL.CLKACT = 0), DevC unreadable".into()),
            detail: json!({
                "int_sts": int_sts.map(hex),
                "status": status.map(hex),
                "ctrl": ctrl.map(hex),
                "pcap_clk_ctrl": hex(pcap_clk),
                "slcr_locked": r.r32(SLCR_LOCKSTA) & 1 != 0,
                "lvl_shftr_en": hex(lvl),
                "fpga_rst_ctrl": hex(rst),
                "pcfg_init": status.map(|s| s & STATUS_PCFG_INIT != 0),
                "fst_cfg_b": status.map(|s| s & STATUS_PSS_FST_CFG_B != 0),
                "gts_cfg_b": status.map(|s| s & STATUS_PSS_GTS_CFG_B != 0),
                "pcfg_done_latched": int_sts.map(|s| s & IXR_PCFG_DONE != 0),
                "fclk": self.fclks(),
                "ps_clk_hz": self.config.ps_clk_hz,
                "dma_buffer": self.config.dma_buffer.map(|b| format!("0x{:X}+0x{:X}", b.base, b.size)),
                "aperture": format!("0x{:X}+0x{:X}", self.config.aperture_base, self.config.aperture_len),
            }),
        })
    }

    fn program(&mut self, words: &[u32]) -> Result<ProgramReport, String> {
        let started = Instant::now();
        let bytes = words.len() * 4;

        // --- checks, before any register is touched --------------------------
        let dma = self.dma.as_ref().ok_or(
            "no PCAP DMA buffer: the device tree needs a no-map reserved-memory node named pcap* \
             (see oxibit-zynq7 docs)",
        )?;
        if bytes > dma.len() {
            return Err(format!("bitstream is {bytes} bytes, DMA buffer is {}", dma.len()));
        }

        // --- erase, then stage ------------------------------------------------
        //
        // The buffer is shared: nervezynq's HP rings and ocu region live inside
        // the same reservation, and a running design may be DMA-ing into it.
        // Erasing first stops that design, so nothing can write the buffer
        // between staging, its readback check, and the PCAP DMA.
        let r = &self.regs;
        let _slcr = SlcrUnlocked::new(r);
        self.enter_config()?;
        let erased_ms = started.elapsed().as_millis();

        // The PCAP consumes words in the order a byte-swapped .bin holds them:
        // on this little-endian CPU that is simply each canonical word stored
        // natively.
        //
        // 64-bit accesses (STRD/LDRD): the buffer is mapped strongly ordered,
        // where every access is its own bus transaction, so halving their
        // number roughly halves staging time. The readback stays: bitstreams
        // from xc7frames2bit / oxibit carry no CRC packet, so this compare is
        // the only check that the PCAP is fed what was written.
        dma.write_words(words);
        dsb();
        if let Some(i) = dma.first_mismatch(words) {
            let got = dma.r32(i * 4);
            return Err(format!("staging buffer readback mismatch at word {i}: wrote 0x{:08X}, read 0x{got:08X}", words[i]));
        }
        let staged_ms = started.elapsed().as_millis() - erased_ms;
        let dma_started = Instant::now();

        // Select PCAP, full configuration.
        r.w32(CTRL, r.r32(CTRL) | CTRL_PCAP_PR | CTRL_PCAP_MODE);

        let status = r.r32(STATUS);
        if status & STATUS_DMA_Q_F != 0 || status & STATUS_DMA_Q_E == 0 {
            return Err(format!("DMA command queue not idle (STATUS 0x{status:08X})"));
        }

        // Internal loopback would swallow the bitstream.
        r.w32(MCTRL, r.r32(MCTRL) & !MCTRL_PCAP_LPBK);
        r.w32(INT_MASK, 0xFFFF_FFFF); // poll, no interrupts
        r.w32(INT_STS, IXR_ALL);

        // One transfer from the contiguous buffer; bit 0 marks it the last.
        let base = dma.phys() as u32;
        r.w32(DMA_SRC_ADDR, base | DMA_SRC_LAST_TRANSFER);
        r.w32(DMA_DST_ADDR, DMA_INVALID_ADDRESS);
        r.w32(DMA_SRC_LEN, words.len() as u32);
        r.w32(DMA_DEST_LEN, 0);

        poll("DMA and PCAP done", Duration::from_secs(5), || r.r32(INT_STS) & (IXR_D_P_DONE | IXR_ERROR_FLAGS) != 0)?;
        let sts = r.r32(INT_STS);
        if sts & IXR_ERROR_FLAGS != 0 {
            return Err(format!(
                "DMA error: INT_STS 0x{sts:08X} CTRL 0x{:08X} STATUS 0x{:08X}",
                r.r32(CTRL),
                r.r32(STATUS)
            ));
        }

        if poll("PCFG_DONE", Duration::from_secs(1), || r.r32(INT_STS) & IXR_PCFG_DONE != 0).is_err() {
            return Err(format!(
                "timeout waiting for PCFG_DONE: INT_STS 0x{:08X} STATUS 0x{:08X} CTRL 0x{:08X} MCTRL 0x{:08X}",
                r.r32(INT_STS),
                r.r32(STATUS),
                r.r32(CTRL),
                r.r32(MCTRL)
            ));
        }
        // PCFG_DONE says the engine accepted the stream; the STATUS levels say
        // startup finished and the design is running.
        poll("PSS_FST_CFG_B and PSS_GTS_CFG_B", Duration::from_millis(100), || {
            r.r32(STATUS) & STATUS_CONFIGURED == STATUS_CONFIGURED
        })?;
        let dma_ms = dma_started.elapsed().as_millis();

        // Hand PR control back to the ICAP, then open the boundary in reverse
        // order: PL-to-PS shifters, then release the AXI resets.
        r.w32(CTRL, r.r32(CTRL) & !CTRL_PCAP_PR);
        r.w32(LVL_SHFTR_EN, LVL_SHFTR_ENABLE_PL_TO_PS);
        r.w32(FPGA_RST_CTRL, FPGA_RST_NONE);
        expect_reg(r, LVL_SHFTR_EN, 0xF, LVL_SHFTR_ENABLE_PL_TO_PS, "LVL_SHFTR_EN")?;
        expect_reg(r, FPGA_RST_CTRL, 0xF, FPGA_RST_NONE, "FPGA_RST_CTRL")?;
        let _ = r.r32(LOCK);

        Ok(ProgramReport {
            ms: started.elapsed().as_millis(),
            bytes,
            detail: json!({
                "erase_ms": erased_ms,
                "staged_ms": staged_ms,
                "configure_ms": dma_ms,
                "dma_buffer": format!("0x{:X}", dma.phys()),
            }),
        })
    }

    fn clear(&mut self) -> Result<(), String> {
        {
            let _slcr = SlcrUnlocked::new(&self.regs);
            // Leaves the boundary closed and the AXI resets asserted, as a
            // failed load does: nothing is behind it.
            self.enter_config()?;
        }
        let hw = self.hardware().map_err(|e| e.to_string())?;
        if hw.done || hw.boundary_open {
            return Err(format!("after clear the hardware still reads done={} boundary_open={}", hw.done, hw.boundary_open));
        }
        Ok(())
    }

    fn aperture(&self) -> &ZynqAperture {
        &self.aperture
    }
}

/// SLCR unlocked for its lifetime; on every exit path the lock is put back
/// the way it was found. Linux's zynq SLCR driver unlocks at boot and leaves
/// it unlocked for its own later writes (clock gating, restart), so locking it
/// unconditionally, as `devcfg_load` did, would break the kernel.
struct SlcrUnlocked<'a> {
    regs: &'a PhysMap,
    was_locked: bool,
}

impl<'a> SlcrUnlocked<'a> {
    fn new(regs: &'a PhysMap) -> Self {
        let was_locked = regs.r32(SLCR_LOCKSTA) & 1 != 0;
        regs.w32(SLCR_UNLOCK, SLCR_UNLOCK_KEY);
        SlcrUnlocked { regs, was_locked }
    }
}

impl Drop for SlcrUnlocked<'_> {
    fn drop(&mut self) {
        if self.was_locked {
            self.regs.w32(SLCR_LOCK, SLCR_LOCK_KEY);
        }
    }
}

/// Read back a register that was just written. A write that did not land
/// (SLCR locked by someone else, say) must stop the sequence, not be assumed.
fn expect_reg(r: &PhysMap, off: usize, mask: u32, want: u32, name: &str) -> Result<(), String> {
    let got = r.r32(off) & mask;
    if got == want {
        Ok(())
    } else {
        Err(format!("{name} reads 0x{got:X} after writing 0x{want:X}"))
    }
}

fn poll(what: &str, timeout: Duration, f: impl Fn() -> bool) -> Result<(), String> {
    let start = Instant::now();
    loop {
        if f() {
            return Ok(());
        }
        if start.elapsed() > timeout {
            return Err(format!("timeout waiting for {what} ({} ms)", timeout.as_millis()));
        }
        std::thread::sleep(Duration::from_micros(10));
    }
}

fn hex(v: u32) -> String {
    format!("0x{v:08X}")
}
