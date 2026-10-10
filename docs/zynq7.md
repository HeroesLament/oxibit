# Zynq-7000 backend

How `oxibit-zynq7` loads the PL and decides when the PL may be touched, and
what the hardware showed on a LibreSDR (xc7z020, IDCODE 0x23727093) on
2026-09-29.

## Requirements on the target

- Linux with `/dev/mem`, run as root (`CAP_SYS_RAWIO`).
- A `no-map` reserved-memory node whose name starts with `pcap`, at least as
  large as the bitstream. nerves_system_libresdr has:

  ```dts
  reserved-memory {
      #address-cells = <1>;
      #size-cells = <1>;
      ranges;
      pcap_readback: pcap-readback@3fc00000 {
          reg = <0x3fc00000 0x00400000>;
          no-map;
      };
  };
  ```

  Without it `load` refuses. There is deliberately no fallback: the DevC DMA
  does not snoop the caches, userspace cannot clean them to the point of
  coherency, and an ordinary buffer only works when the L2 happens to have
  written it back.
- `ps-clk-frequency` in the SLCR clock controller node, for FCLK decoding
  (50 MHz on the LibreSDR; 33.333 MHz on most Zynq boards).

## Load sequence

| Step | What | Why |
|---|---|---|
| 1 | parse, analyze, match IDCODE against `PSS_IDCODE` | nothing is touched for a bad or wrong-part file |
| 2 | remove the marker | from here the PL may be blank |
| 3 | stage words in the reserved buffer, read back and compare | uncached, so this is what the DMA will read |
| 4 | clear `INT_STS` | before PROG_B, so a crash anywhere after leaves no stale PCFG_DONE |
| 5 | assert `FPGA_RST_CTRL`, `LVL_SHFTR_EN` 0 then 0xA, **read both back** | a write that did not land stops the load |
| 6 | PROG_B pulse, observing PCFG_INIT at each edge | erases the PL |
| 7 | clear `INT_STS` again | the pulse itself latches flags, PCFG_DONE included |
| 8 | PCAP_PR and PCAP_MODE, loopback off, one DMA from the buffer | contiguous buffer: one transfer, bit 0 of the source marks it last |
| 9 | wait D_P_DONE, then PCFG_DONE, then STATUS FST_CFG_B and GTS_CFG_B | engine accepted, then startup finished |
| 10 | `LVL_SHFTR_EN` 0xF, `FPGA_RST_CTRL` 0, **read both back** | boundary open |
| 11 | guard re-reads hardware; only then write the marker | |

The SLCR lock is restored to whatever it was. Linux leaves it unlocked
(measured: `LOCKSTA` 0) and relies on that for its own writes.

Measured: 175 to 187 ms per load of a 4,045,564-byte xc7z020 bitstream,
of which about 140 ms is staging and read-back through the uncached mapping
and 41 ms is PROG_B through startup.

## When the PL may be touched

`ready` = configured and boundary open, re-read before every access:

- **configured**: DevC `STATUS` bits 10 (`PSS_FST_CFG_B`) and 8
  (`PSS_GTS_CFG_B`) both set. Bit names from FreeBSD `zy7_devcfg.c`.
- **boundary open**: `LVL_SHFTR_EN` = 0xF and `FPGA_RST_CTRL[3:0]` = 0.
- With the PCAP clock gated, DevC cannot be read and the state is reported
  unknown, which counts as not ready.

Measured on dr-c:

| State | STATUS | INT_STS | LVL_SHFTR_EN | FPGA_RST_CTRL |
|---|---|---|---|---|
| after boot, PL blank | 0x40000A30 | 0x00020000 | 0xF | 0x0 |
| loaded | 0x50000F30 | 0x50023004 | 0xF | 0x0 |
| cleared | 0x40000A30 | 0x00020000 | 0xA | 0xF |
| loaded again | 0x50000F30 | 0x50023004 | 0xF | 0x0 |

Note the boot row: the boundary is **open** with the PL blank (U-Boot SPL's
`ps7_post_config` opens it). The boundary alone never proves anything.

### Why not `INT_STS.PCFG_DONE`

It is a latched interrupt flag, not the DONE level. The first version of this
backend gated on it and hung dr-c: `clear` pulsed PROG_B, the pulse latched
PCFG_DONE again (INT_STS read 0x5802000F on a blank PL), the gate opened, and
the next `peek` never returned. The SWDT (`reset-on-timeout`) reset the board.
The mainline driver only reads PCFG_DONE straight after its own DMA, which is
the one place it means something; FreeBSD's `pl_done` clears it and reads it
again for the same reason.

## Register notes

- `PCAP_CLK_CTRL` is SLCR **0x168** (mainline `clkc.c`: `clk_base + 0x68`).
  nervezynq's `devcfg.rs` uses 0x18C, which is `FPGA1_THR_STA`: its "enable
  the PCAP clock" write has never reached the PCAP clock and worked only
  because `ps7_init` enables it. The first oxibit build inherited the offset,
  read THR_STA as the clock, and reported every load as unconfirmed.
- `PSS_IDCODE` (SLCR 0x530) reads the device IDCODE including revision;
  bitstream IDCODEs are compared with bits [31:28] masked.
- FCLKn = PLL / (DIVISOR0 x DIVISOR1), PLL from `FPGAn_CLK_CTRL[5:4]`
  (IO, ARM or DDR), `PLL_FDIV` = `PLL_CTRL[18:12]`, bypass at bit 4. Measured
  on the LibreSDR: FCLK0 100 MHz, FCLK1 200 MHz, FCLK2/3 41.67 MHz.

## Ownership

One `Guard` holds `flock(LOCK_EX)` on `/run/oxibit/lock` (pid inside) for
its lifetime. `oxibit port` holds it while running; `load`, `clear`, `peek`
and `poke` take it per command; `status` never does. Other `/dev/mem` users
(nervezynq's `devcfg_load`, `fabric_port`, `SiliconSweep.Devcfg`) do not
know about the lock; retire them rather than mix them with oxibit.

The marker (`/run/oxibit/loaded.json`, tmpfs) names what oxibit loaded. It is
trusted only while the hardware reads configured; a configured PL with no
marker is reported as configured by something else, identity unknown.
