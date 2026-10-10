# oxibit

Tooling for Xilinx configuration bitstreams, from the file on the host to the
programmable logic on the device.

- **Host:** read, check, identify and convert bitstreams, including the PL
  partition of a Zynq-7000 `BOOT.BIN`.
- **Zynq-7000 target:** load the PL from Linux userspace, and access it,
  without ever letting the CPU touch an unconfigured fabric (a read there
  hangs the core). See [docs/zynq7.md](docs/zynq7.md).

## Crates

| Crate | What |
|---|---|
| `oxibit-bitstream` | Containers (`.bit`, raw `.bin`, byte-swapped `.bin`), configuration packets, CRC, IDCODE to die, Zynq-7000 boot images. No I/O, no hardware. |
| `oxibit-guard` | The safety model, chip-independent: one owner (lock), bitstream and IDCODE checked before hardware, access only while hardware says ready, loaded-bitstream marker. `Backend` and `Aperture` traits. |
| `oxibit-zynq7` | Zynq-7000 backend: DevC/PCAP load from a reserved uncached buffer, PS-PL boundary, M_AXI_GP aperture, FCLK decode. Linux only. |
| `oxibit` | The CLI and the Erlang port. |

## CLI

```sh
cargo build --release
oxibit inspect FILE...            # container, header, die, frames, commands, CRC, hash
oxibit inspect --packets FILE     # plus every packet (NOP runs collapsed)
oxibit verify --part xc7z020 FILE...   # exit 1 if anything is wrong or it targets another die
oxibit convert IN OUT [--to swapped-bin|raw-bin|bit] [--part P] [--design D]
```

On a Zynq-7000 target (build with `scripts/build-armv7.sh`, run as root):

```sh
oxibit status [--json]                     # configured?, boundary, FCLKs, what is loaded, lock holder
oxibit load FILE [--expect 0x0=0x45484431] # any container or BOOT.BIN; --expect checks a word after load
oxibit clear                               # erase the PL, close the boundary
oxibit peek OFFSET [COUNT] [--repeat]      # refused unless ready
oxibit poke OFFSET VALUE...                # refused unless ready
oxibit port                                # Erlang port ({:packet, 4}) on stdin/stdout
```

`--base`/`--len` select the aperture (default M_AXI_GP0, `0x40000000`
+ `0x20000`). `status` takes no lock and never touches the PL; everything
else takes the lock, so a running `port` makes the other commands fail with
the owner's pid.

All commands take `.bit`, raw `.bin`, swapped `.bin`, any of those gzipped,
and Zynq-7000 boot images (`BOOT.BIN`), whose PL partitions are read directly.
`--json` gives one JSON object per result.

### Port protocol

Ops 0x01 to 0x06 are byte-compatible with nervezynq's `fabric_port` (READ32,
WRITE32, READ_BLOCK, WRITE_BLOCK, INFO, READ_REPEAT). Added: 0x10 LOAD (UTF-8
path, JSON reply), 0x11 STATUS, 0x12 CLEAR. Access ops are refused with an
error reply unless the PL is ready, checked in the port itself on every
request; LOAD and CLEAR run in the same process under the same lock.
Details in `crates/oxibit/src/port.rs`.

`convert` defaults to `swapped-bin`, the form the Zynq-7000 PCAP and Linux
`fpga_manager` accept. For a `.bit` input it is byte-for-byte what ehdl's
`Hw.Xilinx.Bit2Bin` produces (checked by `scripts/crosscheck_bit2bin.exs`).

### What `verify` checks

- the data is word-aligned and has a sync word, and every packet header is
  well formed and in bounds;
- the stream writes an IDCODE, the IDCODE is a known die, and it matches the
  `.bit` header's part and `--part` if given;
- every CRC check the stream carries matches a recomputed CRC;
- there is frame data, and the stream ends with DESYNC.

It warns (without failing) on a missing START, frame data that is not a whole
number of 101-word frames, and unknown dies or opcodes.

### The config hash

`config sha256` covers the words from the first sync word through the last
packet. Padding is excluded, so one design has one hash whether it is in a
`.bit`, a raw or swapped `.bin`, or a bootgen partition. That is the hash to
record and to check before loading.

## Validation

| Check | Data | Result |
|---|---|---|
| `convert` vs `Hw.Xilinx.Bit2Bin` | 27 openXC7 `.bit` files in ehdl `designs/` | 27 byte-identical; 6 ECP5 `.bit` files rejected by both |
| `verify --part xc7z020` | 164 files in `libresdr_radio/taps/serve` | 160 ok; the 4 failures are DDR ring images, not bitstreams |
| CRC model | Vivado 2022.2 compressed `system_top.bit` (stock LibreSDR) | 2 of 2 checks match |
| Boot image | stock LibreSDR `BOOT.bin` | header and 3 partition checksums ok; PL partition hashes equal to `system_top.bit` |
| Target load | LibreSDR dr-c, radio bitstream as `.bin` and as `.bit` | 175 to 187 ms; magic word, scratch write/read and heartbeat verified |
| Target refusals | truncated file, non-bitstream, access after `clear`, CLI load while `port` runs | all refused before touching hardware |
| Port protocol | all nine ops from IEx | as specified; errors for out-of-range and misaligned access |

openXC7's `xc7frames2bit` writes RCRC but no CRC checks, so its output does
not exercise the CRC path. The Vivado bitstream does.

## Sources

- CRC algorithm: prjxray `lib/include/prjxray/xilinx/xc7series/crc.h`
  (`icap_crc`), transcribed.
- IDCODEs: prjxray-db revision 517d66a (openXC7 fork), `part.yaml`;
  extraction in `data/prjxray-idcodes.txt`.
- Packet format, registers and commands: UG470.
- Boot image layout: bootgen's format, confirmed field by field against the
  stock LibreSDR `BOOT.bin` (`docs/formats.md`).
- Zynq-7000 load sequence: mainline `drivers/fpga/zynq-fpga.c`, via
  nervezynq's `devcfg.rs`; DevC STATUS bit names: FreeBSD `zy7_devcfg.c`;
  everything else measured, see `docs/zynq7.md`.

## Roadmap

1. **nervezynq on oxibit:** `Nervezynq.Fabric` opens `oxibit port` instead of
   `fabric_port`, `Nervezynq.PL` becomes LOAD/STATUS/CLEAR, and
   `devcfg_load`, `fabric_port` and `pl_load.exs` are retired.
2. **Manifests:** name, config hash, container hash, design ID and build
   parameters per published bitstream; `load` checks against it.
3. **Readback:** compare configuration frames read back through the PCAP
   against the bitstream (masking dynamic bits).
4. **Boot-time load:** a bitstream shipped in the firmware, loaded by the
   application at start through the same guard.

Other families come in behind the same traits when there is hardware to test
on: ZynqMP and Versal through the kernel's `fpga_manager`, standalone 7-series
over JTAG.
