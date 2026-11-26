# oxibit

An FPGA toolchain in Rust: Verilog in, bitstream out, loaded over JTAG, for Lattice ECP5, Lattice CrossLink-NX and Xilinx 7-series (Zynq-7020).

No yosys, no nextpnr. Synthesis, placement, routing, timing and bitstream generation are all in this workspace. The only external inputs are the open chip databases (Project Trellis, Project Oxide, Project X-Ray) and OpenOCD for JTAG.

oxibit is the back end for [EHDL](https://github.com/HeroesLament/ehdl), an Elixir hardware description DSL, but it takes plain Verilog and works on its own.

## Supported targets

| Family | Part | Board | `build` | `prog` | Constraints |
|---|---|---|---|---|---|
| ECP5 | LFE5U-85F | Radiona ULX3S (`--board ulx3s`) | yes | yes | `.lpf`, pins built in |
| CrossLink-NX | LIFCL-40 | Lattice CrossLink-NX EVN (`--board clnx`) | yes | yes | `.pdc`, LED pins built in |
| 7-series | xc7z020clg400-1 | LibreSDR (`--board libresdr`) | yes | see [docs/zynq7.md](docs/zynq7.md) | `.xdc`, AD9363 pins built in |

Only primitive shapes that have been proven on silicon are accepted. An unproven shape is an error, not a guess.

## Install

```sh
git clone https://github.com/HeroesLament/oxibit
cd oxibit
cargo build --release
```

The binary is `target/release/oxibit`. A zsh completion script comes from `oxibit completions zsh`.

### Requirements

- A recent stable Rust toolchain.
- The chip database for the family you target (see below).
- OpenOCD, for `prog` and `--load`.

| Variable | Used for | Default |
|---|---|---|
| `OXIBIT_TRELLIS_DB` | ECP5 database | `~/oss-cad-suite/share/trellis/database` |
| `OXIBIT_PRJOXIDE_DB` | CrossLink-NX database | `~/src/prjoxide/database` |
| `OXIBIT_XRAY_DB` | 7-series prjxray family directory | none, or pass `--db-root` |
| `OXIBIT_XC7_META` | 7-series metadata | none |
| `OPENOCD` | OpenOCD binary | the copy under `~/oss-cad-suite` |

## Usage

Build a design for a board, then load it:

```sh
oxibit build blinky.v --board ulx3s --load
oxibit build blinky.v --board clnx --load
oxibit build radio.v  --board libresdr --constraints libresdr.xdc
```

The output is `<input stem>.bit` in the current directory unless `-o` is given. The part decides the flow: `LFE5...` is ECP5, `LIFCL...` is CrossLink-NX, `xc7...` is 7-series. `--top` picks the top module (default: the last one in the file).

List the top-level port names a board's constraints locate, which is what a design may call its ports:

```sh
oxibit pins --board ulx3s
oxibit pins --board clnx
oxibit pins --board libresdr
```

`pins` reads LPF, PDC and XDC, so it works for any backend. Pass `--constraints` to read your own file instead of the built-in one.

Program a built bitstream into SRAM:

```sh
oxibit prog blinky.bit
oxibit prog blinky.bit --board clnx
```

### Other commands

| Command | Purpose |
|---|---|
| `inspect`, `verify`, `convert` | Describe, check and re-container Xilinx bitstreams |
| `fasm2bit` | FASM to a 7-series `.bit` against a prjxray database |
| `pnr` | Place and route on oxibit's routing graph |
| `timing` | Silicon timing characterization from phase sweeps |
| `route`, `route-port`, `build-port` | Test harnesses and Erlang ports (`{:packet, 4}`) |
| `status`, `load`, `clear`, `peek`, `poke`, `port` | Zynq PL control, Linux on the target only |

`oxibit <command> --help` is the reference for flags.

### Progress and events

Progress goes to stderr: a live status line on a terminal, plain lines otherwise. `--progress json` switches stderr to JSON Lines, `--events FILE` writes the same stream to a file, and `-q` silences it. The event format is in [docs/events.md](docs/events.md).

## How it works

Per family, `build` runs the same stages in one process:

1. Synthesis from Verilog to the family's primitives (`oxibit-hdl`, `oxibit-synth`).
2. Placement and packing (`oxibit-place`).
3. Routing on the family's routing graph (`oxibit-pnr`).
4. Static timing (`oxibit-sta`, `oxibit-timing`).
5. Bitstream generation (`oxibit-ecp5`, `oxibit-nexus`, `oxibit-fasm`, `oxibit-bitstream`).

CrossLink-NX SRAM load plays the Certus JTAG sequence as SVF through OpenOCD. The whole `.bit`, including its 8-byte header, is sent.

## Workspace

| Crate | Role |
|---|---|
| `oxibit` | The CLI |
| `oxibit-hdl` | Verilog front end |
| `oxibit-synth` | Synthesis |
| `oxibit-place` | Placement and packing |
| `oxibit-pnr` | Routing |
| `oxibit-policy` | Synthesis, placement and routing policy |
| `oxibit-sta`, `oxibit-timing` | Static timing and silicon timing data |
| `oxibit-ecp5` | ECP5 device model and bitstream |
| `oxibit-nexus` | CrossLink-NX device model, PDC and bitstream |
| `oxibit-zynq7` | 7-series support and the LibreSDR board file |
| `oxibit-fasm` | FASM handling |
| `oxibit-bitstream` | Xilinx bitstream container parsing and writing |
| `oxibit-cec` | Equivalence checking |
| `oxibit-guard`, `oxibit-report` | Checks and progress reporting |
| `oxibit-prof`, `oxibit-prof-macros` | Profiling |

`oxibit-sta` is excluded from the cargo workspace and builds on its own.

## Documentation

Design notes live in [docs/](docs/): [events](docs/events.md), [formats](docs/formats.md), [fmax](docs/fmax.md), [sypher](docs/sypher.md), [zynq7](docs/zynq7.md), plus placement, routing and synthesis notes.

## Development

```sh
cargo build
cargo test
```

Helper scripts are in [scripts/](scripts/).

## Status

Working end to end on ECP5 (ULX3S) and CrossLink-NX (EVN), and building for 7-series. The same EHDL source, wrapped per board, runs on all three families.

## License

Dual licensed under MIT or Apache-2.0 at your option. See `LICENSE-MIT` and `LICENSE-APACHE`.
