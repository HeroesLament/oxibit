# Synthesis Improvement Flow

*Oxibit Phase 5 — standalone synthesis: closing the three gaps between `oxibit-synth` and a flow that no longer needs yosys.*

## Where we are

The project's placement and routing are now competitive: timing-weighted placement recovers 6.67 ns of the 8 ns gap to nextpnr, our place+route runs ~23x faster than the reference flow, and the routes are legal. But the pipeline is not standalone: synthesis goes through yosys, which costs **1304 s** on picoBEAM, where `oxibit-synth` runs the same input in **5.2 s** — a 250x speedup that we currently give away.

The reason we still need yosys is concrete and measured on the current netlist (design ID `0x0B5E000A`, 4096-bundle code memory):

| oxibit-synth setting | LUTs (device: 53,200) | DSP48E1 (device: 220) | CARRY4 | unmapped memories |
|---|---|---|---|---|
| defaults | 51,693 (97%) | 488 | 7,465 | 54 |
| carry off, DSP off | 91,237 (171%) | 0 | 0 | 54 |
| yosys (reference) | 46,252 | 24 | (used, placed fine) | 0 |

Three gaps keep oxibit from producing a fitting netlist on its own:

1. **No DSP budget.** Multipliers map to DSP48E1s with no regard for the device's 220; turning DSPs off entirely overflows the device to 171%.
2. **54 memories left as `$mem_v2`.** These are picoBEAM's multi-port tables — mailboxes, process tables, link tables, heap halves, and deep async-read stacks, roughly 27 per engine. The BRAM pass rejects them (multiple read ports, multiple write ports, async read deeper than 128) and *nothing else* lowers them to LUTRAM or flops. The reference tool's `memory_map` covers exactly this case.
3. **CARRY4 placement.** The default policy emits 7,465 CARRY4s; the packer's carry pass (`pack/carry.rs`) exists but the bel step reports chains spanning tiles as unsupported, and all validated netlists so far were `-nocarry`.

This document lays out the flow to close all three and make oxibit end-to-end standalone: **Verilog -> oxibit-synth -> pack/place -> route -> fasm2bit, with yosys nowhere in it.**

## The architectural observation that unifies the three gaps

Studying best practices for production FPGA synthesis yields one consistent answer, and it is *not* "better heuristics inside each mapper." It is this:

> **Synthesis is not a device-blind pass that hands a netlist downstream. Synthesis is device-aware from the start, and every resource decision — DSP vs fabric, BRAM vs LUTRAM vs SRL vs flops, carry chain shape — is made *against the target device model*, with fall-through to the next-cheaper representation when the preferred one doesn't fit.**

Concretely, best-practice flows exhibit four properties our synthesis currently lacks:

- **The device model is an input to synthesis.** Resource limits (how many DSPs, how many BRAM columns, what carry shapes are legal) are consulted *during* mapping, not discovered at the packer. The failure "488 DSPs wanted, 220 exist" should be impossible to produce; the mapper should simply never emit more than fit, falling back to fabric LUTs as the budget binds.
- **Memory inference is a fall-through chain, not a pass with rejection criteria.** A candidate memory is tried as BRAM; if it doesn't qualify, it falls to LUTRAM; then to SRLs; then to flops. At each step the inference probes *pipelining legality* — specifically whether an output register exists (or can be inferred) that makes an asynchronous read acceptable. Rejection is never terminal; it routes to the next representation.
- **Compatible RAM instances are grouped.** A compatibility predicate decides which RAM instances can co-reside in shared physical structures — this matters both for BRAM tiling and for LUTRAM packing, and it is the difference between "54 memories become 54 scattered macro placements" and "54 memories become a small number of dense, well-placed blocks."
- **Carry chains are netlist-time geometry, not placement-time discovery.** Carry chains are represented as *relative* placements — site-relative coordinates baked into the mapped cells at netlist construction time — so that placement inherits a pre-shaped, legal chain rather than discovering illegal tile-spanning chains after the fact. The bel step should never be the first place chain legality is checked.

Everything below follows from these four properties. Note the echo of our own placement work: the same "local structure the global objective cannot see" framing that produced the LUT RAM excursion fix applies here — the fall-through chain and RPM-style chain shapes are mechanisms for making local structure explicit *before* the global stages fight it.

## Step 1: Device-aware synthesis (closes the DSP gap)

**Goal:** `oxibit-synth` consults the target device's resource limits during mapping; the DSP count emitted never exceeds what fits.

- Feed the synthesis mapper the same device facts the packer already uses: DSP48E1 count (220 on xc7z020), LUT count, SLICEM/LUTRAM sites, BRAM blocks. This data already exists in the prjxray database layer — no new sources, just a new consumer.
- Multiplier policy: map to DSP48E1 while the budget allows; when the cap binds, fall back to fabric LUTs, size-first (largest multipliers keep DSPs — they benefit most and cost the most fabric).
- **Pipelining probe first:** before committing a multiplier to a DSP, check whether its inputs/outputs have registers available to absorb — a DSP without absorbed pipeline registers is often *slower* than fabric with them. This is a query against the netlist (does an adjacent register exist / can one be inferred), and it should be part of the decision, not a post-hoc retiming pass.
- Acceptance: picoBEAM emits <= 220 DSP48E1s under defaults; LUT count stays within the device; the equivalence check (`oxibit-cec`) still passes. Target for the reference comparison: yosys uses 24 DSPs — we do not need to match its partition, only fit.

**Risk:** greedy fabric fall-back can push LUT count past 53,200. If it does on any design, the budget policy needs a second pass (re-balancing at the mapping level), not a packer-level fix.

## Step 2: Memory fall-through chain (closes the 54-`$mem_v2` gap)

**Goal:** no memory exits synthesis unmapped. Every `$mem_v2` resolves to BRAM, LUTRAM, SRL, or flops — chosen against the device and the memory's own shape.

The chain, in order, each step gating on a qualification check:

1. **BRAM** (existing pass): <= 1 write port behavior, synchronous read, depth that a RAMB36/RAMB18 pair can absorb. Keep current criteria; they are fine *as the first step* rather than the only one.
2. **LUTRAM**: for shallow or port-shaped-out memories — including multi-read-port tables (the distributed-RAM SLICEM primitive supports two reads natively, which covers picoBEAM's mailbox/process-table shapes).
3. **SRL**: for shift-register-shaped memories (single-stream, read position moves monotonically — the async-read stacks are candidates if their access pattern qualifies).
4. **Flops**: the floor. Always legal, always fits; the reference tool's `memory_map` bottoms out here too.

Two mechanisms make this chain behave like best-practice flows rather than a cascade of rejections:

- **The async-read output-register probe.** A memory with an asynchronous read can often be *made* BRAM-legal by absorbing or inferring one output register. Probe for it before falling through — the probe can rescue a memory into BRAM and it costs one netlist query.
- **A compatibility predicate for grouping.** Which memories may share a physical block: matching port shapes, compatible depths, and — critically for the placement side — *compatible placements*. This is where the SLICEM-column contention problem from Phase 4 connects: picoBEAM's ~27 tables per engine will all want sparse LUTRAM columns, and the Phase 4 excursion fix (pre-legalizing LUT RAM sites as placement anchors) must be designed *with* this step, not after it. Otherwise we fix the excursion pattern for one LUT RAM and then manufacture fifty more.

Acceptance: 0 unmapped memories on picoBEAM; `oxibit-cec` equivalence against the yosys netlist passes (this is the real gate — "synthesized it" must mean "synthesized *it*"); LUT/SiteRAM/BRAM counts fit the device; worst path on the resulting netlist re-measured against the Phase 3 table.

**Sizing note:** this is the largest single item in the flow. Budget accordingly and don't let steps 1 and 3 wait on it.

## Step 3: Carry as netlist-time geometry (closes the CARRY4 gap)

**Goal:** the bel step stops discovering illegal tile-spanning carry chains because the chains are already legal by construction.

- Represent carry chains as relative placements at mapping time: when `oxibit-synth` emits a CARRY4 chain, emit it with site-relative geometry (chain member *i* at CARRY4 site offset *i* within the same slice column) derived from the device model's site coordinates. Placement then inherits a shaped chain and only has to *place the shape*, not validate it.
- The device data is already in the prjxray layer (site coordinates, CARRY4 positions within sites, column structure). This is representation work, not new analysis.
- **Fallback if time-boxed:** map carry chains to LUTs (a no-carry policy for chains that would span), measure the timing cost on picoBEAM, and only then decide whether true CARRY4 chain placement is worth finishing this phase. The netlist fits without carry — yosys's reference used CARRY4s and placed fine, so parity is achievable either way, but the LUT-mapped variant must be measured, not assumed equal.
- Acceptance: `-carry` netlists pack and place with 0 unsupported-chain reports; timing table re-run on the carry netlist.

## Step 4: Re-baseline and re-measure

Two things must be settled before any of the above is measured, because the benchmark itself is currently in motion:

- **The EHDL elaboration break:** current picoBEAM source no longer elaborates under current EHDL (`memory write ... in a case arm at or after a binary pattern`). Fix elaboration, re-emit once, and re-baseline *everything* — the Nov 3 emit (which all current numbers use) has design ID `0x0B5E000A` and 4096-bundle code memory, and the design is actively moving. Comparing future synthesis work against a stale emit is how baseline-mismatch bugs (like the packing-mismatch incident in Phase 3) happen again, bigger.
- **The full timing table on the oxibit-synth netlist:** same rows as Phase 3 (both packings), plus total end-to-end runtime including synthesis. This is the phase's headline: at 5.2 s synth + ~141 s place+route vs yosys's 1304 s + the same P&R, the question is not whether the flow is faster — it is by two orders of magnitude — but *at what netlist-quality cost*. State the LUT delta and the worst-path delta honestly, including our STA's known ~5.6% optimism vs the reference's own report.

## Ordering and dependencies

```
Step 0: fix EHDL elaboration, re-emit benchmark, re-baseline Phase 3 table
Step 1: device-aware DSP budget        (independent, smallest, do first)
Step 2: memory fall-through chain      (the long pole; start early)
   -> coordinate with Phase 4 LUT RAM anchor work — shared SLICEM column budget
Step 3: carry as netlist-time geometry (or LUT-mapped fallback, measured)
Step 4: full re-measure, both packings, end-to-end runtime
```

Step 1 is safe and quick. Step 2 is the phase; everything else is scheduling around it. Step 3 can shrink to a measured fallback if Step 2 consumes the calendar.

## Standing acceptance discipline (unchanged)

- One change at a time; verdict rows in `docs/` per item; determinism checks (byte-identical outputs where applicable) before and after; full test suites (oxibit-pnr, oxibit-place, plus new synthesis tests as they appear) green; clippy clean on changed lines; nothing committed until the phase table is written.
- The equivalence check (`oxibit-cec`) is the acceptance gate for every synthesis change: a faster, smaller, fitter netlist that isn't *equivalent* is a bug wearing a win.
- Route time and worst path are guard rails, not just targets: a synthesis change that improves LUT count while degrading the critical path is a regression.
- Targets: 0 unmapped memories, <= 220 DSPs, fitting netlist under defaults, CEC pass, and an end-to-end standalone flow that turns picoBEAM Verilog into a bitstream without yosys — at which point "23x faster P&R" becomes "fastest open toolchain on this silicon, full stop" and we go to the board.

## Results

### Step 0: EHDL elaboration, re-emit, baseline

The break was EHDL refusing a memory write in a `case` arm whose pattern is a binary pattern (`<<0::3>>`): per-arm write enables need each arm's match condition, and binary patterns had none. `lib/hw/compile/elaborate/sequential.ex` now derives it from the pattern's literal segments (each literal segment compares its slice of the subject; `_` segments match anything; the arm also requires no earlier arm matched). Test: `test/mem_write_case_test.exs`, "binary-pattern arms write only on their own match" (iverilog). EHDL suite: 375 tests, 0 failures.

Only six memory writes in picoBEAM sit in such arms: `ment`, `rpid`, `rmeta`, once per engine, all in the idle arm of `lib/pico_beam/rtl/signal_table.ex`. Reading that code, the older netlists (no arm enable on those writes) were benign: the writes land in the first free slot and the slot's valid bit was gated correctly. Not checked on a board.

New baseline: `_emit/xc7/xc7_m5_top.v`, 22,731,970 bytes, sha256 `07bc9558...de2f`. All numbers below are on it.

### Equivalence check flow

`scripts/cec_synth.sh SRC.v OUT.check.json TOP WORKDIR` checks the checking view against the source Verilog (gold: yosys's front end on the source; gate: the checking view with yosys's `cells_sim.v` models). Both sides are flattened and cut at every flop (`expose -evert-dff`, named after the Q wire, which oxibit keeps) and every memory (each `$mem_v2` becomes a named black box, write address and data compared under their enables); the PS7 and MMCM stay black boxes and are cut at their ports. `miter -equiv` pairs the cuts by name, and oxibit-cec proves each compared output. A port on one side only gets a stand-in and is listed, not proven. The flow catches a one-bit LUT INIT mutation on a small test design (FAIL on the affected output).

Gold is the source, not the yosys netlist the plan names: the source is what synthesis must be equivalent to, and the yosys netlist has its memories in BRAM primitives, which the cuts could not pair.

### Step 1: DSP budget

The 488 DSPs were mostly not multipliers. Census of the 2,417 `$mul` cells: 2,175 have an operand of at most 4 bits (already fabric), 110 multiply by a power of two (shifts written as `x * 134_217_728`), 132 multiply two nets. Of the 488 DSPs, 136 went to the power-of-two shifts.

| change | LUTs | DSP48E1 | CARRY4 | FF | CEC |
|---|---|---|---|---|---|
| baseline (new emit) | 51,959 | 488 | 7,465 | 13,421 | (not run) |
| 1a: constant-operand `$mul` left to lowering | 48,302 | 352 | 6,739 | 13,413 | covered by 1b |
| 1b: budget 220, size-first fabric fall-back | 48,168 | 220 | 6,618 | 13,381 | CEC_1B |
| reference: all multipliers in fabric (OXIBIT_DSP=0, with 1a) | 56,320 | 0 | 8,474 | 13,381 | |

- **1a.** A multiplier with a constant operand takes no DSP; lowering folds its partial-product rows against the constant (630 such `$mul`, of which 110 powers of two). LUTs fall too: the DSP mapping had been adding the partial products of shifts.
- **1b.** `DeviceCaps::xc7z020().sites.dsp48e1` (220, from the prjxray tilegrid: 110 DSP tiles) is the budget, less any DSP48E1 the design instantiates. Multipliers are ranked by operand area (largest first, then fewer DSPs, then netlist order) and keep DSPs while they fit; a multiplier that does not fit is skipped and smaller ones may still take what is left. Here 352 were wanted: the 53 four-DSP multipliers (33x23 up to 45x33) keep 212, four two-DSP ones keep 8, and 66 two-DSP multipliers (33x5 to 33x17) go to fabric. LUTs fall again (-134): a 33xN multiplier with small N costs less as fabric than the 48-bit adders that sum its two DSP partials. That says the size-first rule is not the last word: a cost model (fabric LUTs vs DSPs plus partial-sum adders) would keep fewer DSPs and fewer LUTs. Not done here.
- Output byte-identical across two runs (netlist and checking view).
- Tests: `dsp::tests` (constant operand takes no DSP; budget keeps the largest, counting the design's own DSP48E1; unbounded budget maps everything).
- **Pipelining probe: not done.** The DSP mode oxibit emits is combinational (`Dsp48Comb`); absorbing registers needs the registered DSP modes in the packer and pnr first. The budget decision is size-only.
- Step 1 targets: <= 220 DSPs met (220). LUTs within the device: 48,168 of 53,200 (90.5%), met, before the 54 memories of Step 2 are mapped. CEC: checked together with Step 2 (below), on the Step 2 netlist, which contains both Step 1 changes.

### Step 2: memories

The 54 memories in three shapes (per engine, x2):

| shape | memories | why the BRAM pass refused |
|---|---|---|
| 32 deep, 1 to 22 wide, 2 or 3 write ports (mailbox and process tables) | 16 | write ports > 1 |
| several read ports, one write port (heaps, link tables, `dz_vstk`, `mbox_words`, `ser_stk`) | 9 | read ports != 1 |
| async read, 128 / 512 deep (`ctch`, `frames_rets`) | 2 | asynchronous read deeper than 128 |

In every multi-write table the second (and third) write is a clearing sweep under `rst` (`if (pb_eng_rst == 1) mem[swp] <= 0`) next to the working write under `!rst`.

| change | LUTs (logic) | FF | RAM32M / RAM64M / RAM128X1D | RAMB18 / RAMB36 | memories left |
|---|---|---|---|---|---|
| after Step 1 | 48,168 | 13,381 | 80 / 0 / 0 | 49 / 38 | 54 |
| 2a: `memflop`: multi-write memories as registers | 52,868 | 17,669 | 80 / 0 / 0 | 49 / 38 | 22 |
| 2b: `memshare`: exclusive write ports merged first | 48,372 | 13,381 | 110 / 0 / 0 | 49 / 38 | 30 |
| 2c: BRAM pass normalizes: address cut to size, one copy per read port, banked RAM128X1D to 512 deep | 48,372 (+ LUTs of 2c below) | 13,381 | 322 / 96 / 168 | 61 / 46 | 0 |

- **2a `memflop`** (RTL pass, before the checking view): a memory with more than one write port becomes one register per word, named `MEMID[i]` as yosys `memory_map` names them, write ports applied in order (later wins), reads as multiplexer trees. Costs +4,700 LUTs and +4,288 FFs here, which by itself does not fit (99.4% LUTs before the other 22 memories). Kept as the floor for write ports that are not exclusive.
- **2b `memshare`** (RTL pass, before `memflop`): write ports whose enables are never on together become one port (enables ORed, address and data selected by the enable). Exclusivity is proven on the lowered AIG with the cut points free, so it holds in every state: first structurally (one enable's AND tree has a literal the other's has complemented), else by oxibit-cec proving `!(a & b)` on the two cones. All 32 multi-write memories merged (38 ports removed); the structural test alone proved 2 of the 32 (the `rst == 1` compare lowers to different nodes in the two enables), the solver the other 30. `memflop` then has nothing to do.
- **2c BRAM pass** (post-check, `bram::normalize`): address bits above the memory's size are dropped, with the write enable gated off when any is set (a `$mem_v2` write past the end is ignored; a read past the end is x); a memory with several read ports and at most one write port becomes one copy per read port; async reads 129 to 512 deep become RAM128X1D banks with a write-enable LUT per bank and a select LUT per bit. The guard and select LUTs come on top of the logic LUT count above: the whole netlist has 50,277 LUT cells plus 2,344 LUT sites of LUT RAM, about 52,600 of 53,200.
- Tests: `memflop::tests`, `memshare::tests` (reset sweep merges, solver-only case merges, independent enables stay), `bram::tests` (read-port copies, banking and guard counts, bank-select truth table). 25 synth tests pass.
- Not done: the async-read output-register probe (no memory here needed it after 2c), and the grouping predicate. The post-check shapes (copies, guards, banks) are covered by unit tests only, not yet by `models/cosim.sh`, which the BRAM pass's stated evidence names.
- Step 2 targets: 0 unmapped memories met. CEC: CEC_2C. Fit: see Step 3.

### Step 3: carry chains, fit, placement

On the Step 2 netlist the oxibit packer left 60,769 cells unplaced and needed 18,440 slices of 13,300. Two causes: the packer fed nearly every CARRY4 input through a LUT1 (32,749 feed LUTs), and the bel step placed no carry chain at all.

What drives the 26,472 CARRY4 inputs of each kind (census of the Step 2 netlist):

| pin | from another CARRY4 | constant | LUT6 | FF / DSP / RAM | single-use LUT1-5 |
|---|---|---|---|---|---|
| DI | 10,786 (shared) | 5,783 | 1,755 | about 1,600 | most of the rest |
| S | 982 + 150 shared | 1,710 | 3,821 | about 2,000 | most of the rest |

| change | LUT cells | feed LUTs | CARRY4 | slices needed (of 13,300) | cells unplaced |
|---|---|---|---|---|---|
| Step 2 netlist, packer as it was (`--no-compact`) | 83,303 | 32,749 | 6,618 | 18,440 | 60,769 |
| 3a: DI on the bypass pin, `--compact` | 56,154 | 5,600 | 6,618 | 13,681 | 38,951 |
| 3b: exact area mapping (2 passes), neighbour fracturing | 55,960 | 5,656 | 6,618 | 13,500 | 38,956 |
| 3c: `carry_min` 3 -> 8 | 53,930 | 3,198 | 5,437 | 12,877 (96.8%) | 31,990 |
| 3d: chains legalised and laid out | 53,930 | 3,198 | 5,437 | 12,877 | 0 |

- **3a** (`pack/carry.rs`): the CY0 mux takes DI from O5 or from the letter's bypass pin (AX..DX), so DI needs no LUT of its own: it goes on the 5LUT only when a single-use LUT fits beside S (five inputs between them), else the bypass pin (`carry.di_bypass`: 21,813). S takes a LUT6 too now. The pnr side already wrote CY0 for AX (`mux_line`).
- **3b**: exact area recovery in `map` (ABC `if`'s: each cover node takes the stored cut that adds the fewest LUTs, by reference counting, under its required time): 1 pass -242 LUTs, 2 passes -250, depth 15 unchanged. Small: the mapper is not where the gap to yosys is. Fracturing also pairs neighbours with disjoint inputs (one drives the other, or both drive one cell): +352 fractured letters (2,205 -> 2,557). Most small LUTs are S LUTs inside chains.
- **3c**: `carry_min` swept (LUTs / CARRY4): 3: 48,372 / 6,618; 8: 49,185 / 5,437; 12: 50,264 / 4,964; 16: 50,925 / 4,719; 0 (no carry): 71,349 / 0. Every chain row takes a whole slice, so fewer short chains is the cheaper side.
- **3d** (`placer/legalise_spread.rs`, `placer/bels.rs`, `placer/mod.rs`): a chain continues into the slice one row up in the same slice column, through HCLK and BRKH rows (prjxray tileconn: CLB COUT joins the CIN above across `HCLK_CLB` and `BRKH_CLB`, grid step 1 or 2), and stops where the column ends (`Problem::site_up`). Legalisation sites every chain, longest first, on the free run whose middle is nearest its global position, before any slice; the bel step lays each member on its slot in its row's slice. The member's row comes back from the packer's offset by `carry::chain_row` (the offset stays nextpnr's, which `place::import` reads). 855 chains on 5,475 slices, none unsited, 0 collisions.
- **Global placement with chains.** A chain as one point charge of one site left mean slice displacement at 43 tiles; as one charge of its full area, Nesterov stalled (overflow 0.16, max bin utilisation 3.3, lambda 3.4e8) and global wirelength was 3.69 M. As a bar of unit charges one grid row apart (`Problem::footprint`, in the density, its gradient and the overflow measure): global 1.42 M, legal 1.56 M, mean slice displacement 4.8 tiles, chain displacement 2.5. Still about 1.9x the yosys -nocarry netlist's 0.82 M on the same flow, at higher utilisation.
- Tests: `carry::tests::chain_row_inverts_the_offset`, `legalise_spread::tests::a_carry_chain_takes_a_run_up_one_column_before_any_slice`, `a_chain_no_column_holds_is_an_error`. 27 place tests pass.
- Timing gap: the STA's timing graph skips CARRY4s (5,475 ignored), so no path through an adder is timed on a carry netlist; its worst paths there are optimistic.
- CEC of 3b/3c (`exact_passes` 2, `carry_min` 8, now the exploration defaults): running. The check pairs registers by name, and 797 gold outputs have no gate counterpart (among them 64 `procs_states` and 64 `mbox_cnts` words, both engines together: memories merged by `memshare` on the gate side, flops after `memory_map` on the gold side), so those are not yet compared.

### Routing fault found on silicon: clock nets in general routing

Not synthesis, but it blocked every oxibit-routed bitstream: oxibit's route of nextpnr's own placement of picoBEAM faulted on all three LibreSDR boards at every pb_clk from 10 MHz to 1.25 MHz, with every static check (`fasm_equiv`, cell config, BUFG/BUFHCE/MMCM settings, BRAM INITs, `fasm2bit` against the reference bitstream) finding the two FASMs equivalent. Pip kinds the oxibit route used that nextpnr's never did showed `INT_R.CLK1.FAN_BOUNCE5` 67 times: slice CLK pins fed by pb_clk off its GCLK line through GFAN, BYP_BOUNCE, FAN_ALT and FAN_BOUNCE. The same source, so equivalence held; the clock skew broke the flops on silicon. `router.rs` now keeps global clock nets off general routing (`is_fabric_wire`, test `a_clock_may_not_cross_general_routing`). The re-route passes on dr-a, dr-b and dr-c: signature 0xC5182FE4, out0 [55, 55, 10, 5], the reference's.
