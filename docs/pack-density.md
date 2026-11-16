# Pack density: mimohalow on the xc7z020 (2026-11-16)

mimohalow's netlist (synthesised by main, byte-identical to old main's:
42,958 LUTs, 6,290 CARRY4, 130 DSP, 39 RAMB18 + 29 RAMB36) stopped at
`pack --gp` on merged main: `7199 slices, 5974 free slice sites`. This
records why, what changed, and the first route.

## Why it did not fit

| | slices |
|---|---|
| slice sites on the device | 13,300 |
| carry chains (1,510 chains, one slice per CARRY4 row) | 6,486 |
| memory macros (one SLICEM each: 680 shift registers, LUT RAM) | 840 |
| free for slice logic (`Problem::slice_room`) | 5,974 |
| slice logic as packed (order of appearance) | 7,199 |
| LUT bound: 19,650 LUT slots / 4 | 4,913 |
| FF bound: 17,495 FF slots / 4 | 4,374 |

The packer, not the slice model, was the limit. Best-choice is compacted
to the count order-of-appearance packing reaches (`--no-compact` aside),
and that count was 46% over the LUT bound. Order of appearance strands
slices: a slice whose LUTs are full but whose flops are not stays open,
and once the 64-slice window holds such slices of different control sets
(519 sets here), every LUT and every flop of another set opens a slice.
An FF capacity of 8 would not have helped: the FF bound is already below
the LUT bound.

## Changes

1. `slices::pack_dense`: flops first, one control set at a time; LUTs
   without flops then fill free LUT slots anywhere, largest first.
   Reported beside the other packers (`"dense"`).
2. `slices::budget`: best-choice is compacted to order of appearance's
   count while that fills at most `TIGHT_FILL` (95%) of the free sites,
   as before; otherwise to 95% of them, never below `pack_dense`'s count.
   The pack example computes the room before packing.
3. `bels::assign_macros`: a shift register or LUT RAM not in a cluster
   takes D6LUT of the SLICEM legalisation reserved for it. Before, it was
   skipped and left unplaced (673 on mimohalow).
4. `macros::fix_dsp_cascades`: occupied DSP sites were keyed by the
   tile-relative site name and looked up by the device-wide one, so an
   occupant was never seen and cascade cells landed on placed DSPs (two
   collisions on mimohalow). Keyed by `SiteKey` now; a chain's own cells
   do not block it from staying where it is; a displaced DSP takes the
   nearest free site, not the first on the die.
5. The legaliser's error names the sites carry chains and memory macros
   took.

## Verdicts

| design | packer counts (order / dense / budget / best-choice) | placement | route |
|---|---|---|---|
| mimohalow | 7,199 / 4,913 / 5,675 / 5,675 (2.2 s) | 0 unplaced, 0 collisions | 18 passes, 557 s, 2.46 GB, bitstream assembled |
| picoBEAM ms14c (bins 24) | 7,847 / 6,551 / 7,847 / 7,847 | `placed.json` byte-identical to ms14c | not rerun (identical input) |

- Old main on mimohalow t7b (different packer and placer): 19 passes,
  1,055 s. Untimed both times.
- oxibit-place: 27 tests pass, plus 2 for `budget`. Clippy clean on the
  changed files.

## Not done, seen on the way

- Global placement on mimohalow stopped on its runaway guard (iteration
  305, overflow 12.7%); legalisation absorbed it.
- The timing graph ignores CARRY4 (6,486 cells on mimohalow), so timing
  on carry paths, and timing-driven routing, are blind there.
- Hold (no-skew model): negative slack at block RAM data inputs on both
  clocks (worst -0.50 ns data_clk, -0.34 ns axi_clk). The hold pass was
  not run.
- Shift registers four to a SLICEM would free about 600 sites.
- Not on the boards yet.
