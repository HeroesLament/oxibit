# oxibit-place A/B results

Routed comparisons for the packer gain function, LUT fracturing and the
analytic placer, against nextpnr's placement of the same netlist. One
design so far. Numbers are from 2026-11-11.

## Setup

- Design: `libresdr_radio`, build `build_radio_check`, part
  `xc7z020clg400-1`. 1156 LUTs, 723 flops, 10 RAMB36, 3 BUFG, 1 F7 mux,
  the PS.
- Reference: nextpnr 1.0.0 placement of the same packed netlist
  (`~/.cache/oxl/pl/radio.json.gz`), imported with `place::import` and
  legalised with `place::legalise`.
- Router: `oxibit-pnr` example `placed_route`, PERM `a5`, at most 40
  iterations, default options. Same binary and settings for every column.
- Placement: `oxibit-place` example `pack --gp` (macro legalisation, two
  passes of quadratic start + Nesterov + spreading, bel assignment).

Commands (DB = `~/src/openxc7/prjxray-db/zynq7`, META = the extracted
`~/.cache/oxl/meta7.tgz`, SYNTH = the build's `libresdr_radio.json`,
REF = the gunzipped reference):

```
pack --gp [--packer rules|rules+score] [--no-fracture] DB xc7z020clg400-1 libresdr.xdc META SYNTH OUT.json REF
pack DB xc7z020clg400-1 libresdr.xdc META SYNTH OUT.json REF        # reference, imported
placed_route DB xc7z020clg400-1 META OUT.json a5 40
```

## Metrics

- HPWL: cell-accurate half-perimeter wirelength in prjxray tile-grid
  units, over the 2292 nets both placements cover. Constants, clock nets
  and nets of more than 200 cells are left out.
- Routed span: for each net, its distinct wires weighted by nominal
  7-series segment length in tiles (single 1, double 2, quad 4, hex 6,
  horizontal long 12, vertical long 18; site pins and wires within a tile
  0). A long wire tapped partway counts in full. The lengths are nominal,
  not read from the database.
- Pips: routing hops over all nets.
- Heap pops: the router's search effort, summed over iterations.

## Results

Every column routes to zero overused nodes, zero unroutable sinks and
zero pin conflicts, and routes the same 1629 nets.

| | best-choice + fracture | order of appearance + fracture | best-choice, no fracture | nextpnr |
|---|---|---|---|---|
| slices | 225 | 225 | 290 | 316 |
| letters with two LUTs | 261 | 261 | 0 | 714 |
| HPWL | 23554 | 25050 | 24100 | 29118 |
| routed span, tiles | 12711 | 13634 | 13004 | 15419 |
| pips | 21717 | 22245 | 23367 | 24855 |
| LUT input pins | 3819 | 3819 | 4390 | 4161 |
| IMUX wires | 4958 | 4970 | 5553 | 5606 |
| bypass wires | 865 | 743 | 845 | 689 |
| sinks | 5248 | 5214 | 5879 | 5628 |
| router iterations | 7 | 5 | 7 | 6 |
| heap pops | 94.6 M | 89.9 M | 86.0 M | 69.2 M |

Against nextpnr, best-choice + fracture: HPWL 0.81, routed span 0.82,
pips 0.87, heap pops 1.37.

### Packer A/B: the gain function

Best-choice (`rules+score`) against order of appearance (`rules`), both
with the same fracturing and the same placer, so what is measured is the
gain function alone:

| | best-choice | order of appearance | ratio |
|---|---|---|---|
| HPWL | 23554 | 25050 | 0.94 |
| routed span | 12711 | 13634 | 0.93 |
| pips | 21717 | 22245 | 0.98 |
| router iterations | 7 | 5 | |
| heap pops | 94.6 M | 89.9 M | 1.05 |

Best-choice wins on every wirelength metric and costs two more router
iterations and 5% more search. Placement-free, at equal slice count, it
cuts 31% fewer net crossings (1238 against 1788).

### Fracturing

Best-choice with and without two LUTs to a letter: LUT input pins fall
13% (4390 to 3819), pips 7%, routed span 2%, slices from 290 to 225.
Router search does not fall with it (86.0 M to 94.6 M pops). That cost
is real and is not explained by fracturing itself; see below.

## Router search effort

Heap pops by iteration, with the overused nodes left after each:

| iteration | nextpnr pops | nextpnr overuse | ours pops | ours overuse |
|---|---|---|---|---|
| 1 | 46.6 M | 2081 | 61.5 M | 1909 |
| 2 | 16.5 M | 609 | 24.6 M | 554 |
| 3 | 5.2 M | 71 | 7.7 M | 64 |
| 4 | 0.85 M | 2 | 0.46 M | 24 |
| 5 to 7 | 0.01 M | 0 | 0.38 M | 0 |

("ours" is best-choice + fracture.)

- Our placement starts with less overuse than nextpnr's and keeps less
  after every one of the first three iterations. The extra search is not
  congestion.
- 15 M of the 25 M gap is in iteration 1, where every net is routed once
  with no congestion history. In both runs the 100 costliest nets take
  over 90% of that iteration's pops (42.0 M of 46.6 M for nextpnr, 57.3 M
  of 61.5 M for ours), and the difference between those two top-100
  totals is the whole iteration-1 gap.

### Iteration 1 by what the nets connect

Pops of iteration 1 summed by the tile type of each net's source and of
its sinks (slices of either kind as CLB; `PSS2` is the PS). The router
reports this per iteration as `pops_by_ends`. This is the before
picture for macro placement.

| source > sinks | nets | sinks | nextpnr pops | ours pops |
|---|---|---|---|---|
| BRAM > PS | 64 | 64 | 28.82 M | 43.75 M |
| CLB > CLB + PS | 99 | 290 / 268 | 12.12 M | 12.67 M |
| PS > PS | 14 | 14 | 1.49 M | 1.49 M |
| BUFG > everything (2 nets) | 2 | 295 / 257 | 1.72 M | 1.71 M |
| CLB > BRAM + CLB | 67 | 860 / 826 | 1.05 M | 0.67 M |
| CLB > CLB | 949 | 3025 / 2739 | 0.50 M | 0.44 M |
| CLB > PS | 33 | 33 | 0.33 M | 0.24 M |
| CLB > BRAM | 72 | 112 | 0.21 M | 0.24 M |
| all other | 329 | | 0.34 M | 0.31 M |
| total | 1629 | | 46.58 M | 61.52 M |

(Where two sink counts are given they are nextpnr's and ours: fractured
letters differ.)

- The 64 single-sink nets from block RAM outputs to PS pins take 62% of
  nextpnr's iteration 1 and 71% of ours. The difference on that one row,
  14.9 M pops, is the whole iteration-1 gap (14.9 M).
- Every net with a PS sink together is 44.0 M of nextpnr's 46.6 M and
  59.1 M of our 61.5 M. Routing to the PS dominates the router's work on
  either placement; that part is a router matter (`Opts::entry_aim`
  describes it: a PS pin's node runs across the PS tiles, far from its
  interconnect).
- Slice-to-slice nets, 949 of the 1629, cost half a million pops, and
  ours cost less than nextpnr's.

Conclusion at this point: the search gap is on the block RAM to PS
nets. It is not slice density and not the packer. The next subsection
tests whether the distance those nets cover explains it. It does not.

### PS pin positions: the check and the fix

The placer had every PS pin at the PS site's one tile, (32, 51). The
64 block RAM to PS sinks enter the PS from interconnect at x 51 to 52,
y 94 to 103: 66 tiles from that point (`placed_route` reports this per
PS sink as `ps_sinks`). Mean distance from the driving block RAM to
the entry: 77.5 tiles in our placement, 44.0 in nextpnr's.

Fix: `oxibit-pnr` example `ps_entries` writes, for each PS wire, the
interconnect tile it is entered or left through; `pack --pins FILE`
gives those to the placer as the positions of the PS's pins
(`placer::PinMap`, 3187 pins), inside the quadratic and Nesterov
solves. The reported HPWL uses the same positions, for both placements.

| | ours, PS pins at the site tile | ours, PS pins at their entry | nextpnr |
|---|---|---|---|
| HPWL, PS pins at their entry | 24413 | 18682 | 28219 |
| routed span, tiles | 12711 | 11445 | 15419 |
| pips | 21717 | 21299 | 24855 |
| router iterations | 7 | 5 | 6 |
| overuse after iteration 1 | 1909 | 1597 | 2081 |
| heap pops | 94.6 M | 87.7 M | 69.2 M |
| block RAM to PS entry, mean tiles | 77.5 | 11.4 | 44.0 |
| BRAM > PS pops, iteration 1 | 43.75 M | 43.93 M | 28.82 M |
| macro displacement at legalisation, mean tiles | 40.7 | 35.8 | |

Against nextpnr with the pin map: HPWL 0.66, routed span 0.74, pips
0.86, heap pops 1.27.

These figures supersede the 0.81 / 0.82 / 0.87 of the Results table.
HPWL here is computed with PS pins at their entry, for both
placements, and is not comparable with the Results table, which had
them at the site tile. Every later table uses the pin map (`--pins`),
so the series is comparable from here on.

The pin model was wrong and fixing it improved every wire metric and
the iteration count. It did not move the BRAM > PS row: those 64 nets
cost the same search from 11 tiles away as from 77.

### Macro displacement

How far hard macros land from where the first placement wanted them,
with the pin map:

| type | cells | mean tiles | max tiles |
|---|---|---|---|
| BUFGCTRL | 3 | 111.3 | 120.0 |
| RAMB36E1 | 10 | 13.2 | 22.7 |
| all | 13 | 35.8 | 120.0 |

- The 35.8 mean (40.7 before the pin map) is mostly the three clock
  buffers, measured from the cell driving their input to the buffer
  column. Their nets are clock nets, outside HPWL, so that distance is
  not a wirelength error.
- Block RAMs are 13.2 tiles off on average. Tested whether that costs
  anything once the slices are placed: `pack --macro-rounds N` moves
  each macro to the free site of its type that most shortens its own
  nets, then places the slices again (`macros::refine`). With the pin
  map no macro moves: each block RAM is already on the best free site
  for its nets as placed. Placement and route are identical with and
  without it. The test is single moves only; swaps and joint moves are
  not tried.
- Block RAM pins, measured on one tile (`ps_entries ... BRAM_L_X22Y50/`):
  outputs meet the interconnect 2 columns to one side over 5 rows (0 to
  4 rows from the tile's coordinate), inputs 1 column to the other side
  over mostly the same 5 rows (30 of 631 input wires are 6 to 8 rows
  off). So nearly all of a block RAM's pins are within about 4 tiles of
  where the placer has them, against 66 for the PS. Not
  modelled: a per-pin map needs the bel pin to routing wire mapping
  (not a naming rule: data outputs split between two wire groups), and
  the mirrored tile type was not measured.

### Known router property: searches for PS sinks

Out of scope for the placement sprint; recorded so router work starts
from the evidence. The `pops_by_ends` diagnostic is what made it
visible.

- Nets with a PS sink take 44.0 M of 46.6 M iteration-1 pops on
  nextpnr's placement and 59.1 M of 61.5 M on ours (before the pin
  map).
- For the costliest single-sink nets in iteration 1 (the router's
  `worst40`), with the pin map: the route found is 10 hops, cost 10.5,
  while the router's own distance to where it aims is 74 tiles and its
  starting estimate 44. On nextpnr's placement: 11 hops, aim distance
  59, estimate 35, and fewer pops. On ours before the pin map: 14 hops,
  aim distance 53.
- In all three the starting estimate is three to four times the cost
  of the route found (35 against 11.5, 32 against 15, 44 against 11):
  the search is aimed at the PS pin's own node (`Opts::entry_aim` off),
  tens of tiles from where the pin is entered. The cost of these nets
  does not follow the distance the route covers (same pops at 11 tiles
  as at 77). It does not simply follow the aim distance either
  (nextpnr's placement has the middle aim distance and the fewest
  pops), so what sets it is not yet known.
- Consequence for this document: heap pops are not a placement-quality
  measure while most of them are PS-sink searches. Iterations, overuse
  and wire use are.

## Noise, and what it does to the tables above

The placer is deterministic but sensitive: moving two atoms between
slices changed HPWL by 5%. So every arm was re-run at seven bin-grid
sizes (`--bins 29` to `35`; 32 is the default), which perturbs the
placement without changing the problem. HPWL with the pin map,
best-choice + fracture unless said:

| arm | 29 | 30 | 31 | 32 | 33 | 34 | 35 | median | vs baseline |
|---|---|---|---|---|---|---|---|---|---|
| baseline (compacted to 225 slices) | 19356 | 19528 | 19754 | 18682 | 19008 | 19301 | 21503 | 19356 | |
| order of appearance | 21396 | 21472 | 21232 | 19852 | 21218 | 20787 | 23584 | 21232 | +9.7% |
| no compaction (268 slices) | 17446 | 16930 | 17650 | 17066 | 16709 | 19607 | 17391 | 17391 | -10.2% |
| distribution breaking, limit 16 | 19196 | 19491 | 19835 | 19575 | 18728 | 19307 | 20112 | 19491 | +0.7% |

- Spread within an arm is about 5% either side of its median, with one
  high outlier in most arms. A single-run difference under about 5% is
  not a result.
- The default grid (32) happens to give the baseline its best run of
  the seven. The figures quoted earlier in this document are single
  runs at 32: against nextpnr's 28219 the baseline is 0.66 at 32 and
  0.69 at the median.
- Routed metrics have not been swept this way (20 s a route). The
  routed tables above are single runs.

### Packer A/B, restated

Best-choice beats order of appearance at every one of the seven grids;
median HPWL 0.91 of it. The placement-free cut (31% fewer crossings)
does not depend on the placer and stands as is.

### Distribution breaking: no effect, removed

Design tried: while a slice reads more than a limit of nets from
outside itself, move out the atom whose leaving lowers that most, into
the slice it shares most with (if that stays within the limit) or a
slice of its own. Same netlist, fracturing, pin map and compaction
budget.

| | baseline | limit 16 | limit 12 |
|---|---|---|---|
| atoms moved | 0 | 2 | 31 |
| slices | 225 | 226 | 240 |
| most external inputs on a slice | 20 | 16 | 12 |
| HPWL, median of 7 | 19356 | 19491 | not swept |
| HPWL at 32 | 18682 | 19575 | 19439 |
| routed span at 32 | 11445 | 11886 | 11506 |
| pips at 32 | 21299 | 21623 | 21413 |
| router iterations | 5 | 5 | 5 |
| overuse after iteration 1 | 1597 | 1741 | 1720 |

Nothing moves beyond noise, and nothing moves the right way: the
router converges in the same five iterations with or without it. On
this design best-choice already leaves only 2 of 225 slices above 16
external inputs (order of appearance leaves 28). The code is not kept.
The per-slice external-input figures stay in the packer's report
(`ext_in`), so a design that does have dense slices will show it.

### Control-set matching: nothing to match, not built

The design has 3 distinct control sets among its flops, and 3 after
making equivalent ones one (enable tied high, reset tied inactive,
sync flag without a reset: `Ctrl::normalised`). Slices are at 225
against a floor of 224 from LUT slots alone, so flop control sets do
not bind anywhere. The packer reports both counts (`ctrl_sets`,
`ctrl_sets_merged`); matching is worth building when a design shows
them differ.

### Compaction: costs wirelength, and is ours to drop

Not one of the planned items; found while isolating the above.
Best-choice's own merges give 268 slices. Compacting them to the 225
order of appearance reaches (a rule adopted here so the packer A/B
compared equal slice counts) costs 10% HPWL at the median, and at 32
it costs 10% routed span (11445 against 10244) and 2% pips, for one
fewer router iteration (5 against 6). No compaction is better at six
of the seven grids. The device has 13300 slices; nothing needs the 43
back. `--no-compact` selects it; it is not yet the default.

## Parameters

| parameter | value | source | swept |
|---|---|---|---|
| net weight in the score | 1 / pins | Alpert et al., ISPD 2005 | no |
| score fanout cap | 64 pins | ours | no |
| physical term weight, distance | 0.1, 4.0 | ours | no (no positions fed yet) |
| fracture threshold | 1 shared input | ours | no |
| slice capacity | 4 LUT slots, 4 FF slots | device | n/a |
| initial placement rounds | 20 | RePlAce release docs | no |
| placement fanout cap | 200 | RePlAce release docs | no |
| target overflow | 0.1 | RePlAce release docs | no |
| lambda multiplier bounds | 0.95, 1.05 | RePlAce release docs | no |
| gamma schedule, backtrack 0.95, lambda start | see `nesterov.rs` | ePlace, recalled, not yet checked against the paper | no |
| lambda wirelength reference | 0.02 of current HPWL | ours | no |
| bins | 32 a side | ours | no |
| PS pin positions | per pin, from `ps_entries` | routing graph | n/a |
| distribution breaking | limit 16 and 12 tried; no effect; removed | ours, by outcome | 2 points, sweep not pursued |
| control-set matching | not built: 3 sets, none mergeable | ours, by outcome | n/a on this design |
| compaction budget | order-of-appearance slice count; `--no-compact` for none | ours, by outcome | 2 points (on, off) |
| macro refinement rounds | 0 (diagnostic) | ours | 0 and 4: identical |
| target density | 1.0 | ours | no |

Apart from the points noted, nothing has been swept. Every number above
is a first value.

## Bugs found by routing

1. A flop fed through AX on letter A of the slice holding an F7 mux: AX
   is the mux select. Fixed in `placer/bels.rs`; regression test
   `a_bypass_flop_stays_off_letter_a_beside_an_f7_mux`.
2. The packer's capacity check did not count letter A's FF as out of
   reach in a mux slice, so order of appearance seated four bypass flops
   where three fit. Fixed in `pack/slices.rs` (`Atom::blocks_ff`);
   regression test
   `packing_seats_no_more_bypass_flops_beside_an_f7_mux_than_it_has_ffs_for`.
3. The two leaf LUTs of an F7 mux were packed into different slices.
   Fixed by tying them (`PackCtx::ties`).

## Correction to earlier reference numbers

Reference figures quoted before this document (21909 pips, 13781 span,
62.7 M pops, 1405 nets) were from an incomplete route. The reference
names bels by absolute site, `place::legalise` only recognises
fractured letters in grid form, so its 714 5LUTs kept an O6 port and the
router skipped their 714 nets as undriven. The `pack` example now
converts the reference to grid form before importing it. HPWL figures
were not affected.

## Limits

- One design, small against the device (225 of 13300 slices).
- No timing: nothing here says whether the shorter routes are faster.
- Clock nets are in the router's totals for every column but outside
  HPWL.
- DSP, F8 trees, carry chains, distributed RAM and shift registers are
  not exercised: the design has none, and the placer reports the last
  four as unsupported.
