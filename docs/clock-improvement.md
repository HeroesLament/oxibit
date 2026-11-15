# Oxibit Clock Improvement Strategy

Status: Phases 1, 2 and 3 landed (uncommitted). Phase 3 closed at
54.36 ns on pb_clk (target about 53 to 54); see "Phase 3: timing-weighted
placement" below. Phase 4 (LUT RAM placement, unresolved pins, first hold
report): 54.31 ns, router exit 0; see "Phase 4" below.
Measurements come from `oxibit_pnr::sta`: `standalone` prints them after
every route, the `sta` example does the same for a FASM from elsewhere.

## Results so far

picoBEAM `xc7_m5` (nocarry), oxibit placement (`/tmp/pbF-placed.json`),
oxibit routing, one run per row (the router is deterministic; nothing here
is swept for noise). Periods given to the router: pb_clk 50 ns, axi_clk
8 ns (nextpnr's own result).

| run | pb_clk worst path | axi_clk worst path | iterations | route time |
|---|---|---|---|---|
| no timing (baseline) | 74.94 ns | 8.64 ns | 7 | 55 s |
| `delay_weight` 0.3, no criticality | 70.11 ns | 8.63 ns | 8 | 81 s |
| timing, tau 1, 6 passes, min_rise 0.05 | 65.80 ns | 8.78 ns | 7 | 59 s |
| timing, tau 3, 12 passes, min_rise 0.02 (defaults) | 62.38 ns | 8.64 ns | 11 | 62 s |
| timing, tau 6, 12 passes, min_rise 0.02 | 62.61 ns | 8.56 ns | 14 | 63 s |
| timing, tau 3, `astar_delay` 0.1 | 63.12 ns | 8.69 ns | 12 | 68 s |

- Phase 2 criteria met by the defaults row: 12.6 ns better on pb_clk
  (target 10), axi_clk unchanged (target under 3% worse). Every row is
  legal: 0 shorts, 0 missing sinks.
- The last three rows sit within 0.8 ns of each other: more weight on
  delay no longer helps. 12.3 ns of the gap to nextpnr (50.1 ns)
  remains. The baseline table attributes 6 ns to placement, so about
  6 ns of the router's 18.7 is not recovered.
- The baseline row, measured in-process from the routes, agrees with the
  FASM-based figure in the table below (74.945) to 3 ps.

### Router variants after the plateau

Same setup. One change per row against the row it names.

| run | change | pb_clk worst path | axi_clk | iterations | route time |
|---|---|---|---|---|---|
| e1 | defaults + `min_rise` -1 (re-rip every failing net each pass), `discount` 0.9, `hot_max` 1500 | 61.49 ns | 8.74 ns | 19 | 76 s |
| e2 | defaults + `exact` 0.8 (critical searches near cheapest-path) | 60.82 ns | 8.68 ns | 9 | 113 s |
| e3 | e1 + e2 | 60.98 ns | 8.74 ns | 21 | 177 s |
| d9 | defaults + `discount` 0.9 | 61.93 ns | 8.67 ns | 16 | 63 s |
| d5 | defaults + `discount` 0.5 | 62.62 ns | 8.68 ns | 12 | 66 s |

All within about 1.6 ns of the defaults (62.38). Single runs: the
router is deterministic, but any change reshuffles the whole route, and
the tuning rows above already spread over 0.8 ns, so differences of
about 1 ns are not distinguishable from that.

### Path search: `Opts::span_aware`

The routing graph placed every node at one tile of its wire
(`Graph::node_xy`), whichever came last in tile order. For a long line
(18 tiles) or a hex (6) the A* distance to the target was then off by up
to the wire's length, either way: a long line heading for the target
could look like no progress, and one whose recorded tile was the target
like arrival. `Graph::node_box` now holds each node's span, and with
`span_aware` the distance is measured from the nearest tile of it.

| run | placement | timing | pb_clk worst path | axi_clk | iterations | route time | long-line entries |
|---|---|---|---|---|---|---|---|
| base | oxibit | off | 74.94 ns | 8.64 ns | 7 | 55 s | 2060 |
| s0 | oxibit | off | 70.37 ns | 8.96 ns | 8 | 93 s | 3296 |
| def2 | oxibit | defaults | 62.38 ns | 8.64 ns | 11 | 59 s | 2034 |
| s1 | oxibit | defaults | 62.92 ns | 8.84 ns | 10 | 100 s | 3237 |
| nt | nextpnr | defaults | 56.20 ns | 8.82 ns | 143 | 153 s | 4442 |
| nts | nextpnr | defaults | 54.08 ns | 8.94 ns | 12 | 149 s | 6166 |
| (nextpnr's own route) | nextpnr | | 50.1 ns (52.96 its report) | | | | 6960 |

Long-line entries: pips whose destination is an `LV`/`LH` wire
(`/tmp/longlines.sh`). Timing alone never changed that count (2060 to
2034): the delay term cannot steer towards wires the search does not
see, which is why the cost knobs plateaued.

The six worst-path connections of `nt` against nextpnr's routing of the
same placement (`target/show-cmp.txt`; one line per connection: ps,
hops, wires entered):

| connection | nextpnr | nt | nts |
|---|---|---|---|
| 92 tiles | 1818 ps, `WW4 LV18 LV18 LV18 LH0 SW6 SL1` | 3329 ps, 20 hops of hex and doubles | 1996 ps, `WW4 LV18 LH0 LV18 LV18 SW6 SL1 BYP` |
| 2 | (other pin) | 2095 ps | 1513 ps |
| 3 | 791 ps | 1756 ps, `NR1 LV_L18 NE6`: a long line entered and left in one tile | 1464 ps, no long line |
| 4 | 138 ps | 138 ps | 138 ps |
| 5 | 983 ps | 1620 ps, two BYP bounces at the sink | 1379 ps |
| 6 | (other pin) | 865 ps | 559 ps |

- Acceptance met: long lines on the 92-tile connection, no same-tile
  bounce.
- Convergence on the foreign placement: 143 iterations to 12. Poor
  search guidance was compounding negotiation, as suspected.
- On oxibit's own placement span_aware helps untimed routing by 4.6 ns
  but adds nothing on top of timing (s1 vs def2, within the 1 ns band
  above). On nextpnr's it adds 2.1 ns. Not explained yet.
- Route time on oxibit's placement is up 60 to 70% (heap pops only about
  20%); on nextpnr's it is flat because iterations fell.
- Still there: destination detours. The 92-tile connection arrives on
  `SW6` track 0 and needs a `BYP` bounce to reach `IMUX2`, where
  nextpnr arrives on track 1; connection 3 still overshoots west and
  walks back.

### Search cost: `Opts::skip_stale`, `Opts::detour`

`Stats::heap_pushes` now counts pushes. span_aware doubled them (0.66 B
to 1.36 B, pushes per pop 4.6 to 8.0): long lines and hexes are popped
more, and each pushes a large fan-out in every direction.

- `skip_stale`: drop a popped entry whose node was reached more cheaply
  since it was pushed. Bit-identical FASM with and without it, on both
  `base` and `s0` (`cmp`). Saves only about 4% of pops and no time: stale
  entries were rare. Left on in the runs below; changes no route.
- `detour (factor, margin)`: push a node only if its distance to the
  target is at most factor x the least distance popped so far plus
  margin tiles; a search that finds nothing so runs again unbounded.

| run | change | pb_clk worst path | heap pushes | iterations | route time |
|---|---|---|---|---|---|
| base | none | 74.94 ns | 0.66 B | 7 | 55 to 59 s |
| s0 | span_aware | 70.37 ns | 1.36 B | 8 | 93 to 95 s |
| s0b | s0 + detour (1.5, 6) | 69.13 ns | 0.91 B | 8 | 83 s |
| s1 | span_aware + timing | 62.92 ns | | 10 | 100 s |
| s1b | s1 + detour (1.5, 6) | 61.88 ns | 0.99 B | 15 | 90 s |
| s0c | s0 + detour (1.2, 3) | 73.36 ns, not legal | 0.78 B | 200 (cap), 151 overused, 2340 shorts | 124 s |
| s0d | s0 + detour (1.0, 2) | 79.64 ns, not legal | 1.09 B | 200 (cap), 1334 overused, 17359 shorts | 214 s |
| ntb | nextpnr placement, timing, discount 0.9, detour (1.5, 6) | 53.23 ns, not legal | 1.31 B | 200 (cap), 1 overused, 5 shorts | 136 s |
| ntbx | ntb + exact 0.8 | 53.41 ns, not legal | 1.37 B | 200 (cap), 4 overused, 27 shorts | 162 s |

At (1.5, 6) the bound cuts a third of the pushes at no cost in delay
and converges as unbounded does (8 iterations, 1664 searches fell back
to unbounded); route time is still about 40% over `base`. At (1.2, 3)
routing fails: a tight bound still finds a path, only a congested one,
so it never falls back, and negotiation cannot move nets around a hot
spot. The bound cannot be tightened past what congestion detours need.
On nextpnr's placement, which is more congested, even (1.5, 6) fails
the same way (`ntb`, `ntbx`; the same run unbounded, `ntsd9`, converges
in 17). `Opts::detour` therefore now bounds only a net's first route;
every reroute after congestion searches freely.

| run | change | pb_clk worst path | heap pushes | iterations | route time |
|---|---|---|---|---|---|
| ntb2 | ntsd9 + detour (1.5, 6), first route only | 53.68 ns | 1.81 B | 16 | 153 s |
| s0b2 | s0 + detour (1.5, 6), first route only | 70.48 ns | 1.16 B | 7 | 122 s (see below) |

First-route-only converges on both placements but saves no time on
nextpnr's (153 s against `ntsd9`'s 148 s). `s0b2` ran while the link to
the machine was down; if it slept, the wall time is not usable, though
its push count is. Verdict: the bound has no demonstrated benefit once
made safe. Kept, off by default.

### Congestion discount, re-tested after span_aware

`TimingView::discount` 0.9 (a critical sink pays 10% of the congestion
cost at criticality 1), with span_aware, skip_stale and timing on:

| run | placement | pb_clk worst path | iterations | route time | vs |
|---|---|---|---|---|---|
| sd9 | oxibit | 61.03 ns | 19 | 112 s | s1 62.92 ns |
| ntsd9 | nextpnr | 53.51 ns | 17 | 148 s | nts 54.08 ns |

The six connections of `ntsd9` against `nts`:

| connection | nextpnr | nts | ntsd9 |
|---|---|---|---|
| 92 tiles | 1818 ps | 1996 ps, ends `BYP_ALT0 BYP_BOUNCE0` | 1996 ps, same |
| 2 | (other pin) | 1513 ps | 965 ps |
| 3 | 791 ps | 1464 ps, overshoots west | 650 ps, `SS6 SS6 SS2` |
| 4 | 138 ps | 138 ps | 138 ps |
| 5 | 983 ps | 1379 ps | 1404 ps |
| 6 | (other pin) | 559 ps | 533 ps |

Connection 3's detour is gone (650 ps, under nextpnr's own), connection
2 drops by a third. The aggregate gain (1.9 and 0.6 ns) is at the edge
of the 1 ns band, so this is weaker evidence than the per-connection
rows. The 92-tile connection's last-switchbox bounce is unchanged, so
it is probably not congestion (with discount 0.9 a critical sink barely
pays for it). The likelier cause: its track is chosen at the `SW6` two
hops out, and the greedy estimate (`per_tile` 0.6, about 11x the
admissible bound) scores both tracks the same there, so the first one
popped wins. `TimingView::exact` is the knob for that; re-test it.

Re-tested, with span_aware, skip_stale, timing and discount 0.9, no
detour bound:

| run | change | pb_clk worst path | iterations | route time | vs |
|---|---|---|---|---|---|
| ntsd9x | ntsd9 + exact 0.8 | 53.99 ns | 13 | 202 s | ntsd9 53.51 ns, 148 s |

No gain, 37% slower, and the 92-tile connection got worse (2531 ps).
Its 1815 ps in `ntbx` was incidental. `exact` stays off.

### Defaults after the path-search work

`Opts::span_aware` is now on by default (timed and untimed routing; the
60% route-time cost is accepted) and `TimingView::discount` defaults to
0.9. `skip_stale`, `detour` and `exact` stay off. With these defaults
the timed configuration is the one `ntsd9` and `sd9` ran, so those are
the controls below.

### Bidirectional connection search: `Opts::bidir`

Each connection is searched from both ends: the forward front as
before, and a backward front from the sink over reversed edges, each
step costing what the forward search pays to enter the node, aimed at
the tree's bounding box. The fronts alternate one pop each. A
connection whose fronts never meet, or whose halves would form a loop,
is searched again forward (counted as a fallback).

| run | change | pb_clk worst path | iterations | route time | heap pops | heap pushes | vs |
|---|---|---|---|---|---|---|---|
| ntbd (first) | meet where one front pops a node the other has only reached | did not converge: 7718 overused at iteration 141, stopped | | 640 s+ | | | |
| ntbd | meet where both fronts have popped the node | 54.30 ns | 36 | 113 s | 88 M | 0.96 B | ntsd9 53.51 ns, 17, 148 s, 259 M, 2.0 B |
| sbd | as ntbd, oxibit placement | 62.97 ns | 19 | 73 s | 61 M | 0.68 B | sd9 61.03 ns, 19, 112 s, 192 M, 1.5 B |

The first version took the first node one front popped that the other
had merely generated, at whatever provisional cost, so its paths
ignored congestion and negotiation never settled. Meeting on nodes both
fronts have settled fixed that. 5723 connections fell back to forward
search.

The six connections (nextpnr / `ntsd9` / `ntbd`):

| connection | nextpnr | ntsd9 | ntbd |
|---|---|---|---|
| 92 tiles | 1818 ps | 1996 ps, ends in a `BYP` bounce | 2049 ps, `LV_L18` x3 + `LH0`, no bounce |
| 2 | (other pin) | 965 ps | 1347 ps |
| 3 | 791 ps | 650 ps | 911 ps |
| 4 | 138 ps | 138 ps | 138 ps |
| 5 | 983 ps | 1404 ps | 775 ps |
| 6 | (other pin) | 533 ps | 533 ps |

Verdict: no delay gain. 0.8 ns worse on nextpnr's placement (inside
the 1 ns band; connections move both ways) and 1.9 ns worse on ours
(at its edge). It is cheaper: a third of the heap pops and 24 to 35%
less route time. Kept, off by default; a speed option, not a delay
one.

### Post-route cleanup: `Opts::cleanup_rounds`

After a legal timed route, each round re-times (fresh slack), takes
every sink with negative slack, most critical first, rips up only its
own branch, and searches it again for delay alone (`CLEANUP_ASTAR`, a
near-lower-bound estimate) through nodes no other net occupies. The new
branch is kept only if it arrives earlier and overlaps nothing, so no
connection gets slower and legality holds. A round that keeps nothing
ends the pass.

| run | change | pb_clk worst path | cleanup |
|---|---|---|---|
| ntc | ntbd + 3 cleanup rounds (bidir on by accident, see below) | 54.20 ns (54.30 before) | 20009 sinks tried, 433 kept, 77.8 ns of arrival saved in all, worst slack -4.296 to -4.198 ns, 8 s |
| ntc2 | defaults + 3 cleanup rounds | 53.30 ns (53.51 before, `ntsd9` exactly) | 22333 tried, 253 kept, 55.2 ns saved in all, worst slack -3.508 to -3.300 ns, 11 s |

`ntc` was meant to be the defaults plus cleanup. A Ctrl-C had stopped
the chain that exported `OXIBIT_BIDIR=1` before its `unset`, so it ran
with bidirectional search on. Its cleanup numbers are still a fair
measure of the pass on a legal route.

`ntc2`'s pre-cleanup slack equals `ntsd9`'s to the ps: the new
defaults reproduce that control, and the router is deterministic.

Only 1 to 2% of failing sinks found a faster branch through free
wires, and the worst path moved 0.1 to 0.2 ns. Verdict: correct,
monotone and cheap (8 to 11 s), but small. Kept, off by default. The routes are already close to the
fastest path the free resources allow; what is left is contention for
the fast wires themselves, which a pass that only uses free wires
cannot touch.

### Where the router stands

On nextpnr's placement every variant since span_aware lands between
53.2 and 54.3 ns (`nts`, `ntsd9`, `ntsd9x`, `ntb2`, `ntbd`, `ntc2`),
3 to 4 ns behind nextpnr's own 50.1, and inside the 1 ns band of one
another. The cleanup result says the remaining loss is not detours a
local re-search through free wires can remove: it is which net holds
the fast wires. That is a
second plateau. span_aware was the one change with a clear, structural
effect (long lines used, convergence from 143 iterations to 12);
discount 0.9 has per-connection evidence but no clear aggregate one.
On oxibit's placement the best legal route is 61.0 ns (`sd9`), so the
placement gap (about 8 ns) is now twice the router's.

Reproduce a row:

    OXIBIT_TIMING="pb_clk=50000,axi_clk=8000" [OXIBIT_TAU=.. OXIBIT_TIMING_PASSES=.. OXIBIT_MIN_RISE=..] \
      ./target/release/examples/standalone ~/src/openxc7/prjxray-db/zynq7 xc7z020clg400-1 /tmp/oxl-meta PLACED.json OUT.fasm

## Phase 3: timing-weighted placement

`oxibit_place::placer::timing`. The placer builds an `sta::Design` from
its own cells (`Design::from_cells`; arcs per net and sink cell, no
routing behind them) and, every `every` Nesterov iterations, sets each
arc's delay from the Manhattan distance between its cells' current
positions (`wire_ps`: a piecewise-linear table fitted to routed arcs of
`nts`, about 54 ps per tile to 20 tiles, 33 to 40 beyond), runs
`Timer::analyze`, and multiplies each net's wirelength weight by
`1 + beta * rel^exp`. `rel` is `SlackTable::net_rel`: the slowest path
through the net over its clock's slowest path (VPR's relative
criticality), so a path need not fail to be weighted. Defaults beta 4,
exp 4. Every clock is given a 1 ms period in the placer so every path
has a `rel`; the real periods only matter to the router.
`OXIBIT_PT=EVERY[,BETA[,EXP]]` on the `pack` example; unset or 0 is off
and leaves the placement byte-identical to before.

All rows: picoBEAM `xc7_m5` nocarry, routed with the defaults (span_aware,
discount 0.9, timing pb_clk 50 ns / axi_clk 8 ns; bidir and cleanup
off). Times are wall clock on the same machine, one run each.

| run | packing | weights | pb_clk worst path | axi_clk | failing nets | place (gp) | route | iterations | INT pips |
|---|---|---|---|---|---|---|---|---|---|
| base | `--no-compact`, 9531 slices | off | 61.029 ns | 8.605 ns | 5820 | 6.4 s | 120 s | 19 | 440153 |
| e10 | `--no-compact` | every 10 iterations | 54.356 ns | 8.205 ns | 3293 | 8.1 s | 115 s | 13 | 451135 |
| e1 | `--no-compact` | every iteration | 54.667 ns | 8.213 ns | 3302 | 29.3 s | 128 s | 13 | 451186 |
| cbase | compacted, 8375 slices | off | 77.163 ns | 8.371 ns | 9676 | | 189 s | 52 | |
| c10 | compacted | every 10 iterations | 62.006 ns | 8.328 ns | 6130 | 9.0 s | 161 s | 79 | 518346 |

- `base` is byte-identical to `/tmp/pbF-placed.json` and routes to
  `sd9`'s 61.03 ns: the control holds. `e10` run twice (`e10`, `fnc`)
  gives identical placement JSON and identical FASM: weighting at a
  fixed iteration cadence is deterministic. `c10` run twice likewise
  reproduces 62.006 ns and the same heap counts.
- Gain on the packing of record: 6.67 ns (61.03 to 54.36), of the
  8 ns placement gap. axi_clk 0.4 ns better, failing nets down 43%,
  router iterations 19 to 13.
- Cost: legal HPWL +6.3% (482534 to 512800), routed INT pips +2.5%,
  placement 6.4 to 8.1 s (1.27x, inside the 1.5x limit). 33 timing
  updates take 0.55 s in all, about 17 ms each.
- Cadence: every iteration is 0.3 ns worse than every 10 (inside the
  noise band) and 4.6x the placement time, of which the 313 updates are
  only 6.2 s: the rest is the gradient recomputed after each reweigh.
  Every 10 stays. Per-iteration retiming is not worth a single-precision
  or incremental timer; the full `analyze` at this cadence costs under
  10% of placement.
- The placer's own estimate of the worst path (62.5 ns at the last
  update of `e10`) is 8 ns over the routed 54.4: the table is fitted to
  arcs of a congested route, so it overstates the uncongested ones. It
  only sets relative weights, so the bias does not matter yet; it would
  for a slack-based exit (step 3).
- Compaction: weights recover 15.2 ns on the 8375-slice packing, but it
  still ends 7.6 ns behind the loose packing (62.0 against 54.4), with
  18% more routed pips and 79 router iterations. Density costs clock
  here; `--no-compact` stays the timing-driven default.

### Critical-path dump

`/tmp/critarcs.sh TAG PLACED` (STA trace of the pb_clk path, `a5` pin
mode, which reproduces the routed figure to the ps), `target/crit-*.txt`:

| run | path | bounding box (tiles) | tile travel | end to end | out-and-back excursions (15+ tiles) |
|---|---|---|---|---|---|
| base | 61.03 ns, 64 levels | 46 x 97 | 602 | 90 | 1 (47 tiles out, 44 back) |
| e10 | 54.36 ns, 71 levels | 51 x 45 | 390 | 56 | 2 |
| c10 | 62.01 ns, 71 levels | 52 x 82 | 760 | 28 | 6 |

The six slowest connections of `e10` (route ps, wires entered):

| step | ps | tiles | wires |
|---|---|---|---|
| 1 | 1300 | 25 | `NN6 NN6 NW6 NN6 NL1`: BRAM out, straight north |
| 19 | 1557 | 10 | 12 hops, `SW2 NW6 NE2 NE2 EL1 ...`: wanders for 10 tiles |
| 23 | 1586 | 6 | `WW2 SS2 SS2` (542 ps on the sink shown; the 1586 is the other pin) |
| 37 | 2070 | 30 | `WW4` x7 west to `CLBLM_R_X5Y38` |
| 38 | 1717 | 34 | `EE4` x5 straight back east |
| 55 | 1457 | 18 | `SW6 SS2 EE2 SS6 SS2 ...` |

Acceptance: not met in full. The path's box halved and its travel fell
by a third, but two out-and-back excursions remain, and both are on
LUT RAM read cells: `pb_eng1_procs_states2/DPR2_1` (steps 37 and 38,
3.8 ns of routing to visit one cell 30 tiles away) and
`pb_eng1_mbox_heads/DPR2_0`. LUT RAMs sit in SLICEM slice macros
(438 of them), placed as a whole and legalised onto the sparse SLICEM
columns, so the timing weight on their read nets moves them less than
ordinary LUTs. That is the next placement lever, and a narrower one
than congestion inflation (step 4).

### Final numbers

| flow | packing | pb_clk worst path (our STA) | Fmax | place | route |
|---|---|---|---|---|---|
| nextpnr place + route | nextpnr | 50.1 ns (52.83 its own report) | 19.96 MHz (18.93 its own) | 48.5 s (HeAP) | 3136 s (router2 + router1) |
| nextpnr place + oxibit route | nextpnr | 53.30 ns (`ntc2`; 53.51 without cleanup) | 18.76 MHz | | 148 s |
| oxibit, `--no-compact` + weights | 9531 slices | 54.36 ns | 18.40 MHz | 11.4 s | 129 s |
| oxibit, compacted + weights | 8375 slices | 62.01 ns | 16.13 MHz | 11.9 s | 173 s |

`place` and `route` are wall time of the whole step (`/tmp/oxflow.sh`),
pack included in `place`. `fasm2bit` adds 1.7 s. Synthesis is yosys's
existing `xc7_m5.json`, not rerun: 1304 s wall in its log, 2.5 GB
peak, identical for both flows. nextpnr's times are from its
`pnr.log`.

- Target (end to end at or under about 53 to 54 ns): missed by 0.36 ns
  against 54, 1.36 against 53. Gained 6.67 of the 8 ns placement gap.
- Against nextpnr on the same STA: 8.5% slower clock (54.36 against
  50.1). Our STA reads nextpnr's own route 5.6% under nextpnr's report
  on pb_clk (50.1 against 52.83 to 52.96) and 7% under on axi_clk; if
  that held on our route, nextpnr would time it at about 57.4 ns. That
  is an estimate: nextpnr cannot read our routing, so the cross-check
  on the same routed design was not possible, and the calibration is
  from nextpnr's route only.
- Speed: place and route in 141 s against nextpnr's 3185 s on this
  design, about 23x.
- Router gap on the new placement: not measured. It needs nextpnr to
  route oxibit's placement, which this flow cannot hand it. On
  nextpnr's placement it stands at 3.2 ns (53.30 against 50.1).
- None of these bitstreams is ready for hardware: every run, nextpnr's
  placement included, has 17 `no_input_pin` entries in the import
  (`"unresolved"`, which is also why `standalone` exits 1 on a legal
  route). Phase 3 did not cause it; it has to be fixed before a board
  run.
- Hold and clock skew: not analyzed. Nothing in these runs reports
  hold, so nothing is flagged; that is absence of a check, not a pass.

## Phase 4: LUT RAM placement, unresolved pins, first hold report

Timing weights are now the placer default (`TimingOpts::default`,
every 10); `OXIBIT_PT=0` turns them off. With `OXIBIT_PT` unset the
placement is byte-identical to `e10`.

### LUT RAM excursions

Measured first (`bels::Stats::memory_disp`, reporting only): every one
of the 438 memory macros (distributed RAM; picoBEAM has no shift
registers) was more than 5 tiles from where global placement left it,
418 more than 15; mean 43.8, max 82.7. Global placement gave them no
area, and they were sited last (`bels::assign_macros`), on the nearest
SLICEM still free, after slice legalisation, which uses SLICEL and
SLICEM alike, had taken 2900 of the 4350.

Candidate (a), an excursion penalty in the weights, was not run: the
displacement happens after global placement, at site assignment, where
no force acts. (b), in two steps:

- `m1`: `legalise_spread` sites the memory macros first, each on the
  nearest free SLICEM to its global position (`Problem::needs_slicem`,
  `site_slicem`), and spreads the slices over the rest.
- `m2`: also charges them in global placement (`Problem::takes_site`:
  Nesterov's density and fillers, the overflow), so it makes room for
  them before legalisation takes it.

| run | packing | change | pb_clk | axi_clk | failing nets | memory disp mean / max | slice disp mean / max | route | iterations |
|---|---|---|---|---|---|---|---|---|---|
| e10 | `--no-compact` | (Phase 3) | 54.356 ns | 8.205 ns | 3293 | 43.8 / 82.7 | 3.1 / 34 | 115 s | 13 |
| m1 | `--no-compact` | memories sited first | 61.571 ns | 8.222 ns | 5860 | 4.2 / 14.6 | 4.6 / 66 | 125 s | 15 |
| m2 | `--no-compact` | m1 + memories take room in global placement | 54.310 ns | 7.888 ns | 2520 | 3.9 / 16.5 | 3.2 / 49 | 112 s | 12 |
| c10 | compacted | (Phase 3) | 62.006 ns | 8.328 ns | 6130 | | 2.9 / 100 | 161 s | 79 |
| c2 | compacted | as m2 | 61.001 ns | 8.405 ns | 5582 | 3.6 / 11.7 | 3.0 / 112 | 145 s | 22 |

- m1 alone is worse: reserving 438 sites where global placement had
  packed slices tight around the memories pushed those slices out
  (slice max displacement 34 to 66). Legal HPWL is not comparable
  across this change: before it, the memories were counted at their
  global positions, not the far sites they got.
- m2: the LUT RAM excursions are gone from the critical path, and the
  design is broadly faster (failing nets -23%, axi_clk -0.3 ns, two
  fewer router iterations, 3% less route time), but the worst path is
  the same 54.3 ns: a different path, `levels 63`, through ordinary
  LUTs. Compact: -1.0 ns and 79 router iterations down to 22.
- Determinism: `m2` placed and routed three times (`m2`, `m2b`, `m2c`),
  identical placement JSON and FASM each time.
- Critical-path dump (`target/crit-m2.txt`): bounding box 21 x 82,
  tile travel 423, one out-and-back excursion left (19 tiles), an
  ordinary LUT (`$abc...$42925`) that m1 also pulled away. Compact
  (`target/crit-c2.txt`) still has six, all ordinary LUTs and one F7
  mux; two of them (`$45301`, the `mux7`) are the same cells as in
  `c10`. `$45301` sits on 125- and 182-pin nets and drives a 40-pin
  one: net-level weights cannot hold one critical sink of a wide net.
  Connection-level weighting (a pseudo-net per critical driver-sink
  pair) is the next placement lever; not tried.

Target (at or under about 50 to 51 ns): not met. The LUT RAM fix
removed its excursions without moving the worst path, so they were not
what set it.

### Unresolved pins and the exit code

The 17 `no_input_pin` entries are all one case: a constant 0 on an F7
mux data input (`F7AMUX.1`, `F7BMUX.0` and so on), whose only driver is
the O6 of one LUT inside the site. In every one of them that letter's
LUTs are empty and the FASM has no INIT for them, so the input reads 0
already (`/tmp/f7const.py` against `m2`'s FASM: 17 of 17). The import
now resolves exactly that case (`placed::f7_input_lut`, counted in
`Import::const_empty_lut`): a constant 0, an F7 mux data input, and no
cell on that letter's 6LUT or 5LUT. A constant 1 there, or a used 5LUT,
stays unresolved. FASM unchanged (`m2b` against `m2`: identical).

`standalone` exit codes: 0 routed and every pin resolved; 1 the routing
failed; 2 routed, but the import left pins unresolved; 3 an error.
The final line carries `routed` and `resolved`. picoBEAM now exits 0.

### First hold report

`sta::Timer::hold`: per clock, earliest arrivals from its registers at
the fast corner (`delay::clk_to_q_min`, `comb_min`; routing as its slow
delay times `delay::HOLD_ROUTE_SCALE`, 0.239, the smallest
fast-min/slow-max ratio of any interconnect pip) against
`delay::hold`, the largest fast-corner hold of the cell type. No clock
skew; same-clock checks only. Every bound is chosen pessimistic, so a
pass is meaningful and a failure is a candidate. `standalone` and the
`sta` example print a `hold` line per clock.

| routing | clock | checks | under 0.3 ns | worst | slack |
|---|---|---|---|---|---|
| oxibit, `m2` | pb_clk | 36053 | 23024 | FF `slice$366587` to BRAM `ob1` DIBDI11, 0 cells | -0.506 ns |
| oxibit, `m2` | axi_clk | 328 | 328 | FF `slice$366952` to BRAM `img` DIADI5, 0 cells | -0.463 ns |
| nextpnr, its placement | pb_clk | 36053 | 25012 | FF `slice$358463` to BRAM `pb_eng0_mbox_hv` ADDRBWRADDRL11 | -0.439 ns |
| nextpnr, its placement | axi_clk | 328 | 278 | FF `slice$366947` to BRAM `img` DIADI0 | -0.309 ns |

- Both worst pairs are a flop driving a block RAM input directly,
  against the block RAM's 0.667 ns hold. That figure is its worst
  configuration; the data inputs' hold depends on the write mode
  (prjxray `BRAM_L.sdf`: 0.405 READ_FIRST, 0.667 WRITE_FIRST and
  NO_CHANGE). `ob1` is READ_FIRST, so its slack is -0.244; `img` is
  WRITE_FIRST, so -0.463 stands.
- nextpnr's own routing of its own placement shows the same class at
  the same size, so this is not something oxibit's flow introduced.
  Whether it is real needs what this leaves out: per-pin, per-mode
  block RAM hold; fast-corner pip delays per pip instead of one scale;
  and the clock insertion delay of a block RAM's clock pin against a
  slice's, which is skew in exactly these pairs.
- Phase 5 item, named: flop to block RAM data and address inputs, both
  clocks (`ob1`, `img`, `pb_eng0_mbox_hv`). Refine the model first;
  only then decide whether the router needs hold fixing.

### STA calibration, second point

Not measured. nextpnr cannot read oxibit's routing, so it cannot time
the same routed design. No second xc7 design here has both a nextpnr
FASM and its report: the logs under `ehdl/.scratch/fuzz` have no FASM
next to them. The way to the second point: run nextpnr with `--fasm` on
a second design (libresdr_radio), and time that FASM with the `sta`
example (`none` pin mode) against nextpnr's report, as was done for
picoBEAM.

## Where the implementation differs from the plan below

- Criticality is per sink as well as per net (`CriticalityMap::of_sink`):
  a high-fanout net with one late sink has only that sink routed for
  delay, and only that branch ripped up.
- Slack needs a period, and the design has no constraint file: periods
  come from `sta::Constraints` (`OXIBIT_TIMING`). A clock with none has
  paths but no slack, and its nets have criticality 0.
- Only the annotation is incremental (`TimingSource::update` re-times the
  nets just routed). Arrivals and required times are propagated over the
  whole graph each time; that costs a few ms.
- `TimingView` sits in `router::Opts` as planned, holding its source
  behind `Rc<RefCell<dyn TimingSource>>` so `Opts` stays `Clone`.
- "Re-time top-N worst paths" became `TimingView::hot_max`: at most that
  many failing nets are ripped up for timing per iteration.
- Block RAM and DSP48E1 cell delays are from prjxray-db's
  `timings/*.sdf` (slow corner maximum). A LUT RAM's write-side setup
  and a registered DSP48E1 are still placeholders, flagged `assumed` in
  any path that uses them; picoBEAM's reported paths use neither.
- A thin `sta` example remains (arguments in, module call, JSON out).

## The problem (original baseline)

picoBEAM `xc7_m5` (nocarry), same measurement tool for all rows:

| placement | routing | pb_clk worst path | pb_clk Fmax | axi_clk Fmax |
|---|---|---|---|---|
| nextpnr | nextpnr | 50.1 ns | 19.9 MHz | 122.9 MHz |
| nextpnr | oxibit | 68.9 ns | 14.5 MHz | 114.7 MHz |
| oxibit | oxibit | 74.9 ns | 13.3 MHz | 115.8 MHz |

- Router accounts for ~18.7 ns of the ~24.8 ns gap (row 1 vs row 2).
- Placement adds ~6 ns more (row 2 vs row 3).
- The worst path is 65 logic levels and zig-zags across the device;
  14 hops over 1.5 ns each account for ~33 ns of ~64 ns routing delay.
- axi_clk is within 6% across rows — the loss is concentrated on the
  long single-clock path where per-hop quality dominates.

## Strategy

Closed-loop slack feedback into the P&R cost functions, in this order:

### Phase 1: STA as a first-class engine (foundation)
Promote the analyzer from a scratch example into a crate-level module.
- `crates/oxibit-pnr/src/sta.rs` (new), with `sta::Analysis` producing,
  per net: worst slack, endpoint arrival, and criticality
  `crit(net) = max(0, -slack) / (1 + max(0, -slack))` (normalized 0..1).
- Incremental annotation: re-time only nets touched since the last
  analysis during router iterations (needed for Phase 2 to be cheap).
- Cell delays: keep the current constants table in
  `oxibit_pnr/src/delay.rs`; add BRAM/DSP delays with unit tests.

### Phase 2: timing-aware routing (highest expected payoff)
In `oxibit_pnr/src/router.rs` (the rip-up cost function):
- Add a per-net criticality weight to the present-congestion/history
  term: `cost = base + pres_fac*overuse + hist_fac*history
  + tau * crit(net) * edge_delay(edge)`.
- Two-lever rip-up: a path stays "hot" (eligible for rip-up and
  re-timing) only while its slack is negative; once clear, it stops
  contributing criticality so effort concentrates on the failing cone.
- Re-time top-N worst paths per iteration batch (not all nets).
- Type-system integration: the router currently treats routing choices
  as opaque node ids; introduce a `TimingView` struct handed to the
  router opts, so cost lambdas read typed
  `(&SlackTable) -> &CriticalityMap` instead of raw numbers. This keeps
  the router testable with a stubbed clock-free view.

Expected: recovers a large fraction of the 18.7 ns router gap because
14 of the bad hops sit on one path the router never knew was critical.

### Phase 3: timing-weighted placement (the parked weeks 5/6 work)
In `oxibit_place` (heuristic cost in the placer loop):
- Cost = wirelength + sum over nets of `crit(net) * est_delay(net)`.
- Criticality from a pre-route STA estimate (use existing
  `oxibit_pnr::delay` estimates on the placed, unrouted design).
- Do not start until Phase 2 lands and is measured; per the baseline,
  placement is the smaller term (~6 ns) and benefits from a real
  slack oracle existing first.

### Phase 4: logic-level path shortening (optional, later)
- 65 logic levels is a synthesis/packing outcome; investigate
  LUT-input sharing/packing and register retiming hints at the
  `oxibit-pnr` import boundary before placement. Parked.

### Phase 5: MMCM value solving (design-flow level, not P&R)
- The MMCM static config (DIVCLK=1, M=10, CLKOUT0/1 divide=20 →
  50 MHz out) lives in the FASM site features
  (`CMT_TOP_L_LOWER_B_X178Y61.MMCME2_ADV.*`). If the harness retunes
  the clock at runtime this is out of our scope; if we ever compute
  static settings, solve VCO/DIV from measured per-domain Fmax rather
  than fixing 50 MHz. Parked.

## Measurement discipline
- Every phase lands with a before/after table row on picoBEAM
  `xc7_m5` nocarry, same seed, measured by the promoted STA module.
- Success criteria Phase 2: pb_clk worst path improves by >= 10 ns vs
  the oxibit/oxibit row; axi_clk regresses < 3%.

## Cleanup debt (prereq)
- `sta.rs` example: promoted to `oxibit_pnr::sta`; a thin driver remains.
- Scratch examples `dsp_check.rs`, `dsp_map.rs`, `dsp_reach.rs`,
  `dsp_edges.rs` and `dbg_grid.rs`: moved out of the tree to
  `/tmp/oxibit-scratch/`. `dsp_grid.rs` and `dsp_union.rs` are still in
  oxibit-fasm's examples (not on this list; undecided).
- Newtype the three site-name conventions
  (`SiteNameDb` / `SiteNameRel` / `GraphNodeName`) as discussed; the
  DSP-cascade bug class cannot recur once `grid_loc` takes
  `&SiteNameDb` only.
