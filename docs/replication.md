# Replication

A placer puts a cell in one spot. A LUT or flop driving sinks all over the
device lands between them, far from each, and no net weight shortens that.
A copy of the cell per group of sinks can.

`examples/replicate_plan` reads a routed design's timed arcs (`sta`'s
STA_ARCS), takes LUT and flop drivers with enough sinks and enough mean
routed delay to them, splits each one's sinks into groups by position, and
writes a plan: which sinks move to which copy. `pack` applies a plan with
OXIBIT_REPLICATE before packing (`replicate::apply`). Unset, the netlist
is as read.

What prompted it: picoBEAM's worst axi_clk path doubles back on itself
through one LUT with 43 sinks, 3.5 ns from them on average.

## First run: the worst 400 copies by total delay

Defaults (8 sinks, 1500 ps mean, groups of 24 tiles, 8 groups) qualify
1,645 cells on picoBEAM and 692 on mimohalow. Taking the worst by total
routed delay until 400 copies:

| mimohalow | no replication | 397 copies of 74 cells |
|---|---|---|
| axi_clk worst path (target 10 ns) | 13.142 ns, Sy 0.377 | 12.396 ns, Sy 0.400 |
| data_clk worst path (target 31.25 ns) | 42.274 ns, Sy 0.734 | 43.696 ns, Sy 0.710 |
| route passes | 18 | 58 |

Not a win: 0.75 ns on axi_clk, still failing, for 1.4 ns lost on data_clk
and three times the routing effort. The ranking is the fault. Total delay
to the sinks picks the cells with the most sinks, 74 cells here with 490
sinks each on average: resets and enables, not the cells on failing
paths. Each copy of such a net is also a new net under the placer's
200-sink fan-out limit, so it now pulls on placement where the original
was left out. The plan should be chosen by slack, not by total delay.

picoBEAM, same selection (398 copies of 76 cells), went the other way:

| picoBEAM | no replication | 398 copies of 76 cells |
|---|---|---|
| axi_clk worst path (target 10 ns) | 9.962 ns, Sy 0.311 | 7.154 ns, Sy 0.434 |
| pb_clk worst path (goal 50 ns) | 64.146 ns, Sy 0.378 | 61.220 ns, Sy 0.393 |
| route | 200 passes, then the stuck rescue (27 nodes) | 85 passes, converged |

Every figure better, and the first picoBEAM route to converge by
negotiation alone. So the same rule helps one design and hurts the other.
mimohalow is 94% full of LUTs and its copies averaged 92 sinks; picoBEAM
has room and its copies averaged 48. Whether that is the difference is
not shown. Choosing by slack remains the next step, and both designs are
its test.

## Second run: chosen by slack

`replicate_plan ... slack_ps=0`: STA_ARCS now carries each arc's slack, and
a cell is copied only for its sinks on arcs that miss their clock's target
and are routed 1000 ps or longer; the original keeps the other sinks.
Worst slack first, 400 copies at most. Targets: axi_clk 10 ns, data_clk
31.25 ns, pb_clk 50 ns.

| mimohalow | none | by total delay (397) | by slack (96 copies of 52 cells) |
|---|---|---|---|
| axi_clk | 13.142 ns | 12.396 ns | 14.149 ns |
| data_clk | 42.274 ns | 43.696 ns | 47.381 ns |
| route passes | 18 | 58 | 17 |

| picoBEAM | none | by total delay (398) | by slack (378 copies of 244 cells) |
|---|---|---|---|
| axi_clk | 9.962 ns | 7.154 ns | not routed |
| pb_clk | 64.146 ns | 61.220 ns | not routed |
| route | 200 passes + stuck rescue | 85 passes, converged | 200 passes, 115 nodes overused |

Worse than no replication on both. Leaving the large nets alone did put
mimohalow's routing effort back (17 passes), so that part of the reading
of the first run holds. The rest does not: copying the cells on failing
paths made the paths slower, and picoBEAM unroutable.

Two things are not separated by these runs. mimohalow's data_clk target is
at its floor, so nearly every deep arc fails and the worst-slack cells are
the cells of one path, each of them split; and picoBEAM's pb_clk fails its
target on 2,299 nets. And 5 ns on data_clk from 96 cells in 50,000 is a
large answer to a small change: how far these results move under any
change to the netlist has not been measured, and until it is no single
run here ranks one method over another. picoBEAM without replication
already sits where a different placement decides whether it routes.

Also open: about a quarter of the planned sinks did not resolve to
netlist cells (packed carry and LUT RAM names).

## Noise floor

Five runs per design, each copying 5 LUTs that cannot matter: two sinks,
every arc with over 3 ns of slack, one sink moved to the copy. A different
5 per run. Everything else as the base run.

| mimohalow | base | five runs | spread |
|---|---|---|---|
| axi_clk | 13.142 ns | 13.801 13.497 11.237 12.855 14.383 | 3.1 ns |
| data_clk | 42.274 ns | 42.230 43.144 43.056 42.625 45.183 | 3.0 ns |
| route passes | 18 | 14 13 15 25 21 | |

| picoBEAM | route | axi_clk | pb_clk |
|---|---|---|---|
| base | 200 passes + stuck rescue | 9.962 ns | 64.146 ns |
| run 1 | 200 passes + stuck rescue | 9.686 ns | 62.456 ns |
| run 2 | failed, 126 nodes overused | | |
| run 3 | failed, 88 overused | | |
| run 4 | failed, 47 overused | | |
| run 5 | failed, 134 overused | | |

Five LUTs in 50,000 move mimohalow's clocks by 3 ns, and picoBEAM routes
in one run of five. Read against that:

- Timing under either method is inside the band on mimohalow, except
  data_clk by slack (47.381 ns), 2.2 ns past the worst of five.
- picoBEAM failing to route by slack is what four of five harmless
  changes do. It says nothing about the method.
- Route effort by total delay is outside the band both ways: 58 passes
  on mimohalow against 13 to 25, and picoBEAM converged in 85 where no
  other run has converged at all. picoBEAM's axi_clk at 7.154 ns is
  also well past the one routed noise run.

Runs 2 and 4 exit with "LUT pin taken twice or out of range" after the
cap, as the by-slack picoBEAM run did; runs 3 and 5 exit 1. Whether the
first is only a symptom of the unresolved overuse is unchecked.

A single run cannot rank two placer or replication settings on these
designs. Either compare over several perturbed runs, or take the best
of several as the result: run 3 on mimohalow gave axi_clk 11.237 ns,
better than any method tried here.

## Giant nets only, over five perturbed runs

`replicate_plan arcs.tsv fanout=200 mean_ps=0 budget=400`: only cells
driving 200 sinks or more (picoBEAM 81 cells, 373 copies; mimohalow 73,
398). This is nearly the plan of the first run, which was giant nets in
all but name. Each run adds the same 5-LUT perturbation as the noise runs.

| picoBEAM | route | axi_clk | pb_clk |
|---|---|---|---|
| run 1 | 120 passes + stuck rescue | 7.942 ns | 62.859 ns |
| run 2 | 200 passes + stuck rescue | 7.819 ns | 61.060 ns |
| run 3 | failed, 56 nodes overused | | |
| run 4 | 136 passes + stuck rescue | 8.833 ns | 61.666 ns |
| run 5 | 135 passes, converged | 8.749 ns | 61.371 ns |
| without (noise runs) | 1 of 5 routed | 9.686 ns | 62.456 ns |

| mimohalow | five runs | without (noise runs) |
|---|---|---|
| route passes | 102 33 45 71 70 | 13 to 25 |
| axi_clk | 13.607 14.227 11.855 11.041 11.845 | 11.237 to 14.383 ns |
| data_clk | 45.031 43.500 44.590 47.660 49.401 | 42.230 to 45.183 ns |

picoBEAM: 4 of 5 route against 1 of 5, axi_clk about 1.5 ns better in
every routed run, pb_clk unchanged. Three of the four still need the
stuck rescue, so the first run's 85-pass convergence was the lucky end of
this. mimohalow: two to five times the route passes and data_clk slower
in most runs. A picoBEAM setting, not a default.

Of the planned sinks, 2,711 of 21,376 (picoBEAM) and 2,259 of 37,019
(mimohalow) stayed on the original. `resolve` now also takes packed carry
and suffixed names, and `pack` prints the first skipped sinks with the
reason; how many that recovers is not yet measured.

The "LUT pin taken twice or out of range" exit is lutinit::permute being
handed two routes that end on one LUT input, which only an unresolved
route produces. Every run that hit it had nodes overused at the cap.
