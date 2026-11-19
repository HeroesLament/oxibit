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
