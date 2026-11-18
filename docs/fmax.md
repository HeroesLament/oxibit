# Where Fmax goes

Setup timing of two routed designs, measured with `examples/sta` on the FASM
that was actually built (`DELAY_MODEL=intrinsic`, `STA_SITE_PIP=20`). These
are model delays, not silicon measurements, so treat a few percent as noise.

## The split

| design | path | total | logic | routing | Fmax |
|---|---|---|---|---|---|
| picoBEAM ms14c | pb_clk -> pb_clk | 66.03 ns | 13.247 | 52.783 (80%) | 15.14 MHz |
| picoBEAM ms14c | pb_clk -> axi_clk | 54.616 ns | 9.861 | 44.755 (82%) | |
| picoBEAM ms14c | axi_clk -> pb_clk | 14.761 ns | 1.426 | 13.335 (90%) | |
| picoBEAM ms14c | axi_clk -> axi_clk | 9.592 ns | 1.597 | 7.995 (83%) | 104.25 MHz |
| mimohalow pd_r3 | data_clk -> data_clk | 42.274 ns | 16.42 | 25.854 (61%) | 23.66 MHz |
| mimohalow pd_r3 | axi_clk -> axi_clk | 13.142 ns | 1.002 | 12.14 (92%) | 76.09 MHz |

Routing is 61% to 92% of every critical path in both designs. picoBEAM's
worst path is 80% wire: its logic alone would close at 75 MHz, and routing
takes it to 15. Both of picoBEAM's worst paths run BRAM to BRAM, which is
the register window and the frame RAM.

## What that means for the work

The order follows from the split, and it is not the intuitive order:

1. **Timing-driven placement.** Wire length is the placer's to decide, and
   wire is most of the path. This is where the headroom is.
2. **Timing-driven routing.** The router already has the machinery --
   `crit`, the `w`/`kd` weights in `timed_step` -- but congestion and
   history dominate the cost. Letting critical nets win detour budget is
   tuning what exists.
3. **RTL pipelining.** Worth real effort only after the wire term falls.
   Pipelining picoBEAM's logic to zero would still leave 52.8 ns on the
   critical path: 19 MHz, against 15.14 today.

Put the other way: the entire logic depth of picoBEAM's critical path is
13.2 ns. Even eliminating all of it buys 26%. Halving the routing buys 66%.

## The clock the machine is actually given

`lib/fab_farm/pb_shell.ex` defaults to 15.6 MHz, a 64.1 ns period, against a
66.03 ns path. `scripts/pb_raft_host.exs` sets no clock, so it inherits that
default; `scripts/pb_raft_matrix.exs` defaults to 10 MHz (100 ns), which has
ample margin.

So the Raft debugging runs have been marginally over Fmax while the matrix
runs have not. 15.6 against 15.14 is 3%, inside the delay model's own error,
so this is a hypothesis rather than a finding. It is worth testing because
of how much it would explain:

- A pure function returning atom 15 is what a corrupted BRAM read looks like,
  and both worst paths end at a BRAM.
- The dependence on sharing engine 0 needs no logic bug: engine occupancy
  decides which addresses and ports are exercised, so a marginal path only
  fails in some interleavings.
- No RTL simulation reproduced it because none could. Icarus is zero-delay
  and functional; a setup violation is invisible there by construction.

The test costs nothing: run the same Raft scenario at 10 MHz and at 15.6 MHz.
Clean at 10 and faulty at 15.6 is timing. Faulty at both is RTL.
