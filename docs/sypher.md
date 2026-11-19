# The Sypher number

One dimensionless figure for how much of a design's speed the physical
flow delivers:

    Sy = T_floor / T_crit        0 < Sy <= 1
    Fmax = F_floor * Sy          F_floor = 1 / T_floor

`T_crit` is the worst same-clock register-to-register path of the routed
design. `T_floor` is the same path analysis with every arc at its floor:
what the netlist would run at were nothing lost to packing, placement or
routing. `F_floor` belongs to the RTL and synthesis; `Sy` to everything
after. An unroutable design has `Sy = 0`.

## The chain

Each stage idealises fewer arcs than the one before, so T only grows:

    T_floor       the netlist alone
    T_pack        the packing fixed which arcs leave a slice
    T_place_min   positions fixed, shortest wires on an empty fabric
    T_crit        routed, detours and all

    Sy = Sy_pack * Sy_place * Sy_route

each factor the ratio of consecutive T, so the product is exact and each
factor has one owner. As losses, `L = 10 log10(1 / Sy_stage)` dB, they
add like a link budget. `Sy_req = T_floor / T_target` is the budget a
target period leaves the physical flow; above 1, no backend can meet it.

## Floor delays

Per arc (`sta::Timer::floor`):

- 0 for what a slice wires inside itself: a LUT into a flop, F7/F8 mux or
  carry; a mux into a flop or the next mux; a carry into a flop. At
  `Floor::Netlist`, wherever it could; at `Floor::Packed`, only where
  driver and sink share a site.
- its routed delay for a carry out into the next CIN: the one dedicated
  wire, no router's doing (0 where nothing was routed). A carry into any
  other carry pin is routed like the rest.
- `w0` for the rest: one minimal routed hop. 301 ps for now, the (0, 0)
  cell of the delay grid measured from picoBEAM's and mimohalow's routes;
  per pin class once the empty-fabric grid exists.

An arc is never raised to its floor, so the three analyses cannot cross.
Same delay model and corner throughout: a bias in cell delays is in both
T and largely leaves the ratio.

## Reading it

`examples/sta` prints a `sypher` line per clock. `sy_place_route` is
`Sy_place * Sy_route` until `T_place_min` is measured. A design's Sy is
its binding clock's. Hold is reported beside it, not inside it.

The worst path is a maximum, and which path that is can change between
runs; steer by total negative slack or a top-k form, report plain Sy.

## First measurements

Current `main` routes (40745de placements, default options), w0 = 301 ps:

| design, clock | T_floor | T_crit | F_floor | Fmax | Sy | loss | levels (floor / routed) |
|---|---|---|---|---|---|---|---|
| picoBEAM pb_clk | 24.238 ns | 64.146 ns | 41.26 MHz | 15.59 MHz | 0.378 | 4.23 dB | 55 / 53 |
| picoBEAM axi_clk | 3.102 ns | 9.962 ns | 322.4 MHz | 100.4 MHz | 0.311 | 5.07 dB | 4 / 4 |
| mimohalow data_clk | 31.039 ns | 42.274 ns | 32.22 MHz | 23.66 MHz | 0.734 | 1.34 dB | 88 / 73 |
| mimohalow axi_clk | 4.956 ns | 13.142 ns | 201.8 MHz | 76.09 MHz | 0.377 | 4.24 dB | 11 / 1 |

picoBEAM at a 50 ns target: Sy_req 0.485 (3.14 dB of budget), 1.09 dB
short. Levels count carry cells: picoBEAM's worst routed path is a BRAM,
31 LUTs, 22 carries and a BRAM, and the 32 routed hops carry 99% of its
50.4 ns of wire, 12.6 ns of it on the two hops at the BRAMs.

`Sy_pack` reads 1.000 on all four: no connection that could have stayed
in a slice is split on any worst floor path. Connections inside a site
carry no routed arc and are 0 in every analysis.

The unrouted floor (`sta ... placed.json -`) agrees with the routed one:
24.204 against 24.238 ns on pb_clk, 30.579 against 31.039 on data_clk,
the difference being the 20 ps site pip each routed CO-to-CIN hop keeps.

An earlier version of this table (commit f5c7109) read T_floor 29.804 ns
and Sy 0.465 for pb_clk, 35.481 ns and 0.839 for data_clk. It classed
every carry-to-carry arc as dedicated and so kept routed delays on
carries feeding other chains through general routing. The two floor
analyses disagreeing by 5 ns is what found it.
