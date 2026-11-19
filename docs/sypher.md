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

- 0 for a LUT, F7/F8 mux or carry into a flop, mux or carry: a connection
  that can stay inside a slice. At `Floor::Netlist`, wherever it could; at
  `Floor::Packed`, only where driver and sink share a site.
- its routed delay for carry to carry: dedicated wires, no router's doing.
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
| picoBEAM pb_clk | 29.804 ns | 64.146 ns | 33.55 MHz | 15.59 MHz | 0.465 | 3.33 dB | 49 / 53 |
| picoBEAM axi_clk | 3.102 ns | 9.962 ns | 322.4 MHz | 100.4 MHz | 0.311 | 5.07 dB | 4 / 4 |
| mimohalow data_clk | 35.481 ns | 42.274 ns | 28.18 MHz | 23.66 MHz | 0.839 | 0.76 dB | 45 / 73 |
| mimohalow axi_clk | 4.956 ns | 13.142 ns | 201.8 MHz | 76.09 MHz | 0.377 | 4.24 dB | 11 / 1 |

picoBEAM at a 50 ns target: Sy_req 0.596 (2.25 dB of budget), 1.08 dB
short. Its worst path is 53 cells deep, so its ceiling is 33.5 MHz
whatever the backend does, and its routed arcs average 950 ps against a
301 ps hop: to meet 50 ns they must average 683 ps.

mimohalow's data_clk is at 84% of its ceiling: the physical flow has
0.76 dB left to give, and the rest is the RTL's depth. Its axi_clk worst
path is one flop-to-BRAM arc of 12.1 ns.

`Sy_pack` reads 1.000 on all four: no connection that could have stayed
in a slice is split on any worst floor path. Connections inside a site
carry no routed arc and are 0 in every analysis.
