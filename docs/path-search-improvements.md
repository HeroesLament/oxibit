# Oxibit Path Search Improvements

Status: in progress. Companion to `docs/clock-improvement.md`.
Baseline table and phase results are recorded there.

## Context

Timing-aware routing (Phase 2 of clock-improvement.md) landed and met
its target, but the router plateaued: delay-weight knobs (tau),
re-rip rules, and a near-exact search estimate each moved the worst
path by under ~1.6 ns, and combinations bought nothing. The remaining
gap to the reference router splits roughly half router, half placement
(~6 ns each on pb_clk worst path).

A wire-by-wire dump of the six worst connections, our route vs the
reference on the same placement, found concrete structural causes
rather than cost-function issues:

1. **Long lines go unused.** On a 92-tile connection the reference
   took three 18-tile long lines (9 hops, 1.8 ns); oxibit chained
   6-tile wires (21 hops, 3.3 ns).
2. **Same-tile bounce on a long line.** Entered and exited a long line
   in the same tile, paying its full ~0.3 ns delay for zero distance.
3. **Destination detours.** Two connections overshot or double-bounced
   in the final switchbox because the direct entry wire was occupied.

## Root cause (items 1 and 2)

The routing graph records each multi-tile wire's position as ONE
arbitrary tile of its span. Depending on which end that is:

- A long line heading toward the target looks like no progress, so the
  A* heuristic and greedy expansion never take it.
- A long line whose recorded tile is the target tile looks like arrival,
  so the search commits to it and pays full delay for no distance.

Both are search-guidance defects, not cost defects: a long line is
already cheaper than chained short wires under any sane cost table; the
search simply cannot see that from where it stands. This is why cost
tuning plateaued — you cannot weight your way out of edges that are
never expanded.

## Work already in flight (uncommitted)

- Full span of each wire added to the routing graph.
- `Opts::span_aware` (default off): distance-to-target measured from
  the wire's NEAREST point of its span rather than its recorded tile.
- Two runs pending: timing-on (compare vs 62.4 ns worst path) and
  timing-off (compare vs 74.9 ns baseline, and watch route time).

## Guidance for approaching this

### 1. Land and measure span_aware first, alone

Do not stack other search changes under test with it; attribution
discipline is how the plateau got diagnosed. Acceptance criteria are
concrete, not just the ns number:

- Re-dump the six worst connections: long lines present on the 92-tile
  connection, zero same-tile bounces.
- pb_clk worst path should improve substantially (worst path has
  several connections of the same shape as the 92-tile one).
- Route time must not balloon. If it does, the answer is bounding
  (below), not reverting.

### 2. If search time explodes: detour-factor bounding

Span-aware makes long lines look attractive everywhere, so more nodes
get expanded. The standard remedy is to bound the expansion: allow an
edge only if taking it does not move the search more than a factor
beyond the best-known remaining distance. Implement as an expansion
filter in the A* loop, tunable factor, default conservative.

### 3. Re-test the congestion discount AFTER span_aware

The destination detours (item 3) are an incumbent-vs-arriver problem:
a critical connection arrives to find the direct entry wire taken and
detours, because our rip-up only triggers when the LATER net's
criticality rises. An earlier experiment (TimingView::discount,
congestion cost reduced for critical connections) showed no effect —
but it was tested while the heuristic was lying and criticality was
saturated across the whole 65-level path. With span_aware, signals
are less saturated; re-run discount before concluding it is dead.
Also consider pre-emptive rerouting: a newly arriving critical net
forces the incumbent off the entry wire rather than detouring.

### 4. Watch convergence on foreign placements

On the reference tool's placement, our router with timing on took 143
iterations / 153 s (vs 11 iterations on our own placement). Poor
search guidance compounds convergence problems; if span_aware brings
that down it corroborates the diagnosis. If it does not, investigate
the negotiation dynamics separately.

### 5. Only then consider net decomposition

Multi-sink net tree construction (per-sink priority by criticality,
branch sharing for slack-rich sinks) is the next search-region topic,
but it is confounded with everything above. Sequence it last.

## Measurement discipline

- Same seed, same placement, per-run rows in clock-improvement.md.
- Wire-dump of the six worst connections is the acceptance artifact
  for each change, not only the aggregate ns.
- One search change in the loop at a time.
