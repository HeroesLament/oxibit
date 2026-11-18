# Router performance

What the router spends its time on, and which changes paid. The design of
record is mimohalow on the merged `main` placement (`pd_r3_mh`): 18 passes,
557 s, 2.46 GB, bitstream assembled. Every row below reroutes that same
`placed.json`, so a row is only a result if its `top.fasm` is byte-identical
to the baseline's.

## Where the time goes

PathFinder relaxation is memory-bound, not compute-bound. One pop touches
nine arrays indexed by node (`reserved`, `vcc_kept`, `fabric`, `occ`, `tag`,
`cost`, `hist`, `prev`, `node_box`/`xy`), each a separate cache line, and
then follows the tile's `Arc<TypeData>` to `pips[pi]` for the edge's kind
and delay. On the 44.7 M pops of a congested pass that is tens of millions
of L2/L3 misses; the arithmetic per pop is a handful of adds.

So the levers, in order of expected return:

1. Fewer pops (bounding, better estimates) — attacks the term that matters.
2. Fewer cache lines per pop (pack the per-node hot fields into one record).
3. Cheaper constant factors (hashing, diagnostics, codegen).

## Measured

| change | mimohalow reroute | `top.fasm` | verdict |
|---|---|---|---|
| baseline (merged `main`) | 18 passes, 557 s, 2.46 GB | 74cfbe71e882e0c0e757a09cf078f9b4 | reference |
| + fat LTO, 1 codegen unit, FxHash, `by_f` gated | 18 passes, 533 s, 2.47 GB | 74cfbe71e882e0c0e757a09cf078f9b4 | kept, 4.3% |
| + `NodeHot`: cost/hist/tag/occ in one 16 B record | 18 passes, 508 s, 2.48 GB | 74cfbe71e882e0c0e757a09cf078f9b4 | kept, 4.8% (8.9% cumulative) |

All three rows above were measured with another router running on the same
host. On a quiet box the same `NodeHot` build routes in 423 s, not 508: the
contention was worth about 17%, which is larger than any single change
measured here. Treat those rows as ordered, not as deltas. Pairs measured
one at a time on an idle host:

| arm | mimohalow reroute | `top.fasm` | verdict |
|---|---|---|---|
| `NodeHot` record (stage A) | 18 passes, 423.36 s, 2.48 GB | 74cfbe71e882e0c0e757a09cf078f9b4 | reference for stage B |
| + the four node flags in the record's spare byte | 18 passes, 391.24 s, 2.47 GB | 74cfbe71e882e0c0e757a09cf078f9b4 | kept, 7.6% |

The lesson for later rows: run the arms one at a time, on an idle host, or
the measurement says more about the neighbours than the change.

The 533 s run shared virt2 with another router, so it paid L3 contention
the baseline may not have: 4.3% is a floor, not a ceiling. Same pass count,
same peak RSS to within 0.4%, byte-identical FASM. None of these three
changes can alter a routing decision, and the FASM confirms none did.

## Bounding the hold-repair pass

`hold_pass` walks three rounds over the hold targets; each target tries up
to `DEPTH * PER_DEPTH` waypoint candidates, and each candidate runs two
`route_net` searches. Nothing stopped a target that no waypoint could fix
from paying all of that, and nothing stopped the pass as a whole, so a run
whose placement had many unfixable short paths spent hours there.

Three bounds, none of which changes a repair the pass would have found
early:

- `GIVE_UP = 24`: candidates in a row that produce no better detour before
  the target is abandoned. A target that is going to be fixed is fixed by
  an early candidate; one that is not fails all of them identically.
- `Opts::hold_budget_s` (default 300 s, `OXIBIT_HOLD_BUDGET=S`): a wall
  budget checked per target. On expiry the pass stops and the report says
  so rather than silently returning a partial result.
- The two legs are built as fresh `Net`s instead of `..net.clone()`, which
  was cloning the sink and pin-group vectors of the whole net per candidate
  — on a 2,105-sink net, per candidate, per target.

The JSON report gained `given_up`, `budget_s` and `budget_hit` so a capped
run is distinguishable from a complete one.

Rejected for now: seeding leg 1 from the replaced branch only. `Reroute::keep`
is both the search's start set and the subtree merged into the result, so
shrinking it to the branch stem would drop the rest of the tree from
`routes[i]` and strand the net's other sinks. Separating the two roles is a
real change to `route_net`'s contract and wants its own measurement, not a
ride-along.

## Cheaper constant factors

- `fx.rs`: FxHash (the rustc hasher) behind `HashMap`/`HashSet` aliases, for
  the router's `u32`-keyed maps and sets. SipHash's quality buys nothing for
  dense small integers and costs a multiply-rotate chain per probe.
- `by_f` diagnostics (`OXIBIT_BY_F`) are off unless asked for. The histogram
  push sat in the relaxation loop and ran on every pop of every search.
- Release builds get `lto = "fat"` and `codegen-units = 1`. The hot path
  crosses crate boundaries into `oxibit-fabric` on every edge lookup, and
  those calls cannot inline across codegen units.

## Packing the per-node hot fields

The relaxation reads, for every edge it considers and indexed by the
destination node: `reserved`, `vcc_kept`, `fabric` (clock nets), `occ` and
`tag` (twice), `cost`, and `occ`/`hist` again inside `node_cost`. As separate
`Vec`s that is one cache line per array per node touched, for arithmetic
that is a handful of adds.

`NodeHot` puts `cost`, `hist`, `tag` and `occ` in one 16-byte record, four to
a line. `occ` is `u16` and the record carries 2 bytes of padding to land on
16 exactly; a test pins the size, because the layout *is* the optimisation
and a field added later would silently undo it.

`prev` stays outside: it is written only when a relaxation improves a node
and read only during traceback, so it would dilute the record for the common
case. The whole-array sweeps (`hist` update, overuse counting, contested-node
collection) become strided over 16-byte records instead of contiguous, but
they run once per pass over the nodes against tens of millions of pops.
`run_observed`'s public `&[u16]` of occupancy is materialised once per pass
rather than changing the signature.

One pleasant consequence: the history update read `occ` and wrote `hist` in
the same loop, which the packing turned into a borrow error and then into a
single `iter_mut` sweep — correct, and better localised than the two-array
version it replaced.

## Folding the node flags into the record

`reserved`, `vcc_kept`, `fabric` and `muxed` were four `Vec<bool>`s, and the
first three are read on every edge relaxation. They fit in the padding byte
`NodeHot` already carried, so the record stays 16 bytes and three more cache
lines per node touched disappear. `muxed` and `fabric` never change once
built, so they are folded in as the record is constructed.

The packing also forced out something worse. Two rip-up attempts reserved
every occupied node by cloning the whole `reserved` array, routing, then
assigning the clone back: two full-array copies per attempt. They now record
the nodes they actually marked and clear exactly those, which is what the
packed record requires and also strictly less work than before.
