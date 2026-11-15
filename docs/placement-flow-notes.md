# The incumbent's placement flow, stage by stage, as guidance for oxibit

A narrative reading of how the commercial flow gets from a synthesized
netlist to a placed design, distilled into what oxibit should do at each
stage. Algorithmic content is standard published material (see
`improving-oxibit.md` sources); what this adds is the *ordering*, the
feedback loops, and the operational habits that make it production-grade
rather than a paper implementation.

## Stage 0: parameterization and instrumentation

The commercial flow is driven end-to-end by a string-keyed parameter
manager consulted at *every* phase boundary (verbose levels, dump flags,
iteration knobs, strategy selectors). Its stages self-instrument with
named timers and stage labels, and its flow emits a stage-by-stage
progression trace (before-packing, after-packing, per-solve, per-fix).

Guidance for oxibit:
- Keep the existing `oxibit-report` event schema as the equivalent: one
  progress event per stage boundary, carrying wirelength, overflow triple,
  utilization and iteration count. Placement quality regressions are
  debugged from these traces; do not bolt this on later.
- A `PlaceParams` struct with serde, not a global: every knob we tune
  (step sizes, thresholds, schedules) gets a name, a default and a place
  in the report events, so A/B runs are comparable.

## Stage 1: hybrid packing (interleaved with early placement)

Order of operations, per the flow structure:

1. **Density partitioning first.** The netlist is partitioned by bin-grid
   density (capacity- and occupancy-based boundaries) before any packing
   decisions. Packing is per-partition, which both bounds the problem and
   keeps physical locality.
2. **Scored candidate generation, then greedy best-choice.** Logical
   neighbors (shared nets) and physical neighbors (bin distance under a
   per-iteration distance threshold) become candidates; every candidate
   pair gets a merging score; a priority structure pops the best pair,
   merges, and updates neighbor scores. Iterations run with a schedule
   of progressively relaxed distance thresholds.
3. **Distribution breaking.** After the greedy pass, slices with too
   many attracted instances are broken up by a priority rule and the
   freed instances re-enter the candidate pool. This is the
   anti-congestion pressure that keeps the greedy stable.
4. **Control-set clustering as a matching problem.** FF clock/CE/SR sets
   are clustered via bipartite matching so conflicting sets share slices
   maximally rather than first-come-first-served.
5. **Packing runs concurrently with early placement iterations** - the
   physical term of the score is *current* estimated position, updated as
   placement progresses. "Hybrid" is the operative word: it is a loop
   between packing and spreading, not a one-shot.

Guidance for oxibit: our sprint composes these as pure functions in a
bounded loop (spread -> pack -> place -> re-pack). The distance-threshold
schedule and the distribution-breaking rule are the two details most
likely to make the difference between a toy and a real packer; budget
tuning time for both. Control-set matching slots in beside `ffs.rs::Ctrl`
without touching the constraint itself.

## Stage 2: global placement - the analytic engine

One engine, two modes (global and detailed) behind a stage flag; the
structure per mode:

1. **B2B wirelength assembly.** Per net: two-pin edges, extreme-pin
   edges, and internal-to-extreme edges, each contributing a weight `w`
   to the quadratic (diagonal `+w`, off-diagonal `-w` between the pair)
   and a span term to the right-hand side. The assembly is sparse and
   incremental - weights are updated in place as the flow iterates.
2. **CG solve.** The quadratic is solved by conjugate gradient with a
   selectable preconditioner. Solve *runtime is accumulated and reported*;
   the solve is the inner loop, run every iteration.
3. **Nesterov outer loop with adaptive step.** The step length is
   recomputed per iteration from gradient norms; convergence is judged by
   the overflow triple (overflowing-bin count / max overflow / total
   overflow) against a target density. Early on, an initial-weight
   adjustment phase exists in two flavors - utilization-ratio-driven and
   distance-to-legal-driven - picking the initial step so the first
   iterations neither crawl nor explode.
4. **Congestion feedback.** Between solve rounds, block areas are
   *inflated* in congested regions (bloating factors per region, adjusted
   by overflow), and the expanded-area cells are re-solved. The flow has
   multiple year-tagged strategies for this inflation - it is the piece
   under active commercial development, and the one where we should expect
   to iterate too.
5. **Exit criteria are plural.** Wirelength-ratio flatness, max
   utilization, max macro utilization, congestion, and a hard iteration
   cap; any can fire. The placer does not run to convergence - it runs
   until *good enough* by the metric that matters, then hands off.

Guidance for oxibit: our `placer/` skeleton matches this shape. The
operational lessons are (a) accumulate and report per-phase timing from
day one, (b) make the exit criteria a first-class, individually
toggleable list, and (c) treat congestion bloating as a strategy layer
over the core engine - version it, A/B it, keep it out of the Nesterov
math proper.

## Stage 3: legalization - concurrent, constraint-ordered

The legalization pass is explicitly structured as:

1. Cache the lower-bound placement (the best legal-ish solution so far).
2. Build residual capacity maps from the current placement.
3. **Compute a constraint order** - legalization constraints are solved in
   a computed priority sequence, not spatial order.
4. For each constraint: build a subproblem, solve it, commit it -
   serially, with the commit updating the residual capacity maps the next
   constraint sees.
5. Export the placement; fall back to the cached lower bound if the
   result regressed.

Guidance for oxibit: our `legalise()` is per-site greedy today. The
concepts worth borrowing are the constraint priority ordering, the
residual-capacity accounting between subproblem commits, and the
lower-bound cache as a monotonicity guard (never legalize into something
worse than you started with). All three are pure-function-friendly.

## Stage 4: detailed placement and the packing interplay

The detailed stage reuses the same engine in DP mode, with additional
moves: optimal-location moves per instance, slice re-packing during
placement (the packing manager is *re-invoked* as instances move), and
path smoothing between the two stages. Multiple threads work
SLR-partitioned regions under a lock - device-specific; on Zynq-7 the
analogous boundary is clock regions and PS/PL proximity, likely single-
threaded at our scale.

Guidance for oxibit: defer this stage entirely until the GP engine is
validated. The one durable idea is re-packing during detailed placement
(packing is not one-shot at the end either); it strengthens the case for
the hybrid-loop milestone but changes nothing in the earlier ones.

## Stage 5: timing feedback (the moat)

Incremental wire-delay updates flow back into placement weights and router
decisions every iteration, not at stage boundaries. The timing engine is
differential: only nets touched by moves are re-evaluated. The router
contains an additional learned linear-regression penalty model for branch
congestion - i.e. the commercial flow already treats parts of PNR as a
data problem.

Guidance for oxibit: our measured per-chip timing models are the data
advantage; the STA core in sprint week 5-6 is where it pays off. The
lesson on *structure* is incremental, differential evaluation - never
recompute timing wholesale during the placement loop.

## What this changes in the sprint

Nothing structural - the sprint's milestones stand. What the stage-by-
stage reading adjusts:

- Week 1-2 gains the distance-threshold schedule and distribution
  breaking as explicit, tuned deliverables rather than one-liners.
- Week 3-4 gains the congestion-bloating strategy layer as a separate
  module (versioned, A/B-able) and plural, toggleable exit criteria.
- Week 5-6 is confirmed as differential-only timing updates.
- A new standing item: stage-boundary report events from the first
  commit, in the existing `oxibit-report` schema.
