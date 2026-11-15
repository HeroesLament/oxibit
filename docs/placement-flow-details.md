# Placement flow details: parameters, data structures, and inner loops

Companion to placement-flow-notes.md - the same stages, one level deeper:
parameter groups, data-structure shapes, and inner-loop mechanics.
Rust-side sketches are the oxibit translations. Parameters below are
described functionally; our own knob names live in PlaceParams so A/B
notes and bug reports can name knobs unambiguously.

## A. The tuning surface (what the knob panel looks like)

Hundreds of string-keyed parameters drive the flow.
The ones that matter for the analytic engine:

**Iteration control**
- iteration hard cap and current count, with an ignore-cap escape hatch
- a conjugate-gradient inner iteration cap
- state checkpointing cadence

**Initial weight (the RePlAce mu0 equivalent)**
- min/max bounds
- how many early iterations are spent calibrating it, and a knob that
  extends the calibration phase
- the distance-to-legal band that judges the calibration
- two calibration flavors: utilization-ratio-driven vs
  distance-to-legal-driven, chosen by config
- a rate clamp (default/max/min) on how far the weight may drift per
  adjustment round

**Target density**
- a master target density, plus per-cell-type variants (FF, carry, macro)
- congestion-driven auto density - target density auto-derived from
  congestion measurement
- DP-side mirrors (with congestion, emulation, and large-device variants)

**Epsilon (CG solve quality schedule)**
- the epsilon family (base, init, additive, dynamic, and a device-family
  variant): the residual tolerance is a schedule, not a constant - loose
  early, tight as placement converges, stepped per iteration
- a separate tolerance for the fixed-cell anchor term

**Congestion bloating**
- congestion bloating factors - bloating is *per resource type*
- congestion bloating and overflow feedback
  - feedback: overflow grows bloating, bounded by a max level
- an L2-norm bloating switch - an alternative overflow metric
  (gradient L2 norm) for the bloating response
- bloated-cell ratio, low-util bloat threshold
- rollback hooks - when congestion response fires, wirelength
  weights are rolled back first; bloating is the second response

**Exit criteria**
- WL ratio between recent iterations,
  WL flatness window, utilization coefficient
- effort level - a master dial that shifts several of the above
  at once (the "-effort" knob users see)

**B2B net weighting**
- multiplicative bounds (min/max) on per-net weights
- timing slack turns into net weight
  (slack-based weight adjustment)
- super-long-line variants (SLR; skip on Zynq-7)
- net-degree area adjustment - high-fanout nets inflate their
  cells' areas

**Oxibit translation.** One PlaceParams serde struct, grouped the same
way (iteration / initial_weight / target_density / epsilon / bloating /
exit / net_weight), a master effort: EffortLevel that shifts defaults,
and *every* group emitted into the stage-boundary report events. When we
A/B, the exact parameter vector rides along with the result. Do not
inherit the string-keyed lookup - keep the typed struct and a debug dump.

## B. Data structures of the QP problem

The QP problem decomposes into:

- the flat edge store - reserve/clear/swap and, notably,
  whole-set scaling: scalar scaling of all edge weights at once. That is the
  wirelength-vs-density rebalance lever: one multiply.
- the result vector, with per-problem
  start/end index ranges, an overall
  index map and validity flags -
  i.e. multiple QP subproblems share one solution vector by index
  ranges; cells can be invalidated (moved to fixed) without renumbering.
- star variables: nets above a threshold are modeled as
  star nets with a virtual center variable; the center participates in
  the QP and is mapped back to real pin locations after the solve.
- solution-space stretching and un-stretching: when
  regions are inflated (bloating), the QP coordinate space itself is
  stretched and solutions mapped back, rather than re-modifying areas.
- solver lifecycle:
  solver teardown and anchor reset - the CG solver object is retained across iterations
  and reset (warm restart), anchors (fixed cells) reset separately.
  The manager holds an active-block iterator, initial net-weight
  computation,
  edge-weight adjustment variants, and
  accumulated runtime stats per phase.

**Oxibit translation.** struct B2BProblem { edges: Vec<Edge>,
rhs: Vec<f64>, star_vars: StarVarTable } where Edge { a: u32, b: u32,
w: f64 }; solution as Vec<f64> with an active: Vec<bool> (invalid
entries skipped, not removed); scale_weights(k) on the whole edge set;
star variables as a side table mapping net -> virtual variable index. CG
solver struct persisted across iterations with reset() - warm-started,
not rebuilt.

## C. The B2B assembly inner loop

Per net, the B2B assembly visits pin offsets from the netlist (pin
*offsets* relative to the cell, not absolute), classifies pins into
extreme (L/R for x, B/T for y) and internal, and for each edge:

- diagonal entries +w at each endpoint, off-diagonal -w (stored
  via COO triplet insertion),
- RHS contribution w * span split between endpoints with opposite
  signs,
- the RHS vector updated in place (rhs[i] -= w*dx; rhs[j] += w*dx),
- with three insertion shapes (two-pin, extreme-to-internal,
  extreme-to-extreme) selected by branch.

**Oxibit translation.** One pass per net in build_b2b: compute pin
extremes from netlist pin offsets, emit w per the B2B formula (net
degree d: 2/(d-1) normalized), accumulate into CSR directly (counting
sort by column, no hash map - the sparse storage is exactly a hash-ified
CSR; we can do better deterministically). Star conversion above a net
degree threshold. Parallelize over nets with per-thread partial CSR
buffers, then merge - the same shape the incumbent parallelizes with
task subflows.

## D. Nesterov state machine

The optimizer state machine has initialization, solve, update, restart, parameter setting, search-direction, step-length, variable removal, and accounting. Structure:

- init: computes initial step and coefficients from gradient norms
- solve: runs the update sequence; the search-direction step forms the accelerated direction (beta_k * (x_k - x_{k-1}) - step *
  grad), the standard Nesterov momentum form
- step-length computation is a parallel *reduce* (sum over cells of gradient
  inner products) - the only synchronization point per iteration
- restart resets momentum without losing the current point (used after
  weight rollsbacks and bloating changes: when the objective changes
  under you, momentum from the old objective is stale)
- variable removal - incremental removal of fixed cells from the state
  vectors, no renumbering (same philosophy as Result's validity flags)
- parameter setting plus memory accounting - runtime tunable

**Oxibit translation.** struct Nesterov { x, x_prev, grad: Vec<f64>,
beta, step, coeff: f64 } with init, search_direction() (pure),
step_length(&ctx) -> f64 (parallel sum), restart() and an active mask matching B2BProblem's. Momentum restart on any parameter change
that alters the objective - that is a correctness rule, not an
optimization: otherwise the accelerated direction mixes objectives.

## E. Congestion loop mechanics

Congestion-level computation produces a level
(congested-bin ratio thresholds); overflow-driven bloating scales the per-resource
bloating factors within the bloat-adjust level bound. The
congestion-bloating variants are evolution-tagged
*strategies* over the same primitives (which cells to bloat: by
occupancy? by net degree? gradient-norm-gated?), selected by device
class and effort level.

solution-space stretching and un-stretching is the mechanical trick: inflated areas
stretch the coordinate space; the QP is solved in stretched space and
the solution mapped back - so bloating needs no matrix rebuild, only a
scaling pass and the edge weight multiply.

**Oxibit translation.** Congestion as a per-bin map
(Vec<f32> occupancy / capacity, parallel scatter-add over cells);
bloating as a BloatStrategy trait with an initial implementation
  (occupancy-over-threshold bloat by a factor, capped at
  a max level) and room to grow. Coordinate stretch as a
post-solve affine map per region - no matrix rebuild.

## F. Legalization (concurrent spreading) detail

The legalization entry point:
1. a lower-bound snapshot - snapshot the incoming (illegal but
   low-objective) coordinates; anything worse can be rolled back to here
2. residual-capacity initialization - per-bin capacity minus current occupancy =
   the space each subsequent subproblem may consume
3. constraint ordering - constraints (bins/regions) sorted into
   a priority order (observed to be computed from congestion/packing
   density, not spatial sweep)
4. loop: subproblem build -> subproblem solve ->
   solution commit - each commit decrements the residual
   capacity maps that later constraints consult
5. final export

Inner helpers: overflowed-bin collection and ranking,
fragment ranking (instances ranked per bin; tuples sorted),
in-bin-box spreading (spreading within a bin's box),
fragment location commit, incremental rank maintenance (incremental rank
maintenance as fragments move).

**Oxibit translation.** Our legalise() gains: a best-so-far
monotonicity guard (compare wirelength before commit; roll back
worse), residual capacity accounting instead of free-form greedy, and
overflowed-bin prioritization. Fragment ranking maps to
  (inst, priority) pairs - Abacus-style spreading within the bin box is
  the known-good academic equivalent.

## G. Clustering in the flow

Mid-flow clustering (core and generic variants):
the GP flow re-clusters (compact LUT-FF pairs into movable clusters)
mid-flow, interleaved with spreading iterations. The QP variables are
thus cluster centroids at some phases and raw cells at others.
The super-long variants are SLR-specific
  (skip). Carry-area caching means carry chains get
their area cached once and reused across iterations.

**Oxibit translation.** Mid-flow re-clustering is deferred (one-shot
LUT-FF pairing at pack time first); the design point to keep open is
that the placer's variable set is not the netlist cell set - keep
coordinates and edges indexed by *variable id* with an
assignment: Vec<CellId> indirection, so clustering, star vars, and
SLR-like constructs are all the same mechanism (variables != cells).

## H. The GP/DP mode split

Both modes share the engine; the stage flag selects parameter groups
(the GP and DP parameter groups above), exit criteria, and which
post-passes run. DP runs per-region with slice re-packing during
placement and path smoothing between stages.

**Oxibit translation.** enum Mode { Global, Detail } selecting a
ParamsForMode view over PlaceParams - same engine, no separate
detailed placer until the GP is proven.

## What to build first

1. PlaceParams with the groups above (typed, serde, report-emitted)
2. B2BProblem + CSR assembly from the netlist (C)
3. warm-started CG with the epsilon schedule (A: epsilon group)
4. Nesterov state machine (D) with restart-on-objective-change
5. exit criteria as a Vec<ExitCriterion> evaluated in order (A: exit
   group)
6. bloating strategy v1 + coordinate stretch (E)
7. legalize upgrades: monotonicity guard, residual capacity,
   overflow-first order (F)
