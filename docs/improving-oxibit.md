# Improving oxibit: adding the incumbent's high-performance placement and packing

## The thesis

Oxibit's packer is a declarative rule system (`pack/mod.rs`: `Rule` =
typed port/param rewrites, nextpnr-xilinx lineage) with structural control-set
legality (`ffs.rs` `Ctrl`). That is an improvement over the incumbent's paradigm on
its own terms: legality is one source of truth, statically typed, testable.
What the incumbent has that we lack is *not* better legality - it is the
optimization logic layered on top: scoring, candidate search, and the
iteration structure that ties packing to placement. The plan is to port that
logic as **pure functions over our existing data model**, not to rebuild
the incumbent's 500-symbol god classes.

```
oxibit rules system (declarative legality, typed rewrites)     <- stays
  + the incumbent's optimization logic (pure scoring/selection fns)  <- new layer
    = candidates from rules, choice by score
```

## What the incumbent has that oxibit does not

### 1. The packer gain function (highest priority)

The incumbent has a single merging-score function: how good it is to merge two
instances into one slice. Everything else in the best-choice packer is
machinery around this number:

- logical and physical candidate generation - logical
  (connectivity) and physical (bin-grid distance) neighbors
- neighbor-distance re-tuning - the
  threshold re-tuned per iteration (an iteration schedule schedules
  several iterations with different thresholds)
- the best-choice loop - a priority queue over
  candidates; pop best, merge, update neighbors, repeat
- distribution breaking - a priority
  rule splitting instances back out of over-congested slices
- control-set matching - clustering via
  bipartite matching to maximize clock/CE/SR sharing across conflicting sets

Oxibit today packs in order of appearance. The first concrete milestone of
this effort is `merging_score() -> f64` as a free function over
`(CellId, CellId, &PackCtx)`, with the BestChoice loop around it, A/B'd
against the current packer on wirelength (via `compare.rs` and the existing
report tooling). The score form follows our local analysis.

### 2. The analytic placer

the incumbent's global placement is an ePlace/RePlAce-family engine, confirmed by flow shape and published-symbol lineage:

- a flow driver: entry thunks set a
  GP/DP stage flag and dispatch; the iteration core
  is pre-iteration, iteration body, incremental iteration,
  post-iteration
- the B2B edge builder for the bound2bound wirelength model
- the QP solve via a conjugate-gradient solver
- a Nesterov optimizer with init/solve/step-length entry points
  - Nesterov acceleration with adaptive step (RePlAce, Cheng et al. 2019)
- electrostatic density via FFT Poisson solve (FFTW)
- concurrent spreading legalization (bin-box spreading)
- incremental wire-delay updates feeding
  placement weights each iteration

Scope reduction for Zynq-7000: no SLRs, no macro placement worth the name,
no clock-region spreading. That deletes roughly a third of the flow driver
(SLR cut, SLR guidance, clock-region placers all
  go). GP and DP share one engine behind a mode flag - we can do the same
with an enum.

The published algorithms (B2B quadratic + CG + Nesterov with the RePlAce
step rule) are implementable from the papers; production parameters are
tuned (initial step, target-density schedule, exit
criteria 
  (`WL-ratio exit`, `WLFlat`, `MaxUtil` thresholds), which
are the constants that separate a demo from a production placer.

### 3. Timing-driven everything

The router library contains `an ML branch-penalty regression` (an ML
model inside the router) and `timing-change callback` callbacks -
the incumbent pushes incremental timing updates into routing and placement
decisions. Oxibit's advantage: `oxibit-timing` characterizes *measured*
per-chip delays (phase-sweep edge fits) rather than datasheet tables. Wiring
those measured delays into placement weights is not matching the incumbent - it is
potentially beating its timing accuracy on characterized parts. The gap is
that oxibit-timing is characterization, not STA; an STA engine that consumes
oxibit-timing models and feeds both the placer (the incremental wire-delay update equivalent) and the router is the third milestone.

## Architecture: composition, not porting

Every the incumbent piece lands as a pure function:

| the incumbent | Oxibit landing spot | Shape |
|---|---|---|
| the merging-score function | `pack/luts.rs`-style modules | `fn merging_score(a, b, &PackCtx) -> f64` |
| Best-choice kernel | new `pack/bestchoice.rs` | PQ over scored pairs; serial pops |
| bipartite ctrl-set matching | beside `ffs.rs::Ctrl` | matching over the same constraint data |
| B2B + Nesterov + CG | new `placer/` module | free functions over netlist + coords |
| concurrent spreading legalization | optional post-placer | replaceable with Abacus-style spreading |
| incremental timing updates | `oxibit-timing` consumer | measured delays -> weights |

The hybrid loop (the incumbent packs *concurrently with* early placement, so its
physical-neighbor term is real distance) decomposes into two pure functions
composed: initial spread -> pack with physical term -> place -> optional
re-pack. Each stage testable alone; a crude initial placement (fanout
centroid spread) suffices for the packer's distance term to start.

## Concurrency

the incumbent uses a task-graph runtime throughout and a a mutex for SLR
thread state. Oxibit reaches for Rayon. the incumbent's iteration structure is
phased, which is the textbook Rayon pattern and needs no mutexes:

1. **Parallel scoring** - read-only over the netlist: `PackCtx<'a>` holding
   `&'a Netlist` + positions + fanouts (`Sync`), `candidates.par_iter()
   .map(|(a,b)| merging_score(a, b, &ctx))`.
2. **Serial decisions** - PQ pops are a global priority and cannot be
   parallelized anyway (PQ pops are serial there too):
   `&mut Packer`.
3. **Parallel cleanup** - disjoint by construction:
   `partitions.par_iter_mut().for_each(...)`.

In the placer, the Nesterov inner loops are per-cell embarrassingly parallel
(`coords.par_iter_mut().zip(grad.par_iter())` with the netlist as `&`);
the only synchronization is the step-length sum (Rayon `sum()`). Borrow
checker proves disjointness statically where the incumbent needed a mutex.
Granularity: partition-grain in the packer (serial
priority-queue cores), cell-grain in the placer. `oxibit-prof`'s `#[hot]` spans compose
correctly across Rayon threads, so profiling needs no changes.

## Milestones

1. `merging_score()` + BestChoice loop in the packer; A/B on wirelength
   against order-of-appearance. *
   
   
2. `placer/` skeleton: B2B quadratic + CG (faer or sprs) + RePlAce Nesterov
   from the published equations; 
   
   tuned parameters.
3. STA engine over `oxibit-timing` measured models feeding placement weights
   and (later) router rip-up decisions; `the router ML model`
   equivalents only if the data justifies it.
4. Hybrid loop: placement feeding the packer's physical term, converging
   iterations - composed pure functions, Rayon between the phases.

## Validation

Same culture as the README's bitstream checks: run the incumbent's placer on the
same designs, dump cell locations, compare wirelength ratio (placement is
not byte-exact; same-parameter B2B+Nesterov should land within a few
percent HPWL) via `oxibit-place/compare.rs` and the report tooling.

## Sources

- RePlAce: Cheng, Kahng, Kang, Wang, TCAD 2019 (Nesterov + step rule)
- SimPL: Kim, Lee, Markov, DAC 2012
- ePlace: Lu et al. (electrostatics)
- Best-choice / partitioned packing: T-VPack, iRIP, Rottler et al.
