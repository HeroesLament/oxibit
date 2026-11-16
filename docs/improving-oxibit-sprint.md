# Improving oxibit sprint

Goal: close the placement and packing performance gap against the commercial
flow, without giving up oxibit's architecture. Success is measured the same
way the README measures bitstreams: on real designs, against the incumbent,
with numbers. Full background in `improving-oxibit.md`; this document is
the execution plan.

Scope guard (Zynq-7000, no SLR, no macro placement, no clock-region
spreading): every item below is cut to what the target device needs.

## Week 1-2: the packer gain function

Oxibit packs in order of appearance; the incumbent scores every candidate pair and
takes the best. This sprint item adds scoring to our rules system - the
rules stay the source of legality, the score only ranks legal candidates.

- [ ] `pack/PackCtx` - read-only context struct: netlist borrow, net
      fanouts, shared-net counts, per-cell positions (from a coarse spread
      or legalization), the per-BEL-type capacity tables we already have
- [ ] `pack::merging_score(a, b, &PackCtx) -> f64` - logical term
      (connectivity: shared nets, weighted by fanout) plus physical term
      (bin-grid distance, weighted by an `neighbor-distance threshold` threshold)
- [ ] `pack/bestchoice.rs` - the loop: score all legal candidate pairs in
      parallel, insert into a priority structure, pop best, merge, update
      neighbor scores, repeat until no positive-gain pairs remain
- [ ] distribution breaking: post-pass that splits instances out of
      over-congested slices and re-queues them, with a priority function
- [ ] control-set assignment: keep `ffs.rs::Ctrl` as the constraint, add
      a matching step that maximizes sharing when sets conflict

Deliverable: a `--packer {rules,rules+score}` switch in oxibit-place.
Validation: wirelength after placement (same placer both arms) on the
existing test designs, via `compare.rs`; target is a measurable win over
order-of-appearance, zero packing regressions (same utilization, no
illegal slices - the rules guarantee legality either way).

## Week 3-4: the analytic placer skeleton

Oxibit has no global placer; placement today is import + legalize. This
item stands up the classic engine: B2B wirelength quadratic, conjugate
gradient solve, RePlAce-style Nesterov acceleration with overflow-driven
step adaptation.

- [ ] `placer/` module: `PlacementState` (coords, instance bounds),
      `B2BGraph` (per-net pin pair edges with weights), bin grid
- [ ] B2B model build from the netlist (two-pin, star, and extreme-edge
      cases - the same case split as the packing neighbor generation)
- [ ] CG solve of the quadratic per iteration; `faer` or `sprs` sparse
      backend behind a small trait so it is swappable
- [ ] Nesterov state: initial step from the wirelength gradient norms,
      adaptive step rule, iteration counter, the overflow triple
      (count / max / sum over bins) as the convergence signal
- [ ] exit criteria: wirelength-ratio, flatness, max-utilization -
      tunable thresholds, defaults from the published settings
- [ ] legalization handoff: the spreader output feeds the existing
      `legalise()` path unchanged

Deliverable: `oxibit place --gp` producing coordinates instead of importing
them. Validation: HPWL within a few percent of the incumbent's global
placement on the same designs (placement is not byte-exact by nature; this
is the honest comparison bar). Concurrency from day one: per-cell parallel
gradient and coordinate updates, serial step-length reduce - the phased
Rayon pattern, no locks.

## Week 5-6: measured timing into placement

The advantage nobody else has: oxibit-timing's phase-sweep edge fits are
per-chip measured delays, not datasheet tables. Wire them into placement
so timing-critical nets carry bigger B2B weights, updated incrementally
during the Nesterov loop.

- [ ] small STA core over the synthesized netlist (arrival/required times,
      slack per net) consuming oxibit-timing models
- [ ] incremental update path: delays re-estimated after each placement
      iteration only for moved cells' nets
- [ ] criticality -> B2B weight function (the standard exponential
      criticality form, exponent tunable)

Deliverable: `--timing-driven` on the placer. Validation: post-route
slack (and clock frequency) on the same designs, rules-only vs
timing-driven arms.

## Week 7-8: the hybrid loop

the incumbent packs concurrently with early placement so its physical term is real
distance. We compose the two as pure functions instead:

- [ ] initial spread (fanout-centroid) -> pack with physical term ->
      place -> re-pack (one refinement round to start, bounded)
- [ ] the packer's `neighbor-distance threshold` schedule across rounds, mirroring
      best-choice iteration practice
- [ ] end-to-end: synth -> pack -> place -> route on the demo designs,
      numbers against the incumbent in the report format

Deliverable: the full loop integrated in oxibit-place, `oxibit-prof`
flamegraphs confirming the parallel phases scale.

## Standing practices

- Every optimization piece is a pure function over the existing data model;
  the `Rule` engine and `Ctrl` legality are never bypassed.
- Rayon between phases: read-only scoring in parallel (`&PackCtx`),
  serial priority decisions, disjoint cleanup (`par_iter_mut`). No
  `Arc<Mutex<>>` anywhere in the placer/packer.
- `#[hot]` spans on every phase; performance is part of the definition of
  done, not an afterthought.
- Each milestone A/Bs against the previous one; a regression blocks the
  merge.
