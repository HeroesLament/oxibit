//! Combinational equivalence checking.
//!
//! The input is an AIGER miter whose outputs should all be constant (yosys
//! builds it from the gold and gate netlists with name-matched registers
//! cut; see the oxibit equivalence scripts). `sweep` decides each output:
//! proven, failed (a counterexample exists), or open (budget ran out).
//! Pure Rust, with batsat as the SAT solver.

pub mod aiger;
pub mod sweep;
