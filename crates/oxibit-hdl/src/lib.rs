//! Verilog front end: source text to a word-level netlist.
//!
//! `expr` parses expressions with IEEE 1364-2005 precedence; `netlist` is
//! the elaborated form, shaped like yosys's RTLIL after `proc`;
//! `yosys_json` writes it in yosys's JSON format so yosys can prove it
//! equivalent to its own reading of the same source.

pub mod ast;
pub mod elab;
pub mod expr;
pub mod netlist;
pub mod parser;
pub mod yosys_json;
