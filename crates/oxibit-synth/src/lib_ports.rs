//! Port directions of primitive modules, scanned from Verilog library
//! sources (yosys's `cells_sim.v` / `cells_xtra.v`, or a stub file).
//!
//! Only declarations are read: `input` / `output` / `inout` followed by
//! names, in ANSI headers or in the module body. Everything else is
//! skipped, so behavioural bodies and `specify` blocks do not matter.

use std::collections::HashMap;

use oxibit_hdl::netlist::Dir;

pub type PortDirs = HashMap<String, HashMap<String, Dir>>;

fn tokens(src: &str) -> Vec<&str> {
    let b = src.as_bytes();
    let mut out = Vec::new();
    let mut i = 0;
    while i < b.len() {
        let c = b[i];
        if c.is_ascii_whitespace() {
            i += 1;
        } else if b[i..].starts_with(b"//") {
            while i < b.len() && b[i] != b'\n' {
                i += 1;
            }
        } else if b[i..].starts_with(b"/*") {
            i += 2;
            while i + 1 < b.len() && !b[i..].starts_with(b"*/") {
                i += 1;
            }
            i += 2;
        } else if b[i..].starts_with(b"(*") {
            i += 2;
            while i + 1 < b.len() && !b[i..].starts_with(b"*)") {
                i += 1;
            }
            i += 2;
        } else if c == b'"' {
            i += 1;
            while i < b.len() && b[i] != b'"' {
                i += 1;
            }
            i += 1;
        } else if c.is_ascii_alphanumeric() || c == b'_' || c == b'$' || c == b'\\' || c == b'`' {
            let s = i;
            while i < b.len() && (b[i].is_ascii_alphanumeric() || b[i] == b'_' || b[i] == b'$' || b[i] == b'\\' || b[i] == b'`') {
                i += 1;
            }
            out.push(&src[s..i]);
        } else {
            out.push(&src[i..i + 1]);
            i += 1;
        }
    }
    out
}

pub fn scan(src: &str) -> PortDirs {
    let t = tokens(src);
    let mut out = PortDirs::new();
    let mut i = 0;
    while i < t.len() {
        if t[i] != "module" {
            i += 1;
            continue;
        }
        let name = t.get(i + 1).copied().unwrap_or("").to_string();
        let mut ports = HashMap::new();
        i += 2;
        let mut dir: Option<Dir> = None;
        let mut depth = 0i32; // [] nesting
        while i < t.len() && t[i] != "endmodule" {
            match t[i] {
                "input" => dir = Some(Dir::Input),
                "output" => dir = Some(Dir::Output),
                "inout" => dir = Some(Dir::Inout),
                "[" => depth += 1,
                "]" => depth -= 1,
                ";" | ")" => dir = None,
                "wire" | "reg" | "signed" | "logic" | "integer" | "," => {}
                tok if depth == 0 && dir.is_some() => {
                    let ok = tok.as_bytes()[0].is_ascii_alphabetic() || tok.starts_with('_') || tok.starts_with('\\');
                    if ok {
                        ports.insert(tok.trim_start_matches('\\').to_string(), dir.unwrap());
                    } else if tok == "=" || tok == "(" {
                        dir = None;
                    }
                }
                _ => {}
            }
            i += 1;
        }
        out.insert(name, ports);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ansi_and_body() {
        let d = scan(
            "(* abc9_lut=1 *) module LUT2(output O, input I0, I1);\n parameter [3:0] INIT = 0;\n assign O = INIT[{I1,I0}];\nendmodule\n\
             module FDRE (Q, C, CE, D, R);\n output reg Q; input C; input [1:0] CE, D; input R;\n always @(posedge C) Q <= D;\nendmodule",
        );
        assert_eq!(d["LUT2"]["O"], Dir::Output);
        assert_eq!(d["LUT2"]["I1"], Dir::Input);
        assert_eq!(d["FDRE"]["Q"], Dir::Output);
        assert_eq!(d["FDRE"]["D"], Dir::Input);
        assert_eq!(d["FDRE"].len(), 5);
    }
}
