//! Verilog-2005 module parser: source text to `ast::ModuleAst`.
//!
//! Hand-written recursive descent over `expr::tokenize`'s tokens, with
//! expressions parsed by `expr::Parser` (precedence-correct). Covers the
//! module structure the EHDL emitter writes and the usual hand-written
//! forms: ANSI and non-ANSI port lists, `parameter`/`localparam`, `wire`/
//! `reg` declarations with ranges, arrays and initialisers, `assign`,
//! `always` with `@(*)`, `@*` or edge lists, `initial`, `begin`/`end`,
//! `if`/`else`, `case`/`casez`/`casex`, blocking and non-blocking
//! assignments, system task calls, and instances with `#(...)` parameter
//! and named port lists. Anything else is an error naming the token.

use crate::ast::{CaseKind, Item, ModuleAst, Range, Sens, Stmt};
use crate::expr::{self, Expr, ParseError, Parser, Tok};
use crate::netlist::Dir;

type R<T> = Result<T, ParseError>;

fn perr<T>(m: impl Into<String>) -> R<T> {
    Err(ParseError(m.into()))
}

struct P {
    p: Parser,
}

impl P {
    fn peek(&self) -> Option<&Tok> {
        self.p.peek()
    }
    fn is_kw(&self, k: &str) -> bool {
        matches!(self.peek(), Some(Tok::Ident(s)) if s == k)
    }
    fn is_op(&self, o: &str) -> bool {
        self.p.is_op(o)
    }
    fn bump(&mut self) {
        self.p.pos += 1;
    }
    fn eat_kw(&mut self, k: &str) -> bool {
        if self.is_kw(k) {
            self.bump();
            true
        } else {
            false
        }
    }
    fn eat_op(&mut self, o: &str) -> bool {
        if self.is_op(o) {
            self.bump();
            true
        } else {
            false
        }
    }
    fn expect_op(&mut self, o: &str) -> R<()> {
        self.p.expect(o)
    }
    fn expect_kw(&mut self, k: &str) -> R<()> {
        if self.eat_kw(k) { Ok(()) } else { perr(format!("expected {k}, found {:?}", self.peek())) }
    }
    fn ident(&mut self) -> R<String> {
        match self.peek().cloned() {
            Some(Tok::Ident(s)) => {
                self.bump();
                Ok(s)
            }
            t => perr(format!("expected an identifier, found {t:?}")),
        }
    }
    fn expr(&mut self) -> R<Expr> {
        self.p.expr()
    }
    /// An lvalue: an identifier with selects, or a concatenation. Parsed
    /// as a unary-level expression so `a <= b` is not read as a compare.
    fn lvalue(&mut self) -> R<Expr> {
        if self.is_op("{") {
            self.bump();
            let mut items = vec![self.lvalue()?];
            while self.eat_op(",") {
                items.push(self.lvalue()?);
            }
            self.expect_op("}")?;
            return Ok(Expr::Concat(items));
        }
        let name = self.ident()?;
        let mut e = Expr::Ident(name);
        while self.eat_op("[") {
            let a = self.expr()?;
            if self.eat_op(":") {
                let b = self.expr()?;
                self.expect_op("]")?;
                e = Expr::Range(Box::new(e), Box::new(a), Box::new(b));
            } else if self.is_op("+:") || self.is_op("-:") {
                let up = self.is_op("+:");
                self.bump();
                let w = self.expr()?;
                self.expect_op("]")?;
                e = Expr::IndexedRange { e: Box::new(e), base: Box::new(a), width: Box::new(w), up };
            } else {
                self.expect_op("]")?;
                e = Expr::Index(Box::new(e), Box::new(a));
            }
        }
        Ok(e)
    }
    fn range(&mut self) -> R<Option<Range>> {
        if !self.eat_op("[") {
            return Ok(None);
        }
        let msb = self.expr()?;
        self.expect_op(":")?;
        let lsb = self.expr()?;
        self.expect_op("]")?;
        Ok(Some(Range { msb, lsb }))
    }
}

/// Parse every module in `src`.
pub fn parse(src: &str) -> R<Vec<ModuleAst>> {
    let toks = expr::tokenize(src)?;
    let mut p = P { p: Parser::new(toks) };
    let mut out = Vec::new();
    while p.peek().is_some() {
        if p.eat_kw("module") {
            out.push(module(&mut p)?);
        } else {
            return perr(format!("expected module, found {:?}", p.peek()));
        }
    }
    Ok(out)
}

fn dir_of(k: &str) -> Option<Dir> {
    match k {
        "input" => Some(Dir::Input),
        "output" => Some(Dir::Output),
        "inout" => Some(Dir::Inout),
        _ => None,
    }
}

fn module(p: &mut P) -> R<ModuleAst> {
    let name = p.ident()?;
    let mut items = Vec::new();
    let mut ports = Vec::new();
    // #(parameter A = 1, ...)
    if p.eat_op("#") {
        p.expect_op("(")?;
        while !p.is_op(")") {
            p.eat_kw("parameter");
            p.eat_kw("integer");
            let _ = p.range()?;
            let n = p.ident()?;
            p.expect_op("=")?;
            let v = p.expr()?;
            items.push(Item::Param { name: n, value: v, local: false });
            if !p.eat_op(",") {
                break;
            }
        }
        p.expect_op(")")?;
    }
    // Port list: names (non-ANSI) or declarations (ANSI).
    if p.eat_op("(") {
        let mut cur: Option<(Dir, bool, bool, Option<Range>)> = None;
        while !p.is_op(")") {
            if let Some(Tok::Ident(k)) = p.peek().cloned() {
                if let Some(d) = dir_of(&k) {
                    p.bump();
                    let is_reg = p.eat_kw("reg");
                    p.eat_kw("wire");
                    let signed = p.eat_kw("signed");
                    let r = p.range()?;
                    cur = Some((d, is_reg, signed, r));
                }
            }
            let n = p.ident()?;
            if let Some((d, is_reg, signed, r)) = &cur {
                items.push(Item::Decl {
                    name: n.clone(),
                    dir: Some(*d),
                    is_reg: *is_reg,
                    signed: *signed,
                    range: r.clone(),
                    array: None,
                    init: None,
                    attrs: vec![],
                });
            }
            ports.push(n);
            if !p.eat_op(",") {
                break;
            }
        }
        p.expect_op(")")?;
    }
    p.expect_op(";")?;
    loop {
        if p.eat_kw("endmodule") {
            break;
        }
        item(p, &mut items)?;
    }
    Ok(ModuleAst { name, ports, items })
}

fn item(p: &mut P, items: &mut Vec<Item>) -> R<()> {
    let Some(Tok::Ident(k)) = p.peek().cloned() else { return perr(format!("unexpected {:?} in module body", p.peek())) };
    match k.as_str() {
        "input" | "output" | "inout" | "wire" | "reg" | "integer" => {
            p.bump();
            let dir = dir_of(&k);
            let mut is_reg = k == "reg" || k == "integer";
            if dir.is_some() {
                is_reg |= p.eat_kw("reg");
                p.eat_kw("wire");
            }
            let signed = p.eat_kw("signed") || k == "integer";
            let range = if k == "integer" {
                let n = |v: i64| Expr::Number { width: None, signed: true, bits: (0..32).map(|i| Some((v >> i) & 1 == 1)).collect(), z: vec![] };
                Some(Range { msb: n(31), lsb: n(0) })
            } else {
                p.range()?
            };
            loop {
                let name = p.ident()?;
                let array = p.range()?;
                let init = if p.eat_op("=") { Some(p.expr()?) } else { None };
                items.push(Item::Decl { name, dir, is_reg, signed, range: range.clone(), array, init, attrs: vec![] });
                if !p.eat_op(",") {
                    break;
                }
            }
            p.expect_op(";")
        }
        "parameter" | "localparam" => {
            p.bump();
            p.eat_kw("integer");
            p.eat_kw("signed");
            let _ = p.range()?;
            loop {
                let name = p.ident()?;
                p.expect_op("=")?;
                let value = p.expr()?;
                items.push(Item::Param { name, value, local: k == "localparam" });
                if !p.eat_op(",") {
                    break;
                }
            }
            p.expect_op(";")
        }
        "assign" => {
            p.bump();
            loop {
                let lhs = p.lvalue()?;
                p.expect_op("=")?;
                let rhs = p.expr()?;
                items.push(Item::Assign { lhs, rhs });
                if !p.eat_op(",") {
                    break;
                }
            }
            p.expect_op(";")
        }
        "always" => {
            p.bump();
            p.expect_op("@")?;
            let sens = sensitivity(p)?;
            let body = stmt(p)?;
            items.push(Item::Always { sens, body });
            Ok(())
        }
        "initial" => {
            p.bump();
            let body = stmt(p)?;
            items.push(Item::Initial(body));
            Ok(())
        }
        "generate" | "endgenerate" | "genvar" | "function" | "task" => perr(format!("{k} is not supported yet")),
        _ => instance(p, items),
    }
}

fn sensitivity(p: &mut P) -> R<Sens> {
    if p.eat_op("*") {
        return Ok(Sens::Comb);
    }
    p.expect_op("(")?;
    if p.eat_op("*") {
        p.expect_op(")")?;
        return Ok(Sens::Comb);
    }
    let mut edges = Vec::new();
    let mut plain = false;
    loop {
        if p.eat_kw("posedge") {
            edges.push((true, p.expr()?));
        } else if p.eat_kw("negedge") {
            edges.push((false, p.expr()?));
        } else {
            let _ = p.expr()?;
            plain = true;
        }
        if !(p.eat_kw("or") || p.eat_op(",")) {
            break;
        }
    }
    p.expect_op(")")?;
    match (plain, edges.is_empty()) {
        (_, true) => Ok(Sens::Comb),
        (false, false) => Ok(Sens::Edges(edges)),
        (true, false) => perr("mixed edge and level sensitivity"),
    }
}

fn stmt(p: &mut P) -> R<Stmt> {
    if p.eat_op(";") {
        return Ok(Stmt::Empty);
    }
    if p.eat_kw("begin") {
        if p.eat_op(":") {
            p.ident()?;
        }
        let mut v = Vec::new();
        while !p.eat_kw("end") {
            v.push(stmt(p)?);
        }
        return Ok(Stmt::Block(v));
    }
    if p.eat_kw("if") {
        p.expect_op("(")?;
        let cond = p.expr()?;
        p.expect_op(")")?;
        let then = Box::new(stmt(p)?);
        let els = if p.eat_kw("else") { Some(Box::new(stmt(p)?)) } else { None };
        return Ok(Stmt::If { cond, then, els });
    }
    for (kw, kind) in [("case", CaseKind::Case), ("casez", CaseKind::CaseZ), ("casex", CaseKind::CaseX)] {
        if p.eat_kw(kw) {
            p.expect_op("(")?;
            let subject = p.expr()?;
            p.expect_op(")")?;
            let mut arms = Vec::new();
            let mut default = None;
            while !p.eat_kw("endcase") {
                if p.eat_kw("default") {
                    p.eat_op(":");
                    default = Some(Box::new(stmt(p)?));
                    continue;
                }
                let mut labels = vec![p.expr()?];
                while p.eat_op(",") {
                    labels.push(p.expr()?);
                }
                p.expect_op(":")?;
                arms.push((labels, stmt(p)?));
            }
            return Ok(Stmt::Case { kind, subject, arms, default });
        }
    }
    if let Some(Tok::Ident(n)) = p.peek().cloned() {
        if n.starts_with('$') {
            p.bump();
            let mut args = Vec::new();
            if p.eat_op("(") {
                if !p.is_op(")") {
                    loop {
                        args.push(p.expr()?);
                        if !p.eat_op(",") {
                            break;
                        }
                    }
                }
                p.expect_op(")")?;
            }
            p.expect_op(";")?;
            return Ok(Stmt::SysCall(n, args));
        }
    }
    let lhs = p.lvalue()?;
    let blocking = if p.eat_op("=") {
        true
    } else if p.eat_op("<=") {
        false
    } else {
        return perr(format!("expected = or <= after {lhs:?}, found {:?}", p.peek()));
    };
    let rhs = p.expr()?;
    p.expect_op(";")?;
    Ok(Stmt::Assign { lhs, rhs, blocking })
}

fn instance(p: &mut P, items: &mut Vec<Item>) -> R<()> {
    let ty = p.ident()?;
    let mut params = Vec::new();
    if p.eat_op("#") {
        p.expect_op("(")?;
        while !p.is_op(")") {
            p.expect_op(".")?;
            let n = p.ident()?;
            p.expect_op("(")?;
            let v = p.expr()?;
            p.expect_op(")")?;
            params.push((n, v));
            if !p.eat_op(",") {
                break;
            }
        }
        p.expect_op(")")?;
    }
    let name = p.ident()?;
    p.expect_op("(")?;
    let mut conns = Vec::new();
    while !p.is_op(")") {
        if !p.eat_op(".") {
            return perr(format!("{ty} {name}: positional port connections are not supported yet"));
        }
        let port = p.ident()?;
        p.expect_op("(")?;
        let e = if p.is_op(")") { None } else { Some(p.expr()?) };
        p.expect_op(")")?;
        conns.push((port, e));
        if !p.eat_op(",") {
            break;
        }
    }
    p.expect_op(")")?;
    p.expect_op(";")?;
    items.push(Item::Instance { ty, name, params, conns });
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn small_module() {
        let src = r#"
module top (a, b, clk, y);
  input [3:0] a;
  input [3:0] b;
  input clk;
  output reg [4:0] y;
  localparam K = 2'd3;
  wire [4:0] s;
  reg [7:0] mem [0:15];
  assign s = a + b;
  always @(posedge clk) begin
    if (s == 5'd0) y <= 5'd1;
    else y <= s;
  end
  initial y = 5'd0;
  BUFG g (.I(clk), .O());
endmodule
"#;
        let m = parse(src).unwrap();
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].ports, vec!["a", "b", "clk", "y"]);
        assert_eq!(m[0].items.len(), 11);
    }
}
