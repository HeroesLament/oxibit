//! The module-level syntax elaboration works from: declarations,
//! continuous assignments, processes, instances. Whatever parses the
//! source fills these in; expressions are `expr::Expr`.

use crate::expr::Expr;
use crate::netlist::Dir;

#[derive(Debug, Clone, PartialEq)]
pub struct Range {
    pub msb: Expr,
    pub lsb: Expr,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Item {
    /// `input`, `output`, `wire`, `reg`, with optional packed range and
    /// unpacked (memory) range.
    Decl {
        name: String,
        dir: Option<Dir>,
        is_reg: bool,
        signed: bool,
        range: Option<Range>,
        array: Option<Range>,
        init: Option<Expr>,
        attrs: Vec<(String, Option<Expr>)>,
    },
    Param { name: String, value: Expr, local: bool },
    Assign { lhs: Expr, rhs: Expr },
    Always { sens: Sens, body: Stmt },
    Initial(Stmt),
    Instance { ty: String, name: String, params: Vec<(String, Expr)>, conns: Vec<(String, Option<Expr>)> },
}

#[derive(Debug, Clone, PartialEq)]
pub enum Sens {
    /// `@(*)` or a list of plain signals.
    Comb,
    /// `@(posedge a or negedge b ...)`.
    Edges(Vec<(bool, Expr)>),
}

#[derive(Debug, Clone, PartialEq)]
pub enum Stmt {
    Block(Vec<Stmt>),
    /// `lhs = rhs` (`blocking`) or `lhs <= rhs`.
    Assign { lhs: Expr, rhs: Expr, blocking: bool },
    If { cond: Expr, then: Box<Stmt>, els: Option<Box<Stmt>> },
    /// `case`, `casez` (`z`), `casex` (`x`).
    Case { kind: CaseKind, subject: Expr, arms: Vec<(Vec<Expr>, Stmt)>, default: Option<Box<Stmt>> },
    /// `$readmemh("file", mem)` and the like, kept by name.
    SysCall(String, Vec<Expr>),
    Empty,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CaseKind {
    Case,
    CaseZ,
    CaseX,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ModuleAst {
    pub name: String,
    pub ports: Vec<String>,
    pub items: Vec<Item>,
}
