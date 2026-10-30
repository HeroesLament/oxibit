//! Verilog expressions: tokenizer and precedence-correct parser.
//!
//! sv-parser builds binary expressions without operator precedence (its
//! `expression_binary` is `expression op expression`, so `a + b * c` and
//! `a == b ? c : d` come back in whatever shape the packrat parse found).
//! Expressions are therefore re-parsed here from their source text, with
//! the IEEE 1364-2005 table 5-4 precedences: unary binds tightest, then
//! `**`, `* / %`, `+ -`, shifts, relational, equality, `&`, `^ ~^`, `|`,
//! `&&`, `||`, and `?:` (right associative) loosest. Binary operators are
//! left associative.

use std::fmt;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Tok {
    Ident(String),
    /// Sized or unsized literal: width (None = unsized, 32 bits), signed,
    /// bits LSB first (`None` = x or z).
    Number { width: Option<u32>, signed: bool, bits: Vec<Option<bool>>, z: Vec<bool> },
    Str(String),
    /// A real literal, as written (`200.0`, `1.5e3`).
    Real(String),
    Op(&'static str),
}

#[derive(Debug, Clone, PartialEq)]
pub enum Expr {
    Ident(String),
    Number { width: Option<u32>, signed: bool, bits: Vec<Option<bool>> },
    Unary(&'static str, Box<Expr>),
    Binary(&'static str, Box<Expr>, Box<Expr>),
    Ternary(Box<Expr>, Box<Expr>, Box<Expr>),
    /// `{a, b, c}`, most significant first, as written.
    Concat(Vec<Expr>),
    /// `{n{a, b}}`.
    Repeat(Box<Expr>, Vec<Expr>),
    /// `x[i]`.
    Index(Box<Expr>, Box<Expr>),
    /// `x[msb:lsb]`.
    Range(Box<Expr>, Box<Expr>, Box<Expr>),
    /// `x[base +: width]` (`up`) or `x[base -: width]`.
    IndexedRange { e: Box<Expr>, base: Box<Expr>, width: Box<Expr>, up: bool },
    /// `$signed(x)`, `$clog2(x)`, `f(x, y)`.
    Call(String, Vec<Expr>),
    /// A string literal (parameter values, `$readmemh` file names).
    Str(String),
    /// A real literal, as written.
    Real(String),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParseError(pub String);

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

type R<T> = Result<T, ParseError>;

/// Longest match first.
const OPS: [&str; 46] = [
    "<<<", ">>>", "===", "!==", "~&", "~|", "~^", "^~", "**", "<<", ">>", "<=", ">=", "==", "!=", "&&", "||", "+:", "-:", "+", "-",
    "*", "/", "%", "<", ">", "!", "~", "&", "|", "^", "?", ":", "(", ")", "[", "]", "{", "}", ",", "=", ";", "@", "#", ".", "'",
];

fn digit_bits(c: char, base: u32) -> R<Vec<Option<bool>>> {
    let per = match base {
        2 => 1,
        8 => 3,
        16 => 4,
        _ => unreachable!(),
    };
    match c.to_ascii_lowercase() {
        'x' | 'z' | '?' => Ok(vec![None; per]),
        d => {
            let v = d.to_digit(base).ok_or_else(|| ParseError(format!("bad digit {d} for base {base}")))?;
            Ok((0..per).map(|i| Some((v >> i) & 1 == 1)).collect())
        }
    }
}

/// Bits of an unbounded decimal string, LSB first.
fn decimal_bits(s: &str) -> R<Vec<Option<bool>>> {
    if s.chars().any(|c| matches!(c.to_ascii_lowercase(), 'x' | 'z' | '?')) {
        return Ok(vec![None]);
    }
    let mut digits: Vec<u32> = s.chars().map(|c| c.to_digit(10).ok_or_else(|| ParseError(format!("bad decimal {s}")))).collect::<R<_>>()?;
    let mut bits = Vec::new();
    while digits.iter().any(|&d| d != 0) {
        let mut rem = 0;
        for d in digits.iter_mut() {
            let v = rem * 10 + *d;
            *d = v / 2;
            rem = v % 2;
        }
        bits.push(Some(rem == 1));
    }
    if bits.is_empty() {
        bits.push(Some(false));
    }
    Ok(bits)
}

pub fn tokenize(src: &str) -> R<Vec<Tok>> {
    let c: Vec<char> = src.chars().collect();
    let mut i = 0;
    let mut out = Vec::new();
    while i < c.len() {
        let ch = c[i];
        if ch.is_whitespace() {
            i += 1;
            continue;
        }
        // Comments and attributes (* ... *) are skipped.
        if ch == '/' && c.get(i + 1) == Some(&'/') {
            while i < c.len() && c[i] != '\n' {
                i += 1;
            }
            continue;
        }
        if ch == '/' && c.get(i + 1) == Some(&'*') {
            i += 2;
            while i + 1 < c.len() && !(c[i] == '*' && c[i + 1] == '/') {
                i += 1;
            }
            i += 2;
            continue;
        }
        if ch == '(' && c.get(i + 1) == Some(&'*') && c.get(i + 2) != Some(&')') {
            i += 2;
            while i + 1 < c.len() && !(c[i] == '*' && c[i + 1] == ')') {
                i += 1;
            }
            i += 2;
            continue;
        }
        if ch == '"' {
            let mut s = String::new();
            i += 1;
            while i < c.len() && c[i] != '"' {
                if c[i] == '\\' && i + 1 < c.len() {
                    i += 1;
                }
                s.push(c[i]);
                i += 1;
            }
            i += 1;
            out.push(Tok::Str(s));
            continue;
        }
        if ch.is_ascii_alphabetic() || ch == '_' || ch == '$' {
            let st = i;
            while i < c.len() && (c[i].is_ascii_alphanumeric() || c[i] == '_' || c[i] == '$') {
                i += 1;
            }
            out.push(Tok::Ident(c[st..i].iter().collect()));
            continue;
        }
        if ch == '\\' {
            let st = i + 1;
            while i < c.len() && !c[i].is_whitespace() {
                i += 1;
            }
            out.push(Tok::Ident(c[st..i].iter().collect()));
            continue;
        }
        // Numbers: [size]'[s]base digits, or a plain decimal.
        if ch.is_ascii_digit() || (ch == '\'' && c.get(i + 1).is_some_and(|n| "sSbBoOdDhH".contains(*n))) {
            let mut size = None;
            if ch.is_ascii_digit() {
                let st = i;
                while i < c.len() && (c[i].is_ascii_digit() || c[i] == '_') {
                    i += 1;
                }
                let dec: String = c[st..i].iter().filter(|&&x| x != '_').collect();
                let mut j = i;
                while j < c.len() && c[j].is_whitespace() {
                    j += 1;
                }
                // A real: digits '.' digits, or an exponent.
                if c.get(i) == Some(&'.') && c.get(i + 1).is_some_and(|d| d.is_ascii_digit()) || c.get(i).is_some_and(|e| *e == 'e' || *e == 'E') {
                    let rs = st;
                    if c.get(i) == Some(&'.') {
                        i += 1;
                        while i < c.len() && (c[i].is_ascii_digit() || c[i] == '_') {
                            i += 1;
                        }
                    }
                    if c.get(i).is_some_and(|e| *e == 'e' || *e == 'E') {
                        i += 1;
                        if c.get(i).is_some_and(|x| *x == '+' || *x == '-') {
                            i += 1;
                        }
                        while i < c.len() && c[i].is_ascii_digit() {
                            i += 1;
                        }
                    }
                    out.push(Tok::Real(c[rs..i].iter().filter(|&&x| x != '_').collect()));
                    continue;
                }
                if c.get(j) != Some(&'\'') {
                    out.push(Tok::Number { width: None, signed: true, bits: decimal_bits(&dec)?, z: vec![] });
                    continue;
                }
                size = Some(dec.parse::<u32>().map_err(|e| ParseError(format!("size {dec}: {e}")))?);
                i = j;
            }
            i += 1; // '
            let mut signed = false;
            if c.get(i).is_some_and(|x| *x == 's' || *x == 'S') {
                signed = true;
                i += 1;
            }
            let base = match c.get(i).map(|x| x.to_ascii_lowercase()) {
                Some('b') => 2,
                Some('o') => 8,
                Some('d') => 10,
                Some('h') => 16,
                o => return Err(ParseError(format!("bad base {o:?}"))),
            };
            i += 1;
            while i < c.len() && c[i].is_whitespace() {
                i += 1;
            }
            let st = i;
            while i < c.len() && (c[i].is_ascii_alphanumeric() || c[i] == '_' || c[i] == '?') {
                i += 1;
            }
            let digits: Vec<char> = c[st..i].iter().copied().filter(|&x| x != '_').collect();
            let mut bits: Vec<Option<bool>>;
            let mut z = Vec::new();
            if base == 10 {
                bits = decimal_bits(&digits.iter().collect::<String>())?;
            } else {
                bits = Vec::new();
                for d in digits.iter().rev() {
                    let db = digit_bits(*d, base)?;
                    let isz = matches!(d.to_ascii_lowercase(), 'z' | '?');
                    z.extend(std::iter::repeat_n(isz, db.len()));
                    bits.extend(db);
                }
            }
            // Width: the size given, else 32 (unsized); extend with the
            // top digit's x/z, else zero.
            let w = size.unwrap_or(32.max(bits.len() as u32));
            let top = bits.last().copied().flatten();
            let top_xz = bits.last().is_some_and(|b| b.is_none());
            let topz = z.last().copied().unwrap_or(false);
            bits.resize(w as usize, if top_xz { None } else { Some(false) });
            z.resize(w as usize, top_xz && topz);
            let _ = top;
            out.push(Tok::Number { width: size, signed, bits, z });
            continue;
        }
        let rest: String = c[i..c.len().min(i + 3)].iter().collect();
        match OPS.iter().find(|op| rest.starts_with(**op)) {
            Some(op) => {
                out.push(Tok::Op(op));
                i += op.len();
            }
            None => return Err(ParseError(format!("unexpected character {ch:?}"))),
        }
    }
    Ok(out)
}

/// Binary operator precedence (higher binds tighter).
fn binary_prec(op: &str) -> Option<u8> {
    Some(match op {
        "**" => 12,
        "*" | "/" | "%" => 11,
        "+" | "-" => 10,
        "<<" | ">>" | "<<<" | ">>>" => 9,
        "<" | "<=" | ">" | ">=" => 8,
        "==" | "!=" | "===" | "!==" => 7,
        "&" => 6,
        "^" | "~^" | "^~" => 5,
        "|" => 4,
        "&&" => 3,
        "||" => 2,
        _ => return None,
    })
}

pub struct Parser {
    pub toks: Vec<Tok>,
    pub pos: usize,
}

impl Parser {
    pub fn new(toks: Vec<Tok>) -> Self {
        Parser { toks, pos: 0 }
    }

    pub fn at_end(&self) -> bool {
        self.pos >= self.toks.len()
    }

    pub fn peek(&self) -> Option<&Tok> {
        self.toks.get(self.pos)
    }

    pub fn is_op(&self, op: &str) -> bool {
        matches!(self.peek(), Some(Tok::Op(o)) if *o == op)
    }

    pub fn expect(&mut self, op: &str) -> R<()> {
        if self.is_op(op) {
            self.pos += 1;
            Ok(())
        } else {
            Err(ParseError(format!("expected {op}, found {:?}", self.peek())))
        }
    }

    /// A full expression, `?:` included.
    pub fn expr(&mut self) -> R<Expr> {
        let c = self.binary(0)?;
        if self.is_op("?") {
            self.pos += 1;
            let t = self.expr()?;
            self.expect(":")?;
            let f = self.expr()?;
            return Ok(Expr::Ternary(Box::new(c), Box::new(t), Box::new(f)));
        }
        Ok(c)
    }

    fn binary(&mut self, min: u8) -> R<Expr> {
        let mut lhs = self.unary()?;
        loop {
            let Some(Tok::Op(op)) = self.peek().cloned() else { break };
            let Some(p) = binary_prec(op) else { break };
            if p < min.max(1) {
                break;
            }
            self.pos += 1;
            let rhs = self.binary(p + 1)?;
            lhs = Expr::Binary(op, Box::new(lhs), Box::new(rhs));
        }
        Ok(lhs)
    }

    fn unary(&mut self) -> R<Expr> {
        if let Some(Tok::Op(op)) = self.peek().cloned() {
            if matches!(op, "+" | "-" | "!" | "~" | "&" | "~&" | "|" | "~|" | "^" | "~^" | "^~") {
                self.pos += 1;
                let e = self.unary()?;
                return Ok(Expr::Unary(op, Box::new(e)));
            }
        }
        let p = self.primary()?;
        self.selects(p)
    }

    fn selects(&mut self, mut e: Expr) -> R<Expr> {
        while self.is_op("[") {
            self.pos += 1;
            let a = self.expr()?;
            if self.is_op(":") {
                self.pos += 1;
                let b = self.expr()?;
                self.expect("]")?;
                e = Expr::Range(Box::new(e), Box::new(a), Box::new(b));
            } else if self.is_op("+:") || self.is_op("-:") {
                let up = self.is_op("+:");
                self.pos += 1;
                let w = self.expr()?;
                self.expect("]")?;
                e = Expr::IndexedRange { e: Box::new(e), base: Box::new(a), width: Box::new(w), up };
            } else {
                self.expect("]")?;
                e = Expr::Index(Box::new(e), Box::new(a));
            }
        }
        Ok(e)
    }

    fn primary(&mut self) -> R<Expr> {
        match self.peek().cloned() {
            Some(Tok::Number { width, signed, bits, .. }) => {
                self.pos += 1;
                Ok(Expr::Number { width, signed, bits })
            }
            Some(Tok::Str(v)) => {
                self.pos += 1;
                Ok(Expr::Str(v))
            }
            Some(Tok::Real(v)) => {
                self.pos += 1;
                Ok(Expr::Real(v))
            }
            Some(Tok::Ident(name)) => {
                self.pos += 1;
                if self.is_op("(") {
                    self.pos += 1;
                    let mut args = Vec::new();
                    if !self.is_op(")") {
                        loop {
                            args.push(self.expr()?);
                            if self.is_op(",") {
                                self.pos += 1;
                            } else {
                                break;
                            }
                        }
                    }
                    self.expect(")")?;
                    return Ok(Expr::Call(name, args));
                }
                Ok(Expr::Ident(name))
            }
            Some(Tok::Op("(")) => {
                self.pos += 1;
                let e = self.expr()?;
                self.expect(")")?;
                Ok(e)
            }
            Some(Tok::Op("{")) => {
                self.pos += 1;
                let first = self.expr()?;
                if self.is_op("{") {
                    // replication
                    self.pos += 1;
                    let mut items = vec![self.expr()?];
                    while self.is_op(",") {
                        self.pos += 1;
                        items.push(self.expr()?);
                    }
                    self.expect("}")?;
                    self.expect("}")?;
                    return Ok(Expr::Repeat(Box::new(first), items));
                }
                let mut items = vec![first];
                while self.is_op(",") {
                    self.pos += 1;
                    items.push(self.expr()?);
                }
                self.expect("}")?;
                Ok(Expr::Concat(items))
            }
            t => Err(ParseError(format!("unexpected {t:?} in expression"))),
        }
    }
}

/// Parse one complete expression from source text.
pub fn parse(src: &str) -> R<Expr> {
    let mut p = Parser::new(tokenize(src)?);
    let e = p.expr()?;
    if !p.at_end() {
        return Err(ParseError(format!("trailing tokens after expression in {src:?}")));
    }
    Ok(e)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn s(e: &Expr) -> String {
        match e {
            Expr::Ident(n) => n.clone(),
            Expr::Number { bits, .. } => {
                let v: u64 = bits.iter().take(64).enumerate().map(|(i, b)| u64::from(b.unwrap_or(false)) << i).sum();
                v.to_string()
            }
            Expr::Unary(o, a) => format!("({o}{})", s(a)),
            Expr::Binary(o, a, b) => format!("({} {o} {})", s(a), s(b)),
            Expr::Ternary(c, t, f) => format!("({} ? {} : {})", s(c), s(t), s(f)),
            Expr::Concat(v) => format!("{{{}}}", v.iter().map(s).collect::<Vec<_>>().join(",")),
            Expr::Repeat(n, v) => format!("{{{}{{{}}}}}", s(n), v.iter().map(s).collect::<Vec<_>>().join(",")),
            Expr::Index(e, i) => format!("{}[{}]", s(e), s(i)),
            Expr::Range(e, a, b) => format!("{}[{}:{}]", s(e), s(a), s(b)),
            Expr::IndexedRange { e, base, width, up } => format!("{}[{}{}{}]", s(e), s(base), if *up { "+:" } else { "-:" }, s(width)),
            Expr::Call(n, a) => format!("{n}({})", a.iter().map(s).collect::<Vec<_>>().join(",")),
            Expr::Str(v) => format!("{v:?}"),
            Expr::Real(v) => v.clone(),
        }
    }

    #[test]
    fn precedence() {
        assert_eq!(s(&parse("a + b * c").unwrap()), "(a + (b * c))");
        assert_eq!(s(&parse("a - b - c").unwrap()), "((a - b) - c)");
        assert_eq!(s(&parse("a == b ? c : d").unwrap()), "((a == b) ? c : d)");
        assert_eq!(s(&parse("a ? b : c ? d : e").unwrap()), "(a ? b : (c ? d : e))");
        assert_eq!(s(&parse("a & b | c ^ d").unwrap()), "((a & b) | (c ^ d))");
        assert_eq!(s(&parse("a << 2 + 1").unwrap()), "(a << (2 + 1))");
        assert_eq!(s(&parse("!a && b || c").unwrap()), "(((!a) && b) || c)");
        assert_eq!(s(&parse("~&x[3:0]").unwrap()), "(~&x[3:0])");
    }

    #[test]
    fn primaries() {
        assert_eq!(s(&parse("{a, b[3], {2{c}}}").unwrap()), "{a,b[3],{2{c}}}");
        assert_eq!(s(&parse("x[i +: 4]").unwrap()), "x[i+:4]");
        assert_eq!(s(&parse("$signed(a) >>> 3").unwrap()), "($signed(a) >>> 3)");
        assert_eq!(s(&parse("_mux_1037[11:0]").unwrap()), "_mux_1037[11:0]");
    }

    #[test]
    fn reals_strings() {
        assert_eq!(parse("200.0").unwrap(), Expr::Real("200.0".into()));
        assert_eq!(parse("\"VAR_LOAD\"").unwrap(), Expr::Str("VAR_LOAD".into()));
    }

    #[test]
    fn numbers() {
        let Expr::Number { width, bits, .. } = parse("12'hA5").unwrap() else { panic!() };
        assert_eq!(width, Some(12));
        assert_eq!(bits.len(), 12);
        assert_eq!(bits[0], Some(true));
        assert_eq!(bits[1], Some(false));
        let Expr::Number { width, bits, .. } = parse("32'd4294967295").unwrap() else { panic!() };
        assert_eq!(width, Some(32));
        assert!(bits.iter().all(|b| *b == Some(true)));
        let Expr::Number { bits, .. } = parse("4'bx1").unwrap() else { panic!() };
        assert_eq!(bits, vec![Some(true), None, None, None]);
        let Expr::Number { width, .. } = parse("5").unwrap() else { panic!() };
        assert_eq!(width, None);
    }
}
