//! Tiny expression language for dimensions and parameters.
//!
//! `10`, `2.5 in`, `width / 2 + 3mm`, `sin(30 deg) * r`, `(a + b) * 2`.
//! Lengths evaluate to millimetres, angles to degrees. A bare number takes the
//! default unit of the field it is typed into.

use std::collections::HashMap;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Quantity {
    Length,
    Angle,
    Scalar,
}

#[derive(Debug, Clone, PartialEq)]
enum Tok {
    Num(f64),
    Ident(String),
    Op(char),
    LParen,
    RParen,
    Comma,
}

fn lex(s: &str) -> Result<Vec<Tok>, String> {
    let mut out = Vec::new();
    let cs: Vec<char> = s.chars().collect();
    let mut i = 0;
    while i < cs.len() {
        let c = cs[i];
        if c.is_whitespace() {
            i += 1;
        } else if c.is_ascii_digit() || c == '.' {
            let st = i;
            while i < cs.len() && (cs[i].is_ascii_digit() || cs[i] == '.') {
                i += 1;
            }
            // exponent
            if i < cs.len() && (cs[i] == 'e' || cs[i] == 'E') {
                let save = i;
                i += 1;
                if i < cs.len() && (cs[i] == '+' || cs[i] == '-') {
                    i += 1;
                }
                if i < cs.len() && cs[i].is_ascii_digit() {
                    while i < cs.len() && cs[i].is_ascii_digit() {
                        i += 1;
                    }
                } else {
                    i = save;
                }
            }
            let t: String = cs[st..i].iter().collect();
            out.push(Tok::Num(t.parse().map_err(|_| format!("bad number '{t}'"))?));
        } else if c.is_alphabetic() || c == '_' {
            let st = i;
            while i < cs.len() && (cs[i].is_alphanumeric() || cs[i] == '_') {
                i += 1;
            }
            out.push(Tok::Ident(cs[st..i].iter().collect()));
        } else if c == '"' {
            out.push(Tok::Ident("in".into()));
            i += 1;
        } else if c == '°' {
            out.push(Tok::Ident("deg".into()));
            i += 1;
        } else if "+-*/^%".contains(c) {
            out.push(Tok::Op(c));
            i += 1;
        } else if c == '(' {
            out.push(Tok::LParen);
            i += 1;
        } else if c == ')' {
            out.push(Tok::RParen);
            i += 1;
        } else if c == ',' {
            out.push(Tok::Comma);
            i += 1;
        } else {
            return Err(format!("unexpected '{c}'"));
        }
    }
    Ok(out)
}

fn unit_factor(name: &str) -> Option<(f64, Quantity)> {
    Some(match name {
        "mm" => (1.0, Quantity::Length),
        "cm" => (10.0, Quantity::Length),
        "m" => (1000.0, Quantity::Length),
        "um" => (0.001, Quantity::Length),
        "in" | "inch" => (25.4, Quantity::Length),
        "ft" => (304.8, Quantity::Length),
        "deg" => (1.0, Quantity::Angle),
        "rad" => (180.0 / std::f64::consts::PI, Quantity::Angle),
        _ => return None,
    })
}

struct Parser<'a> {
    toks: Vec<Tok>,
    pos: usize,
    lookup: &'a dyn Fn(&str) -> Option<f64>,
}

impl Parser<'_> {
    fn peek(&self) -> Option<&Tok> {
        self.toks.get(self.pos)
    }
    fn next(&mut self) -> Option<Tok> {
        let t = self.toks.get(self.pos).cloned();
        self.pos += 1;
        t
    }
    fn expr(&mut self) -> Result<f64, String> {
        let mut v = self.term()?;
        while let Some(Tok::Op(c)) = self.peek() {
            let c = *c;
            if c != '+' && c != '-' {
                break;
            }
            self.pos += 1;
            let r = self.term()?;
            v = if c == '+' { v + r } else { v - r };
        }
        Ok(v)
    }
    fn term(&mut self) -> Result<f64, String> {
        let mut v = self.unary()?;
        loop {
            match self.peek() {
                Some(Tok::Op(c)) if *c == '*' || *c == '/' || *c == '%' => {
                    let c = *c;
                    self.pos += 1;
                    let r = self.unary()?;
                    v = match c {
                        '*' => v * r,
                        '/' => {
                            if r == 0.0 {
                                return Err("division by zero".into());
                            }
                            v / r
                        }
                        _ => v % r,
                    };
                }
                // implicit multiplication: "2 width", "2(3+4)"
                Some(Tok::LParen) | Some(Tok::Num(_)) => {
                    let r = self.unary()?;
                    v *= r;
                }
                Some(Tok::Ident(id)) if unit_factor(id).is_none() => {
                    let r = self.unary()?;
                    v *= r;
                }
                _ => break,
            }
        }
        Ok(v)
    }
    fn unary(&mut self) -> Result<f64, String> {
        if let Some(Tok::Op('-')) = self.peek() {
            self.pos += 1;
            return Ok(-self.unary()?);
        }
        if let Some(Tok::Op('+')) = self.peek() {
            self.pos += 1;
            return self.unary();
        }
        self.power()
    }
    fn power(&mut self) -> Result<f64, String> {
        let b = self.postfix()?;
        if let Some(Tok::Op('^')) = self.peek() {
            self.pos += 1;
            let e = self.unary()?;
            return Ok(b.powf(e));
        }
        Ok(b)
    }
    fn postfix(&mut self) -> Result<f64, String> {
        let mut v = self.atom()?;
        while let Some(Tok::Ident(id)) = self.peek() {
            match unit_factor(id) {
                Some((f, _)) => {
                    self.pos += 1;
                    v *= f;
                }
                None => break,
            }
        }
        Ok(v)
    }
    fn atom(&mut self) -> Result<f64, String> {
        match self.next() {
            Some(Tok::Num(n)) => Ok(n),
            Some(Tok::LParen) => {
                let v = self.expr()?;
                match self.next() {
                    Some(Tok::RParen) => Ok(v),
                    _ => Err("missing ')'".into()),
                }
            }
            Some(Tok::Ident(id)) => {
                if let Some(Tok::LParen) = self.peek() {
                    self.pos += 1;
                    let mut args = vec![self.expr()?];
                    while let Some(Tok::Comma) = self.peek() {
                        self.pos += 1;
                        args.push(self.expr()?);
                    }
                    match self.next() {
                        Some(Tok::RParen) => {}
                        _ => return Err("missing ')'".into()),
                    }
                    return call(&id, &args);
                }
                match id.as_str() {
                    "pi" | "PI" => return Ok(std::f64::consts::PI),
                    "e" => return Ok(std::f64::consts::E),
                    _ => {}
                }
                if let Some(v) = (self.lookup)(&id) {
                    return Ok(v);
                }
                if let Some((f, _)) = unit_factor(&id) {
                    // a lone unit, e.g. "in" → 1 in
                    return Ok(f);
                }
                Err(format!("unknown parameter '{id}'"))
            }
            Some(t) => Err(format!("unexpected {t:?}")),
            None => Err("unexpected end of expression".into()),
        }
    }
}

fn call(name: &str, a: &[f64]) -> Result<f64, String> {
    let d2r = std::f64::consts::PI / 180.0;
    let one = |f: fn(f64) -> f64| -> Result<f64, String> {
        if a.len() != 1 {
            return Err(format!("{name}() takes 1 argument"));
        }
        Ok(f(a[0]))
    };
    match name {
        "sin" => one(|x| (x * std::f64::consts::PI / 180.0).sin()),
        "cos" => one(|x| (x * std::f64::consts::PI / 180.0).cos()),
        "tan" => one(|x| (x * std::f64::consts::PI / 180.0).tan()),
        "asin" => one(|x| x.asin() * 180.0 / std::f64::consts::PI),
        "acos" => one(|x| x.acos() * 180.0 / std::f64::consts::PI),
        "atan" => one(|x| x.atan() * 180.0 / std::f64::consts::PI),
        "sqrt" => one(f64::sqrt),
        "abs" => one(f64::abs),
        "floor" => one(f64::floor),
        "ceil" => one(f64::ceil),
        "round" => one(f64::round),
        "min" if !a.is_empty() => Ok(a.iter().cloned().fold(f64::INFINITY, f64::min)),
        "max" if !a.is_empty() => Ok(a.iter().cloned().fold(f64::NEG_INFINITY, f64::max)),
        "atan2" if a.len() == 2 => Ok(a[0].atan2(a[1]) / d2r),
        _ => Err(format!("unknown function '{name}'")),
    }
}

/// Evaluate `src`. Parameter lookup returns values already in base units.
pub fn eval_with(src: &str, lookup: &dyn Fn(&str) -> Option<f64>) -> Result<f64, String> {
    let toks = lex(src)?;
    if toks.is_empty() {
        return Err("empty expression".into());
    }
    let mut p = Parser { toks, pos: 0, lookup };
    let v = p.expr()?;
    if p.pos < p.toks.len() {
        return Err(format!("unexpected {:?}", p.toks[p.pos]));
    }
    if !v.is_finite() {
        return Err("result is not a finite number".into());
    }
    Ok(v)
}

/// Identifiers referenced by an expression (for dependency tracking / rename).
pub fn identifiers(src: &str) -> Vec<String> {
    lex(src)
        .unwrap_or_default()
        .into_iter()
        .filter_map(|t| match t {
            Tok::Ident(s) if unit_factor(&s).is_none() && s != "pi" && s != "PI" => Some(s),
            _ => None,
        })
        .collect()
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, PartialEq)]
pub struct Parameter {
    pub name: String,
    pub expr: String,
    #[serde(default)]
    pub comment: String,
}

/// Evaluated parameter table.
#[derive(Debug, Clone, Default)]
pub struct ParamValues {
    pub values: HashMap<String, f64>,
    pub errors: HashMap<String, String>,
}

impl ParamValues {
    pub fn evaluate(params: &[Parameter]) -> Self {
        let mut out = ParamValues::default();
        let by_name: HashMap<&str, &Parameter> = params.iter().map(|p| (p.name.as_str(), p)).collect();
        fn visit(
            name: &str,
            by_name: &HashMap<&str, &Parameter>,
            out: &mut ParamValues,
            stack: &mut Vec<String>,
        ) -> Option<f64> {
            if let Some(v) = out.values.get(name) {
                return Some(*v);
            }
            if out.errors.contains_key(name) {
                return None;
            }
            let p = by_name.get(name)?;
            if stack.iter().any(|s| s == name) {
                out.errors.insert(name.to_string(), "circular reference".into());
                return None;
            }
            stack.push(name.to_string());
            let mut deps = HashMap::new();
            for id in identifiers(&p.expr) {
                if by_name.contains_key(id.as_str()) {
                    if let Some(v) = visit(&id, by_name, out, stack) {
                        deps.insert(id, v);
                    }
                }
            }
            stack.pop();
            match eval_with(&p.expr, &|n| deps.get(n).copied()) {
                Ok(v) => {
                    out.values.insert(name.to_string(), v);
                    Some(v)
                }
                Err(e) => {
                    out.errors.insert(name.to_string(), e);
                    None
                }
            }
        }
        for p in params {
            let mut stack = Vec::new();
            visit(&p.name, &by_name, &mut out, &mut stack);
        }
        out
    }

    pub fn eval(&self, src: &str) -> Result<f64, String> {
        eval_with(src, &|n| self.values.get(n).copied())
    }
}

/// Format a value compactly for display in a dimension label.
pub fn fmt_num(v: f64) -> String {
    if (v - v.round()).abs() < 1e-9 {
        format!("{}", v.round() as i64)
    } else {
        let s = format!("{v:.3}");
        s.trim_end_matches('0').trim_end_matches('.').to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn ev(s: &str) -> f64 {
        eval_with(s, &|n| match n {
            "w" => Some(10.0),
            "h" => Some(4.0),
            _ => None,
        })
        .unwrap()
    }
    #[test]
    fn basics() {
        assert_eq!(ev("1 + 2 * 3"), 7.0);
        assert_eq!(ev("(1 + 2) * 3"), 9.0);
        assert_eq!(ev("-2^2"), -4.0);
        assert_eq!(ev("2 in"), 50.8);
        assert_eq!(ev("1cm + 5mm"), 15.0);
        assert_eq!(ev("w / 2 + h"), 9.0);
        assert_eq!(ev("2w"), 20.0);
        assert!((ev("sin(30 deg)") - 0.5).abs() < 1e-12);
        assert!((ev("1 rad") - 57.29577951308232).abs() < 1e-9);
        assert_eq!(ev("max(1, w, 3)"), 10.0);
        assert_eq!(ev("1e1"), 10.0);
        assert!(eval_with("foo + 1", &|_| None).is_err());
        assert!(eval_with("1 +", &|_| None).is_err());
    }
    #[test]
    fn params() {
        let ps = vec![
            Parameter { name: "a".into(), expr: "b * 2".into(), comment: String::new() },
            Parameter { name: "b".into(), expr: "3 cm".into(), comment: String::new() },
            Parameter { name: "c".into(), expr: "c + 1".into(), comment: String::new() },
        ];
        let v = ParamValues::evaluate(&ps);
        assert_eq!(v.values["a"], 60.0);
        assert!(v.errors.contains_key("c"));
        assert_eq!(v.eval("a + b").unwrap(), 90.0);
    }
}
