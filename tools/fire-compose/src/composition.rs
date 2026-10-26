//! A deliberately bounded, domain-independent source composition representation.
//! No compiler IR, dynamics, or FIRE presets belong in this module.
use crate::{identifier, Result};

#[derive(Clone, Debug)]
pub enum Value {
    Bool(bool),
    Integer(i64),
    Real(f64),
    Text(String),
    Array(Vec<Value>),
    Record(String, Vec<Modifier>),
    Reference(String),
}

#[derive(Clone, Debug)]
pub enum Modifier {
    Bind(String, Value),
    Nested(String, Vec<Modifier>),
    RedeclareModel {
        name: String,
        class: String,
        modifiers: Vec<Modifier>,
    },
}

#[derive(Clone, Debug)]
pub enum Prefix {
    Component,
    Input,
    Output,
    Parameter,
}

#[derive(Clone, Debug)]
pub struct Instance {
    pub class: String,
    pub name: String,
    pub modifiers: Vec<Modifier>,
    pub dimensions: Vec<usize>,
    pub prefix: Prefix,
    pub binding: Option<Value>,
}

#[derive(Clone, Debug)]
pub struct CompositionSpec {
    pub name: String,
    pub base: Option<String>,
    pub modifiers: Vec<Modifier>,
    pub components: Vec<Instance>,
    /// Declarative equations with bounded literal, array or reference expressions.
    pub equalities: Vec<(String, Value)>,
    /// Conserving connector connections, never ordinary value assignments.
    pub connections: Vec<(String, String)>,
}

fn path(name: &str) -> Result<&str> {
    if name.split('.').all(identifier) {
        Ok(name)
    } else {
        Err(format!("invalid Modelica identifier path: {name}"))
    }
}

fn class_path(name: &str) -> Result<&str> {
    if ["Real", "Integer", "Boolean", "String"].contains(&name) {
        Ok(name)
    } else {
        path(name)
    }
}

// Subscripts are positive integer literals, never arbitrary source expressions.
fn reference(name: &str) -> Result<&str> {
    for segment in name.split('.') {
        if let Some((base, suffix)) = segment.split_once('[') {
            let indices = suffix
                .strip_suffix(']')
                .ok_or("invalid reference subscript")?;
            if !identifier(base)
                || !indices.split(',').all(|i| {
                    !i.is_empty()
                        && i.bytes().all(|c| c.is_ascii_digit())
                        && i.parse::<usize>().is_ok_and(|n| n > 0)
                })
            {
                return Err(format!("invalid Modelica reference: {name}"));
            }
        } else if !identifier(segment) {
            return Err(format!("invalid Modelica reference: {name}"));
        }
    }
    Ok(name)
}

fn value(v: &Value) -> Result<String> {
    Ok(match v {
        Value::Bool(v) => v.to_string(),
        Value::Integer(v) => v.to_string(),
        Value::Real(v) if v.is_finite() => format!("{v:?}"),
        Value::Real(_) => return Err("non-finite Modelica literal".into()),
        Value::Text(v) => serde_json::to_string(v).map_err(|e| e.to_string())?,
        Value::Reference(v) => reference(v)?.to_owned(),
        Value::Array(items) => format!(
            "{{{}}}",
            items
                .iter()
                .map(value)
                .collect::<Result<Vec<_>>>()?
                .join(",")
        ),
        Value::Record(class, fields) => format!("{}({})", path(class)?, modifiers(fields)?),
    })
}

fn modifiers(items: &[Modifier]) -> Result<String> {
    let mut names = std::collections::BTreeSet::new();
    let mut result = Vec::new();
    for item in items {
        let name = match item {
            Modifier::Bind(n, _) | Modifier::Nested(n, _) => n,
            Modifier::RedeclareModel { name, .. } => name,
        };
        path(name)?;
        if !names.insert(name) {
            return Err(format!("duplicate modifier: {name}"));
        }
        result.push(match item {
            Modifier::Bind(n, v) => format!("{n}={}", value(v)?),
            Modifier::Nested(n, m) => format!("{n}({})", modifiers(m)?),
            Modifier::RedeclareModel {
                name,
                class,
                modifiers: m,
            } => format!("redeclare model {name}={}({})", path(class)?, modifiers(m)?),
        });
    }
    Ok(result.join(",\n    "))
}

impl CompositionSpec {
    pub fn emit(&self) -> Result<String> {
        if !identifier(&self.name) {
            return Err("invalid top-level model identifier".into());
        }
        let mut text = format!("within;\nmodel {}\n", self.name);
        if let Some(base) = &self.base {
            text.push_str(&format!(
                "  extends {}(\n    {});\n",
                path(base)?,
                modifiers(&self.modifiers)?
            ));
        } else if !self.modifiers.is_empty() {
            return Err("base modifiers require an extended class".into());
        }
        let mut names = std::collections::BTreeSet::new();
        for c in &self.components {
            if !identifier(&c.name) || !names.insert(&c.name) {
                return Err(format!("invalid or duplicate component name: {}", c.name));
            }
            let prefix = match c.prefix {
                Prefix::Component => "",
                Prefix::Input => "input ",
                Prefix::Output => "output ",
                Prefix::Parameter => "parameter ",
            };
            let dimensions = if c.dimensions.is_empty() {
                String::new()
            } else {
                if c.dimensions.contains(&0) {
                    return Err("component dimensions must be positive".into());
                }
                format!(
                    "[{}]",
                    c.dimensions
                        .iter()
                        .map(usize::to_string)
                        .collect::<Vec<_>>()
                        .join(",")
                )
            };
            let binding = c
                .binding
                .as_ref()
                .map(value)
                .transpose()?
                .map(|v| format!(" = {v}"))
                .unwrap_or_default();
            text.push_str(&format!(
                "  {prefix}{} {}{dimensions}(\n    {}){binding};\n",
                class_path(&c.class)?,
                c.name,
                modifiers(&c.modifiers)?
            ));
        }
        if !self.equalities.is_empty() || !self.connections.is_empty() {
            text.push_str("equation\n");
        }
        for (left, right) in &self.equalities {
            text.push_str(&format!("  {} = {};\n", reference(left)?, value(right)?));
        }
        for (left, right) in &self.connections {
            text.push_str(&format!(
                "  connect({}, {});\n",
                reference(left)?,
                reference(right)?
            ));
        }
        text.push_str(&format!("end {};\n", self.name));
        Ok(text)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn references_allow_fixed_indices_but_not_source_expressions() {
        for valid in [
            "commands.normalizedCommand[3]",
            "rotors[2].force_r[3]",
            "plant.rotorForce_r",
            "a[2,3]",
        ] {
            assert!(value(&Value::Reference(valid.into())).is_ok(), "{valid}");
        }
        for invalid in [
            "a[0]",
            "a[-1]",
            "a[i]",
            "a[1+1]",
            "a[1]; end X",
            "a[1][2]",
            "a[]",
            "a[1,]",
            "a..b",
            "a[184467440737095516160]",
        ] {
            assert!(
                value(&Value::Reference(invalid.into())).is_err(),
                "{invalid}"
            );
        }
    }
}
