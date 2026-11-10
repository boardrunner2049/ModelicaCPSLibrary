//! Provenance of a composed vehicle's peripherals and payloads: for every
//! parameter of every fitted device, the value used and where it came from.
//!
//! The library tags each parameter with where its value should come from, as
//! the last word of its description: `[datasheet]` (a figure a device's
//! datasheet states), `[physics]`, `[environment]`, `[configuration]` (the
//! vehicle's: mounts, loads, initial state), `[ArduPilot]` (the autopilot's
//! conventions) or `[assumed]` (a chosen value no datasheet gives). A composed
//! vehicle then says, for each parameter, whether its value is a datasheet's
//! (the part, the figures and the conversion), the configuration's, or the
//! library's default, with the tag. It goes into the manifest (`provenance`)
//! and into the composed model's documentation.
use crate::config::{ResolvedFireConfig, Table};
use serde_json::{json, Value as Json};
use toml::Value as Toml;

/// A parameter declaration: (name, default expression, description, tag).
#[derive(Clone, Debug)]
pub struct Declared {
    pub name: String,
    pub default: String,
    pub tag: String,
}

/// The parameters a class declares, with those of the classes it extends in
/// the same package (Partial*), parsed from the library's sources.
pub fn declared(package: &str, class: &str) -> Vec<Declared> {
    let dir = crate::artifact::library_root().join(package.replace('.', "/"));
    let mut out = Vec::new();
    let Ok(text) = std::fs::read_to_string(dir.join(format!("{class}.mo"))) else {
        return out;
    };
    for line in text.lines() {
        let l = line.trim_start();
        if let Some(rest) = l.strip_prefix("extends ") {
            let parent = rest
                .split(|c: char| !(c.is_ascii_alphanumeric() || c == '_'))
                .next()
                .unwrap_or("");
            if !parent.is_empty() {
                out.extend(declared(package, parent));
            }
        }
    }
    // Declarations may span lines: join each up to its ';'.
    let mut statement = String::new();
    for line in text.lines() {
        let l = line.trim();
        if statement.is_empty() && !l.starts_with("parameter ") {
            continue;
        }
        statement.push(' ');
        statement.push_str(l);
        if !l.ends_with(';') {
            continue;
        }
        if let Some(d) = parse(&statement) {
            out.retain(|x: &Declared| x.name != d.name);
            out.push(d);
        }
        statement.clear();
    }
    out
}

fn parse(statement: &str) -> Option<Declared> {
    let s = statement.trim().strip_prefix("parameter ")?;
    let mut words = s.split_whitespace();
    words.next()?; // the type
    let rest: String = words.collect::<Vec<_>>().join(" ");
    let name: String = rest
        .chars()
        .take_while(|c| c.is_ascii_alphanumeric() || *c == '_')
        .collect();
    // The description is the last string; the default what follows the first
    // '=' after the modifiers, up to the description.
    let desc_start = rest.rfind(" \"")?;
    let desc = rest[desc_start + 2..].trim_end_matches(';').trim_end_matches('"');
    let head = &rest[..desc_start];
    let mut depth = 0;
    let mut eq = None;
    for (i, c) in head.char_indices() {
        match c {
            '(' | '[' | '{' => depth += 1,
            ')' | ']' | '}' => depth -= 1,
            '=' if depth == 0 => {
                eq = Some(i);
                break;
            }
            _ => {}
        }
    }
    let default = eq.map(|i| head[i + 1..].trim().to_string()).unwrap_or_default();
    let tag = desc
        .rsplit_once('[')
        .and_then(|(_, t)| t.strip_suffix(']'))
        .unwrap_or("untagged")
        .to_string();
    Some(Declared { name, default, tag })
}

/// The configuration key that sets a model parameter.
fn key_of(parameter: &str) -> &str {
    match parameter {
        "position_b" => "position",
        "R_bs" => "orientation",
        "targetPosition_w" => "target",
        other => other,
    }
}

fn text(v: &Toml) -> String {
    match v {
        Toml::String(s) => s.clone(),
        other => other.to_string(),
    }
}

/// One device's parameters: value and origin.
fn device(
    component: &str,
    package: &str,
    class: &str,
    configured: &Table,
    report: Option<&Json>,
) -> Json {
    let mut rows = Vec::new();
    for d in declared(package, class) {
        let key = key_of(&d.name);
        let filled = report.and_then(|r| r["parameters"].get(&d.name)).filter(|p| {
            p["value"].as_str() != Some("explicit in the configuration")
                && !p["value"].as_str().is_some_and(|s| s.starts_with("not a parameter"))
        });
        let (value, origin, source) = match configured.get(key) {
            Some(v) if filled.is_some() => {
                let p = filled.unwrap();
                let from = p["from"]
                    .as_array()
                    .map(|a| a.iter().filter_map(Json::as_str).collect::<Vec<_>>().join(", "))
                    .unwrap_or_default();
                let how = p["how"].as_str().unwrap_or("");
                (
                    text(v),
                    "datasheet",
                    format!(
                        "{}: {}{}",
                        report.unwrap()["name"].as_str().unwrap_or(""),
                        if from.is_empty() { how.to_string() } else { format!("{from} ({how})") },
                        ""
                    ),
                )
            }
            Some(v) => (text(v), "configuration", String::new()),
            None => (d.default.clone(), "library default", String::new()),
        };
        rows.push(json!({
            "parameter": d.name, "value": value, "origin": origin, "tag": d.tag,
            "source": source,
        }));
    }
    json!({"component": component, "class": format!("ModelicaCPSLibrary.{package}.{class}"),
           "part": report.map(|r| r["name"].clone()).unwrap_or(Json::Null),
           "document": report.map(|r| r["source"].clone()).unwrap_or(Json::Null),
           "parameters": rows})
}

/// Every fitted peripheral and payload, and a power module's pack.
pub fn table(c: &ResolvedFireConfig) -> Vec<Json> {
    let report = |kind: &str| c.parts.iter().rev().find(|r| r["kind"] == kind);
    let mut out = Vec::new();
    for kind in crate::peripherals::KINDS {
        let Some(sel) = c.peripherals.get(kind.name).and_then(Toml::as_table) else {
            continue;
        };
        let level = kind.level(sel["model"].as_str().unwrap()).unwrap();
        out.push(device(kind.component, "Systems.Peripherals", level.class, sel, report(kind.name)));
        if kind.name == "power_module" {
            let pack = sel.get("battery").and_then(Toml::as_table).cloned().unwrap_or_default();
            let mut d = device("powerModule.battery", "Systems.Power", "Battery", &pack, None);
            // A pack named by its part: its figures are the datasheet's.
            if let Some(r) = report("battery") {
                for row in d["parameters"].as_array_mut().unwrap() {
                    if row["origin"] == "configuration" && r["parameters"].get(row["parameter"].as_str().unwrap()).is_some() {
                        row["origin"] = json!("datasheet");
                        row["source"] = json!(r["name"].as_str().unwrap_or(""));
                    }
                }
                d["part"] = r["name"].clone();
                d["document"] = r["source"].clone();
                for a in r["assumed"].as_array().into_iter().flatten() {
                    if let Some(s) = a.as_str() {
                        if s.starts_with("pack resistance") {
                            for row in d["parameters"].as_array_mut().unwrap() {
                                if row["parameter"] == "resistance" {
                                    row["origin"] = json!("assumed");
                                    row["source"] = json!(s);
                                }
                            }
                        }
                    }
                }
            }
            out.push(d);
        }
    }
    for kind in crate::payloads::KINDS {
        let Some(sel) = c.payloads.get(kind.name).and_then(Toml::as_table) else {
            continue;
        };
        out.push(device(kind.component, "Systems.Payloads", kind.class, sel, report(kind.name)));
    }
    out
}

/// The table as the composed model's documentation (HTML).
pub fn html(rows: &[Json]) -> String {
    let esc = |s: &str| s.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;").replace('"', "&quot;");
    let mut h = String::from(
        "<html><h3>Provenance of the peripherals and payloads</h3><p>Origin: datasheet (the part's \
         figures), configuration, or library default; tag: where the parameter's value should come \
         from (datasheet, physics, environment, configuration, ArduPilot, assumed).</p>",
    );
    for d in rows {
        h += &format!(
            "<h4>{} ({}){}</h4><table border=\"1\"><tr><th>parameter</th><th>value</th><th>origin</th><th>tag</th><th>from</th></tr>",
            esc(d["component"].as_str().unwrap_or("")),
            esc(d["class"].as_str().unwrap_or("")),
            d["part"].as_str().map(|p| format!(": {}", esc(p))).unwrap_or_default()
        );
        for r in d["parameters"].as_array().into_iter().flatten() {
            h += &format!(
                "<tr><td>{}</td><td>{}</td><td>{}</td><td>{}</td><td>{}</td></tr>",
                esc(r["parameter"].as_str().unwrap_or("")),
                esc(r["value"].as_str().unwrap_or("")),
                esc(r["origin"].as_str().unwrap_or("")),
                esc(r["tag"].as_str().unwrap_or("")),
                esc(r["source"].as_str().unwrap_or(""))
            );
        }
        h += "</table>";
    }
    h + "</html>"
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_library_parameter_is_tagged() {
        for (package, classes) in [
            ("Systems.Peripherals", crate::peripherals::KINDS.iter().flat_map(|k| k.levels.iter().map(|l| l.class)).collect::<Vec<_>>()),
            ("Systems.Payloads", crate::payloads::KINDS.iter().map(|k| k.class).collect()),
            ("Systems.Power", vec!["Battery"]),
        ] {
            for class in classes {
                let d = declared(package, class);
                assert!(!d.is_empty(), "{class}");
                for p in d {
                    assert!(
                        ["datasheet", "physics", "environment", "configuration", "ArduPilot", "assumed"].contains(&p.tag.as_str()),
                        "{class}.{}: tag {:?}", p.name, p.tag
                    );
                }
            }
        }
    }

    #[test]
    fn a_part_s_figures_are_its_datasheet_s() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../configs/quad.toml");
        let raw = std::fs::read_to_string(root).unwrap()
            + "\n[peripherals.rangefinder]\npart = \"tfmini_plus\"\nnoiseStd = 0.05\n";
        let c = crate::config::parse_at(&raw, None).unwrap();
        let t = table(&c);
        let rows = t[0]["parameters"].as_array().unwrap();
        let row = |n: &str| rows.iter().find(|r| r["parameter"] == n).unwrap().clone();
        assert_eq!(row("maxRange")["origin"], "datasheet");
        assert_eq!(row("maxRange")["value"], "12.0");
        assert_eq!(row("noiseStd")["origin"], "configuration");
        assert_eq!(row("groundDown")["origin"], "library default");
        assert_eq!(row("groundDown")["tag"], "environment");
    }
}
