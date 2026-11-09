//! Real powertrain parts: motors, propellers, batteries and ESCs as a
//! multirotor's propulsion (P0-P4), servos as a plane's control-surface servos
//! or a rover's steering (S0/S1), gearmotors as a rover's drive (D0/D1).
//!
//! A powertrain level is one assembly of several parts, so it is selected in
//! one table:
//!
//! ```toml
//! [models.propulsion]          # multirotor
//! motor = "tmotor_mn2212_kv920"
//! propeller = "apc_10x47sf"    # optional with a motor whose bench table names a propeller
//! battery = "tattu_4s_5200"    # or [power] dcVoltage
//! esc = "..."                  # optional; P3/P4 use its switch figures
//! level = "P2"                 # P0..P4; default P2
//! battery_state = "nominal"    # full | nominal | storage | empty
//!
//! [models.servos]              # fixed-wing; rover: [models.steering]
//! part = "hitec_hs5085mg"
//! level = "S1"                 # S0 | S1; default S1
//! voltage = 6.0                # optional: the rated voltage whose speed to use
//!
//! [models.drive]               # rover
//! part = "pololu_37d_50to1_12v"
//! count = 2                    # motors driving the axle; default 1
//! battery = "..."              # optional; default the rated voltage
//! level = "D1"
//! ```
//!
//! The figures are converted to the level's parameters ([motor], [rotor],
//! [power], [esc]; [plant] for a rover or plane) and explicit keys win, as for
//! sensor parts. The report adds `checks`: the model's steady state against
//! the manufacturer's own bench table, and the currents and speeds against
//! the parts' limits.
use crate::config::Table;
use crate::Result;
use serde_json::{json, Value as Json};
use std::collections::BTreeMap;
use std::f64::consts::PI;
use toml::Value;

pub const KINDS: [&str; 6] = ["motor", "propeller", "battery", "esc", "servo", "gearmotor"];

const G: f64 = 9.80665;
const RPM: f64 = 2.0 * PI / 60.0;
const LBF: f64 = 4.448_221_615_260_5;
const INLBF: f64 = 0.112_984_829_027_6;
const KGCM: f64 = G / 100.0;
const OZ: f64 = 28.349_523_125;

/// Library defaults the conversions depend on (Vehicles/*Plant.mo).
const FIXED_WING_MAX_AILERON: f64 = 0.3491;
const ROVER_MAX_STEER: f64 = 0.45;

const METADATA: [&str; 6] = ["name", "kind", "manufacturer", "source", "notes", "url"];

/// One loaded part.
struct Part {
    kind: &'static str,
    id: String,
    t: Table,
}

impl Part {
    fn load(kind: &'static str, name: &str) -> Result<Self> {
        let (id, t) = crate::parts::find(kind, name)?;
        if let Some(k) = t.get("kind").and_then(Value::as_str) {
            if k != kind {
                return Err(format!("part {name:?} is a {k}, not a {kind}"));
            }
        }
        Ok(Part { kind, id, t })
    }
    fn f(&self, key: &str) -> Option<f64> {
        float(self.t.get(key)?)
    }
    fn name(&self) -> &str {
        self.t
            .get("name")
            .and_then(Value::as_str)
            .unwrap_or(&self.id)
    }
    fn rows(&self, key: &str) -> Vec<&Table> {
        self.t
            .get(key)
            .and_then(Value::as_array)
            .map(|a| a.iter().filter_map(Value::as_table).collect())
            .unwrap_or_default()
    }
    fn figure(&self, key: &str) -> String {
        format!("{}.{key}", self.kind)
    }
}

fn float(v: &Value) -> Option<f64> {
    v.as_float().or(v.as_integer().map(|i| i as f64))
}
fn row_f(t: &Table, key: &str) -> Option<f64> {
    float(t.get(key)?)
}

fn num(x: f64) -> Value {
    let r: f64 = format!("{x:.6e}").parse().unwrap();
    Value::Float(r)
}

/// The conversion's output and its account.
#[derive(Default)]
struct Out {
    /// (section, key) -> (value, figures, how)
    params: BTreeMap<(String, String), (Value, Vec<String>, String)>,
    assumed: Vec<String>,
    not_stated: BTreeMap<String, String>,
    /// Figures used (as "kind.key").
    used: Vec<String>,
    /// Figures dropped, with why.
    dropped: BTreeMap<String, String>,
    checks: Vec<String>,
}

impl Out {
    fn put(
        &mut self,
        section: &str,
        key: &str,
        value: Value,
        from: &[String],
        how: impl Into<String>,
    ) {
        self.used.extend(from.iter().cloned());
        self.params.insert(
            (section.into(), key.into()),
            (value, from.to_vec(), how.into()),
        );
    }
    fn assume(&mut self, what: impl Into<String>) {
        self.assumed.push(what.into());
    }
}

fn figures(p: &Part, keys: &[&str]) -> Vec<String> {
    keys.iter()
        .filter(|k| p.t.contains_key(**k))
        .map(|k| p.figure(k))
        .collect()
}

/// The parts a report names, and the figures of each the conversion did not
/// use, with why.
fn finish(
    kind: &str,
    parts: &[&Part],
    level: &str,
    model: &str,
    out: Out,
    explicit: &BTreeMap<String, Table>,
) -> (BTreeMap<String, Table>, Json) {
    let mut params = serde_json::Map::new();
    let mut sections: BTreeMap<String, Table> = BTreeMap::new();
    for ((section, key), (value, from, how)) in &out.params {
        let name = format!("{section}.{key}");
        if explicit.get(section).is_some_and(|t| t.contains_key(key)) {
            params.insert(
                name,
                json!({"value": "explicit in the configuration", "from": from, "how": how}),
            );
            continue;
        }
        let shown = serde_json::to_value(value).unwrap_or(Json::Null);
        params.insert(name, json!({"value": shown, "from": from, "how": how}));
        sections
            .entry(section.clone())
            .or_default()
            .insert(key.clone(), value.clone());
    }
    let mut not_represented = serde_json::Map::new();
    for p in parts {
        for k in p.t.keys() {
            let f = p.figure(k);
            if METADATA.contains(&k.as_str()) || out.used.contains(&f) {
                continue;
            }
            let why = out
                .dropped
                .get(&f)
                .cloned()
                .or_else(|| reason(p.kind, k).map(str::to_string))
                .unwrap_or_else(|| format!("not represented at {level} ({model})"));
            not_represented.insert(f, json!(why));
        }
    }
    let report = json!({
        "kind": kind,
        "part": parts.iter().map(|p| p.id.as_str()).collect::<Vec<_>>().join(" + "),
        "name": parts.iter().map(|p| p.name()).collect::<Vec<_>>().join(", "),
        "source": parts.iter().map(|p| format!("{}: {}", p.name(), p.t.get("source").and_then(Value::as_str).unwrap_or(""))).collect::<Vec<_>>().join("; "),
        "parts": parts.iter().map(|p| json!({"kind": p.kind, "id": p.id, "name": p.name(),
            "source": p.t.get("source").and_then(Value::as_str).unwrap_or("")})).collect::<Vec<_>>(),
        "level": level, "model": model, "mode": Json::Null, "tolerances": "none",
        "parameters": params, "assumed": out.assumed, "checks": out.checks,
        "not_represented": not_represented, "not_stated": out.not_stated,
    });
    (sections, report)
}

/// Why a figure of a part of `kind` goes into no parameter, when it never does.
fn reason(kind: &str, key: &str) -> Option<&'static str> {
    let table: &[(&str, &str)] = match kind {
        "motor" => &MOTOR_REASONS,
        "propeller" => &PROP_REASONS,
        "battery" => &BATTERY_REASONS,
        "esc" => &ESC_REASONS,
        "servo" => &SERVO_REASONS,
        _ => &GEARMOTOR_REASONS,
    };
    table
        .iter()
        .find(|(r, _)| *r == key || (r.ends_with('*') && key.starts_with(&r[..r.len() - 1])))
        .map(|(_, w)| *w)
}

// --- propulsion ------------------------------------------------------------------

/// A motor-propeller pair at steady state: k [N m/A], R [ohm], b [N m s],
/// kT [N s^2], kQ [N m s^2].
#[derive(Clone, Copy)]
struct Pair {
    k: f64,
    r: f64,
    b: f64,
    kt: f64,
    kq: f64,
}

impl Pair {
    /// Speed at average winding voltage `v` (six-step DC equivalent).
    fn omega(&self, v: f64) -> f64 {
        let bb = self.b + self.k * self.k / self.r;
        let c = self.k * v / self.r;
        if self.kq <= 0.0 {
            return c / bb;
        }
        (-bb + (bb * bb + 4.0 * self.kq * c).sqrt()) / (2.0 * self.kq)
    }
    fn current(&self, w: f64) -> f64 {
        (self.kq * w * w + self.b * w) / self.k
    }
    fn voltage(&self, w: f64) -> f64 {
        self.k * w + self.r * self.current(w)
    }
}

fn pct(model: f64, data: f64) -> String {
    format!("{:+.0} %", 100.0 * (model - data) / data)
}

/// Least-squares k with y = k x^2 over (x, y).
fn fit_square(xy: &[(f64, f64)]) -> f64 {
    let num: f64 = xy.iter().map(|(x, y)| y * x * x).sum();
    let den: f64 = xy.iter().map(|(x, _)| x.powi(4)).sum();
    num / den
}

fn worst(xy: &[(f64, f64)], k: f64) -> f64 {
    xy.iter()
        .map(|(x, y)| ((k * x * x - y) / y).abs())
        .fold(0.0, f64::max)
}

fn battery_voltage(b: &Part, state: &str, out: &mut Out) -> Result<(f64, Vec<String>)> {
    let s = b
        .f("cells_series")
        .ok_or(format!("battery {}: cells_series is required", b.id))?;
    let liion =
        b.t.get("chemistry")
            .and_then(Value::as_str)
            .is_some_and(|c| c.to_ascii_lowercase().contains("ion"));
    let (key, fallback, what) = match state {
        "full" => ("cell_full_v", 4.2, "full charge"),
        "nominal" | "" => ("cell_nominal_v", if liion { 3.6 } else { 3.7 }, "nominal"),
        "storage" => ("", 3.85, "storage charge"),
        "empty" => ("cell_cutoff_v", if liion { 2.5 } else { 3.5 }, "cutoff"),
        other => {
            return Err(format!(
                "models.propulsion.battery_state = {other:?}: full, nominal, storage or empty"
            ))
        }
    };
    let mut from = figures(b, &["cells_series"]);
    let cell = match b.f(key) {
        Some(v) => {
            from.push(b.figure(key));
            v
        }
        None => {
            out.assume(format!(
                "{} cell voltage {fallback} V ({what}; not stated)",
                b.name()
            ));
            fallback
        }
    };
    Ok((s * cell, from))
}

fn battery_resistance(b: &Part, out: &mut Out) -> (f64, Vec<String>) {
    let s = b.f("cells_series").unwrap_or(1.0);
    let p = b.f("cells_parallel").unwrap_or(1.0);
    if let Some(r) = b.f("internal_resistance_pack_mohm") {
        return (r / 1000.0, figures(b, &["internal_resistance_pack_mohm"]));
    }
    // A cell figure: DC before AC (the AC 1 kHz impedance leaves out the
    // slower polarization the load sees).
    let mut keys: Vec<&String> =
        b.t.keys()
            .filter(|k| k.starts_with("internal_resistance_cell"))
            .collect();
    keys.sort_by_key(|k| !k.contains("dc"));
    if let Some(key) = keys.first() {
        let r = b.f(key).unwrap_or(0.0);
        return (
            s * r / p / 1000.0,
            figures(b, &[key.as_str(), "cells_series", "cells_parallel"]),
        );
    }
    let ah = b.f("capacity_mah").unwrap_or(5000.0) / 1000.0 / p;
    let cell = 0.015 / ah;
    out.assume(format!(
        "{}: internal resistance not stated; {:.1} mOhm per cell (15 mOhm Ah / {ah:.2} Ah, typical of LiPo cells)",
        b.name(),
        cell * 1000.0
    ));
    (
        s * cell / p,
        figures(b, &["cells_series", "cells_parallel", "capacity_mah"]),
    )
}

/// The rotor count and the vehicle mass of a multirotor root, when stated.
fn airframe(root: &Table) -> (Option<f64>, Option<f64>) {
    let g = root.get("geometry").and_then(Value::as_table);
    let n = g
        .and_then(|g| g.get("nActuators").and_then(float))
        .or_else(|| {
            let preset = g?.get("preset")?.as_str()?.to_ascii_lowercase();
            Some(if preset.starts_with("quad") {
                4.0
            } else if preset.starts_with("hex") {
                6.0
            } else if preset.starts_with("octo") {
                8.0
            } else {
                return None;
            })
        });
    let m = root
        .get("mass")
        .and_then(Value::as_table)
        .and_then(|m| m.get("aggregate"))
        .and_then(Value::as_table)
        .and_then(|a| a.get("mass"))
        .and_then(float);
    (n, m)
}

const PROPULSION_OPTIONS: [&str; 6] = [
    "motor",
    "propeller",
    "battery",
    "esc",
    "level",
    "battery_state",
];

const MOTOR_REASONS: [(&str, &str); 12] = [
    ("weight_*", "mass: part of [mass], not of the motor model"),
    (
        "stator_slots",
        "slot count: the models have no cogging or slot harmonics",
    ),
    ("stator_*", "stator dimensions: the motor models are lumped"),
    (
        "max_continuous_current_a",
        "a limit: checked against the full-throttle current",
    ),
    (
        "max_power_w",
        "a limit: checked against the full-throttle power",
    ),
    ("cells_min", "a limit: checked against the battery"),
    ("cells_max", "a limit: checked against the battery"),
    (
        "test",
        "the bench table: compared with the model's steady state in checks",
    ),
    ("max_*", "a limit: no thermal or saturation model"),
    ("peak_*", "a limit: no current limit model"),
    ("shaft_*", "mechanical dimension"),
    ("magnet_poles", "commutation: represented at P3/P4 only"),
];
const PROP_REASONS: [(&str, &str); 6] = [
    ("max_rpm", "a limit: checked against the full-throttle speed"),
    ("blades", "geometry: the rotor model is thrust and torque coefficients"),
    ("static", "static thrust and torque table: fitted as kT and kQ"),
    ("pitch_in", "geometry: the rotor model has no inflow, so pitch speed is not represented"),
    ("diameter_*", "geometry: the rotor model is thrust and torque coefficients (the diameter enters the inertia at P2+)"),
    ("mass_*", "mass: enters the rotor inertia at P2+; the airframe's [mass] includes it"),
];
const BATTERY_REASONS: [(&str, &str); 11] = [
    ("capacity_mah", "capacity: no state-of-charge model; choose battery_state"),
    ("weight_g", "mass: part of [mass], not of the supply model"),
    ("c_rating_continuous", "a limit: checked against the full-throttle current"),
    ("c_rating_burst", "a limit: no time-limited current model"),
    ("burst_duration_s", "a limit: no time-limited current model"),
    ("max_charge_c", "charging: not simulated"),
    ("cell_discharge_continuous_a", "a limit: checked against the full-throttle current"),
    ("cell_*", "another charge state's cell voltage (battery_state picks one), or a cell figure the supply model has no use for"),
    ("pack_nominal_v", "the pack voltage: the cell count x the cell voltage is used"),
    ("cells_parallel", "parallel strings: only the source resistance (P3/P4) uses them"),
    ("chemistry", "chemistry: sets the cell voltages a datasheet leaves out"),
];
const GEARMOTOR_REASONS: [(&str, &str); 7] = [
    (
        "no_load_current_a",
        "no-load loss: the drive model has no friction term",
    ),
    ("weight_*", "mass: part of the rover's mass"),
    ("max_power_w", "a limit: no thermal model"),
    (
        "stall_current_a",
        "stall current: D0 has no electrical model",
    ),
    ("encoder*", "encoder: not modelled"),
    (
        "max_efficiency*",
        "efficiency point: implied by the line model, not set",
    ),
    ("rated_*", "rating: the figures are scaled from it"),
];
const ESC_REASONS: [(&str, &str); 12] = [
    (
        "current_continuous_a",
        "a limit: checked against the full-throttle current",
    ),
    ("current_burst_a", "a limit: no current limit model"),
    ("burst_duration_s", "a limit: no current limit model"),
    ("cells_min", "a limit: checked against the battery"),
    ("cells_max", "a limit: checked against the battery"),
    ("weight_g", "mass: part of [mass]"),
    (
        "firmware",
        "firmware: no protocol, timing advance or demag model",
    ),
    (
        "pwm_frequency_*",
        "options: pwm_frequency_khz is the one used",
    ),
    ("min_voltage_v", "a limit: checked against the supply"),
    ("max_voltage_v", "a limit: checked against the supply"),
    ("bec*", "BEC: not modelled"),
    (
        "throttle_*",
        "throttle signal: the interface's PWM calibration is used",
    ),
];

/// `[models.propulsion]`: the multirotor's ESC, motor, rotor and power from
/// real parts.
fn propulsion(root: &Table, sel: &Table) -> Result<(Table, BTreeMap<String, Table>, Json)> {
    for k in sel.keys() {
        if !PROPULSION_OPTIONS.contains(&k.as_str()) {
            return Err(format!(
                "models.propulsion.{k}: the options are {}",
                PROPULSION_OPTIONS.join(", ")
            ));
        }
    }
    let get = |k: &str| sel.get(k).and_then(Value::as_str);
    let level = get("level").unwrap_or("P2");
    let n_level = crate::config::PROPULSION
        .iter()
        .position(|(l, ..)| *l == level)
        .ok_or(format!(
            "models.propulsion.level = {level:?}: one of P0, P1, P2, P3, P4"
        ))?;
    let (_, esc_model, motor_model, _) = crate::config::PROPULSION[n_level];
    let model = format!("{esc_model} + {motor_model}");
    let load = |kind: &'static str| get(kind).map(|n| Part::load(kind, n)).transpose();
    let motor = load("motor")?;
    let prop = load("propeller")?;
    let battery = load("battery")?;
    let esc = load("esc")?;
    if motor.is_none() && prop.is_none() {
        return Err("models.propulsion needs a motor or a propeller part".into());
    }
    let explicit: BTreeMap<String, Table> = ["motor", "rotor", "power", "esc"]
        .iter()
        .filter_map(|s| Some((s.to_string(), root.get(*s)?.as_table()?.clone())))
        .collect();
    let ex = |s: &str, k: &str| explicit.get(s).and_then(|t| t.get(k)).and_then(float);
    let mut out = Out::default();
    let (n_rotors, mass) = airframe(root);

    // Supply voltage.
    let mut volts: Option<(f64, Vec<String>, String)> = None;
    if let Some(v) = ex("power", "dcVoltage") {
        volts = Some((v, vec![], "explicit [power] dcVoltage".into()));
    } else if let Some(b) = &battery {
        let state = get("battery_state").unwrap_or("nominal");
        let (v, from) = battery_voltage(b, state, &mut out)?;
        volts = Some((
            v,
            from,
            format!(
                "{} cells x the {state} cell voltage",
                b.f("cells_series").unwrap_or(0.0)
            ),
        ));
    }
    if n_level >= 1 {
        if let Some((v, from, how)) = &volts {
            out.put("power", "dcVoltage", num(*v), from, how.clone());
        } else {
            return Err(format!("{level} needs a battery part or [power] dcVoltage"));
        }
    }

    // Motor constants.
    let mut pair = Pair {
        k: 0.0,
        r: 0.0,
        b: 0.0,
        kt: 0.0,
        kq: 0.0,
    };
    let mut j_motor = 0.0;
    let mut l = 0.0;
    let mut motor_from: Vec<String> = Vec::new();
    if let Some(m) = &motor {
        let kv = m
            .f("kv_rpm_per_v")
            .ok_or(format!("motor {}: kv_rpm_per_v is required", m.id))?;
        pair.k = 1.0 / (kv * RPM);
        motor_from = figures(m, &["kv_rpm_per_v"]);
        match m.f("resistance_mohm") {
            Some(r) => pair.r = r / 1000.0,
            None => {
                pair.r = 0.1;
                out.not_stated.insert(
                    m.figure("resistance_mohm"),
                    "not stated: 0.1 ohm assumed".into(),
                );
                out.assume("winding resistance 0.1 ohm (not stated)");
            }
        }
        l = match m.f("inductance_uh") {
            Some(l) => l * 1e-6,
            None if n_level < 2 => pair.r * 0.2e-3,
            None => {
                out.assume(format!(
                    "{}: line-to-line inductance not stated; L = 0.2 ms x R = {:.0} uH (typical outrunner L/R)",
                    m.name(),
                    pair.r * 0.2e-3 * 1e6
                ));
                pair.r * 0.2e-3
            }
        };
        if let (Some(i0), Some(v0)) = (m.f("no_load_current_a"), m.f("no_load_current_at_v")) {
            let w0 = (v0 - pair.r * i0) / pair.k;
            pair.b = pair.k * i0 / w0;
        } else {
            out.not_stated.insert(
                m.figure("no_load_current_a"),
                "not stated: no friction (b = 0)".into(),
            );
        }
        j_motor = match m.f("rotor_inertia_gcm2") {
            Some(j) => j * 1e-7,
            None => {
                let w = m.f("weight_g").unwrap_or(55.0) / 1000.0;
                let r = 0.0135 * (w / 0.055).cbrt();
                out.assume(format!(
                    "{}: rotor inertia not stated; 40 % of the motor's mass as a ring of radius {:.1} mm (the bell of a 2212-size motor, scaled with mass^(1/3)) = {:.2e} kg m^2",
                    m.name(), r * 1000.0, 0.4 * w * r * r
                ));
                0.4 * w * r * r
            }
        };
    }

    // Propeller coefficients: the propeller's static data, else the motor's
    // bench table.
    let mut j_prop = 0.0;
    let mut rotor_from: Vec<String> = Vec::new();
    let mut rotor_how = String::new();
    let mut fitted_from_bench = false;
    if let Some(p) = &prop {
        let rows = p.rows("static");
        let max_rpm = p.f("max_rpm").unwrap_or(f64::INFINITY);
        let mut t = Vec::new();
        let mut q = Vec::new();
        for r in &rows {
            let Some(rpm) = row_f(r, "rpm") else { continue };
            if rpm > max_rpm {
                continue;
            }
            let w = rpm * RPM;
            if let Some(f) = row_f(r, "thrust_lbf")
                .map(|x| x * LBF)
                .or(row_f(r, "thrust_n"))
                .or(row_f(r, "thrust_g").map(|x| x * G / 1000.0))
            {
                t.push((w, f));
            }
            if let Some(f) = row_f(r, "torque_inlbf")
                .map(|x| x * INLBF)
                .or(row_f(r, "torque_nm"))
            {
                q.push((w, f));
            }
        }
        if t.is_empty() || q.is_empty() {
            return Err(format!(
                "propeller {}: needs [[static]] rows with rpm, thrust and torque",
                p.id
            ));
        }
        pair.kt = fit_square(&t);
        pair.kq = fit_square(&q);
        rotor_from = vec![p.figure("static")];
        rotor_how = format!(
            "least squares of T = kT w^2, Q = kQ w^2 over the {} static rows up to {:.0} rpm (worst row {:.0} % / {:.0} %)",
            t.len(),
            t.iter().map(|x| x.0).fold(0.0, f64::max) / RPM,
            100.0 * worst(&t, pair.kt),
            100.0 * worst(&q, pair.kq)
        );
        let mass = p.f("mass_g").or(p.f("mass_oz").map(|o| o * OZ));
        let d = p.f("diameter_in").map(|d| d * 0.0254);
        match (mass, d) {
            (Some(m), Some(d)) => {
                j_prop = m / 1000.0 * d * d / 24.0;
                out.assume(format!(
                    "{}: inertia m D^2 / 24 = {:.2e} kg m^2 (half a uniform rod: the blades taper and the hub is near the axis)",
                    p.name(),
                    j_prop
                ));
            }
            _ => {
                out.not_stated
                    .insert(p.figure("mass_g"), "not stated: propeller inertia 0".into());
            }
        }
    } else if let Some(m) = &motor {
        // The bench table: thrust per rpm, and torque per rpm if given or
        // from the current (Q = k (I - I0)).
        let i0 = m.f("no_load_current_a").unwrap_or(0.0);
        let mut t = Vec::new();
        let mut q = Vec::new();
        let (prop_name, rows, _) = bench(m, None);
        for r in &rows {
            let (Some(rpm), Some(g)) = (row_f(r, "rpm"), row_f(r, "thrust_g")) else {
                continue;
            };
            let w = rpm * RPM;
            t.push((w, g * G / 1000.0));
            if let Some(nm) = row_f(r, "torque_nm") {
                q.push((w, nm));
            } else if let Some(i) = row_f(r, "current_a") {
                q.push((w, pair.k * (i - i0).max(0.0)));
            }
        }
        if t.is_empty() || q.is_empty() {
            return Err(format!(
                "motor {} has no bench table with rpm and thrust: name a propeller part",
                m.id
            ));
        }
        pair.kt = fit_square(&t);
        pair.kq = fit_square(&q);
        fitted_from_bench = true;
        rotor_from = vec![m.figure("test")];
        rotor_how = format!(
            "least squares of T = kT w^2 and Q = kQ w^2 over the motor's bench table with {prop_name} ({} rows{})",
            t.len(),
            if rows.iter().any(|r| r.contains_key("torque_nm")) { "" } else { "; torque from the current, Q = k (I - I0)" }
        );
        out.assume(format!("rotor: no propeller part; {prop_name} of the bench table, inertia 0 unless [rotor] J is set"));
    }
    if pair.kt > 0.0 {
        out.put("rotor", "kT", num(pair.kt), &rotor_from, rotor_how.clone());
        out.put("rotor", "kQ", num(pair.kq), &rotor_from, rotor_how.clone());
    }
    if n_level >= 2 {
        let from = figures(
            prop.as_ref().unwrap_or_else(|| motor.as_ref().unwrap()),
            &["mass_g", "mass_oz", "diameter_in"],
        );
        if prop.is_some() {
            out.put(
                "rotor",
                "J",
                num(j_prop),
                &from,
                "m D^2 / 24 (assumed shape)",
            );
        } else if ex("rotor", "J").is_none() {
            return Err(format!(
                "{level} needs the propeller's inertia: name a propeller part or set [rotor] J"
            ));
        }
    }
    if let Some(j) = ex("rotor", "J") {
        j_prop = j;
    }

    // Steady state, hover and the motor's parameters per level.
    if let Some(m) = &motor {
        let v = volts.as_ref().map(|x| x.0).or_else(|| {
            let v = m
                .rows("test")
                .iter()
                .filter_map(|r| row_f(r, "voltage_v"))
                .fold(0.0, f64::max);
            (v > 0.0).then(|| {
                out.assume(format!(
                    "supply {v} V: the bench table's (no battery or [power] dcVoltage)"
                ));
                v
            })
        });
        let Some(v) = v else {
            return Err(format!("{level} needs a battery part or [power] dcVoltage to place the motor's speed range"));
        };
        if let Some((_, from, _)) = &volts {
            out.used.extend(from.iter().cloned());
        }
        let j_total = j_motor + j_prop;
        let w_full = pair.omega(v);
        let hover = match (n_rotors, mass) {
            (Some(n), Some(mass)) if pair.kt > 0.0 => {
                Some(((mass * G / n / pair.kt).sqrt(), n, mass))
            }
            _ => None,
        };
        let w_lin = hover.map(|h| h.0).unwrap_or_else(|| pair.omega(0.5 * v));
        let at = if hover.is_some() {
            "hover"
        } else {
            "half throttle (no [mass] aggregate or rotor count)"
        };
        let tau = j_total / (pair.b + pair.k * pair.k / pair.r + 2.0 * pair.kq * w_lin);
        // Least squares of omega = Kv_eff * u over u in [0.3, 1] V.
        let us: Vec<f64> = (0..15).map(|i| v * (0.3 + 0.7 * i as f64 / 14.0)).collect();
        let kv_eff = us.iter().map(|u| pair.omega(*u) * u).sum::<f64>()
            / us.iter().map(|u| u * u).sum::<f64>();
        let lin_err = us
            .iter()
            .map(|u| ((kv_eff * u - pair.omega(*u)) / pair.omega(*u)).abs())
            .fold(0.0, f64::max);
        let mut all = motor_from.clone();
        all.extend(figures(
            m,
            &[
                "resistance_mohm",
                "no_load_current_a",
                "no_load_current_at_v",
            ],
        ));
        all.extend(rotor_from.clone());
        let tau_how = format!(
            "J / (b + k^2/R + 2 kQ w) linearized at {at}, w = {:.0} rpm, J = {:.2e} kg m^2 (motor + propeller)",
            w_lin / RPM,
            j_total
        );
        let speed_how = format!(
            "least squares of the loaded speed over 30-100 % of {v:.1} V: {:.0} rpm/V effective vs {:.0} rpm/V unloaded (worst {:.0} %)",
            kv_eff / RPM, 1.0 / pair.k / RPM, 100.0 * lin_err
        );
        let k_from = motor_from.clone();
        let mut r_from = figures(m, &["resistance_mohm"]);
        let l_from = figures(m, &["inductance_uh"]);
        let b_from = figures(
            m,
            &[
                "no_load_current_a",
                "no_load_current_at_v",
                "kv_rpm_per_v",
                "resistance_mohm",
            ],
        );
        let j_from = figures(m, &["rotor_inertia_gcm2", "weight_g"]);
        match n_level {
            0 => {
                out.put("motor", "omegaMax", num(kv_eff * v), &all, speed_how);
                out.put("motor", "tau", num(tau), &all, tau_how);
            }
            1 => {
                out.put("motor", "Kv", num(kv_eff), &all, speed_how);
                out.put("motor", "tauOmega", num(tau), &all, tau_how);
            }
            2 => {
                out.put(
                    "motor",
                    "R",
                    num(pair.r),
                    &r_from,
                    "line-to-line resistance (two phases conduct in six-step)",
                );
                out.put("motor", "L", num(l), &l_from, "line-to-line inductance");
                out.put("motor", "k", num(pair.k), &k_from, "60 / (2 pi KV)");
                out.put("motor", "J", num(j_motor), &j_from, "rotor (bell) inertia");
                out.put(
                    "motor",
                    "b",
                    num(pair.b),
                    &b_from,
                    "no-load loss as viscous friction: k I0 / w0",
                );
            }
            _ => {
                out.put(
                    "motor",
                    "RPhase",
                    num(pair.r / 2.0),
                    &r_from,
                    "half the line-to-line resistance",
                );
                out.put(
                    "motor",
                    "LPhase",
                    num(l / 2.0),
                    &l_from,
                    "half the line-to-line inductance",
                );
                out.put(
                    "motor",
                    "ke",
                    num(pair.k / 2.0),
                    &k_from,
                    "per phase: the line-to-line constant 60 / (2 pi KV) over 2 (trapezoidal)",
                );
                out.put("motor", "J", num(j_motor), &j_from, "rotor (bell) inertia");
                out.put(
                    "motor",
                    "b",
                    num(pair.b),
                    &b_from,
                    "no-load loss as viscous friction: k I0 / w0",
                );
                out.put(
                    "motor",
                    "trapezoidal",
                    Value::Boolean(true),
                    &[],
                    "outrunner BLDC: trapezoidal back-EMF",
                );
                match m.f("magnet_poles") {
                    Some(p) => out.put(
                        "motor",
                        "polePairs",
                        Value::Integer((p / 2.0).round() as i64),
                        &figures(m, &["magnet_poles"]),
                        "magnet poles / 2",
                    ),
                    None => {
                        out.not_stated.insert(
                            m.figure("magnet_poles"),
                            "not stated: polePairs 7 (12N14P) assumed".into(),
                        );
                        out.put(
                            "motor",
                            "polePairs",
                            Value::Integer(7),
                            &[],
                            "assumed 12N14P",
                        );
                    }
                }
            }
        }
        r_from.clear();
        // Checks.
        let i_full = pair.current(w_full);
        let fmt_g = |w: f64| pair.kt * w * w / G * 1000.0;
        out.checks.push(format!(
            "full throttle at {v:.1} V: {:.0} rpm, {:.0} g thrust, {:.1} A, {:.0} W per motor",
            w_full / RPM,
            fmt_g(w_full),
            i_full,
            i_full * pair.voltage(w_full)
        ));
        if let Some((wh, n, mass)) = hover {
            let tw = n * pair.kt * w_full * w_full / (mass * G);
            out.checks.push(format!(
                "hover ({mass} kg, {n} rotors): {:.0} rpm, {:.0} % throttle, {:.1} A per motor; thrust/weight {tw:.2}{}",
                wh / RPM, 100.0 * pair.voltage(wh) / v, pair.current(wh),
                if tw < 1.0 { " -- CANNOT HOVER" } else if tw < 1.6 { " -- marginal (< 1.6)" } else { "" }
            ));
            if tw < 1.0 {
                return Err(format!(
                    "models.propulsion: these parts cannot lift {mass} kg (thrust/weight {tw:.2})"
                ));
            }
        }
        if let Some(imax) = m.f("max_continuous_current_a") {
            if i_full > imax {
                out.checks.push(format!(
                    "WARNING: full-throttle current {i_full:.1} A exceeds the motor's {imax} A"
                ));
            }
        }
        if let Some(pmax) = m.f("max_power_w") {
            let pw = i_full * pair.voltage(w_full);
            if pw > pmax {
                out.checks.push(format!(
                    "WARNING: full-throttle power {pw:.0} W exceeds the motor's {pmax} W"
                ));
            }
        }
        if let Some(p) = &prop {
            if let Some(max) = p.f("max_rpm") {
                if w_full / RPM > max {
                    out.checks.push(format!(
                        "WARNING: full-throttle {:.0} rpm exceeds the propeller's {max} rpm limit",
                        w_full / RPM
                    ));
                }
            }
        }
        if let Some(b) = &battery {
            let cells = b.f("cells_series").unwrap_or(0.0);
            for p in [Some(m), esc.as_ref()].into_iter().flatten() {
                if let (Some(lo), Some(hi)) = (p.f("cells_min"), p.f("cells_max")) {
                    if cells < lo || cells > hi {
                        out.checks.push(format!(
                            "WARNING: {}S battery outside {}'s {lo}-{hi}S",
                            cells,
                            p.name()
                        ));
                    }
                }
            }
            let limit = match (
                b.f("c_rating_continuous"),
                b.f("capacity_mah"),
                b.f("cell_discharge_continuous_a"),
            ) {
                (Some(c), Some(mah), _) => Some((c * mah / 1000.0, format!("{c}C"))),
                (_, _, Some(a)) => Some((
                    a * b.f("cells_parallel").unwrap_or(1.0),
                    "the cells' continuous rating".to_string(),
                )),
                _ => None,
            };
            if let (Some((limit, c)), Some(n)) = (limit, n_rotors) {
                let total = n * i_full;
                out.checks.push(format!(
                    "battery: {total:.0} A at full throttle vs {limit:.0} A continuous ({c}){}",
                    if total > limit {
                        " -- WARNING: exceeds"
                    } else {
                        ""
                    }
                ));
            }
        }
        if let Some(e) = &esc {
            if let (Some(lo), Some(hi)) = (e.f("min_voltage_v"), e.f("max_voltage_v")) {
                if v < lo || v > hi {
                    out.checks.push(format!(
                        "WARNING: {v:.1} V outside {}'s {lo}-{hi} V",
                        e.name()
                    ));
                }
            }
            if let Some(c) = e.f("current_continuous_a") {
                if i_full > c {
                    out.checks.push(format!(
                        "WARNING: full-throttle current {i_full:.1} A exceeds {}'s {c} A",
                        e.name()
                    ));
                }
            }
        }
        // The model against the manufacturer's bench table, at the row's
        // voltage x throttle (ESC throttle curves and battery sag make this
        // approximate).
        let (bench_prop, rows, same) = bench(m, prop.as_ref());
        if !rows.is_empty() {
            out.checks.push(format!(
                "bench table ({bench_prop}{}): model vs datasheet at voltage x throttle, or at the row's current{}",
                if same { "" } else { ", not the selected propeller" },
                if fitted_from_bench { "; the rotor was fitted to it" } else { "" }
            ));
            for r in rows {
                let vv = row_f(r, "voltage_v").unwrap_or(v);
                let (w, at) = match (row_f(r, "throttle_pct"), row_f(r, "current_a")) {
                    (Some(th), _) => (
                        pair.omega(vv * th / 100.0),
                        format!("{th:>3.0} % at {vv} V"),
                    ),
                    (None, Some(i)) => {
                        // The speed where the load takes this current: k I = kQ w^2 + b w.
                        let w = if pair.kq > 0.0 {
                            (-pair.b + (pair.b * pair.b + 4.0 * pair.kq * pair.k * i).sqrt())
                                / (2.0 * pair.kq)
                        } else {
                            pair.k * i / pair.b.max(1e-12)
                        };
                        (w, format!("{i:>5.1} A at {vv} V"))
                    }
                    _ => continue,
                };
                let mut line = format!("  {at}:");
                if let Some(rpm) = row_f(r, "rpm") {
                    line += &format!(" {:.0} rpm vs {rpm:.0} ({})", w / RPM, pct(w / RPM, rpm));
                }
                if let Some(g) = row_f(r, "thrust_g") {
                    line += &format!(", {:.0} g vs {g:.0} ({})", fmt_g(w), pct(fmt_g(w), g));
                }
                if row_f(r, "throttle_pct").is_some() {
                    if let Some(a) = row_f(r, "current_a") {
                        line += &format!(
                            ", {:.1} A vs {a:.1} ({})",
                            pair.current(w),
                            pct(pair.current(w), a)
                        );
                    }
                } else if vv > 0.0 {
                    line += &format!(", needs {:.1} V", pair.voltage(w));
                }
                out.checks.push(line);
            }
        }
    } else if n_level == 0 {
        out.assume(
            "motor: no motor part; [motor] omegaMax and tau stay as set or at their defaults",
        );
    } else {
        return Err(format!("{level} needs a motor part"));
    }

    // Power and ESC at P3/P4.
    if n_level >= 3 {
        if let Some(b) = &battery {
            let (r, from) = battery_resistance(b, &mut out);
            out.put(
                "power",
                "sourceResistance",
                num(r),
                &from,
                "battery internal resistance",
            );
        } else if ex("power", "sourceResistance").is_none() {
            return Err(format!(
                "{level} needs a battery part or [power] sourceResistance"
            ));
        }
        let e = esc.as_ref();
        let ef = |k: &str| e.and_then(|e| e.f(k));
        let efrom = |k: &[&str]| e.map(|e| figures(e, k)).unwrap_or_default();
        let ename = e
            .map(|e| e.name().to_string())
            .unwrap_or("no ESC part".into());
        match ef("capacitance_uf") {
            Some(c) => out.put(
                "power",
                "dcLinkCapacitance",
                num(c * 1e-6),
                &efrom(&["capacitance_uf"]),
                "the ESC's input capacitor",
            ),
            None => {
                out.assume(format!("{ename}: input capacitance not stated; 470 uF"));
                out.put("power", "dcLinkCapacitance", num(470e-6), &[], "assumed");
            }
        }
        let par = ef("mosfets_parallel").unwrap_or(1.0);
        let ron = match (ef("phase_resistance_mohm"), ef("mosfet_rds_on_mohm")) {
            (Some(r), _) => Some((
                r / 1000.0,
                efrom(&["phase_resistance_mohm"]),
                "the inverter's stated resistance per phase",
            )),
            (None, Some(r)) => Some((
                r / 1000.0 / par,
                efrom(&["mosfet_rds_on_mohm", "mosfets_parallel"]),
                "the MOSFET's Rds(on) / MOSFETs in parallel",
            )),
            _ => None,
        };
        if let (Some((_, from, _)), Some(e)) = (&ron, e) {
            for k in ["phase_resistance_mohm", "mosfet_rds_on_mohm"] {
                if e.t.contains_key(k) && !from.contains(&e.figure(k)) {
                    out.dropped.insert(
                        e.figure(k),
                        "superseded by the inverter's per-phase resistance".into(),
                    );
                }
            }
        }
        match ron.clone() {
            Some((r, from, how)) => out.put("esc", "Ron", num(r), &from, how),
            None => {
                out.assume(format!("{ename}: switch on-resistance not stated; 5 mOhm"));
                out.put("esc", "Ron", num(0.005), &[], "assumed");
            }
        }
        let (ron_v, ron_from) = ron.map(|(r, f, _)| (r, f)).unwrap_or((0.005, vec![]));
        out.assume(format!("{ename}: body-diode slope resistance taken as Ron"));
        out.put("esc", "Rdiode", num(ron_v), &ron_from, "taken as Ron");
        match ef("mosfet_diode_vf_v") {
            Some(v) => out.put(
                "esc",
                "Vdiode",
                num(v),
                &efrom(&["mosfet_diode_vf_v"]),
                "body-diode forward voltage",
            ),
            None => {
                out.assume(format!(
                    "{ename}: body-diode forward voltage not stated; 0.8 V"
                ));
                out.put("esc", "Vdiode", num(0.8), &[], "assumed");
            }
        }
        if n_level == 4 {
            match ef("pwm_frequency_khz") {
                Some(f) => out.put(
                    "esc",
                    "pwmFrequency",
                    num(f * 1000.0),
                    &efrom(&["pwm_frequency_khz"]),
                    "the ESC's PWM frequency",
                ),
                None => {
                    out.assume(format!(
                        "{ename}: PWM frequency not stated; 24 kHz (BLHeli_32 default)"
                    ));
                    out.put("esc", "pwmFrequency", num(24e3), &[], "assumed");
                }
            }
            match ef("dead_time_ns") {
                Some(d) => out.put(
                    "esc",
                    "deadTime",
                    num(d * 1e-9),
                    &efrom(&["dead_time_ns"]),
                    "dead time",
                ),
                None => {
                    out.not_stated
                        .insert("esc.dead_time_ns".into(), "not stated: deadTime 0".into());
                }
            }
        }
    }
    let _ = &l;
    let mut parts: Vec<&Part> = Vec::new();
    parts.extend(motor.as_ref());
    parts.extend(prop.as_ref());
    parts.extend(battery.as_ref());
    parts.extend(esc.as_ref());
    if n_level < 3 {
        for p in &parts {
            if p.kind == "esc" {
                for k in p.t.keys().filter(|k| {
                    k.starts_with("mosfet")
                        || k.starts_with("capacitance")
                        || k.starts_with("dead_time")
                        || k.starts_with("pwm")
                }) {
                    out.dropped.insert(
                        p.figure(k),
                        format!("switch figures: used at P3/P4, not at {level}"),
                    );
                }
            }
            if p.kind == "battery" {
                for k in p.t.keys().filter(|k| k.starts_with("internal_resistance")) {
                    out.dropped.insert(
                        p.figure(k),
                        format!("no source resistance below P3 ({level} has an ideal supply)"),
                    );
                }
            }
            if p.kind == "motor" && n_level < 2 {
                for k in ["inductance_uh", "rotor_inertia_gcm2"] {
                    if p.t.contains_key(k) && n_level == 0 {
                        out.dropped.insert(
                            p.figure(k),
                            format!("winding dynamics: not represented at {level}"),
                        );
                    }
                }
            }
            if p.kind == "motor" && n_level < 3 {
                out.dropped.insert(
                    p.figure("magnet_poles"),
                    format!("commutation: not represented at {level} (P3/P4)"),
                );
            }
        }
    }
    let mut models = Table::new();
    models.insert("esc".into(), Value::String(esc_model.into()));
    models.insert("motor".into(), Value::String(motor_model.into()));
    models.insert("rotor".into(), Value::String("quadratic_blade".into()));
    let (sections, report) = finish("propulsion", &parts, level, &model, out, &explicit);
    Ok((models, sections, report))
}

/// The bench rows to use: those with the selected propeller if the table has
/// it, else the first propeller's. (propeller name, rows, same as selected)
fn bench<'a>(m: &'a Part, prop: Option<&Part>) -> (String, Vec<&'a Table>, bool) {
    let rows = m.rows("test");
    let name = |r: &Table| {
        r.get("propeller")
            .and_then(Value::as_str)
            .unwrap_or("?")
            .to_string()
    };
    let Some(first) = rows.first() else {
        return (String::new(), rows, false);
    };
    let pick = prop
        .and_then(|p| rows.iter().map(|r| name(r)).find(|n| same_propeller(p, n)))
        .map(|n| (n, true))
        .unwrap_or((name(first), prop.is_none()));
    let chosen = rows.into_iter().filter(|r| name(r) == pick.0).collect();
    (pick.0, chosen, pick.1)
}

fn same_propeller(p: &Part, bench: &str) -> bool {
    let (Some(d), Some(pitch)) = (p.f("diameter_in"), p.f("pitch_in")) else {
        return false;
    };
    let digits: String = bench.chars().filter(|c| c.is_ascii_digit()).collect();
    let want = format!("{}{}", d as i64, (pitch * 10.0).round() as i64);
    digits.contains(&want)
}

// --- servos ------------------------------------------------------------------------

const SERVO_REASONS: [(&str, &str); 9] = [
    (
        "torque_kgcm",
        "stall torque: the servo models have no hinge-moment load",
    ),
    (
        "torque_*",
        "stall torque: the servo models have no hinge-moment load",
    ),
    ("weight_g", "mass: part of the airframe's mass"),
    (
        "voltage_min_v",
        "supply range: the servo supply is not modelled",
    ),
    (
        "voltage_max_v",
        "supply range: the servo supply is not modelled",
    ),
    ("digital", "digital/analog: no update-rate model"),
    (
        "pulse_neutral_us",
        "neutral: the PWM calibration is [interface]'s",
    ),
    (
        "speed_*",
        "the speed at the other rated voltage (voltage picks one)",
    ),
    ("travel_*", "travel: only its angle per microsecond is used"),
];

/// `[models.servos|steering] part = servo`: a plane's control-surface servos
/// or a rover's steering servo.
fn servo(
    root: &Table,
    key: &str,
    sel: &Table,
    kind: &crate::vehicle::Kind,
) -> Result<(String, Table, Json)> {
    for k in sel.keys() {
        if !["part", "level", "voltage"].contains(&k.as_str()) {
            return Err(format!(
                "models.{key}.{k}: the options are part, level, voltage"
            ));
        }
    }
    let p = Part::load(
        "servo",
        sel["part"]
            .as_str()
            .ok_or(format!("models.{key}.part: expected a string"))?,
    )?;
    let actuator = kind.actuators.iter().find(|a| a.key == key).unwrap();
    let level = sel.get("level").and_then(Value::as_str).unwrap_or("S1");
    let (model, ..) = actuator
        .levels
        .iter()
        .find(|(m, l, ..)| *l == level || *m == level)
        .ok_or(format!("models.{key}.level = {level:?}: S0 or S1"))?;
    let plant = root
        .get("plant")
        .and_then(Value::as_table)
        .cloned()
        .unwrap_or_default();
    let explicit = BTreeMap::from([("plant".to_string(), plant.clone())]);
    let (max_key, max_default, rate_key, band_key) = if key == "servos" {
        (
            "maxAileron",
            FIXED_WING_MAX_AILERON,
            "maxServoRate",
            "servoDeadband",
        )
    } else {
        (
            "maxSteerAngle",
            ROVER_MAX_STEER,
            "maxSteerRate",
            "steeringDeadband",
        )
    };
    let mut out = Out::default();
    let max = plant.get(max_key).and_then(float).unwrap_or(max_default);
    let iface = root.get("interface").and_then(Value::as_table);
    let lo = iface
        .and_then(|t| t.get("pwm_min"))
        .and_then(float)
        .unwrap_or(kind.pwm.0);
    let hi = iface
        .and_then(|t| t.get("pwm_max"))
        .and_then(float)
        .unwrap_or(kind.pwm.1);
    let half = (hi - lo) / 2.0;
    let deg_per_us = match p.f("operating_angle_deg_per_us") {
        Some(x) => (x, figures(&p, &["operating_angle_deg_per_us"])),
        None => match (
            p.f("travel_deg"),
            p.t.get("travel_pulse_us").and_then(Value::as_array),
        ) {
            (Some(deg), Some(a)) if a.len() == 2 => {
                let span = float(&a[1]).unwrap_or(0.0) - float(&a[0]).unwrap_or(0.0);
                (deg / span, figures(&p, &["travel_deg", "travel_pulse_us"]))
            }
            _ => {
                out.assume(format!(
                    "{}: angle per microsecond not stated; 0.09 deg/us (90 deg over 1000 us)",
                    p.name()
                ));
                (0.09, vec![])
            }
        },
    };
    let servo_deg = deg_per_us.0 * half;
    let ratio = max.to_degrees() / servo_deg;
    out.assume(format!(
        "linkage: full command (+/-{half:.0} us = {servo_deg:.0} deg of servo) gives {max_key} = {:.1} deg, a {ratio:.2}:1 linkage",
        max.to_degrees()
    ));
    let want_v = sel.get("voltage").and_then(float);
    let (t60, at_v, from) = match (want_v, p.f("speed_low_at_v")) {
        (Some(v), Some(low)) if (v - low).abs() < 1e-6 => (
            p.f("speed_low_s_per_60deg")
                .ok_or("servo: speed_low_s_per_60deg missing")?,
            low,
            figures(&p, &["speed_low_s_per_60deg", "speed_low_at_v"]),
        ),
        (Some(v), _) if p.f("speed_at_v").is_some_and(|x| (x - v).abs() > 1e-6) => {
            return Err(format!(
                "models.{key}.voltage = {v}: {} states speeds at {} V{}",
                p.name(),
                p.f("speed_at_v").unwrap(),
                p.f("speed_low_at_v")
                    .map(|x| format!(" and {x} V"))
                    .unwrap_or_default()
            ));
        }
        _ => (
            p.f("speed_s_per_60deg")
                .ok_or(format!("servo {}: speed_s_per_60deg is required", p.id))?,
            p.f("speed_at_v").unwrap_or(0.0),
            figures(&p, &["speed_s_per_60deg", "speed_at_v"]),
        ),
    };
    let rate = (60f64).to_radians() / t60 * ratio;
    let mut rfrom = from.clone();
    rfrom.extend(deg_per_us.1.clone());
    if *model == "rate_limited" {
        out.put("plant", rate_key, num(rate), &rfrom, format!(
            "60 deg / {t60} s at {at_v} V, through the {ratio:.2}:1 linkage (unloaded: the datasheet speed has no load)"));
        let range =
            p.t.get("deadband_range_us")
                .and_then(Value::as_array)
                .filter(|a| a.len() == 2)
                .map(|a| (float(&a[0]).unwrap_or(0.0) + float(&a[1]).unwrap_or(0.0)) / 2.0);
        if p.f("deadband_us").is_none() && range.is_some() {
            out.assume(format!(
                "{}: dead band stated as a range; its midpoint {} us",
                p.name(),
                range.unwrap()
            ));
        }
        match p
            .f("deadband_us")
            .map(|d| (d, "deadband_us"))
            .or(range.map(|d| (d, "deadband_range_us")))
        {
            Some((db, key)) => out.put(
                "plant",
                band_key,
                num(db / half * max),
                &figures(&p, &[key]),
                format!("{db} us of the +/-{half:.0} us command span, as {max_key}"),
            ),
            None => {
                out.not_stated.insert(
                    p.figure("deadband_us"),
                    format!("not stated: {band_key} stays at the library default"),
                );
            }
        }
    } else {
        for k in [
            "speed_s_per_60deg",
            "speed_at_v",
            "deadband_us",
            "deadband_range_us",
        ] {
            out.dropped.insert(p.figure(k), format!("rate and deadband: not represented at {level} ({model}: first-order, no rate limit)"));
        }
    }
    out.not_stated.insert(
        "servo.time_constant".into(),
        "no datasheet states a small-signal time constant: the first-order lag stays at the library default".into(),
    );
    for k in deg_per_us.1.iter() {
        out.used.push(k.clone());
    }
    let (sections, report) = finish(key, &[&p], level, model, out, &explicit);
    Ok((
        model.to_string(),
        sections.get("plant").cloned().unwrap_or_default(),
        report,
    ))
}

// --- rover drive ------------------------------------------------------------------

fn drive(root: &Table, sel: &Table, kind: &crate::vehicle::Kind) -> Result<(String, Table, Json)> {
    for k in sel.keys() {
        if !["part", "level", "count", "battery", "battery_state"].contains(&k.as_str()) {
            return Err(format!(
                "models.drive.{k}: the options are part, level, count, battery, battery_state"
            ));
        }
    }
    let p = Part::load(
        "gearmotor",
        sel["part"]
            .as_str()
            .ok_or("models.drive.part: expected a string")?,
    )?;
    let battery = sel
        .get("battery")
        .and_then(Value::as_str)
        .map(|b| Part::load("battery", b))
        .transpose()?;
    let actuator = kind.actuators.iter().find(|a| a.key == "drive").unwrap();
    let level = sel.get("level").and_then(Value::as_str).unwrap_or("D1");
    let (model, ..) = actuator
        .levels
        .iter()
        .find(|(m, l, ..)| *l == level || *m == level)
        .ok_or(format!("models.drive.level = {level:?}: D0 or D1"))?;
    let count = sel.get("count").and_then(float).unwrap_or(1.0);
    let plant = root
        .get("plant")
        .and_then(Value::as_table)
        .cloned()
        .unwrap_or_default();
    let explicit = BTreeMap::from([("plant".to_string(), plant.clone())]);
    let mut out = Out::default();
    let rated = p
        .f("rated_voltage_v")
        .ok_or(format!("gearmotor {}: rated_voltage_v is required", p.id))?;
    let (v, vfrom, vhow) = if let Some(v) = plant.get("supplyVoltage").and_then(float) {
        (v, vec![], "explicit [plant] supplyVoltage".to_string())
    } else if let Some(b) = &battery {
        let state = sel
            .get("battery_state")
            .and_then(Value::as_str)
            .unwrap_or("nominal");
        let (v, from) = battery_voltage(b, state, &mut out)?;
        (v, from, format!("the battery, {state}"))
    } else {
        (
            rated,
            figures(&p, &["rated_voltage_v"]),
            "the rated voltage".to_string(),
        )
    };
    let s = v / rated;
    if (s - 1.0).abs() > 1e-9 {
        out.assume(format!(
            "torque and speed scaled linearly from {rated} V to {v:.2} V"
        ));
    }
    let nl = p
        .f("no_load_speed_rpm")
        .ok_or("gearmotor: no_load_speed_rpm is required")?;
    let st = p
        .f("stall_torque_kgcm")
        .ok_or("gearmotor: stall_torque_kgcm is required")?;
    let mut from = figures(&p, &["no_load_speed_rpm", "rated_voltage_v"]);
    from.extend(vfrom.clone());
    out.put(
        "plant",
        "motorNoLoadSpeed",
        num(nl * RPM * s),
        &from,
        format!("output-shaft no-load speed x {s:.3}"),
    );
    let mut from = figures(&p, &["stall_torque_kgcm", "rated_voltage_v"]);
    from.extend(vfrom.clone());
    out.put(
        "plant",
        "motorStallTorque",
        num(st * KGCM * s * count),
        &from,
        format!("output-shaft stall torque x {s:.3} x {count} motor(s) on the axle"),
    );
    if count != 1.0 {
        out.assume(format!("{count} identical motors share the axle (count)"));
    }
    if *model == "dc_motor" {
        out.put("plant", "supplyVoltage", num(v), &vfrom, vhow);
        out.put(
            "plant",
            "gearRatio",
            num(p
                .f("gear_ratio")
                .ok_or("gearmotor: gear_ratio is required")?),
            &figures(&p, &["gear_ratio"]),
            "gear ratio",
        );
        out.not_stated.insert(
            p.figure("inductance_mh"),
            "not stated: motorInductance stays at the library default".into(),
        );
        if let Some(ist) = p.f("stall_current_a") {
            // The plant's line model: k = V / (N w0), R = N k V / T_stall, so
            // its stall current is T_stall w0 / V.
            let model = st * KGCM * s * count * nl * RPM * s / v;
            let data = ist * s * count;
            out.checks.push(format!(
                "stall current: the plant's {model:.1} A vs the datasheet's {data:.1} A ({}; the line model has no no-load loss or gear efficiency)",
                pct(model, data)
            ));
            out.used.push(p.figure("stall_current_a"));
        }
    } else {
        out.dropped.insert(
            p.figure("gear_ratio"),
            "gearing: not represented at D0".into(),
        );
    }
    let mut parts = vec![&p];
    parts.extend(battery.as_ref());
    let (sections, report) = finish("drive", &parts, level, model, out, &explicit);
    Ok((
        model.to_string(),
        sections.get("plant").cloned().unwrap_or_default(),
        report,
    ))
}

// --- expanding a configuration ---------------------------------------------------------

/// Replace powertrain part selections by their models and parameters.
pub fn expand(root: &mut Table, reports: &mut Vec<Json>) -> Result<()> {
    let vehicle = root
        .get("vehicle")
        .and_then(Value::as_str)
        .unwrap_or("multirotor")
        .to_string();
    let Some(models) = root.get("models").and_then(Value::as_table).cloned() else {
        return Ok(());
    };
    let merge = |root: &mut Table, section: &str, params: Table| {
        let mut t = root
            .get(section)
            .and_then(Value::as_table)
            .cloned()
            .unwrap_or_default();
        for (k, v) in params {
            t.entry(k).or_insert(v);
        }
        if !t.is_empty() {
            root.insert(section.into(), Value::Table(t));
        }
    };
    if let Some(sel) = models.get("propulsion") {
        if vehicle != "multirotor" {
            return Err("models.propulsion is for multirotors (a rover has models.drive, a plane its plant)".into());
        }
        let sel = sel
            .as_table()
            .ok_or("models.propulsion: expected a table")?;
        for k in ["esc", "motor", "rotor"] {
            if models
                .get(k)
                .is_some_and(|v| !(k == "rotor" && v.as_str() == Some("quadratic_blade")))
            {
                return Err(format!(
                    "models.{k}: set by models.propulsion (its level); remove one"
                ));
            }
        }
        let (m, sections, report) = propulsion(root, sel)?;
        for (section, params) in sections {
            merge(root, &section, params);
        }
        let mt = root.get_mut("models").unwrap().as_table_mut().unwrap();
        mt.remove("propulsion");
        mt.extend(m);
        reports.push(report);
        return Ok(());
    }
    if vehicle == "multirotor" {
        return Ok(());
    }
    let kind = crate::vehicle::kind(&vehicle)?;
    for key in ["servos", "steering", "drive"] {
        let Some(sel) = models.get(key).and_then(Value::as_table) else {
            continue;
        };
        if !kind.actuators.iter().any(|a| a.key == key) {
            return Err(format!("models.{key}: a {vehicle} has no {key}"));
        }
        let (model, params, report) = if key == "drive" {
            drive(root, sel, kind)?
        } else {
            servo(root, key, sel, kind)?
        };
        merge(root, "plant", params);
        root.get_mut("models")
            .unwrap()
            .as_table_mut()
            .unwrap()
            .insert(key.into(), Value::String(model));
        reports.push(report);
    }
    Ok(())
}

/// The report of one powertrain selection, as `fire-compose part` shows it:
/// `kind` is a powertrain part kind, `sel` the selection table and `root` a
/// configuration around it (vehicle, mass, geometry, explicit tables).
pub fn report(kind: &str, sel: &Table, root: &Table) -> Result<Json> {
    let mut root = root.clone();
    let (vehicle, key) = match kind {
        "servo" => {
            let v = sel
                .get("vehicle")
                .and_then(Value::as_str)
                .unwrap_or("fixed_wing")
                .to_string();
            let key = if v == "rover" { "steering" } else { "servos" };
            (v, key)
        }
        "gearmotor" => ("rover".to_string(), "drive"),
        _ => ("multirotor".to_string(), "propulsion"),
    };
    let mut sel = sel.clone();
    sel.remove("vehicle");
    if vehicle != "multirotor" {
        root.insert("vehicle".into(), Value::String(vehicle));
    }
    let mut models = Table::new();
    models.insert(key.into(), Value::Table(sel));
    root.insert("models".into(), Value::Table(models));
    let mut reports = Vec::new();
    expand(&mut root, &mut reports)?;
    reports.pop().ok_or_else(|| "no report".into())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn steady_state_balances_torque() {
        let p = Pair {
            k: 0.0104,
            r: 0.133,
            b: 1e-6,
            kt: 1.2e-5,
            kq: 1.9e-7,
        };
        let w = p.omega(12.0);
        let i = (12.0 - p.k * w) / p.r;
        assert!((p.k * i - p.kq * w * w - p.b * w).abs() < 1e-9);
        assert!((p.voltage(w) - 12.0).abs() < 1e-9);
    }

    #[test]
    fn square_fit_is_exact_on_quadratic_data() {
        let xy: Vec<(f64, f64)> = (1..8)
            .map(|i| (i as f64 * 100.0, 2e-5 * (i as f64 * 100.0).powi(2)))
            .collect();
        assert!((fit_square(&xy) - 2e-5).abs() < 1e-12);
        assert!(worst(&xy, 2e-5) < 1e-12);
    }

    #[test]
    fn library_defaults_match_the_plants() {
        let root = crate::artifact::library_root();
        let fw = std::fs::read_to_string(root.join("Vehicles/FixedWing/FixedWingPlant.mo"))
            .unwrap_or_default();
        let rv =
            std::fs::read_to_string(root.join("Vehicles/Rover/RoverPlant.mo")).unwrap_or_default();
        if fw.is_empty() || rv.is_empty() {
            return;
        }
        assert!(fw.contains(&format!("maxAileron = {FIXED_WING_MAX_AILERON}")));
        assert!(rv.contains(&format!("maxSteerAngle = {ROVER_MAX_STEER}")));
    }

    /// Every powertrain part converts at every level it serves, alone or in
    /// the assemblies the parts directory allows.
    #[test]
    fn every_powertrain_part_converts() {
        let all = crate::parts::all();
        let ids = |k: &str| {
            all.iter()
                .filter(|(kind, ..)| kind == k)
                .map(|(_, id, _)| id.clone())
                .collect::<Vec<_>>()
        };
        let battery = ids("battery").into_iter().find(|b| b.contains("4s"));
        let prop = ids("propeller").first().cloned();
        for m in ids("motor") {
            for level in ["P0", "P1", "P2", "P3", "P4"] {
                let mut sel = Table::new();
                sel.insert("motor".into(), Value::String(m.clone()));
                if let Some(p) = &prop {
                    sel.insert("propeller".into(), Value::String(p.clone()));
                }
                if let Some(b) = &battery {
                    sel.insert("battery".into(), Value::String(b.clone()));
                } else {
                    continue;
                }
                sel.insert("level".into(), Value::String(level.into()));
                report("motor", &sel, &Table::new()).unwrap_or_else(|e| panic!("{m} {level}: {e}"));
            }
        }
        for s in ids("servo") {
            for (vehicle, level) in [("fixed_wing", "S0"), ("fixed_wing", "S1"), ("rover", "S1")] {
                let mut sel = Table::new();
                sel.insert("part".into(), Value::String(s.clone()));
                sel.insert("level".into(), Value::String(level.into()));
                sel.insert("vehicle".into(), Value::String(vehicle.into()));
                report("servo", &sel, &Table::new())
                    .unwrap_or_else(|e| panic!("{s} {vehicle} {level}: {e}"));
            }
        }
        for g in ids("gearmotor") {
            for level in ["D0", "D1"] {
                let mut sel = Table::new();
                sel.insert("part".into(), Value::String(g.clone()));
                sel.insert("level".into(), Value::String(level.into()));
                report("gearmotor", &sel, &Table::new())
                    .unwrap_or_else(|e| panic!("{g} {level}: {e}"));
            }
        }
    }
}
