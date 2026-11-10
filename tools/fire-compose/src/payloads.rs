//! Payloads: ArduPilot's optional hardware that loads or moves the airframe
//! (ModelicaCPSLibrary.Systems.Payloads), selected per kind:
//!
//! ```toml
//! [payloads.winch]
//! part = "okaya_daiwa_winch"   # optional: a real device (parts/winch/)
//! position = [0.0, 0.0, 0.08]  # attachment, body FRD from the combined CG [m]
//! mass = 1.0                   # any parameter of the model
//! ```
//!
//! Each payload reads the vehicle's truth; those with servos read the vehicle
//! interface's `aux_pwm` (payload_actuation_v1, element k = host servo 9+k),
//! assigned in the order of [`KINDS`]. Their forces and moments are summed
//! with the host's external wrench (`Payloads.Bus`) into the plant. Power
//! sources (generator, tether) supply the power module's pack, which they
//! need.
use crate::composition::{Instance, Modifier, Prefix, Value};
use crate::config::{array, number, table, ResolvedFireConfig, Table};
use crate::Result;
use serde_json::{json, Value as Json};
use toml::Value as Toml;

pub struct Kind {
    pub name: &'static str,
    pub title: &'static str,
    pub class: &'static str,
    pub component: &'static str,
    /// Model parameters a configuration may set (all real-valued).
    pub parameters: &'static [&'static str],
    /// ArduPilot servo functions of its channels, in order, with their start pulses.
    pub servos: &'static [(u32, f64)],
    /// A power source for the power module's pack.
    pub supply: bool,
    /// Exported diagnostics: (variable, the component's output, dimension).
    pub outputs: &'static [(&'static str, &'static str, usize)],
    pub ardupilot: &'static str,
    pub description: &'static str,
}

const P: &str = "ModelicaCPSLibrary.Systems.Payloads.";
pub const KINDS: [&Kind; 8] = [
    &Kind {
        name: "gripper",
        title: "Gripper",
        class: "Gripper",
        component: "gripperPayload",
        parameters: &[
            "mass", "stiffness", "damping", "groundDown", "groundStiffness", "groundDamping",
            "groundFriction", "hookLength", "releasePulse", "grabPulse",
        ],
        servos: &[(28, 1500.0)],
        supply: false,
        outputs: &[("gripper_holding", "holdingLevel", 0), ("gripper_tension_n", "tension", 0)],
        ardupilot: "Grippers",
        description: "a load on a short hook line, released and grabbed by its servo",
    },
    &Kind {
        name: "winch",
        title: "Winch",
        class: "Winch",
        component: "winchPayload",
        parameters: &[
            "mass", "stiffness", "damping", "groundDown", "groundStiffness", "groundDamping",
            "groundFriction", "maxRate", "maxLength", "length_start", "deadband",
        ],
        servos: &[(88, 1500.0)],
        supply: false,
        outputs: &[("winch_line_m", "paidOut", 0), ("winch_tension_n", "tension", 0)],
        ardupilot: "Winch",
        description: "a load on a line paid out and wound in at the servo's rate",
    },
    &Kind {
        name: "parachute",
        title: "Parachute",
        class: "Parachute",
        component: "parachutePayload",
        parameters: &[
            "diameter", "dragCoefficient", "deployDelay", "inflationTime", "releasePulse",
            "pressure0", "temperature0",
        ],
        servos: &[(27, 1100.0)],
        supply: false,
        outputs: &[("parachute_inflation", "inflation", 0), ("parachute_drag_n", "drag", 0)],
        ardupilot: "Parachute",
        description: "a canopy released by its servo, deploying, inflating, then drag",
    },
    &Kind {
        name: "sprayer",
        title: "Crop sprayer",
        class: "Sprayer",
        component: "sprayerPayload",
        parameters: &["tankVolume", "volume_start", "maxFlow", "density", "pwmMin", "pwmMax"],
        servos: &[(22, 1000.0)],
        supply: false,
        outputs: &[("sprayer_volume_l", "volume", 0), ("sprayer_flow_lpm", "flowRate", 0)],
        ardupilot: "Crop Sprayer",
        description: "a liquid tank emptied by its pump; the liquid's weight on the vehicle",
    },
    &Kind {
        name: "landing_gear",
        title: "Retractable landing gear",
        class: "LandingGear",
        component: "landingGearPayload",
        parameters: &["retractTime", "dragArea", "pwmMin", "pwmMax"],
        servos: &[(29, 2000.0)],
        supply: false,
        outputs: &[("landing_gear_deployed", "deployed", 0)],
        ardupilot: "Landing Gear/ Retractable Camera Mount",
        description: "retract servo (deployed at the maximum pulse) at a finite speed; the deployed gear's drag",
    },
    &Kind {
        name: "gimbal",
        title: "Gimbal",
        class: "Gimbal",
        component: "gimbalPayload",
        parameters: &["angleMin", "angleMax", "maxRate", "timeConstant", "pwmMin", "pwmMax", "groundDown"],
        servos: &[(7, 1500.0), (8, 1500.0), (6, 1500.0)],
        supply: false,
        outputs: &[("gimbal_angle", "angle", 3), ("gimbal_ground_point_ned", "groundPoint_w", 3)],
        ardupilot: "Cameras & Gimbals",
        description: "tilt, roll and pan following their servos at limited rates; the camera's ground point",
    },
    &Kind {
        name: "tether",
        title: "Power tether",
        class: "Tether",
        component: "tetherPayload",
        parameters: &["anchor_w", "tension0", "massPerLength", "maxLength", "maxPower"],
        servos: &[],
        supply: true,
        outputs: &[("tether_length_m", "length", 0), ("tether_power_w", "supply", 0)],
        ardupilot: "Power Tether",
        description: "constant winch tension toward the ground station, half the tether's weight, power up to its limit",
    },
    &Kind {
        name: "generator",
        title: "Generator",
        class: "Generator",
        component: "generatorPayload",
        parameters: &["ratedPower", "fuelRate", "tankVolume", "fuel_start", "fuelDensity"],
        servos: &[],
        supply: true,
        outputs: &[
            ("generator_power_w", "supply", 0),
            ("fuel_flow_lph", "fuelFlow", 0),
            ("fuel_level", "fuelLevel", 0),
        ],
        ardupilot: "Generators; Fuel Flow and Level Sensors",
        description: "follows the electrical demand up to its rating, burning fuel; the fuel's weight",
    },
];

pub fn kind(name: &str) -> Option<&'static Kind> {
    KINDS.iter().copied().find(|k| k.name == name)
}

fn vector(kind: &str, name: &str) -> Option<usize> {
    match (kind, name) {
        ("tether", "anchor_w") => Some(3),
        ("gimbal", "angleMin" | "angleMax" | "maxRate") => Some(3),
        _ => None,
    }
}

/// `[payloads]`, checked.
pub fn resolve(raw: &Toml, peripherals: &Table) -> Result<Table> {
    let names: Vec<&str> = KINDS.iter().map(|k| k.name).collect();
    let t = table(raw, &names, "payloads")?;
    for (name, sel) in t {
        let kind = kind(name).unwrap();
        let label = format!("payloads.{name}");
        let mut keys = vec!["position"];
        keys.extend(kind.parameters);
        let sel = table(sel, &keys, &label)?;
        for (k, v) in sel {
            let label = format!("{label}.{k}");
            if k == "position" {
                array(v, &[3], &label)?;
            } else if let Some(n) = vector(name, k) {
                array(v, &[n], &label)?;
            } else {
                number(v, &label, false, k.starts_with("ground") && k != "groundDown")?;
            }
        }
        if kind.supply && !peripherals.contains_key("power_module") {
            return Err(format!(
                "{label}: a {} supplies the power module's pack: add [peripherals.power_module]",
                kind.title.to_lowercase()
            ));
        }
    }
    Ok(t.clone())
}

fn reference(name: &str) -> Value {
    Value::Reference(name.into())
}

/// The first aux channel (1-based) of each selected payload with servos.
pub fn channels(payloads: &Table) -> Vec<(&'static Kind, usize)> {
    let mut next = 1;
    let mut out = Vec::new();
    for kind in KINDS {
        if payloads.contains_key(kind.name) && !kind.servos.is_empty() {
            out.push((kind, next));
            next += kind.servos.len();
        }
    }
    out
}

/// Whether the configuration has payloads (the plant's external force then comes from the bus).
pub fn any(c: &ResolvedFireConfig) -> bool {
    !c.payloads.is_empty()
}

/// The plant's external force and moment: the bus's totals with payloads, else the host's.
pub fn plant_wrench(c: &ResolvedFireConfig, force: &str, moment: &str) -> (Value, Value) {
    if any(c) {
        (reference("payloadBus.totalForce"), reference("payloadBus.totalMoment"))
    } else {
        (reference(force), reference(moment))
    }
}

/// The payloads, the aux servo inputs, the bus and the diagnostics.
pub fn components(
    c: &ResolvedFireConfig,
    host_force: &str,
    host_moment: &str,
    components: &mut Vec<Instance>,
    equalities: &mut Vec<(String, Value)>,
) -> Result<()> {
    if c.payloads.is_empty() {
        return Ok(());
    }
    let assigned = channels(&c.payloads);
    let n_aux: usize = assigned.iter().map(|(k, _)| k.servos.len()).sum();
    if n_aux > 0 {
        let mut starts = Vec::new();
        for (k, _) in &assigned {
            starts.extend(k.servos.iter().map(|(_, s)| Value::Real(*s)));
        }
        let mut aux = Instance {
            class: "Real".into(),
            name: "aux_pwm".into(),
            modifiers: vec![Modifier::Bind("start".into(), Value::Array(starts))],
            dimensions: vec![n_aux],
            prefix: Prefix::Input,
            binding: None,
        };
        aux.modifiers.push(Modifier::Bind("unit".into(), Value::Text("us".into())));
        components.push(aux);
    }
    let mut bus = 0;
    let mut supplies = Vec::new();
    for kind in KINDS {
        let Some(sel) = c.payloads.get(kind.name).and_then(Toml::as_table) else {
            continue;
        };
        let mut modifiers = Vec::new();
        for (k, v) in sel {
            let name = if k == "position" { "position_b" } else { k.as_str() };
            modifiers.push(Modifier::Bind(name.into(), crate::lower::literal(v)?));
        }
        if let Some((_, first)) = assigned.iter().find(|(k, _)| k.name == kind.name) {
            let pulse = if kind.servos.len() == 1 {
                reference(&format!("aux_pwm[{first}]"))
            } else {
                Value::Array(
                    (0..kind.servos.len())
                        .map(|i| reference(&format!("aux_pwm[{}]", first + i)))
                        .collect(),
                )
            };
            modifiers.push(Modifier::Bind("pulse".into(), pulse));
        }
        if kind.supply {
            modifiers.push(Modifier::Bind("utilization".into(), reference("powerModule.utilization")));
            supplies.push(reference(&format!("{}.capacity", kind.component)));
        }
        components.push(Instance {
            class: format!("{P}{}", kind.class),
            name: kind.component.into(),
            modifiers,
            dimensions: vec![],
            prefix: Prefix::Component,
            binding: None,
        });
        equalities.push((format!("{}.truth", kind.component), reference("truth")));
        bus += 1;
        equalities.push((
            format!("payloadBus.force[{bus}]"),
            reference(&format!("{}.force_b", kind.component)),
        ));
        equalities.push((
            format!("payloadBus.moment[{bus}]"),
            reference(&format!("{}.moment_b", kind.component)),
        ));
        for (export, field, dim) in kind.outputs {
            components.push(Instance {
                class: "Real".into(),
                name: (*export).into(),
                modifiers: vec![],
                dimensions: if *dim == 0 { vec![] } else { vec![*dim] },
                prefix: Prefix::Output,
                binding: Some(reference(&format!("{}.{field}", kind.component))),
            });
        }
    }
    components.push(Instance {
        class: format!("{P}Bus"),
        name: "payloadBus".into(),
        modifiers: vec![
            Modifier::Bind("n".into(), Value::Integer(bus as i64)),
            Modifier::Bind("hostForce".into(), reference(host_force)),
            Modifier::Bind("hostMoment".into(), reference(host_moment)),
        ],
        dimensions: vec![],
        prefix: Prefix::Component,
        binding: None,
    });
    if !supplies.is_empty() {
        // The power module (crate::peripherals) takes the sources' power.
        for i in components.iter_mut() {
            if i.name == "powerModule" {
                i.modifiers.push(Modifier::Bind(
                    "nSupplies".into(),
                    Value::Integer(supplies.len() as i64),
                ));
                i.modifiers.push(Modifier::Bind(
                    "supplyPower".into(),
                    Value::Array(supplies.clone()),
                ));
            }
        }
    }
    Ok(())
}

pub fn catalog() -> Json {
    Json::Array(
        KINDS
            .iter()
            .map(|k| {
                json!({
                    "name": k.name, "title": k.title, "description": k.description,
                    "ardupilot": k.ardupilot, "keys": k.parameters,
                    "servo_functions": k.servos.iter().map(|s| s.0).collect::<Vec<_>>(),
                    "power_source": k.supply,
                    "outputs": k.outputs.iter().map(|o| o.0).collect::<Vec<_>>(),
                })
            })
            .collect(),
    )
}

// --- real parts ---------------------------------------------------------------

fn figure(part: &Table, key: &str) -> Option<f64> {
    let v = part.get(key)?;
    v.as_float()
        .or_else(|| v.as_integer().map(|n| n as f64))
        .filter(|x| x.is_finite())
}

/// A payload part's figures as the model's parameters: (parameters, report).
fn convert(kind: &str, part: &Table) -> (Table, Vec<(String, String)>, Vec<String>) {
    let mut out = Table::new();
    let mut how = Vec::new();
    let mut assumed = Vec::new();
    let mut put = |out: &mut Table, key: &str, v: f64, why: String| {
        out.insert(key.into(), Toml::Float(v));
        how.push((key.to_string(), why));
    };
    let f = |k: &str| figure(part, k);
    match kind {
        "winch" => {
            if let Some(v) = f("line_speed_mps").or(f("payout_speed_mps")) {
                put(&mut out, "maxRate", v, format!("line speed {v} m/s"));
            }
            if let Some(v) = f("line_length_m") {
                put(&mut out, "maxLength", v, format!("{v} m of line"));
            }
            if let Some(kg) = f("max_load_kg") {
                assumed.push(format!("the load's mass is the configuration's (the winch lifts up to {kg} kg)"));
            }
        }
        "parachute" => {
            if let Some(v) = f("canopy_diameter_m") {
                put(&mut out, "diameter", v, "canopy diameter".into());
            } else if let Some(inch) = f("canopy_diameter_in") {
                put(&mut out, "diameter", inch * 0.0254, format!("{inch} in"));
            }
            if let Some(v) = f("drag_coefficient") {
                put(&mut out, "dragCoefficient", v, "drag coefficient".into());
            }
            if let Some(v) = f("deployment_time_s") {
                put(&mut out, "deployDelay", v, "deployment time".into());
            }
        }
        "sprayer" => {
            if let Some(v) = f("flow_rate_lpm").or(f("max_flow_rate_lpm")) {
                put(&mut out, "maxFlow", v, format!("{v} l/min pump"));
            }
            if let Some(r) = part.get("pwm_range_us").and_then(Toml::as_array) {
                let x = |i: usize| r.get(i).and_then(|v| v.as_float().or(v.as_integer().map(|n| n as f64)));
                if let (Some(a), Some(b)) = (x(0), x(1)) {
                    put(&mut out, "pwmMin", a, "PWM range".into());
                    put(&mut out, "pwmMax", b, "PWM range".into());
                }
            }
            if let Some(v) = f("tank_capacity_l") {
                put(&mut out, "tankVolume", v, "tank".into());
                put(&mut out, "volume_start", v, "a full tank".into());
            }
        }
        "landing_gear" => {
            if let Some(v) = f("retract_time_s") {
                put(&mut out, "retractTime", v, "retract time".into());
            } else {
                assumed.push("retractTime 2 s (not stated)".into());
            }
        }
        "gimbal" => {
            let range = |k: &str| -> Option<(f64, f64)> {
                let a = part.get(k)?.as_array()?;
                let x = |i: usize| a.get(i).and_then(|v| v.as_float().or(v.as_integer().map(|n| n as f64)));
                Some((x(0)?, x(1)?))
            };
            let deg = std::f64::consts::PI / 180.0;
            let axes = [
                ["ardupilot_pitch_limits_deg", "pitch_range_deg", "tilt_range_deg"],
                ["roll_range_ctrl_deg", "roll_range_deg", ""],
                ["ardupilot_yaw_limits_deg", "yaw_range_deg", "pan_range_deg"],
            ];
            let mut lo = [-90.0, -30.0, -160.0];
            let mut hi = [25.0, 30.0, 160.0];
            let mut stated = false;
            for (i, keys) in axes.iter().enumerate() {
                if let Some((a, b)) = keys.iter().filter(|k| !k.is_empty()).find_map(|k| range(k)) {
                    lo[i] = a;
                    hi[i] = b;
                    stated = true;
                }
            }
            if stated {
                out.insert("angleMin".into(), Toml::Array(lo.iter().map(|x| Toml::Float(x * deg)).collect()));
                out.insert("angleMax".into(), Toml::Array(hi.iter().map(|x| Toml::Float(x * deg)).collect()));
                how.push(("angleMin, angleMax".into(), "the stated tilt, roll and pan ranges".into()));
            }
            if let Some(v) = f("max_rate_dps") {
                out.insert("maxRate".into(), Toml::Array(vec![Toml::Float(v * deg); 3]));
                how.push(("maxRate".into(), format!("{v} °/s on every axis")));
            } else if let Some(t) = part.get("max_rate_dps").and_then(Toml::as_table) {
                let r = |k: &str| t.get(k).and_then(|v| v.as_float().or(v.as_integer().map(|n| n as f64)));
                if let (Some(tilt), Some(roll), Some(pan)) = (r("tilt"), r("roll"), r("pan")) {
                    out.insert("maxRate".into(), Toml::Array([tilt, roll, pan].iter().map(|x| Toml::Float(x * deg)).collect()));
                    how.push(("maxRate".into(), format!("tilt {tilt}, roll {roll}, pan {pan} °/s")));
                }
            }
        }
        "tether" => {
            if let Some(v) = f("max_tether_length_m") {
                put(&mut out, "maxLength", v, "tether length".into());
            }
            if let Some(v) = f("max_power_w").or(f("continuous_power_w")) {
                put(&mut out, "maxPower", v, "continuous power".into());
            }
            if let Some(g) = f("tether_mass_per_m_g") {
                put(&mut out, "massPerLength", g / 1000.0, format!("{g} g/m"));
            } else {
                assumed.push("massPerLength 0.02 kg/m (not stated)".into());
            }
        }
        "generator" => {
            if let Some(v) = f("rated_power_w").or(f("continuous_power_w")) {
                put(&mut out, "ratedPower", v, "rated power".into());
            }
            let top = part.get("fuel_consumption_l_per_h").and_then(Toml::as_array)
                .and_then(|a| a.last()).and_then(|v| v.as_float().or(v.as_integer().map(|n| n as f64)));
            if let Some(v) = f("fuel_consumption_lph").or(f("fuel_consumption_l_per_h")).or(top) {
                put(&mut out, "fuelRate", v, format!("{v} l/h at rated power"));
            } else {
                assumed.push("fuelRate 1.5 l/h at rated power (not stated)".into());
            }
        }
        _ => {}
    }
    (out, how, assumed)
}

/// Replace `part` in `[payloads.KIND]` by the part's parameters (explicit keys win).
pub fn expand(root: &mut Table, reports: &mut Vec<Json>) -> Result<()> {
    let Some(payloads) = root.get("payloads").and_then(Toml::as_table).cloned() else {
        return Ok(());
    };
    let mut out = payloads.clone();
    for (name, sel) in &payloads {
        let Some(mut sel) = sel.as_table().cloned() else { continue };
        let Some(Toml::String(wanted)) = sel.remove("part") else { continue };
        let (id, part) = crate::parts::find(name, &wanted)?;
        let (params, how, assumed) = convert(name, &part);
        let mut parameters = serde_json::Map::new();
        for (k, v) in &params {
            if sel.contains_key(k) {
                parameters.insert(k.clone(), json!({"value": "explicit in the configuration", "how": "", "from": []}));
                continue;
            }
            let why = how.iter().find(|(h, _)| h == k || h.starts_with(k)).map(|h| h.1.clone()).unwrap_or_default();
            parameters.insert(k.clone(), json!({"value": v, "how": why, "from": []}));
            sel.insert(k.clone(), v.clone());
        }
        reports.push(json!({
            "kind": name, "part": id,
            "name": part.get("name").and_then(Toml::as_str).unwrap_or(&id),
            "source": part.get("source").and_then(Toml::as_str).unwrap_or(""),
            "level": "physical", "model": kind(name).map(|k| k.class).unwrap_or(""),
            "parameters": parameters, "assumed": assumed,
            "not_represented": {},
            "not_stated": part.get("not_stated").and_then(Toml::as_array).map(|a| a.iter()
                .filter_map(Toml::as_str).map(|f| (f.to_string(), json!("not stated"))).collect::<serde_json::Map<_, _>>())
                .unwrap_or_default(),
        }));
        out.insert(name.clone(), Toml::Table(sel));
    }
    root.insert("payloads".into(), Toml::Table(out));
    Ok(())
}

/// The report of one payload part: `fire-compose part KIND PART`.
pub fn report(kind_name: &str, part: &str, explicit: &Table) -> Result<Json> {
    let mut sel = explicit.clone();
    sel.insert("part".into(), Toml::String(part.into()));
    let mut root = Table::new();
    root.insert(
        "payloads".into(),
        Toml::Table(Table::from_iter([(kind_name.to_string(), Toml::Table(sel))])),
    );
    let mut reports = Vec::new();
    expand(&mut root, &mut reports)?;
    reports.pop().ok_or_else(|| format!("{kind_name}: not a payload kind"))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn quad(extra: &str) -> std::result::Result<ResolvedFireConfig, String> {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../configs/quad.toml");
        crate::config::parse_at(&(std::fs::read_to_string(root).unwrap() + extra), None)
    }

    #[test]
    fn servos_take_aux_channels_in_kind_order() {
        let c = quad("\n[payloads.gimbal]\n[payloads.gripper]\n[payloads.tether]\n[peripherals.power_module]\n").unwrap();
        let ch: Vec<_> = channels(&c.payloads).iter().map(|(k, n)| (k.name, *n)).collect();
        assert_eq!(ch, vec![("gripper", 1), ("gimbal", 2)]);
        assert!(quad("\n[payloads.generator]\n").is_err(), "a generator needs a power module");
        assert!(quad("\n[payloads.gripper]\nmass = 0.5\nposition = [0, 0, 0.05]\n").is_ok());
        assert!(quad("\n[payloads.gripper]\nlength = 2\n").is_err());
    }
}
