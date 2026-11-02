//! Domain validation and default resolution; validation has no filesystem effects.
use crate::{identifier, Result};
use serde::Serialize;
use std::collections::BTreeSet;
use toml::{map::Map, Value};

pub(crate) type Table = Map<String, Value>;
const IDENTITY: [[f64; 3]; 3] = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]];

#[derive(Clone, Debug, Serialize)]
pub struct ResolvedFireConfig {
    pub schema_version: i64,
    pub acquisition: String,
    pub model_name: String,
    pub preset: String,
    pub geometry: Table,
    pub mass: Table,
    pub motor: Table,
    #[serde(skip_serializing_if = "Table::is_empty")]
    pub esc: Table,
    pub rotor: Table,
    #[serde(skip_serializing_if = "Table::is_empty")]
    pub power: Table,
    pub landing_gear: Table,
    pub initial: Table,
    #[serde(skip_serializing_if = "Table::is_empty")]
    pub models: Table,
    #[serde(skip_serializing_if = "Table::is_empty")]
    pub interface: Table,
    #[serde(skip_serializing_if = "Table::is_empty")]
    pub imu: Table,
    #[serde(skip_serializing_if = "Table::is_empty")]
    pub acoustic: Table,
}

impl ResolvedFireConfig {
    pub fn voltage_driven(&self) -> bool {
        self.models.get("motor").and_then(Value::as_str) != Some("first_order_speed")
    }

    pub fn electromechanical(&self) -> bool {
        matches!(
            self.models.get("motor").and_then(Value::as_str),
            Some("dynamic_dc_equivalent" | "three_phase_pm")
        )
    }

    pub fn three_phase(&self) -> bool {
        self.models.get("motor").and_then(Value::as_str) == Some("three_phase_pm")
    }

    pub fn switching(&self) -> bool {
        self.models.get("esc").and_then(Value::as_str) == Some("switching_three_phase")
    }

    pub fn propulsion_level(&self) -> &'static str {
        if self.switching() {
            "P4"
        } else if self.three_phase() {
            "P3"
        } else if self.electromechanical() {
            "P2"
        } else if self.voltage_driven() {
            "P1"
        } else {
            "P0"
        }
    }

    pub fn esc_class(&self) -> &'static str {
        match self.models["esc"].as_str().unwrap() {
            "switching_three_phase" => {
                "ModelicaCPSLibrary.Physical.Electrical.Drives.ESC.SwitchingThreePhase"
            }
            "average_three_phase" => {
                "ModelicaCPSLibrary.Physical.Electrical.Drives.ESC.AverageThreePhase"
            }
            "average_unidirectional" => {
                "ModelicaCPSLibrary.Physical.Electrical.Drives.ESC.AverageUnidirectional"
            }
            "average_voltage" => "ModelicaCPSLibrary.Physical.Electrical.Drives.ESC.AverageVoltage",
            _ => "ModelicaCPSLibrary.Physical.Electrical.Drives.ESC.PassThrough",
        }
    }

    pub fn motor_class(&self) -> &'static str {
        match self.models["motor"].as_str().unwrap() {
            "three_phase_pm" => "ModelicaCPSLibrary.Systems.Propulsion.Motors.ThreePhasePM",
            "dynamic_dc_equivalent" => {
                "ModelicaCPSLibrary.Systems.Propulsion.Motors.DynamicDCEquivalent"
            }
            "first_order_voltage_to_speed" => {
                "ModelicaCPSLibrary.Systems.Propulsion.Motors.FirstOrderVoltageToSpeed"
            }
            _ => "ModelicaCPSLibrary.Systems.Propulsion.Motors.FirstOrderSpeed",
        }
    }
}

pub(crate) fn table<'a>(v: &'a Value, allowed: &[&str], label: &str) -> Result<&'a Table> {
    let t = v
        .as_table()
        .ok_or_else(|| format!("{label}: expected a table"))?;
    for k in t.keys() {
        if !allowed.contains(&k.as_str()) {
            return Err(format!("{label}: unknown fields {k}"));
        }
    }
    Ok(t)
}
fn section(t: &Table, key: &str) -> Value {
    t.get(key)
        .cloned()
        .unwrap_or_else(|| Value::Table(Table::new()))
}
pub(crate) fn number(v: &Value, label: &str, positive: bool, nonnegative: bool) -> Result<f64> {
    let x = v
        .as_float()
        .or_else(|| v.as_integer().map(|n| n as f64))
        .ok_or_else(|| format!("{label}: expected a finite number"))?;
    if !x.is_finite() || (positive && x <= 0.) || (nonnegative && x < 0.) {
        return Err(format!(
            "{label}: expected a finite {}number",
            if positive {
                "positive "
            } else if nonnegative {
                "nonnegative "
            } else {
                ""
            }
        ));
    }
    Ok(x)
}
pub(crate) fn integer(v: &Value, label: &str, min: i64) -> Result<usize> {
    let n = v
        .as_integer()
        .ok_or_else(|| format!("{label}: expected integer >= {min}"))?;
    if n < min {
        return Err(format!("{label}: expected integer >= {min}"));
    }
    usize::try_from(n).map_err(|_| format!("{label}: integer is too large"))
}
pub(crate) fn array(v: &Value, dims: &[usize], label: &str) -> Result<()> {
    if dims.is_empty() {
        number(v, label, false, false)?;
        return Ok(());
    }
    let a = v
        .as_array()
        .filter(|a| a.len() == dims[0])
        .ok_or_else(|| format!("{label}: expected dimensions {dims:?}"))?;
    for x in a {
        array(x, &dims[1..], label)?;
    }
    Ok(())
}
fn matrix(v: &Value, label: &str) -> Result<[[f64; 3]; 3]> {
    array(v, &[3, 3], label)?;
    let mut a = [[0.; 3]; 3];
    for (i, row) in a.iter_mut().enumerate() {
        for (j, x) in row.iter_mut().enumerate() {
            *x = number(&v[i][j], label, false, false)?;
        }
    }
    Ok(a)
}
fn det(a: &[[f64; 3]; 3]) -> f64 {
    a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])
}
fn rotation(v: &Value, label: &str) -> Result<()> {
    let a = matrix(v, label)?;
    let mut error: f64 = 0.;
    for i in 0..3 {
        for j in 0..3 {
            error = error.max(
                ((0..3).map(|k| a[k][i] * a[k][j]).sum::<f64>() - if i == j { 1. } else { 0. })
                    .abs(),
            );
        }
    }
    if error > 1e-9 || (det(&a) - 1.).abs() > 1e-9 {
        return Err(format!(
            "{label}: expected an orthonormal rotation with determinant +1"
        ));
    }
    Ok(())
}
fn inertia(v: &Value, label: &str) -> Result<()> {
    let a = matrix(v, label)?;
    let scale = a.iter().flatten().fold(0f64, |s, x| s.max(x.abs()));
    for (i, row) in a.iter().enumerate() {
        for (j, value) in row.iter().enumerate() {
            if (value - a[j][i]).abs() > 1e-12 * scale.max(1e-30) {
                return Err(format!("{label}: inertia must be symmetric"));
            }
        }
    }
    let b = a.map(|r| r.map(|x| x / scale));
    if scale == 0. || b[0][0] <= 0. || b[0][0] * b[1][1] - b[0][1] * b[1][0] <= 0. || det(&b) <= 0.
    {
        return Err(format!("{label}: inertia must be positive definite"));
    }
    let half_trace = (b[0][0] + b[1][1] + b[2][2]) / 2.;
    let mut second = [[0.; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            second[i][j] = (if i == j { half_trace } else { 0. }) - b[i][j];
        }
    }
    let mut minors = vec![det(&second)];
    for i in 0..3 {
        minors.push(second[i][i]);
        for j in i + 1..3 {
            minors.push(second[i][i] * second[j][j] - second[i][j] * second[j][i]);
        }
    }
    if minors.iter().any(|x| *x < -1e-10) {
        return Err(format!(
            "{label}: inertia must represent a physical mass distribution"
        ));
    }
    Ok(())
}
fn component_id(v: &Value) -> Result<()> {
    let s = v.as_str().ok_or("componentId: expected string")?;
    if s.len() > 128
        || !s
            .bytes()
            .all(|c| c.is_ascii_alphanumeric() || b"_.:/-".contains(&c))
    {
        return Err("componentId: invalid characters or length".into());
    }
    Ok(())
}
fn vec3(v: [f64; 3]) -> Value {
    Value::Array(v.into_iter().map(Value::Float).collect())
}
fn mat(a: [[f64; 3]; 3]) -> Value {
    Value::Array(a.into_iter().map(vec3).collect())
}
fn int(n: usize) -> Value {
    Value::Integer(n as i64)
}
fn part(v: &Value, label: &str, position: [f64; 3]) -> Result<Value> {
    let mut p = table(v, &["mass", "inertia", "r_C", "R_Cj", "componentId"], label)?.clone();
    for k in ["mass", "inertia"] {
        if !p.contains_key(k) {
            return Err(format!("{label}: mass and inertia are required"));
        }
    }
    number(&p["mass"], label, true, false)?;
    inertia(&p["inertia"], label)?;
    p.entry("r_C").or_insert_with(|| vec3(position));
    p.entry("R_Cj").or_insert_with(|| mat(IDENTITY));
    p.entry("componentId")
        .or_insert_with(|| Value::String(String::new()));
    array(&p["r_C"], &[3], label)?;
    rotation(&p["R_Cj"], label)?;
    component_id(&p["componentId"])?;
    Ok(Value::Table(p))
}

fn geometry(raw: &Value) -> Result<(String, Table)> {
    let g = table(
        raw,
        &[
            "preset",
            "nActuators",
            "actuatorIndex",
            "armMount",
            "rotorArmIndex",
            "rotorPosition_C",
            "R_br",
            "spinSign",
        ],
        "geometry",
    )?;
    let preset = g.get("preset").and_then(Value::as_str).unwrap_or("QuadX");
    if g.get("preset").is_some_and(|v| !v.is_str()) {
        return Err("geometry.preset: expected string".into());
    }
    let (na, nr, radius) = match preset {
        "QuadX" => (4, 4, 0.25),
        "HexaX" => (6, 6, 0.30),
        "OctoX" => (8, 8, 0.35),
        "CoaxialX8" => (4, 8, 0.25),
        _ => return Err("geometry.preset must be QuadX, HexaX, OctoX, or CoaxialX8".into()),
    };
    let nc = integer(
        g.get("nActuators").unwrap_or(&int(nr)),
        "geometry.nActuators",
        1,
    )?;
    // Bound source expansion; this is a tooling limit, not a physical model limit.
    if nc > 4096 {
        return Err("geometry: at most 4096 actuator channels are supported".into());
    }
    let polygon = |n: usize, r: f64, z: f64, phase: f64| {
        Value::Array(
            (0..n)
                .map(|i| {
                    let theta = 2. * std::f64::consts::PI * i as f64 / n as f64 + phase;
                    vec3([r * theta.cos(), -r * theta.sin(), z])
                })
                .collect(),
        )
    };
    let mounts = g
        .get("armMount")
        .cloned()
        .unwrap_or_else(|| polygon(na, radius, 0., std::f64::consts::PI / na as f64));
    array(&mounts, &[na, 3], "geometry.armMount")?;
    let arms = g
        .get("rotorArmIndex")
        .cloned()
        .unwrap_or_else(|| Value::Array((0..nr).map(|i| int(1 + i % na)).collect()));
    let channels = g
        .get("actuatorIndex")
        .cloned()
        .unwrap_or_else(|| Value::Array((1..=nr).map(int).collect()));
    for (key, values, max) in [
        ("rotorArmIndex", &arms, na),
        ("actuatorIndex", &channels, nc),
    ] {
        array(values, &[nr], key)?;
        for v in values.as_array().unwrap() {
            if integer(v, key, 1)? > max {
                return Err(format!("geometry.{key}: index exceeds {max}"));
            }
        }
    }
    let positions = g.get("rotorPosition_C").cloned().unwrap_or_else(|| {
        Value::Array(
            (0..nr)
                .map(|i| {
                    let p = &mounts[arms[i].as_integer().unwrap() as usize - 1];
                    let dz = if preset == "CoaxialX8" {
                        if i < 4 {
                            -0.025
                        } else {
                            0.025
                        }
                    } else {
                        0.
                    };
                    vec3([
                        number(&p[0], "mount", false, false).unwrap(),
                        number(&p[1], "mount", false, false).unwrap(),
                        number(&p[2], "mount", false, false).unwrap() + dz,
                    ])
                })
                .collect(),
        )
    });
    let spins = g.get("spinSign").cloned().unwrap_or_else(|| {
        Value::Array(
            (0..nr)
                .map(|i| {
                    Value::Integer(
                        (if i % 2 == 0 { 1 } else { -1 })
                            * if preset == "CoaxialX8" && i >= 4 {
                                -1
                            } else {
                                1
                            },
                    )
                })
                .collect(),
        )
    });
    array(&spins, &[nr], "geometry.spinSign")?;
    if spins
        .as_array()
        .unwrap()
        .iter()
        .any(|x| !matches!(x.as_integer(), Some(1 | -1)))
    {
        return Err("geometry.spinSign entries must be integers +1 or -1".into());
    }
    let rotations = g.get("R_br").cloned().unwrap_or_else(|| {
        Value::Array(vec![mat([[1., 0., 0.], [0., -1., 0.], [0., 0., -1.]]); nr])
    });
    array(&rotations, &[nr, 3, 3], "geometry.R_br")?;
    for r in rotations.as_array().unwrap() {
        rotation(r, "geometry.R_br")?;
    }
    array(&positions, &[nr, 3], "geometry.rotorPosition_C")?;
    let t = [
        ("nArms", int(na)),
        ("nRotors", int(nr)),
        ("nActuators", int(nc)),
        ("armMount", mounts),
        ("rotorArmIndex", arms),
        ("actuatorIndex", channels),
        ("rotorPosition_C", positions),
        ("R_br", rotations),
        ("spinSign", spins),
    ]
    .into_iter()
    .map(|(k, v)| (k.into(), v))
    .collect();
    Ok((preset.into(), t))
}

fn mass(raw: &Value, g: &Table) -> Result<Table> {
    let m = table(
        raw,
        &[
            "mode",
            "aggregate",
            "core",
            "arm",
            "payloads",
            "additional_parts",
        ],
        "mass",
    )?;
    let mode = m.get("mode").and_then(Value::as_str).unwrap_or("aggregate");
    if m.get("mode").is_some_and(|v| !v.is_str()) {
        return Err("mass.mode: expected string".into());
    }
    let mut out = Table::new();
    out.insert("mode".into(), Value::String(mode.into()));
    if mode == "aggregate" {
        if m.keys()
            .any(|k| !["mode", "aggregate"].contains(&k.as_str()))
        {
            return Err("mass: constituent fields are not permitted in aggregate mode".into());
        }
        let default = Value::Table(
            [
                ("mass".into(), Value::Float(1.5)),
                (
                    "inertia".into(),
                    mat([[0.02, 0., 0.], [0., 0.02, 0.], [0., 0., 0.04]]),
                ),
            ]
            .into_iter()
            .collect(),
        );
        out.insert(
            "aggregate".into(),
            part(
                m.get("aggregate").unwrap_or(&default),
                "mass.aggregate",
                [0.; 3],
            )?,
        );
    } else if mode == "assembled" {
        if m.contains_key("aggregate") || !m.contains_key("core") || !m.contains_key("arm") {
            return Err("mass: assembled mode requires core and arm, and forbids aggregate".into());
        }
        out.insert("core".into(), part(&m["core"], "mass.core", [0.; 3])?);
        let mut arm = table(
            &m["arm"],
            &["mass", "inertia", "R_Cj", "componentIds"],
            "mass.arm",
        )?
        .clone();
        let na = g["nArms"].as_integer().unwrap() as usize;
        let ids = arm.remove("componentIds").unwrap_or_else(|| {
            Value::Array(
                (1..=na)
                    .map(|i| Value::String(format!("arm.{i}")))
                    .collect(),
            )
        });
        let ids = ids
            .as_array()
            .filter(|v| v.len() == na)
            .ok_or_else(|| format!("mass.arm.componentIds: expected {na} entries"))?;
        let mut arms = Vec::new();
        for (i, id) in ids.iter().enumerate() {
            arm.insert("componentId".into(), id.clone());
            let mut p = [0.; 3];
            for (j, x) in p.iter_mut().enumerate() {
                *x = number(&g["armMount"][i][j], "mount", false, false)? / 2.;
            }
            arms.push(part(&Value::Table(arm.clone()), "mass.arm", p)?);
        }
        out.insert("arms".into(), Value::Array(arms));
        for key in ["payloads", "additional_parts"] {
            let mut parts = Vec::new();
            if let Some(v) = m.get(key) {
                for p in v
                    .as_array()
                    .ok_or_else(|| format!("mass.{key}: expected array of tables"))?
                {
                    parts.push(part(p, key, [0.; 3])?);
                }
            }
            out.insert(key.into(), Value::Array(parts));
        }
        let mut ids = BTreeSet::new();
        let parts = std::iter::once(&out["core"]).chain(
            ["arms", "payloads", "additional_parts"]
                .into_iter()
                .flat_map(|k| out[k].as_array().unwrap()),
        );
        for p in parts {
            let id = p["componentId"].as_str().unwrap();
            if !id.is_empty() && !ids.insert(id) {
                return Err("mass: duplicate nonempty componentId".into());
            }
        }
    } else {
        return Err("mass.mode must be aggregate or assembled".into());
    }
    Ok(out)
}

pub fn parse(source: &str) -> Result<ResolvedFireConfig> {
    parse_at(source, None)
}

/// Select a TOML table before applying the standalone physics schema.
/// The selector uses TOML dotted-key syntax, including quoted key segments.
pub fn parse_at(source: &str, path: Option<&str>) -> Result<ResolvedFireConfig> {
    let mut raw: Value = toml::from_str(source).map_err(|e| e.to_string())?;
    if let Some(path) = path {
        let selector: Value = toml::from_str(&format!("[{path}]"))
            .map_err(|e| format!("invalid config table path {path:?}: {e}"))?;
        let mut cursor = &selector;
        let mut selected = &raw;
        loop {
            let keys = cursor
                .as_table()
                .ok_or("config table path must name a table")?;
            if keys.is_empty() {
                break;
            }
            if keys.len() != 1 {
                return Err("config table path must name exactly one table".into());
            }
            let (key, child) = keys.iter().next().unwrap();
            selected = selected
                .get(key)
                .ok_or_else(|| format!("config table {path:?} not found"))?;
            cursor = child;
        }
        if !selected.is_table() {
            return Err(format!("config table {path:?} must be a table"));
        }
        raw = selected.clone();
    }
    let root = raw.as_table().ok_or("configuration must be a table")?;
    let version = root
        .get("schema_version")
        .and_then(Value::as_integer)
        .ok_or(
            "schema_version must be 3 (NED/FRD continuous); migrate legacy ENU/FLU data explicitly",
        )?;
    let allowed = vec![
        "schema_version",
        "model_name",
        "geometry",
        "mass",
        "motor",
        "esc",
        "rotor",
        "power",
        "landing_gear",
        "initial",
        "models",
        "interface",
        "acquisition",
        "imu",
        "vulnerabilities",
    ];
    if version != 3 {
        return Err(
            "schema_version must be 3 (NED/FRD continuous); migrate legacy ENU/FLU data explicitly"
                .into(),
        );
    }
    table(&raw, &allowed, "config")?;
    let acquisition = root
        .get("acquisition")
        .and_then(Value::as_str)
        .unwrap_or("continuous");
    if acquisition != "continuous"
        || root
            .get("acquisition")
            .is_some_and(|v| v.as_str().is_none())
    {
        return Err("global acquisition must remain continuous; select independent IMU acquisition via models.imu.model".into());
    }
    let name = root
        .get("model_name")
        .cloned()
        .unwrap_or(Value::String("ConfiguredCopter".into()));
    let name = name.as_str().ok_or("model_name must be an identifier")?;
    if !identifier(name) || !name.starts_with(|c: char| c.is_ascii_uppercase()) {
        return Err("model_name must be an identifier starting with an uppercase letter".into());
    }
    let (preset, g) = geometry(&section(root, "geometry"))?;
    let m = mass(&section(root, "mass"), &g)?;
    let nr = g["nRotors"].as_integer().unwrap() as usize;
    let models = resolve_models(&section(root, "models"))?;
    let motor_model = models["motor"].as_str().unwrap();
    let voltage_driven = motor_model != "first_order_speed";
    let p2 = motor_model == "dynamic_dc_equivalent";
    let three_phase = motor_model == "three_phase_pm";
    let switching = models["esc"].as_str() == Some("switching_three_phase");
    let motor = if three_phase {
        resolve_three_phase_motor(&section(root, "motor"), nr)?
    } else if p2 {
        resolve_p2_motor(&section(root, "motor"), nr)?
    } else {
        resolve_motor(&section(root, "motor"), nr, voltage_driven)?
    };
    let esc = if three_phase {
        resolve_three_phase_esc(&section(root, "esc"), nr, switching)?
    } else {
        if root.contains_key("esc") {
            return Err("esc parameter table requires a three-phase ESC".into());
        }
        Table::new()
    };
    let rotor = resolve_rotor(&section(root, "rotor"), nr, p2 || three_phase)?;
    let power = if voltage_driven {
        let raw = section(root, "power");
        let keys: &[&str] = if three_phase {
            &["dcVoltage", "sourceResistance", "dcLinkCapacitance"]
        } else {
            &["dcVoltage"]
        };
        let t = table(&raw, keys, "power")?;
        if three_phase {
            for key in ["sourceResistance", "dcLinkCapacitance"] {
                let value = t
                    .get(key)
                    .ok_or_else(|| format!("P3/P4 require explicit power.{key}"))?;
                number(value, &format!("power.{key}"), true, false)?;
            }
        }
        let voltage = t
            .get("dcVoltage")
            .ok_or("Voltage-driven propulsion requires power.dcVoltage [V]")?;
        number(voltage, "power.dcVoltage", false, true)?;
        t.clone()
    } else {
        if root.contains_key("power") {
            return Err("power requires a compatible voltage-driven ESC/motor combination".into());
        }
        Table::new()
    };
    let landing_gear = resolve_landing_gear(&section(root, "landing_gear"))?;
    let initial = resolve_initial(&section(root, "initial"))?;
    let interface = resolve_interface(&section(root, "interface"))?;
    let imu = crate::imu::resolve(&section(root, "imu"), &models)?;
    let acoustic = crate::acoustic::resolve(&section(root, "vulnerabilities"), &models)?;
    Ok(ResolvedFireConfig {
        schema_version: version,
        acquisition: acquisition.into(),
        model_name: name.into(),
        preset,
        geometry: g,
        mass: m,
        motor,
        esc,
        rotor,
        power,
        landing_gear,
        initial,
        models,
        interface,
        imu,
        acoustic,
    })
}

fn resolve_motor(raw: &Value, n: usize, voltage_driven: bool) -> Result<Table> {
    let fields = if voltage_driven {
        [("Kv", 1000. / 16.), ("tauOmega", 0.03), ("omega_start", 0.)]
    } else {
        [("omegaMax", 1000.), ("tau", 0.03), ("omega_start", 0.)]
    };
    let t = table(raw, &fields.map(|(k, _)| k), "motor")?;
    if voltage_driven && !t.contains_key("Kv") {
        return Err("P1 requires motor.Kv in (rad/s)/V, not rpm/V".into());
    }
    let mut out = Table::new();
    for (k, default) in fields {
        let v = t.get(k).cloned().unwrap_or(Value::Float(default));
        let v = if v.is_array() {
            v
        } else {
            Value::Array(vec![v; n])
        };
        array(&v, &[n], &format!("motor.{k}"))?;
        for x in v.as_array().unwrap() {
            number(
                x,
                &format!("motor.{k}"),
                ["omegaMax", "tau", "Kv", "tauOmega"].contains(&k),
                true,
            )?;
        }
        out.insert(k.into(), v);
    }
    for i in 0..n {
        if !voltage_driven
            && number(&out["omega_start"][i], "motor", false, false)?
                > number(&out["omegaMax"][i], "motor", false, false)?
        {
            return Err("motor.omega_start must not exceed omegaMax".into());
        }
    }
    Ok(out)
}

fn resolve_p2_motor(raw: &Value, n: usize) -> Result<Table> {
    let keys = ["R", "L", "k", "J", "b", "i_start", "omega_start"];
    let t = table(raw, &keys, "motor")?;
    let mut out = Table::new();
    for key in keys {
        let required = ["R", "L", "k", "J"].contains(&key);
        let v = match t.get(key) {
            Some(v) => v.clone(),
            None if required => return Err(format!("P2 requires explicit motor.{key}")),
            None => Value::Float(0.),
        };
        let v = if v.is_array() {
            v
        } else {
            Value::Array(vec![v; n])
        };
        array(&v, &[n], &format!("motor.{key}"))?;
        for x in v.as_array().unwrap() {
            number(x, &format!("motor.{key}"), required, true)?;
        }
        out.insert(key.into(), v);
    }
    Ok(out)
}

// Scalars expand per rotor; a supplied vector must have exactly n entries.
fn rotor_vector(value: Value, n: usize, label: &str) -> Result<Value> {
    let v = if value.is_array() {
        value
    } else {
        Value::Array(vec![value; n])
    };
    if v.as_array().is_none_or(|a| a.len() != n) {
        return Err(format!("{label}: expected {n} entries"));
    }
    Ok(v)
}

fn resolve_three_phase_motor(raw: &Value, n: usize) -> Result<Table> {
    let keys = [
        "RPhase",
        "LPhase",
        "ke",
        "polePairs",
        "J",
        "b",
        "theta0",
        "omega_start",
        "trapezoidal",
        "i_start",
    ];
    let t = table(raw, &keys, "motor")?;
    let mut out = Table::new();
    for key in keys.into_iter().filter(|k| *k != "i_start") {
        let required = ["RPhase", "LPhase", "ke", "polePairs", "J"].contains(&key);
        let value = match t.get(key) {
            Some(v) => v.clone(),
            None if required => return Err(format!("P3/P4 require explicit motor.{key}")),
            None if key == "trapezoidal" => Value::Boolean(true),
            None => Value::Float(0.),
        };
        let label = format!("motor.{key}");
        let v = rotor_vector(value, n, &label)?;
        for x in v.as_array().unwrap() {
            match key {
                "polePairs" => {
                    integer(x, &label, 1)?;
                }
                "trapezoidal" => {
                    if !x.is_bool() {
                        return Err(format!("{label}: expected boolean"));
                    }
                }
                _ => {
                    number(x, &label, required, key != "theta0")?;
                }
            }
        }
        out.insert(key.into(), v);
    }
    let currents = t
        .get("i_start")
        .cloned()
        .unwrap_or_else(|| Value::Array(vec![vec3([0.; 3]); n]));
    array(&currents, &[n, 3], "motor.i_start")?;
    for row in currents.as_array().unwrap() {
        let sum = row
            .as_array()
            .unwrap()
            .iter()
            .map(|v| number(v, "motor.i_start", false, false).unwrap())
            .sum::<f64>();
        if sum.abs() >= 1e-10 {
            return Err("motor.i_start: each floating-star current triple must sum to zero".into());
        }
    }
    out.insert("i_start".into(), currents);
    Ok(out)
}

fn resolve_three_phase_esc(raw: &Value, n: usize, switching: bool) -> Result<Table> {
    let mut keys = vec!["Ron", "Rdiode", "Vdiode", "Goff", "currentThreshold"];
    if switching {
        keys.extend(["pwmFrequency", "deadTime"]);
    }
    let t = table(raw, &keys, "esc")?;
    let mut out = Table::new();
    for key in keys {
        let required = ["Ron", "Rdiode", "Vdiode", "pwmFrequency"].contains(&key);
        let value = match t.get(key) {
            Some(v) => v.clone(),
            None if required => return Err(format!("P3/P4 require explicit esc.{key}")),
            None => Value::Float(match key {
                "Goff" => 1e-5,
                "currentThreshold" => 1e-3,
                _ => 0.,
            }),
        };
        let label = format!("esc.{key}");
        let v = rotor_vector(value, n, &label)?;
        for x in v.as_array().unwrap() {
            number(x, &label, !["Vdiode", "deadTime"].contains(&key), true)?;
        }
        out.insert(key.into(), v);
    }
    if switching {
        for i in 0..n {
            if number(&out["pwmFrequency"][i], "esc.pwmFrequency", true, false)?
                * number(&out["deadTime"][i], "esc.deadTime", false, true)?
                >= 0.5
            {
                return Err("esc.deadTime must be less than half a PWM period".into());
            }
        }
    }
    Ok(out)
}

fn resolve_rotor(raw: &Value, n: usize, p2: bool) -> Result<Table> {
    let allowed: &[&str] = if p2 {
        &["kT", "kQ", "J"]
    } else {
        &["kT", "kQ"]
    };
    let t = table(raw, allowed, "rotor")?;
    if p2 && !t.contains_key("J") {
        return Err("P2/P3/P4 require explicit rotor.J".into());
    }
    let mut defaults = vec![("kT", 1e-5), ("kQ", 1.5e-7)];
    if p2 {
        defaults.push(("J", 0.));
    }
    let mut out = Table::new();
    for (key, default) in defaults {
        let v = t.get(key).cloned().unwrap_or(Value::Float(default));
        let v = if v.is_array() {
            v
        } else {
            Value::Array(vec![v; n])
        };
        array(&v, &[n], &format!("rotor.{key}"))?;
        for x in v.as_array().unwrap() {
            number(x, &format!("rotor.{key}"), false, true)?;
        }
        out.insert(key.into(), v);
    }
    Ok(out)
}

fn resolve_landing_gear(raw: &Value) -> Result<Table> {
    let t = table(
        raw,
        &[
            "enabled",
            "position_C",
            "groundZ",
            "stiffness",
            "damping",
            "tangentialDamping",
            "frictionCoefficient",
        ],
        "landing_gear",
    )?;
    let enabled = t.get("enabled").cloned().unwrap_or(Value::Boolean(true));
    if !enabled.is_bool() {
        return Err("landing_gear.enabled: expected boolean".into());
    }
    let positions = t.get("position_C").cloned().unwrap_or_else(|| {
        Value::Array(
            [
                [0.17, 0.17, 0.1],
                [-0.17, -0.17, 0.1],
                [0.17, -0.17, 0.1],
                [-0.17, 0.17, 0.1],
            ]
            .into_iter()
            .map(vec3)
            .collect(),
        )
    });
    array(&positions, &[4, 3], "landing_gear.position_C")?;
    let mut out = Table::new();
    out.insert("enabled".into(), enabled);
    out.insert("position_C".into(), positions);
    for (key, default) in [
        ("groundZ", 0.),
        ("stiffness", 3000.),
        ("damping", 150.),
        ("tangentialDamping", 25.),
        ("frictionCoefficient", 0.6),
    ] {
        let value = t.get(key).cloned().unwrap_or(Value::Float(default));
        number(
            &value,
            &format!("landing_gear.{key}"),
            false,
            key != "groundZ",
        )?;
        out.insert(key.into(), value);
    }
    Ok(out)
}

fn resolve_initial(raw: &Value) -> Result<Table> {
    let t = table(
        raw,
        &["p_start", "v_start", "q_start", "omega_start"],
        "initial",
    )?;
    let mut out = Table::new();
    for (k, default) in [
        ("p_start", vec![0., 0., -1.]),
        ("v_start", vec![0.; 3]),
        (
            "q_start",
            vec![
                std::f64::consts::FRAC_1_SQRT_2,
                0.,
                0.,
                std::f64::consts::FRAC_1_SQRT_2,
            ],
        ),
        ("omega_start", vec![0.; 3]),
    ] {
        let n = default.len();
        let v = t
            .get(k)
            .cloned()
            .unwrap_or_else(|| Value::Array(default.into_iter().map(Value::Float).collect()));
        array(&v, &[n], &format!("initial.{k}"))?;
        out.insert(k.into(), v);
    }
    let norm = out["q_start"]
        .as_array()
        .unwrap()
        .iter()
        .try_fold(0f64, |s, x| {
            Ok::<_, String>(s.hypot(number(x, "initial.q_start", false, false)?))
        })?;
    if norm <= 1e-12 {
        return Err("initial.q_start must be nonzero".into());
    }
    Ok(out)
}

fn resolve_models(raw: &Value) -> Result<Table> {
    let t = table(
        raw,
        &[
            "esc",
            "motor",
            "rotor",
            "imu",
            "magnetometer",
            "gnss",
            "barometer",
        ],
        "models",
    )?;
    let mut out = t.clone();
    for (key, default, supported) in [
        (
            "esc",
            "pass_through",
            &[
                "pass_through",
                "average_voltage",
                "average_unidirectional",
                "average_three_phase",
                "switching_three_phase",
            ][..],
        ),
        (
            "motor",
            "first_order_speed",
            &[
                "first_order_speed",
                "first_order_voltage_to_speed",
                "dynamic_dc_equivalent",
                "three_phase_pm",
            ][..],
        ),
        ("rotor", "quadratic_blade", &["quadratic_blade"][..]),
    ] {
        let selected = out
            .entry(key)
            .or_insert_with(|| Value::String(default.into()));
        if !selected.as_str().is_some_and(|s| supported.contains(&s)) {
            return Err(format!(
                "models.{key}: supported models are {}; select ESC, motor and rotor separately",
                supported.join(", ")
            ));
        }
    }
    let pair = (out["esc"].as_str().unwrap(), out["motor"].as_str().unwrap());
    if !matches!(
        pair,
        ("pass_through", "first_order_speed")
            | ("average_voltage", "first_order_voltage_to_speed")
            | ("average_unidirectional", "dynamic_dc_equivalent")
            | (
                "average_three_phase" | "switching_three_phase",
                "three_phase_pm"
            )
    ) {
        return Err("models: incompatible ESC/motor combination; use pass_through/first_order_speed, average_voltage/first_order_voltage_to_speed, average_unidirectional/dynamic_dc_equivalent, or average_three_phase|switching_three_phase/three_phase_pm".into());
    }
    if let Some(raw) = t.get("imu") {
        out.insert("imu".into(), Value::Table(crate::imu::selection(raw)?));
    }
    for key in ["magnetometer", "gnss", "barometer"] {
        if let Some(raw) = t.get(key) {
            let v = table(raw, &["response"], &format!("models.{key}"))?;
            if v.get("response").and_then(Value::as_str) != Some("ideal") {
                return Err(format!(
                    "models.{key}: only ideal response is supported; dynamics are deprecated"
                ));
            }
        }
    }
    Ok(out)
}

fn resolve_interface(raw: &Value) -> Result<Table> {
    let fields = [
        ("pwm_min", 1100.),
        ("pwm_max", 1900.),
        ("lat0", 40.414929),
        ("lon0", -86.932387),
        ("ground_alt_wgs84", 149.),
        ("earth_radius_m", 6378137.),
    ];
    let t = table(raw, &fields.map(|(k, _)| k), "interface")?;
    let mut out = Table::new();
    for (k, d) in fields {
        let v = t.get(k).cloned().unwrap_or(Value::Float(d));
        number(&v, &format!("interface.{k}"), k == "earth_radius_m", false)?;
        out.insert(k.into(), v);
    }
    let get = |k| number(&out[k], k, false, false);
    if get("pwm_max")? <= get("pwm_min")? {
        return Err("interface: pwm_max must exceed pwm_min".into());
    }
    if get("lat0")?.abs() >= 90. || get("lon0")?.abs() > 180. {
        return Err("interface: invalid reference latitude/longitude".into());
    }
    Ok(out)
}
