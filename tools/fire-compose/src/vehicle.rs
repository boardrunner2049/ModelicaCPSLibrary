//! Rover and fixed-wing vehicles: a concrete library export with configured
//! plant and sensor modifiers.
//!
//! The export (Exports.FMU.Rover, Exports.FMU.FixedWing) owns the vehicle
//! interface, the PWM channel meaning and every default value. A configuration
//! names only the values it changes, so an empty configuration composes the
//! export unchanged. Sensor selection uses the same [models.imu]/[imu] tables
//! as a multirotor.
use crate::composition::{CompositionSpec, Modifier};
use crate::config::{
    array, inertia, number, resolve_interface_with, rotation, section, table, ResolvedFireConfig,
    Table,
};
use crate::Result;
use toml::Value as Toml;

#[derive(Clone, Copy, PartialEq)]
enum Shape {
    /// Finite number.
    Real,
    /// Finite positive number.
    Positive,
    /// Finite nonnegative number.
    Nonnegative,
    Vec3,
    Rotation,
    Inertia,
    Quaternion,
}

/// A configurable vehicle kind.
pub struct Kind {
    pub name: &'static str,
    /// Concrete vehicle interface export the composed model extends.
    pub export: &'static str,
    /// Plant class the export instantiates as `plant` (for documentation and tests).
    pub plant: &'static str,
    pub default_model_name: &'static str,
    /// Export defaults of pwm_min/pwm_max, used only to validate a partial [interface].
    pwm: (f64, f64),
    /// [plant] keys: plant and sensor-mount parameters of the plant class.
    parameters: &'static [(&'static str, Shape)],
    /// [plant.RECORD] tables: record parameters set field by field.
    records: &'static [(&'static str, &'static [&'static str])],
    /// [initial] keys and the modifier each sets: (key, target, shape, on the plant).
    initial: &'static [(&'static str, &'static str, Shape, bool)],
}

const MOUNTS: [(&str, Shape); 5] = [
    ("imuPosition_b", Shape::Vec3),
    ("R_bImu", Shape::Rotation),
    ("R_bMag", Shape::Rotation),
    ("gnssPosition_b", Shape::Vec3),
    ("barometerPosition_b", Shape::Vec3),
];

macro_rules! with_mounts {
    ($($item:expr),* $(,)?) => {
        &[$($item,)* MOUNTS[0], MOUNTS[1], MOUNTS[2], MOUNTS[3], MOUNTS[4]]
    };
}

pub const ROVER: Kind = Kind {
    name: "rover",
    export: "ModelicaCPSLibrary.Exports.FMU.Rover",
    plant: "ModelicaCPSLibrary.Vehicles.Rover.RoverWithSensors",
    default_model_name: "ConfiguredRover",
    pwm: (1000., 2000.),
    parameters: with_mounts![
        ("mass", Shape::Positive),
        ("inertia", Shape::Inertia),
        ("wheelbase", Shape::Positive),
        ("track", Shape::Positive),
        ("cgToFrontAxle", Shape::Real),
        ("cgHeight", Shape::Positive),
        ("wheelRadius", Shape::Positive),
        ("suspensionStiffness", Shape::Positive),
        ("suspensionDamping", Shape::Nonnegative),
        ("frictionCoefficient", Shape::Nonnegative),
        ("longitudinalStiffness", Shape::Nonnegative),
        ("corneringStiffness", Shape::Nonnegative),
        ("rollingResistance", Shape::Nonnegative),
        ("maxSteerAngle", Shape::Positive),
        ("steeringTimeConstant", Shape::Positive),
        ("motorStallTorque", Shape::Nonnegative),
        ("motorNoLoadSpeed", Shape::Positive),
        ("wheelInertia", Shape::Positive),
        ("dragArea", Shape::Nonnegative),
        ("airDensity", Shape::Nonnegative),
        ("groundZ", Shape::Real),
    ],
    records: &[],
    initial: &[("q_start", "q_start", Shape::Quaternion, true)],
};

pub const FIXED_WING: Kind = Kind {
    name: "fixed_wing",
    export: "ModelicaCPSLibrary.Exports.FMU.FixedWing",
    plant: "ModelicaCPSLibrary.Vehicles.FixedWing.FixedWingWithSensors",
    default_model_name: "ConfiguredFixedWing",
    pwm: (1000., 2000.),
    parameters: with_mounts![
        ("mass", Shape::Positive),
        ("inertia", Shape::Inertia),
        ("aeroReference_b", Shape::Vec3),
        ("airDensity", Shape::Nonnegative),
        ("maxAileron", Shape::Positive),
        ("maxElevator", Shape::Positive),
        ("maxRudder", Shape::Positive),
        ("servoTimeConstant", Shape::Positive),
        ("maxThrust", Shape::Nonnegative),
        ("propellerPitchSpeed", Shape::Positive),
        ("motorTimeConstant", Shape::Positive),
        ("thrustPosition_b", Shape::Vec3),
        ("cgHeight", Shape::Positive),
        ("wheelRadius", Shape::Positive),
        ("noseWheelX", Shape::Real),
        ("mainWheelX", Shape::Real),
        ("mainWheelTrack", Shape::Positive),
        ("maxNoseWheelSteer", Shape::Nonnegative),
        ("gearStiffness", Shape::Positive),
        ("gearDamping", Shape::Nonnegative),
        ("frictionCoefficient", Shape::Nonnegative),
        ("corneringStiffness", Shape::Nonnegative),
        ("rollingResistance", Shape::Nonnegative),
        ("groundZ", Shape::Real),
    ],
    records: &[(
        "aero",
        &[
            "S",
            "b",
            "c",
            "CL0",
            "CLalpha",
            "CLq",
            "CLelevator",
            "stallBlend",
            "alphaStall",
            "oswald",
            "CDp",
            "CDq",
            "CDelevator",
            "CY0",
            "CYbeta",
            "CYp",
            "CYr",
            "CYaileron",
            "CYrudder",
            "Cl0",
            "Clbeta",
            "Clp",
            "Clr",
            "Claileron",
            "Clrudder",
            "Cm0",
            "Cmalpha",
            "Cmq",
            "Cmelevator",
            "Cn0",
            "Cnbeta",
            "Cnp",
            "Cnr",
            "Cnaileron",
            "Cnrudder",
        ],
    )],
    initial: &[
        ("altitude", "start_altitude_m", Shape::Nonnegative, false),
        ("speed", "start_speed_mps", Shape::Nonnegative, false),
        ("q_start", "q_start", Shape::Quaternion, true),
    ],
};

pub const KINDS: [&Kind; 2] = [&ROVER, &FIXED_WING];

pub fn kind(name: &str) -> Result<&'static Kind> {
    KINDS
        .into_iter()
        .find(|k| k.name == name)
        .ok_or_else(|| format!("vehicle must be multirotor, rover or fixed_wing, not {name:?}"))
}

fn check(value: &Toml, shape: Shape, label: &str) -> Result<()> {
    match shape {
        Shape::Real => number(value, label, false, false).map(drop),
        Shape::Positive => number(value, label, true, false).map(drop),
        Shape::Nonnegative => number(value, label, false, true).map(drop),
        Shape::Vec3 => array(value, &[3], label),
        Shape::Rotation => rotation(value, label),
        Shape::Inertia => inertia(value, label),
        Shape::Quaternion => {
            array(value, &[4], label)?;
            let norm = value.as_array().unwrap().iter().try_fold(0f64, |s, x| {
                Ok::<_, String>(s.hypot(number(x, label, false, false)?))
            })?;
            if norm <= 1e-12 {
                return Err(format!("{label} must be nonzero"));
            }
            Ok(())
        }
    }
}

fn resolve_plant(raw: &Toml, kind: &Kind) -> Result<Table> {
    let mut allowed: Vec<&str> = kind.parameters.iter().map(|(k, _)| *k).collect();
    allowed.extend(kind.records.iter().map(|(k, _)| *k));
    let t = table(raw, &allowed, "plant")?;
    for (key, value) in t {
        let label = format!("plant.{key}");
        if let Some((_, fields)) = kind.records.iter().find(|(k, _)| k == key) {
            for (field, x) in table(value, fields, &label)? {
                number(x, &format!("{label}.{field}"), false, false)?;
            }
        } else {
            let (_, shape) = kind.parameters.iter().find(|(k, _)| k == key).unwrap();
            check(value, *shape, &label)?;
        }
    }
    Ok(t.clone())
}

fn resolve_initial(raw: &Toml, kind: &Kind) -> Result<Table> {
    let keys: Vec<&str> = kind.initial.iter().map(|(k, ..)| *k).collect();
    let t = table(raw, &keys, "initial")?;
    for (key, value) in t {
        let (_, _, shape, _) = kind.initial.iter().find(|(k, ..)| k == key).unwrap();
        check(value, *shape, &format!("initial.{key}"))?;
    }
    Ok(t.clone())
}

fn resolve_models(raw: &Toml, kind: &Kind) -> Result<Table> {
    let t = table(raw, &["imu", "magnetometer", "gnss", "barometer"], "models").map_err(|e| {
        format!(
            "{e} (a {} has no esc/motor/rotor selection; its drive is part of the plant, set in [plant])",
            kind.name
        )
    })?;
    let mut out = t.clone();
    if let Some(raw) = t.get("imu") {
        out.insert("imu".into(), Toml::Table(crate::imu::selection(raw)?));
    }
    for key in ["magnetometer", "gnss", "barometer"] {
        if let Some(raw) = t.get(key) {
            crate::config::ideal_sensor(raw, key)?;
        }
    }
    Ok(out)
}

/// Resolve a rover or fixed-wing configuration (schema_version 3, already checked).
pub fn parse(root: &Table, vehicle: &str) -> Result<ResolvedFireConfig> {
    let kind = kind(vehicle)?;
    table(
        &Toml::Table(root.clone()),
        &[
            "schema_version",
            "model_name",
            "vehicle",
            "acquisition",
            "plant",
            "initial",
            "interface",
            "models",
            "imu",
            "vulnerabilities",
        ],
        "config",
    )
    .map_err(|e| {
        format!(
            "{e} (geometry, mass, motor, esc, rotor, power and landing_gear are multirotor tables)"
        )
    })?;
    let models = resolve_models(&section(root, "models"), kind)?;
    Ok(ResolvedFireConfig {
        schema_version: 3,
        acquisition: crate::config::acquisition(root)?,
        model_name: crate::config::model_name(root, kind.default_model_name)?,
        vehicle: kind.name.into(),
        preset: String::new(),
        geometry: Table::new(),
        mass: Table::new(),
        motor: Table::new(),
        esc: Table::new(),
        rotor: Table::new(),
        power: Table::new(),
        landing_gear: Table::new(),
        initial: resolve_initial(&section(root, "initial"), kind)?,
        interface: resolve_interface_with(
            &section(root, "interface"),
            kind.pwm.0,
            kind.pwm.1,
            false,
        )?,
        imu: crate::imu::resolve(&section(root, "imu"), &models)?,
        acoustic: crate::acoustic::resolve(&section(root, "vulnerabilities"), &models)?,
        models,
        plant: resolve_plant(&section(root, "plant"), kind)?,
    })
}

/// Compose `model NAME extends EXPORT(interface, initial, plant(..., sensors(...)))`
/// plus the selected sensors' boundary variables.
pub fn compose(c: &ResolvedFireConfig, profile: &str) -> Result<CompositionSpec> {
    let kind = kind(&c.vehicle)?;
    if crate::canonical_profile(profile)? != "vio" {
        return Err(format!(
            "a {} composes only the vio profile: it extends the vehicle interface export {}",
            kind.name, kind.export
        ));
    }
    let literal = crate::lower::literal;
    let mut modifiers: Vec<Modifier> = c
        .interface
        .iter()
        .map(|(k, v)| Ok(Modifier::Bind(k.clone(), literal(v)?)))
        .collect::<Result<_>>()?;
    let mut plant = Vec::new();
    for (key, value) in &c.plant {
        plant.push(match value {
            Toml::Table(fields) => Modifier::Nested(
                key.clone(),
                fields
                    .iter()
                    .map(|(k, v)| Ok(Modifier::Bind(k.clone(), literal(v)?)))
                    .collect::<Result<_>>()?,
            ),
            _ => Modifier::Bind(key.clone(), literal(value)?),
        });
    }
    for (key, value) in &c.initial {
        let (_, target, _, on_plant) = kind.initial.iter().find(|(k, ..)| k == key).unwrap();
        let bind = Modifier::Bind((*target).into(), literal(value)?);
        if *on_plant {
            plant.push(bind);
        } else {
            modifiers.push(bind);
        }
    }
    let mut components = Vec::new();
    let mut equalities = Vec::new();
    crate::lower::sensor_components(c, &mut components)?;
    crate::lower::imu_boundary(c, &mut components, &mut equalities);
    if let Some(sensors) = crate::lower::sensor_modifiers(c)? {
        plant.push(sensors);
    }
    if !plant.is_empty() {
        modifiers.push(Modifier::Nested("plant".into(), plant));
    }
    Ok(CompositionSpec {
        name: c.model_name.clone(),
        base: Some(kind.export.into()),
        modifiers,
        components,
        equalities,
        connections: Vec::new(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::Path;

    fn source(class: &str) -> String {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let rel = class
            .strip_prefix("ModelicaCPSLibrary.")
            .unwrap()
            .replace('.', "/");
        std::fs::read_to_string(root.join(format!("{rel}.mo"))).unwrap()
    }

    /// Every configurable key must be a parameter the library declares, so the
    /// allow-lists cannot drift from the Modelica classes.
    #[test]
    fn configurable_keys_are_library_parameters() {
        let declares = |text: &str, name: &str| {
            text.lines().any(|line| {
                let line = line.trim_start();
                line.starts_with("parameter ")
                    && line
                        .split(|c: char| !(c.is_ascii_alphanumeric() || c == '_'))
                        .any(|word| word == name)
            })
        };
        for kind in KINDS {
            let with_sensors = source(kind.plant);
            let base = source(&kind.plant.replace("WithSensors", "Plant"));
            let export = source(kind.export);
            for (key, _) in kind.parameters {
                assert!(
                    declares(&with_sensors, key) || declares(&base, key),
                    "{}: plant.{key} is not a parameter of {}",
                    kind.name,
                    kind.plant
                );
            }
            for (key, fields) in kind.records {
                assert!(declares(&base, key), "{}: plant.{key}", kind.name);
                let record = source(
                    "ModelicaCPSLibrary.Physical.Mechanical.Aerodynamics.Airframe.Coefficients",
                );
                for field in *fields {
                    assert!(
                        declares(&record, field),
                        "{}: plant.{key}.{field}",
                        kind.name
                    );
                }
            }
            for (_, target, _, on_plant) in kind.initial {
                let text = if *on_plant { &base } else { &export };
                assert!(declares(text, target), "{}: initial -> {target}", kind.name);
            }
        }
    }

    #[test]
    fn empty_configuration_extends_the_export_unchanged() {
        for kind in KINDS {
            let c = crate::config::parse(&format!(
                "schema_version = 3\nvehicle = \"{}\"\n",
                kind.name
            ))
            .unwrap();
            let text = compose(&c, "vio").unwrap().emit().unwrap();
            assert_eq!(
                text,
                format!(
                    "within;\nmodel {0}\n  extends {1}(\n    );\nend {0};\n",
                    kind.default_model_name, kind.export
                )
            );
        }
    }

    #[test]
    fn plant_initial_interface_and_sensors_become_modifiers() {
        let c = crate::config::parse(
            r#"
schema_version = 3
vehicle = "fixed_wing"
model_name = "Glider"
[plant]
mass = 2.5
gnssPosition_b = [0.1, 0.0, -0.05]
[plant.aero]
CL0 = 0.6
[initial]
altitude = 30.0
speed = 15.0
[interface]
lat0 = 47.0
[models.imu]
model = "digital"
[imu]
samplePeriod = 0.0025
adcDecimation = 4
fullScale = [156.9, 156.9, 156.9, 34.9, 34.9, 34.9]
quantum = [0.0048, 0.0048, 0.0048, 0.0011, 0.0011, 0.0011]
digitalTau = [0.002, 0.002, 0.002, 0.002, 0.002, 0.002]
analogTau = [0.001, 0.001, 0.001, 0.001, 0.001, 0.001]
"#,
        )
        .unwrap();
        let text = compose(&c, "vio").unwrap().emit().unwrap();
        for expected in [
            "extends ModelicaCPSLibrary.Exports.FMU.FixedWing(",
            "lat0=47.0",
            "start_altitude_m=30.0",
            "start_speed_mps=15.0",
            "plant(aero(CL0=0.6)",
            "gnssPosition_b={0.1,0.0,-0.05}",
            "mass=2.5",
            "redeclare model ImuModel=ModelicaCPSLibrary.Systems.Sensing.IMU.DigitalSensor()",
            "output Integer imuSequence",
            "imuSequence = plant.sensors.imu.sequence;",
        ] {
            assert!(text.contains(expected), "missing {expected:?} in\n{text}");
        }
        assert!(
            !text.contains("pwm_min"),
            "unset interface fields keep the export default"
        );
    }

    #[test]
    fn rejects_multirotor_tables_unknown_parameters_and_bad_values() {
        for (body, message) in [
            ("[motor]\ntau = 0.1", "multirotor tables"),
            (
                "[models]\nmotor = \"first_order_speed\"",
                "part of the plant",
            ),
            ("[plant]\nwingspan = 2.0", "unknown fields wingspan"),
            ("[plant]\nmass = -1.0", "plant.mass"),
            (
                "[plant]\nR_bImu = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]]",
                "rotation",
            ),
            ("[plant.aero]\nCL9 = 1.0", "unknown fields CL9"),
            ("[initial]\nq_start = [0.0, 0.0, 0.0, 0.0]", "nonzero"),
            (
                "[interface]\npwm_max = 900.0",
                "pwm_max must exceed pwm_min",
            ),
            (
                "[models.gnss]\nmodel = \"statistical\"",
                "only the ideal model",
            ),
        ] {
            let error = crate::config::parse(&format!(
                "schema_version = 3\nvehicle = \"fixed_wing\"\n{body}\n"
            ))
            .unwrap_err();
            assert!(error.contains(message), "{body}: {error}");
        }
        let error = crate::config::parse("schema_version = 3\nvehicle = \"boat\"\n").unwrap_err();
        assert!(error.contains("multirotor, rover or fixed_wing"), "{error}");
        let error = crate::config::parse(
            "schema_version = 3\nvehicle = \"rover\"\n[initial]\naltitude = 1.0\n",
        )
        .unwrap_err();
        assert!(error.contains("unknown fields altitude"), "{error}");
    }

    #[test]
    fn host_configuration_table_selects_the_vehicle() {
        let host = r#"
[FMU]
active = "rover"
[FMU.models.rover.physics]
schema_version = 3
vehicle = "rover"
model_name = "HostRover"
[FMU.models.rover.physics.plant]
mass = 3.5
"#;
        let c = crate::config::parse_at(host, Some("FMU.models.rover.physics")).unwrap();
        assert_eq!(
            (c.vehicle.as_str(), c.model_name.as_str()),
            ("rover", "HostRover")
        );
        assert!(compose(&c, "vio")
            .unwrap()
            .emit()
            .unwrap()
            .contains("plant(mass=3.5)"));
        assert!(compose(&c, "plant")
            .unwrap_err()
            .contains("only the vio profile"));
    }
}
