//! Magnetometer (M0/M1), barometer (B0/B1) and GNSS (G0/G1) selection:
//! `[models.KIND] model = ...` picks the level, `[KIND]` sets its parameters.
//! Every vehicle kind composes them into `sensors(...)` the same way.
use crate::composition::Modifier;
use crate::config::{array, integer, number, table, Table};
use crate::Result;
use toml::Value;

#[derive(Clone, Copy, PartialEq)]
enum Shape {
    Positive,
    Nonnegative,
    Time,
    Count,
    Seed,
    Vec3,
    NonnegativeVec3,
    PositiveVec3,
    PositiveVec6,
    Mat3,
    Mat6,
}

pub struct Level {
    pub level: &'static str,
    pub model: &'static str,
    pub class: &'static str,
    pub description: &'static str,
    /// [KIND] parameters: (key, shape). The ideal level's apply to every level.
    parameters: &'static [(&'static str, Shape)],
}

pub struct Sensor {
    /// Table and [models] key.
    pub name: &'static str,
    /// SensorSuite replaceable model and component.
    replaceable: &'static str,
    component: &'static str,
    pub levels: &'static [Level],
}

const SAMPLING: [(&str, Shape); 5] = [
    ("samplePeriod", Shape::Positive),
    ("startTime", Shape::Time),
    ("latencySamples", Shape::Count),
    ("seed", Shape::Seed),
    ("streamId", Shape::Seed),
];

macro_rules! sampled {
    ($($item:expr),* $(,)?) => {
        &[SAMPLING[0], SAMPLING[1], SAMPLING[2], SAMPLING[3], SAMPLING[4], $($item,)*]
    };
}

pub const MAGNETOMETER: Sensor = Sensor {
    name: "magnetometer",
    replaceable: "MagnetometerModel",
    component: "magnetometer",
    levels: &[
        Level {
            level: "M0",
            model: "ideal",
            class: "ModelicaCPSLibrary.Systems.Sensing.Magnetometer.Sensor",
            description: "mounted Earth field plus a hard-iron offset, continuous",
            parameters: &[("bias", Shape::Vec3)],
        },
        Level {
            level: "M1",
            model: "sampled",
            class: "ModelicaCPSLibrary.Systems.Sensing.Magnetometer.SampledSensor",
            description: "sampled; soft-iron matrix, hard-iron offset, noise, bias drift, quantization, clipping, latency",
            parameters: sampled![
                ("softIron", Shape::Mat3),
                ("noiseStd", Shape::NonnegativeVec3),
                ("biasStd", Shape::NonnegativeVec3),
                ("biasTau", Shape::PositiveVec3),
                ("quantum", Shape::NonnegativeVec3),
                ("fullScale", Shape::PositiveVec3),
            ],
        },
    ],
};

pub const BAROMETER: Sensor = Sensor {
    name: "barometer",
    replaceable: "BarometerModel",
    component: "barometer",
    levels: &[
        Level {
            level: "B0",
            model: "ideal",
            class: "ModelicaCPSLibrary.Systems.Sensing.Barometer.Sensor",
            description:
                "ambient pressure and temperature at the sensor (ISA troposphere), continuous",
            parameters: &[
                ("pressureBias", Shape::Time),
                ("temperatureBias", Shape::Time),
                ("altitudeBias", Shape::Time),
            ],
        },
        Level {
            level: "B1",
            model: "sampled",
            class: "ModelicaCPSLibrary.Systems.Sensing.Barometer.SampledSensor",
            description:
                "sampled; pressure noise and drift, temperature noise, quantization, latency",
            parameters: sampled![
                ("pressureNoiseStd", Shape::Nonnegative),
                ("pressureDriftStd", Shape::Nonnegative),
                ("pressureDriftTau", Shape::Positive),
                ("pressureQuantum", Shape::Nonnegative),
                ("temperatureNoiseStd", Shape::Nonnegative),
                ("temperatureQuantum", Shape::Nonnegative),
            ],
        },
    ],
};

pub const GNSS: Sensor = Sensor {
    name: "gnss",
    replaceable: "GnssModel",
    component: "gnss",
    levels: &[
        Level {
            level: "G0",
            model: "ideal",
            class: "ModelicaCPSLibrary.Systems.Sensing.GNSS.Sensor",
            description: "antenna position and velocity, continuous, with fixed biases",
            parameters: &[("positionBias", Shape::Vec3), ("velocityBias", Shape::Vec3)],
        },
        Level {
            level: "G1",
            model: "statistical",
            class: "ModelicaCPSLibrary.Systems.Sensing.GNSS.StatisticalSensor",
            description: "PVT epochs with correlated white noise, Gauss-Markov errors and latency",
            parameters: sampled![
                ("noiseFactor", Shape::Mat6),
                ("biasFactor", Shape::Mat6),
                ("biasTau", Shape::PositiveVec6),
            ],
        },
    ],
};

pub const SENSORS: [&Sensor; 3] = [&MAGNETOMETER, &BAROMETER, &GNSS];

impl Sensor {
    pub fn level(&self, model: &str) -> Option<&'static Level> {
        self.levels.iter().find(|l| l.model == model)
    }

    /// The [KIND] keys a level accepts: the ideal level's and its own.
    pub fn keys(&self, model: &str) -> Vec<&'static str> {
        let mut keys: Vec<&str> = self.levels[0].parameters.iter().map(|p| p.0).collect();
        if let Some(level) = self
            .level(model)
            .filter(|l| l.model != self.levels[0].model)
        {
            keys.extend(level.parameters.iter().map(|p| p.0));
        }
        keys
    }

    fn shape(&self, model: &str, key: &str) -> Option<Shape> {
        let level = self.level(model)?;
        self.levels[0]
            .parameters
            .iter()
            .chain(level.parameters)
            .find(|p| p.0 == key)
            .map(|p| p.1)
    }

    /// The selected model name in a resolved `[models]` table.
    pub fn selected<'a>(&self, models: &'a Table) -> &'a str {
        models
            .get(self.name)
            .and_then(Value::as_table)
            .and_then(|t| t.get("model"))
            .and_then(Value::as_str)
            .unwrap_or("ideal")
    }
}

/// `[models.KIND]`: `model = "<level model>"` (legacy `response = "ideal"`).
pub fn selection(sensor: &Sensor, raw: &Value) -> Result<Table> {
    let label = format!("models.{}", sensor.name);
    let t = table(raw, &["model", "response"], &label)?;
    let name = match (t.get("model"), t.get("response")) {
        (Some(m), None) => m.as_str(),
        (None, Some(r)) if r.as_str() == Some("ideal") => Some("ideal"),
        _ => None,
    };
    match name.and_then(|n| sensor.level(n)) {
        Some(level) => Ok(Table::from_iter([(
            "model".into(),
            Value::String(level.model.into()),
        )])),
        None => Err(format!(
            "{label}.model must be {}",
            sensor
                .levels
                .iter()
                .map(|l| format!("\"{}\" ({})", l.model, l.level))
                .collect::<Vec<_>>()
                .join(" or ")
        )),
    }
}

/// `[KIND]`: the selected level's parameters, checked.
pub fn resolve(sensor: &Sensor, raw: &Value, models: &Table) -> Result<Table> {
    let model = sensor.selected(models);
    let keys = sensor.keys(model);
    let t = table(raw, &keys, sensor.name).map_err(|e| {
        format!(
            "{e} ({} model \"{model}\"; select another in [models.{}])",
            sensor.name, sensor.name
        )
    })?;
    for (key, value) in t {
        let label = format!("{}.{key}", sensor.name);
        match sensor.shape(model, key).expect("accepted keys have shapes") {
            Shape::Positive => {
                number(value, &label, true, false)?;
            }
            Shape::Nonnegative => {
                number(value, &label, false, true)?;
            }
            Shape::Time => {
                number(value, &label, false, false)?;
            }
            Shape::Count => {
                let n = integer(value, &label, 0)?;
                if n > 10000 {
                    return Err(format!("{label}: exceeds supported bound 10000"));
                }
            }
            Shape::Seed => {
                let n = integer(value, &label, 1)?;
                if n > i32::MAX as usize {
                    return Err(format!("{label}: exceeds supported bound {}", i32::MAX));
                }
            }
            Shape::Vec3 => array(value, &[3], &label)?,
            Shape::Mat3 => array(value, &[3, 3], &label)?,
            Shape::Mat6 => array(value, &[6, 6], &label)?,
            Shape::NonnegativeVec3 | Shape::PositiveVec3 | Shape::PositiveVec6 => {
                let n = if matches!(sensor.shape(model, key), Some(Shape::PositiveVec6)) {
                    6
                } else {
                    3
                };
                array(value, &[n], &label)?;
                let positive = !matches!(sensor.shape(model, key), Some(Shape::NonnegativeVec3));
                for (i, x) in value.as_array().unwrap().iter().enumerate() {
                    number(x, &format!("{label}[{}]", i + 1), positive, !positive)?;
                }
            }
        }
    }
    Ok(t.clone())
}

/// Check and normalize every `[models.KIND]` of these sensors in `models` into `out`.
pub fn select_all(models: &Table, out: &mut Table) -> Result<()> {
    for sensor in SENSORS {
        if let Some(raw) = models.get(sensor.name) {
            out.insert(sensor.name.into(), Value::Table(selection(sensor, raw)?));
        }
    }
    Ok(())
}

/// Resolve the `[magnetometer]`, `[barometer]` and `[gnss]` tables of `root`.
pub fn resolve_all(root: &Table, models: &Table) -> Result<[Table; 3]> {
    let get = |sensor: &Sensor| resolve(sensor, &crate::config::section(root, sensor.name), models);
    Ok([get(&MAGNETOMETER)?, get(&BAROMETER)?, get(&GNSS)?])
}

/// The `sensors(...)` modifiers for these sensors: a redeclaration for a
/// non-ideal level and the configured parameters, or nothing.
pub fn modifiers(models: &Table, tables: [&Table; 3]) -> Result<Vec<Modifier>> {
    let mut out = Vec::new();
    for (sensor, parameters) in SENSORS.into_iter().zip(tables) {
        let level = sensor
            .level(sensor.selected(models))
            .expect("resolved selection");
        if level.model != sensor.levels[0].model {
            out.push(Modifier::RedeclareModel {
                name: sensor.replaceable.into(),
                class: level.class.into(),
                modifiers: vec![],
            });
        }
        if !parameters.is_empty() {
            let fields = parameters
                .iter()
                .map(|(k, v)| Ok(Modifier::Bind(k.clone(), crate::lower::literal(v)?)))
                .collect::<Result<Vec<_>>>()?;
            out.push(Modifier::Nested(sensor.component.into(), fields));
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn models(sensor: &str, model: &str) -> Table {
        let mut t = Table::new();
        t.insert(
            sensor.into(),
            Value::Table(Table::from_iter([(
                "model".into(),
                Value::String(model.into()),
            )])),
        );
        t
    }

    #[test]
    fn levels_select_and_check_parameters() {
        let raw: Value = toml::from_str("model = \"sampled\"").unwrap();
        assert_eq!(
            selection(&BAROMETER, &raw).unwrap()["model"].as_str(),
            Some("sampled")
        );
        assert!(selection(&BAROMETER, &toml::from_str("model = \"x\"").unwrap()).is_err());
        let m = models("barometer", "sampled");
        let ok: Value =
            toml::from_str("pressureNoiseStd = 2.0\nsamplePeriod = 0.02\nseed = 3").unwrap();
        assert!(resolve(&BAROMETER, &ok, &m).is_ok());
        let bad: Value = toml::from_str("pressureNoiseStd = -1.0").unwrap();
        assert!(resolve(&BAROMETER, &bad, &m).is_err());
        // An M1 key on the ideal level is refused.
        let raw: Value = toml::from_str("noiseStd = [1e-7, 1e-7, 1e-7]").unwrap();
        assert!(resolve(&MAGNETOMETER, &raw, &Table::new()).is_err());
        assert!(resolve(&MAGNETOMETER, &raw, &models("magnetometer", "sampled")).is_ok());
        let g: Value = toml::from_str("biasTau = [1.0, 1.0, 1.0, 1.0, 1.0, 0.0]").unwrap();
        assert!(resolve(&GNSS, &g, &models("gnss", "statistical")).is_err());
    }

    /// Every parameter a level accepts is declared by its Modelica class.
    #[test]
    fn parameters_are_declared_by_the_classes() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        for sensor in SENSORS {
            for level in sensor.levels {
                let mut source = String::new();
                for file in level_files(level.class) {
                    source += &std::fs::read_to_string(root.join(file)).unwrap();
                }
                for key in sensor.keys(level.model) {
                    assert!(
                        source.contains(&format!(" {key}")) && source.contains("parameter"),
                        "{} does not declare {key}",
                        level.class
                    );
                    let declared = source.lines().any(|l| {
                        l.trim_start().starts_with("parameter")
                            && l.split(|c: char| !c.is_alphanumeric() && c != '_')
                                .any(|w| w == key)
                    });
                    assert!(declared, "{} does not declare parameter {key}", level.class);
                }
            }
        }
    }

    fn level_files(class: &str) -> Vec<String> {
        let path = class
            .trim_start_matches("ModelicaCPSLibrary.")
            .replace('.', "/");
        let dir = path.rsplit_once('/').unwrap().0.to_string();
        let mut files = vec![format!("{path}.mo")];
        for partial in ["PartialSensor", "PartialSampledSensor"] {
            files.push(format!("{dir}/{partial}.mo"));
        }
        files
            .into_iter()
            .filter(|f| {
                std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                    .join("../..")
                    .join(f)
                    .is_file()
            })
            .collect()
    }
}
