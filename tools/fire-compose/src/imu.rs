//! Current I0-I3 presets and explicit legacy compositions.
use crate::config::{array, integer, number, table, ResolvedFireConfig, Table};
use crate::Result;
use toml::Value;
const COMMON: &[&str] = &["accelBias", "gyroBias"];
const CLOCK: &[&str] = &["samplePeriod", "latencySamples"];
const STAT: &[&str] = &[
    "seed",
    "streamId",
    "scale",
    "accelMisalignment",
    "gyroMisalignment",
    "gyroGSensitivity",
    "noiseStd",
    "biasStd",
    "biasTau",
];
const ADC: &[&str] = &["adcDecimation", "fullScale", "quantum", "digitalTau"];
const TEMP: &[&str] = &["referenceTemperature", "temperatureCoefficient"];
pub const ACOUSTIC: &[&str] = &[
    "resonanceFrequency",
    "dampingRatio",
    "pressureGain",
    "displacementGain",
    "velocityGain",
    "quadraticGain",
];

pub fn selection(raw: &Value) -> Result<Table> {
    let t = table(raw, &["model", "response"], "models.imu")?;
    if t.contains_key("response") {
        if t.len() != 1 || t["response"].as_str() != Some("ideal") {
            return Err("models.imu.response is legacy ideal only; select model instead".into());
        }
        return Ok(Table::from_iter([(
            "model".into(),
            Value::String("ideal".into()),
        )]));
    }
    let name = t.get("model").and_then(Value::as_str).unwrap_or("");
    if ![
        "ideal",
        "digital",
        "errors",
        "electromechanical",
        "statistical",
        "device",
        "acoustic",
    ]
    .contains(&name)
    {
        return Err("models.imu.model must be ideal, digital, errors, electromechanical (legacy: statistical, device, acoustic)".into());
    }
    Ok(t.clone())
}
impl ResolvedFireConfig {
    pub fn imu_model(&self) -> &str {
        self.models
            .get("imu")
            .and_then(Value::as_table)
            .and_then(|t| t.get("model"))
            .and_then(Value::as_str)
            .unwrap_or("ideal")
    }
    pub fn imu_level(&self) -> &'static str {
        match self.imu_model() {
            "digital" => "I1",
            "errors" => "I2",
            "electromechanical" => "I3",
            "statistical" => "legacy_I1",
            "device" => "legacy_I2",
            "acoustic" => "legacy_I3",
            _ => "I0",
        }
    }
    pub fn imu_class(&self) -> &'static str {
        match self.imu_model() {
            "digital" => "ModelicaCPSLibrary.Systems.Sensing.IMU.DigitalSensor",
            "errors" => "ModelicaCPSLibrary.Systems.Sensing.IMU.ErrorSensor",
            "electromechanical" => "ModelicaCPSLibrary.Systems.Sensing.IMU.ElectromechanicalSensor",
            "statistical" => "ModelicaCPSLibrary.Systems.Sensing.IMU.StatisticalSensor",
            "device" => "ModelicaCPSLibrary.Systems.Sensing.IMU.DeviceSensor",
            "acoustic" => "ModelicaCPSLibrary.Systems.Sensing.IMU.AcousticSensor",
            _ => "ModelicaCPSLibrary.Systems.Sensing.IMU.Sensor",
        }
    }
    pub fn imu_temperature(&self) -> bool {
        matches!(
            self.imu_model(),
            "errors" | "electromechanical" | "device" | "acoustic"
        )
    }
}

pub fn resolve(raw: &Value, models: &Table) -> Result<Table> {
    let model = models
        .get("imu")
        .and_then(Value::as_table)
        .and_then(|t| t.get("model"))
        .and_then(Value::as_str)
        .unwrap_or("ideal");
    let mut allowed = if model == "digital" {
        Vec::new()
    } else {
        COMMON.to_vec()
    };
    if model != "ideal" {
        allowed.extend(CLOCK);
    }
    if !matches!(model, "ideal" | "digital") {
        allowed.extend(STAT);
    }
    if !matches!(model, "ideal" | "statistical") {
        allowed.extend(ADC);
    }
    if matches!(model, "digital" | "errors" | "device" | "acoustic") {
        allowed.push("analogTau");
    }
    if matches!(
        model,
        "errors" | "electromechanical" | "device" | "acoustic"
    ) {
        allowed.extend(TEMP);
    }
    if model == "acoustic" {
        allowed.extend(ACOUSTIC);
    }
    if model == "electromechanical" {
        allowed.extend(["gyroPhysics", "accelPhysics"]);
    }
    let t = table(raw, &allowed, "imu")?;
    let mut required = Vec::new();
    if model != "ideal" {
        required.push("samplePeriod");
    }
    if !matches!(model, "ideal" | "digital") {
        required.extend(["seed", "streamId"]);
    }
    if !matches!(model, "ideal" | "statistical") {
        required.extend(ADC);
    }
    if matches!(model, "digital" | "errors" | "device" | "acoustic") {
        required.push("analogTau");
    }
    if model == "acoustic" {
        required.extend([
            "resonanceFrequency",
            "dampingRatio",
            "pressureGain",
            "displacementGain",
        ]);
    }
    if model == "electromechanical" {
        required.extend(["gyroPhysics", "accelPhysics"]);
    }
    for key in required {
        if !t.contains_key(key) {
            return Err(format!("{model} IMU requires explicit imu.{key}"));
        }
    }
    let mut out = t.clone();
    for (key, value) in t {
        let label = format!("imu.{key}");
        if ["gyroPhysics", "accelPhysics"].contains(&key.as_str()) {
            out.insert(
                key.clone(),
                Value::Table(resolve_physics(value, key == "gyroPhysics", &label)?),
            );
        } else if ["seed", "streamId", "latencySamples", "adcDecimation"].contains(&key.as_str()) {
            let n = integer(value, &label, if key == "latencySamples" { 0 } else { 1 })?;
            let max = if key == "latencySamples" {
                10000
            } else {
                i32::MAX as usize
            };
            if n > max {
                return Err(format!("{label}: exceeds supported bound {max}"));
            }
        } else if ["samplePeriod", "referenceTemperature"].contains(&key.as_str()) {
            number(value, &label, true, false)?;
        } else if ["accelMisalignment", "gyroMisalignment", "gyroGSensitivity"]
            .contains(&key.as_str())
        {
            array(value, &[3, 3], &label)?;
        } else {
            array(
                value,
                &[if COMMON.contains(&key.as_str()) { 3 } else { 6 }],
                &label,
            )?;
            let positive = [
                "biasTau",
                "analogTau",
                "fullScale",
                "resonanceFrequency",
                "dampingRatio",
            ]
            .contains(&key.as_str());
            let nonnegative =
                ["noiseStd", "biasStd", "quantum", "digitalTau"].contains(&key.as_str());
            for v in value.as_array().unwrap() {
                number(v, &label, positive, nonnegative)?;
            }
        }
    }
    if let Some(period) = t.get("samplePeriod") {
        let period = number(period, "imu.samplePeriod", true, false)?;
        let decimation = t
            .get("adcDecimation")
            .and_then(Value::as_integer)
            .unwrap_or(1);
        if period / (decimation as f64) < 1e-12 {
            return Err("IMU internal sample period must be at least 1e-12 s".into());
        }
    }
    Ok(out)
}

pub fn resolve_physics(raw: &Value, gyro: bool, label: &str) -> Result<Table> {
    let mut defaults: Vec<(&str, f64)> = if gyro {
        vec![
            ("mass", 2.5e-9),
            ("driveFrequency", 1000.),
            ("senseFrequency", 1500.),
            ("referenceFrequency", 1000.),
            ("driveQ", 30.),
            ("senseQ", 20.),
            ("driveAmplitude", 2e-7),
            ("crossStiffness", 0.),
            ("driveC0", 1e-14),
            ("driveLength", 5e-6),
            ("biasVoltage", 2.),
            ("maxDriveVoltage", 1.),
            ("readC0", 1e-14),
            ("readLength", 5e-6),
            ("carrierVoltage", 1.),
            ("feedbackCapacitance", 1e-13),
            ("afeTau", 1e-5),
            ("rail", 3.3),
            ("detectorTau", 0.003),
            ("senseTau", 0.003),
            ("agcKp", 0.5),
            ("agcKi", 20.),
            ("pllKp", 40.),
            ("pllKi", 400.),
            ("pllRange", 0.15),
            ("startupFraction", 1.),
            ("demodPhaseError", 0.),
        ]
    } else {
        vec![
            ("mass", 1e-8),
            ("frequency", 2000.),
            ("quality", 0.7),
            ("readC0", 2e-13),
            ("readLength", 5e-6),
            ("carrierVoltage", 1.),
            ("feedbackCapacitance", 1e-12),
            ("afeTau", 1e-5),
            ("rail", 3.3),
        ]
    };
    let allowed: Vec<&str> = defaults.iter().map(|x| x.0).collect();
    let t = table(raw, &allowed, label)?;
    for key in if gyro {
        vec![
            "mass",
            "driveFrequency",
            "senseFrequency",
            "driveQ",
            "senseQ",
            "driveAmplitude",
        ]
    } else {
        vec!["mass", "frequency", "quality"]
    } {
        if !t.contains_key(key) {
            return Err(format!("{label}.{key} must be explicit (illustrative defaults are not hardware calibration)"));
        }
    }
    if gyro {
        defaults
            .iter_mut()
            .find(|x| x.0 == "referenceFrequency")
            .unwrap()
            .1 = number(&t["driveFrequency"], label, true, false)?;
    }
    let mut out = Table::new();
    for (key, default) in defaults {
        let v = t.get(key).cloned().unwrap_or(Value::Float(default));
        let signed = ["crossStiffness", "demodPhaseError"].contains(&key);
        let zero = ["agcKp", "agcKi", "pllKp", "pllKi", "startupFraction"].contains(&key);
        number(&v, &format!("{label}.{key}"), !signed && !zero, zero)?;
        out.insert(key.into(), v);
    }
    let n = |key: &str| number(&out[key], key, false, false).unwrap();
    if gyro {
        if n("pllRange") >= 1. || n("startupFraction") > 1. {
            return Err(format!(
                "{label}: pllRange must be <1 and startupFraction <=1"
            ));
        }
        let wd = 2. * std::f64::consts::PI * n("driveFrequency");
        let ws = 2. * std::f64::consts::PI * n("senseFrequency");
        if n("crossStiffness").powi(2) >= n("mass").powi(2) * wd.powi(2) * ws.powi(2) {
            return Err(format!(
                "{label}: stiffness matrix must be positive definite"
            ));
        }
        if n("driveAmplitude") >= n("driveLength").min(n("readLength")) {
            return Err(format!("{label}: driveAmplitude exceeds electrode stroke"));
        }
        let u0 = n("mass") * wd / n("driveQ")
            * (2. * std::f64::consts::PI * n("referenceFrequency"))
            * n("driveAmplitude")
            / (2. * n("driveC0") * n("biasVoltage") / n("driveLength"));
        if !u0.is_finite() || u0 >= n("maxDriveVoltage") {
            return Err(format!("{label}: insufficient drive voltage headroom"));
        }
    }
    Ok(out)
}
