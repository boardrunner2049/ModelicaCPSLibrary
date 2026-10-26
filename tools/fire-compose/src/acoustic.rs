//! Independent low-order acoustic vulnerability. No implicit attachment to an IMU tier.
use crate::config::{array, number, table, Table};
use crate::{imu, Result};
use toml::Value;
pub fn resolve(raw: &Value, models: &Table) -> Result<Table> {
    let top = table(raw, &["acoustic"], "vulnerabilities")?;
    let Some(raw) = top.get("acoustic") else {
        return Ok(Table::new());
    };
    let t = table(
        raw,
        &[
            "model",
            "effectiveArea",
            "incidenceAngle",
            "driveCoupling",
            "senseCoupling",
            "accelCoupling",
            "frequency",
            "phase",
            "gyroPhysics",
        ],
        "vulnerabilities.acoustic",
    )?;
    let model = t.get("model").and_then(Value::as_str).unwrap_or("");
    let sensor = models
        .get("imu")
        .and_then(Value::as_table)
        .and_then(|v| v.get("model"))
        .and_then(Value::as_str)
        .unwrap_or("ideal");
    if !((model == "modal_force" && sensor == "electromechanical")
        || (model == "equivalent_rate" && matches!(sensor, "digital" | "errors")))
    {
        return Err("acoustic modal_force requires electromechanical IMU; equivalent_rate requires digital/errors IMU".into());
    }
    for key in ["effectiveArea", "driveCoupling", "senseCoupling"] {
        if !t.contains_key(key) {
            return Err(format!("vulnerabilities.acoustic.{key} must be explicit"));
        }
    }
    let mut out = t.clone();
    for (key, value) in t {
        let label = format!("vulnerabilities.acoustic.{key}");
        match key.as_str() {
            "model" => {}
            "driveCoupling" | "senseCoupling" | "accelCoupling" => {
                array(value, &[3], &label)?;
            }
            "gyroPhysics" => {
                out.insert(
                    key.clone(),
                    Value::Table(imu::resolve_physics(value, true, &label)?),
                );
            }
            _ => {
                number(
                    value,
                    &label,
                    matches!(key.as_str(), "effectiveArea" | "frequency"),
                    false,
                )?;
            }
        }
    }
    if model == "modal_force"
        && ["frequency", "phase", "gyroPhysics"]
            .iter()
            .any(|k| t.contains_key(*k))
    {
        return Err("modal_force consumes instantaneous pressure; frequency/phase/gyroPhysics belong to source or sensor".into());
    }
    if model == "equivalent_rate" {
        if t.contains_key("accelCoupling") {
            return Err("equivalent_rate models gyro only; accelCoupling is unsupported".into());
        }
        for key in ["frequency", "gyroPhysics"] {
            if !t.contains_key(key) {
                return Err(format!("equivalent_rate requires {key}"));
            }
        }
    }
    Ok(out)
}
