//! What a configuration can select, as data: vehicle kinds with their
//! configurable parameters, and the models available for each part. Frontends
//! (tools/compose) read it with `fire-compose catalog`, so they offer
//! exactly what the composer accepts.
use crate::{config, imu, parts, sensors, vehicle};
use serde_json::{json, Value};

pub fn catalog() -> Value {
    let imu: Vec<Value> = imu::MODELS
        .iter()
        .enumerate()
        .map(|(level, model)| {
            let (keys, required) = imu::keys(model);
            json!({"level": format!("I{level}"), "model": model, "keys": keys, "required": required})
        })
        .collect();
    let propulsion: Vec<Value> = config::PROPULSION
        .iter()
        .map(|(level, esc, motor, about)| {
            json!({"level": level, "esc": esc, "motor": motor, "description": about})
        })
        .collect();
    let mut vehicles = serde_json::Map::new();
    vehicles.insert(
        "multirotor".into(),
        json!({
            "parts": ["airframe", "propulsion", "imu", "gnss", "magnetometer", "barometer"],
            "tables": ["geometry", "mass", "motor", "esc", "power", "rotor", "initial",
                       "landing_gear", "interface"],
        }),
    );
    for kind in vehicle::KINDS {
        vehicles.insert(kind.name.into(), kind.describe());
    }
    json!({
        "vehicles": vehicles,
        "models": {
            "imu": imu,
            "propulsion": propulsion,
            "rotor": config::ROTORS,
            "magnetometer": sensor_levels(&sensors::MAGNETOMETER),
            "barometer": sensor_levels(&sensors::BAROMETER),
            "gnss": sensor_levels(&sensors::GNSS),
        },
        "parts": parts::catalog(),
        "vendor_configs": {
            "gnss": [{
                "vendor": "m9n", "part": "ublox_neo_m9n", "name": "u-blox NEO-M9N",
                "model": "ideal", "levels": ["G0"],
                "vendor_model": "ModelicaCPSLibrary.Systems.Sensing.GNSS.VendorConfig.M9N",
                "defaults": {"reportedPdop": 1.0},
                "keys": ["samplePeriod", "reportedPdop", "pdopLimit", "staticHoldSpeedThreshold", "staticHoldMaxDistance"]
            }],
            "barometer": [{
                "vendor": "bmp280", "part": "bosch_bmp280", "name": "Bosch BMP280",
                "source": "BMP280 vendor configuration model", "model": "ideal", "levels": ["B0"],
                "vendor_model": "ModelicaCPSLibrary.Systems.Sensing.Barometer.VendorConfig.BMP280",
                "defaults": {"pressureNoiseStd": 1.3},
                "keys": ["initialFilter", "normalModeConfigWritesAccepted", "pressureNoiseStd"]
            }]
        },
        "peripherals": crate::peripherals::catalog(),
        "payloads": crate::payloads::catalog(),
    })
}

fn sensor_levels(sensor: &sensors::Sensor) -> Vec<Value> {
    sensor
        .levels
        .iter()
        .map(|l| {
            json!({"level": l.level, "model": l.model, "description": l.description,
                   "keys": sensor.keys(l.model)})
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The configs/quad_pN.toml and quad_iN.toml examples are the catalog's levels.
    #[test]
    fn levels_match_the_example_configurations() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../configs");
        for (n, (_, esc, motor, _)) in config::PROPULSION.iter().enumerate().skip(1) {
            let raw = std::fs::read_to_string(root.join(format!("quad_p{n}.toml"))).unwrap();
            let c = config::parse_at(&raw, None).unwrap();
            assert_eq!(c.models["esc"].as_str(), Some(*esc));
            assert_eq!(c.models["motor"].as_str(), Some(*motor));
        }
        for (n, model) in imu::MODELS.iter().enumerate().skip(1) {
            let raw = std::fs::read_to_string(root.join(format!("quad_i{n}.toml"))).unwrap();
            assert_eq!(config::parse_at(&raw, None).unwrap().imu_model(), *model);
        }
        let c = catalog();
        for kind in ["multirotor", "rover", "fixed_wing"] {
            assert!(c["vehicles"][kind].is_object(), "{kind}");
        }
        assert_eq!(c["models"]["gnss"].as_array().unwrap().len(), 2);
        assert_eq!(c["models"]["barometer"].as_array().unwrap().len(), 2);
        assert_eq!(c["vendor_configs"]["gnss"][0]["vendor"], "m9n");
        assert_eq!(c["vendor_configs"]["barometer"][0]["vendor"], "bmp280");
    }
}
