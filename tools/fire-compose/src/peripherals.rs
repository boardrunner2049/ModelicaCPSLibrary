//! Peripherals: optional devices on a vehicle (ArduPilot's "optional
//! hardware"), each a kind with an ideal and a sampled level
//! (ModelicaCPSLibrary.Systems.Peripherals), selected per kind:
//!
//! ```toml
//! [peripherals.rangefinder]
//! part = "benewake_tfmini_plus"   # optional: a real device (parts/rangefinder/), as for sensors
//! level = "L1"                    # or model = "sampled"; default the part's (sampled), else ideal
//! position = [0.0, 0.0, 0.05]     # mount from the combined CG, body FRD [m]
//! orientation = "down"            # boresight (device +z): down | forward | back | left | right | up,
//!                                 #   or a 3x3 device-to-body matrix
//! noiseStd = 0.02                 # any parameter of the level; explicit values win over the part
//!
//! [peripherals.power_module]
//! part = "holybro_pm02d"
//! battery = "tattu_4s_5200_35c"   # the pack (parts/battery/), or a table of Power.Battery parameters
//! supply = true                   # the pack feeds the propulsion (multirotor P1-P4); default true
//! ```
//!
//! The composer adds the device to the vehicle (`truth` and, per kind, the
//! vehicle's shaft speeds or propulsion currents) and exports its readings as
//! the vehicle interface's optional capability (contract/vehicle_interface.toml),
//! which a host reads by name (rangefinder_distance_m, ...).
use crate::composition::{Instance, Modifier, Prefix, Value};
use crate::config::{array, integer, number, table, ResolvedFireConfig, Table};
use crate::Result;
use serde_json::{json, Value as Json};
use toml::Value as Toml;

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Shape {
    Real,
    Positive,
    Nonnegative,
    Fraction,
    Count,
    Seed,
    Vec3,
    /// A vector of any length >= 2.
    Vector,
}

pub struct Level {
    pub level: &'static str,
    pub model: &'static str,
    pub class: &'static str,
    pub description: &'static str,
    pub parameters: &'static [(&'static str, Shape)],
}

pub struct Kind {
    /// `[peripherals]` key and parts/ directory.
    pub name: &'static str,
    pub title: &'static str,
    /// Component name in the composed model.
    pub component: &'static str,
    pub capability: &'static str,
    /// Rigidly mounted (position, orientation, fed `truth`).
    pub mounted: bool,
    pub levels: &'static [Level],
    /// (exported variable, the component's output, dimension: 0 scalar, N fixed, usize::MAX one per shaft).
    pub outputs: &'static [(&'static str, &'static str, usize)],
    /// What ArduPilot calls it (its optional hardware page).
    pub ardupilot: &'static str,
}

const PER_SHAFT: usize = usize::MAX;

/// Outputs only some levels export: (kind, level, its outputs as in Kind::outputs).
pub const LEVEL_OUTPUTS: &[(&str, &str, &[(&str, &str, usize)])] = &[
    (
        "power_module",
        "W2",
        &[
            ("battery_voltage_sense_v", "voltageSense", 0),
            ("battery_current_sense_v", "currentSense", 0),
        ],
    ),
    (
        "rpm",
        "R2",
        &[
            ("rpm_pulse_count", "pulseCount", PER_SHAFT),
            ("rpm_last_edge_time_s", "lastEdgeTime", PER_SHAFT),
            ("rpm_edge_times_s", "edgeTimes", PER_SHAFT_BUFFER),
        ],
    ),
];
/// One element per shaft and buffered edge (R2): shafts times `bufferSize`.
const PER_SHAFT_BUFFER: usize = usize::MAX - 1;
const P: &str = "ModelicaCPSLibrary.Systems.Peripherals.";

macro_rules! sampled {
    ($($item:expr),* $(,)?) => {
        &[("samplePeriod", Shape::Positive), ("startTime", Shape::Nonnegative),
          ("latencySamples", Shape::Count), ("seed", Shape::Seed), ("streamId", Shape::Seed),
          $($item,)*]
    };
}

pub const RANGEFINDER: Kind = Kind {
    name: "rangefinder",
    title: "Rangefinder",
    component: "rangefinderSensor",
    capability: "rangefinder_v1",
    mounted: true,
    ardupilot: "Rangefinders (Sonar, Lidar, Depth Cameras)",
    levels: &[
        Level {
            level: "L0",
            model: "ideal",
            class: "Rangefinder",
            description: "slant range along the boresight to the ground plane, continuous",
            parameters: &[
                ("minRange", Shape::Nonnegative),
                ("maxRange", Shape::Positive),
                ("groundDown", Shape::Real),
            ],
        },
        Level {
            level: "L1",
            model: "sampled",
            class: "SampledRangefinder",
            description: "sampled; bias, range-proportional noise, resolution, limits, latency",
            parameters: sampled![
                ("minRange", Shape::Nonnegative),
                ("maxRange", Shape::Positive),
                ("groundDown", Shape::Real),
                ("bias", Shape::Real),
                ("noiseStd", Shape::Nonnegative),
                ("noiseSlope", Shape::Nonnegative),
                ("quantum", Shape::Nonnegative),
            ],
        },
    ],
    outputs: &[
        ("rangefinder_distance_m", "reported", 0),
        ("rangefinder_valid", "validity", 0),
    ],
};

pub const AIRSPEED: Kind = Kind {
    name: "airspeed",
    title: "Airspeed sensor",
    component: "airspeedSensor",
    capability: "airspeed_v1",
    mounted: true,
    ardupilot: "Airspeed Sensor",
    levels: &[
        Level {
            level: "A0",
            model: "ideal",
            class: "Airspeed",
            description: "pitot-static differential pressure of the axial air velocity and the air temperature, continuous",
            parameters: &[("pressure0", Shape::Positive), ("temperature0", Shape::Positive)],
        },
        Level {
            level: "A1",
            model: "sampled",
            class: "SampledAirspeed",
            description: "sampled; pressure offset, drift, noise, resolution and full scale; temperature noise",
            parameters: sampled![
                ("pressure0", Shape::Positive),
                ("temperature0", Shape::Positive),
                ("pressureBias", Shape::Real),
                ("pressureDriftStd", Shape::Nonnegative),
                ("pressureDriftTau", Shape::Positive),
                ("pressureNoiseStd", Shape::Nonnegative),
                ("pressureQuantum", Shape::Nonnegative),
                ("fullScale", Shape::Positive),
                ("temperatureBias", Shape::Real),
                ("temperatureNoiseStd", Shape::Nonnegative),
                ("temperatureQuantum", Shape::Nonnegative),
            ],
        },
    ],
    outputs: &[
        ("airspeed_differential_pressure_pa", "differentialPressure", 0),
        ("airspeed_temperature_k", "temperature", 0),
        ("airspeed_valid", "validity", 0),
    ],
};

pub const OPTICAL_FLOW: Kind = Kind {
    name: "optical_flow",
    title: "Optical flow sensor",
    component: "opticalFlowSensor",
    capability: "optical_flow_v1",
    mounted: true,
    ardupilot: "Optical Flow Sensor",
    levels: &[
        Level {
            level: "F0",
            model: "ideal",
            class: "OpticalFlow",
            description: "image flow and body rates over the ground plane, continuous",
            parameters: &[
                ("minRange", Shape::Nonnegative),
                ("maxRange", Shape::Positive),
                ("groundDown", Shape::Real),
            ],
        },
        Level {
            level: "F1",
            model: "sampled",
            class: "SampledOpticalFlow",
            description: "per frame; flow noise and pixel resolution, maximum rate, gyro noise, latency",
            parameters: sampled![
                ("minRange", Shape::Nonnegative),
                ("maxRange", Shape::Positive),
                ("groundDown", Shape::Real),
                ("flowNoiseStd", Shape::Nonnegative),
                ("flowQuantum", Shape::Nonnegative),
                ("maxFlowRate", Shape::Positive),
                ("flowScale", Shape::Real),
                ("gyroNoiseStd", Shape::Nonnegative),
                ("gyroBias", Shape::Real),
            ],
        },
    ],
    outputs: &[
        ("flow_rate_xy", "flowRate", 2),
        ("flow_body_rate_xy", "bodyRate", 2),
        ("flow_valid", "validity", 0),
    ],
};

pub const RPM: Kind = Kind {
    name: "rpm",
    title: "RPM sensor",
    component: "rpmSensor",
    capability: "rpm_v1",
    mounted: false,
    ardupilot: "RPM Sensor",
    levels: &[
        Level {
            level: "R0",
            model: "ideal",
            class: "Rpm",
            description: "shaft speeds (rotors; a rover's rear wheels), continuous",
            parameters: &[],
        },
        Level {
            level: "R1",
            model: "sampled",
            class: "SampledRpm",
            description: "pulses counted over a gate: one pulse of resolution, noise, latency",
            parameters: sampled![
                ("pulsesPerRevolution", Shape::Positive),
                ("noiseStd", Shape::Nonnegative),
                ("scale", Shape::Real),
                ("maxRpm", Shape::Positive),
            ],
        },
        Level {
            level: "R2",
            model: "pulses",
            class: "PulseRpm",
            description: "phase-continuous pulses per shaft: exact edge counts, buffered edge times, rpm from the latest two edges",
            parameters: &[
                ("pulsesPerRevolution", Shape::Positive),
                ("dutyCycle", Shape::Fraction),
                ("bufferSize", Shape::Count),
                ("edgeDelay", Shape::Nonnegative),
                ("edgeJitterStd", Shape::Nonnegative),
                ("timeout", Shape::Positive),
                ("seed", Shape::Seed),
                ("streamId", Shape::Seed),
            ],
        },
    ],
    outputs: &[("rpm", "rpm", PER_SHAFT), ("rpm_valid", "validity", 0)],
};

const POWER: &[(&str, Shape)] = &[
    ("avionicsCurrent", Shape::Nonnegative),
    ("efficiency", Shape::Fraction),
    ("propellerDiskArea", Shape::Positive),
    ("figureOfMerit", Shape::Fraction),
];

pub const POWER_MODULE: Kind = Kind {
    name: "power_module",
    title: "Power module and battery",
    component: "powerModule",
    capability: "battery_v1",
    mounted: false,
    ardupilot: "Power Modules; Smart Batteries",
    levels: &[
        Level {
            level: "W0",
            model: "ideal",
            class: "PowerModule",
            description: "battery pack (state of charge, resistance, polarization) and its voltage and current, measured ideally",
            parameters: POWER,
        },
        Level {
            level: "W1",
            model: "sampled",
            class: "SampledPowerModule",
            description: "sampled; voltage and current gain error, offset, noise, resolution and range",
            parameters: sampled![
                ("avionicsCurrent", Shape::Nonnegative),
                ("efficiency", Shape::Fraction),
                ("propellerDiskArea", Shape::Positive),
                ("figureOfMerit", Shape::Fraction),
                ("voltageScale", Shape::Real),
                ("voltageBias", Shape::Real),
                ("voltageNoiseStd", Shape::Nonnegative),
                ("voltageQuantum", Shape::Nonnegative),
                ("maxVoltage", Shape::Positive),
                ("currentScale", Shape::Real),
                ("currentBias", Shape::Real),
                ("currentNoiseStd", Shape::Nonnegative),
                ("currentQuantum", Shape::Nonnegative),
                ("maxCurrent", Shape::Positive),
            ],
        },
        Level {
            level: "W2",
            model: "analog",
            class: "AnalogPowerModule",
            description: "the module's analog sense voltages (Sensing.PowerModule.AnalogSensor), read through the autopilot's ADC and scaling",
            parameters: sampled![
                ("avionicsCurrent", Shape::Nonnegative),
                ("efficiency", Shape::Fraction),
                ("propellerDiskArea", Shape::Positive),
                ("figureOfMerit", Shape::Fraction),
                ("voltageGain", Shape::Positive),
                ("voltageOffset", Shape::Real),
                ("currentGain", Shape::Positive),
                ("currentOffset", Shape::Real),
                ("currentGainTempco", Shape::Real),
                ("outputMin", Shape::Real),
                ("outputMax", Shape::Positive),
                ("adcReference", Shape::Positive),
                ("adcBits", Shape::Count),
                ("adcNoiseStd", Shape::Nonnegative),
                ("voltageMultiplier", Shape::Positive),
                ("ampsPerVolt", Shape::Positive),
                ("ampsOffset", Shape::Real),
            ],
        },
    ],
    outputs: &[
        ("battery_voltage_v", "voltage", 0),
        ("battery_current_a", "current", 0),
        ("battery_consumed_ah", "consumed", 0),
        ("battery_remaining", "remaining", 0),
        ("battery_valid", "validity", 0),
    ],
};

/// Power.Battery parameters: `[peripherals.power_module.battery]`.
pub const BATTERY: &[(&str, Shape)] = &[
    ("cells", Shape::Count),
    ("capacity", Shape::Positive),
    ("resistance", Shape::Nonnegative),
    ("polarizationResistance", Shape::Nonnegative),
    ("polarizationTau", Shape::Positive),
    ("soc_start", Shape::Fraction),
    ("socPoints", Shape::Vector),
    ("cellVoltagePoints", Shape::Vector),
    ("currentTau", Shape::Positive),
];

pub const TEMPERATURE: Kind = Kind {
    name: "temperature",
    title: "Temperature sensor",
    component: "temperatureSensor",
    capability: "temperature_v1",
    mounted: true,
    ardupilot: "Temperature Sensors",
    levels: &[
        Level {
            level: "T0",
            model: "ideal",
            class: "Temperature",
            description: "air temperature at the mount (ISA troposphere), continuous",
            parameters: &[("temperature0", Shape::Positive), ("selfHeating", Shape::Real)],
        },
        Level {
            level: "T1",
            model: "sampled",
            class: "SampledTemperature",
            description: "thermal lag, sampled; offset, noise, resolution",
            parameters: sampled![
                ("temperature0", Shape::Positive),
                ("selfHeating", Shape::Real),
                ("timeConstant", Shape::Positive),
                ("bias", Shape::Real),
                ("noiseStd", Shape::Nonnegative),
                ("quantum", Shape::Nonnegative),
            ],
        },
    ],
    outputs: &[("temperature_k", "temperature", 0), ("temperature_valid", "validity", 0)],
};

pub const EXTERNAL_AHRS: Kind = Kind {
    name: "external_ahrs",
    title: "External AHRS / INS",
    component: "externalAhrs",
    capability: "external_ahrs_v1",
    mounted: true,
    ardupilot: "External AHRS Systems",
    levels: &[
        Level {
            level: "E0",
            model: "ideal",
            class: "ExternalAhrs",
            description: "attitude, rates, specific force, position and velocity: truth, continuous",
            parameters: &[("gravity", Shape::Positive)],
        },
        Level {
            level: "E1",
            model: "sampled",
            class: "SampledExternalAhrs",
            description: "at the output rate; attitude and heading noise and drift, gyro/accel noise and bias drift, position and velocity errors",
            parameters: sampled![
                ("gravity", Shape::Positive),
                ("attitudeNoiseStd", Shape::Nonnegative),
                ("attitudeDriftStd", Shape::Nonnegative),
                ("headingNoiseStd", Shape::Nonnegative),
                ("headingDriftStd", Shape::Nonnegative),
                ("attitudeDriftTau", Shape::Positive),
                ("gyroNoiseStd", Shape::Nonnegative),
                ("gyroBiasStd", Shape::Nonnegative),
                ("accelNoiseStd", Shape::Nonnegative),
                ("accelBiasStd", Shape::Nonnegative),
                ("biasTau", Shape::Positive),
                ("positionNoiseStd", Shape::Nonnegative),
                ("verticalNoiseStd", Shape::Nonnegative),
                ("positionDriftStd", Shape::Nonnegative),
                ("verticalDriftStd", Shape::Nonnegative),
                ("positionDriftTau", Shape::Positive),
                ("velocityNoiseStd", Shape::Nonnegative),
            ],
        },
    ],
    outputs: &[
        ("ahrs_quaternion", "quaternion", 4),
        ("ahrs_rate_frd", "rate", 3),
        ("ahrs_accel_frd", "accel", 3),
        ("ahrs_position_ned", "position", 3),
        ("ahrs_velocity_ned", "velocity", 3),
        ("ahrs_valid", "validity", 0),
    ],
};

pub const VISUAL_ODOMETRY: Kind = Kind {
    name: "visual_odometry",
    title: "Visual odometry",
    component: "visualOdometry",
    capability: "visual_odometry_v1",
    mounted: true,
    ardupilot: "Non-GPS navigation (visual odometry)",
    levels: &[
        Level {
            level: "V0",
            model: "ideal",
            class: "VisualOdometry",
            description: "camera pose and velocity in the odometry frame: truth, continuous",
            parameters: &[],
        },
        Level {
            level: "V1",
            model: "sampled",
            class: "SampledVisualOdometry",
            description: "at the output rate; drift growing with the distance travelled, heading walk, noise, latency",
            parameters: sampled![
                ("driftFraction", Shape::Nonnegative),
                ("driftTau", Shape::Positive),
                ("headingWalk", Shape::Nonnegative),
                ("positionNoiseStd", Shape::Nonnegative),
                ("velocityNoiseStd", Shape::Nonnegative),
                ("attitudeNoiseStd", Shape::Nonnegative),
            ],
        },
    ],
    outputs: &[
        ("odometry_position_ned", "position", 3),
        ("odometry_velocity_ned", "velocity", 3),
        ("odometry_quaternion", "quaternion", 4),
        ("odometry_valid", "validity", 0),
    ],
};

pub const BEACON: Kind = Kind {
    name: "beacon",
    title: "Beacon positioning",
    component: "beaconTag",
    capability: "beacon_v1",
    mounted: true,
    ardupilot: "Non-GPS navigation (beacons)",
    levels: &[
        Level {
            level: "U0",
            model: "ideal",
            class: "Beacon",
            description: "tag position in the beacon frame: truth, continuous",
            parameters: &[],
        },
        Level {
            level: "U1",
            model: "sampled",
            class: "SampledBeacon",
            description: "at the update rate; horizontal and vertical noise, resolution, latency",
            parameters: sampled![
                ("horizontalNoiseStd", Shape::Nonnegative),
                ("verticalNoiseStd", Shape::Nonnegative),
                ("quantum", Shape::Nonnegative),
            ],
        },
    ],
    outputs: &[("beacon_position_ned", "position", 3), ("beacon_valid", "validity", 0)],
};

pub const PRECISION_LANDING: Kind = Kind {
    name: "precision_landing",
    title: "Precision landing sensor",
    component: "landingCamera",
    capability: "precision_landing_v1",
    mounted: true,
    ardupilot: "Precision Landing and Loiter",
    levels: &[
        Level {
            level: "K0",
            model: "ideal",
            class: "PrecisionLanding",
            description: "line-of-sight angles to the target inside the field of view and range, continuous",
            parameters: &[
                ("targetPosition_w", Shape::Vec3),
                ("fieldOfViewX", Shape::Positive),
                ("fieldOfViewY", Shape::Positive),
                ("maxRange", Shape::Positive),
            ],
        },
        Level {
            level: "K1",
            model: "sampled",
            class: "SampledPrecisionLanding",
            description: "per frame; angle noise and pixel resolution, distance noise, latency",
            parameters: sampled![
                ("targetPosition_w", Shape::Vec3),
                ("fieldOfViewX", Shape::Positive),
                ("fieldOfViewY", Shape::Positive),
                ("maxRange", Shape::Positive),
                ("angleNoiseStd", Shape::Nonnegative),
                ("angleQuantum", Shape::Nonnegative),
                ("distanceNoiseStd", Shape::Nonnegative),
            ],
        },
    ],
    outputs: &[
        ("landing_target_angle", "angle", 2),
        ("landing_target_distance_m", "distance", 0),
        ("landing_target_valid", "validity", 0),
    ],
};

pub const KINDS: [&Kind; 10] = [
    &RANGEFINDER,
    &AIRSPEED,
    &OPTICAL_FLOW,
    &RPM,
    &POWER_MODULE,
    &TEMPERATURE,
    &EXTERNAL_AHRS,
    &VISUAL_ODOMETRY,
    &BEACON,
    &PRECISION_LANDING,
];

pub fn kind(name: &str) -> Option<&'static Kind> {
    KINDS.iter().copied().find(|k| k.name == name)
}

impl Kind {
    pub fn level(&self, wanted: &str) -> Option<&'static Level> {
        self.levels
            .iter()
            .find(|l| l.model == wanted || l.level.eq_ignore_ascii_case(wanted))
    }
    /// The keys a level accepts besides its parameters.
    fn extra(&self) -> Vec<&'static str> {
        let mut keys = vec!["model", "level"];
        if self.mounted {
            keys.extend(["position", "orientation"]);
        }
        if self.name == "power_module" {
            keys.extend(["battery", "supply", "smart"]);
        }
        if self.name == "precision_landing" {
            keys.push("target");
        }
        keys
    }
    pub fn keys(&self, level: &Level) -> Vec<&'static str> {
        let mut keys = self.extra();
        keys.extend(level.parameters.iter().map(|p| p.0));
        keys
    }
}

/// Orientations of a device's boresight (+z) in body FRD, as R_bs (rows).
pub const ORIENTATIONS: [(&str, [[f64; 3]; 3]); 6] = [
    ("down", [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]),
    ("forward", [[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]]),
    ("back", [[0., 0., -1.], [0., 1., 0.], [1., 0., 0.]]),
    ("right", [[1., 0., 0.], [0., 0., 1.], [0., -1., 0.]]),
    ("left", [[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]]),
    ("up", [[1., 0., 0.], [0., -1., 0.], [0., 0., -1.]]),
];

fn check(label: &str, value: &Toml, shape: Shape) -> Result<()> {
    match shape {
        Shape::Real => {
            number(value, label, false, false)?;
        }
        Shape::Positive => {
            number(value, label, true, false)?;
        }
        Shape::Nonnegative => {
            number(value, label, false, true)?;
        }
        Shape::Fraction => {
            let x = number(value, label, false, true)?;
            if x > 1.0 {
                return Err(format!("{label}: expected a fraction in 0..1"));
            }
        }
        Shape::Count => {
            integer(value, label, 0)?;
        }
        Shape::Seed => {
            let n = integer(value, label, 1)?;
            if n > i32::MAX as usize {
                return Err(format!("{label}: exceeds {}", i32::MAX));
            }
        }
        Shape::Vec3 => array(value, &[3], label)?,
        Shape::Vector => {
            let n = value.as_array().map(Vec::len).unwrap_or(0);
            if n < 2 {
                return Err(format!("{label}: expected an array of at least 2 numbers"));
            }
            array(value, &[n], label)?;
        }
    }
    Ok(())
}

/// `[peripherals]`, checked: per kind the selected `model` and its keys.
pub fn resolve(raw: &Toml, vehicle: &str) -> Result<Table> {
    let names: Vec<&str> = KINDS.iter().map(|k| k.name).collect();
    let t = table(raw, &names, "peripherals")?;
    let mut out = Table::new();
    for (name, sel) in t {
        let kind = kind(name).unwrap();
        let label = format!("peripherals.{name}");
        let sel = sel
            .as_table()
            .ok_or_else(|| format!("{label}: expected a table"))?;
        let wanted = sel
            .get("model")
            .or_else(|| sel.get("level"))
            .map(|v| v.as_str().ok_or(format!("{label}.model: expected a string")))
            .transpose()?
            .unwrap_or(kind.levels[0].model);
        let level = kind.level(wanted).ok_or_else(|| {
            format!(
                "{label}: no level {wanted:?}; {}",
                kind.levels
                    .iter()
                    .map(|l| format!("{} ({})", l.level, l.model))
                    .collect::<Vec<_>>()
                    .join(", ")
            )
        })?;
        let keys = kind.keys(level);
        table(&Toml::Table(sel.clone()), &keys, &label)
            .map_err(|e| format!("{e} (level {} {})", level.level, level.model))?;
        if name == "rpm" && vehicle == "fixed_wing" {
            return Err(format!(
                "{label}: a fixed-wing exports no shaft speed (its thrust model has none)"
            ));
        }
        let mut resolved = Table::new();
        resolved.insert("model".into(), Toml::String(level.model.into()));
        for (k, v) in sel {
            let label = format!("{label}.{k}");
            match k.as_str() {
                "model" | "level" => continue,
                "position" | "target" => array(v, &[3], &label)?,
                "orientation" => match v {
                    Toml::String(s) => {
                        if !ORIENTATIONS.iter().any(|(n, _)| n == s) {
                            return Err(format!(
                                "{label}: {s:?}; one of {}",
                                ORIENTATIONS.map(|(n, _)| n).join(", ")
                            ));
                        }
                    }
                    other => crate::config::rotation(other, &label)?,
                },
                "supply" | "smart" => {
                    v.as_bool()
                        .ok_or_else(|| format!("{label}: expected true or false"))?;
                }
                "battery" => {
                    let b = table(v, &BATTERY.iter().map(|p| p.0).collect::<Vec<_>>(), &label)?;
                    for (bk, bv) in b {
                        let shape = BATTERY.iter().find(|p| p.0 == bk).unwrap().1;
                        check(&format!("{label}.{bk}"), bv, shape)?;
                    }
                    let n = |key: &str| b.get(key).and_then(Toml::as_array).map(Vec::len);
                    if n("socPoints").is_some() != n("cellVoltagePoints").is_some()
                        || n("socPoints") != n("cellVoltagePoints")
                    {
                        return Err(format!(
                            "{label}: socPoints and cellVoltagePoints go together, of equal length"
                        ));
                    }
                }
                _ => {
                    let shape = level.parameters.iter().find(|p| p.0 == k).unwrap().1;
                    check(&label, v, shape)?;
                }
            }
            resolved.insert(k.clone(), v.clone());
        }
        out.insert(name.clone(), Toml::Table(resolved));
    }
    Ok(out)
}

fn reference(name: &str) -> Value {
    Value::Reference(name.into())
}

fn literal(v: &Toml) -> Result<Value> {
    crate::lower::literal(v)
}

fn matrix(m: &[[f64; 3]; 3]) -> Value {
    Value::Array(
        m.iter()
            .map(|r| Value::Array(r.iter().map(|x| Value::Real(*x)).collect()))
            .collect(),
    )
}

/// The power module's pack drives the propulsion supply: multirotor P1-P4 with `supply` (default true).
pub fn supplies_propulsion(c: &ResolvedFireConfig) -> bool {
    c.vehicle == "multirotor"
        && c.voltage_driven()
        && c.peripherals
            .get("power_module")
            .and_then(Toml::as_table)
            .is_some_and(|t| t.get("supply").and_then(Toml::as_bool).unwrap_or(true))
}

/// Shafts of the vehicle: (count, the composed model's speed vector).
fn shafts(c: &ResolvedFireConfig) -> Result<(usize, &'static str)> {
    match c.vehicle.as_str() {
        "multirotor" => Ok((
            c.geometry["nRotors"].as_integer().unwrap() as usize,
            "rotorSpeed",
        )),
        "rover" => Ok((2, "wheelSpeed")),
        other => Err(format!("peripherals.rpm: a {other} exports no shaft speed")),
    }
}

/// The selected peripherals as components of the composed model, with their
/// inputs and the exported readings.
pub fn components(
    c: &ResolvedFireConfig,
    components: &mut Vec<Instance>,
    equalities: &mut Vec<(String, Value)>,
) -> Result<()> {
    for kind in KINDS {
        let Some(sel) = c.peripherals.get(kind.name).and_then(Toml::as_table) else {
            continue;
        };
        let level = kind
            .level(sel["model"].as_str().unwrap())
            .expect("resolved");
        let mut modifiers = Vec::new();
        let mut shaft_count = 0;
        for (k, v) in sel {
            match k.as_str() {
                "model" | "supply" | "smart" => {}
                "position" => modifiers.push(Modifier::Bind("position_b".into(), literal(v)?)),
                "target" => {
                    modifiers.push(Modifier::Bind("targetPosition_w".into(), literal(v)?))
                }
                "orientation" => {
                    let m = match v.as_str() {
                        Some(name) => {
                            matrix(&ORIENTATIONS.iter().find(|(n, _)| *n == name).unwrap().1)
                        }
                        None => literal(v)?,
                    };
                    modifiers.push(Modifier::Bind("R_bs".into(), m));
                }
                "battery" => {
                    let fields = v
                        .as_table()
                        .unwrap()
                        .iter()
                        .map(|(bk, bv)| Ok(Modifier::Bind(bk.clone(), literal(bv)?)))
                        .collect::<Result<Vec<_>>>()?;
                    modifiers.push(Modifier::Nested("battery".into(), fields));
                }
                _ => modifiers.push(Modifier::Bind(k.clone(), literal(v)?)),
            }
        }
        match kind.name {
            "rpm" => {
                let (n, speed) = shafts(c)?;
                shaft_count = n;
                modifiers.push(Modifier::Bind("n".into(), Value::Integer(n as i64)));
                modifiers.push(Modifier::Bind("speed".into(), reference(speed)));
            }
            "power_module" => power_inputs(c, &mut modifiers),
            _ => {}
        }
        components.push(Instance {
            class: format!("{P}{}", level.class),
            name: kind.component.into(),
            modifiers,
            dimensions: vec![],
            prefix: Prefix::Component,
            binding: None,
        });
        if kind.mounted {
            equalities.push((format!("{}.truth", kind.component), reference("truth")));
        }
        let extra = LEVEL_OUTPUTS
            .iter()
            .filter(|(k, l, _)| *k == kind.name && *l == level.level)
            .flat_map(|(_, _, o)| o.iter());
        for (export, field, dim) in kind.outputs.iter().chain(extra) {
            let dims = match *dim {
                0 => vec![],
                PER_SHAFT => vec![shaft_count],
                PER_SHAFT_BUFFER => vec![shaft_count
                    * sel.get("bufferSize").and_then(Toml::as_integer).unwrap_or(32) as usize],
                n => vec![n],
            };
            components.push(Instance {
                class: "Real".into(),
                name: (*export).into(),
                modifiers: vec![],
                dimensions: dims,
                prefix: Prefix::Output,
                binding: Some(reference(&format!("{}.{field}", kind.component))),
            });
        }
    }
    Ok(())
}

/// What the power module's pack supplies, per vehicle and propulsion level.
fn power_inputs(c: &ResolvedFireConfig, m: &mut Vec<Modifier>) {
    match c.vehicle.as_str() {
        "multirotor" => {
            let nr = c.geometry["nRotors"].as_integer().unwrap();
            if c.three_phase() {
                m.push(Modifier::Bind(
                    "propulsionCurrent".into(),
                    Value::Array(vec![reference("totalDcCurrent")]),
                ));
            } else if c.electromechanical() {
                m.push(Modifier::Bind("nCurrents".into(), Value::Integer(nr)));
                m.push(Modifier::Bind(
                    "propulsionCurrent".into(),
                    reference("dcCurrent"),
                ));
            } else {
                m.push(Modifier::Bind("nShafts".into(), Value::Integer(nr)));
                m.push(Modifier::Bind(
                    "shaftTorque".into(),
                    reference("rotors.loadTorque"),
                ));
                m.push(Modifier::Bind("shaftSpeed".into(), reference("motors.omega")));
            }
        }
        "rover" => m.push(Modifier::Bind(
            "propulsionCurrent".into(),
            Value::Array(vec![reference("plant.motorCurrent")]),
        )),
        _ => {
            m.push(Modifier::Bind("thrust".into(), reference("plant.thrust")));
            m.push(Modifier::Bind("airspeed".into(), reference("plant.airspeed")));
        }
    }
}

/// What the composer offers: every kind, its levels and keys.
pub fn catalog() -> Json {
    Json::Array(
        KINDS
            .iter()
            .map(|k| {
                json!({
                    "name": k.name, "title": k.title, "capability": k.capability,
                    "mounted": k.mounted, "ardupilot": k.ardupilot,
                    "outputs": k.outputs.iter().map(|o| o.0).collect::<Vec<_>>(),
                    "levels": k.levels.iter().map(|l| json!({
                        "level": l.level, "model": l.model, "description": l.description,
                        "keys": k.keys(l),
                    })).collect::<Vec<_>>(),
                    "orientations": if k.mounted { ORIENTATIONS.map(|(n, _)| n).to_vec() } else { vec![] },
                })
            })
            .collect(),
    )
}

// --- real parts ---------------------------------------------------------------

/// One parameter a part's figures produced.
struct Entry {
    key: &'static str,
    value: Toml,
    from: Vec<&'static str>,
    how: String,
    /// Evidence class (parts::EVIDENCE), where the converter classifies it.
    class: Option<&'static str>,
}

#[derive(Default)]
struct Conversion {
    entries: Vec<Entry>,
    assumed: Vec<String>,
    missing: Vec<(&'static str, &'static str)>,
    /// Figures that go into no parameter, and why.
    unused: Vec<(String, String)>,
}

struct Figures<'a> {
    part: &'a Table,
    explicit: &'a Table,
}

impl Figures<'_> {
    fn f(&self, key: &str) -> Option<f64> {
        let v = self.part.get(key)?;
        v.as_float()
            .or_else(|| v.as_integer().map(|n| n as f64))
            .filter(|x| x.is_finite())
    }
    /// The first of several figures stated, and its key.
    fn any(&self, keys: &[&'static str]) -> Option<(f64, &'static str)> {
        keys.iter().find_map(|k| self.f(k).map(|v| (v, *k)))
    }
    /// A number, or the n-th element of an array (a range given as [min, max]).
    fn at(&self, key: &str, n: usize) -> Option<f64> {
        self.f(key)
            .or_else(|| {
                let v = self.part.get(key)?.as_array()?.get(n)?;
                v.as_float().or_else(|| v.as_integer().map(|n| n as f64))
            })
            .filter(|x| x.is_finite())
    }
    /// The sample period: explicit, else from the first rate figure stated.
    fn period(&self, c: &mut Conversion, rates: &[&'static str]) -> f64 {
        if let Some(p) = self.explicit.get("samplePeriod") {
            return p
                .as_float()
                .or_else(|| p.as_integer().map(|n| n as f64))
                .unwrap_or(0.02);
        }
        for r in rates {
            // A rate, or the top of a stated range of rates.
            let hz = self.f(r).or_else(|| {
                let n = self.part.get(*r)?.as_array()?.len();
                self.at(r, n.saturating_sub(1))
            });
            if let Some(hz) = hz.filter(|hz| *hz > 0.0) {
                if hz > MAX_RATE_HZ {
                    c.put("samplePeriod", 1.0 / MAX_RATE_HZ, &[r],
                          format!("{hz} Hz, capped at {MAX_RATE_HZ} Hz"));
                    c.assumed.push(format!(
                        "{MAX_RATE_HZ} Hz output: the autopilot's drivers read no faster; the device's {hz} Hz is not simulated"
                    ));
                    return 1.0 / MAX_RATE_HZ;
                }
                c.put("samplePeriod", 1.0 / hz, &[r], format!("1 / {hz} Hz"));
                return 1.0 / hz;
            }
        }
        if let Some(r) = rates.first() {
            c.missing.push((r, "samplePeriod"));
        }
        0.02
    }
}

impl Conversion {
    fn put(&mut self, key: &'static str, value: f64, from: &[&'static str], how: impl Into<String>) {
        self.entries.push(Entry {
            key,
            value: Toml::Float(value),
            from: from.to_vec(),
            how: how.into(),
            class: None,
        });
    }
    /// `put` with an evidence class.
    fn put_as(&mut self, class: &'static str, key: &'static str, value: f64, from: &[&'static str], how: impl Into<String>) {
        self.put(key, value, from, how);
        self.entries.last_mut().unwrap().class = Some(class);
    }
    fn put_value(&mut self, key: &'static str, value: Toml, from: &[&'static str], how: impl Into<String>) {
        self.entries.push(Entry {
            key,
            value,
            from: from.to_vec(),
            how: how.into(),
            class: None,
        });
    }
    /// A `±` accuracy figure as a 2-sigma bound.
    fn sigma(&mut self, x: &Figures, figure: &'static str, key: &'static str, scale: f64, unit: &str) -> Option<f64> {
        match x.f(figure) {
            Some(v) => {
                let s = v * scale / 2.0;
                self.put(key, s, &[figure], format!("±{v}{unit} taken as 2 sigma"));
                Some(s)
            }
            None => {
                self.missing.push((figure, key));
                None
            }
        }
    }
}

const DEG: f64 = std::f64::consts::PI / 180.0;
/// Fastest output a part converts to (the SITL drivers poll at 50-100 Hz).
const MAX_RATE_HZ: f64 = 100.0;

fn rangefinder(x: &Figures, c: &mut Conversion) {
    if let Some(v) = x.f("range_min_m") {
        c.put("minRange", v, &["range_min_m"], "minimum range");
    } else {
        c.missing.push(("range_min_m", "minRange"));
    }
    if let Some(v) = x.f("range_max_m") {
        c.put("maxRange", v, &["range_max_m"], "maximum range (the datasheet's target and conditions)");
    } else {
        c.missing.push(("range_max_m", "maxRange"));
    }
    let period = x.period(c, &["update_rate_hz", "update_rate_max_hz"]);
    if let Some(r) = x.f("repeatability_1sigma_m") {
        c.put("noiseStd", r, &["repeatability_1sigma_m"], "1-sigma repeatability");
        if x.f("accuracy_m").is_some() {
            c.unused.push(("accuracy_m".into(), "an accuracy bound: the noise is the stated repeatability".into()));
        }
    } else if x.f("accuracy_m").is_some() {
        c.sigma(x, "accuracy_m", "noiseStd", 1.0, " m");
    }
    if let Some(ms) = x.f("latency_max_ms") {
        let n = (ms / 1000.0 / period).round();
        c.put_value("latencySamples", Toml::Integer(n as i64), &["latency_max_ms"], format!("{ms} ms at the output rate"));
    }
    if let Some(p) = x.f("accuracy_percent") {
        c.put("noiseSlope", p / 100.0 / 2.0, &["accuracy_percent"], format!("±{p} % of range taken as 2 sigma"));
    }
    if let Some(bands) = x.part.get("accuracy_bands").and_then(Toml::as_array) {
        // [[from_m, to_m, value], ...]: the first band's value; a value below 1 is metres, else percent.
        if let Some(first) = bands.first().and_then(Toml::as_array) {
            let v = first.get(2).and_then(|v| v.as_float().or(v.as_integer().map(|n| n as f64)));
            if let Some(v) = v.filter(|_| x.f("accuracy_m").is_none() && x.f("repeatability_1sigma_m").is_none()) {
                c.put("noiseStd", v / 2.0, &["accuracy_bands"], format!("first band ±{v} m taken as 2 sigma"));
                c.unused.push(("accuracy_bands".into(), "later bands: one noise level and a slope only".into()));
            }
        }
    }
    if x.f("accuracy_m").is_none() && x.f("accuracy_percent").is_none() && x.part.get("accuracy_bands").is_none() {
        c.missing.push(("accuracy_m", "noiseStd"));
    }
    if let Some(v) = x.f("resolution_m") {
        c.put("quantum", v, &["resolution_m"], "resolution");
    }
    for f in ["fov_deg", "beam_divergence_deg"] {
        if x.part.contains_key(f) {
            c.unused.push((f.into(), "the ray is a line: no beam width".into()));
        }
    }
}

fn airspeed(x: &Figures, c: &mut Conversion) {
    let span = match (x.at("pressure_range_pa", 0), x.at("pressure_range_pa", 1)) {
        (Some(a), Some(b)) if x.part.get("pressure_range_pa").and_then(Toml::as_array).is_some() => {
            c.put("fullScale", a.abs().max(b.abs()), &["pressure_range_pa"], "the larger end of the range");
            Some((b - a).abs())
        }
        (Some(a), _) => {
            c.put("fullScale", a.abs(), &["pressure_range_pa"], "± range");
            Some(2.0 * a.abs())
        }
        _ => {
            c.missing.push(("pressure_range_pa", "fullScale"));
            None
        }
    };
    x.period(c, &["update_rate_hz", "output_rate_hz"]);
    if let Some(sf) = x.f("pressure_scale_factor_per_pa") {
        c.put("pressureQuantum", 1.0 / sf, &["pressure_scale_factor_per_pa"], format!("one count: 1 / {sf} Pa"));
    }
    if let Some(v) = x.f("zero_point_accuracy_pa") {
        c.put("pressureDriftStd", v / 2.0, &["zero_point_accuracy_pa"], format!("±{v} Pa zero point taken as 2 sigma"));
        c.assumed.push("pressureDriftTau 300 s (the zero-point accuracy as a slow drift)".into());
    }
    if let Some(v) = x.f("noise_pa") {
        c.put("pressureNoiseStd", v, &["noise_pa"], "RMS noise");
    } else {
        c.missing.push(("noise_pa", "pressureNoiseStd"));
    }
    if let Some(v) = x.f("pressure_resolution_pa") {
        c.put("pressureQuantum", v, &["pressure_resolution_pa"], "resolution");
    } else if x.f("pressure_scale_factor_per_pa").is_some() {
    } else if let (Some(counts), Some(span)) = (
        x.f("output_span_counts").or_else(|| Some(x.f("output_counts_at_pmax")? - x.f("output_counts_at_pmin")?)),
        span,
    ) {
        c.put("pressureQuantum", span / counts, &["output_span_counts", "output_counts_at_pmax", "pressure_range_pa"],
              format!("the {span} Pa span over its {counts} output counts"));
    } else if let (Some(bits), Some(span)) = (x.f("resolution_bits"), span) {
        c.put("pressureQuantum", span / 2f64.powf(bits), &["resolution_bits", "pressure_range_pa"],
              format!("{span} Pa over 2^{bits}"));
    }
    if x.f("zero_point_accuracy_pa").is_some() {
    } else if x.f("accuracy_pa").is_some() {
        c.sigma(x, "accuracy_pa", "pressureDriftStd", 1.0, " Pa");
        c.assumed.push("pressureDriftTau 300 s (the accuracy as a slow drift)".into());
    } else if let (Some((pct, key)), Some(span)) = (
        x.any(&["accuracy_percent_span", "total_error_band_typ_percent_span", "total_error_band_percent_span"]),
        span,
    ) {
        c.put("pressureDriftStd", pct / 100.0 * span / 2.0, &[key, "pressure_range_pa"],
              format!("±{pct} % of the {span} Pa span taken as 2 sigma"));
        c.assumed.push("pressureDriftTau 300 s (the error band as a slow drift; the autopilot zeroes the offset at boot)".into());
    }
    if let Some(v) = x.f("temperature_resolution_c") {
        c.put("temperatureQuantum", v, &["temperature_resolution_c"], "resolution");
    }
}

fn optical_flow(x: &Figures, c: &mut Conversion) {
    let period = x.period(c, &["frame_rate_hz", "update_rate_hz"]);
    if let Some(v) = x.f("min_height_m").or(x.at("working_range_m", 0)) {
        c.put("minRange", v, &["min_height_m", "working_range_m"], "closest working distance");
    }
    if let Some(v) = x.f("max_height_m").or(x.at("working_range_m", 1)).or(x.f("range_sensor_max_m")) {
        c.put("maxRange", v, &["working_range_m", "range_sensor_max_m"], "farthest working distance");
    }
    if let Some(v) = x.f("max_speed_rad_s") {
        c.put("maxFlowRate", v, &["max_speed_rad_s"], "maximum flow rate");
    }
    let fov = x.f("fov_deg");
    let px = x.at("resolution_px", 0);
    if let (Some(fov), Some(px)) = (fov, px) {
        let q = fov * DEG / px / period;
        c.put("flowQuantum", q, &["fov_deg", "resolution_px"],
              format!("one pixel ({fov}° / {px}) per frame of {period} s"));
    } else {
        c.missing.push(("fov_deg", "flowQuantum"));
    }
}

fn rpm(x: &Figures, c: &mut Conversion) {
    let ppr = x.f("pulses_per_rev");
    if let Some(p) = ppr {
        c.put("pulsesPerRevolution", p, &["pulses_per_rev"], "pulses per revolution");
    }
    if let Some(r) = x.at("rpm_range", 1) {
        c.put("maxRpm", r, &["rpm_range"], "top of the rpm range");
    }
    if let Some(f) = x.f("max_frequency_hz") {
        let p = ppr.unwrap_or(1.0);
        c.put("maxRpm", f * 60.0 / p, &["max_frequency_hz"], format!("{f} Hz at {p} pulse(s) per revolution"));
    }
    if ppr.is_none() {
        c.assumed.push("pulses per revolution: set by the magnets or marks (configuration)".into());
    }
    // R2: edge timing. A switching delay or jitter is rarely stated; rise and
    // fall times bound the edge's sharpness, not its delay.
    if x.f("edge_delay_us").is_none() {
        c.assumed.push("R2: edge delay and jitter 0 (not stated; rise/fall times are not delays)".into());
    }
}

fn power_module(x: &Figures, c: &mut Conversion) {
    x.period(c, &["update_rate_hz"]);
    if let Some((v, key)) = x.any(&["input_voltage_max_v", "max_voltage_v"]) {
        c.put("maxVoltage", v, &[key], "highest input voltage");
    } else if let Some(cells) = x.at("input_voltage_range_cells", 1) {
        c.put("maxVoltage", cells * 4.2, &["input_voltage_range_cells"], format!("{cells} cells at 4.2 V"));
    }
    if let Some(v) = x.f("max_current_a").or(x.f("max_current_continuous_a")) {
        c.put("maxCurrent", v, &["max_current_a"], "current range");
    }
    if let Some(p) = x.f("current_accuracy_percent") {
        c.put("currentNoiseStd", 0.0, &[], "");
        c.entries.pop();
        c.unused.push(("current_accuracy_percent".into(),
            format!("a gain tolerance (±{p} %): set currentScale to apply one")));
    }
    if let Some(p) = x.f("voltage_accuracy_percent") {
        c.unused.push(("voltage_accuracy_percent".into(),
            format!("a gain tolerance (±{p} %): set voltageScale to apply one")));
    }
    let ina = x.part.get("ina226").and_then(Toml::as_table);
    if let Some(ina) = ina {
        let g = |k: &str| ina.get(k).and_then(|v| v.as_float().or(v.as_integer().map(|n| n as f64)));
        if let Some(lsb) = g("bus_voltage_lsb_mv") {
            c.put("voltageQuantum", lsb / 1000.0, &["ina226"], format!("INA226 bus-voltage LSB {lsb} mV"));
        }
        if let (Some(bits), Some(i)) = (g("adc_bits"), x.f("max_current_sensing_a_lv")) {
            c.put("currentQuantum", i / 2f64.powf(bits - 1.0), &["ina226", "max_current_sensing_a_lv"],
                  format!("{i} A sensing range over 2^{} (INA226, signed)", bits - 1.0));
        }
        c.unused.push(("ina228".into(), "the INA226 version is modelled".into()));
    } else if let Some(bits) = x.f("adc_bits") {
        if let Some(i) = x.f("max_current_a") {
            c.put("currentQuantum", i / 2f64.powf(bits - 1.0), &["adc_bits", "max_current_a"],
                  format!("{i} A over 2^{} (signed)", bits - 1.0));
        }
        if let Some(v) = x.at("input_voltage_range_v", 1) {
            c.put("voltageQuantum", v / 2f64.powf(bits - 1.0), &["adc_bits", "input_voltage_range_v"],
                  format!("{v} V over 2^{}", bits - 1.0));
        }
    } else {
        // An analog module, read by the autopilot's ADC.
        let vref = x.f("adc_reference_v").unwrap_or(3.3);
        let mut adc = false;
        if let Some((s, key)) = x.any(&["current_scale", "current_scale_a_per_v"]) {
            c.put("currentQuantum", s * vref / 4096.0, &[key], format!("{s} A/V through a 12-bit {vref} V ADC"));
            adc = true;
        }
        if let Some((s, key)) = x.any(&["voltage_scale", "voltage_divider_ratio"]) {
            c.put("voltageQuantum", s * vref / 4096.0, &[key], format!("divider {s} through a 12-bit {vref} V ADC"));
            adc = true;
        }
        if adc {
            c.assumed.push("the autopilot's ADC: 12 bits".into());
        }
        // W2: the module's analog transfer functions, and the autopilot's
        // scaling as its setup page gives it.
        if let Some((s, key)) = x.any(&["voltage_scale", "voltage_divider_ratio"]) {
            c.put_as("derived", "voltageGain", 1.0 / s, &[key], format!("1 / {s} (the divider)"));
            c.put_as("specified", "voltageMultiplier", s, &[key], "the autopilot's voltage multiplier (BATT_VOLT_MULT)");
        }
        if let Some((s, key)) = x.any(&["current_scale", "current_scale_a_per_v"]) {
            c.put_as("derived", "currentGain", 1.0 / s, &[key], format!("1 / {s} A/V"));
            c.put_as("specified", "ampsPerVolt", s, &[key], "the autopilot's amps per volt (BATT_AMP_PERVLT)");
        }
        if x.f("adc_reference_v").is_some() {
            c.put_as("specified", "outputMax", vref, &["adc_reference_v"], "outputs designed for this ADC range");
            c.put_as("specified", "adcReference", vref, &["adc_reference_v"], "ADC full scale");
        }
        c.put_value("adcBits", Toml::Integer(12), &[], "the autopilot's ADC: 12 bits");
        c.entries.last_mut().unwrap().class = Some("assumed");
        if x.f("zero_current_offset_v").is_none() {
            c.assumed.push("W2: zero-current and zero-voltage offsets 0 (not stated)".into());
        }
    }
}

fn temperature(x: &Figures, c: &mut Conversion) {
    x.period(c, &["update_rate_hz", "update_rate_max_hz"]);
    if let Some(ms) = x.f("conversion_time_ms") {
        if x.explicit.get("samplePeriod").is_none() && !c.entries.iter().any(|e| e.key == "samplePeriod") {
            c.put("samplePeriod", (ms / 1000.0).max(0.1), &["conversion_time_ms"],
                  format!("10 Hz, or the {ms} ms conversion if slower"));
        }
    }
    if x.f("accuracy_c").is_some() {
        c.sigma(x, "accuracy_c", "noiseStd", 1.0, " °C");
        c.assumed.push("the accuracy as sample noise (no fixed offset is drawn)".into());
    }
    if let Some(v) = x.f("resolution_c") {
        c.put("quantum", v, &["resolution_c"], "resolution");
    }
    if let Some(v) = x.f("self_heating_c") {
        c.put("selfHeating", v, &["self_heating_c"], "self-heating");
    }
    if let Some((v, key)) = x.any(&["time_constant_air_s", "response_time_s"]) {
        c.put("timeConstant", v, &[key], "thermal response as the time constant");
    } else {
        c.assumed.push("timeConstant 1 s (the element's thermal response is not stated)".into());
    }
}

fn external_ahrs(x: &Figures, c: &mut Conversion) {
    let period = x.period(c, &["output_rate_hz_max", "output_rate_hz"]);
    if let Some((v, key)) = x.any(&["pitch_roll_accuracy_dynamic_deg", "pitch_roll_accuracy_deg", "pitch_roll_accuracy_static_deg"]) {
        c.put("attitudeDriftStd", v * DEG, &[key], format!("{v}° RMS as a Gauss-Markov error"));
    }
    if let Some((v, key)) = x.any(&["heading_accuracy_ins_dynamic_deg", "heading_accuracy_dynamic_gnss_deg",
                                    "heading_accuracy_gnss_deg", "heading_accuracy_deg", "heading_accuracy_magnetic_deg",
                                    "heading_accuracy_static_deg"]) {
        c.put("headingDriftStd", v * DEG, &[key], format!("{v}° RMS as a Gauss-Markov error"));
    }
    if let Some(v) = x.f("gyro_bias_instability_deg_per_hr") {
        c.put("gyroBiasStd", v * DEG / 3600.0, &["gyro_bias_instability_deg_per_hr"], format!("{v} °/h"));
    }
    let bw = (0.5 / period).sqrt();
    if let Some(v) = x.f("gyro_noise_density_dps_per_rthz") {
        c.put("gyroNoiseStd", v * DEG * bw, &["gyro_noise_density_dps_per_rthz"],
              format!("{v} °/s/√Hz over the {:.0} Hz output bandwidth", 0.5 / period));
    } else if let Some(v) = x.f("gyro_noise_density_deg_per_hr_per_rthz") {
        c.put("gyroNoiseStd", v / 3600.0 * DEG * bw, &["gyro_noise_density_deg_per_hr_per_rthz"],
              format!("{v} °/h/√Hz over the {:.0} Hz output bandwidth", 0.5 / period));
    } else if let Some((v, key)) = x.any(&["gyro_arw_deg_per_rthr", "gyro_random_walk_deg_per_rthr"]) {
        c.put("gyroNoiseStd", v / 60.0 * DEG * bw, &[key],
              format!("{v} °/√h angle random walk = {} °/s/√Hz over the output bandwidth", v / 60.0));
    }
    if let Some(v) = x.f("accel_noise_density_ug_per_rthz") {
        c.put("accelNoiseStd", v * 1e-6 * 9.80665 * bw, &["accel_noise_density_ug_per_rthz"], format!("{v} µg/√Hz"));
    } else if let Some(v) = x.f("accel_noise_density_mg_per_rthz") {
        c.put("accelNoiseStd", v * 1e-3 * 9.80665 * bw, &["accel_noise_density_mg_per_rthz"], format!("{v} mg/√Hz"));
    }
    if let Some(v) = x.f("accel_bias_instability_ug") {
        c.put("accelBiasStd", v * 1e-6 * 9.80665, &["accel_bias_instability_ug"], format!("{v} µg"));
    } else if let Some(v) = x.at("accel_bias_instability_mg", 0) {
        c.put("accelBiasStd", v * 1e-3 * 9.80665, &["accel_bias_instability_mg"], format!("{v} mg (the lowest range's)"));
    }
    if let Some((v, key)) = x.any(&["position_accuracy_horizontal_m", "position_accuracy_m"]) {
        c.put("positionDriftStd", v, &[key], format!("{v} m RMS as a Gauss-Markov error"));
    }
    if let Some(v) = x.f("position_accuracy_vertical_m") {
        c.put("verticalDriftStd", v, &["position_accuracy_vertical_m"], format!("{v} m RMS"));
    }
    if let Some(v) = x.f("velocity_accuracy_mps") {
        c.put("velocityNoiseStd", v, &["velocity_accuracy_mps"], format!("{v} m/s RMS"));
    }
    if let Some(ms) = x.f("latency_ms") {
        let n = (ms / 1000.0 / period).round();
        c.put_value("latencySamples", Toml::Integer(n as i64), &["latency_ms"], format!("{ms} ms at the output rate"));
    }
    c.assumed.push("attitude and position accuracies as Gauss-Markov errors with 60 s correlation".into());
}

fn visual_odometry(x: &Figures, c: &mut Conversion) {
    let period = x.period(c, &["output_rate_hz", "update_rate_hz"]);
    if let Some(p) = x.f("drift_percent") {
        c.put("driftFraction", p / 100.0, &["drift_percent"], format!("{p} % of the distance travelled"));
        c.assumed.push("drift direction correlated over 60 s".into());
    } else {
        c.missing.push(("drift_percent", "driftFraction"));
    }
    if let Some(ms) = x.f("latency_ms") {
        let n = (ms / 1000.0 / period).round();
        c.put_value("latencySamples", Toml::Integer(n as i64), &["latency_ms"], format!("{ms} ms at the output rate"));
    }
}

fn beacon(x: &Figures, c: &mut Conversion) {
    let period = x.period(c, &["update_rate_hz"]);
    if let Some(v) = x.f("position_accuracy_2d_m") {
        c.put("horizontalNoiseStd", v / 2.0, &["position_accuracy_2d_m"], format!("±{v} m taken as 2 sigma"));
        if let Some(z) = x.f("position_accuracy_3d_z_m") {
            c.put("verticalNoiseStd", z / 2.0, &["position_accuracy_3d_z_m"], format!("±{z} m taken as 2 sigma"));
        }
    } else if x.f("position_accuracy_m").is_some() {
        c.sigma(x, "position_accuracy_m", "horizontalNoiseStd", 1.0, " m");
        if let Some(v) = x.f("vertical_accuracy_m") {
            c.put("verticalNoiseStd", v / 2.0, &["vertical_accuracy_m"], format!("±{v} m taken as 2 sigma"));
        } else if let Some(v) = x.f("position_accuracy_m") {
            c.put("verticalNoiseStd", v / 2.0, &["position_accuracy_m"], "vertical not stated: as horizontal");
        }
    } else {
        c.missing.push(("position_accuracy_m", "horizontalNoiseStd"));
    }
    if let Some(ms) = x.at("latency_ms", 0) {
        let n = (ms / 1000.0 / period).round();
        c.put_value("latencySamples", Toml::Integer(n as i64), &["latency_ms"], format!("{ms} ms (the lowest stated)"));
    }
}

fn precision_landing(x: &Figures, c: &mut Conversion) {
    x.period(c, &["frame_rate_hz", "update_rate_hz"]);
    let h = x.f("fov_h_deg");
    if let Some(v) = h {
        c.put("fieldOfViewX", v * DEG, &["fov_h_deg"], "horizontal field of view");
    }
    if let Some(v) = x.f("fov_v_deg") {
        c.put("fieldOfViewY", v * DEG, &["fov_v_deg"], "vertical field of view");
    }
    if let (Some(fov), Some(px)) = (h, x.at("resolution_px", 0)) {
        c.put("angleQuantum", fov * DEG / px, &["fov_h_deg", "resolution_px"], format!("one pixel: {fov}° / {px}"));
    }
    if let Some(v) = x.f("detection_range_max_m") {
        c.put("maxRange", v, &["detection_range_max_m"], "farthest detection of the target");
    }
}

fn battery_pack(part: &Table) -> (Table, Vec<String>) {
    let f = |k: &str| part.get(k).and_then(|v| v.as_float().or(v.as_integer().map(|n| n as f64)));
    let mut t = Table::new();
    let mut notes = Vec::new();
    let cells = f("cells_series").or(f("cells")).unwrap_or(4.0);
    t.insert("cells".into(), Toml::Integer(cells as i64));
    if let Some(mah) = f("capacity_mah") {
        t.insert("capacity".into(), Toml::Float(mah / 1000.0));
    }
    let parallel = f("cells_parallel").unwrap_or(1.0);
    let r = f("internal_resistance_pack_mohm")
        .map(|r| r / 1000.0)
        .or(f("internal_resistance_cell_mohm_dc_10a_1s").map(|r| r / 1000.0 * cells / parallel))
        .or(f("internal_resistance_cell_mohm").map(|r| r / 1000.0 * cells / parallel));
    match r {
        Some(r) => {
            t.insert("resistance".into(), Toml::Float(r));
        }
        None => {
            if let Some(mah) = f("capacity_mah") {
                let r = 0.015 / (mah / 1000.0 / parallel) * cells / parallel;
                t.insert("resistance".into(), Toml::Float(r));
                notes.push(format!("pack resistance {r:.4} Ω assumed (15 mΩ·Ah per cell)"));
            }
        }
    }
    (t, notes)
}

/// Replace every `part` of `[peripherals.KIND]` by its parameters (explicit keys win).
pub fn expand(root: &mut Table, reports: &mut Vec<Json>) -> Result<()> {
    let Some(peripherals) = root.get("peripherals").and_then(Toml::as_table).cloned() else {
        return Ok(());
    };
    let mut out = peripherals.clone();
    for (name, sel) in &peripherals {
        let Some(kind) = kind(name) else { continue };
        let Some(sel) = sel.as_table() else { continue };
        let mut sel = sel.clone();
        // A battery named by its part.
        if let Some(Toml::String(b)) = sel.get("battery").cloned() {
            let (id, part) = crate::parts::find("battery", &b)?;
            let (pack, notes) = battery_pack(&part);
            reports.push(json!({
                "kind": "battery", "part": id,
                "name": part.get("name").and_then(Toml::as_str).unwrap_or(&id),
                "source": part.get("source").and_then(Toml::as_str).unwrap_or(""),
                "level": "pack", "model": "Power.Battery",
                "parameters": pack.iter().map(|(k, v)| (k.clone(), json!({"value": v}))).collect::<serde_json::Map<_, _>>(),
                "assumed": notes,
                "not_represented": {"c_rating_continuous": "a limit, not modelled", "weight_g": "the vehicle's [mass] includes the pack"},
                "not_stated": {}
            }));
            sel.insert("battery".into(), Toml::Table(pack));
        }
        let calibration = sel.remove("calibration");
        let Some(Toml::String(wanted)) = sel.remove("part") else {
            if calibration.is_some() {
                return Err(format!("peripherals.{name}.calibration: needs a part"));
            }
            out.insert(name.clone(), Toml::Table(sel));
            continue;
        };
        let (id, part) = crate::parts::find(name, &wanted)?;
        let level = match sel.get("level").or(sel.get("model")).and_then(Toml::as_str) {
            Some(l) => kind
                .level(l)
                .ok_or_else(|| format!("peripherals.{name}: no level {l:?}"))?,
            // The sampled level when there is one: W2 (analog) is chosen explicitly.
            None => kind.levels.iter().find(|l| l.model == "sampled").unwrap_or(kind.levels.last().unwrap()),
        };
        let x = Figures { part: &part, explicit: &sel };
        let mut c = Conversion::default();
        match name.as_str() {
            "rangefinder" => rangefinder(&x, &mut c),
            "airspeed" => airspeed(&x, &mut c),
            "optical_flow" => optical_flow(&x, &mut c),
            "rpm" => rpm(&x, &mut c),
            "power_module" => power_module(&x, &mut c),
            "temperature" => temperature(&x, &mut c),
            "external_ahrs" => external_ahrs(&x, &mut c),
            "visual_odometry" => visual_odometry(&x, &mut c),
            "beacon" => beacon(&x, &mut c),
            "precision_landing" => precision_landing(&x, &mut c),
            _ => {}
        }
        let accepted: Vec<&str> = level.parameters.iter().map(|p| p.0).collect();
        let cal = match &calibration {
            None => None,
            Some(v) => {
                let path = v.as_str().ok_or(format!("peripherals.{name}.calibration: expected a path"))?;
                let (values, rep) = crate::parts::read_calibration(&format!("peripherals.{name}"), name, &id,
                                                                   level.level, path, &accepted)?;
                for v in values {
                    let key = *accepted.iter().find(|k| **k == v.key).unwrap();
                    c.entries.retain(|e| e.key != key);
                    c.entries.push(Entry { key, value: v.value, from: vec![], how: v.how, class: Some(v.class) });
                }
                Some(rep)
            }
        };
        let mut derived = serde_json::Map::new();
        let mut used = std::collections::BTreeSet::new();
        for e in &c.entries {
            used.extend(e.from.iter().copied());
            if !accepted.contains(&e.key) {
                derived.insert(e.key.into(), json!({"value": format!("not a parameter of {} ({})", level.level, level.model), "from": e.from, "how": e.how}));
                continue;
            }
            if sel.contains_key(e.key) {
                derived.insert(e.key.into(), json!({"value": "explicit in the configuration", "from": e.from, "how": e.how}));
                continue;
            }
            let mut d = json!({"value": e.value, "from": e.from, "how": e.how});
            if let Some(class) = e.class {
                d["class"] = json!(class);
            }
            derived.insert(e.key.into(), d);
            sel.insert(e.key.into(), e.value.clone());
        }
        sel.insert("model".into(), Toml::String(level.model.into()));
        sel.remove("level");
        let mut not_represented = serde_json::Map::new();
        for (k, why) in &c.unused {
            not_represented.insert(k.clone(), json!(why));
        }
        for k in part.keys() {
            if ["name", "kind", "manufacturer", "source", "notes", "url", "not_stated", "interface"]
                .contains(&k.as_str())
                || used.contains(k.as_str())
                || not_represented.contains_key(k)
            {
                continue;
            }
            not_represented.insert(k.clone(), json!("not used by the converter"));
        }
        let mut not_stated = serde_json::Map::new();
        for (figure, parameter) in &c.missing {
            if accepted.contains(parameter) && !sel.contains_key(*parameter) {
                not_stated.insert((*figure).into(), json!(format!("not stated: {parameter} stays at the library default")));
            }
        }
        let mut report = json!({
            "kind": name, "part": id,
            "name": part.get("name").and_then(Toml::as_str).unwrap_or(&id),
            "source": part.get("source").and_then(Toml::as_str).unwrap_or(""),
            "level": level.level, "model": level.model,
            "parameters": derived, "assumed": c.assumed,
            "not_represented": not_represented, "not_stated": not_stated,
            "evidence": crate::parts::evidence_of(cal.as_ref()),
        });
        if let Some(cal) = cal {
            report["calibration"] = cal;
        }
        reports.push(report);
        out.insert(name.clone(), Toml::Table(sel));
    }
    root.insert("peripherals".into(), Toml::Table(out));
    Ok(())
}

/// The report of one part as a peripheral: `fire-compose part KIND PART`.
pub fn report(kind: &str, part: &str, level: Option<&str>, explicit: &Table) -> Result<Json> {
    let mut sel = explicit.clone();
    sel.insert("part".into(), Toml::String(part.into()));
    if let Some(l) = level {
        sel.insert("level".into(), Toml::String(l.into()));
    }
    let mut root = Table::new();
    root.insert(
        "peripherals".into(),
        Toml::Table(Table::from_iter([(kind.to_string(), Toml::Table(sel))])),
    );
    let mut reports = Vec::new();
    expand(&mut root, &mut reports)?;
    reports
        .into_iter()
        .rev()
        .find(|r| r["kind"] == kind)
        .ok_or_else(|| format!("{kind}: not a peripheral kind"))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn quad(extra: &str) -> std::result::Result<ResolvedFireConfig, String> {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../configs/quad.toml");
        let raw = std::fs::read_to_string(root).unwrap() + extra;
        crate::config::parse_at(&raw, None)
    }

    #[test]
    fn levels_and_keys_are_checked() {
        let c = quad("\n[peripherals.rangefinder]\nlevel = \"L1\"\nnoiseStd = 0.02\norientation = \"down\"\n").unwrap();
        assert_eq!(c.peripherals["rangefinder"]["model"].as_str(), Some("sampled"));
        assert!(quad("\n[peripherals.rangefinder]\nnoiseStd = 0.02\n").is_err(), "L0 has no noise");
        assert!(quad("\n[peripherals.rangefinder]\norientation = \"sideways\"\n").is_err());
        assert!(quad("\n[peripherals.sonar]\n").is_err());
    }

    #[test]
    fn every_kind_composes_and_exports_its_capability() {
        let mut extra = String::new();
        for k in KINDS {
            extra += &format!("\n[peripherals.{}]\nlevel = \"{}\"\n", k.name, k.levels[1].level);
        }
        let c = quad(&extra).unwrap();
        let mut components = Vec::new();
        let mut equalities = Vec::new();
        components_for_test(&c, &mut components, &mut equalities);
        for k in KINDS {
            assert!(components.iter().any(|i| i.name == k.component), "{}", k.name);
            for (export, ..) in k.outputs {
                assert!(components.iter().any(|i| i.name == *export), "{export}");
            }
        }
        assert!(equalities.iter().any(|(n, _)| n == "rangefinderSensor.truth"));
    }

    fn components_for_test(c: &ResolvedFireConfig, i: &mut Vec<Instance>, e: &mut Vec<(String, Value)>) {
        components(c, i, e).unwrap();
    }

    #[test]
    fn orientations_are_rotations_with_the_named_boresight() {
        for (name, m) in ORIENTATIONS {
            let z = [m[0][2], m[1][2], m[2][2]];
            let want = match name {
                "down" => [0., 0., 1.],
                "forward" => [1., 0., 0.],
                "back" => [-1., 0., 0.],
                "right" => [0., 1., 0.],
                "left" => [0., -1., 0.],
                _ => [0., 0., -1.],
            };
            assert_eq!(z, want, "{name}");
            let v = toml::Value::Array(m.iter().map(|r| toml::Value::Array(r.iter().map(|x| toml::Value::Float(*x)).collect())).collect());
            crate::config::rotation(&v, name).unwrap();
        }
    }
}
