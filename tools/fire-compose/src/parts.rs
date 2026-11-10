//! Real parts as aliases of the library's sensor models.
//!
//! A part (`parts/<kind>/<id>.toml`) holds a device's datasheet figures as
//! published, with their source, never model parameters. Naming it in a
//! configuration,
//!
//! ```toml
//! [models.gnss]
//! part = "ublox_neo_m9n"   # an id, a name ("u-blox NEO-M9N"), or a path to a part file
//! level = "G1"             # optional: a level or model name; default the highest this kind converts to
//! mode = "sbas"            # optional, per kind: gnss standalone|sbas|rtk, barometer low_noise|fast
//! tolerances = "draw"      # optional: draw offsets, scale and cross-axis errors within the
//!                          #   datasheet's ± tolerances (deterministic in [KIND].seed); default "none"
//! ```
//!
//! selects that level and fills its `[KIND]` parameters from the figures. Keys
//! set explicitly in `[KIND]` win. One converter per kind owns the conversion
//! (CEP to a standard deviation, noise density to per-sample noise, bits and
//! range to an LSB, ...); each parameter records the figures it came from, and
//! every figure the selected level cannot represent is reported, with each
//! value the conversion had to assume. The report is the configuration's
//! `parts` entry and goes into the manifest.
use crate::config::Table;
use crate::Result;
use serde_json::{json, Value as Json};
use std::collections::BTreeSet;
use std::path::{Path, PathBuf};
use toml::Value;

const G: f64 = 9.80665;
const DEG: f64 = std::f64::consts::PI / 180.0;
/// CEP (50 % radius) of a circular 2-D normal error is 1.1774 sigma.
const CEP_SIGMA: f64 = 1.177_410_022_515_474_6;
/// The median of |N(0, sigma)| is 0.6745 sigma.
const MEDIAN_SIGMA: f64 = 0.674_489_750_196_081_7;

/// Kinds a part can be, the `[models]` key, and the levels in order.
pub const KINDS: [(&str, &[(&str, &str)]); 4] = [
    (
        "imu",
        &[
            ("I0", "ideal"),
            ("I1", "digital"),
            ("I2", "errors"),
            ("I3", "electromechanical"),
        ],
    ),
    ("gnss", &[("G0", "ideal"), ("G1", "statistical")]),
    ("barometer", &[("B0", "ideal"), ("B1", "sampled")]),
    ("magnetometer", &[("M0", "ideal"), ("M1", "sampled")]),
];
/// The level a part selects when the configuration names none: the highest
/// whose parameters the datasheet figures can fill without more input (I3
/// also needs the resonator physics, which no datasheet gives).
const DEFAULT_LEVEL: [(&str, &str); 4] = [
    ("imu", "errors"),
    ("gnss", "statistical"),
    ("barometer", "sampled"),
    ("magnetometer", "sampled"),
];
const METADATA: [&str; 6] = ["name", "kind", "manufacturer", "source", "notes", "url"];
const OPTIONS: [&str; 8] = [
    "part",
    "level",
    "model",
    "mode",
    "tolerances",
    "range",
    "drift",
    "calibration",
];
/// Evidence classes of a parameter value (docs/sensor-calibration.md):
/// stated by the datasheet under its conditions, calculated from stated
/// figures, estimated from a named dataset, or chosen without sufficient
/// information. `configuration` is a modelling choice of the user.
pub const EVIDENCE: [&str; 5] = ["specified", "derived", "measured", "assumed", "configuration"];

/// One parameter the conversion produced.
struct Entry {
    key: &'static str,
    value: Value,
    from: Vec<&'static str>,
    how: String,
    /// One of EVIDENCE, when the converter classifies its values.
    class: Option<&'static str>,
}

#[derive(Default)]
struct Conversion {
    entries: Vec<Entry>,
    assumed: Vec<String>,
    /// Figures that go into no parameter of any level, and why.
    unused: Vec<(&'static str, &'static str)>,
    /// Tolerances not applied (tolerances = "none").
    tolerances: Vec<&'static str>,
    /// Figures the conversion would use that the part does not state: (figure,
    /// the parameter left at the library default).
    missing: Vec<(&'static str, &'static str)>,
    /// Figures used to check the configuration (rates), not converted.
    checked: Vec<&'static str>,
    /// Properties the model needs that no figure determines (kept unknown in
    /// the report even when a runnable value had to be assumed).
    unknown: Vec<String>,
}

impl Conversion {
    fn need(&mut self, x: &Inputs, figure: &'static str, parameter: &'static str) -> Option<f64> {
        let v = x.f(figure);
        if v.is_none() {
            self.missing.push((figure, parameter));
        }
        v
    }
    fn put(
        &mut self,
        key: &'static str,
        value: Value,
        from: &[&'static str],
        how: impl Into<String>,
    ) {
        self.entries.push(Entry {
            key,
            value,
            from: from.to_vec(),
            how: how.into(),
            class: None,
        });
    }
    /// `put` with an evidence class.
    fn put_as(
        &mut self,
        class: &'static str,
        key: &'static str,
        value: Value,
        from: &[&'static str],
        how: impl Into<String>,
    ) {
        self.put(key, value, from, how);
        self.entries.last_mut().unwrap().class = Some(class);
    }
}

struct Inputs<'a> {
    part: &'a Table,
    /// The configuration's explicit [KIND] values (they win, and the
    /// conversion uses their sample period).
    explicit: &'a Table,
    mode: String,
    draw: Option<Rng>,
    range: Option<&'a Table>,
    /// Barometer: "relative_accuracy" (default) or "none".
    drift: String,
}

impl Inputs<'_> {
    fn f(&self, key: &str) -> Option<f64> {
        self.part
            .get(key)
            .and_then(|v| v.as_float().or(v.as_integer().map(|i| i as f64)))
    }
    fn explicit_f(&self, key: &str) -> Option<f64> {
        self.explicit
            .get(key)
            .and_then(|v| v.as_float().or(v.as_integer().map(|i| i as f64)))
    }
}

/// Deterministic draws for tolerances (splitmix64).
struct Rng(u64);
impl Rng {
    fn new(seed: u64, kind: &str) -> Self {
        let mut s = seed ^ 0x9E37_79B9_7F4A_7C15;
        for b in kind.bytes() {
            s = s.rotate_left(7) ^ b as u64;
        }
        Rng(s)
    }
    /// Uniform in [-1, 1].
    fn signed(&mut self) -> f64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^= z >> 31;
        (z >> 11) as f64 / (1u64 << 53) as f64 * 2.0 - 1.0
    }
}

fn num(x: f64) -> Value {
    // Keep TOML literals short and stable.
    let r: f64 = format!("{x:.6e}").parse().unwrap();
    Value::Float(r)
}
fn vec(xs: &[f64]) -> Value {
    Value::Array(xs.iter().map(|x| num(*x)).collect())
}
fn mat(m: [[f64; 3]; 3]) -> Value {
    Value::Array(m.iter().map(|r| vec(r)).collect())
}
fn mat6(diag: [f64; 6]) -> Value {
    Value::Array(
        (0..6)
            .map(|i| {
                vec(&(0..6)
                    .map(|j| if i == j { diag[i] } else { 0.0 })
                    .collect::<Vec<_>>())
            })
            .collect(),
    )
}

// --- converters ----------------------------------------------------------------

fn imu(x: &mut Inputs) -> Result<Conversion> {
    let mut c = Conversion::default();
    let period = x.explicit_f("samplePeriod").unwrap_or(0.0025);
    let decimation = x.explicit_f("adcDecimation").unwrap_or(4.0);
    if !x.explicit.contains_key("samplePeriod") {
        c.put(
            "samplePeriod",
            num(period),
            &[],
            "host default (400 Hz output); set imu.samplePeriod",
        );
        c.put(
            "adcDecimation",
            Value::Integer(4),
            &[],
            "host default (1.6 kHz ADC ticks)",
        );
    }
    let tick_hz = decimation / period;
    for (key, odr) in [
        ("accel_odr_max_hz", "accelerometer"),
        ("gyro_odr_max_hz", "gyro"),
    ] {
        if let Some(max) = x.f(key) {
            c.checked.push(key);
            if tick_hz > max * 1.0001 {
                return Err(format!(
                    "imu: ADC ticks at {tick_hz:.0} Hz exceed the part's {odr} data rate {max} Hz"
                ));
            }
        }
    }
    let range = |name: &str, key: &'static str, options: &str| -> Result<Option<f64>> {
        let pick = x
            .range
            .and_then(|r| r.get(name))
            .and_then(|v| v.as_float().or(v.as_integer().map(|i| i as f64)));
        match pick {
            None => Ok(x.f(key)),
            Some(p) => {
                let listed = x.part.get(options).and_then(Value::as_array);
                if let Some(list) = listed {
                    if !list
                        .iter()
                        .any(|v| v.as_float().or(v.as_integer().map(|i| i as f64)) == Some(p))
                    {
                        return Err(format!(
                            "models.imu.range.{name} = {p}: the part offers {options} = {}",
                            Value::Array(list.clone())
                        ));
                    }
                }
                Ok(Some(p))
            }
        }
    };
    let accel_fs = range(
        "accel_g",
        "accel_full_scale_g",
        "accel_full_scale_options_g",
    )?;
    let gyro_fs = range(
        "gyro_dps",
        "gyro_full_scale_dps",
        "gyro_full_scale_options_dps",
    )?;
    if accel_fs.is_none() {
        c.missing.push(("accel_full_scale_g", "fullScale"));
    }
    if gyro_fs.is_none() {
        c.missing.push(("gyro_full_scale_dps", "fullScale"));
    }
    if let (Some(a), Some(g)) = (accel_fs, gyro_fs) {
        let fs = [a * G, a * G, a * G, g * DEG, g * DEG, g * DEG];
        c.put(
            "fullScale",
            vec(&fs),
            &["accel_full_scale_g", "gyro_full_scale_dps"],
            format!("±{a} g, ±{g} °/s (the largest range unless models.imu.range picks one)"),
        );
        if let (Some(ab), Some(gb)) = (
            c.need(x, "accel_adc_bits", "quantum"),
            c.need(x, "gyro_adc_bits", "quantum"),
        ) {
            let q =
                [0, 1, 2, 3, 4, 5].map(|i| 2.0 * fs[i] / 2f64.powf(if i < 3 { ab } else { gb }));
            c.put(
                "quantum",
                vec(&q),
                &["accel_adc_bits", "gyro_adc_bits"],
                format!("LSB = 2 × full scale / 2^bits ({ab} and {gb} bits)"),
            );
        }
    }
    if let (Some(an), Some(gn)) = (
        c.need(x, "accel_noise_density_ug_rthz", "noiseStd"),
        c.need(x, "gyro_noise_density_dps_rthz", "noiseStd"),
    ) {
        let k = (tick_hz / 2.0).sqrt();
        let a = an * 1e-6 * G * k;
        let az = x.f("accel_noise_density_z_ug_rthz").unwrap_or(an) * 1e-6 * G * k;
        let g = gn * DEG * k;
        c.put(
            "noiseStd",
            vec(&[a, a, az, g, g, g]),
            &[
                "accel_noise_density_ug_rthz",
                "accel_noise_density_z_ug_rthz",
                "gyro_noise_density_dps_rthz",
            ],
            format!("per ADC tick: density × √(f_tick/2), f_tick = {tick_hz:.0} Hz"),
        );
    }
    if let Some(b) = c.need(x, "gyro_bias_instability_dph", "biasStd") {
        let g = b * DEG / 3600.0;
        c.put(
            "biasStd",
            vec(&[0.0, 0.0, 0.0, g, g, g]),
            &["gyro_bias_instability_dph"],
            "gyro Gauss-Markov σ = the bias instability",
        );
        c.put("biasTau", vec(&[100.0; 6]), &[], "assumed");
        c.assumed
            .push("bias drift correlation time 100 s (the datasheet gives none)".into());
    }
    c.put(
        "analogTau",
        vec(&[0.001; 6]),
        &[],
        "library default: the datasheet's anti-alias filter is configuration-dependent",
    );
    c.put(
        "digitalTau",
        vec(&[0.002; 6]),
        &[],
        "library default: the digital low-pass is configuration-dependent",
    );
    for key in ["seed", "streamId"] {
        c.put(key, Value::Integer(1), &[], "default");
    }
    // Tolerances: drawn or reported.
    let mut draw = x.draw.take();
    let mut tol = |c: &mut Conversion,
                   key: &'static str,
                   figures: &[&'static str],
                   f: &dyn Fn(&mut Rng, &[f64]) -> Value,
                   how: &str| {
        let vals: Option<Vec<f64>> = figures.iter().map(|k| x.f(k)).collect();
        let Some(vals) = vals else { return };
        match draw.as_mut() {
            Some(rng) => {
                let v = f(rng, &vals);
                c.put(key, v, figures, format!("drawn uniformly within ±{how}"));
            }
            None => c.tolerances.extend(figures),
        }
    };
    tol(
        &mut c,
        "accelBias",
        &["accel_zero_g_offset_mg"],
        &|r, v| vec(&[0, 1, 2].map(|_| r.signed() * v[0] * 1e-3 * G)),
        "the zero-g offset",
    );
    tol(
        &mut c,
        "gyroBias",
        &["gyro_zero_rate_offset_dps"],
        &|r, v| vec(&[0, 1, 2].map(|_| r.signed() * v[0] * DEG)),
        "the zero-rate offset",
    );
    tol(
        &mut c,
        "scale",
        &[
            "accel_sensitivity_tolerance_pct",
            "gyro_sensitivity_tolerance_pct",
        ],
        &|r, v| vec(&[0, 1, 2, 3, 4, 5].map(|i| r.signed() * v[i / 3] / 100.0)),
        "the sensitivity tolerance",
    );
    let cross = |r: &mut Rng, v: &[f64]| {
        let mut m = [[0.0; 3]; 3];
        for (i, row) in m.iter_mut().enumerate() {
            for (j, e) in row.iter_mut().enumerate() {
                if i != j {
                    *e = r.signed() * v[0] / 100.0;
                }
            }
        }
        mat(m)
    };
    tol(
        &mut c,
        "accelMisalignment",
        &["accel_cross_axis_pct"],
        &cross,
        "the cross-axis sensitivity",
    );
    tol(
        &mut c,
        "gyroMisalignment",
        &["gyro_cross_axis_pct"],
        &cross,
        "the cross-axis sensitivity",
    );
    tol(
        &mut c,
        "gyroGSensitivity",
        &["gyro_linear_accel_sensitivity_dps_per_g"],
        &|r, v| mat([0, 1, 2].map(|_| [0, 1, 2].map(|_| r.signed() * v[0] * DEG / G))),
        "the g-sensitivity",
    );
    tol(
        &mut c,
        "temperatureCoefficient",
        &[
            "accel_zero_g_tempco_mg_per_c",
            "gyro_zero_rate_tempco_dps_per_c",
        ],
        &|r, v| {
            vec(&[0, 1, 2, 3, 4, 5]
                .map(|i| r.signed() * if i < 3 { v[0] * 1e-3 * G } else { v[1] * DEG }))
        },
        "the offset temperature coefficients",
    );
    c.unused = vec![
        ("gyro_rms_noise_dps", "the noise density is used instead"),
        ("accel_rms_noise_mg", "the noise density is used instead"),
        ("gyro_nonlinearity_pct", "no nonlinearity in the IMU models"),
        (
            "accel_nonlinearity_pct",
            "no nonlinearity in the IMU models",
        ),
        (
            "gyro_sensitivity_tempco_pct_per_c",
            "the models' temperature term is an offset, not a scale",
        ),
        (
            "accel_sensitivity_tempco_pct_per_c",
            "the models' temperature term is an offset, not a scale",
        ),
        (
            "gyro_mechanical_frequency_khz",
            "I3's resonator physics (imu.gyroPhysics) are not derived from it",
        ),
    ];
    Ok(c)
}

fn gnss(x: &mut Inputs) -> Result<Conversion> {
    let mut c = Conversion::default();
    let (cep_key, vert_key, rtk) = match x.mode.as_str() {
        "" | "standalone" => ("horizontal_cep_m", "vertical_median_m", false),
        "sbas" => ("horizontal_cep_sbas_m", "vertical_median_sbas_m", false),
        "rtk" => ("horizontal_cep_rtk_m", "vertical_rtk_m", true),
        m => return Err(format!("models.gnss.mode = {m:?}: standalone, sbas or rtk")),
    };
    let rate_key = if rtk {
        "nav_rate_max_rtk_hz"
    } else {
        "nav_rate_max_hz"
    };
    let max_rate = x.f(rate_key).or(x.f("nav_rate_max_hz"));
    let period = match x.explicit_f("samplePeriod") {
        Some(p) => p,
        None => {
            let p = max_rate.map_or(0.2, |r| (1.0 / r).max(0.2));
            c.put(
                "samplePeriod",
                num(p),
                &[],
                "5 Hz (ArduPilot's GPS_RATE_MS default), or the part's maximum if slower",
            );
            p
        }
    };
    if let Some(r) = max_rate {
        c.checked.push(if x.f(rate_key).is_some() {
            rate_key
        } else {
            "nav_rate_max_hz"
        });
        if 1.0 / period > r * 1.0001 {
            return Err(format!(
                "gnss: {:.1} Hz exceeds the part's maximum navigation rate {r} Hz ({rate_key})",
                1.0 / period
            ));
        }
    }
    let Some(mut cep) = x.f(cep_key) else {
        return Err(format!(
            "gnss mode {:?}: the part states no {cep_key}",
            x.mode
        ));
    };
    let mut from: Vec<&'static str> = vec![cep_key];
    if rtk {
        if let Some(ppm) = x.f("rtk_ppm") {
            cep += ppm * 1e-6 * 1000.0;
            from.push("rtk_ppm");
            c.assumed
                .push("RTK baseline 1 km (the CEP's ppm term)".into());
        }
    }
    let sh = cep / CEP_SIGMA;
    let sv = match x.f(vert_key) {
        Some(v) => {
            from.push(vert_key);
            v / MEDIAN_SIGMA
        }
        None => {
            c.assumed.push(format!(
                "vertical σ = 1.5 × horizontal (the part states no {vert_key})"
            ));
            1.5 * sh
        }
    };
    let white: f64 = 0.1;
    c.assumed.push(format!(
        "position error {}% Gauss-Markov (τ {} s), the rest white per epoch: the datasheet gives the total only",
        100.0 - white * 100.0, if rtk { 10 } else { 60 }));
    let (bh, bv) = (
        (1.0 - white * white).sqrt() * sh,
        (1.0 - white * white).sqrt() * sv,
    );
    let pos_from = from.clone();
    let vel = c
        .need(x, "velocity_accuracy_mps", "noiseFactor")
        .map(|v| v / MEDIAN_SIGMA);
    if vel.is_some() {
        from.push("velocity_accuracy_mps");
        c.assumed
            .push("the 50 % velocity accuracy applies per axis, as white noise".into());
    }
    let vs = vel.unwrap_or(0.0);
    c.put(
        "noiseFactor",
        mat6([white * sh, white * sh, white * sv, vs, vs, vs]),
        &from,
        format!("σ_h = CEP/1.1774 = {sh:.3} m, σ_v {sv:.3} m, σ_vel = 50 %/0.6745 = {vs:.3} m/s"),
    );
    c.put(
        "biasFactor",
        mat6([bh, bh, bv, 0.0, 0.0, 0.0]),
        &pos_from,
        "the correlated part of the position error",
    );
    let tau = if rtk { 10.0 } else { 60.0 };
    c.put(
        "biasTau",
        vec(&[tau, tau, tau, 10.0, 10.0, 10.0]),
        &[],
        "assumed",
    );
    for key in ["seed", "streamId"] {
        c.put(
            key,
            Value::Integer(if key == "seed" { 1 } else { 201 }),
            &[],
            "default",
        );
    }
    c.unused = vec![
        (
            "heading_accuracy_deg",
            "heading is derived from velocity by the autopilot",
        ),
        ("time_pulse_accuracy_ns", "no time pulse in the models"),
        (
            "cold_start_s",
            "no acquisition phase in G1: a fix from the first epoch",
        ),
        ("hot_start_s", "no acquisition phase in G1"),
        ("tracking_sensitivity_dbm", "no signal model in G1"),
        ("max_dynamics_g", "operational limits are not enforced"),
        ("max_velocity_mps", "operational limits are not enforced"),
        ("max_altitude_m", "operational limits are not enforced"),
        ("concurrent_gnss", "no constellation model in G1"),
        (
            "nav_rate_max_single_gnss_hz",
            "the multi-constellation rate is used",
        ),
        ("horizontal_cep_m", "another mode is selected"),
        ("horizontal_cep_sbas_m", "another mode is selected"),
        ("horizontal_cep_rtk_m", "another mode is selected"),
        ("rtk_ppm", "another mode is selected"),
        ("vertical_rtk_m", "another mode is selected"),
        ("vertical_median_m", "another mode is selected"),
        ("vertical_median_sbas_m", "another mode is selected"),
        ("nav_rate_max_rtk_hz", "another mode is selected"),
    ];
    Ok(c)
}

fn barometer(x: &mut Inputs) -> Result<Conversion> {
    let mut c = Conversion::default();
    let (noise_key, mode_key, time_key) = match x.mode.as_str() {
        "" | "low_noise" => (
            "pressure_rms_noise_pa",
            "pressure_rms_noise_mode",
            "conversion_time_lowest_noise_ms",
        ),
        "fast" => (
            "pressure_rms_noise_fast_pa",
            "pressure_rms_noise_fast_mode",
            "conversion_time_fast_ms",
        ),
        m => return Err(format!("models.barometer.mode = {m:?}: low_noise or fast")),
    };
    // One reading is a pressure and a temperature measurement: when the part
    // states both measurement times, they share the period (DPS310: rate_T *
    // time_T + rate_P * time_P < 1 s).
    let pressure_time = x.f(time_key).map(|ms| ms * 1e-3);
    let temperature_time = x.f("temperature_measurement_time_ms").map(|ms| ms * 1e-3);
    let odr = x.f("odr_max_hz").map(|r| 1.0 / r);
    let measured = match (pressure_time, temperature_time) {
        (Some(p), Some(t)) => Some(p + t),
        (Some(p), None) => Some(p),
        _ => None,
    };
    let min_period = match (measured, odr) {
        (Some(m), Some(o)) => Some(m.max(o)),
        (m, o) => m.or(o),
    };
    let period = match x.explicit_f("samplePeriod") {
        Some(p) => p,
        None => {
            let p = min_period.map_or(0.02, |m| m.max(0.02));
            let mut from = Vec::new();
            if pressure_time.is_some() {
                from.push(time_key);
            }
            if temperature_time.is_some() && pressure_time.is_some() {
                from.push("temperature_measurement_time_ms");
            }
            if odr.is_some() && min_period == odr {
                from.push("odr_max_hz");
            }
            let derived = min_period.is_some_and(|m| m >= 0.02);
            c.put_as(
                if derived { "derived" } else { "configuration" },
                "samplePeriod",
                num(p),
                if derived { &from } else { &[] },
                if derived {
                    "the shortest period the part allows in this mode: pressure plus temperature measurement time, at most the maximum rate"
                } else {
                    "50 Hz (a modelling choice), slower than any limit the part states"
                },
            );
            p
        }
    };
    if let Some(m) = min_period {
        if pressure_time.is_some() {
            c.checked.push(time_key);
        }
        if temperature_time.is_some() {
            c.checked.push("temperature_measurement_time_ms");
        }
        if odr.is_some() {
            c.checked.push("odr_max_hz");
        }
        if period < m * 0.9999 {
            return Err(format!(
                "barometer: {:.1} Hz is faster than the part allows in this mode ({:.1} Hz: a reading needs {:.1} ms of pressure{} measurement)",
                1.0 / period,
                1.0 / m,
                m * 1e3,
                if temperature_time.is_some() { " and temperature" } else { "" }
            ));
        }
    }
    if let Some(n) = c.need(x, noise_key, "pressureNoiseStd") {
        let mode = x.part.get(mode_key).and_then(Value::as_str).unwrap_or("");
        c.put_as(
            "specified",
            "pressureNoiseStd",
            num(n),
            &[noise_key],
            format!("RMS noise per reading ({mode})"),
        );
    }
    if let Some(q) = c.need(x, "pressure_resolution_pa", "pressureQuantum") {
        c.put_as(
            "specified",
            "pressureQuantum",
            num(q),
            &["pressure_resolution_pa"],
            "output LSB",
        );
    }
    match x.drift.as_str() {
        "" | "relative_accuracy" => {
            if let Some(r) = c.need(x, "relative_accuracy_pa", "pressureDriftStd") {
                c.put_as(
                    "assumed",
                    "pressureDriftStd",
                    num(r / 2.0),
                    &["relative_accuracy_pa"],
                    "assumed: half the ± relative accuracy as the σ of a slow drift (the figure bounds the error of a pressure step, not a drift over time)",
                );
                c.put_as(
                    "assumed",
                    "pressureDriftTau",
                    num(300.0),
                    &[],
                    "assumed: no figure states a correlation time",
                );
                c.assumed
                    .push("pressure drift: σ = relative accuracy / 2, τ 300 s".into());
            }
        }
        "none" => {
            c.put_as(
                "assumed",
                "pressureDriftStd",
                num(0.0),
                &[],
                "no-drift scenario (drift = \"none\")",
            );
            c.assumed.push("pressure drift: none (drift = \"none\")".into());
        }
        d => {
            return Err(format!(
                "models.barometer.drift = {d:?}: relative_accuracy or none"
            ))
        }
    }
    c.unknown.push(
        "pressure drift over time: no figure states a drift process or its correlation time"
            .into(),
    );
    if x.part.get("latency_ms").is_none() {
        c.unknown
            .push("delivery latency: not stated (latencySamples stays 0)".into());
    }
    if x.part.get("temperature_rms_noise_c").is_none() {
        c.unknown
            .push("temperature noise: not stated (temperatureNoiseStd stays 0)".into());
    }
    if let Some(q) = c.need(x, "temperature_resolution_c", "temperatureQuantum") {
        c.put_as(
            "specified",
            "temperatureQuantum",
            num(q),
            &["temperature_resolution_c"],
            "output LSB",
        );
    }
    for key in ["seed", "streamId"] {
        c.put_as(
            "configuration",
            key,
            Value::Integer(if key == "seed" { 1 } else { 301 }),
            &[],
            "default",
        );
    }
    let mut draw = x.draw.take();
    match draw.as_mut() {
        Some(r) => {
            if let Some(a) = x.f("absolute_accuracy_pa") {
                c.put_as(
                    "assumed",
                    "pressureBias",
                    num(r.signed() * a),
                    &["absolute_accuracy_pa"],
                    "drawn uniformly within ± the absolute accuracy (the uniform distribution is assumed)",
                );
            }
            if let Some(t) = x.f("temperature_accuracy_c") {
                c.put_as(
                    "assumed",
                    "temperatureBias",
                    num(r.signed() * t),
                    &["temperature_accuracy_c"],
                    "drawn uniformly within ± the temperature accuracy (the uniform distribution is assumed)",
                );
            }
        }
        None => {
            c.tolerances
                .extend(["absolute_accuracy_pa", "temperature_accuracy_c"]);
            c.unknown.push(
                "this unit's pressure and temperature offsets: the accuracy figures bound them, a calibration measures them".into(),
            );
        }
    }
    c.unused = vec![
        (
            "pressure_range_pa",
            "no pressure clipping in the barometer models",
        ),
        (
            "temperature_coefficient_offset_pa_per_k",
            "no temperature-dependent offset in B1",
        ),
        ("long_term_stability_pa_per_year", "beyond a flight"),
        ("long_term_stability_pa", "beyond a flight"),
        (
            "solder_drift_pa",
            "an offset after assembly: part of the absolute accuracy",
        ),
        ("pressure_rms_noise_fast_pa", "another mode is selected"),
        ("pressure_rms_noise_pa", "another mode is selected"),
        ("conversion_time_fast_ms", "another mode is selected"),
        ("conversion_time_lowest_noise_ms", "another mode is selected"),
        ("pressure_oversampling", "the mode's figures are used"),
        ("pressure_measurement_time_ms", "the mode's figures are used"),
        ("pressure_precision_pa", "the mode's figures are used"),
        ("power_supply_rejection_pa", "no supply input in B1"),
    ];
    Ok(c)
}

fn magnetometer(x: &mut Inputs) -> Result<Conversion> {
    let mut c = Conversion::default();
    let max_rate = x.f("odr_at_noise_hz").or(x.f("odr_max_hz"));
    let period = match x.explicit_f("samplePeriod") {
        Some(p) => p,
        None => {
            let p = max_rate.map_or(0.01, |r| (1.0 / r).max(0.01));
            c.put(
                "samplePeriod",
                num(p),
                &[],
                "100 Hz, or the part's rate at the quoted noise if slower",
            );
            p
        }
    };
    if let Some(r) = max_rate {
        c.checked.push(if x.f("odr_at_noise_hz").is_some() {
            "odr_at_noise_hz"
        } else {
            "odr_max_hz"
        });
        if 1.0 / period > r * 1.0001 {
            return Err(format!(
                "magnetometer: {:.1} Hz exceeds the part's {r} Hz at the quoted noise",
                1.0 / period
            ));
        }
    }
    const MG: f64 = 1e-7; // tesla per milligauss
    if let Some(n) = c.need(x, "rms_noise_mgauss", "noiseStd") {
        let setting = x
            .part
            .get("rms_noise_setting")
            .and_then(Value::as_str)
            .unwrap_or("");
        c.put(
            "noiseStd",
            vec(&[n * MG; 3]),
            &["rms_noise_mgauss"],
            format!("RMS noise per sample ({setting})"),
        );
    }
    if let Some(q) = c.need(x, "resolution_mgauss", "quantum") {
        c.put(
            "quantum",
            vec(&[q * MG; 3]),
            &["resolution_mgauss"],
            "output LSB",
        );
    }
    if let Some(fs) = c.need(x, "full_scale_gauss", "fullScale") {
        c.put(
            "fullScale",
            vec(&[fs * 1e-4; 3]),
            &["full_scale_gauss"],
            format!("±{fs} G"),
        );
    }
    for key in ["seed", "streamId"] {
        c.put(
            key,
            Value::Integer(if key == "seed" { 1 } else { 401 }),
            &[],
            "default",
        );
    }
    let mut draw = x.draw.take();
    match draw.as_mut() {
        Some(r) => {
            if let Some(o) = x.f("offset_mgauss") {
                c.put(
                    "bias",
                    vec(&[0, 1, 2].map(|_| r.signed() * o * MG)),
                    &["offset_mgauss"],
                    "drawn within ± the zero-field offset",
                );
            }
            let s = x.f("sensitivity_tolerance_pct");
            let k = x.f("cross_axis_pct");
            if s.is_some() || k.is_some() {
                let mut m = [[0.0; 3]; 3];
                for (i, row) in m.iter_mut().enumerate() {
                    for (j, e) in row.iter_mut().enumerate() {
                        *e = if i == j {
                            1.0 + r.signed() * s.unwrap_or(0.0) / 100.0
                        } else {
                            r.signed() * k.unwrap_or(0.0) / 100.0
                        };
                    }
                }
                let from: Vec<&'static str> =
                    [("sensitivity_tolerance_pct", s), ("cross_axis_pct", k)]
                        .iter()
                        .filter(|(_, v)| v.is_some())
                        .map(|(n, _)| *n)
                        .collect();
                c.put(
                    "softIron",
                    mat(m),
                    &from,
                    "drawn within ± the sensitivity tolerance and cross-axis sensitivity",
                );
            }
        }
        None => c.tolerances.extend([
            "offset_mgauss",
            "sensitivity_tolerance_pct",
            "cross_axis_pct",
        ]),
    }
    c.unused = vec![
        (
            "offset_tempco_mgauss_per_c",
            "no temperature input to the magnetometer models",
        ),
        (
            "sensitivity_tempco_pct_per_c",
            "no temperature input to the magnetometer models",
        ),
        (
            "linearity_pct",
            "no nonlinearity in the magnetometer models",
        ),
        ("adc_bits", "the stated resolution is used"),
        ("odr_max_hz", "the sample period is checked against it"),
        ("odr_at_noise_hz", "the sample period is checked against it"),
        ("full_scale_options_gauss", "the largest range is used"),
    ];
    Ok(c)
}

// --- finding parts -----------------------------------------------------------------

pub fn directory() -> PathBuf {
    crate::artifact::library_root().join("parts")
}

fn normalized(s: &str) -> String {
    s.chars()
        .filter(|c| c.is_ascii_alphanumeric())
        .collect::<String>()
        .to_ascii_lowercase()
}

/// Every part file: (kind, id, table).
pub fn all() -> Vec<(String, String, Table)> {
    let mut out = Vec::new();
    for kind in KINDS
        .iter()
        .map(|k| k.0)
        .chain(crate::powertrain::KINDS)
        .chain(crate::peripherals::KINDS.iter().map(|k| k.name))
        .chain(crate::payloads::KINDS.iter().map(|k| k.name))
        .chain(["frame", "arm_tube"])
    {
        let Ok(dir) = std::fs::read_dir(directory().join(kind)) else {
            continue;
        };
        let mut files: Vec<PathBuf> = dir
            .filter_map(|e| e.ok().map(|e| e.path()))
            .filter(|p| p.extension().is_some_and(|e| e == "toml"))
            .collect();
        files.sort();
        for p in files {
            if let Ok(t) = std::fs::read_to_string(&p)
                .map_err(|e| e.to_string())
                .and_then(|s| toml::from_str::<Table>(&s).map_err(|e| e.to_string()))
            {
                out.push((
                    kind.to_string(),
                    p.file_stem().unwrap().to_string_lossy().into(),
                    t,
                ));
            }
        }
    }
    out
}

/// A part by id, name, or a unique suffix of either (case and punctuation
/// ignored), or a path to a part file.
pub fn find(kind: &str, name: &str) -> Result<(String, Table)> {
    if name.ends_with(".toml") || name.contains('/') {
        let p = Path::new(name);
        let t: Table =
            toml::from_str(&std::fs::read_to_string(p).map_err(|e| format!("part {name}: {e}"))?)
                .map_err(|e| format!("part {name}: {e}"))?;
        return Ok((p.file_stem().unwrap().to_string_lossy().into(), t));
    }
    let want = normalized(name);
    let parts: Vec<_> = all().into_iter().filter(|(k, ..)| k == kind).collect();
    let keys = |id: &str, t: &Table| {
        [
            normalized(id),
            normalized(t.get("name").and_then(Value::as_str).unwrap_or("")),
        ]
    };
    let exact: Vec<_> = parts
        .iter()
        .filter(|(_, id, t)| keys(id, t).contains(&want))
        .collect();
    let hits = if exact.is_empty() {
        parts
            .iter()
            .filter(|(_, id, t)| !want.is_empty() && keys(id, t).iter().any(|k| k.contains(&want)))
            .collect()
    } else {
        exact
    };
    match hits.as_slice() {
        [(_, id, t)] => Ok((id.clone(), t.clone())),
        [] => Err(format!(
            "{kind} part {name:?}: no such part in parts/{kind}/ (have: {})",
            parts
                .iter()
                .map(|(_, id, _)| id.as_str())
                .collect::<Vec<_>>()
                .join(", ")
        )),
        many => Err(format!(
            "{kind} part {name:?} is ambiguous: {}",
            many.iter()
                .map(|(_, id, _)| id.as_str())
                .collect::<Vec<_>>()
                .join(", ")
        )),
    }
}

// --- resolving a configuration --------------------------------------------------

fn level_of(kind: &str, wanted: Option<&str>) -> Result<(&'static str, &'static str)> {
    let levels = KINDS.iter().find(|(k, _)| *k == kind).unwrap().1;
    let want = wanted.unwrap_or_else(|| DEFAULT_LEVEL.iter().find(|(k, _)| *k == kind).unwrap().1);
    levels
        .iter()
        .copied()
        .find(|(l, m)| *l == want || *m == want)
        .ok_or_else(|| {
            format!(
                "models.{kind}.level = {want:?}: one of {}",
                levels
                    .iter()
                    .map(|(l, m)| format!("{l} ({m})"))
                    .collect::<Vec<_>>()
                    .join(", ")
            )
        })
}

fn accepted(kind: &str, model: &str) -> Vec<&'static str> {
    match kind {
        "imu" => crate::imu::keys(model).0,
        "gnss" => crate::sensors::GNSS.keys(model),
        "barometer" => crate::sensors::BAROMETER.keys(model),
        _ => crate::sensors::MAGNETOMETER.keys(model),
    }
}

/// One value of a calibration profile.
pub struct Calibrated {
    pub key: String,
    pub value: Value,
    pub class: &'static str,
    pub how: String,
}

/// Read a calibration profile (docs/sensor-calibration.md) for one part at
/// one level. It names the part and level it was fitted for and gives
/// parameter values with an evidence class; the part file stays as published.
/// Returns the values and the report's `calibration` entry.
pub fn read_calibration(
    label: &str,
    kind: &str,
    part: &str,
    level: &str,
    path: &str,
    keys: &[&str],
) -> Result<(Vec<Calibrated>, Json)> {
    let full = {
        let p = PathBuf::from(path);
        if p.is_absolute() || p.exists() {
            p
        } else {
            crate::artifact::library_root().join(p)
        }
    };
    let text = std::fs::read_to_string(&full)
        .map_err(|e| format!("{label}.calibration = {path:?}: {e}"))?;
    let profile: Table = toml::from_str(&text)
        .map_err(|e| format!("{label}.calibration = {path:?}: {e}"))?;
    let head = profile
        .get("profile")
        .and_then(Value::as_table)
        .ok_or(format!("{label}.calibration: [profile] missing in {path}"))?;
    let field = |k: &str| head.get(k).and_then(Value::as_str).unwrap_or("");
    if field("kind") != kind {
        return Err(format!("{label}.calibration: the profile is for a {:?}, not a {kind}", field("kind")));
    }
    if field("part") != part {
        return Err(format!(
            "{label}.calibration: the profile was fitted for part {:?}, not {part:?}",
            field("part")
        ));
    }
    if !field("level").is_empty() && field("level") != level {
        return Err(format!(
            "{label}.calibration: the profile was fitted at level {:?}, not {level}",
            field("level")
        ));
    }
    let params = profile
        .get("parameters")
        .and_then(Value::as_table)
        .ok_or(format!("{label}.calibration: [parameters] missing in {path}"))?;
    let mut measured = Vec::new();
    let mut applied = serde_json::Map::new();
    let mut out = Vec::new();
    for (name, entry) in params {
        if !keys.contains(&name.as_str()) {
            return Err(format!("{label}.calibration: {name} is not a parameter of {level}"));
        }
        let t = entry
            .as_table()
            .ok_or(format!("{label}.calibration: parameters.{name} must be a table"))?;
        let value = t
            .get("value")
            .cloned()
            .ok_or(format!("{label}.calibration: parameters.{name}.value missing"))?;
        let class = t.get("class").and_then(Value::as_str).unwrap_or("measured");
        let class = EVIDENCE
            .iter()
            .copied()
            .find(|c| *c == class)
            .ok_or(format!("{label}.calibration: parameters.{name}.class = {class:?}: one of {}", EVIDENCE.join(", ")))?;
        let dataset = t.get("dataset").and_then(Value::as_str).unwrap_or("");
        if class == "measured" {
            if dataset.is_empty() {
                return Err(format!(
                    "{label}.calibration: parameters.{name} is measured but names no dataset"
                ));
            }
            measured.push(name.clone());
        }
        let how = t.get("how").and_then(Value::as_str).unwrap_or(match class {
            "measured" => "fitted to measurements",
            _ => "set by the calibration profile",
        });
        let note = if dataset.is_empty() {
            format!("calibration profile: {how}")
        } else {
            format!("calibration profile: {how} (dataset {dataset})")
        };
        out.push(Calibrated { key: name.clone(), value, class, how: note });
        let mut a = serde_json::to_value(t).unwrap_or(Json::Null);
        a["class"] = json!(class);
        applied.insert(name.clone(), a);
    }
    Ok((out, json!({
        "path": path,
        "synthetic": head.get("synthetic").and_then(Value::as_bool).unwrap_or(false),
        "label": field("label"),
        "unit": field("unit"),
        "created": field("created"),
        "measured": measured,
        "parameters": applied,
        "datasets": profile.get("datasets").map(|d| serde_json::to_value(d).unwrap_or(Json::Null)).unwrap_or(Json::Null),
    })))
}

/// The evidence label of a profile: datasheet-derived without measured values.
pub fn evidence_of(calibration: Option<&Json>) -> &'static str {
    match calibration {
        Some(c) if c["measured"].as_array().is_some_and(|m| !m.is_empty()) => {
            // A profile fitted to synthetic data demonstrates the pipeline; it
            // is not a measurement of a device and is never labelled as one.
            if c["synthetic"] == true { "synthetic-calibrated (not a device)" } else { "measurement-calibrated" }
        }
        _ => "datasheet-derived",
    }
}

fn apply_calibration(
    label: &str,
    kind: &str,
    part: &str,
    level: &str,
    path: &str,
    keys: &[&'static str],
    conv: &mut Conversion,
) -> Result<Json> {
    let (values, report) = read_calibration(label, kind, part, level, path, keys)?;
    for v in values {
        let key = *keys.iter().find(|k| **k == v.key).unwrap();
        conv.entries.retain(|e| e.key != key);
        conv.put_as(v.class, key, v.value, &[], v.how);
    }
    let measured: Vec<String> = report["measured"].as_array().unwrap().iter().filter_map(|m| m.as_str().map(String::from)).collect();
    if !measured.is_empty() {
        conv.assumed.retain(|a| {
            !(a.starts_with("pressure drift") && measured.iter().any(|m| m.starts_with("pressureDrift")))
        });
    }
    Ok(report)
}

/// Resolve one part use: the level, the parameters it fills (explicit keys
/// excluded) and the report.
pub fn resolve(
    kind: &str,
    selection: &Table,
    explicit: &Table,
) -> Result<(&'static str, Table, Json)> {
    let label = format!("models.{kind}");
    for k in selection.keys() {
        if !OPTIONS.contains(&k.as_str()) {
            return Err(format!(
                "{label}.{k}: with a part, the options are {}",
                OPTIONS.join(", ")
            ));
        }
    }
    let name = selection["part"]
        .as_str()
        .ok_or(format!("{label}.part: expected a string"))?;
    let (id, part) = find(kind, name)?;
    if let Some(k) = part.get("kind").and_then(Value::as_str) {
        if k != kind {
            return Err(format!("{label}.part = {name:?} is a {k}, not a {kind}"));
        }
    }
    let wanted = selection
        .get("level")
        .or(selection.get("model"))
        .map(|v| v.as_str().unwrap_or(""));
    let (level, model) = level_of(kind, wanted)?;
    let tolerances = selection
        .get("tolerances")
        .and_then(Value::as_str)
        .unwrap_or("none");
    if !["none", "draw"].contains(&tolerances) {
        return Err(format!("{label}.tolerances = {tolerances:?}: none or draw"));
    }
    let seed = explicit
        .get("seed")
        .and_then(Value::as_integer)
        .unwrap_or(1) as u64;
    let mut inputs = Inputs {
        part: &part,
        explicit,
        mode: selection
            .get("mode")
            .and_then(Value::as_str)
            .unwrap_or("")
            .into(),
        draw: (tolerances == "draw").then(|| Rng::new(seed, kind)),
        range: selection.get("range").and_then(Value::as_table),
        drift: selection
            .get("drift")
            .and_then(Value::as_str)
            .unwrap_or("")
            .into(),
    };
    if !inputs.drift.is_empty() && kind != "barometer" {
        return Err(format!("{label}.drift: only barometer parts choose a drift scenario"));
    }
    if !inputs.mode.is_empty() && !["gnss", "barometer"].contains(&kind) {
        return Err(format!(
            "{label}.mode: only gnss and barometer parts have modes"
        ));
    }
    if inputs.range.is_some() && kind != "imu" {
        return Err(format!("{label}.range: only IMU parts pick a range"));
    }
    let mut conv = match kind {
        "imu" => imu(&mut inputs)?,
        "gnss" => gnss(&mut inputs)?,
        "barometer" => barometer(&mut inputs)?,
        _ => magnetometer(&mut inputs)?,
    };
    let keys = accepted(kind, model);
    let calibration = match selection.get("calibration") {
        None => None,
        Some(v) => {
            let path = v
                .as_str()
                .ok_or(format!("{label}.calibration: expected a path"))?;
            Some(apply_calibration(&label, kind, &id, level, path, &keys, &mut conv)?)
        }
    };
    let mut params = Table::new();
    let mut derived = serde_json::Map::new();
    let mut used: BTreeSet<&str> = BTreeSet::new();
    let mut dropped: BTreeSet<&str> = BTreeSet::new();
    for e in &conv.entries {
        if !keys.contains(&e.key) {
            dropped.extend(e.from.iter().copied());
            continue;
        }
        used.extend(e.from.iter().copied());
        if explicit.contains_key(e.key) {
            derived.insert(
                e.key.into(),
                json!({"value": "explicit in the configuration", "from": e.from, "how": e.how}),
            );
            continue;
        }
        let mut entry = json!({"value": e.value, "from": e.from, "how": e.how});
        if let Some(class) = e.class {
            entry["class"] = json!(class);
        }
        derived.insert(e.key.into(), entry);
        params.insert(e.key.into(), e.value.clone());
    }
    used.extend(conv.checked.iter().copied());
    let mut not_represented = serde_json::Map::new();
    for f in dropped.difference(&used) {
        not_represented.insert(
            (*f).into(),
            json!(format!("not represented at {level} ({model})")),
        );
    }
    for (k, v) in &part {
        if METADATA.contains(&k.as_str())
            || k.ends_with("_mode")
            || k.ends_with("_setting")
            || used.contains(k.as_str())
            || not_represented.contains_key(k)
        {
            continue;
        }
        if conv.tolerances.contains(&k.as_str()) {
            not_represented.insert(
                k.clone(),
                json!("a tolerance: not applied (tolerances = \"draw\" draws one)"),
            );
        } else if let Some((_, why)) = conv.unused.iter().find(|(f, _)| f == k) {
            if !k.ends_with("_options_g") && !k.ends_with("_options_dps") || inputs.range.is_none()
            {
                not_represented.insert(k.clone(), json!(why));
            }
        } else if !(k.ends_with("_options_g") || k.ends_with("_options_dps")) {
            let _ = v;
            not_represented.insert(k.clone(), json!("not used by the converter"));
        }
    }
    let mut not_stated = serde_json::Map::new();
    for (figure, parameter) in &conv.missing {
        if keys.contains(parameter) && !explicit.contains_key(*parameter) {
            not_stated.insert(
                (*figure).into(),
                json!(format!(
                    "not stated: {parameter} stays at the library default"
                )),
            );
        }
    }
    let report = json!({
        "kind": kind, "part": id, "name": part.get("name").and_then(Value::as_str).unwrap_or(&id),
        "source": part.get("source").and_then(Value::as_str).unwrap_or(""),
        "level": level, "model": model,
        "mode": selection.get("mode").and_then(Value::as_str),
        "tolerances": tolerances,
        "parameters": derived, "assumed": conv.assumed, "not_represented": not_represented, "not_stated": not_stated,
    });
    let mut report = report;
    if !conv.unknown.is_empty() {
        report["unknown"] = json!(conv.unknown);
    }
    if kind == "barometer" {
        report["drift"] = json!(if inputs.drift.is_empty() { "relative_accuracy" } else { inputs.drift.as_str() });
    }
    report["evidence"] = json!(evidence_of(calibration.as_ref()));
    if let Some(c) = calibration {
        report["calibration"] = c;
    }
    Ok((model, params, report))
}

/// Replace every `part` selection of a configuration root by its level and
/// parameters. Returns the expanded root and one report per part.
pub fn expand(root: &Table) -> Result<(Table, Vec<Json>)> {
    let Some(models) = root.get("models").and_then(Value::as_table) else {
        return Ok((root.clone(), Vec::new()));
    };
    let mut out = root.clone();
    let mut reports = Vec::new();
    for (kind, _) in KINDS {
        let Some(sel) = models.get(kind).and_then(Value::as_table) else {
            continue;
        };
        if !sel.contains_key("part") {
            continue;
        }
        let explicit = root
            .get(kind)
            .and_then(Value::as_table)
            .cloned()
            .unwrap_or_default();
        let (model, params, report) = resolve(kind, sel, &explicit)?;
        let mut merged = explicit.clone();
        for (k, v) in params {
            merged.entry(k).or_insert(v);
        }
        let m = out.get_mut("models").unwrap().as_table_mut().unwrap();
        m.insert(
            kind.into(),
            Value::Table(Table::from_iter([(
                "model".into(),
                Value::String(model.into()),
            )])),
        );
        if merged.is_empty() {
            out.remove(kind);
        } else {
            out.insert(kind.into(), Value::Table(merged));
        }
        reports.push(report);
    }
    crate::powertrain::expand(&mut out, &mut reports)?;
    Ok((out, reports))
}

/// A part report as text: what each parameter is and where it came from.
pub fn summary(report: &Json) -> String {
    let mut out = format!(
        "{} -> {} ({}){}\n  source: {}\n",
        report["name"].as_str().unwrap_or(""),
        report["level"].as_str().unwrap_or(""),
        report["model"].as_str().unwrap_or(""),
        report["mode"]
            .as_str()
            .map(|m| format!(", mode {m}"))
            .unwrap_or_default(),
        report["source"].as_str().unwrap_or(""),
    );
    let section = |out: &mut String, title: &str, map: &Json| {
        if let Some(m) = map.as_object().filter(|m| !m.is_empty()) {
            out.push_str(&format!("  {title}:\n"));
            for (k, v) in m {
                out.push_str(&format!("    {k}: {}\n", v.as_str().unwrap_or("")));
            }
        }
    };
    out.push_str("  parameters:\n");
    if let Some(m) = report["parameters"].as_object() {
        for (k, v) in m {
            let value = match &v["value"] {
                Json::String(s) => s.clone(),
                other => other.to_string(),
            };
            let from = v["from"]
                .as_array()
                .map(|a| {
                    a.iter()
                        .filter_map(Json::as_str)
                        .collect::<Vec<_>>()
                        .join(", ")
                })
                .unwrap_or_default();
            out.push_str(&format!(
                "    {k} = {value}{}\n      {}{}\n",
                v["class"].as_str().map(|c| format!("  ({c})")).unwrap_or_default(),
                v["how"].as_str().unwrap_or(""),
                if from.is_empty() {
                    String::new()
                } else {
                    format!(" [{from}]")
                }
            ));
        }
    }
    if let Some(a) = report["assumed"].as_array().filter(|a| !a.is_empty()) {
        out.push_str("  assumed:\n");
        for x in a {
            out.push_str(&format!("    {}\n", x.as_str().unwrap_or("")));
        }
    }
    if let Some(a) = report["unknown"].as_array().filter(|a| !a.is_empty()) {
        out.push_str("  unknown:\n");
        for x in a {
            out.push_str(&format!("    {}\n", x.as_str().unwrap_or("")));
        }
    }
    if let Some(e) = report["evidence"].as_str() {
        out.push_str(&format!("  evidence: {e}\n"));
    }
    if let Some(a) = report["checks"].as_array().filter(|a| !a.is_empty()) {
        out.push_str("  checks:\n");
        for x in a {
            out.push_str(&format!("    {}\n", x.as_str().unwrap_or("")));
        }
    }
    section(
        &mut out,
        "not stated in the datasheet",
        &report["not_stated"],
    );
    section(
        &mut out,
        "datasheet figures not represented",
        &report["not_represented"],
    );
    out
}

/// The catalog's parts: per kind, id, name, manufacturer, source and the
/// figures stated.
pub fn catalog() -> Json {
    let mut out = serde_json::Map::new();
    for (kind, id, t) in all() {
        let entry = json!({
            "id": id,
            "name": t.get("name").and_then(Value::as_str).unwrap_or(&id),
            "manufacturer": t.get("manufacturer").and_then(Value::as_str).unwrap_or(""),
            "source": t.get("source").and_then(Value::as_str).unwrap_or(""),
            "default_level": DEFAULT_LEVEL.iter().find(|(k, _)| *k == kind).map(|(_, m)| level_of(&kind, Some(m)).unwrap().0),
        });
        out.entry(kind)
            .or_insert_with(|| json!([]))
            .as_array_mut()
            .unwrap()
            .push(entry);
    }
    Json::Object(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn part(text: &str) -> Table {
        toml::from_str(text).unwrap()
    }

    fn inputs<'a>(p: &'a Table, e: &'a Table) -> Inputs<'a> {
        Inputs {
            part: p,
            explicit: e,
            mode: String::new(),
            draw: None,
            range: None,
            drift: String::new(),
        }
    }

    #[test]
    fn gnss_cep_becomes_a_standard_deviation() {
        let p = part("horizontal_cep_m = 2.0\nvelocity_accuracy_mps = 0.05\nnav_rate_max_hz = 25");
        let e = Table::new();
        let c = gnss(&mut inputs(&p, &e)).unwrap();
        let bias = c.entries.iter().find(|e| e.key == "biasFactor").unwrap();
        let noise = c.entries.iter().find(|e| e.key == "noiseFactor").unwrap();
        let b = bias.value.as_array().unwrap()[0].as_array().unwrap()[0]
            .as_float()
            .unwrap();
        let n = noise.value.as_array().unwrap()[0].as_array().unwrap()[0]
            .as_float()
            .unwrap();
        assert!(((b * b + n * n).sqrt() - 2.0 / 1.1774).abs() < 1e-3);
        let v = noise.value.as_array().unwrap()[3].as_array().unwrap()[3]
            .as_float()
            .unwrap();
        assert!((v - 0.05 / 0.6745).abs() < 1e-3);
    }

    #[test]
    fn gnss_rate_above_the_parts_maximum_is_refused() {
        let p = part("horizontal_cep_m = 2.5\nnav_rate_max_hz = 5");
        let e = part("samplePeriod = 0.1");
        assert!(gnss(&mut inputs(&p, &e)).is_err());
    }

    #[test]
    fn imu_noise_density_scales_with_the_tick_rate() {
        let p = part("accel_noise_density_ug_rthz = 70.0\ngyro_noise_density_dps_rthz = 0.0028\n\
                      accel_full_scale_g = 16\ngyro_full_scale_dps = 2000\naccel_adc_bits = 16\ngyro_adc_bits = 16");
        let e = part("samplePeriod = 0.001\nadcDecimation = 2");
        let c = imu(&mut inputs(&p, &e)).unwrap();
        let n = c
            .entries
            .iter()
            .find(|e| e.key == "noiseStd")
            .unwrap()
            .value
            .as_array()
            .unwrap()
            .clone();
        let a = n[0].as_float().unwrap();
        assert!((a - 70e-6 * G * (2000.0f64 / 2.0).sqrt()).abs() < 1e-6);
        let q = c
            .entries
            .iter()
            .find(|e| e.key == "quantum")
            .unwrap()
            .value
            .as_array()
            .unwrap()[3]
            .as_float()
            .unwrap();
        assert!((q - 2.0 * 2000.0 * DEG / 65536.0).abs() < 1e-9);
    }

    #[test]
    fn tolerance_draws_are_deterministic_and_bounded() {
        let p = part("offset_mgauss = 10.0\nsensitivity_tolerance_pct = 2.0");
        let e = Table::new();
        let draw = || {
            let mut x = inputs(&p, &e);
            x.draw = Some(Rng::new(7, "magnetometer"));
            let c = magnetometer(&mut x).unwrap();
            c.entries
                .iter()
                .find(|e| e.key == "bias")
                .unwrap()
                .value
                .clone()
        };
        assert_eq!(draw(), draw());
        for v in draw().as_array().unwrap() {
            assert!(v.as_float().unwrap().abs() <= 10.0 * 1e-7);
        }
    }

    #[test]
    fn a_lower_level_reports_the_figures_it_cannot_represent() {
        let mut sel = Table::new();
        sel.insert(
            "part".into(),
            Value::String(
                directory()
                    .join("gnss/ublox_neo_m9n.toml")
                    .to_string_lossy()
                    .into(),
            ),
        );
        sel.insert("level".into(), Value::String("G0".into()));
        if !directory().join("gnss/ublox_neo_m9n.toml").exists() {
            return;
        }
        let (model, params, report) = resolve("gnss", &sel, &Table::new()).unwrap();
        assert_eq!(model, "ideal");
        assert!(params.is_empty());
        assert!(report["not_represented"]["horizontal_cep_m"]
            .as_str()
            .unwrap()
            .contains("G0"));
    }

    /// Every part file converts at every level, and a configuration naming
    /// parts composes with explicit keys winning.
    #[test]
    fn every_part_converts_and_composes() {
        for (kind, id, _) in all() {
            let Some((_, levels)) = KINDS.iter().find(|(k, _)| *k == kind) else {
                continue;
            };
            for (level, _) in *levels {
                let mut sel = Table::new();
                sel.insert("part".into(), Value::String(id.clone()));
                sel.insert("level".into(), Value::String((*level).into()));
                resolve(&kind, &sel, &Table::new())
                    .unwrap_or_else(|e| panic!("{kind} {id} {level}: {e}"));
            }
        }
        let c = crate::config::parse(
            "schema_version = 3\nvehicle = \"rover\"\n[models.gnss]\npart = \"m9n\"\n[gnss]\nseed = 9\n\
             [models.barometer]\npart = \"bmp388\"\n[models.imu]\npart = \"bmi088\"\n",
        );
        let Ok(c) = c else {
            return assert!(all().is_empty(), "{c:?}");
        };
        assert_eq!(c.gnss["seed"].as_integer(), Some(9));
        assert_eq!(c.imu_model(), "errors");
        assert_eq!(c.parts.len(), 3);
    }
}
