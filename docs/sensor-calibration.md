# Sensor calibration and validation

This is how a COMPOSE sensor model goes from a real part's datasheet to a
calibration fitted to measurements of one unit, with the evidence for every
value recorded. It covers four cases:

| sensor | models | experiment | data |
|---|---|---|---|
| Infineon DPS310 barometer (the pilot) | B1 `Systems.Sensing.Barometer.SampledSensor` | `dps310_synthetic.toml` | synthetic |
| Analog power module, Holybro PM02 V3 as the reference configuration | A1–A3 `Systems.Sensing.PowerModule.*`; vehicle level W2 | `power_pm02_synthetic.toml` | synthetic bench |
| Pulse RPM sensor, Melexis US5881 on a 12-magnet ring | E1 `Systems.Sensing.Rpm.PulseSensor`; vehicle level R2 | `rpm_us5881_synthetic.toml` | synthetic edges |
| Existing IMU, reuse case | I1 `Systems.Sensing.IMU.StatisticalSensor` | `imu_miluv_static.toml` | public MILUV static recordings (CC BY 4.0) |
| DPS310 noise against oversampling | the part's datasheet figures | `python -m cp_calibration chambers` | public Causal Chambers wind tunnel, four DPS310 units (CC BY 4.0) |

The power module and RPM references are named configurations, not the lab's
hardware, which is still to be identified. Two cases use real measurements:
- **IMU:** its chips are not identified, so it tests the workflow and
  validates no part.
- **DPS310 (Causal Chambers):** real DPS310 units, but it tests only the
  per-reading noise figures.

The bench plans for the lab's hardware are in
[sensor-bench-plans.md](sensor-bench-plans.md).

COMPOSE owns the sensor's behaviour and its timing. It exposes them through
the model and FMU interfaces. FastDyn decides how those outputs reach
firmware. Nothing here depends on ArduPilot's JSON interface.

## Evidence classes

Every parameter value the composer fills from a part, and every entry in a
calibration report, carries one class:

| class | meaning | DPS310 example |
|---|---|---|
| specified | stated by the datasheet under its stated conditions | `pressureNoiseStd` = 0.2 Pa, 64x mode (Table 7, Table 16) |
| derived | calculated from specified figures with a stated equation | `samplePeriod` = 104.4 ms + 3.6 ms = 0.108 s |
| measured | estimated from a named dataset | an offset fitted to a bench record |
| assumed | chosen without sufficient information | drift σ = relative accuracy / 2, τ = 300 s |
| configuration | a modelling choice of the user | `seed`, an explicit `samplePeriod` |

Quantities no figure determines are also listed as **unknown** in the report,
even when the model runs on an assumed value. For the DPS310 these are:
- the drift process over time;
- the delivery latency;
- the temperature noise;
- this unit's offsets.

A min/max or "±" tolerance is not a distribution. `tolerances = "draw"` draws
an offset uniformly within it, and the report records that uniformity as
assumed.

A profile carries one of three evidence labels:
- **Datasheet-derived:** the part's figures plus stated conversions and
  assumptions.
- **Measurement-calibrated:** some parameters fitted to named measurements of
  a unit.
- **Synthetic-calibrated (not a device):** a profile fitted to generated
  data. It demonstrates the pipeline, and the composer never labels it
  measured.

A "validated" claim names the predictions that passed independent held-out
tests, and the conditions they were tested under. A profile can be calibrated
for one property and unvalidated for another.

## The DPS310 datasheet audit

Audited against V1.2, 2020-10-15 (sha256
`c856826f913193c53a83fe73bc853b7517d36a2c5a75d1de19e701c1a88e0d00`). The
part file now cites table and page for each figure. What changed:

- **Typical values, not bounds.** Accuracy and noise sit in Table 7's "Typ."
  column, measured at 1000 hPa and 25 °C: ±100 Pa absolute, ±6 Pa relative,
  0.2 Pa RMS. They are not guaranteed bounds, and the datasheet gives no
  distribution for them.
- **Relative accuracy is not drift.** It is the error of "any Δ1 hPa in
  800–1200 hPa at any constant temperature in 20–60 °C". That is a
  pressure-step error, not a drift over time. The converter keeps
  σ = 3 Pa, τ = 300 s for compatibility, but both are now classed as
  *assumed*. `[models.barometer] drift = "none"` selects a no-drift scenario
  instead, and a calibration profile can supply a measured drift.
- **Pressure and temperature share the measurement time.** The note to
  Table 17 (p30) requires rate_T·time_T + rate_P·time_P < 1 s. B1 produces a
  pressure and a temperature every sample, so a reading needs 104.4 ms
  (64x pressure) plus 3.6 ms (single-oversampled temperature, the TMP_CFG
  default, p31).
  - The default period is now 0.108 s; it was 0.1044 s.
  - Shorter explicit periods are refused.
  - Table 16 now sits in the part file, as `pressure_oversampling`,
    `pressure_measurement_time_ms` and `pressure_precision_pa`.

## Timing contract

Every barometer level outputs, in simulation time:
- `sampleTime`: when the delivered reading was acquired;
- `deliveryTime`: when it became available, i.e. acquisition plus
  `latencySamples · samplePeriod`;
- `sequence`: the count of delivered readings, which increments exactly once
  per new reading;
- `valid`.

Before the first delivery, `valid` is false, `sequence` is 0 and both times
are -1. B0 is continuous: it is always valid, both times equal `time`, and
`sequence` is 0. Starting a simulation, or resetting an FMU, restarts the
acquisition clock, the delivery queue and the noise streams.

`SensorSuite` passes these through as `barometer*`. Each vehicle passes them
on, and every vehicle FMU exports them as the optional capability
`barometer_timing_v1`:
- `baro_sample_time_s`
- `baro_delivery_time_s`
- `baro_sequence`
- `baro_valid`

These capabilities are in `contract/vehicle_interface.toml`. A consumer that
steps coarser than the sample period sees the latest reading, and counts the
readings it missed from jumps in `baro_sequence`. Existing consumers are
unaffected: the capability is optional and `barometer_v1` is unchanged.

## Calibration profiles

A profile is a TOML file kept beside the part, never merged into it. Select it
explicitly in a configuration:

```toml
[models.barometer]
part = "infineon_dps310"
calibration = "path/to/unit.profile.toml"
```

You can also pass `fire-compose part barometer infineon_dps310 --calibration FILE`.

```toml
[profile]
kind = "barometer"
part = "infineon_dps310"      # must match the selected part
level = "B1"                  # and level
unit = "serial or batch"
label = "what this profile is"
synthetic = false             # true: labelled synthetic, never measured
[datasets.bench1]             # names, hashes, roles, conditions
[parameters]
pressureBias = { value = 11.5, class = "measured", dataset = "bench1", interval95 = [10.9, 12.1] }
[not_identified]              # parameters the data did not bound (informational)
```

The rules:
- A `measured` entry must name its dataset.
- Values not in the profile keep their datasheet-derived value and class.
- Explicit `[barometer]` keys still win.

The manifest's `provenance` table lists the barometer's parameters with their
origin and evidence.

## The harness

Code lives in `tools/calibration/cp_calibration`:

| module | role |
|---|---|
| `fmu.py` | cached Rumoca FMI 3 export (key: library sources, wrapper, compiler); `Slave`, one FMPy instance reset between runs, with zero-order-hold inputs and communication points at every input breakpoint |
| `dataset.py` | readings CSV + reference CSV, on separate time bases, + JSON description; SHA-256 recorded, changed files refused |
| `barometer.py` | the datasheet report via `fire-compose part --json`; structural variants; least squares through the FMU; latency by profile over compiled variants |
| `stochastic.py` | exact Kalman likelihood of white noise + first-order Gauss-Markov drift, profile-likelihood intervals, Allan deviation, Ljung-Box |
| `pipeline.py` | toolchain check, synthetic generation, staged fit, identifiability and mismatch cases, held-out evaluation, profile and report |
| `power.py`, `rpm.py`, `imu.py` | the power-module, RPM and IMU pipelines |
| `crosscheck.py` | the same test models simulated by Rumoca and by OpenModelica, compared |
| `chambers.py` | DPS310 noise on the Causal Chambers wind tunnel data |

```sh
python3 -m venv build/venv-calib && build/venv-calib/bin/pip install fmpy scipy numpy
cd tools/calibration
PATH=<without ~/bin> ../../build/venv-calib/bin/python -m cp_calibration toolchain
../../build/venv-calib/bin/python -m cp_calibration datasheet infineon_dps310 [--drift none] [--calibration F]
../../build/venv-calib/bin/python -m cp_calibration run experiments/dps310_synthetic.toml
#   -> build/calibration/dps310_synthetic/{report.md,report.json,dps310_synthetic.profile.toml,data/}
```

To run the tests, use `python3 tools/test_calibration.py` for the statistics,
datasets and composer profiles. With the venv's Python, these flags add
further tests:
- `FIRE_TEST_FMU=1`: the toolchain checks;
- `FIRE_TEST_FMU=1 FIRE_TEST_OMC=1`: the compiler cross-check;
- `FIRE_TEST_CALIBRATION=1`: the pilots, and the pipelines on recorded files;
- `FIRE_TEST_MILUV=1`: the MILUV IMU reuse;
- `FIRE_TEST_CHAMBERS=1`: the Causal Chambers comparison.

**Recorded data.** A `[[datasets]]` entry with `file`, `sha256` and
`reference_sha256` is read from disk rather than generated. `python -m
cp_calibration hash <file>` prints the two hashes. An experiment without
`[truth]` skips same-model recovery, and `[mismatch]` becomes optional. See
[sensor-bench-plans.md](sensor-bench-plans.md) for the layout.

### Workflow

1. **Part and mode.** The composer's datasheet report gives the starting
   profile, the class of each value, and the unknowns.
2. **Experiment.** An experiment TOML names the datasets, their roles (train
   or holdout) and the fit settings. A separate thresholds file holds the
   acceptance criteria with their rationale. The pipeline reads and hashes the
   thresholds before it evaluates any held-out data.
3. **Deterministic stage.** Run with noise, drift and quantization off.
   - Fit the offset by least squares through the FMU.
   - `latencySamples` sizes the delivery queue, so Rumoca fixes it at compile
     time. It is fitted by refitting at each compiled candidate and comparing
     costs.
   - Fit the temperature offset on the temperature channel.
   - The Jacobian's singular values flag directions the data cannot resolve.
4. **Stochastic stage.** On the residuals of a static record, the offset,
   white noise, drift amplitude and drift time are fitted jointly by maximum
   likelihood, with quantization as q²/12.
   - Each parameter gets a 95 % profile-likelihood interval.
   - A parameter whose interval reaches its search range is reported *not
     identifiable* and left out of the profile.
   - An interval spanning more than a decade is reported *weakly identified*.
5. **Held-out evaluation.** The calibrated and datasheet-derived profiles are
   both scored on the same held-out sets: mean error, log-likelihood per
   reading under each profile's own noise model, innovation spread, and
   Ljung-Box whiteness.
6. **Report.** Records the device and mode, toolchain versions, the
   datasheet profile, datasets with hashes, thresholds, both stages,
   same-model recovery, the identifiability and mismatch cases, the held-out
   comparison, the profile, and the limitations.

### What the synthetic pilot tests

The synthetic data tests the implementation and the experiment design. It does
not test a DPS310.

- **Same-model recovery:**
  - data generated by B1 itself, with an offset of 12 Pa, latency 2,
    0.2 Pa noise and a 3 Pa / 60 s drift;
  - the truth must lie in each identifiable interval.
- **Unidentifiable cases, which must be flagged:**
  - `pressureBias` and `altitudeBias` from the altitude output alone (one
    combination, rank-deficient Jacobian);
  - a 300 s drift time from a 30 s record.
- **Model mismatch:**
  - data from `models/BarometerTemperatureOffset.mo` (B1 plus the DPS310's
    0.5 Pa/K temperature sensitivity), fitted with B1;
  - the training residual must show the temperature structure, and the
    held-out record at 45 °C must fail the thresholds.

## Toolchain findings (Rumoca 0.10.1, FMPy 0.3.32)

- **Frozen seeds (fixed everywhere).** Rumoca evaluates start values at
  compile time, so a noise stream seeded through
  `rngState(start = initialState(streamId, seed))` ignored `seed` and
  `streamId` set in the FMU. They were declared tunable, but they were inert.
  - Every sensor now seeds at its first draw through
    `Systems.Sensing.seededState` (the barometer through `seededNormalDraw`).
    That function uses whole-array assignments and scalar conditionals,
    because Rumoca generates no code for array slices or branch-local
    definitions.
  - On the exported FMUs, every model's default-seed output is bit-identical
    to before, and a different seed now changes it.
- **Buffer shifts in state events.** In a state-event `when`, Rumoca rejects a
  for-loop shift `b[j] = pre(b[j-1])` ("invalid array subscript"), although it
  accepts the same loop under a time event. `b = cat(1, {x}, pre(b[1:n-1]))`
  works.
- **The integrator is adaptive.** A first-order model with τ = 1 ms stays
  within 2 µV of the analytic step response at communication steps up to
  20 ms.
- **Structural parameters are fixed in the FMU.** `latencySamples` is exported
  as a fixed calculated parameter. The harness refuses to set it and exports
  one variant per value instead.
- **No event mode.** The FMU has no Co-Simulation event mode
  (`hasEventMode = false`). Sample and state events inside a step are located
  internally. A consumer sees the values at its communication points:
  - for sampled sensors, `sequence` makes missed readings countable;
  - for the RPM sensor, the count plus the edge buffer carry every edge (see
    below).
- **A random draw in a state-event clause lagged by one event (fixed).**
  In `PulseSensor`, the jitter draw `(normal, rngState) = normalDraw(...)`
  and its use `lastEdgeTime = time + ... + edgeJitterStd*normal[1]` were two
  equations in a `when cycles >= ...` clause. Rumoca's FMU gave each edge the
  previous edge's draw, and the first edge none; OpenModelica gave the
  current one.
  - **Check:** OpenModelica matched an independent run of the MSL
    `ModelicaRandom.c` generator draw for draw.
  - **Fix:** the draw and its use are now one equation (`Rpm.reportedEdge`).
  - **Effect on results:** the jitter's distribution was unaffected, so the
    earlier RPM estimates stand statistically.
  - **Not affected:** sample-clocked sensors (barometer, magnetometer,
    `Sampler`) agree between the two compilers to round-off.
- **Cross-check against OpenModelica** (`python -m cp_calibration
  crosscheck`). It covers power-module A1–A3, the pulse sensor with jitter,
  the barometer with noise, drift and quantization, `Sampler`, and the
  magnetometer. Agreement:
  - continuous outputs within 1e-7 relative;
  - edge and sample times within 1e-13 s;
  - counts and sequences exact.
  - The composed W2 and R2 vehicles pass OpenModelica `checkModel`.
- **Rumoca reads every `.mo` file under the library root.** Keep generated
  Modelica out of the tree. The harness writes its wrappers to a temporary
  directory.

## Analog power module (milestone 4)

The sensor ends at its two analog outputs. The autopilot's ADC and scaling
(ArduPilot BATT_VOLT_MULT, BATT_AMP_PERVLT, BATT_AMP_OFFSET) are the
consumer's.

**Models** (`Systems.Sensing.PowerModule`). Inputs are the true voltage and
current, plus the module temperature; outputs are `voltageSense` and
`currentSense` in volts, clipped at the rails.

| level | class | what it represents |
|---|---|---|
| A1 | `AnalogSensor` | gain and offset per channel; optional current-gain temperature coefficient |
| A2 | `FirstOrderAnalogSensor` | A1 plus a first-order output response per channel |
| A3 | `ShuntAnalogSensor` | component level: divider, shunt (temperature coefficient, self-heating through a thermal resistance and capacitance), amplifier |

From terminal measurements only the product amplifier gain × shunt
resistance is identifiable, not the two factors.

**In a vehicle:** `[peripherals.power_module] level = "W2"` (`AnalogPowerModule`).
- **Input:** the pack's true voltage and current, from the plant.
- **Chain:** A1, then an ADC (`adcReference`, `adcBits`), then the autopilot's
  scaling (`voltageMultiplier`, `ampsPerVolt`, `ampsOffset`). The result is
  `battery_v1`.
- **Exports:** the analog outputs, as `power_module_analog_v1`
  (`battery_voltage_sense_v`, `battery_current_sense_v`).
- **Default level:** a power-module part without a level still selects W1.

**PM02 V3 conversion (W2):**
- gains are *derived* (1/18.182 V/V, 1/36.364 V/A);
- the autopilot's scales and the 3.3 V range are *specified* (from the setup
  documentation);
- the 12-bit ADC and the zero offsets are *assumed*, because no figure states
  them.

**Pipeline** (`cp_calibration.power`). Bench data is the true V/I as a
reference plus a logger's samples of the two outputs, taken off the stimulus
clock. The logger is simulated: 1 mV rms noise, 16 bits.
- Static gains and offsets are fitted through the A1 FMU.
  - **Two standard errors are reported:** one assuming independent residuals,
    and a Newey–West (HAC) one that allows for autocorrelation, with the
    residuals' lag-1 correlation.
  - **The profile carries the larger.** A real logger's filtering or slow
    drift would otherwise make the independent ones optimistic.
- The output time constant is fitted from a 20 kHz current step (A2), then
  refined jointly with the gain and offset.
- **Identifiability:** A3's amplifier gain and shunt resistance must be
  flagged structural.
- **Mismatch:** a self-heating copper-trace shunt (A3), fitted with A1. The
  residual must trend within constant-current dwells, and a sustained 60 A
  hold must fail.
- **Held-out:** a flight-like current profile with pack sag, and a voltage
  staircase at 35 °C.

## Pulse RPM sensor (milestone 5)

**Model** (`Systems.Sensing.Rpm.PulseSensor`, E1). Shaft speed is integrated
into a pulse phase (`cycles`), so edges stay phase-continuous as speed
changes.
- **Edges:** rising edge k is the state event `cycles = k`.
- **Outputs:**
  - `count`: exact;
  - `level`: high for `dutyCycle` of each period;
  - `lastEdgeTime` and `edgeTimes[bufferSize]`: newest first, including
    `edgeDelay` and seeded `edgeJitterStd`;
  - `rpm`: from the latest two edges, 0 after `timeout`.
- **Not modelled:** missed pulses and the device's switching limit.

**Event contract.** A consumer reading every h seconds recovers every edge
since its previous read.
- **New edges:** `count - previous`, timed by the first that many entries of
  `edgeTimes`.
- **Overflow:** more than `bufferSize` edges in a step keeps the count exact
  but loses the older times, and the consumer detects it.
- **Demonstrated on the exported FMU** at 6000 rpm × 12 pulses/rev (1200
  edges/s), and accelerating from 1000 to 11000 rpm, at consumer steps of 0.5
  to 50 ms:
  - counts are exact;
  - recovered edge times are within 1e-13 s of the analytic ones;
  - overflow is detected exactly when a step spans more than 32 edges.
- **Edge cases:** coincident edges cannot occur (one per pulse period). A
  reset restarts the phase.

**In a vehicle:** `[peripherals.rpm] level = "R2"` (`PulseRpm`), per shaft.
It exports `rpm_pulses_v1`:
- `rpm_pulse_count[n]`;
- `rpm_last_edge_time_s[n]`;
- `rpm_edge_times_s[n·bufferSize]`, flattened shaft-major, because the
  contract is one-dimensional.

**Pipeline** (`cp_calibration.rpm`). Reported edge times, as the consumer
recovers them, are compared with a reference encoder's true edges.
- Edge delay is the mean of reported minus reference; jitter is its spread.
- **Identifiability:** delay and an unknown clock offset between sensor and
  encoder must be flagged structural.
- **Mismatch:** a constant *angular* lag fitted as a time delay. The delay
  must scale with the pulse period across training speeds and fail at 12000
  rpm.
- **Requires edge-level references:** sampled rpm cannot establish jitter or
  latency.

## IMU reuse (milestone 6)

`cp_calibration.imu` runs the same stochastic workflow on MILUV's 16 h static
recordings: Uvify IFO-S ifo001, Pixhawk 4 IMU through PX4 and RealSense D435i
IMU, both at 200 Hz.
- **Getting the data:** the harness reads the one vehicle's bag out of
  `imu_calib.zip` with HTTP range requests (1.7 GB of 5 GB) into
  `build/calibration/datasets/miluv/`. It records the hashes and the CC BY 4.0
  attribution.
- **Split, declared before analysis:** the first hour is dropped as warm-up,
  1–8.5 h is training, 8.5–16 h is held out.
- **Per channel, it reports:**
  - the white noise density and a Gauss-Markov bias, from 1 s means;
  - a profile interval for the bias time;
  - the per-sample spread and lag-1 correlation, which show the PX4 filtering;
  - the held-out Allan deviation against the fitted model's 5–95 % band;
  - a refit on the held-out window (stationarity);
  - MILUV's own `allan_variance_ros` noise density beside it.
- **Reuse:** the fitted values then configure the existing I1 IMU
  (`StatisticalSensor`), whose static output's Allan deviation is compared
  with the fitted model's.
- **No part validated:** no COMPOSE part is established for these chips, so
  the result is an explicit `[imu]` configuration snippet, not a part
  overlay.

**What the MILUV run found** (`build/calibration/imu_miluv_static/report.md`):
- **The streams are not what the configs say.** MILUV's configs give 200 Hz;
  the PX4 stream averages 213 Hz with mixed 4 and 8 ms intervals (2 million
  gaps over 1.5 periods), and the camera stream has 194 non-increasing
  stamps. The analysis therefore bins by time, not by sample count.
- **The PX4 stream is filtered.** Its lag-1 correlation is 0.2–0.6, and its
  per-sample spread is below the white-noise equivalent. Its gyro z reads a
  0.355 rad/s offset while static (not bias-compensated). The camera stream
  looks white.
- **Noise densities agree with MILUV's own `allan_variance_ros`:**
  - camera, per axis: within 1–10 % (gyro) and 4–20 % (accelerometer);
  - PX4: ours are 15–60 % lower, consistent with the filtering.
- **The held-out check fails on every channel**, with only 6–39 % of cluster
  times in the fitted model's band, against the predeclared 80 %:
  - the values drift between the two 7.5 h halves (see the refit column);
  - short-τ ratios are 0.87–1.02, but the band there is a few percent wide;
  - the slow component differs by up to ×3;
  - most PX4 bias times are longer than the window, so their intervals are
    unbounded.

  One white-noise-plus-Gauss-Markov fit from one window therefore does not
  predict another window of the same unit. The workflow reports this rather
  than hiding it. The fix is repeated recordings, with the variation between
  them reported (as in Bakalli et al.).
- **Reuse works.** The fitted values in the existing I1 model
  (`StatisticalSensor`) reproduce the fitted model's Allan deviation within
  ±20 % from 1 to 180 s, except where a 30-minute run gives about ten
  clusters.
- **The bag's barometer topic is rejected.** `/mavros/imu/static_pressure`
  sits at about 198 kPa, twice atmospheric, and its consecutive samples
  spread 16 Pa, against the MS5611's 1.2–6.5 Pa. It cannot be attributed to
  the Pixhawk 4's MS5611, so no barometer noise is reported from it. The
  screen was written after a first look at the level.

## DPS310 noise on public data (Causal Chambers)

The Causal Chambers wind tunnel (Gamella, Peters and Bühlmann, 2025; data
CC BY 4.0) carries four DPS310s. Its firmware reads them with Infineon's
library at 1×–8× oversampling. Dataset `wt_test_v1`, experiment
`osr_barometers`, holds 100 readings of all four at 1×, then 100 at 8×.
`cp_calibration.chambers` downloads the 2.6 MB archive and checks its
published MD5. It then reports each unit's noise after removing a linear
trend, with chi-square intervals.

| oversampling | datasheet (Table 16) | units (Pa) | ratio |
|---|---|---|---|
| 1× (the composer's `fast` mode) | 2.5 Pa | 1.54, 1.54, 1.62, 2.12 | 0.62–0.85 |
| 8× | 0.4 Pa | 0.25, 0.26, 0.29, 0.29 | 0.61–0.72 |

- **Datasheet figures are conservative.** All four units are at or below
  them; the figures behave as typical values.
- **Scaling matches.** The pooled 8×/1× ratio (0.157) matches the
  datasheet's (0.160). Independent averaging alone would give 0.354.
- **Composer default.** The `fast` mode default (2.5 Pa) is therefore
  conservative for these units.
- **Not tested:** absolute accuracy, drift, the temperature coefficient,
  latency and 64×. There is no reference instrument, and each setting is
  only 15 s.
- **Shared variation.** The units share some variation (flow or ambient),
  which the per-unit figures include.
- **Thresholds.** The comparison was written after the data was first
  examined, so it reports ratios, not a pass/fail verdict.

## Data a real calibration needs

Without the data below, a DPS310 profile stays datasheet-derived. The
synthetic profile is not a substitute.

- **Static holds and noise.** A stable pressure source, or a sealed chamber
  with a calibrated reference (uncertainty well below the 6 Pa relative
  accuracy for offset claims), with temperature logged. Hold for at least
  10× the drift time being resolved, as several separate runs, and keep one
  run or condition for validation.
- **Steps and latency.** Pressure steps with the reference logged on its own
  clock, and the sensor's data-ready or delivery timestamp. The timestamp
  must say what it represents and how uncertain it is.
- **Temperature dependence.** Static holds at two or more controlled
  temperatures, before any claim about the 0.5 Pa/K figure.
- **For every dataset.** The unit, the oversampling mode, the supply, the
  mounting, and any filtering or downsampling in the logger.

A second barometer helps as a comparison, but it is not ground truth.
