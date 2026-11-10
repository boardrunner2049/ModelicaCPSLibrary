# Sensor calibration and validation

This is how a COMPOSE sensor model goes from a real part's datasheet to a
calibration fitted to measurements of one unit, with the evidence for every
value recorded. The pilot is the Infineon DPS310 barometer at level B1
(`Systems.Sensing.Barometer.SampledSensor`). The same workflow is meant to
cover the analog power module, the RPM sensor and the existing IMU next.

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

```sh
python3 -m venv build/venv-calib && build/venv-calib/bin/pip install fmpy scipy numpy
cd tools/calibration
PATH=<without ~/bin> ../../build/venv-calib/bin/python -m cp_calibration toolchain
../../build/venv-calib/bin/python -m cp_calibration datasheet infineon_dps310 [--drift none] [--calibration F]
../../build/venv-calib/bin/python -m cp_calibration run experiments/dps310_synthetic.toml
#   -> build/calibration/dps310_synthetic/{report.md,report.json,dps310_synthetic.profile.toml,data/}
```

To run the tests, use `python3 tools/test_calibration.py` for the statistics,
datasets and composer profiles. With the venv's Python, `FIRE_TEST_FMU=1`
adds the toolchain check and `FIRE_TEST_CALIBRATION=1` adds the whole pilot.

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

- **Frozen seeds (fixed for the barometer).** Rumoca evaluates start values at
  compile time, so a noise stream seeded through
  `rngState(start = initialState(streamId, seed))` ignores `seed` and
  `streamId` set in the FMU. They are declared tunable, but they are inert.
  - The barometer now seeds at its first acquisition through
    `Systems.Sensing.seededNormalDraw`, written with whole-array assignments
    and scalar conditionals because Rumoca generates no code for array
    slices or branch-local definitions.
  - Its draws are bit-identical to the old seeding (checked against the
    previous FMU).
  - **The other sensors that seed the same way still have frozen seeds in
    exported FMUs.** A seed set by the composer in the Modelica source works.
- **Structural parameters are fixed in the FMU.** `latencySamples` is exported
  as a fixed calculated parameter. The harness refuses to set it and exports
  one variant per value instead.
- **No event mode.** The FMU has no Co-Simulation event mode
  (`hasEventMode = false`). Sample events inside a step are handled
  internally. A consumer sees the latest delivery per communication point;
  `sequence` makes missed readings countable. Edge-level event exposure for
  the RPM sensor must be designed against this.
- **Rumoca reads every `.mo` file under the library root.** Keep generated
  Modelica out of the tree. The harness writes its wrappers to a temporary
  directory.

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
