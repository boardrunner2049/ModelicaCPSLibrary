# Bench capture plans: power module and RPM sensor

What to record so `tools/calibration` can calibrate and validate the analog
power module and the pulse RPM sensor on a real unit. The DPS310 plan is in
[sensor-calibration.md](sensor-calibration.md#data-a-real-calibration-needs).
Each plan says what the data must contain, how the files must look for the
harness to read them, and which numbers to fix before the held-out data is
looked at.

## Common to both

- **Identify the unit.** Record the part number, the board revision, the
  serial number or a label, and a photo. Without the part number, a profile
  attaches to the reference part (PM02 V3, US5881). The report then says so,
  and the result is a configuration, not a part validation.
- **Name the reference instrument.** Record its make, model, range,
  calibration date and stated uncertainty. The uncertainty enters the
  thresholds.
- **Use one clock, or measure the offset.** The reading and the reference are
  logged by the same logger. If they aren't, measure the offset between the
  two clocks and state it as `reference.clock_offset_s`. Never align the
  streams by the latency being estimated: the delay and an unknown offset are
  not identifiable together (the harness flags this).
- **Split before fitting.** Keep at least one run per plan for validation, at
  a condition the training runs don't cover (another temperature or speed).
  Write the thresholds file before looking at that run. The DPS310 pilot
  shows what that costs: its held-out step check fails at 2.8 Pa against a
  predeclared 2.5 Pa, and the threshold stays.
- **Never edit raw files.** Corrections go in the description. A changed
  dataset gets a new name.

**File layout** (`cp_calibration/dataset.py`): per run, `<name>.csv` (the
readings as logged), `<name>.reference.csv` (the reference on its own time
base, a zero-order hold), and `<name>.json` (`kind`, columns, units, what
each timestamp means, the unit, the reference instrument and its
uncertainty, conditions, `reference.clock_offset_s`).

An experiment names each recorded run by file and hashes. `python -m
cp_calibration hash <file>` prints the hashes:

```toml
[[datasets]]
name = "bench_current_25c"
role = "train"                 # or "holdout"
tags = ["current"]             # power module: voltage / current / step
file = "data/bench_current_25c"
sha256 = "…"
reference_sha256 = "…"
```

An experiment with no `[truth]` is a recorded experiment. The harness skips
same-model recovery. The identifiability and mismatch checks run on
simulated data, and are optional there (leave out `[mismatch]`).

## Analog power module

**Goal.** Fit A1 (gain and offset per channel) and the A2 output time
constant, and test whether A1 holds at another temperature and under a
flight-like load.

**Equipment:**
- A programmable DC supply covering the pack range.
- An electronic load to the module's rated current.
- A reference DMM on the battery voltage.
- A reference current measurement:
  - a calibrated shunt with a DMM, or
  - a clamp meter, which is only good enough if its uncertainty is well
    below the module's (about 1 % of reading).
- A logger on the two sense outputs, at least 16 bits and at least 1 kHz for
  the step runs. Characterize the logger's own noise and offset with its
  inputs shorted, and log that as its own run.
- A thermocouple on the module, near the shunt.

**Readings:** `time_s, voltage_sense_v, current_sense_v`.
**Reference:** `time_s, voltage_v, current_a, temperature_k`.

| run | role | stimulus | why |
|---|---|---|---|
| shorted inputs | logger characterization | 60 s | logger noise and offset, which go into the thresholds |
| voltage staircase | train, `voltage` | 6 → 50 V (or the module's max), 12 steps, 2 s dwells, about 1 A load | voltage gain and offset |
| current staircase | train, `current` | 0 → rated current, 14 steps, 2 s dwells, nominal pack voltage | current gain and offset; the top step past the rail checks clipping |
| zero current | train, `current` | 5 min at 0 A | the zero-current offset, which no Holybro document states |
| current steps | train, `step` | 10 % ↔ 80 % rated current, square wave, logged at ≥ 10 kHz | the output time constant (A2). W2 is static, so the value is reported but not used in a vehicle |
| sustained high current | train | 80 % rated current for 5 min | self-heating: a trend within the dwell means A1 is too simple (the synthetic mismatch case shows this) |
| flight profile | holdout | a recorded flight's current trace replayed on the load, with pack sag | held-out check at 25 °C |
| staircase at 35–40 °C | holdout | the voltage and current staircases with the module warmed | temperature dependence |

**Thresholds** (write them before the holdout). Hold-out mean and rms error
per channel are bounded by the logger noise plus the reference uncertainty,
converted through the gain. A temperature tolerance should come from the
reference's datasheet or from the 25 °C run, not from the warm run.

## Pulse RPM sensor

**Goal.** Measure the edge delay and the jitter of the reported edge times,
check that the count is exact across the speed range, and check whether the
delay is a time or an angle.

**Equipment:**
- A motor on a test stand with the lab's magnet ring, or slotted disk, and
  the Hall sensor mounted as on the vehicle.
- A reference encoder or optical gate on the same shaft. It must have
  edge-level timestamps and a known pulse-to-magnet alignment.
- A logic analyzer or timer-capture logger, at ≥ 10 MHz, recording the Hall
  output and the reference on one clock. That gives one clock, so
  `clock_offset_s = 0`.
- Optionally, the autopilot's own capture of the same Hall line. That is the
  consumer the model serves, and it adds the autopilot's latency.

**Readings:** `time_s` (reported edge time), `edge` (index),
`reference_time_s` (the matched reference edge).
**Reference:** `time_s, speed_rad_s`. Set the experiment's
`pulses_per_revolution` to the magnet count. Give each dataset its `rpm`.

| run | role | stimulus | why |
|---|---|---|---|
| constant speeds | train | 1500, 3000, 6000 rpm, ≥ 1000 edges each | delay and jitter; across speeds, whether the delay scales with the period (an angular lag, as the mismatch case shows) |
| ramp | train | slow acceleration over the range | edges stay phase-continuous; no missed or doubled pulses |
| top speed | holdout | the highest rated speed (or 12000 rpm) | held-out delay and jitter; nearness to the US5881's 10 kHz switching limit |
| air gap | optional | the same speed at two magnet gaps | sensitivity of the delay to field strength, which the model does not represent |

**Matching.** Pair each reported edge with the reference edge it lags, and
drop the pairs where the reference shows a missed or extra pulse. Report
those as counts, not as timing outliers. Missed pulses are not modelled; any
count above zero is a finding, not noise.

**Thresholds** (before the holdout). The mean timing error is bounded by the
logic analyzer's resolution plus the reference's alignment uncertainty. The
jitter ratio is bounded, as in `rpm_us5881_synthetic.thresholds.toml`, from
the training spread.

## What the public data already covers

- **DPS310 noise against oversampling.** Covered by the Causal Chambers wind
  tunnel (`python -m cp_calibration chambers`): four real units at 1× and 8×.
  - The noise is 0.6–0.85× the datasheet figures, and the 8×/1× ratio
    matches.
  - Not covered: absolute accuracy, drift, temperature, latency and the 64×
    mode.
- **Power module and RPM sensor.** No public data with a reference was
  found.
  - The wind tunnel's fan tachometers (`tach_resolution` in `wt_test_v1`)
    have no independent encoder, so they can't give edge delay or jitter.
