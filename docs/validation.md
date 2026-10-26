# Minimum core validation

The aggregate record default fix below resolves the three historical mass/CG failures.
The current native suite passes all 30 simulations and all four invalid-configuration checks.

Run from the library root with OpenModelica and Modelica 4.0.0 installed:

```sh
omc tools/simulate_landing_gear.mos
python3 tools/verify.py
FIRE_TEST_OMC=1 python3 -m unittest discover -s tools -p 'test_*.py'
python3 tools/probe_export.py
python3 tools/probe_p2_export.py
cargo clippy --locked --offline --manifest-path tools/fire-compose/Cargo.toml --all-targets -- -D warnings
```

`verify.py` runs every active physics test and all hover/standalone-IMU examples.
It checks analytical P0 speed response and saturation, local aerodynamic torque signs,
CG moment arms/channel mapping, replacement rotor coefficients, quaternion dynamics,
full inertia, freefall, hover, static mass/payload rebasing and invalid mass configurations.
`NedFrdContract` compares full ENU/FLU and NED/FRD motion at a nontrivial attitude,
with off-diagonal inertia, rotated/displaced IMU, magnetic field, GNSS antenna velocity
and Up-positive barometer altitude. `ContinuousSignals` checks values between former
sampling instants. Quad/hexa/octo/coaxial hover altitude is +1 m (NED z=-1 m).
Default local IMU z is +9.80665 m/s2; body FRD specific-force z is negative.

Landing-gear tests additionally check unilateral force (no tension), inclined surface
projection, Coulomb-capped dissipative friction, angular tip velocity, CG rebasing and
moment balance, zero-leg/freefall behavior, uneven touchdown, and four-foot quad/hexa
settling → takeoff → second landing. The static load is mg, compression is mg/(4k), and
the supported body-FRD accelerometer reads -g. The physics assertions run with native DASSL.
`Examples.QuadGroundCycle` is also available directly in OMEdit; its CLI script writes
`build/landing-gear/QuadGroundCycle_res.csv` for plotting.

`test_composer.py` checks schema-3 defaults, explicit rejection of schema 1/2 and deprecated
settings, nested-table selection, source/manifest ownership, frame metadata, omission of
archived sources, and native simulations of generated quad/hexa FastDyn boundaries.
Generated quad/hexa tests also exercise custom foot positions, ground height and contact
coefficients, grounded accelerometer outputs and takeoff. Invalid landing-gear shapes,
negative/nonfinite coefficients and nonboolean enable flags are rejected before emission.
Set FIRE_TEST_OMC=1 to include these generated-model simulations.
Mass metadata tests reject duplicate nonempty IDs across core, arms, payloads and
additional parts before emission, allow blank IDs, and verify that IDs remain in the
manifest without affecting generated Modelica. The generated assembled-payload test
checks mass and CG; `ChassisMassProperties` checks the full rotated inertia tensor.
Duplicate IDs are tested in the composer rather than `ChassisRejectedConfigurations`,
whose remaining negative cases cover physical validity in Modelica.

`probe_export.py` composes P0/P1 quad/hexa through `--profile vio`, builds FMI 2.0 CS artifacts, initializes real FMU instances,
sets all channels, calls doStep/getReal and checks freefall, independent motor response,
FRD specific-force sign, NED position/velocity and Up-positive altitude/climb rate.
The 0.1 ms FMI step permits <1 rad/s motor-speed error versus the analytic solution;
native solver tests use tighter model-specific tolerances. A second instance of each FMU
checks unpowered settling, symmetric-thrust liftoff and landing after motor cutoff.
The probe currently targets Linux64.

**OpenModelica 1.26.1 FMI runtime workaround:** with contact event indicators,
`internalGetEventIndicators` evaluates ODE equations then clears `_need_update`, which
can leave algebraic sensor outputs one CS step behind the state-derived truth outputs.
The probe reapplies the unchanged held PWM inputs immediately before `getReal` to invalidate
that cache and refresh all outputs. This uses ordinary FMI calls and changes no input values
or continuous states. The report records this workaround; these results do not certify a
host that omits the refresh or uses the uncorrected runtime without it. The native DASSL
simulations do not require this workaround. No installed compiler/runtime files are modified.

Native and FMU reports are written under build/verification and build/export-validation.
They describe the current sources only. Historical ENU/FLU contact, sampling, fixed-wing
and rover results remain with the archived package and do not certify this profile.
These checks do not exercise Rumoca component RBC reuse/linking, FMI 3 packaging, actual
FastDyn firmware integration, or flight calibration. Aircraft coefficients are illustrative.

## P1 extension validation (2026-10-12)

`IdealEscVoltage`, `VoltagePropulsion` and `VoltageMultirotor` check command clamping,
per-channel PWM ranges, supply steps/zero supply, nonzero initial speed, the analytical
first-order trajectory, independent speed states, nominal P0/P1 equivalence and common
vehicle wrench/contact/sensor behavior. They pass with OpenModelica 1.26.1.
`Examples.QuadVoltageSag` also simulates successfully: its 16-to-12 V supply change
reduces speed from 606.42343 to 454.81757 rad/s, a ratio of 0.75.

All 20 composer tests pass with `FIRE_TEST_OMC=1`, including generated P1 quad/hexa
`plant` and `vio` profiles, independent gains/time constants/initial speeds,
nontrivial actuator mapping, invalid/cross-profile parameters, and verification-cache
invalidation when selecting P1. Rust clippy passes with warnings denied.

At this stage, the complete native suite passed 24/27 simulations. The failures were
`ChassisMassProperties` (aggregate mass), `ContactWrenchCG` (shifted-CG contact moment)
and `MultirotorFrames` (shifted-CG hub moment). All three reproduce with the **unchanged
pre-P1 HEAD `fedff92`** in a separate package copy under the same compiler. They are
existing failures; the main verification runner stopped before its separate invalid-chassis
checks. The aggregate record default fix documented below now resolves all three.
Artifacts for this run are in `build/p1-verification/`, including `native.log`,
`prechange-baseline.log` and `p1-report.json`; composer output is in `build/p1-composer-tests.log`.

Run the additional FMI 3 integration tests from the **FastDyn repository root**:

```sh
FASTDYN_TEST_RUMOCA_DIR=third_party/common/rumoca PYTHONPATH=src \
  python3 -m unittest discover -s tests/integration -p test_fire_propulsion_profiles.py
```

All four cases pass: P0 quad, nominal P1 quad, half-voltage P1 quad, and P1 hexa with
8 PWM channels mapped onto 6 rotors. Each case composes, compiles with Rumoca, packages
the source FMU, loads the packaged binary and its own metadata, initializes and calls
`fmi3DoStep` for 0.08 s with a 0.1 ms step. Checks cover the sole external input `pwm`,
array shapes, channel mapping, analytical rotor speed (absolute tolerance 1e-3 rad/s),
finite sensor outputs and the internal DC supply. Logs are in FastDyn's `out/export-layer-fmi3-tests.log`.
This test does not run flight-controller firmware or validate battery/current/shaft physics.

A persistent runnable P1 quad artifact can be built from FastDyn with:

```sh
python3 utils/build_fmi3_fmu.py --config configs/copter462_fire_p1.toml --skip-submodules
```

`configs/copter462_fire_p0.toml` selects P0 (renamed from `copter462_fire.toml`). Both profiles retain
identical external PWM and sensor field names; P1's additional voltage connection is internal.


## Adapter/export separation validation (2026-10-12)

`Adapters.VirtualIO` contains only `PwmInput` and `SensorValues`. The two copter IO
classes moved to `Exports.FMU`; the composer owns plant selection, signal connections
and constant atmospheric supply. `MultirotorValueAdapter` and `MultirotorFmu` are removed.
The canonical `vio` profile and legacy `fastdyn` CLI alias are unchanged.

All 20 composer tests pass with `FIRE_TEST_OMC=1`. They check P0/P1 selection, profile
alias equivalence, source staging, metadata, supply binding and removal of obsolete
adapter classes on regeneration. The former eight-channel vehicle test is now run
against composer-generated P0 and P1 models, checking clipping, independent analytical
speed responses, rotated IMU/magnetometer output and pressure/temperature conversion.
`Tests.AdapterChannels` tests the two stateless converters without a vehicle instance.

At this stage, the full native suite passed 24/27 simulations; the same three mass/CG
failures remained. They are resolved by the aggregate record default fix below. Stage logs: `build/export-layer-native/native.log`
and `build/export-layer-composer-tests.log`. Rust clippy passes with warnings denied.
The earlier baseline evidence remains in `build/p1-verification/`.

The rewritten `probe_export.py` passes all four P0/P1 quad/hexa FMI2 cases. It builds
standard composer-generated models with `pwm`, FRD sensor outputs, Gauss, Celsius,
geodetic GPS and diagnostic rotor speeds. It verifies freefall, independent speed
response, geodesy, atmospheric supply, settling, takeoff and landing. The PWM calibration
comes from the generated manifest. OpenModelica 1.26.1 exposes record-shaped `truth`
fields as local diagnostic variables; the probe reads their actual value references
and checks public sensor output causality separately. It retains the documented
held-input refresh workaround. Report: `build/export-layer-fmi2/report.json`.

All four Rumoca FMI3 tests pass: P0 quad, P1 quad, half-voltage P1 quad and eight-channel
P1 hexa. They exercise PWM clipping, channel mapping, analytic speed, finite sensor
outputs and the internal supply; `pwm` remains the only external input. Log:
FastDyn `out/export-layer-fmi3-tests.log`.

The 13 FastDyn configuration/cache unit tests passed during the preceding common-adapter
update; this separation keeps the configuration names and paths. Actual flight-controller
execution remains unverified. The FastDyn C FMU host still handles four PWM channels;
the Python FMI3 probe independently verifies the eight-channel/six-rotor artifact.

Both persistent configs were rebuilt with the separated export package:
`configs/copter462_fire_p0.toml` -> `out/fmi3/ConfiguredQuad/ConfiguredQuad.fmu` and
`configs/copter462_fire_p1.toml` -> `out/fmi3/ConfiguredQuadP1/ConfiguredQuadP1.fmu`.
Staged libraries contain the two value converters under `Adapters/VirtualIO` and both
copter interfaces under `Exports/FMU`. Build logs are `out/export-layer-p0-build.log`
and `out/export-layer-p1-build.log` in FastDyn.


## Independent ESC / motor / rotor composition (2026-10-12)

The composer now creates separate `escs`, `motors` and `rotors` arrays beside the
same `Vehicles.Copter.MultirotorWithSensors` for P0/P1. The common vehicle consumes
rotor-local hub force/moment. Voltage-specific vehicles and integrated rotor classes
are removed. P0/P1 remain combination labels; TOML selects ESC/motor/rotor separately
and stores aerodynamic coefficients under `[rotor]`.

- All 23 composer tests pass with `FIRE_TEST_OMC=1`. They include quad/hexa/octo,
  shared four-channel/eight-rotor coaxial mapping, unequal motor time constants and
  gains, initial speeds, independent blade coefficients, nominal P0/P1 equivalence,
  an in-simulation supply step, PWM clipping, sensor mounts and incompatible-pair rejection.
- Both `plant` combinations also build FMI2 CS FMUs. Tests require public normalized
  commands (plus P1 voltage) and reject accidental exposure of internally computed hub
  wrenches. Generated models contain the mechanical `plant` instead of inheriting its
  input ports. OMC can fold default-bound environmental arrays into constants: this
  test does not claim every default-bound environmental signal remains a tunable FMU input.
- The updated `Examples.QuadVoltageSag` also simulates successfully for 2 s.
- At this stage, native Modelica passed 24/27 simulations. `ChassisMassProperties`,
  `ContactWrenchCG` and `MultirotorFrames` still failed, so the invalid-chassis checks
  were not reached. The aggregate record default fix below resolves these failures.
- The VIO FMI2 probe passes P0/P1 quad/hexa (four cases), including actual stepping,
  sensor values, contact settling, takeoff and landing.
- FastDyn Rumoca FMI3 passes P0 quad, P1 quad, half-voltage P1 quad and eight-channel
  P1 hexa (four cases). PWM remains the only external command input.
- FastDyn configuration/cache tests pass 13/13. Rust's bounded-reference test and
  `cargo clippy --all-targets -- -D warnings` pass.

Logs: FIRE `build/component-composer-final.log`, `build/component-native/native.log`,
`build/component-fmi2/report.json`; FastDyn `out/component-fmi3-tests.log` and
`out/component-config-tests.log`. The persistent P0/P1 artifacts were rebuilt at
`out/fmi3/ConfiguredQuad/ConfiguredQuad.fmu` and
`out/fmi3/ConfiguredQuadP1/ConfiguredQuadP1.fmu`, with logs
`out/component-p0-build.log` and `out/component-p1-build.log`.
Actual firmware execution remains outside these verification results.


## Aggregate record default fix (2026-10-12)

`ChassisAssembly.aggregate` and `MultirotorPlant.aggregate` now specify defaults as
field modifications (`aggregate(mass=..., inertia=...)`) instead of a whole-record
binding (`aggregate = MassProperties(...)`). This permits callers to override only
mass, CG or inertia while retaining defaults for the other fields. Existing callers
that supply a complete record, including composer-generated models, remain supported.
The default values and physics equations are unchanged.

The previous declarations combined a whole-record value binding with callers' nested
field value modifications. That combination is invalid under Modelica's modification
merging rules. OpenModelica 1.26.1 retained the whole-record defaults without diagnosing
the conflict, so the tests detected incorrect mass/CG at initialization. This was a
parameter-binding problem, not a runtime reset or a change in moment-arm physics.
See [Modelica 3.6, merging of modifications](https://specification.modelica.org/maint/3.6/inheritance-modification-and-redeclaration.html#merging-of-modifications).

The existing test equations, assertions and tolerances were retained for this fix:

- Native OpenModelica: 27/27 simulations pass with no warnings, including all three
  previously failing cases. All four invalid-chassis configurations are rejected.
- `ChassisMassProperties`: aggregate mass is 7 kg and CG is `{4,5,6}` m.
- `ContactWrenchCG`: vehicle mass is 2 kg and CG is `{0.04,-0.03,0.02}` m.
- `MultirotorFrames`: CG is `{0.03,-0.02,0.01}` m and the original hub-moment assertion passes.
- Composer: 23/23 tests pass with `FIRE_TEST_OMC=1`, including generated whole-record
  configuration bindings and both `plant` FMI2 exports.
- VIO FMI2: all four P0/P1 quad/hexa cases pass, including motor/sensor outputs,
  contact settling, takeoff and landing, with the runtime workaround described above.
- Rumoca FMI3: all four cases pass (P0 quad, P1 quad, half-voltage P1 quad, P1 hexa).

Native results: `build/mass-binding-native/report.json` and `native.log` in the same
directory. Composer log: `build/mass-binding-composer.log`. FMI2 report:
`build/mass-binding-fmi2/report.json`. FMI3 log: FastDyn `out/mass-binding-fmi3-tests.log`.

Both persistent FastDyn P0/P1 artifacts were rebuilt with these declarations:
`out/fmi3/ConfiguredQuad/ConfiguredQuad.fmu` and
`out/fmi3/ConfiguredQuadP1/ConfiguredQuadP1.fmu`. Build logs are
`out/mass-binding-p0-build.log` and `out/mass-binding-p1-build.log` in FastDyn.


## P2 electromechanical validation (2026-10-12)

The P0/P1 empirical response models retain their existing behavior. P2 composes a
bidirectional `SeriesRL`, ideal `RotationalEMF`, shaft inertia/friction, a unidirectional
averaged ESC and the existing quadratic blade with explicit load-torque feedback.
The motor resets the actual inductor state only on the ESC-reported zero-current
event; command cutoff does not erase stored magnetic energy.

Verified with OpenModelica 1.26.1 and Modelica 4.0.0:

- Native suite: 30/30 simulations, no warnings, and four expected invalid-chassis
  rejections. `PassiveRL` checks signed current, analytical RL transients and energy.
  `ElectromechanicalCoast` checks continuous cutoff current, its freewheel path,
  blocked low command, open-circuit coast, restart, integrated energy balance and
  hub reaction. A second motor checks initially rotating zero-current coast with
  zero supply, analytical viscous decay and bearing reaction.
  `ElectromechanicalLoad` checks steady torque/speed, aerodynamic-load feedback,
  increased inertia and inductance-dependent startup current.
- Composer: 25/25 tests with native simulation enabled. P2 covers required physical
  parameters, invalid values/arrays/pairings, manifest contracts and quad/hexa
  `plant` and `vio` compositions with unequal per-rotor parameters and nontrivial
  channel mapping. P0/P1 regressions remain included.
- `Examples.QuadElectromechanicalCoast` runs hover, cutoff and restart for 1 s.
- Quad/hexa FMI 2 Co-Simulation FMUs build and run through powered operation,
  cutoff, blocked low command, restart and a second cutoff. The probe checks
  public electrical diagnostics, nonnegative current and zero coast input power.
  The hexa case maps eight command channels to six rotors. Ground contact is
  disabled in this electrical-transient probe.
- Rust unit tests and `cargo clippy --all-targets -- -D warnings` pass.

The FMI2 probe compares eight sample times with native DASSL at tolerance 1e-10.
The maximum sampled errors decrease by approximately half when the step is halved:

| Vehicle | Step [ms] | Speed error [rad/s] | Current error [A] |
| --- | ---: | ---: | ---: |
| Quad | 0.10 | 0.149460 | 0.016299 |
| Quad | 0.05 | 0.074437 | 0.008124 |
| Hexa | 0.10 | 0.170979 | 0.020293 |
| Hexa | 0.05 | 0.085546 | 0.010114 |

These step sizes are evidence for the illustrative parameter sets only. An
appropriate integration step depends on the electrical time scale `L/R`.
Reports are in `build/p2-verification/report.json` and
`build/p2-export-validation/report.json`; composer, Rust and example logs are
in `build/p2-development/`.

**Rumoca/FastDyn FMI 3 remains unsupported for this P2 implementation.** An actual
FastDyn build with the staged portable Modelica library currently stops at Rumoca
`ED001`: 1283 equations versus 1291 unknowns for the quad composition. A standalone
shaft probe compiles, while a separate SeriesRL export reaches another compiler
restriction: `FMI C profile: only unscheduled assertions without relation memory
are supported`. Native OpenModelica and FMI2 accept the same library models;
this does not establish the root cause of the Rumoca equation imbalance.
No compiler checks were bypassed, no Rumoca source was modified, and no P2 FMI3
artifact or firmware execution is claimed. Diagnostic logs are
`build/p2-development/rumoca-build.log` and
`build/p2-fastdyn-fmi3/ElectricalProbe.log`. The existing P0/P1 FMI3 validation above
does not certify P2.

## P3/P4 three-phase propulsion (2026-10-14)

P3 is a known-position six-step averaged inverter and P4 is an asynchronous
high-side PWM six-switch inverter. Both use the same floating-star three-phase
PM motor, shaft and shared DC link. These are the six-step profiles documented in
[propulsion_modeling.md](propulsion_modeling.md), not the future dq/FOC proposal
in the earlier FastDyn interface document. No Rumoca source was changed.

OpenModelica 1.26.1 / Modelica 4.0.0 native results:

- 34 simulations pass, without warnings, with four expected invalid-chassis
  rejections. This includes the prior P0/P1/P2 suite.
- `ThreePhaseConversion` checks phase-peak ke, electrical/mechanical angle,
  sinusoidal abc torque and lossless conversion power.
- `ThreePhaseCoast` checks signed currents, retained current after cutoff,
  recirculation-to-Hi-Z, restart, a nonzero turn-on delay and integrated energy.
- `ThreePhaseRegeneration` checks initially disabled gates with a spinning motor:
  body diodes return energy, the bus voltage rises and mechanical energy falls.
- `ThreePhaseAveraging` checks the P3 locked-shaft electrical equilibrium and
  P4 CCM means over 40–50 ms at duty 0.6, 10 kHz and zero dead time.

`tools/probe_p34.py` bounds the internal DASSL maximum step, rather than treating
CSV output interval as a solver step. The PWM ripple is measured during a full
steady carrier period. Both runs use tolerance 1e-9.

| Quantity | Maximum step 5 µs | Maximum step 2.5 µs |
|---|---:|---:|
| P3 mean phase current [A] | 22.528889288 | 22.528889288 |
| P4 mean phase current [A] | 22.501038066 | 22.501038093 |
| Mean-current difference [%] | 0.123624 | 0.123624 |
| P3 mean ESC input power [W] | 212.629207227 | 212.629207226 |
| P4 mean ESC input power [W] | 212.118393427 | 212.118393915 |
| Mean-power difference [%] | 0.240237 | 0.240237 |
| P4 phase-current peak-to-peak ripple [A] | 0.194694829 | 0.194694830 |
| Maximum integrated energy error [J] | 1.56e-7 | 1.46e-7 |

Mean-current and mean-power comparison tolerances are 1%; step-refinement
changes must be below 0.1%. Doubling `Goff` from 1e-5 S to 2e-5 S changes the
mean current and power by less than 0.1% at this benchmark. This sensitivity check
does not calibrate the leakage to hardware. P3's CCM reduction does not establish
agreement during PWM-cycle DCM, fast commutation, large bus ripple or every duty.

The four P3/P4 composer tests pass with `FIRE_TEST_OMC=1`, including eight
native vehicle cases: quad/hexa × P3/P4 × plant/VIO. These cases use the bundled
20 kHz P4 configuration, unequal command channels, the hexa's 8-to-6 channel
mapping, a shared bus, integrated energy balance and command cutoff. Ground
contact is disabled for these electrical-transient cases. Both new single-motor
examples also simulate successfully without warnings. The complete fast composer
suite has 29 tests (9 native-only tests skipped when `FIRE_TEST_OMC` is unset).
The existing native P0/P1/P2 composer regressions were also exercised during development.

Reports: `build/p34-native/report.json`, `build/p34-validation/report.json`.
Rust unit tests, clippy with warnings denied, package registration and the document's
TOML example also pass. P3/P4 FMI export, FMI stepping, FastDyn firmware execution,
and Rumoca support are not claimed by these results.

## Standalone SPI signal interface (2026-10-14)

The draft [SPI contract](spi_interface.md) adds a standalone eight-bit
`Adapters.VirtualIO.SpiTransfer` with ideal logical master/slave models.
OpenModelica 1.26.1 passes `Examples.SpiTransfer`, `Tests.SpiSignals` and
`Tests.SpiDisruption` without warnings. Checks cover all four modes and both bit
orders, repeated identical bytes, 0/255 payloads, CS hold/release, inactive-CS
rejection, mid-byte cancellation/recovery, and a one-bit MOSI disturbance.
The example completes requests at 10 and 22 microseconds, eight microseconds
after submission at 1 MHz. Results: `build/spi-native/report.json`.

The composer suite passes 20 tests, with its 9 native integration tests skipped.
Its source-bundle expectation includes the new adapter. No SPI copter profile,
boardrunner bridge, physical EMI model or Rumoca/FMI runtime integration is
claimed by these checks.

## IMU I0–I3, revised fidelity (2026-10-26)

The fidelity definition is `digital_errors_electromechanical_v2`: I0 stays ideal,
I1 is deterministic digital readout, I2 adds calibration/statistical errors, and
I3 resolves gyro/accelerometer mechanics and averaged electronics. Acoustic
pressure-to-force coupling is selected independently. The prior statistical,
device and pressure-driven classes remain available as `legacy_I1/I2/I3`; their
old equations do not define the new tiers. See
[IMU equations and limits](../Systems/Sensing/IMU/README.md) and
[acoustic coupling](../Vulnerabilities/Acoustic/README.md).

Validation with OpenModelica 1.26.1 and Modelica 4.0.0:

- Full native suite: **42 simulations passed**, plus four expected invalid-chassis
  rejections. Existing ideal sensors, geometry and propulsion regressions pass.
  The composer suite passes **23 tests**, including native integration cases.
- `ImuDigitalErrors` checks exact equality of I1 and zero-error I2 with the same
  bandwidth/ADC/IIR/decimation/delay, and calibrated I2 static values.
- `ImuElectromechanical` checks all six axes and ensures true rate is not added
  directly to I3 output. The largest final gyro error for rates {0.2,-0.3,0.1}
  rad/s is **2.061e-4 rad/s**. A separate gyro starting at 10% amplitude with a
  1003 Hz mechanical drive and 1000 Hz reference reaches 1002.868 Hz and
  1.99860e-7 m amplitude versus the 2e-7 m target at 1 s.
- `ImuAcousticCoupling` checks power/pressure units, zero exposure/coupling,
  AGC response to drive disturbance and the reduced model against the physical
  gyro. A 1001 Hz, 0.01 Pa RMS tone gives a maximum reduced/physical difference
  of **0.001637 rad/s** after startup (0.15–0.49 s). With tolerance 1e-10, limiting
  DASSL steps to 20 versus 10 microseconds produces agreement within 1e-4 rad/s.
  These illustrative coefficients are not measured hardware susceptibility.
- Legacy regressions retain statistical and device coverage: white-noise variance
  0.038471 versus target 0.04; bias lag correlation 0.959793 versus
  `exp(-0.001/0.02)=0.951229`; RNG replay is exact across solver settings.
  They also cover analog response, temperature, clipping/rounding, IIR, delay and
  decimation. The legacy acoustic case checks 110 Hz/100 Hz aliasing (10 Hz),
  linear transfer and quadratic rectification; analytic error is 7.47e-9.
- Generated current I1/I2/I3 × quad/hexa × `plant`/`vio`: **12 native cases pass**,
  including exported acquisition/delivery timestamps, sequence and validity.
- Current I1–I3 quad FMI2 CS binaries build and step with CVODE. At a 70 microsecond
  communication step each delivers eight samples in 21 ms, with 2.5 ms modeled
  latency and no missing/duplicated sequence numbers. Fresh instances reproduce
  identical outputs. Temperature, raw force inputs, lock diagnostics and typed
  metadata are checked in the FMU XML and runtime outputs.
- Separate acoustic configurations build and simulate natively and as FMI2 CS.
  Changing exported pressure from zero to 0.01 Pa RMS produces peak differences
  of 0.2230 rad/s (I2 envelope path) and 0.2244 rad/s (I3 instantaneous waveform
  path). This checks input causality and effect, not FMI/native transfer accuracy.
  The pressure experiment runs for approximately 0.12 s with a
  `20/sqrt(2)` microsecond communication step and a 1001 Hz carrier.
- Composer checks reject invalid/unused fields, missing physical settings,
  malformed arrays, invalid clocks/seeds, nonfinite coefficients and incompatible
  vulnerability/tier combinations. Readout changes invalidate cached verification;
  old selectors are marked legacy. Rust unit tests and clippy with warnings denied
  pass.

Reproduce from the FIRE root:

```sh
FIRE_TEST_OMC=1 python3 -m unittest discover -s tools -p test_imu.py -v
FIRE_TEST_OMC=1 python3 -m unittest discover -s tools -p test_composer.py -v
python3 tools/verify.py --output build/imu-v2-native
cargo test --locked --offline --manifest-path tools/fire-compose/Cargo.toml
cargo clippy --locked --offline --manifest-path tools/fire-compose/Cargo.toml --all-targets -- -D warnings
```

Evidence: `build/imu-v2-native/report.json`,
`build/imu-validation/{current,physics,fmi2,acoustic-fmi2}/report.json` and
corresponding `run.log` files. Composer/native regression and focused final
physical/FMI logs are under `build/imu-v2-smoke`.

FMI2 CS tests explicitly build with `--fmiFlags=s:cvode` and use the probe's
existing OpenModelica output-cache refresh. An **unresolved runtime restriction**
was observed with OpenModelica 1.26.1 CVODE: the I2 acoustic configuration returned
`fmi2Fatal` while stepping from 7.48 to 7.50 ms on an exact 20 microsecond grid,
even with zero acoustic input. The root cause has not been established. The
acoustic input test therefore uses the non-aligned grid above; passing that test
is not evidence that arbitrary communication steps, or exact ADC-event stops,
work. The separate timing/replay test covers its stated 70 microsecond grid only.
Default forward Euler at a coarse step is unsuitable for the I3 oscillators.
Pressure waveform inputs must resolve the acoustic carrier regardless of the
internal adaptive solver; the reduced RMS-envelope path avoids that carrier-rate
input requirement.

Physical transfer/convergence evidence above is native DASSL evidence. These tests
do not certify hardware calibration, proprietary chip circuitry, force rebalance,
Rumoca/FMI3, FastDyn firmware timestamp/batch handling or H-Flow integration.
