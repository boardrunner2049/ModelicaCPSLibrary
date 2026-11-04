# FMI harnesses

Each model of this library gets one harness that compiles it with Rumoca
(`--target fmi3`) and runs it as an FMI 3 Co-Simulation FMU in the `world`
runtime (github.com/PSecLab/world), with physical acceptance checks on the
trace.

## Setup

- Rumoca with the FMI 3 C profile (github.com/Arslan8/rumoca, branch
  `rumoca-bitcode-v1`): put the binary at `tools/fmi/bin/rumoca` or on PATH.
- A built `world` checkout with its Python venv (`world/venv`).
- The Modelica Standard Library 4.x (`Modelica/`, `ModelicaServices/`,
  `Complex.mo` in one directory).
- `cp local.env.example local.env` and set `MODELICA_LIBS` and `WORLD_ROOT`.

```
harnesses/<Model>/
  Makefile      MODEL := <Model> (+ WRAPPER / EXPECT_REFUSAL); include ../../harness.mk
  world.toml    world configuration (generated from the FMU interface)
  harness.c     the scenario: which inputs get which signals, what is printed
  checks.toml   physical acceptance checks over the trace
  Wrapper.mo    only for components that cannot stand alone (wired to sources/loads)
  out/          build products: model.fmu, bindings, harness binary, trace.csv
```

## Use

```sh
cd harnesses/Systems.Propulsion.Motors.FirstOrderSpeed
make run        # build FMU + bindings + harness, simulate, evaluate checks.toml
make view       # world's read-only trace observer
python3 tools/run_all.py [filter...]   # every harness; results/summary.md
python3 sweep.py sweep/                 # export every library model; sweep/status.jsonl
```

New harness: `$WORLD_ROOT/venv/bin/python tools/new_harness.py <Model>`
(add `--wrapper Wrapper.mo --wrapper-model <Name>` or `--expect-refusal TEXT`).
It writes `world.toml` from the FMU and a starting `harness.c`/`checks.toml`.

## Shared pieces

| file | role |
|---|---|
| `harness.mk` | build rules every harness includes |
| `lib/fire_harness.{h,c}` | the driver: signals (constant/step/ramp/sine/pulse/function), time loop, zero-order-hold inputs, output sampling, finiteness |
| `tools/make_fmu.py` | Rumoca `--target fmi3` for a FIRE model or wrapper, then `build_fmu.py` |
| `tools/build_fmu.py` | compiles a source FMU into `binaries/x86_64-linux/<id>.so` |
| `tools/fire_compile.py` | the one Rumoca command line for this library, model enumeration, machine settings |
| `tools/new_harness.py`, `fmu_vars.py` | harness scaffolding from the FMU's model description |
| `tools/check_trace.py` | evaluates `checks.toml` over `out/trace.csv` |
| `tools/expect_refusal.py` | harness of a deliberately invalid model: export must be refused |
| `tools/run_all.py` | batch build/run/summary |
| `sweep.py` | export every library model with one Rumoca binary (status per model) |
| `lib/fmi3-headers` | the FMI 3 C headers (BSD-2, Modelica Association) |
| `vehicle/` | the generic vehicle harness (below) |

`bin/rumoca`, when present, is the compiler the harnesses use; replacing it
re-exports every FMU (each harness's FMU depends on it).

`harness.mk` also takes `LIBRARY_SOURCES` (compile against another
ModelicaCPSLibrary source root, e.g. a fire-compose staged copy),
`HARNESS_SOURCES`/`EXTRA_INCLUDES`/`HARNESS_DEPS` (several C sources) and
`FMU_DEPS`; `make_fmu.py --library-root` is the matching option. A scenario
may set `control` in `fh_scenario_t`: a callback at every communication point
that reads world outputs and writes world inputs (closed loop).

Inputs are applied as a zero-order hold at each communication point, as a
co-simulation master does. Array variables are addressed per element
(`m.R_wb[3]`, 1-based, row-major).

## Vehicle harness

`vehicle/` flies any fire-compose multirotor (QuadX, HexaX, OctoX,
CoaxialX8; propulsion P0..P3) as an FMI 3 Co-Simulation FMU in `world`, under
a generic closed-loop flight controller, through a scripted mission, with
checks on the truth record.

```sh
cd vehicle
make VEHICLE=configs/quad.toml run       # emit, export, build, fly, check
make VEHICLE=configs/hex_p2.toml all     # build only; also: view, clean
make VEHICLE=test-configs/octo.toml MISSION=my_mission.toml run
python3 ../tools/run_all.py --vehicles [CONFIG...]   # every config; results/vehicles/
```

`VEHICLE` is a fire-compose configuration, relative to `vehicle/` or to the
library root.

### Build path

`tools/vehicle.py prepare` (called by the Makefile):

1. `fire-compose emit CONFIG --profile vio` into
   `<library>/build/fmi-vehicle/<config>/`: `<Model>.mo` (extends
   `Exports.FMU.CopterInterface`), `<Model>.manifest.json` and the staged
   `sources/ModelicaCPSLibrary`. fire-compose only writes under `build/`, and
   files are only rewritten when their content changes, so the FMU is
   re-exported only when the composition or library changed.
2. Writes `vehicle/out/<config>/`, an ordinary harness directory whose
   generated `Makefile` sets `WRAPPER` to the emitted model,
   `LIBRARY_SOURCES` to the staged copy and the vehicle C sources, then includes
   `harness.mk`. From there everything is as for the other harnesses:
   `make_fmu.py` → `rumoca --target fmi3` → `build_fmu.py` → `out/model.fmu`;
   `wm_generate.py world.toml` → bindings; harness → `out/trace.csv` →
   `check_trace.py checks.toml`.

| generated in `out/<config>/` | |
|---|---|
| `world.toml` | inputs `pwm_1..pwm_N`; outputs accel, gyro, mag, gps, vel_ned, yaw_deg, baro_altitude_m; traces truth (`p_w`, `v_w`, `R_wb`, `omega_b`), `pwm`, `commands.normalizedCommand`, `rotorSpeed`, gear contact, chassis mass/CG/inertia |
| `vehicle_config.h` | `VEHICLE` (vehicle description), `MISSION`, sensor/PWM glue to the bindings |
| `checks.toml` | `checks.template.toml` filled for the vehicle and mission |
| `vehicle.json`, `mission.json` | the same description and timeline as data (`vehicle.py metrics`) |
| `out/controller.csv` | estimate, reference, thrust, tilt and rotor commands every 10 ms |

### Vehicle description (`src/vehicle.h`, from the manifest)

Rotor count and PWM channel count; mass, CG and inertia about the CG
(aggregate or assembled budgets, combined as
`Chassis.Functions.combineMassProperties` does); per rotor the hub position
relative to the CG, thrust direction `R_br e3`, drag reaction per newton
`R_br (0,0,-spinSign) kQ/kT`, spin sign, `actuatorIndex` channel, kT, kQ;
the steady-state speed → command map of the propulsion fidelity
(`command = c0 + c1 w + c2 w^2`: P0 `w/omegaMax`, P1 `w/(Kv V)`, P2 the
DC-equivalent `u V = R i + k w`, `k i = b w + kQ w^2`, P3 the same with
`k = 2 ke`, `R = 2 RPhase` and the diode drop); the speed at full command;
the hover speed and command (minimum-norm allocation of `m g`); the motor time
constant at hover (`tau`, `tauOmega`, or `J/(k^2/R + b + 2 kQ w)`); PWM
calibration; GNSS origin; sensor lever arms. The checks verify mass, CG and
inertia against the FMU's own `plant.chassis`.

### Flight controller (`src/controller.c`)

Firmware-in-the-loop: it reads only the FMU's sensor outputs, never `truth`.

- Estimator: complementary (Mahony) attitude filter, gyro integration with
  accelerometer tilt and magnetometer heading corrections (the field reference
  is learned at initialization from `yaw_deg`); position from GNSS (lat/lon
  back to local NED), height from the barometer, velocity from GNSS, all moved
  from the sensor mounts to the CG. It initializes once the specific force
  reads `g` (at rest), so a vehicle that starts in the air settles first.
- Reference: speed/acceleration-limited path to the phase target, smoothed by
  a critically damped second-order filter at the attitude bandwidth, so the
  velocity and acceleration feed-forward is continuous; yaw rate-limited.
- Cascade: position P → velocity PI (+ feed-forward) → thrust vector
  `m (a - g e3)` with a tilt limit → desired attitude (body z along the
  thrust, commanded heading) → SO(3) attitude error (Lee) P → body-rate PI →
  torque `J alpha + w x J w`.
- Allocation: pseudo-inverse `B' (B B')^-1` of the 4 x n effectiveness matrix
  (collective thrust, three torques per newton of rotor thrust), so any rotor
  count and layout works, coaxial included; yaw is scaled back first when a
  rotor would leave its thrust range. Thrust → speed `sqrt(T/kT)` → command
  map → PWM via `actuatorIndex` (rotors sharing a channel are averaged;
  unused channels stay at `pwm_min`).
- Gains are bandwidths: rate `0.5/tau_a` (damping ~0.7 against the actuator
  lag, limited to 4..20 1/s), attitude rate/2.5, velocity attitude/3,
  position velocity/3, vertical velocity `0.35/tau_a` with a critically damped
  integrator; mass and inertia make them forces and torques. Motors slower
  than 30 ms (voltage-driven P2/P3, `tau ~ 0.09..0.17 s`) get a per-rotor
  lead-lag `(tau s + 1)/(tau_a s + 1)` on the speed demand (lead ratio at most
  5), so the loops see `tau_a = 30 ms` and keep P0 bandwidths.
- Integrators (vertical velocity always; horizontal velocity and body rate
  once 0.3 m up) absorb the command-map error of P1..P3, supply droop and
  CG offsets.
- Landing: `land` phases descend at `land_speed`; touchdown (within 15 cm of
  home height, vertical speed < 0.1 m/s for 0.5 s) disarms.
- If a flying phase commands more than 1.3 m g for 2 s without leaving the
  ground,
  the harness reports `insufficient thrust` and ends the run (run_all:
  run-failed with that reason).

### Mission (`mission.toml`)

Phases of kind `idle` (disarmed), `fly` (to `target = [north, east, up]` m
relative to the arming point, `yaw_deg` relative to the arming heading) and
`land`, with reference limits and the controller period (`step_s`, the world
communication step). The default: 1 s on the ground, take off to 2 m, hover,
3 m north, yaw 90 deg, back, descend to 0.5 m, land, disarm (49 s).

### Checks (`checks.template.toml`)

Filled per vehicle and per mission phase that lists them (`checks = [...]`):
per vehicle, finiteness, tilt < 30 deg throughout, at rest at arming, and the
description's mass/CG/inertia against the FMU; `reach`: position, altitude
and heading at `reach_by_s` (default the phase end); `hold` (hover): altitude
and position error, levelness, no PWM channel saturated (command in
[0.02, 0.95]) and every rotor at its allocated hover speed `sqrt(T/kT)`;
`landed`: at rest, back at home height and position, every channel at zero
command, every leg in contact. Tolerances (`[tolerances]`, overridable in the
mission) may scale with the vehicle (`arm`, `mass`, ...); defaults: waypoint
0.10 + 0.2 arm m, altitude 0.10 m, hover 0.05 m, heading 3 deg.

`vehicle/test-configs/` holds harness-only vehicles: `octo.toml` (OctoX, P0,
permuted channels), `coaxial.toml` (CoaxialX8, P1) and `quad_p3_hover.toml`
(`configs/quad_p3.toml` with one pole pair and 0.25 mH, which can hover; see
below). Config file names must be unique across directories (the build
directories are named after them).

## Status (2026-11-04, Rumoca rumoca-bitcode-v1 6058a2bb)

All 84 harnesses pass (4 of them are deliberately invalid chassis
configurations whose export must be refused).

`python3 sweep.py` exports 106 of the library's 129 models. Of the other 23:
- 13 are components that cannot stand alone (parameters without values,
  connectors left open); their harnesses go through a `Wrapper.mo`.
- 4 are the invalid chassis configurations above.
- 4 GNSS models hit Rumoca gaps in the PVT solver function (`solvePvt`) or
  an external-function call (LAPACK) in a when-clause.
- `Tests.ImuDigitalErrors`: a `when sample` body is still checked against
  synchronous clock ownership (`[ED020]`).
- `Tests.ThreePhaseAveraging` needs runtime events.

Exporting is not running: `Tests.GnssStatistical` stops at t = 0.001 s on its
own "Independent RNG streams collapsed" assertion (Rumoca's native simulator
too), and `Tests.RangePhysics` fails initialization (infinite range of a
parallel ray).

The GNSS and multi-fidelity IMU sensor models have no harness yet.

### Vehicle status (2026-11-03, Rumoca a4dd197b)

Default mission, `python3 tools/run_all.py --vehicles`:

| config | result | hover alt. error | worst waypoint error (xy / alt / yaw) | max tilt |
|---|---|---|---|---|
| quad (P0) | 43/43 | 0.01 cm | 3.4 cm / 0.11 cm / 0.60 deg | 9.1 deg |
| quad_p1 | 43/43 | 0.01 cm | 3.4 cm / 0.11 cm / 0.60 deg | 9.1 deg |
| quad_p2 | 43/43 | 0.01 cm | 3.3 cm / 0.12 cm / 0.60 deg | 9.0 deg |
| hex (P0) | 47/47 | 0.01 cm | 3.4 cm / 0.11 cm / 0.60 deg | 9.1 deg |
| hex_p1 | 47/47 | 0.01 cm | 3.4 cm / 0.11 cm / 0.60 deg | 9.1 deg |
| hex_p2 | 47/47 | 0.00 cm | 3.3 cm / 0.06 cm / 0.60 deg | 9.0 deg |
| payload | 43/43 | 0.06 cm | 3.4 cm / 0.07 cm / 0.60 deg | 9.1 deg |
| test-configs/octo | 51/51 | 0.01 cm | 3.4 cm / 0.11 cm / 0.60 deg | 9.1 deg |
| test-configs/coaxial (P1) | 51/51 | 0.01 cm | 3.4 cm / 0.11 cm / 0.60 deg | 9.1 deg |

A P0..P2 flight takes 10..30 s of wall time.

- `quad_p3`, `hex_p3` export and simulate, but cannot take off: with
  `LPhase` 1 mH and 7 pole pairs the winding reactance at hover speed
  (p w L ~ 4 ohm against 0.2 ohm) caps six-step drive at ~371 rad/s at full
  duty (hover needs 606 / 495 rad/s): thrust/weight ~0.37 / ~0.56. The run
  ends with `insufficient thrust` at t ~ 3.4 s (run-failed).
- `test-configs/quad_p3_hover.toml` (one pole pair, 0.25 mH) flies: up to
  t = 30 s (takeoff, hover, north, yaw) all 29 checks in that span pass (hover
  altitude error 0.04 cm, waypoint error <= 3.5 cm, hover command 0.87).
  P3 FMUs simulate ~500..800x slower than real time (time in
  `evaluate_derivatives`: assertions and torn algebraic solves; the
  communication step makes no difference), so the full 49 s mission takes
  ~10 h and exceeded `TMO=25000`; give run_all a larger `TMO` for P3.
- `quad_p4`, `hex_p4`: export refused, `unsupported-feature:clocks` (the
  switching inverter's clock partitions).
- `quad_i1`, `quad_i2`, `quad_i3`, `quad_i2_acoustic`, `quad_i3_acoustic`,
  `legacy_quad_*`: export fails, `[EI007] redeclaration error for
  sampleTime: selected redeclare class has no such member` (IMU sensor
  models; Rumoca gaps being worked on).
