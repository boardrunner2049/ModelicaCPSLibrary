# fire-compose

Rust configuration frontend for the minimum continuous NED/FRD multicopter core.
It emits standard Modelica source and a provenance manifest; the compiler owns
Modelica semantics. No Rumoca library is needed to compose.

`vehicle = "multirotor"` (the default) composes a multicopter as described
below. `vehicle = "rover"` and `vehicle = "fixed_wing"` compose
`Exports.FMU.Rover` / `Exports.FMU.FixedWing` with the values a configuration
sets in `[plant]` (plant parameters and sensor mounts; `[plant.aero]` for the
fixed-wing coefficients), `[initial]`, `[interface]` and the IMU selection
(`[models.imu]`, `[imu]`), and the `vio` profile only. See
[docs/vehicle-interface.md](../../docs/vehicle-interface.md#vehicle-configuration)
and `configs/rover.toml`, `configs/plane.toml`, `configs/rover_i2.toml`.

```sh
cargo run --locked --offline --manifest-path tools/fire-compose/Cargo.toml -- validate configs/quad.toml --json
cargo run --locked --offline --manifest-path tools/fire-compose/Cargo.toml -- emit configs/hex.toml --profile vio
cargo run --locked --offline --manifest-path tools/fire-compose/Cargo.toml -- check configs/quad.toml
cargo run --locked --offline --manifest-path tools/fire-compose/Cargo.toml -- catalog
```

`catalog` prints what a configuration can select as JSON (vehicle kinds and
their parameters, IMU models and the `[imu]` keys each accepts, propulsion
levels P0-P4); `tools/compose` (Compose) is a browser frontend built on it.

Only `schema_version=3` is accepted. World is NED; body/chassis is FRD. Presets preserve
previous physical rotor/mount order and East-facing initial heading. Explicit numeric
fields must already use the new frames. Schema 1/2 must be migrated with the transformations
in `docs/architecture.md`; changing only the schema number is insufficient.

`acquisition="continuous"` is optional and is the only supported value.
`[models]` selects `esc`, `motor`, and `rotor` separately, plus independent IMU selection as described below.
P0 defaults are `pass_through`, `first_order_speed`, `quadratic_blade`. P1 uses
`average_voltage`, `first_order_voltage_to_speed`, `quadratic_blade`. P2 uses
`average_unidirectional`, `dynamic_dc_equivalent`, `quadratic_blade`. Incompatible ESC/motor
signal types are rejected. Rotor `kT/kQ` belong in `[rotor]`, separate from `[motor]`.
Other sensor responses remain ideal. Legacy `[ground]` / `geometry.nLegs` and archived
sampling modifiers remain unsupported; IMU has its own clock/metadata contract. Static aggregate or assembled mass budgets remain supported.
Mass `componentId` values (and arm `componentIds`) remain configuration metadata.
The composer rejects duplicate nonempty IDs across the selected assembled parts before
emission and preserves the resolved IDs in the manifest's `config.mass`. Empty IDs are
allowed. Generated Modelica mass records contain only `mass`, `r_C`, `R_Cj` and `inertia`;
ID changes alone do not change the numerical model. Direct Modelica constructors no
longer accept `componentId`; callers bypassing the composer must ensure unique physical
parts themselves. Numerical mass/inertia validation remains in Modelica.

`[landing_gear]` accepts `enabled` (default true), `position_C` (exactly 4x3, FRD
relative to chassis C), `groundZ` (NED), `stiffness`, `damping`, `tangentialDamping`,
and `frictionCoefficient`. Coefficients are finite and nonnegative. Positions are
rebased by the combined CG in Modelica. The mass budget must already include physical
gear mass; no extra mass is added by the contact law. Set `enabled=false` for no contact.
Standalone initial position still defaults to NED z=-1 m. Bundled quad/hexa configs
explicitly start at z=-0.12 m, leaving default feet 2 cm above the plane.
The manifest records the chosen contact profile and required ground-contact events.

`[interface]` accepts pwm_min, pwm_max, lat0, lon0, ground_alt_wgs84, earth_radius_m.
Atmospheric pressure and temperature are constant in this minimum profile.

`plant` exposes `normalizedCommand[N]` for all combinations and P1/P2 `dcVoltage`
(an input with a configured default). `--profile vio` adds the flat PWM/sensor boundary
and binds P1/P2 supply internally. All VIO compositions extend `Exports.FMU.CopterInterface`.
The composer maps its shared PWM adapter's `normalizedCommand` channels through
`geometry.actuatorIndex` to separate ESC instances, then connects motors, blades and
one common `MultirotorWithSensors`. The vehicle consumes rotor-local hub wrenches;
it owns no ESC, motor or blade. `[interface] pwm_min/pwm_max` defaults to 1100/1900 us.
Legacy `--profile fastdyn` is accepted but emits canonical `profile="vio"` and identical
source. Manifest `input_adapter` records calibration, continuous normalization and
`command_meaning` and `propulsion_input`; `plant` profile has no PWM adapter metadata. Only the CLI alias is retained:
Modelica packages `Adapters.FastDyn` and `Adapters.VIO` are removed. Value conversion lives in
`Adapters.VirtualIO`; copter IO and diagnostics live in `Exports.FMU`. The former standalone
`MultirotorValueAdapter`/`MultirotorFmu` are removed; use generated models for FMI probes.
The composer binds constant plant pressure/temperature directly; the export classes own no environment supply.
Regeneration replaces the staged library so obsolete package directories are removed.
For embedded physics, use `--config-table FMU.models.quad.physics` (TOML dotted-key syntax).
Other host tables are untouched. Their physics schema 1/2 is also rejected.
`--output-dir` must be under FIRE's build/ or outside the source tree. Hand-authored and
symlink outputs are protected. Only package.order-declared source is staged;
deprecated/ is excluded from source hashing and staging. Manifest version 2 records
`ned_frd_continuous_v1`, frame/profile metadata and separate validation statuses.
This is source composition, not arbitrary contract-aware RBC linking.

`check` runs OpenModelica checkModel and translateModel. Use `tools/probe_export.py`
for actual FMI 2.0 CS generation/stepping. Neither proves Rumoca/FastDyn firmware integration.

## Where things sit

```toml
[geometry]
preset = "QuadX"
armLength = 0.30                   # CG to rotor [m]: sizes the preset's frame (not with armMount)
imuPosition_C = [0.02, 0.0, -0.01] # sensor mounts from the chassis reference, body FRD [m]
gnssPosition_C = [-0.05, 0.0, -0.08]
barometerPosition_C = [0.0, 0.0, 0.0]

[mass.placements.battery]          # aggregate mode: an item of the vehicle's mass placed
mass = 0.44                        #   elsewhere; the CG and the inertia about it move
position = [0.03, 0.0, 0.02]       #   (parallel-axis theorem, the item as a point mass)
from = [0.0, 0.0, 0.0]             #   where the aggregate has it (default the reference)
```

## Real parts

Motors, propellers, batteries, ESCs, servos and gearmotors too: `[models.propulsion]`, `[models.servos]`/`[models.steering]`, `[models.drive]` (src/powertrain.rs); see parts/README.md.


`[models.KIND] part = "..."` (imu, gnss, barometer, magnetometer) names a real
device from `parts/` instead of a model: the composer selects the level and
fills `[KIND]` from the device's datasheet figures, explicit keys winning, and
records what it derived, assumed and could not represent in `config.parts`.
`fire-compose part KIND [ID]` lists the devices or shows one's conversion. See
`parts/README.md`.

## IMU fidelity v2 and independent acoustics

Use `[models.imu] model="ideal"|"digital"|"errors"|"electromechanical"` for I0–I3.
The readout/error/physics settings, mandatory fields and equations are documented
in [IMU README](../../Systems/Sensing/IMU/README.md). The current `quad_i1/i2/i3`
examples use this ordering. `statistical/device/acoustic` are preserved legacy
selections and manifests label them `legacy_I1/I2/I3`.

Optional `[vulnerabilities.acoustic] model="equivalent_rate"` attaches an RMS-envelope
surrogate to I1/I2; `model="modal_force"` attaches instantaneous pressure coupling to I3.
The composer rejects incompatible/unused fields and records pressure semantics.
Without acoustic configuration, I3 exports zero-start modal force inputs; with it,
I3 exports pressure instead. `imuGyroLocked[3]` is separate from sample validity.
Emission verifies configuration only; native/FMU/firmware verification remains
`not_run` until actually executed. See the [acoustic examples](../../Vulnerabilities/Acoustic/README.md).


## P1 voltage-driven selection

```toml
schema_version = 3
model_name = "ConfiguredP1"
[models]
esc = "average_voltage"
motor = "first_order_voltage_to_speed"
rotor = "quadratic_blade"
[power]
dcVoltage = 16.0
[motor]
Kv = 62.5       # (rad/s)/V, required; NOT rpm/V
tauOmega = 0.03 # s
```

`power.dcVoltage` is a required finite nonnegative scalar for P1. `motor.Kv` is
required and positive; `tauOmega` is positive. Motor `Kv`, `tauOmega`, `omega_start` and
rotor `kT/kQ` may be scalar or length nRotors; initial speed is nonnegative.
P1 has no `omegaMax` speed cap. P0 rejects `[power]` and P1 motor fields; P1 rejects
P0 motor fields. Schema 3 retains P0 numerical defaults, but old integrated rotor selectors must be migrated.
Replace `models.rotor="speed_driven"/"voltage_driven"` with the three explicit selections,
move `[motor] kT/kQ` to `[rotor]`, and regenerate source/FMU artifacts.

The manifest records `physics.propulsion`, command contract, unipolar voltage-modulation
mode, input-default/internal-constant supply policy, SI speed gain, one speed state per
rotor, and absence of current/power/load feedback. These contracts document the bounded
composer's selection; they do not claim arbitrary RBC semantic matching.
Use `configs/quad_p1.toml` / `hex_p1.toml` for complete examples.

The `plant` and `vio` generated models both contain a `plant` component. The `plant` profile
forwards environmental inputs and exposes truth, measurements and wrench diagnostics;
computed hub wrenches stay inside the composition. Custom vehicle modifiers belong under `plant(...)`.

## P2 electromechanical propulsion

P2 selects `average_unidirectional` + `dynamic_dc_equivalent` + `quadratic_blade`.
Use `configs/quad_p2.toml` or `configs/hex_p2.toml`; their parameters are illustrative,
not calibrated motor/ESC data. P0/P1 remain empirical speed responses.

`Physical.Electrical.Passives.SeriesRL` composes MSL resistor and inductor with
bidirectional pins. `Physical.Mechanical.Rotational.RigidShaft` composes inertia and
viscous friction. `Systems.Propulsion.Motors.DynamicDCEquivalent` connects these
through MSL `RotationalEMF`. Its current and speed obey:

```text
L*der(i) = Vm - R*i - k*omega
(J + JLoad)*der(omega) = k*i - b*omega - loadTorque
loadTorque = kQ*omega^2
```

Explicit positive `motor.R` [ohm], `L` [H], `k` [N.m/A = V.s/rad] and `J` [kg.m2]
are required, together with explicit nonnegative `rotor.J` and `power.dcVoltage`.
`motor.b`, `i_start`, `omega_start` default to zero and must be nonnegative for P2.
Motor/rotor fields accept scalars or length-nRotors arrays. `rotor.J` is routed to
motor `JLoad`, not the aerodynamic blade. No physical motor defaults are guessed.
`k` uses one consistent DC-equivalent SI basis; a P1 fitted `Kv` or BLDC phase
constant must not be copied without converting/identifying the equivalent circuit.
There is no empirical time constant or speed cap in P2.

The nonregenerative ESC uses average modulation. At zero command, existing positive
inductor current circulates at zero terminal voltage until reaching zero. Only then
does the output block and float to back-EMF; no reverse current/active braking is
allowed. A positive command below back-EMF also remains blocked once current is zero.
The ESC's `coast` signal connects to motor `currentZero`: only at that natural
zero-current event does the motor project its actual inductor state to zero. It does
not reset current at command cutoff. `openCircuitVoltage=motors.backEMF` gives the
blocked terminal voltage while retaining a fixed current state. This avoids the
index-changing ideal-diode/inductor open circuit. These two auxiliary signals accompany
the conserving electrical pins; SeriesRL itself contains no unilateral constraint.

The ideal converter reports `dcCurrent=modulation*motorCurrent` in powered conduction,
and zero supply current in freewheel/coast. Public P2 diagnostics in both profiles are
`motorCurrent`, `dcCurrent`, `electromagneticTorque`, `loadTorque`, `electricalPower`,
`copperLoss`, `frictionLoss`, `loadPower`, and `storedEnergy` (per-rotor arrays).
`electricalPower = copperLoss + frictionLoss + loadPower + der(storedEnergy)` includes
both magnetic and rotational stored energy. Heat is accounted as a loss, without a
thermal state. A kinematic shaft angle may also be retained by the Modelica compiler.

`HubReaction` adds `-spinSign*(J+JLoad)*der(omega)` to the blade's aerodynamic moment.
The vehicle applies the mount rotation and CG moment arm once; do not also add a motor
housing reaction. This is a fixed-carrier axial approximation: carrier angular
acceleration feedback and gyroscopic coupling are excluded. Physical motor/propeller
mass belongs once in the vehicle mass budget; relative spin inertia is counted once
on the shaft. P2 does not add PWM ripple, phase commutation, PWM-cycle discontinuous
conduction, saturation, current limiting, battery/SOC, thermal dynamics or new aerodynamics.

Run `Examples.QuadElectromechanicalCoast` to inspect hover, cutoff and restart.
`python3 tools/probe_p2_export.py` builds and steps quad/hexa FMI2 models and compares
0.1 ms and 0.05 ms steps against native DASSL. Choose the simulation step with respect
to `L/R`; the example convergence result does not certify arbitrary parameter sets.

P2 native/FMI2 verification and the current Rumoca FMI3 compilation limitation
are recorded in [validation](../../docs/validation.md#p2-electromechanical-validation-2026-10-12).

## Three-phase P3/P4

Select `average_three_phase` or `switching_three_phase` with `three_phase_pm`.
The `[esc]` table holds bridge/PWM parameters; `[motor]` uses per-phase
`RPhase`, `LPhase`, `ke`, and `polePairs`, not P2 equivalents.
Both profiles require `power.sourceResistance` and `power.dcLinkCapacitance`
for one shared DC link. See [equations and configuration](../../docs/propulsion_modeling.md).
The VIO pulse-width adapter remains continuous; P4's internal PWM carrier is independent.
Emission records switching/event requirements but does not certify FMI support.
