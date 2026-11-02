# FIRE Minimal Multicopter Models

[Korean version](README_ko.md)

The active package implements the initial execution scope defined in
`MULTICOPTER_INTERFACES.md` v0.8 and `261006_quadrotor-fidelity-port-contract.md`.
The baseline consists of **continuous P0 motors + R0 rotors + a single 6DOF rigid body
+ ideal sensors + four-point compliant landing gear**, using SI units, a NED world frame, and FRD chassis/body frames.

- `Vehicles.Copter.MultirotorPlant`: sums the hub forces and moments of N rotors about the CG.
- `Vehicles.Copter.MultirotorWithSensors`: adds an IMU, magnetometer, GNSS, and barometer.
- `Vehicles.Copter.Presets`: QuadX, HexaX, OctoX, and CoaxialX8. The coaxial preset changes only the layout; aerodynamic interference is not modeled.
- `Physical.Mechanical.Chassis`: computes mass, CG, and the full inertia tensor from aggregate properties or a static assembly of parts. `LandingGear` reuses the archived point-contact models with FRD mounts and a NED ground normal.
- `Adapters.VirtualIO`: PWM/sensor value conversion and a standalone SPI byte-to-signal adapter.
- `Exports.FMU`: common external PWM/sensor ports and diagnostic outputs; the composer supplies the selected P0–P4 propulsion composition.

`demand` is a **target rotor-speed ratio, not a thrust ratio**. Motor lag is applied once.
Default sensors use continuous ideal measurement equations with zero bias and no
sampling, hold, timestamp or response filter. Optional IMU profiles add these mechanisms. Landing gear is enabled by default and supports
the vehicle on a stationary horizontal plane. Set `landingGear(enabled=false)` for freefall
through the plane. P1 adds an ideal voltage-only ESC and voltage-to-speed surrogate.
P2 adds inductive motor current, torque and load feedback. P3/P4 add three-phase
commutation and a shared DC link, with averaged or switched inverters. General terrain,
drag, battery chemistry/SOC and protocol models remain outside the active path.

## Sensor fidelity (2026-10-28, revised)

IMU fidelity is independent of propulsion: **I0 ideal, I1 digital output, I2
calibration/statistical errors, I3 electromechanical MEMS and electronics**.
Select `[models.imu] model="ideal"|"digital"|"errors"|"electromechanical"`.
See [IMU equations, configuration and limits](Systems/Sensing/IMU/README.md).

| Family | Implemented scope | Remaining scope |
|---|---|---|
| IMU | I0–I3; I3 includes drive/sense, capacitive readout, AGC, PLL and open-loop sense demodulation | Hardware parameter identification; optional force rebalance |
| GNSS | G0 continuous, G1 correlated PVT, G2 satellite observables and navigation solution | G3 RF tracking; device/host integration |
| LiDAR rangefinder | L0 ray/plane, L1 errors, sampled delivery and status | L2/L3 optics and AFBR calibration |
| Optical flow | Design for PAA3905E1 | F0–F3 implementation |

[GNSS G0–G2](Systems/Sensing/GNSS/README.md) and [range L0/L1](Systems/Sensing/Range/README.md)
are available by Modelica composition; see `Examples.GnssRange`. G2 accepts external
ECEF states or parsed GPS broadcast ephemerides. `SensorSuite.GnssModel` is replaceable.
The TOML GNSS selector remains ideal; sampled GNSS/range host adapters are separate work.

I1–I3 share ADC/IIR/decimation and sample metadata. I2 adds nominal errors. I3
replaces input physics with three gyros and three proof masses; its C/V electronics
are carrier-averaged blocks, not transistor circuits or a proprietary chip replica.
Examples `configs/quad_i1.toml` through `quad_i3.toml` are illustrative.

[Acoustic vulnerabilities](Vulnerabilities/Acoustic/README.md) are independently
selected under `[vulnerabilities.acoustic]`: `equivalent_rate` for I1/I2 or
`modal_force` for I3. Package coupling is a static pressure-to-force gain and
projection; no package FEM is introduced. The two acoustic examples demonstrate
RMS-envelope and instantaneous-pressure inputs respectively.

Legacy `statistical`, `device`, `acoustic` selections retain their behavior and are
marked legacy in manifests. I0 remains unchanged. See [validation](docs/validation.md)
for native/FMI2 evidence and solver requirements; hardware, Rumoca/FMI3 and FastDyn
firmware integration are not certified. H-Flow flow/range assembly remains proposed.

## P0 and P1 propulsion

P0/P1 label component combinations; class names describe their equations. The composer
instantiates separate ESC, motor and rotor arrays beside one common mechanical vehicle.

```text
P0: PwmInput -> ESC.PassThrough -> Motors.FirstOrderSpeed
                                      -> QuadraticBlade -> MultirotorWithSensors
P1: PwmInput -> ESC.AverageVoltage -> Motors.FirstOrderVoltageToSpeed
                                      -> QuadraticBlade -> MultirotorWithSensors
```

ESC classes live in `Physical.Electrical.Drives.ESC`; motor surrogates live in
`Systems.Propulsion.Motors`. `PassThrough` forwards a normalized command in [0,1].
`AverageVoltage` computes `equivalentVoltage=clamp(escCommand,0,1)*dcVoltage`.
Only the motor owns a speed state:

- `FirstOrderSpeed`: `tau*der(omega)=omegaMax*clamp(demand,0,1)-omega`.
- `FirstOrderVoltageToSpeed`: `tauOmega*der(omega)=Kv*equivalentVoltage-omega`.

Both use `Physical.Mechanical.Aerodynamics.Blades.QuadraticBlade`. The common
`MultirotorPlant` accepts `rotorForce_r[nRotors,3]` and `rotorMoment_r[nRotors,3]`,
and returns rotor-local inflow and body angular velocity for the rotor model.
It rotates hub wrenches and adds the CG moment arm exactly once. It contains no ESC,
motor or rotor components. `MultirotorWithSensors` adds the same sensor suite to either combination.

Choose components independently under `[models]`:

| Combination | esc | motor | rotor |
|---|---|---|---|
| P0 (default) | `pass_through` | `first_order_speed` | `quadratic_blade` |
| P1 | `average_voltage` | `first_order_voltage_to_speed` | `quadratic_blade` |

Incompatible ESC/motor signal types are rejected before emission. P0 motor parameters
are `omegaMax`, `tau`, `omega_start`; P1 uses `Kv`, `tauOmega`, `omega_start`.
P1 requires explicit positive `motor.Kv` in **(rad/s)/V** and nonnegative
`power.dcVoltage` [V]. Convert rpm/V by multiplying by `2*pi/60`.
The rotor coefficients belong in `[rotor] kT/kQ`. Motor and rotor parameters accept
scalars or length-nRotors arrays. The bundled 16 V and 62.5 (rad/s)/V are illustrative.
With `Kv*Vnom=omegaMax`, equal time constants and initial speeds, nominal P1 matches P0.
P1 has no current, power accounting, load feedback, reverse rotation, active braking
or additional speed cap. Zero voltage gives exponential coast-down.

`Examples.QuadVoltageSag` lowers the supply from 16 V to 12 V at 1 s.
The parent FastDyn configs remain `configs/copter462_fire_p0.toml` and
`configs/copter462_fire_p1.toml`. `--profile plant` exposes `normalizedCommand` for both
combinations and a `dcVoltage` input with a configured default for P1.
`--profile vio` keeps PWM as the external command and binds P1 voltage internally.

Migration: replace `models.rotor="speed_driven"/"voltage_driven"` with the selections
above and move `[motor] kT/kQ` into `[rotor]`. The old integrated rotor classes,
`ResponseModels` package and `VoltageDrivenMultirotor*` vehicles are removed;
`IdealVoltage` becomes `ESC.AverageVoltage`. Regenerate composed source and FMUs.

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
P2 is verified with OpenModelica native simulation and FMI2. Rumoca/FastDyn FMI3
currently fails compilation; see [validation details](docs/validation.md#p2-electromechanical-validation-2026-10-12).

## VirtualIO adapters and FMU export

`Adapters.VirtualIO.PwmInput` and `SensorValues` convert values without state or
vehicle assembly. The standalone `SpiTransfer` adapter adds ideal eight-bit SPI
signals with a draft boardrunner interface; see [SPI interface](docs/spi_interface.md).
It is separate from the copter exports. P0/P1/P2 compositions extend
`Exports.FMU.CopterInterface`, whose `PartialCopterInterface` defines sensor ports,
unit/geodetic conversions and separate `truth`/`rotorSpeed` diagnostics.

The shared PWM adapter continuously clamps pulse width to `normalizedCommand` in [0,1].
The composer maps `geometry.actuatorIndex` from adapter channels to `escs.escCommand`.
P0 forwards a speed ratio; P1 interprets it as voltage modulation. Atmospheric supply
is bound by the composer directly to plant pressure/temperature (101325 Pa, 288.15 K).
The export classes do not supply the environment or select/instantiate a physical plant.

Use `--profile vio`; `--profile fastdyn` remains a CLI alias. The external PWM/sensor
names and units are unchanged; `[interface] pwm_min/pwm_max` defaults to 1100/1900 us.
`PwmInput` itself retains 1000/2000 us defaults for independent use or explicit calibration.

The old `Adapters.VIO`/`Adapters.FastDyn` paths and the standalone
`MultirotorValueAdapter`/`MultirotorFmu` assemblies are removed. Move custom subclasses
of the copter interfaces to `Exports.FMU` and regenerate composed sources and FMUs.
`tools/probe_export.py` now uses the composer for P0/P1 quad/hexa FMI2 tests, sharing
the production boundary instead of maintaining a second assembly and port convention.

## Running the models

```sh
omc tools/simulate_landing_gear.mos
python3 tools/verify.py
FIRE_TEST_OMC=1 python3 -m unittest discover -s tools -p test_composer.py
python3 tools/probe_export.py
cargo run --locked --offline --manifest-path tools/fire-compose/Cargo.toml -- \
  check configs/quad.toml --profile vio
```

OpenModelica and Modelica 4.0.0 are required. The FMU probe checks FMI 2.0 Co-Simulation on Linux.
`Examples.QuadHover`, `HexaHover`, `OctoHover`, and `CoaxialHover` are open-loop examples
whose initial rotor speeds balance gravity. They do not include a flight controller.

## Four-point landing gear

Load `package.mo` in OMEdit and simulate `Examples.QuadGroundCycle` for 6 seconds,
or run the `omc` command above from the library root. The example drops from 2 cm foot
clearance, settles, applies symmetric thrust at 2 s, cuts the motors at 2.7 s, and lands again.
Plot `altitude`, `normalLoad`, `contactCount`, and `acceleration_frd[3]`.
The command writes `build/landing-gear/QuadGroundCycle_res.csv`.

`vehicle.landingGear` is a `Physical.Mechanical.Chassis.LandingGear.Parameters` record:

| Parameter | Default / meaning |
|---|---|
| `enabled` | `true`; `false` removes all contacts at translation |
| `position_C[4,3]` | X/Y = ±0.17 m, Z = +0.10 m; FRD tips relative to chassis C |
| `groundZ` | 0 m; horizontal plane's NED Down coordinate |
| `stiffness`, `damping` | 3000 N/m and 150 N·s/m per foot |
| `tangentialDamping`, `frictionCoefficient` | 25 N·s/m, 0.6; viscous friction capped by normal load |

The existing rigid body supplies the sprung mass; gear adds no mass or motion states.
Account for physical leg mass in the aggregate budget or `additionalParts` exactly once.
Every tip is rebased by `chassis.cg_C`. The outward ground normal is `{0,0,-1}`.
Normal force is `max(0, k*max(0,-gap) - c*normalVelocity)` during contact;
friction is dissipative and limited to `mu*normalForce`, with no static-stick constraint.
Contact forces and their `r × F` moments enter the body once, alongside rotor and external loads.
Touchdown/liftoff remain Modelica events; there are no velocity resets or arming conditions.

For a level 1.5 kg quad with four 3000 N/m springs, static compression is
`mg/(4k) = 1.226 mm`, CG altitude is 0.098774 m, and FRD accelerometer Z is −9.80665 m/s².
Standalone plants retain `p_start={0,0,-1}`. Bundled quad/hexa configurations explicitly use
`p_start={0,0,-0.12}` for 2 cm initial clearance with their default CG and mounts.
For a shifted CG or custom attitude, choose the initial CG position using `p_tip = p_CG + R_wb*(position_C-cg_C)`.
Rotor count does not change the four-foot assembly.

Composer schema 3 accepts the optional `[landing_gear]` table with the record fields above:

```toml
[landing_gear]
enabled = true
groundZ = 0.0
stiffness = 3000.0
damping = 150.0
tangentialDamping = 25.0
frictionCoefficient = 0.6
```

## Coordinate frames and configuration migration

`R_ab` represents the attitude of frame b relative to frame a, with component conversion
`v_a = R_ab*v_b`. `q_wb={w,x,y,z}` is the Hamilton quaternion for the same attitude.
The default vehicle preserves the previous East heading with
`q_start={sqrt(0.5),0,0,sqrt(0.5)}`. A position of `p_start={0,0,-1}` is 1 m above the
ground origin. The identity quaternion represents a North heading.
The rotor-local +z thrust axis and sensor-local axes are preserved, so the default
mounting matrices are `diag(1,-1,-1)`. Set `R_bImu=identity(3)` for an IMU aligned with body FRD.

TOML configurations must use **schema_version=3**. Legacy schemas 1/2, the old `[ground]` settings and `geometry.nLegs`,
sample periods, and `first_order` sensor responses are rejected. Embedded physics tables
in existing FastDyn host TOML files require the same migration. Convert positions, vectors,
inertia, mounts, and initial attitudes together; changing only the schema number is insufficient.
See [architecture](docs/architecture.md) for the transformations and ports.

Previous fixed-wing, rover, terrain, sampling, filter, and communication paths, including
the original ENU/FLU landing-gear implementation, are preserved in [deprecated](deprecated/README.md). Do not load the old
classes and the new NED/FRD classes together under the same namespace. Historical validation
records do not certify the new models.

See the [validation scope](docs/validation.md), [sensor contracts](Systems/Sensing/README.md),
and [composition tool](tools/fire-compose/README.md) for details.
The P1 extension additionally exercises Rumoca FMI 3 packaging and real Co-Simulation
steps through the parent FastDyn integration test. RBC linking and actual firmware/flight
validation remain outside that test's scope.

The `plant` and `vio` generated models both contain a `plant` component. The `plant` profile
forwards environmental inputs and exposes truth, measurements and wrench diagnostics;
computed hub wrenches stay inside the composition. Custom vehicle modifiers belong under `plant(...)`.

## P3 and P4 three-phase propulsion

P3 uses a six-step averaged inverter; P4 replaces it with a six-switch PWM inverter. Both share the floating-star `ThreePhasePM` motor and a vehicle-wide `DCLink`. See [Propulsion modeling and configuration](docs/propulsion_modeling.md) for equations, parameter conventions, coast policy and validation limits. Examples: `configs/quad_p3.toml`, `quad_p4.toml`, `hex_p3.toml`, `hex_p4.toml`. These are known-position six-step models, not FOC. Rumoca/FastDyn FMI3 execution is not certified.
