# Vehicle interface

The vehicle interface is the boundary between a CP Models vehicle exported as
an FMI 3 Co-Simulation FMU and a host that loads that FMU **into its own
process**: it supplies actuator commands and environment inputs, advances the
model, and reads vehicle state and simulated sensors.

| Piece | Where | Role |
|---|---|---|
| Contract (normative) | [`contract/vehicle_interface.toml`](../contract/vehicle_interface.toml) | Names, types, shapes, units, frames, capabilities, marker, profiles |
| Reference partials | [`Exports/FMU/VehicleInterface/`](../Exports/FMU/VehicleInterface) | Declare each capability's variables for models authored here |
| Multirotor export | [`Exports/FMU/PartialCopterInterface.mo`](../Exports/FMU/PartialCopterInterface.mo) | Extends the partials; fire-compose `--profile vio` builds on it |
| Checker | [`tools/vehicle_interface/`](../tools/vehicle_interface) | Validates exported metadata; writes the runtime descriptor |
| Runtime | [`runtime/`](../runtime) | Self-contained C library: loads the FMU in-process, drives it |
| Example host | [`runtime/examples/hover_host.c`](../runtime/examples/hover_host.c) | Standalone host using only contract capabilities |
| ArduPilot SITL prototype | [`runtime/hosts/ardupilot/`](../runtime/hosts/ardupilot) | Patch adding SITL model `cpvehicle` (host code, kept for reproducibility) |

CP Models owns the contract, the partials, the checker and the runtime. Hosts
(FastDyn, ArduPilot SITL, …) own their adapters. Nothing here depends on a
host.

**An FMU conforms because of its exported metadata and declared semantics, not
because it inherits from a particular class.** The partials help produce
conforming models; the checker decides.

## Contents

1. [Status](#1-status)
2. [Contract](#2-contract)
3. [Time and values](#3-time-and-values)
4. [Checker and descriptor](#4-checker-and-descriptor)
5. [Runtime](#5-runtime)
6. [Tests](#6-tests)
7. [Known gaps](#7-known-gaps)

## 1. Status

Pinned for this version: Rumoca `e32964fc` (github.com/Arslan8/rumoca,
`rumoca-bitcode-v1`), the binary `tools/fmi/bin/rumoca`.

| Vehicle or host | Designed | Implemented | Tested |
|---|---|---|---|
| CP Models multirotor exports (fire-compose `vio`): quad, hexa with gapped channels | yes | yes | conformance (§6), standalone host, FMI vehicle harness |
| FastDyn `FastDyn.Copter` FMU as built by FastDyn today | yes | legacy profile | checker and runtime conformance (legacy) |
| FastDyn `Copter`/`Rover`/`Plane` migrated to these partials | yes | no | no |
| Rover and fixed-wing compositions in CP Models | contract only | no | no |
| FastDyn host adapter (planned as a device model) | yes | no | no |
| ArduPilot SITL host (`runtime/hosts/ardupilot`) | prototype | prototype on ArduPilot `Copter-4.6.2` | ArduCopter with the CP Models quad flies a guided mission (takeoff, hold, 20 m waypoint, land) in two launches of one SITL directory |
| PX4 SITL, Gazebo, Isaac Sim hosts | design targets | no | no |

A successful conformance check or launch is not evidence of model fidelity or
of a successful mission.

## 2. Contract

The contract is a required version marker plus independently versioned
**capabilities**. A host names the capabilities it requires and is refused
before running if the model lacks one. A model may export any additional
variables. Version 1 keeps the sensor names FastDyn already consumes; new
fields use canonical `state.*`/`external_*` names.

### Version marker

| Variable | Type | Causality / variability | Value |
|---|---|---|---|
| `cp_vehicle_interface_version` | Int32 | `local` (or `output`) / `constant` | `1` |

Declare it as `constant Integer cp_vehicle_interface_version = 1`
(`VehicleInterface.Base`). A `final parameter` is not accepted: Rumoca
exports it as a calculated parameter without a value, so the metadata could
not show which version the model implements. A constant cannot be set
through FMI, so no configuration can change it; the runtime also reads it
back from the live instance after initialization.

### `pwm_actuation_v1`

| Field | FMI | Unit | Required | Meaning |
|---|---|---|---|---|
| `pwm` | Float64 input `[n]` | µs | yes | Element `k+1` is mapped host output channel `k` |
| `pwm_min` | Float64 parameter | µs | no | Pulse width of a zero command |
| `pwm_max` | Float64 parameter | µs | no | Pulse width of a full command |

- `n` is the exported dimension, **not** the rotor count. A hexa composed with
  `actuatorIndex = {1,2,3,4,6,8}` exports `pwm[8]`; channels 4 and 6 are
  valid and drive nothing.
- Every element needs a finite literal `start`; that vector is applied before
  the first step. A missing or ambiguous start is an error, never a guessed
  neutral value.
- The contract imposes no channel limit and no clipping. A host enforces its
  own channel limit and mapping.

### `kinematic_state_v1`

Physical truth, never a measurement.

| Field | Shape | Unit | Frame | Meaning |
|---|---|---|---|---|
| `state.position_ned` | `[3]` | m | local NED | CG position relative to the origin below |
| `state.velocity_ned` | `[3]` | m/s | local NED | CG velocity |
| `state.quaternion_wb` | `[4]` | 1 | FRD → NED | Scalar-first Hamilton `(w,x,y,z)`; `v_ned = q v_frd q*` |
| `state.angular_velocity_frd` | `[3]` | rad/s | body FRD | Body rate relative to NED |
| `lat0` | parameter | deg | WGS84 | Origin latitude |
| `lon0` | parameter | deg | WGS84 | Origin longitude |
| `ground_alt_wgs84` | parameter | m | WGS84 ellipsoid | Origin altitude; NED down is zero here |

- **Reference point:** the vehicle's combined CG (`reference_point =
  "combined_cg"` in the contract). In the multirotor exports, `state` is
  `truth` of `Interfaces.VehicleState`, which `RigidBody6DOF` integrates at
  the combined CG.
- **Quaternion:** unit norm up to integration error (the rigid body applies a
  norm-restoring gain); `q` and `-q` describe the same attitude. Hosts that need
  a strict unit quaternion normalize it.
- GNSS position cannot stand in for true position, and heading cannot stand in
  for full attitude.

### `imu_v1`

| Field | Shape | Unit | Frame | Meaning |
|---|---|---|---|---|
| `accel` | `[3]` | m/s² | body FRD | Specific force: about `(0, 0, -9.81)` at rest and level |
| `gyro` | `[3]` | rad/s | body FRD | Angular rate |

### `magnetometer_v1`

| Field | Shape | Unit | Frame |
|---|---|---|---|
| `mag` | `[3]` | Gauss (1 G = 1e-4 T) | body FRD |

### `gnss_v1`

| Field | Shape | Unit | Required | Meaning |
|---|---|---|---|---|
| `gps` | `[3]` | mixed | yes | Latitude [deg], longitude [deg], altitude above the WGS84 ellipsoid [m] |
| `vel_ned` | `[3]` | m/s | yes | Velocity in NED |
| `yaw_deg` | scalar | deg | no | Heading clockwise from north |
| `earth_radius_m` | parameter | m | no | Radius of the local spherical conversion |

`gps` mixes units, so no single FMI unit can describe it. In the current
multirotor exports `yaw_deg` is derived from attitude truth, not from a GNSS
measurement.

### `barometer_v1`

| Field | Unit | Required | Meaning |
|---|---|---|---|
| `baro_pressure_pa` | Pa | yes | Static pressure |
| `baro_temperature_c` | °C | yes | Sensor temperature |
| `baro_altitude_m` | m | no | Up-positive altitude above the ground plane |
| `baro_climb_rate_mps` | m/s | no | Up-positive climb rate |

### `external_wrench_v1`

| Field | Shape | Unit | Frame | Meaning |
|---|---|---|---|---|
| `external_force_frd` | `[3]` | N | body FRD, at the combined CG | Non-gravity force added to the plant's own |
| `external_moment_frd` | `[3]` | N·m | body FRD, about the combined CG | Moment added to the plant's own |

Zero (the start value) has no effect. Inputs are held between communication
points. The multirotor export wires them to the plant's `externalForce_b` and
`externalMoment_b`. A wrench input alone is not contact coupling: collision
geometry and two-way contact are not part of version 1, and each physical
effect needs exactly one owner (the model or the host), never both.

### Legacy profile `fastdyn_legacy`

An FMU without the marker is refused unless the host **explicitly** selects
this profile; the checker then warns. It offers `pwm_actuation_v1`,
`imu_v1`, `magnetometer_v1`, `gnss_v1` and `barometer_v1` (never
`kinematic_state_v1`), under the same rules. It also accepts a binary under
the FMI 2 platform directory (`binaries/linux64/`), which FastDyn's currently
pinned Rumoca produces; the descriptor records that warning. A marked FMU
cannot use the legacy profile. The profile is removed once every supported
producer declares the marker.

## 3. Time and values

- The **host owns the clock.** At each communication point it sets inputs,
  asks the runtime to advance to a time, and reads values at the reached time.
- Every sensor field is the **latest value at the current communication
  point**. There are no sensor timestamps or batches in version 1. A host
  that delivers sample batches (FastDyn's 17-sample IMU read) repeats one
  value; that does not create distinct samples. True sampled delivery needs
  its own capability.
- **Noise and filtering are the model's.** If a host simulator also adds
  sensor noise or applies an environmental force, the integration configures
  one of them off.
- Hosts must not infer capabilities from model names.

## 4. Checker and descriptor

```sh
cd tools/vehicle_interface
python3 -m cp_vehicle_interface capabilities
python3 -m cp_vehicle_interface check MODEL.fmu --require pwm_actuation_v1 --require kinematic_state_v1
python3 -m cp_vehicle_interface prepare MODEL.fmu OUT_DIR --require ... [--set lat0=47.0]
python3 -m cp_vehicle_interface prepare LEGACY.fmu OUT_DIR --profile fastdyn_legacy
python3 tools/vehicle_interface/build_vehicle_fmu.py configs/hex.toml build/hex.fmu   # from the library root
```

The checker parses the FMI version, the Co-Simulation interface
(`canHandleVariableCommunicationStepSize` must be true, `needsExecutionTool`
false), and each field's type, causality, variability, dimensions (literal,
or a structural parameter with a literal start), start values and declared
units. It reports **all** independent errors together, naming the model and
field. A capability that is partly exported is an error, even if the host
does not require it.

It establishes structural compatibility only. It cannot prove frames, sign
conventions, noise or channel wiring; the conformance tests do (§6). Rumoca
does not export `unit` attributes; units are then a documented producer
obligation, and the descriptor records `unit_declared: false`.

`prepare` extracts the FMU into `OUT_DIR/fmu/` and writes
`OUT_DIR/vehicle.json`, the **descriptor**: an internal runtime artifact, not
the contract. It holds the contract and descriptor-schema versions, the
profile, the capabilities the host required, each provided field's value
reference, size and start values, the marker, every overridable parameter,
the effective overrides, artifact identity, and SHA-256 digests of
`modelDescription.xml` and the shared library. Paths are relative to the
descriptor. Overrides are limited to scalar Float64 parameters with
variability `fixed` or `tunable`; anything else (structural parameters,
arrays, the marker, inputs) is refused. Nothing is written for a
nonconforming FMU.

## 5. Runtime

`runtime/` builds `libcpvehicle` (static and shared) with CMake. It depends
only on libc, libm and libdl. The FMI 3 headers are vendored, and it has its
own small JSON reader and SHA-256, so it never parses XML or unpacks archives.
The API is in
[`runtime/include/cpvehicle/cpvehicle.h`](../runtime/include/cpvehicle/cpvehicle.h).

```
cpv_open -> [cpv_set_parameter] -> cpv_initialize -> { set inputs; cpv_advance_to; read } -> [cpv_reset] -> cpv_close
```

- **`cpv_open`** reads the descriptor and refuses an unknown schema or
  contract version, a legacy profile the host did not allow, a capability the
  host requires but the model lacks, or **a descriptor whose digests no longer
  match the files** (prepare again). It then `dlopen`s the library and
  instantiates (no event mode, no early return). On any failure it cleans up
  everything it created.
- **`cpv_initialize`** applies overrides and the current input vectors
  (initially the model's starts) in Initialization Mode, then checks
  the live `cp_vehicle_interface_version`.
- **`cpv_advance_to`** splits the interval into steps of at most `max_step_s`
  (default 2 ms) and always reports the reached time. The supported FMI
  subset: `fmi3OK`; `fmi3Warning` (completed, reported as `CPV_WARNING`);
  `fmi3Discard`/`fmi3Error` (instance failed, reset allowed); `fmi3Fatal`
  (close only); a termination request (failed); an event request or early
  return, which this runtime does not enable (`CPV_UNSUPPORTED`, failed). A
  failed instance refuses output reads, so no stale value is ever returned
  as current. Non-finite outputs are reported as errors.
- **Reads and writes** go through the descriptor's value references only.
  Absent capabilities return `CPV_UNSUPPORTED`; nothing reads value reference
  zero or fabricates a value.
- **`cpv_reset`** returns to the instantiated state with the descriptor's
  overrides and start inputs.
- **Instances** own their library handle, FMU instance, inputs and errors.
  The runtime has no global mutable state. An instance is not reentrant.
  Multiple instances in one process are tested (§6); thread safety of the FMU
  itself is not claimed.

## 6. Tests

```sh
runtime/tests/run_conformance.sh          # builds FMUs, descriptors, runtime (ASan/UBSan) and runs all
python3 -m unittest tools/vehicle_interface/tests/test_checker.py
```

- **Checker:** synthetic metadata for every refusal rule (marker forms,
  types, causality, shapes, starts, units, partial capabilities, FMI flags,
  duplicates, malformed XML, unsupported overrides, archive path escape) and
  the real quad and hexa exports.
- **Runtime conformance** (`runtime/tests/test_conformance.c`), each against a
  value known in advance:
  - stationary on the gear, specific force `(0, 0, -g)`;
  - the configured 90° yaw maps body forward to east and agrees with
    `yaw_deg`;
  - an upward force of `2 m g` gives acceleration `g`, and a yaw moment gives
    `M / Izz`;
  - distinct values on all eight hexa channels, with untouched starts; each
    used channel moves the vehicle and the two unused ones do not;
  - two interleaved instances stay independent, and a reset reproduces the
    first run;
  - overrides are applied and restored;
  - refusals for missing capabilities, legacy without opt-in, stale digests,
    unknown versions and a wrong token;
  - the legacy FastDyn Copter FMU loads with opt-in and reports no state.
- **Standalone host:** `cpv_hover_host` climbs to and holds 1 m with the quad
  and the hexa using the same code.
- **ArduPilot SITL prototype:** the same quad artifact as the SITL physics
  model, flown by ArduCopter through a checked guided mission. This is
  evidence for one host and one vehicle only, not general SITL support.

## 7. Known gaps

- FastDyn's `Copter`/`Rover`/`Plane` wrappers are not migrated; they run only
  under the legacy profile and expose no `kinematic_state_v1`. CP Models has
  no rover or fixed-wing composition yet.
- Sampled sensors, sensor timestamps and IMU batches need their own
  capability.
- Rotor speeds and other diagnostics are exported but not part of the
  contract.
- Units are not exported by the pinned Rumoca.
- Contact coupling, wind and other environment inputs beyond the external
  wrench are future capabilities.
