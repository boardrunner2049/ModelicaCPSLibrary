# ArduPilot SITL host (prototype)

`--model cpvehicle:NAME` replaces ArduPilot SITL's vehicle physics with a CP
Models vehicle model **linked into the SITL binary** through the vehicle
runtime (`libcpvehicle`). Each vehicle is a TOML file (`configs/`, or any file:
vehicle kind, plant parameters, sensor selection); the autopilot is unchanged.
There is no socket, no second process and nothing loaded at run time.

ArduPilot is upstream code, used as it is: the submodule `third_party/ardupilot`
is pinned to tag `Copter-4.6.2` (1ebd4d996e). Everything CP Models adds lives
here:

| Path | What |
|---|---|
| `src/SIM_CPVehicle.{cpp,h}` | the SITL model: maps SITL servos, time and home to the vehicle interface and its outputs back to SITL |
| `hooks.patch` | the three edits ArduPilot needs to know the model: an entry in SITL's model table, the waf option `--cpvehicle=PATH`, and the link flags (22 lines) |
| `run.sh` | **one TOML in, vehicle generated, flying in SITL** (map, or headless mission check); pick the parts in the browser with `tools/vehicle_builder` |
| `fidelity.sh`, `fidelity.py` | one airframe flown with one part (IMU, propulsion) at each fidelity level, all at once |
| `build.sh`, `runspec.py` | vehicles to SITL binaries; resolving a vehicle name or TOML |
| `examples/` | run files (`*.toml`), parameter files, mission checks (`fly_mission.py`, `fly_auto.py`, on the autopilot's clock: `mavclock.py`) |

`build.sh` applies `hooks.patch` to the submodule's working tree and links
`src/` into `libraries/SITL/`; the pinned commit stays upstream
(`.gitmodules`: `ignore = dirty`).

## One TOML, then fly it

```sh
runtime/hosts/ardupilot/run.sh runtime/hosts/ardupilot/examples/rover_noisy_imu.toml            # map, you fly
runtime/hosts/ardupilot/run.sh runtime/hosts/ardupilot/examples/rover_noisy_imu.toml --mission  # map, mission flies itself
runtime/hosts/ardupilot/run.sh runtime/hosts/ardupilot/examples/rover_noisy_imu.toml --headless # no map: mission check, exit code
runtime/hosts/ardupilot/run.sh rover_i2                                                          # any configs/NAME.toml
runtime/hosts/ardupilot/run.sh rover --headless --instance 2   # SITL instance 2 (ports +20): several vehicles at once
```

A run file is the vehicle plus how to run it:

```toml
[sitl]                        # optional, every key optional
name = "my_rover"             # --model cpvehicle:NAME (default: the file name)
speedup = 1
home = [40.414929, -86.932387, 149.0, 0.0]   # lat, lon, alt, heading
parameters = ["my.parm"]      # extra ArduPilot parameters, relative to this file

[physics]                     # the vehicle: the configs/*.toml schema
schema_version = 3
vehicle = "rover"             # multirotor | rover | fixed_wing
[physics.plant]               # plant parameters and sensor mounts
mass = 4.0
imuPosition_b = [0.08, 0.0, -0.03]
[physics.models.imu]
model = "errors"              # ideal | digital | errors | electromechanical
[physics.imu]                 # that IMU's parameters
...
```

A plain vehicle configuration (no `[sitl]`) works too. `run.sh` composes the
vehicle, builds its FMU (skipped when the composed model, library and compiler
are unchanged), checks it, links it into the runtime with every vehicle built
before, relinks the one autopilot the vehicle kind needs (copter, rover or
plane), measures how fast the model runs and warns when that is below the
requested speedup, then starts SITL and MAVProxy (map and console; the
terminal is the `MAV>` prompt: `mode guided`, `arm throttle`, `takeoff 10`,
right-click the map for "Fly To"). `--mission` also flies the vehicle kind's
mission check (log in `~/.cache/cpvehicle-run/NAME/mission.log`). MAVLink is
also forwarded to UDP 14550 (QGroundControl) and 14561. A new vehicle takes
about a minute from TOML to flying; an unchanged one a few seconds.

Examples: `rover_noisy_imu.toml` (4 kg, I2 IMU mounted off the CG),
`quad_noisy_imu.toml` (1.8 kg, I2 IMU), `plane_digital_imu.toml` (2.2 kg, I1
IMU, GNSS antenna on the tail); each flies its mission check.

### Sensors: what can be mixed, and what the autopilot reads

| Sensor | Selectable in the TOML | Reaches ArduPilot in SITL |
|---|---|---|
| IMU | `ideal`, `digital` (I1), `errors` (I2), `electromechanical` (I3), with parameters; mount on rover/plane | **yes**: gyro and accelerometer are the model's |
| GNSS | ideal; antenna position on rover/plane | no: SITL's GPS simulation from the model's position |
| Magnetometer | ideal; orientation on rover/plane | no: SITL's compass from the model's attitude |
| Barometer | ideal; position on rover/plane | no: SITL's barometer from the model's altitude |

Speed: I0 runs at 14-70x real time, I1/I2 at 3-9x, I3 at 0.04-0.1x (it
integrates the MEMS resonators), so I3 cannot run SITL in real time; `run.sh`
reports the rate it measures with every PWM channel at 1500 µs (motors running;
at rest a model can be much faster). SITL then runs at the model's rate and the autopilot
sees nothing different: the mission checks time out on the autopilot's clock,
so an I3 vehicle flies the same checks, only slower. The FastDyn host reads all
of the model's sensors.

### Fidelity sweeps

```sh
runtime/hosts/ardupilot/fidelity.sh imu rover            # quad | rover | plane; levels 0 1 2 3 by default
runtime/hosts/ardupilot/fidelity.sh imu quad 0 1 2       # skip I3 (the quad's I3 flight takes ~35 min)
runtime/hosts/ardupilot/fidelity.sh propulsion quad 0 1 2
```

The airframe is `configs/KIND.toml`; only the part differs. IMU level N takes
the IMU of `configs/quad_iN.toml` (the same for every airframe), propulsion
level N the ESC, motor and supply of `configs/quad_pN.toml`. The run files go
to `build/fidelity/PART_KIND_LEVEL.toml`. Each level flies on its own SITL
instance. The other parts have one level in a vehicle so far (GNSS,
magnetometer, barometer: ideal; rotor: quadratic blade; rover and plane drive
and servos: first-order models in the plant).

IMU, measured (ArduPilot 4.6.2; I0-I2 at `--speedup 2`, I3 at its own rate;
model rates at rest):

| | I0 ideal | I1 digital | I2 errors | I3 electromechanical |
|---|---|---|---|---|
| model rate (x real time) | quad 14, rover 70, plane 71 | 3.2, 9.3, 8.6 | 3.5, 8.5, 9.2 | 0.038, 0.105, 0.098 |
| quad: hover drift / landed from home / max tilt | 0.03 m / 0.31 m / 17.3° | 0.02 / 0.32 / 17.3 | 0.03 / 0.32 / 17.4 | 0.02 / 0.31 / 17.2 (35 min) |
| rover: worst cross-track / stopped from home | 0.64 m / 0.19 m | 0.64 / 0.23 | 0.62 / 0.25 | 0.63 / 0.22 (14 min) |
| plane: worst cross-track / touchdown before aim / off centreline | 15.9 m / 32.3 m / 0.10 m | 16.1 / 32.1 / 0.09 | 16.1 / 32.2 / 0.11 | 16.1 / 32.0 / 0.09 (25 min) |

All twelve pass. With these illustrative parameters (I2 noise 0.01 m/s² and
0.001 rad/s, a 2.5 ms sample period) the autopilot's estimator absorbs the
difference. `examples/*_noisy_imu.toml` use 5x the noise and 20x the bias.

Propulsion on the quad, measured (`--speedup 2` where the model allows;
model rates with the motors running):

| | P0 first-order speed | P1 voltage, first-order | P2 DC motor | P3 three-phase, averaged | P4 three-phase, switching |
|---|---|---|---|---|---|
| model rate (x real time) | 11.9 | 13.7 | 12.4 | 0.007-0.019 | below 0.001 |
| hover drift / landed from home / max tilt | 0.02 m / 0.23 m / 17.4° | 0.02 / 0.22 / 17.4 | 0.01 / 0.25 / 20.4 | not flown | not flown |

P0-P2 pass. P2's winding dynamics show (tilt 20.4° against 17.4°). P3 and P4
build and conform, but resolve the motor commutation (and P4 the 20 kHz
inverter) far below real time: after P3 armed, the autopilot's clock did not
advance for 120 s and the check gave up (a mission would take hours); P4 was
stopped after 80 minutes. They are for offline simulation, not SITL.

## Build only

```sh
git submodule update --init third_party/ardupilot   # or clone with --recurse-submodules
runtime/hosts/ardupilot/build.sh                   # quad hex rover plane rover_i2
runtime/hosts/ardupilot/build.sh rover path/to/my_vehicle.toml
```

For each vehicle it builds the FMU, checks it and writes its linkable shim
(`build/vehicle-interface/NAME`), builds the runtime with every vehicle
prepared there (`build/runtime-release`), then ArduPilot (`WAF_TARGETS`,
default `copter rover plane`) into `third_party/ardupilot/build/sitl/bin`. It
initializes only the ArduPilot submodules SITL needs (waf, mavlink, gtest,
DroneCAN). ArduPilot's build needs `empy==3.3.4`, `future`, `pexpect`,
`dronecan` and `pymavlink` on PATH, for example in a venv. Every binary
contains every built model; the name picks one, and an unknown name stops
SITL with the list.

MAVProxy for the map, once:

```sh
python3 -m venv --system-site-packages build/venv-gcs   # wxPython from the system
build/venv-gcs/bin/pip install MAVProxy
```

## Run SITL by hand

```sh
AP=third_party/ardupilot EX=runtime/hosts/ardupilot/examples
$AP/build/sitl/bin/arducopter --model cpvehicle:quad \
    --defaults $AP/Tools/autotest/default_params/copter.parm,$EX/cpvehicle_quad.parm \
    --home 40.414929,-86.932387,149,0 --speedup 4 -I1 -w
build/venv-gcs/bin/python $EX/fly_mission.py --connect tcp:127.0.0.1:5770
```

Rover: `ardurover`, `rover.parm` + `cpvehicle_rover.parm`, `fly_auto.py rover`.
Plane: `arduplane`, `Tools/autotest/models/plane.parm` + `cpvehicle_plane.parm`,
`fly_auto.py plane`. `fly_mission.py` waits for the EKF to use GPS, arms in
GUIDED, takes off to 5 m, holds, flies 20 m north and back, lands, and checks
each step. `fly_auto.py` uploads a mission, arms in AUTO and checks every
waypoint, the cross-track error on each leg, the attitude and the end: the
rover drives a 20 m square back to home; the plane takes off north from the
runway, flies a 400 x 200 m circuit at 50 m, approaches from the south and
lands. The models start facing north, so use home heading 0.

## What the adapter does (`src/SIM_CPVehicle.cpp`)

| SITL | From the model (vehicle interface v1) |
|---|---|
| servo outputs `SERVO1..n` | `pwm_actuation_v1`: servo *k* drives `pwm[k+1]`, or `CPV_CHANNEL_MAP` (one SITL servo index per model element). A zero output leaves the channel at its previous value. |
| `position`, `velocity_ef`, `dcm` | `kinematic_state_v1`: CG truth, NED, scalar-first quaternion |
| `gyro`, `accel_body` | `imu_v1` `gyro`, `accel` (FRD): the model's selected IMU |
| home | sets the model's geodetic origin (`lat0`, `lon0`, `ground_alt_wgs84`) before initialization. SITL has no geoid, so home altitude is used as the ellipsoid altitude. |
| time | the model is advanced to SITL's `time_now_us + frame_time_us` (integer µs from start) every frame, in steps of at most 2 ms |

- **Sensors.** The IMU is the model's (the IMU selected in its
  configuration); SITL's `SIM_GYR*_RND`/`SIM_ACC*_RND` (default 0) would add
  to it. GPS, compass and barometer are SITL's simulation from the model's
  position and attitude; the model's GNSS, magnetometer and barometer outputs
  are not used (the library has only ideal ones in a vehicle so far, and the
  plants' default Earth field, `{0, 20e-6, 45e-6}` T in NED, has no north
  component, so their compass would read ~90 degrees off). The FastDyn host
  reads all of the model's sensors.
- **Not modelled by the vehicle here.** Battery voltage is a constant
  `SIM_BATT_VOLTAGE`. Wind (`SIM_WIND_*`) is ignored and reported at start.
  Payloads and clamps are not supported.
- **Failures.** Any runtime error stops SITL (`AP_HAL::panic`) with the
  reason. It never continues on stale physics.
- **Channel order.** `configs/quad.toml` is wired in ArduCopter's Quad-X order
  (`actuatorIndex = [3, 2, 4, 1]`), so the identity map is right. The hexa
  composition is not in ArduCopter's Hexa-X order; it needs a
  `CPV_CHANNEL_MAP` and has not been flown here. The rover (steering
  `SERVO1`, throttle `SERVO3`) and the plane (aileron, elevator, throttle,
  rudder on `SERVO1..4`) use ArduPilot's default order; their `.parm` files
  set the servo ranges to the model's 1000..2000 µs.
- **Floating-point traps.** SITL traps invalid operations, division by zero
  and overflow, and the model runs in its process: a model computing 0/0 even
  in an unselected branch aborts SITL. The CP Models rover and fixed-wing
  tests run with the same traps.

## Measured (ArduPilot 4.6.2, `--speedup 4`)

| Vehicle | Result |
|---|---|
| ArduCopter, quad | hover at 4.99 m with drift ≤ 0.01 m, the 20 m north waypoint reached to within 0.4 m, landed 0.18–0.28 m from home, maximum tilt 17.5° |
| ArduRover, rover | all four waypoints reached, worst cross-track error 0.55–0.63 m, stopped 0.17–0.18 m from home, roll/pitch ≤ 0.2° (stock rover tuning) |
| ArduRover, rover_i2 (I2 IMU; accelerometer only, before the gyro came from the model) | same mission: worst cross-track error 0.58 m, stopped 0.17–0.18 m from home, roll/pitch ≤ 0.2° |
| `run.sh --headless`, gyro and accelerometer from the model | `rover_noisy_imu`: cross-track ≤ 0.73 m, stopped 0.06 m from home; `quad_noisy_imu`: hover drift 0.07 m, landed 0.36 m from home, tilt ≤ 16.9°; `plane_digital_imu`: cross-track ≤ 15 m, touchdown 31.4 m before the aim point, 0.08 m off the centreline |
| ArduPlane, fixed-wing | takeoff to 30 m, all circuit waypoints, worst cross-track error 16 m on 200–600 m legs, maximum bank/pitch 35°, touchdown 31 m before the aim point (the flare starts ~60 m out), rollout stopped 0.09 m off the centreline |

The plane needed model-specific parameters (`cpvehicle_plane.parm`): servo
ranges, airspeeds for this airframe, roll-rate gains scaled for its physical
inertia and ±20° aileron (the stock SITL plane applies unit inertia and
full-scale deflections), rotate speed and a shorter flare (lift/drag ≈ 3).

## Differences from FastDyn (QEMU)

| | FastDyn | This prototype |
|---|---|---|
| Autopilot | firmware binary on an emulated board | native SITL build |
| Sensors | the model's sensor outputs through emulated drivers | the model's IMU; SITL's GPS, compass and barometer from the model's truth |
| Clock | QEMU instruction count, 1 ms board tick | SITL frame loop (default 1200 Hz) |
| Model steps | ≤ 2 ms per step at each timer tick | one step per SITL frame (≈ 0.83 ms) |
