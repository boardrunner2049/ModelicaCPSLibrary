# ArduPilot SITL host (prototype)

`--model cpvehicle:NAME` replaces ArduPilot SITL's vehicle physics with a CP
Models vehicle model **linked into the SITL binary** through the vehicle
runtime (`libcpvehicle`). Each vehicle is a TOML configuration in `configs/`
(vehicle kind, plant parameters, sensor selection); the autopilot is unchanged.
There is no socket, no second process and nothing loaded at run time.

ArduPilot is upstream code, used as it is: the submodule `third_party/ardupilot`
is pinned to tag `Copter-4.6.2` (1ebd4d996e). Everything CP Models adds lives
here:

| Path | What |
|---|---|
| `src/SIM_CPVehicle.{cpp,h}` | the SITL model: maps SITL servos, time and home to the vehicle interface and its outputs back to SITL |
| `hooks.patch` | the three edits ArduPilot needs to know the model: an entry in SITL's model table, the waf option `--cpvehicle=PATH`, and the link flags (22 lines) |
| `build.sh` | configurations to SITL binaries in one step |
| `examples/` | parameter files, mission checks (`fly_mission.py`, `fly_auto.py`) and the map viewer (`view.sh`) |

`build.sh` applies `hooks.patch` to the submodule's working tree and links
`src/` into `libraries/SITL/`; the pinned commit stays upstream
(`.gitmodules`: `ignore = dirty`).

## Build

```sh
git submodule update --init third_party/ardupilot   # or clone with --recurse-submodules
runtime/hosts/ardupilot/build.sh                   # quad hex rover plane rover_i2
runtime/hosts/ardupilot/build.sh rover my_rover    # any configs/NAME.toml
```

For each vehicle it builds the FMU from `configs/NAME.toml` (again when the
configuration is newer), checks it and writes its linkable shim, builds the
runtime with those models (`build/runtime-release`), then ArduPilot
(`WAF_TARGETS`, default `copter rover plane`) into
`third_party/ardupilot/build/sitl/bin`. It initializes only the ArduPilot
submodules SITL needs (waf, mavlink, gtest, DroneCAN). ArduPilot's build needs
`empy==3.3.4`, `future`, `pexpect`, `dronecan` and `pymavlink` on PATH, for
example in a venv. Every binary contains every built model; the name picks
one, and an unknown name stops SITL with the list.

## Watch it

```sh
python3 -m venv --system-site-packages build/venv-gcs   # once; wxPython from the system
build/venv-gcs/bin/pip install MAVProxy
runtime/hosts/ardupilot/examples/view.sh quad          # or hex, rover, rover_i2, plane; [SPEEDUP]
```

`view.sh` starts SITL and MAVProxy with a map and a console; the terminal is
the `MAV>` prompt (`mode guided`, `arm throttle`, `takeoff 10`, right-click
the map for "Fly To", `mode land`). It forwards MAVLink to UDP 14550
(QGroundControl) and 14561, where a mission check can run alongside:

```sh
build/venv-gcs/bin/python runtime/hosts/ardupilot/examples/fly_mission.py --connect udpin:127.0.0.1:14561
build/venv-gcs/bin/python runtime/hosts/ardupilot/examples/fly_auto.py rover --connect udpin:127.0.0.1:14561
```

## Run headless

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
| `position`, `velocity_ef`, `dcm`, `gyro` | `kinematic_state_v1`: CG truth, NED, scalar-first quaternion |
| `accel_body` | `imu_v1` `accel` (specific force, FRD) |
| home | sets the model's geodetic origin (`lat0`, `lon0`, `ground_alt_wgs84`) before initialization. SITL has no geoid, so home altitude is used as the ellipsoid altitude. |
| time | the model is advanced to SITL's `time_now_us + frame_time_us` (integer µs from start) every frame, in steps of at most 2 ms |

- **Sensors.** SITL simulates the autopilot's GPS, compass, barometer and IMU
  from these values, as for every SITL model, and adds its own noise. Of the
  model's sensors only the accelerometer reaches the autopilot; the gyro is
  the model's true angular rate, and the model's GPS, magnetometer and
  barometer outputs are not used. A configured IMU model (for example
  `rover_i2`) therefore changes only the accelerometer here, on top of SITL's
  IMU noise. Feeding all of the model's sensors to the firmware is what the
  FastDyn host does (below).
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
| ArduRover, rover_i2 (I2 accelerometer: 400 Hz, quantized, noise and bias) | same mission: worst cross-track error 0.58 m, stopped 0.17–0.18 m from home, roll/pitch ≤ 0.2° |
| ArduPlane, fixed-wing | takeoff to 30 m, all circuit waypoints, worst cross-track error 16 m on 200–600 m legs, maximum bank/pitch 35°, touchdown 31 m before the aim point (the flare starts ~60 m out), rollout stopped 0.09 m off the centreline |

The plane needed model-specific parameters (`cpvehicle_plane.parm`): servo
ranges, airspeeds for this airframe, roll-rate gains scaled for its physical
inertia and ±20° aileron (the stock SITL plane applies unit inertia and
full-scale deflections), rotate speed and a shorter flare (lift/drag ≈ 3).

## Differences from FastDyn (QEMU)

| | FastDyn | This prototype |
|---|---|---|
| Autopilot | firmware binary on an emulated board | native SITL build |
| Sensors | the model's sensor outputs through emulated drivers | SITL sensor simulation from the model's truth (accelerometer from the model) |
| Clock | QEMU instruction count, 1 ms board tick | SITL frame loop (default 1200 Hz) |
| Model steps | ≤ 2 ms per step at each timer tick | one step per SITL frame (≈ 0.83 ms) |
