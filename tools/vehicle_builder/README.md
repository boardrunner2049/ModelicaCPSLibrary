# Vehicle builder

Pick a vehicle's parts from CP Models in the browser, get its TOML, and fly it
in ArduPilot SITL.

```sh
python3 tools/vehicle_builder/server.py            # http://127.0.0.1:8765
ssh -L 8765:127.0.0.1:8765 HOST                     # from another machine, then open the same URL
```

Standard library only. The page shows, per airframe (quad, rover, plane):

| Part | Levels | Parameters |
|---|---|---|
| Airframe / plant | the airframe of `configs/quad.toml`, `rover.toml`, `plane.toml` | multirotor geometry, mass, rotor, landing gear; every rover/plane plant parameter and aero coefficient, with its Modelica default and description |
| Propulsion (multirotor) | P0 first-order speed, P1 voltage first-order, P2 DC motor, P3 three-phase averaged, P4 three-phase switching | the level's motor, ESC and supply, starting from `configs/quad_pN.toml` |
| Drive, steering (rover); control-surface servos (plane) | D0 steady-state / D1 DC motor; S0 first-order / S1 rate-limited | the level's plant parameters, with Modelica defaults |
| IMU | I0 ideal, I1 digital, I2 errors, I3 electromechanical | every `[imu]` key the level accepts, starting from `configs/quad_iN.toml`; mount (rover/plane) |
| Magnetometer, barometer, GNSS | M0/B0/G0 ideal; M1/B1 sampled, G1 statistical | every key the level accepts, the sampled ones starting from `runtime/hosts/ardupilot/examples/rover_all_sensors.toml`; mounts (rover/plane) |

Each level says how fast it runs in SITL; the slow ones (I3, P3, P4) are marked.
Every edit is composed and checked by `fire-compose validate`; the run file
(`[sitl]` and `[physics]`, as `runtime/hosts/ardupilot/run.sh` takes it) is
shown as you go. An empty parameter keeps the Modelica default. **Save** writes
it anywhere. **Build & fly** writes `build/vehicle-builder/NAME.toml` and runs
`run.sh` on it: map with the mission flying itself, map for you to fly, or a
headless mission check; its output streams into the page and **Stop** ends the
build, SITL and MAVProxy. The map opens on the display of the machine running
the server. Building needs what `run.sh` needs on PATH (see
`runtime/hosts/ardupilot/README.md`). Runs at the same time use separate SITL
instances. `?airframe=rover` opens on an airframe.

The server listens on 127.0.0.1 only (`--host` to change) and refuses
cross-origin POSTs.

## Where the parts come from

Nothing is restated here (`parts.py`):

- `fire-compose catalog`: vehicle kinds and their configurable parameters,
  the IMU, magnetometer, barometer and GNSS levels and the keys each
  accepts, the propulsion levels (ESC and motor pairs) and the rover/plane
  actuator levels. The composer's own tables, so the builder offers exactly what
  it accepts.
- The Modelica plant classes: parameter defaults and descriptions.
- `configs/`: the airframes and each level's example parameters
  (illustrative, not hardware calibration).

A part with a new level in the library appears here when the composer accepts
it (`tools/fire-compose`: `imu::MODELS`, `config::PROPULSION`) and `configs/`
has an example of it. `runtime/hosts/ardupilot/fidelity.sh` uses the same
levels to sweep one part.
