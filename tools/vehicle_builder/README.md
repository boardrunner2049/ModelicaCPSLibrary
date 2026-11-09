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

Above the cards, the **garage** draws the vehicle (quad, rover or plane, from
above). Picking a part, by its card, one of its pickers, the buttons under
the drawing or the part itself, moves the camera onto it and lights it, and
the caption says its level and the device picked. The drawing follows the
parts: propeller diameter, motor size, battery cells and capacity.
Under the drawing, the **shelf** offers the real parts that fit the focused
part (the sensors' devices, a motor, propeller, battery or ESC, a servo, a
gearmotor), each with its headline datasheet figures (`parts.py` `STATS`);
picking one installs it, as its card's picker does. For an assembly
(propulsion, drive) the shelf shows each slot and what is installed in it.
`?focus=propulsion.propeller` (or `imu`, `gnss`, `steering`, ...) opens on a part.

Each sensor also offers real **devices** (`parts/`: a u-blox NEO-M9N, an
ICM-42688-P, ...). Picking one selects its level and fills the parameters from
its datasheet; the fields then show the part's values as placeholders and only
what you type overrides them. "What the level leaves out or assumes" lists the
datasheet figures the level cannot represent. `?gnss=ublox_neo_m9n&imu=tdk_icm42688p`
in the URL starts with those devices.

Propulsion picks a **motor, propeller, battery and ESC**; the plane's servos
and the rover's steering a **servo**; the rover's drive a **gearmotor** (and
battery). After each check the card lists the composer's checks: full
throttle, hover and thrust/weight, the parts' limits, and the model against
the motor maker's bench table. `?motor=tmotor_mn3508_kv580&propeller=apc_12x45mr&battery=tattu_4s_5200_35c&propulsion_level=P2`
starts with those.

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

**Auto / Light / Dark** in the header sets the theme (auto follows the
system; the choice is remembered in the browser; `?theme=dark` for one page).

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
