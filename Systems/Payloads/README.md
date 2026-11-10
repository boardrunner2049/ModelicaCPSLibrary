# Payloads: ArduPilot's optional hardware that loads or moves the airframe

| Kind | Model | ArduPilot (servo function) | Effect on the vehicle |
|---|---|---|---|
| `gripper` | `Gripper` | `GRIP_TYPE 1` (28) | a load on a short hook line; released below 1250 µs, grabbed above 1750 µs near the load, held at neutral |
| `winch` | `Winch` | `WINCH_TYPE 1` (88) | a load on a line paid out (pulse above 1500 µs) and wound in at up to `maxRate` |
| `parachute` | `Parachute` | `CHUTE_TYPE 10` (27) | released above 1200 µs (`CHUTE_SERVO_ON` 1300); deploys after `deployDelay`, inflates over `inflationTime`, then drag ½ρC<sub>D</sub>A\|v\|v |
| `sprayer` | `Sprayer` | `SPRAY_ENABLE` (22, the pump) | the tank empties at the pump's flow; the liquid's weight on the vehicle |
| `landing_gear` | `LandingGear` | `LGR_ENABLE` (29) | deployed at the maximum pulse, retracted at the minimum, over `retractTime`; the deployed gear's drag |
| `gimbal` | `Gimbal` | `MNT1_TYPE 1` (7 tilt, 8 roll, 6 pan) | angles follow their servos at limited rates; the camera's ground point (no reaction torque) |
| `tether` | `Tether` | none (no SITL support) | constant winch tension toward the ground station, half the tether's weight; power to the pack |
| `generator` | `Generator` | none (the model is the source) | power to the pack, burning fuel; the fuel's weight; `fuel_v1` (flow, level) |

```toml
[payloads.winch]
part = "viewpro_dw_5"           # optional: a real device (parts/winch/)
position = [0.05, 0.0, 0.05]    # attachment, body FRD from the combined CG [m]
mass = 0.5                      # the load

[payloads.generator]            # a power source: needs [peripherals.power_module]
part = "loweheiser_hybgen_32"
fuel_start = 1.0
```

The composer gives the payloads with servos the vehicle interface's
`aux_pwm` (`payload_actuation_v1`), in the order of this table: element k is
the host's servo output 9 + k. The ArduPilot host drives it from `SERVO9..16`
and sets each payload's parameters on the same outputs
(`runtime/hosts/ardupilot/devices.py`). Every payload's force and moment is
summed with the host's external wrench (`Bus`) into the plant. The power
module shares its demand between the pack and the power sources: each source
delivers `utilization` × its capacity, the pack the rest.

`configs/quad_payloads.toml` has every payload; flown in SITL, the gear
retracts after takeoff, the gimbal follows MOUNT_CONTROL, the winch pays out,
the gripper releases, and the parachute release disarms the vehicle, which
comes down under its canopy.

## Limits

- Loads (gripper, winch) are point masses on an elastic line: no swing
  damping besides the line's, no load rotation, no snagging.
- Rigid payload masses (sprayer liquid, generator fuel) load the vehicle as
  weight only: the plant's mass and inertia are the configured ones, so the
  change of inertia as they empty is not modelled. Configure `[mass]` without
  the liquid or fuel.
- The parachute's canopy is attached at its mount (no riser length); the
  retracted gear's ground contact is unchanged (the plant's legs stay).
- The gimbal's reaction torques and the camera's view are not modelled.
