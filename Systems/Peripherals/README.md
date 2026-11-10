# Peripherals: ArduPilot's optional hardware as models

The sensors on [ArduPilot's optional hardware page](https://ardupilot.org/copter/docs/common-optional-hardware.html)
that the autopilot reads, each a kind with an ideal and a sampled level. A
vehicle configuration adds them under `[peripherals.KIND]`
(`tools/fire-compose/src/peripherals.rs`); the composer exports each reading as
an optional capability of the vehicle interface, and the ArduPilot SITL host
feeds it to ArduPilot's own driver.

| Kind | Levels | Measurand | Capability | ArduPilot driver (SITL) |
|---|---|---|---|---|
| `rangefinder` | L0 / L1 | slant range along the boresight to the ground plane | `rangefinder_v1` | `RNGFND1_TYPE 100` |
| `airspeed` | A0 / A1 | pitot-static ½ρu\|u\| of the axial air velocity, air temperature | `airspeed_v1` | `ARSPD_TYPE 100` |
| `optical_flow` | F0 / F1 | flow rates (ArduPilot's convention) and body rates | `optical_flow_v1` | `FLOW_TYPE 10` |
| `rpm` | R0 / R1 | shaft speeds (rotors; a rover's rear wheels) | `rpm_v1` | `RPM1_TYPE 10` |
| `power_module` | W0 / W1 | a battery pack (`Power.Battery`) and its voltage and current | `battery_v1` | `BATT_MONITOR 4` |
| `temperature` | T0 / T1 | air temperature at the mount | `temperature_v1` | `TEMP1_TYPE 1` (TSYS01) |
| `external_ahrs` | E0 / E1 | attitude, rates, specific force, position, velocity | `external_ahrs_v1` | `EAHRS_TYPE 1` (VectorNav serial) |
| `visual_odometry` | V0 / V1 | camera pose and velocity in the odometry frame | `visual_odometry_v1` | `VISO_TYPE 1` (vicon serial) |
| `beacon` | U0 / U1 | tag position in the beacon frame | `beacon_v1` | `BCN_TYPE 10` |
| `precision_landing` | K0 / K1 | line-of-sight angles to a landing target | `precision_landing_v1` | `PLND_TYPE 4` |

The ideal level is the measurand at the mount from the vehicle's truth,
continuous. The sampled level passes it through `Sampler`: sample period,
latency, scale error, bias, Gauss-Markov drift, white noise (fixed and
proportional), resolution and clipping, plus what the kind needs (the
rangefinder's limits and status, the flow sensor's range band, the
temperature element's thermal lag, visual odometry's drift with distance).
Devices are rigidly mounted: `position` (body FRD from the combined CG) and
`orientation` (`down`, `forward`, `back`, `left`, `right`, `up`, or a matrix)
of the boresight, device +z.

```toml
[peripherals.rangefinder]
part = "benewake_tfmini_plus"   # a real device (parts/rangefinder/); explicit keys win
position = [0.0, 0.0, 0.05]
orientation = "down"

[peripherals.power_module]
part = "holybro_pm02d"
battery = "tattu_4s_5200_35c"   # the pack (parts/battery/) or a table of Power.Battery parameters
supply = true                   # the pack feeds the multirotor's propulsion (P1-P4)
```

`configs/quad_peripherals.toml` has every kind; it flies ArduCopter's mission
check (`runtime/hosts/ardupilot/run.sh configs/quad_peripherals.toml
--headless`) with the EKF fusing the model's optical flow and the precision
landing finding the model's target.

## Limits

- The power module's pack sees the propulsion's current: the ESCs' at P2,
  the DC bus's at P3/P4, shaft power through an efficiency at P0/P1, a
  rover's drive current, a plane's thrust through momentum theory. At P1-P4
  its terminal voltage is the propulsion supply (`supply = false` keeps
  `[power] dcVoltage`).
- Ranges and flow are over a flat ground plane (`groundDown`); no terrain.
- The precision-landing camera is assumed to look down in SITL (the host
  passes its angles as body angles, as `PLND_ORIENT 25`).
- Visual odometry's drift is a random walk in a slowly varying direction
  scaled by the distance travelled; a datasheet's "1 % drift" is its bound.
- Airspeed has no wind (the vehicle interface has no wind input yet).
