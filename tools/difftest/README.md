# Differential tester

Run two simulators, or take two runs' traces, and measure how far apart they
are, channel by channel: ArduPilot's own SITL physics against a CP Models
vehicle, one fidelity level against another, a CP Models vehicle against
Gazebo or a real flight log.

```sh
P=build/venv-gcs/bin/python      # numpy and pymavlink
$P tools/difftest/difftest.py run tools/difftest/specs/quad_ardupilot_vs_cpmodels.toml
$P tools/difftest/difftest.py compare A.csv B.bin --align takeoff --html report.html
$P tools/difftest/difftest.py replay quad A.csv -o B.csv
python3 -m unittest tools/difftest/test_difftest.py
```

`run` writes `build/difftest/SPEC/`: both traces (`a.csv`, `b.csv`), `report.html`
(the measures, both tracks from above, every channel of A and B overlaid
with B − A) and `result.json`. Exit status 0 passed (or nothing checked), 1 a
check failed, 2 an error. `--reuse` compares the last run's traces again,
to change the checks without running again.

## One trace format

Every simulator becomes the same thing: a CSV with `# key: value` metadata
lines, a header row, a time column `t` (s), and channels named in
`traces.py`: NED position and velocity, the body FRD to NED quaternion and
Euler angles, true body rates, IMU specific force and rates, magnetometer
(Gauss), barometer, GNSS, servo outputs `pwm1`... (µs). A trace may have any
subset; the comparison uses what both have. Other columns are carried along
and compared by name.

## Sources

| Source | What | How |
|---|---|---|
| ArduPilot SITL, any physics | every physics frame: truth state, true rates and specific force, the 16 servo outputs held over the frame | `SITL_TRACE=FILE` (`SITL_TRACE_EVERY=N`), a hook in `Aircraft::update_model` (`runtime/hosts/ardupilot/hooks.patch`). The same columns for ArduPilot's own models (`quad`, `x`, `rover`, `plane`, ...), `cpvehicle:NAME`, and the external backends (JSON, Gazebo, ...). `[x.sitl]` in a spec flies it with the vehicle's mission check |
| ArduPilot DataFlash log (`.bin`) | SITL truth (SIM, SIM2), IMU, compass, barometer, GNSS, RCOU; arm/disarm events. Real flights too (no truth) | `sources.ardupilot_log` |
| CP Models vehicle, open loop | the vehicle driven by another trace's servo outputs, without an autopilot | `cpv_replay` (`runtime/examples/trace_replay.c`); a SITL trace is replayed on its own frames, the same steps as the run that recorded it |
| Any other simulator's CSV | Gazebo, PX4, FastDyn, a bench log | a column map: each channel an expression of the file's columns; ENU/FLU (ROS, Gazebo) attitude converted (`sources.map_csv`) |

## Specs

A spec (`specs/*.toml`) names its two sides, A (the reference) and B, each a
`trace`, a `sitl` run or a `replay` of the other's commands, and how to
compare them. See `difftest.py` for the sides and `compare.py` for:

- alignment: on `start`, `takeoff`, `motion`, the first `command`, or a
  logged `arm`, plus an `offset` and a cross-correlation refinement (`xcorr`);
- the window and the grid;
- the checks per channel (globs allowed): `rmse`, `max`, `bias`, `nrmse`, and
  `tol` + `hold` ("B stays within tol of A for at least hold s": how long
  open-loop runs agree);
- derived measures: `position` (3-D distance), `attitude` (the angle between
  the two attitudes), and `path` (how far each run is from the other's track,
  whatever the timing: for closed-loop runs).

| Spec | A | B | What it shows |
|---|---|---|---|
| `quad_replay_self` | CP Models quad in SITL | the same model, open loop, with A's commands | the host chain is exact: any difference is the adapter, recorder or replay |
| `quad_ardupilot_vs_cpmodels` | ArduPilot's X quad | CP Models quad | same autopilot and mission, two physics, closed loop |
| `quad_ardupilot_replay` | ArduPilot's X quad | CP Models quad driven by A's motor commands | same commands, two airframes, open loop: how long they agree |
| `rover_ardupilot_vs_cpmodels`, `plane_ardupilot_vs_cpmodels` | ArduPilot's rover, plane | CP Models rover, plane | as the quad |

Closed-loop runs are not repeatable to the sample: the mission check talks to
the autopilot over MAVLink in wall-clock time. Compare them by `path`, by
shape after alignment, and with tolerances; compare open-loop replays exactly.

Two SITL sides run at the same time on their own instances (A 6, B 7 by
default; `instance =`), so a spec does not disturb SITL on instance 0.
