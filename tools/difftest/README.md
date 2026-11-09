# Differential tests

CP Models against other simulators, with
[inception](https://github.com/Arslan8/inception), the differential tester
(any two simulators; installed separately). Here: a plugin that adds CP
Models to it, and the specs.

```sh
P=build/venv-gcs/bin/python
$P -m pip install -e PATH/TO/inception      # or git+https://github.com/Arslan8/inception
$P -m inception run tools/difftest/specs/quad_ardupilot_vs_cpmodels.toml
$P tools/difftest/cpmodels.py replay quad A.csv -o B.csv
```

Results go to `build/difftest/SPEC/`: `a.csv`, `b.csv`, `report.html`,
`result.json` (see inception's README for the comparison and the checks).

## The plugin

`cpmodels.py` adds three kinds of side:

| Kind | What |
|---|---|
| `ardupilot` | inception's `ardupilot_sitl` with `third_party/ardupilot`, this repo's mission checks (`runtime/hosts/ardupilot/examples`) and `build/venv-gcs` |
| `cpvehicle` | a CP Models vehicle (`config = "configs/quad.toml"`) in SITL: `cpvehicle:NAME` physics, the vehicle's parameters and home (`runspec.py`), built first (`build.sh`, unless `--no-build`) |
| `cpv_replay` | a CP Models vehicle driven open loop by the other side's servo outputs (`cpv_replay`, `runtime/examples/trace_replay.c`); a SITL trace is replayed on its own frames, the same steps as the run that recorded it |

The SITL frame-by-frame trace (`SITL_TRACE`) is part of
`runtime/hosts/ardupilot/hooks.patch`.

## Specs

| Spec | A | B | What it shows |
|---|---|---|---|
| `quad_replay_self` | CP Models quad in SITL | the same model, open loop, with A's commands | the host chain is exact: any difference is the adapter, recorder or replay |
| `quad_ardupilot_vs_cpmodels` | ArduPilot's X quad | CP Models quad | same autopilot and mission, two physics, closed loop |
| `quad_ardupilot_replay` | ArduPilot's X quad | CP Models quad driven by A's motor commands | same commands, two airframes, open loop: how long they agree |
| `rover_ardupilot_vs_cpmodels`, `plane_ardupilot_vs_cpmodels` | ArduPilot's rover, plane | CP Models rover, plane | as the quad |
