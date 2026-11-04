# FMI harnesses

Each model of this library gets one harness that compiles it with Rumoca
(`--target fmi3`) and runs it as an FMI 3 Co-Simulation FMU in the `world`
runtime (github.com/PSecLab/world), with physical acceptance checks on the
trace.

## Setup

- Rumoca with the FMI 3 C profile (github.com/Arslan8/rumoca, branch
  `rumoca-bitcode-v1`): put the binary at `tools/fmi/bin/rumoca` or on PATH.
- A built `world` checkout with its Python venv (`world/venv`).
- The Modelica Standard Library 4.x (`Modelica/`, `ModelicaServices/`,
  `Complex.mo` in one directory).
- `cp local.env.example local.env` and set `MODELICA_LIBS` and `WORLD_ROOT`.

```
harnesses/<Model>/
  Makefile      MODEL := <Model> (+ WRAPPER / EXPECT_REFUSAL); include ../../harness.mk
  world.toml    world configuration (generated from the FMU interface)
  harness.c     the scenario: which inputs get which signals, what is printed
  checks.toml   physical acceptance checks over the trace
  Wrapper.mo    only for components that cannot stand alone (wired to sources/loads)
  out/          build products: model.fmu, bindings, harness binary, trace.csv
```

## Use

```sh
cd harnesses/Systems.Propulsion.Motors.FirstOrderSpeed
make run        # build FMU + bindings + harness, simulate, evaluate checks.toml
make view       # world's read-only trace observer
python3 tools/run_all.py [filter...]   # every harness; results/summary.md
python3 sweep.py sweep/                 # export every library model; sweep/status.jsonl
```

New harness: `$WORLD_ROOT/venv/bin/python tools/new_harness.py <Model>`
(add `--wrapper Wrapper.mo --wrapper-model <Name>` or `--expect-refusal TEXT`).
It writes `world.toml` from the FMU and a starting `harness.c`/`checks.toml`.

## Shared pieces

| file | role |
|---|---|
| `harness.mk` | build rules every harness includes |
| `lib/fire_harness.{h,c}` | the driver: signals (constant/step/ramp/sine/pulse/function), time loop, zero-order-hold inputs, output sampling, finiteness |
| `tools/make_fmu.py` | Rumoca `--target fmi3` for a FIRE model or wrapper, then `build_fmu.py` |
| `tools/build_fmu.py` | compiles a source FMU into `binaries/x86_64-linux/<id>.so` |
| `tools/fire_compile.py` | the one Rumoca command line for this library, model enumeration, machine settings |
| `tools/new_harness.py`, `fmu_vars.py` | harness scaffolding from the FMU's model description |
| `tools/check_trace.py` | evaluates `checks.toml` over `out/trace.csv` |
| `tools/expect_refusal.py` | harness of a deliberately invalid model: export must be refused |
| `tools/run_all.py` | batch build/run/summary |
| `sweep.py` | export every library model with one Rumoca binary (status per model) |
| `lib/fmi3-headers` | the FMI 3 C headers (BSD-2, Modelica Association) |

`bin/rumoca`, when present, is the compiler the harnesses use; replacing it
re-exports every FMU (each harness's FMU depends on it).

Inputs are applied as a zero-order hold at each communication point, as a
co-simulation master does. Array variables are addressed per element
(`m.R_wb[3]`, 1-based, row-major).

## Status (2026-11-03, Rumoca rumoca-bitcode-v1 a4dd197b)

All 84 harnesses pass (4 of them are deliberately invalid chassis
configurations whose export must be refused).

`python3 sweep.py` exports 79 of the library's 129 models. Of the rest:
- 12 are components that cannot stand alone; their harnesses go through a `Wrapper.mo`.
- 4 are the invalid chassis configurations above.
- The others need features the FMI C profile does not have yet (clocks,
  delay and terminal events, `periodStart = time` in PWM, residual rows at
  initialization) or have no harness yet: the GNSS and multi-fidelity IMU
  sensor models added after the harnesses were written.
