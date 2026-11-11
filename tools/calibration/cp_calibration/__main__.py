"""python -m cp_calibration {toolchain,datasheet,run,crosscheck,hash,chambers} (from tools/calibration)."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from . import barometer as baro
from . import pipeline


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="cp_calibration", description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("toolchain", help="milestone 1: export, load, parameters, reproducibility, the step test")
    d = sub.add_parser("datasheet", help="a part's datasheet-derived profile with evidence classes")
    d.add_argument("part")
    d.add_argument("--mode")
    d.add_argument("--drift")
    d.add_argument("--calibration")
    r = sub.add_parser("run", help="the full pipeline for an experiment file")
    r.add_argument("experiment", type=Path)
    r.add_argument("--out", type=Path)
    h = sub.add_parser("hash", help="the sha256 and reference_sha256 an experiment states for a recorded dataset")
    h.add_argument("dataset", type=Path)
    sub.add_parser("chambers", help="DPS310 noise against oversampling on the Causal Chambers wind tunnel data")
    x = sub.add_parser("crosscheck", help="the new sensor models simulated by Rumoca and by OpenModelica, compared")
    x.add_argument("--out", type=Path)
    a = parser.parse_args(argv)
    if a.command == "chambers":
        from . import chambers
        rep = chambers.run()
        print(chambers.markdown(rep))
        return 0
    if a.command == "hash":
        from . import dataset
        for k, v in dataset.hashes(a.dataset).items():
            print(f'{k} = "{v}"')
        return 0
    if a.command == "crosscheck":
        from . import crosscheck
        rep = crosscheck.run(a.out)
        print(crosscheck.markdown(rep))
        return 0 if rep["pass"] else 1
    if a.command == "toolchain":
        m = baro.Model()
        try:
            res = pipeline.toolchain_check(m)
        finally:
            m.close()
        for c in res["checks"]:
            print(f"{'pass' if c['pass'] else 'FAIL'}  {c['check']}: {c['detail']}")
        print(json.dumps({"fmu": res["fmu"], "versions": res["versions"]}, indent=1))
        return 0 if res["pass"] else 1
    if a.command == "datasheet":
        print(json.dumps(baro.datasheet_report(a.part, a.mode, a.drift, a.calibration), indent=1))
        return 0
    import tomllib
    kind = tomllib.loads(a.experiment.read_text())["experiment"].get("kind", "barometer")
    if kind == "power_module":
        from . import power as runner
    elif kind == "rpm":
        from . import rpm as runner
    elif kind == "imu":
        from . import imu as runner
    else:
        runner = pipeline
    rep = runner.run(a.experiment.resolve(), a.out)
    out = Path(rep["profile"]["path"]).parent
    print(f"report: {out / 'report.md'}\nprofile: {rep['profile']['path']}")
    ok = rep["toolchain"]["pass"]
    print("toolchain:", "pass" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
