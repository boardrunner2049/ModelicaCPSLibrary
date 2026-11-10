"""python -m cp_calibration {toolchain,datasheet,run} (from tools/calibration)."""
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
    a = parser.parse_args(argv)
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
    rep = pipeline.run(a.experiment.resolve(), a.out)
    out = Path(rep["profile"]["path"]).parent
    print(f"report: {out / 'report.md'}\nprofile: {rep['profile']['path']}")
    ok = rep["toolchain"]["pass"]
    print("toolchain:", "pass" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
