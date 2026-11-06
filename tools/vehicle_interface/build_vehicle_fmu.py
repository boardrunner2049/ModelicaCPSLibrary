#!/usr/bin/env python3
"""Build a vehicle interface FMU.

usage: build_vehicle_fmu.py CONFIG.toml|MODEL OUT.fmu [--work DIR]

CONFIG.toml (a multirotor): runs `fire-compose emit CONFIG --profile vio` and
exports the emitted model. MODEL (a concrete library export such as
Exports.FMU.Rover or Exports.FMU.FixedWing): exports it directly. Both go
through tools/fmi/tools/make_fmu.py with the pinned Rumoca (tools/fmi:
bin/rumoca or RUMOCA). Compiles a concrete model, never a partial.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile

LIBRARY_ROOT = Path(__file__).resolve().parents[2]
COMPOSE = LIBRARY_ROOT / "tools" / "fire-compose" / "Cargo.toml"
MAKE_FMU = LIBRARY_ROOT / "tools" / "fmi" / "tools" / "make_fmu.py"


def build_model(model: str, out: Path) -> Path:
    out.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run([sys.executable, str(MAKE_FMU), model, str(out.resolve())], check=True)
    return out


def build(config: Path, out: Path, work: Path) -> Path:
    emit = work / "emit"
    subprocess.run(["cargo", "run", "-q", "--locked", "--manifest-path", str(COMPOSE), "--",
                    "emit", str(config), "--profile", "vio", "--output-dir", str(emit),
                    "--library-root", str(LIBRARY_ROOT)], check=True)
    manifest = next(emit.glob("*.manifest.json"))
    model = json.loads(manifest.read_text())["model"]
    out.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run([sys.executable, str(MAKE_FMU), model, str(out.resolve()),
                    "--wrapper", str(emit / f"{model}.mo"), "--wrapper-model", model,
                    "--library-root", str(emit / "sources")], check=True)
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("config", help="fire-compose CONFIG.toml or a library export model")
    parser.add_argument("out", type=Path)
    parser.add_argument("--work", type=Path)
    args = parser.parse_args()
    if not args.config.endswith(".toml"):
        build_model(args.config, args.out)
    elif args.work:
        build(Path(args.config).resolve(), args.out, args.work.resolve())
    else:
        with tempfile.TemporaryDirectory() as tmp:
            build(Path(args.config).resolve(), args.out, Path(tmp))
    print(args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
