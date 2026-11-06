#!/usr/bin/env python3
"""Build a vehicle interface FMU.

usage: build_vehicle_fmu.py CONFIG.toml|MODEL OUT.fmu [--table PATH] [--if-changed] [--work DIR]

CONFIG.toml (any vehicle kind: multirotor, rover, fixed_wing): runs
`fire-compose emit CONFIG --profile vio` and exports the emitted model. With
--table, the physics configuration is that table of a host configuration, e.g.
--table 'FMU.models.quad.physics' in a FastDyn configuration. MODEL (a concrete
library export such as Exports.FMU.Rover): exports it directly. Both go through
tools/fmi/tools/make_fmu.py with the pinned Rumoca (tools/fmi: bin/rumoca or
RUMOCA). Compiles a concrete model, never a partial.

--if-changed (CONFIG.toml only) skips the compilation when OUT.fmu was built
from the same composed model, library sources and compiler: OUT.fmu.inputs.json
records them.
"""
from __future__ import annotations

import argparse
import hashlib
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


def compiler_identity() -> str:
    """The pinned Rumoca binary by content, and the Modelica Standard Library path."""
    sys.path.insert(0, str(MAKE_FMU.parent))
    import fire_compile  # noqa: E402  (tools/fmi/tools)
    rumoca = hashlib.sha256(Path(fire_compile.RUMOCA).read_bytes()).hexdigest()
    return f"{rumoca} {fire_compile.modelica_libs()}"


def build(config: Path, out: Path, work: Path, table: str | None = None,
          if_changed: bool = False) -> Path:
    emit = work / "emit"
    selector = ["--config-table", table] if table else []
    subprocess.run(["cargo", "run", "-q", "--locked", "--manifest-path", str(COMPOSE), "--",
                    "emit", str(config), *selector, "--profile", "vio", "--output-dir", str(emit),
                    "--library-root", str(LIBRARY_ROOT)], check=True)
    manifest = json.loads(next(emit.glob("*.manifest.json")).read_text())
    model = manifest["model"]
    inputs = {"model": model, "generated_model_sha256": manifest["generated_model_sha256"],
              "library_sources_sha256": manifest["source"]["sha256"],
              "compiler_sha256": compiler_identity()}
    stamp = out.with_name(out.name + ".inputs.json")
    if if_changed and out.is_file() and stamp.is_file() and json.loads(stamp.read_text()) == inputs:
        print(f"{out}: up to date ({model})", file=sys.stderr)
        return out
    out.parent.mkdir(parents=True, exist_ok=True)
    stamp.unlink(missing_ok=True)
    subprocess.run([sys.executable, str(MAKE_FMU), model, str(out.resolve()),
                    "--wrapper", str(emit / f"{model}.mo"), "--wrapper-model", model,
                    "--library-root", str(emit / "sources")], check=True)
    stamp.write_text(json.dumps(inputs, indent=2) + "\n")
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("config", help="fire-compose CONFIG.toml or a library export model")
    parser.add_argument("out", type=Path)
    parser.add_argument("--table", help="TOML table path of the physics configuration")
    parser.add_argument("--if-changed", action="store_true",
                        help="skip the compilation if OUT.fmu has the same inputs")
    parser.add_argument("--work", type=Path)
    args = parser.parse_args()
    if not args.config.endswith(".toml"):
        if args.table or args.if_changed:
            parser.error("--table and --if-changed apply to a CONFIG.toml")
        build_model(args.config, args.out)
    elif args.work:
        build(Path(args.config).resolve(), args.out, args.work.resolve(), args.table,
              args.if_changed)
    else:
        with tempfile.TemporaryDirectory() as tmp:
            build(Path(args.config).resolve(), args.out, Path(tmp), args.table, args.if_changed)
    print(args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
