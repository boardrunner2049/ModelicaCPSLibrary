#!/usr/bin/env python3
"""Compile a FIRE model (or a harness wrapper model) into a runnable FMI 3 FMU.

usage: make_fmu.py MODEL OUT.fmu [--wrapper FILE.mo --wrapper-model NAME]

MODEL is the FIRE class without the ModelicaCPSLibrary. prefix. With
--wrapper, NAME (declared in FILE.mo) is compiled instead, with the FIRE
library on the source path.
"""
import argparse, os, subprocess, sys, tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import build_fmu, fire_compile  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("model")
    parser.add_argument("out", type=Path)
    parser.add_argument("--wrapper", type=Path)
    parser.add_argument("--wrapper-model")
    args = parser.parse_args()
    if args.wrapper:
        source, model = args.wrapper.resolve(), args.wrapper_model
    else:
        source, model = fire_compile.model_file(args.model), fire_compile.PREFIX + args.model
    with tempfile.TemporaryDirectory() as tmp:
        command = fire_compile.compile_command(source, model, "--target", "fmi3")
        env = dict(os.environ, RAYON_NUM_THREADS="1")
        result = subprocess.run(command, cwd=tmp, env=env, capture_output=True, text=True)
        diagnostics = "\n".join(line for line in result.stderr.splitlines() if "warning[" not in line)
        fmus = sorted(Path(tmp).rglob("*.fmu"))
        if result.returncode != 0 or not fmus:
            sys.stderr.write(diagnostics + "\n")
            return result.returncode or 1
        build_fmu.build(fmus[0], args.out.resolve())
    return 0


if __name__ == "__main__":
    sys.exit(main())
