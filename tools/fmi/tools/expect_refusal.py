#!/usr/bin/env python3
"""Harness for a deliberately invalid FIRE model: FMI export must be refused.

usage: expect_refusal.py MODEL TEXT

Passes when Rumoca refuses to export MODEL and its diagnostics contain TEXT.
"""
import os, subprocess, sys, tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import fire_compile  # noqa: E402


def main() -> int:
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    model, text = sys.argv[1], sys.argv[2]
    command = fire_compile.compile_command(fire_compile.model_file(model), fire_compile.PREFIX + model,
                                           "--target", "fmi3")
    with tempfile.TemporaryDirectory() as tmp:
        result = subprocess.run(command, cwd=tmp, capture_output=True, text=True,
                                env=dict(os.environ, RAYON_NUM_THREADS="1"))
    if result.returncode == 0:
        print(f"FAIL {model}: export succeeded but the model is invalid by design")
        return 1
    if text not in result.stderr:
        print(f"FAIL {model}: refused, but not with '{text}':\n{result.stderr[-1500:]}")
        return 1
    print(f"pass {model}: refused with '{text}'")
    return 0


if __name__ == "__main__":
    sys.exit(main())
