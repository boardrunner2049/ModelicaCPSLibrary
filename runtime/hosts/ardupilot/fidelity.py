#!/usr/bin/env python3
"""Write a run file: one airframe with one part at one fidelity level.

usage: fidelity.py PART KIND LEVEL OUT.toml

The airframe is configs/KIND.toml. Only PART changes with LEVEL:

  imu         I0 ideal (the airframe's own), I1 digital, I2 errors,
              I3 electromechanical: [models.imu] and [imu] of configs/quad_iN.toml,
              the same for every airframe.
  propulsion  P0 first-order speed (the airframe's own), P1 average-voltage
              first-order motor, P2 DC motor with winding dynamics, P3
              three-phase PM motor with averaged commutation, P4 the same with
              a switching inverter: ESC, motor, power and rotor inertia of
              configs/quad_pN.toml. Multirotors only.
  magnetometer, barometer, gnss
              M0/B0/G0 ideal (the airframe's own), M1/B1/G1 sampled with noise,
              drift and latency: the parameters of
              runtime/hosts/ardupilot/examples/rover_all_sensors.toml.
  drive, steering (rover), servos (fixed-wing)
              D0/S0 the plant's first-order models, D1 DC motor with winding
              inductance, S1 rate-limited servos (library default parameters).

Parameters are the library's illustrative ones, not hardware calibration.
The levels and their parameters are tools/compose/parts.py's.
"""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools" / "compose"))
import parts  # noqa: E402

PARTS = {
    # part: (level prefix, levels, airframes)
    "imu": ("i", range(4), ("quad", "rover", "plane")),
    "propulsion": ("p", range(5), ("quad",)),
    "magnetometer": ("m", range(2), ("quad", "rover", "plane")),
    "barometer": ("b", range(2), ("quad", "rover", "plane")),
    "gnss": ("g", range(2), ("quad", "rover", "plane")),
    "drive": ("d", range(2), ("rover",)),
    "steering": ("s", range(2), ("rover",)),
    "servos": ("s", range(2), ("plane",)),
}


def main():
    if len(sys.argv) != 5:
        sys.exit(__doc__)
    part, kind, level, out = sys.argv[1], sys.argv[2], int(sys.argv[3]), Path(sys.argv[4])
    if part not in PARTS:
        sys.exit(f"fidelity: part must be one of {', '.join(PARTS)}")
    prefix, levels, kinds = PARTS[part]
    if kind not in kinds or level not in levels:
        sys.exit(f"fidelity: {part} levels {prefix}{levels[0]}..{prefix}{levels[-1]} "
                 f"on {', '.join(kinds)}")
    doc = parts.apply_level(parts.load_config(kind), part, level)
    doc["model_name"] = f"Fidelity{kind.capitalize()}{prefix.upper()}{level}"
    out.write_text(f"# {kind} with {part} level {prefix.upper()}{level} (fidelity.py)\n"
                   + parts.dumps(doc) + "\n")


if __name__ == "__main__":
    main()
