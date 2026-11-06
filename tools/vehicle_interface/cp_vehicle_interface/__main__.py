"""Command line: check an FMU against the vehicle interface, or prepare it for the runtime.

  python -m cp_vehicle_interface check MODEL.fmu [--require CAP ...] [--profile NAME]
  python -m cp_vehicle_interface prepare MODEL.fmu OUT_DIR [--require CAP ...]
         [--profile NAME] [--set NAME=VALUE ...] [--shim NAME]
  python -m cp_vehicle_interface capabilities
"""
from __future__ import annotations

import argparse
import sys

from .check import STANDARD_PROFILE, check
from .contract import load_contract
from .descriptor import PrepareError, prepare
from .metadata import MetadataError, parse, read_model_description


def _overrides(items: list[str]) -> dict[str, float]:
    result = {}
    for item in items:
        name, sep, value = item.partition("=")
        if not sep or not name:
            raise SystemExit(f"--set expects NAME=VALUE, not {item!r}")
        try:
            result[name] = float(value)
        except ValueError:
            raise SystemExit(f"--set {name}: {value!r} is not a number") from None
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="cp_vehicle_interface", description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("check", "prepare"):
        p = sub.add_parser(name)
        p.add_argument("fmu")
        if name == "prepare":
            p.add_argument("out_dir")
            p.add_argument("--set", action="append", default=[], metavar="NAME=VALUE")
            p.add_argument("--shim", metavar="NAME",
                           help="also write OUT_DIR/shim/ to link the model into a host "
                                "as cpv_model_NAME (cmake -DCPV_MODELS=OUT_DIR/shim)")
        p.add_argument("--require", action="append", default=[], metavar="CAPABILITY")
        p.add_argument("--profile", default=STANDARD_PROFILE)
    sub.add_parser("capabilities")
    args = parser.parse_args(argv)
    contract = load_contract()
    if args.command == "capabilities":
        for cap in contract.capabilities.values():
            print(f"{cap.name}: {cap.summary}")
            for f in cap.fields:
                shape = "[" + ",".join(map(str, f.shape)) + "]" if f.shape else "scalar"
                kind = "required" if f.required else "optional"
                print(f"  {f.name:28} {f.type} {'/'.join(f.causality):9} {shape:7} {kind}")
        return 0
    try:
        if args.command == "check":
            report = check(parse(read_model_description(args.fmu)), contract, args.require,
                           args.profile)
            errors = report.errors
        else:
            path, _, report = prepare(args.fmu, args.out_dir, required=args.require,
                                      profile=args.profile, overrides=_overrides(args.set),
                                      contract=contract, shim=args.shim)
            errors = []
    except (MetadataError, PrepareError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    for warning in report.warnings:
        print(f"warning: {warning}", file=sys.stderr)
    for error in errors:
        print(f"error: {error}", file=sys.stderr)
    if errors:
        return 1
    provided = ", ".join(sorted(report.provided)) or "none"
    print(f"{report.model}: conforms ({report.profile}); provides {provided}")
    if args.command == "prepare":
        print(f"descriptor: {path}")
        if args.shim:
            print(f"shim: {path.parent / 'shim'} (cpv_model_{args.shim})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
