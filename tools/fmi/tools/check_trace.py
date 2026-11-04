#!/usr/bin/env python3
"""Evaluate a harness's acceptance checks over its world trace.

usage: check_trace.py checks.toml trace.csv

Each [[check]] selects a series and a condition.

Series (one of):
  variable = "m.omega"        a trace column; an exact name is used as is
                              (so "m.x[3]" works), anything else is a glob
                              ("m.rotor*" selects several columns)
  expr = "{m.a} - {m.b}"      an expression over columns, evaluated per row;
                              "{m.v@1.5}" is column m.v at t = 1.5 s (the last
                              sample at or before it), so differences between
                              times need no absolute value: "{m.v@2} - {m.v@1}".
                              Available: abs sqrt exp log sin cos tan atan2 hypot
                              round floor min max pi, norm(a, b, ...) = Euclidean
                              norm, and t (the row's time in s).

Where (`at`): "end" (default), "all", "max", "min", "mean", or a time in s.
`from` / `to` (seconds) restrict all/max/min/mean to a window. The trace row at
t = 0 is the initialized state before the scenario's first inputs act; an input
held from t takes effect in the row at t + step.

Condition: `finite = true`, `range = [lo, hi]`, or `near = value` with `tol`
(absolute) or `rtol` (relative). Exit status 0 when every check passes.
Boolean trace cells read as 1 (true) and 0 (false).

`known = "reason"` marks a check that states the model's contract but fails
because of a known defect (named in the reason, with where it is tracked): a
failure prints KNOWN and does not fail the harness; a pass prints FIXED, so the
marker can be removed.
"""
import csv, fnmatch, math, re, sys, tomllib
from pathlib import Path

CELL = {"true": 1.0, "false": 0.0, "": math.nan, "nan": math.nan}
PLACEHOLDER = re.compile(r"\{([^{}@]+)(?:@([^{}]+))?\}")
FUNCTIONS = {name: getattr(math, name) for name in
             ("sqrt", "exp", "log", "sin", "cos", "tan", "atan2", "hypot")}
FUNCTIONS.update(abs=abs, min=min, max=max, round=round, floor=math.floor, pi=math.pi, norm=lambda *xs: math.sqrt(sum(x * x for x in xs)))


def join_subscripts(cells: list[str]) -> list[str]:
    """Header names as written: world does not quote them, so a scalarized
    multi-index element such as m.x.history[1,2] arrives split at its comma."""
    names: list[str] = []
    for cell in cells:
        if names and names[-1].count("[") > names[-1].count("]"):
            names[-1] += "," + cell
        else:
            names.append(cell)
    return names


def load(trace: Path):
    with trace.open() as source:
        rows = list(csv.reader(source))
    header, data = join_subscripts(rows[0]), rows[1:]
    times = [int(r[0]) * 1e-9 for r in data]
    columns = {name: [CELL[r[i]] if r[i] in CELL else float(r[i]) for r in data]
               for i, name in enumerate(header) if i}
    return times, columns


def column_names(pattern: str, columns) -> list[str]:
    if pattern in columns:
        return [pattern]
    return [n for n in columns if fnmatch.fnmatchcase(n, pattern)]


def index_at(times, t: float) -> int:
    return max((i for i, time in enumerate(times) if time <= t + 1e-12), default=0)


def expression_series(text: str, times, columns):
    """(label, per-row values) for an expr check; raises KeyError on unknown columns."""
    names = []

    def substitute(match):
        column, at = match.group(1).strip(), match.group(2)
        if column not in columns:
            raise KeyError(column)
        names.append((column, None if at is None else index_at(times, float(at))))
        return f"_v[{len(names) - 1}]"

    code = compile(PLACEHOLDER.sub(substitute, text), "<expr>", "eval")
    values = []
    for row in range(len(times)):
        operands = [columns[c][row if at is None else at] for c, at in names]
        try:
            values.append(float(eval(code, {"__builtins__": {}}, {**FUNCTIONS, "_v": operands, "t": times[row]})))
        except (ArithmeticError, ValueError):
            values.append(math.nan)
    return values


def select(check, times, values):
    at = str(check.get("at", "end"))
    lo, hi = check.get("from", -math.inf), check.get("to", math.inf)
    window = [v for t, v in zip(times, values) if lo - 1e-12 <= t <= hi + 1e-12]
    if at == "all":
        return window
    if at == "end":
        return values[-1:]
    if at in ("max", "min", "mean"):
        finite = [v for v in window if math.isfinite(v)]
        if not finite:
            return [math.nan]
        return [{"max": max, "min": min, "mean": lambda xs: sum(xs) / len(xs)}[at](finite)]
    return [values[index_at(times, float(at))]]


def failures_of(check, name, samples):
    out = []
    for value in samples:
        if check.get("finite") and not math.isfinite(value):
            out.append(f"{name} = {value} is not finite")
        if "range" in check:
            lo, hi = check["range"]
            if not (lo <= value <= hi):
                out.append(f"{name} = {value:.9g} outside [{lo}, {hi}]")
        if "near" in check:
            target = check["near"]
            tol = max(check.get("tol", 0.0), check.get("rtol", 0.0) * abs(target))
            if "tol" not in check and "rtol" not in check:
                tol = 1e-6
            if not abs(value - target) <= tol:
                out.append(f"{name} = {value:.9g} not within {tol:.3g} of {target}")
    return out


def evaluate(check, times, columns):
    if "expr" in check:
        try:
            series = {check["expr"]: expression_series(check["expr"], times, columns)}
        except KeyError as missing:
            return [f"no trace column {missing}"]
    else:
        names = column_names(check["variable"], columns)
        if not names:
            return [f"no trace column matches {check['variable']}"]
        series = {n: columns[n] for n in names}
    failures = []
    for name, values in series.items():
        failures += failures_of(check, name, select(check, times, values))
    return failures


def main() -> int:
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    checks = tomllib.loads(Path(sys.argv[1]).read_text()).get("check", [])
    times, columns = load(Path(sys.argv[2]))
    failed = known = 0
    for check in checks:
        failures = evaluate(check, times, columns)
        subject = check.get("expr", check.get("variable"))
        label = f"{subject} @ {check.get('at', 'end')}: {check.get('why', '')}".rstrip(": ")
        if "known" in check:
            print(("KNOWN " if failures else "FIXED ") + label)
            print(("     known defect: " if failures else "     no longer fails; drop `known`: ") + check["known"])
        else:
            print(("FAIL " if failures else "pass ") + label)
        for failure in failures[:5]:
            print("     " + failure)
        if len(failures) > 5:
            print(f"     ... {len(failures) - 5} more")
        if "known" in check:
            known += bool(failures)
        else:
            failed += bool(failures)
    note = f" ({known} known failure{'s' * (known != 1)})" if known else ""
    print(f"checks: {len(checks) - failed - known}/{len(checks)} passed{note}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
