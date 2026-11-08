"""A comparison as one self-contained HTML page: the measures and checks, the
two tracks from above, and every channel of A and B overlaid with B - A."""
from __future__ import annotations

import html
import math

import numpy as np

import traces
from compare import DERIVED, Result, _error_series, _g

W, H, HE, PAD = 760, 150, 70, 46
RIGHT = 24  # room for the last tick label


def _path(x: np.ndarray, y: np.ndarray, sx, sy) -> str:
    ok = np.isfinite(x) & np.isfinite(y)
    parts, pen = [], "M"
    for xi, yi, good in zip(x, y, ok):
        if not good:
            pen = "M"
            continue
        parts.append(f"{pen}{sx(xi):.1f},{sy(yi):.1f}")
        pen = "L"
    return " ".join(parts)


def _thin(*arrays, n=1500):
    step = max(1, len(arrays[0]) // n)
    return [a[::step] for a in arrays]


def _range(*arrays):
    v = np.concatenate([a[np.isfinite(a)] for a in arrays if a is not None] or [np.zeros(1)])
    if not len(v):
        return -1.0, 1.0
    lo, hi = float(v.min()), float(v.max())
    if hi - lo < 1e-9:
        lo, hi = lo - 1, hi + 1
    pad = 0.05 * (hi - lo)
    return lo - pad, hi + pad


def _axes(x0, x1, y0, y1, height, label):
    """Frame, a few ticks and the y label of one plot."""
    out = [f'<rect x="{PAD}" y="4" width="{W - PAD - RIGHT}" height="{height - 22}" class="frame"/>']
    for i in range(5):
        v = y0 + (y1 - y0) * i / 4
        y = 4 + (height - 22) * (1 - i / 4)
        out.append(f'<text x="{PAD - 4}" y="{y + 3:.1f}" class="tick" text-anchor="end">{_g(v, 3)}</text>')
    for i in range(6):
        v = x0 + (x1 - x0) * i / 5
        x = PAD + (W - PAD - RIGHT) * i / 5
        out.append(f'<text x="{x:.1f}" y="{height - 6}" class="tick" text-anchor="middle">{_g(v, 4)}</text>')
    out.append(f'<text x="{PAD + 4}" y="16" class="label">{html.escape(label)}</text>')
    return out


def _plot(t, series, height, label, zero_line=False):
    x0, x1 = float(t[0]), float(t[-1])
    y0, y1 = _range(*[s for s, _ in series])
    if zero_line:
        y0, y1 = min(y0, 0.0), max(y1, 0.0)

    def sx(v):
        return PAD + (W - PAD - RIGHT) * (v - x0) / (x1 - x0 or 1)

    def sy(v):
        return 4 + (height - 22) * (1 - (v - y0) / (y1 - y0))

    out = [f'<svg viewBox="0 0 {W} {height}" class="plot" role="img" aria-label="{html.escape(label)}">']
    out += _axes(x0, x1, y0, y1, height, label)
    if zero_line:
        out.append(f'<line x1="{PAD}" x2="{W - RIGHT}" y1="{sy(0):.1f}" y2="{sy(0):.1f}" class="zero"/>')
    for s, cls in series:
        tt, ss = _thin(t, s)
        out.append(f'<path d="{_path(tt, ss, sx, sy)}" class="{cls}"/>')
    out.append("</svg>")
    return "\n".join(out)


def _plan(result: Result) -> str:
    pos = ("pos_n", "pos_e")
    if not all(result.a.has(c) and result.b.has(c) for c in pos):
        return ""
    ga, gb = result.grid + result.zero_a, result.grid + result.zero_b + result.shift
    an, ae = (result.a.at(c, ga) for c in pos)
    bn, be = (result.b.at(c, gb) for c in pos)
    e0, e1 = _range(ae, be)
    n0, n1 = _range(an, bn)
    span = max(e1 - e0, n1 - n0)
    ec, nc = (e0 + e1) / 2, (n0 + n1) / 2
    e0, e1, n0, n1 = ec - span / 2, ec + span / 2, nc - span / 2, nc + span / 2
    size = 420

    def sx(v):
        return PAD + (size - PAD - RIGHT) * (v - e0) / span

    def sy(v):
        return 8 + (size - PAD - RIGHT) * (1 - (v - n0) / span)

    out = [f'<svg viewBox="0 0 {size} {size}" class="plan" role="img" aria-label="tracks from above">',
           f'<rect x="{PAD}" y="8" width="{size - PAD - RIGHT}" height="{size - PAD - RIGHT}" class="frame"/>']
    for i in range(5):
        v = e0 + span * i / 4
        out.append(f'<text x="{sx(v):.1f}" y="{size - PAD + 16}" class="tick" text-anchor="middle">{_g(v, 3)}</text>')
        v = n0 + span * i / 4
        out.append(f'<text x="{PAD - 4}" y="{sy(v) + 3:.1f}" class="tick" text-anchor="end">{_g(v, 3)}</text>')
    out.append(f'<text x="{size / 2}" y="{size - 8}" class="label" text-anchor="middle">east, m</text>')
    out.append(f'<text x="12" y="{size / 2}" class="label" transform="rotate(-90 12 {size / 2})" '
               'text-anchor="middle">north, m</text>')
    for n, e, cls in ((an, ae, "a"), (bn, be, "b")):
        n, e = _thin(n, e)
        out.append(f'<path d="{_path(e, n, sx, sy)}" class="{cls}"/>')
    out.append("</svg>")
    return "\n".join(out)


CSS = """
:root { --bg:#fbfbfa; --fg:#1d1d1b; --muted:#6b6b66; --line:#d9d8d2; --a:#2a6fb0; --b:#d0632a;
        --err:#7a5bb5; --ok:#2e7d4f; --bad:#b3261e; --card:#ffffff; }
@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) {
  --bg:#161615; --fg:#ecebe6; --muted:#9a9993; --line:#3a3936; --a:#6aa8e6; --b:#f08a50;
  --err:#ab8ee6; --ok:#5fbf86; --bad:#f2786f; --card:#1f1f1d; } }
:root[data-theme="dark"] { --bg:#161615; --fg:#ecebe6; --muted:#9a9993; --line:#3a3936; --a:#6aa8e6;
  --b:#f08a50; --err:#ab8ee6; --ok:#5fbf86; --bad:#f2786f; --card:#1f1f1d; }
body { background:var(--bg); color:var(--fg); font:14px/1.45 system-ui, sans-serif; margin:0; padding:24px 16px; }
main { max-width:800px; margin:0 auto; }
h1 { font-size:20px; margin:0 0 4px; } h2 { font-size:16px; margin:28px 0 8px; }
.meta { color:var(--muted); font-size:13px; } .meta b { color:var(--fg); font-weight:600; }
.key-a { color:var(--a); font-weight:600; } .key-b { color:var(--b); font-weight:600; }
.verdict { display:inline-block; padding:2px 10px; border-radius:4px; font-weight:600; margin:8px 0; }
.verdict.ok { color:var(--ok); border:1px solid var(--ok); } .verdict.bad { color:var(--bad); border:1px solid var(--bad); }
.scroll { overflow-x:auto; }
table { border-collapse:collapse; font-variant-numeric:tabular-nums; font-size:13px; width:100%; }
th, td { padding:4px 8px; border-bottom:1px solid var(--line); text-align:right; white-space:nowrap; }
th:first-child, td:first-child { text-align:left; } th { color:var(--muted); font-weight:500; }
td.bad { color:var(--bad); font-weight:600; } td.ok { color:var(--ok); }
svg { width:100%; height:auto; display:block; } svg.plan { max-width:420px; }
.frame { fill:var(--card); stroke:var(--line); } .tick { fill:var(--muted); font-size:10px; }
.label { fill:var(--fg); font-size:11px; } .zero { stroke:var(--line); }
path { fill:none; stroke-width:1.3; } path.a { stroke:var(--a); } path.b { stroke:var(--b); }
path.err { stroke:var(--err); }
"""


def render(result: Result, title: str = "Differential test") -> str:
    a, b = result.a, result.b
    out = ["<!doctype html><html lang='en'><head><meta charset='utf-8'>",
           "<meta name='viewport' content='width=device-width, initial-scale=1'>",
           f"<title>{html.escape(title)}</title><style>{CSS}</style></head><body><main>",
           f"<h1>{html.escape(title)}</h1>",
           f"<p class='meta'><span class='key-a'>A</span> <b>{html.escape(a.name)}</b> &nbsp; "
           f"<span class='key-b'>B</span> <b>{html.escape(b.name)}</b><br>",
           f"aligned on {html.escape(str(result.settings['align']))}"
           + (f", B shifted {result.shift:+.3f} s by {html.escape(result.settings['xcorr'])}"
              if result.settings["xcorr"] else "")
           + f"; window {result.grid[0]:.2f}–{result.grid[-1]:.2f} s at "
             f"{result.settings['rate']:g} Hz</p>"]
    if result.checked:
        failed = sum(not m.ok for m in result.measures)
        out.append(f"<div class='verdict {'ok' if result.ok else 'bad'}'>"
                   + ("passed" if result.ok else f"failed: {failed} channel(s)") + "</div>")
    out.append("<div class='scroll'><table><tr><th>channel</th><th>unit</th><th>rmse</th><th>max</th>"
               "<th>bias</th><th>nrmse</th><th>corr</th><th>check</th></tr>")
    for m in result.measures:
        check = "; ".join(f"{c} {_g(v)} ≤ {_g(l)}" for c, l, v, _ in m.checks)
        cls = "" if not m.checks else ("ok" if m.ok else "bad")
        out.append(f"<tr><td>{html.escape(m.channel)}</td><td>{html.escape(m.unit)}</td>"
                   f"<td>{_g(m.rmse)}</td><td>{_g(m.max)}</td><td>{_g(m.bias)}</td>"
                   f"<td>{_g(m.nrmse, 3)}</td><td>{_g(m.corr, 3)}</td>"
                   f"<td class='{cls}'>{html.escape(check)}</td></tr>")
    out.append("</table></div>")
    plan = _plan(result)
    if plan:
        out += ["<h2>Tracks from above</h2>", plan]
    measured = {m.channel for m in result.measures if m.n}
    t = result.grid
    for group, names in traces.GROUPS.items():
        names = [n for n in names if n in measured]
        if not names:
            continue
        out.append(f"<h2>{html.escape(group)}</h2>")
        for n in names:
            va, vb = result.series(n)
            unit = traces.unit(n)
            out.append(_plot(t, [(va, "a"), (vb, "b")], H, f"{n} ({unit})" if unit else n))
            out.append(_plot(t, [(_error_series(result, n), "err")], HE, "B − A", zero_line=True))
    derived = [n for n in DERIVED if n in measured]
    if derived:
        out.append("<h2>derived</h2>")
        for n in derived:
            unit = "rad" if n == "attitude" else "m"
            out.append(_plot(t, [(_error_series(result, n), "err")], H, f"{n} difference ({unit})",
                             zero_line=True))
    out.append("<p class='meta'>time axis: s after each trace's time zero</p></main></body></html>")
    return "\n".join(out)
