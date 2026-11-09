#!/usr/bin/env python3
"""Vehicle builder: pick a vehicle's parts from CP Models in the browser, get its
TOML, and fly it in ArduPilot SITL.

usage: python3 tools/vehicle_builder/server.py [--port 8765] [--host 127.0.0.1]

Open http://127.0.0.1:8765 (from another machine: ssh -L 8765:127.0.0.1:8765 HOST).
Every change is checked by `fire-compose validate`. "Save" writes the run file;
"Fly" runs runtime/hosts/ardupilot/run.sh on it here (build, SITL, and the
MAVProxy map on this machine's display, or a headless mission check), and
streams its output. Building needs what run.sh needs (README there) on PATH.
Standard library only.
"""
from __future__ import annotations

import argparse
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import threading

import parts

HERE = Path(__file__).resolve().parent
ROOT = parts.ROOT
RUN = ROOT / "runtime" / "hosts" / "ardupilot" / "run.sh"
WORK = ROOT / "build" / "vehicle-builder"
NAME = re.compile(r"[a-z][a-z0-9_]{0,47}")
MODES = {"map": [], "mission": ["--mission"], "headless": ["--headless"]}

CATALOG: dict = {}
RUNS: dict[int, "Run"] = {}
LOCK = threading.Lock()


class Run:
    """One run.sh process and its output."""

    def __init__(self, ident: int, name: str, args: list[str], instance: int):
        self.ident, self.name, self.instance = ident, name, instance
        self.lines: list[str] = []
        env = dict(os.environ, RUN_NONINTERACTIVE="1", PYTHONUNBUFFERED="1")
        # Own process group, so Stop ends SITL, MAVProxy and the build with it.
        self.proc = subprocess.Popen([str(RUN), *args], cwd=ROOT, env=env,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     stdin=subprocess.DEVNULL, text=True, bufsize=1,
                                     start_new_session=True)
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in self.proc.stdout:
            self.lines.append(line.rstrip("\n"))
        self.proc.wait()
        self.lines.append(f"[run.sh exited with {self.proc.returncode}]")

    def stop(self):
        if self.proc.poll() is None:
            os.killpg(self.proc.pid, signal.SIGTERM)

    def status(self, start: int) -> dict:
        return {"id": self.ident, "name": self.name, "lines": self.lines[start:],
                "next": len(self.lines), "running": self.proc.poll() is None,
                "exit": self.proc.returncode}


def check(choice: dict) -> dict:
    """Compose, write the run file text and validate it with fire-compose."""
    try:
        run, physics = parts.compose(choice, CATALOG)
    except (ValueError, KeyError) as e:
        return {"ok": False, "message": str(e), "toml": ""}
    text = parts.run_file_text(run, "# Composed with tools/vehicle_builder\n")
    with tempfile.NamedTemporaryFile("w", suffix=".toml", delete=False) as f:
        f.write(text)
    try:
        table = ["--config-table", "physics"]
        r = subprocess.run([str(parts.compose_binary()), "validate", f.name, *table],
                           capture_output=True, text=True)
        reports = []
        if r.returncode == 0:
            j = subprocess.run([str(parts.compose_binary()), "validate", f.name, *table, "--json"],
                               capture_output=True, text=True)
            if j.returncode == 0:
                reports = json.loads(j.stdout).get("parts", [])
    finally:
        os.unlink(f.name)
    message = (r.stdout + r.stderr).strip().removeprefix("fire-compose: ")
    return {"ok": r.returncode == 0, "message": message, "toml": text,
            "model_name": physics["model_name"], "reports": reports}


class Handler(BaseHTTPRequestHandler):
    server_version = "vehicle-builder"

    def log_message(self, fmt, *args):  # quiet
        pass

    def send(self, body, status=HTTPStatus.OK, kind="application/json"):
        data = body if isinstance(body, bytes) else json.dumps(body).encode()
        self.send_response(status)
        self.send_header("Content-Type", kind)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def body(self) -> dict:
        n = int(self.headers.get("Content-Length", 0))
        return json.loads(self.rfile.read(n) or b"{}")

    def do_GET(self):
        path = self.path.split("?")[0]
        if path == "/":
            self.send((HERE / "index.html").read_bytes(), kind="text/html; charset=utf-8")
        elif path == "/garage.js":
            self.send((HERE / "garage.js").read_bytes(), kind="text/javascript; charset=utf-8")
        elif path == "/api/catalog":
            self.send(CATALOG)
        elif m := re.fullmatch(r"/api/runs/(\d+)", path):
            start = int(self.path.partition("from=")[2] or 0)
            run = RUNS.get(int(m.group(1)))
            self.send(run.status(start) if run else {"error": "no such run"},
                      HTTPStatus.OK if run else HTTPStatus.NOT_FOUND)
        else:
            self.send({"error": "not found"}, HTTPStatus.NOT_FOUND)

    def do_POST(self):
        # Same-origin only: refuse requests a page from another site could send.
        origin = self.headers.get("Origin")
        if origin and origin.split("://", 1)[-1] != self.headers.get("Host"):
            return self.send({"error": "cross-origin request refused"}, HTTPStatus.FORBIDDEN)
        try:
            req = self.body()
        except json.JSONDecodeError:
            return self.send({"error": "bad JSON"}, HTTPStatus.BAD_REQUEST)
        path = self.path.split("?")[0]
        if path == "/api/check":
            return self.send(check(req.get("choice", {})))
        if path == "/api/save":
            return self.send(self.save(req))
        if path == "/api/fly":
            return self.send(self.fly(req))
        if m := re.fullmatch(r"/api/runs/(\d+)/stop", path):
            run = RUNS.get(int(m.group(1)))
            if run:
                run.stop()
            return self.send({"stopped": bool(run)})
        self.send({"error": "not found"}, HTTPStatus.NOT_FOUND)

    @staticmethod
    def save(req: dict) -> dict:
        result = check(req.get("choice", {}))
        if not result["ok"]:
            return {"ok": False, "message": "not saved: " + result["message"]}
        target = Path(os.path.expanduser(req.get("path", ""))).resolve() if req.get("path") \
            else None
        if not target or target.suffix != ".toml":
            return {"ok": False, "message": "give a path ending in .toml"}
        if target.exists() and not req.get("overwrite"):
            return {"ok": False, "exists": True, "message": f"{target} exists"}
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(result["toml"])
        return {"ok": True, "message": f"saved {target}", "path": str(target)}

    @staticmethod
    def fly(req: dict) -> dict:
        choice = req.get("choice", {})
        name = choice.get("sitl", {}).get("name", "")
        if not NAME.fullmatch(name or ""):
            return {"ok": False, "message": "the SITL name must match [a-z][a-z0-9_]*"}
        result = check(choice)
        if not result["ok"]:
            return {"ok": False, "message": "not flown: " + result["message"]}
        mode = req.get("mode", "mission")
        if mode not in MODES:
            return {"ok": False, "message": f"mode must be one of {', '.join(MODES)}"}
        with LOCK:
            if any(r.name == name and r.proc.poll() is None for r in RUNS.values()):
                return {"ok": False, "message": f"{name} is already running"}
            busy = {r.instance for r in RUNS.values() if r.proc.poll() is None}
            instance = min(set(range(len(busy) + 1)) - busy)
            WORK.mkdir(parents=True, exist_ok=True)
            path = WORK / f"{name}.toml"
            path.write_text(result["toml"])
            ident = max(RUNS, default=0) + 1
            RUNS[ident] = Run(ident, name, [str(path), *MODES[mode], "--instance",
                                            str(instance)], instance)
        return {"ok": True, "id": ident, "path": str(path), "instance": instance}


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", type=int, default=8765)
    p.add_argument("--host", default="127.0.0.1",
                   help="address to listen on (default: this machine only)")
    a = p.parse_args()
    CATALOG.update(parts.catalog())
    server = ThreadingHTTPServer((a.host, a.port), Handler)
    print(f"vehicle builder: http://{a.host}:{a.port}  (Ctrl-C to stop)", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        for run in RUNS.values():
            run.stop()


if __name__ == "__main__":
    main()
