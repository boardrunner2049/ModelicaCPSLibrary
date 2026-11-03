"""One place that knows how to invoke Rumoca on the library.

Settings come from the environment, falling back to tools/fmi/local.env
(KEY=VALUE lines, untracked; see local.env.example):
  MODELICA_LIBS   directory holding Modelica/, ModelicaServices/ and Complex.mo (required)
  RUMOCA          the Rumoca binary (default: tools/fmi/bin/rumoca if present, else `rumoca`)
  FIRE_FMI_CACHE  Rumoca cache directory (default: tools/fmi/.cache)
"""
import os, re, shutil
from pathlib import Path

FMI_ROOT = Path(__file__).resolve().parent.parent
LIBRARY_ROOT = FMI_ROOT.parent.parent
PREFIX = "ModelicaCPSLibrary."


def _local_settings() -> dict[str, str]:
    path = FMI_ROOT / "local.env"
    if not path.exists():
        return {}
    pairs = (line.split("=", 1) for line in path.read_text().splitlines()
             if "=" in line and not line.lstrip().startswith("#"))
    return {k.strip().rstrip("?").strip(): os.path.expanduser(v.strip()) for k, v in pairs}


def setting(name: str, default: str | None = None) -> str | None:
    return os.environ.get(name) or _local_settings().get(name) or default


def _default_rumoca() -> str:
    local = FMI_ROOT / "bin" / "rumoca"
    return str(local) if local.exists() else (shutil.which("rumoca") or "rumoca")


RUMOCA = Path(setting("RUMOCA", _default_rumoca()))
CACHE = Path(setting("FIRE_FMI_CACHE", str(FMI_ROOT / ".cache")))


def modelica_libs() -> Path:
    libs = setting("MODELICA_LIBS")
    if not libs:
        raise SystemExit("MODELICA_LIBS is not set: point it (environment or tools/fmi/local.env) "
                         "at a directory holding Modelica/, ModelicaServices/ and Complex.mo")
    return Path(libs)


_COMMENT = re.compile(r"//[^\n]*|/\*.*?\*/", re.S)
_STRING = re.compile(r'"(?:[^"\\]|\\.)*"', re.S)
_HEAD = re.compile(r"\b(?:(?:encapsulated|partial|final|replaceable|redeclare|inner|outer|expandable|impure|pure|operator)\s+)*"
                   r"(model|block|class|package|record|connector|function|type)\s+(\w+)\b(?!\s*=)")


def models() -> list[dict]:
    """Every non-partial model, block or class of the library (outside deprecated/ and tools/)
    as {"model": qualified name, "file": .mo path relative to the library root}."""
    out = []
    for f in sorted(LIBRARY_ROOT.rglob("*.mo")):
        rel = f.relative_to(LIBRARY_ROOT)
        if rel.parts[0] in ("deprecated", "tools"):
            continue
        src = _STRING.sub('""', _COMMENT.sub("", f.read_text()))
        within = re.search(r"^\s*within\s+([\w.]*)\s*;", src, re.M)
        stack = []
        for m in re.finditer(r"\bend\s+(\w+)\s*;|" + _HEAD.pattern, src):
            if m.group(0).startswith("end") and m.group(1):
                if stack and stack[-1] == m.group(1):
                    stack.pop()
                continue
            kind, name = m.group(2), m.group(3)
            if re.match(r"\s*=", src[m.end():m.end() + 200]):
                continue
            prefix = src[max(0, m.start() - 40):m.end()]
            partial = re.search(r"\bpartial\s+(model|block|class)\s+" + name + r"\b", prefix)
            path = ".".join(p for p in [within.group(1) if within else ""] + stack + [name] if p)
            stack.append(name)
            if kind in ("model", "block", "class") and not partial:
                out.append({"model": path, "file": str(rel)})
    return out


def model_file(model: str) -> Path:
    """The library file that declares `model` (short or qualified name)."""
    qualified = model if model.startswith(PREFIX) else PREFIX + model
    for entry in models():
        if entry["model"] == qualified:
            return LIBRARY_ROOT / entry["file"]
    raise KeyError(f"unknown library model {model}")


def compile_command(source: Path, model: str, *extra: str, binary: Path = RUMOCA) -> list[str]:
    """Compile `model` declared in `source` with this library and the MSL on the source path."""
    libs = modelica_libs()
    roots = [libs / "Modelica", libs / "ModelicaServices", libs / "Complex.mo", LIBRARY_ROOT]
    command = [str(binary), "compile", str(source), "--model", model]
    for root in roots:
        command += ["--source-root", str(root)]
    return command + ["--cache-dir", str(CACHE), *extra]
