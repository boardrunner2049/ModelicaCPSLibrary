"""Prepare an FMU for the in-process runtime: check it and write a bound descriptor.

The descriptor is an internal runtime artifact, not the contract. It records
the resolved value references, shapes, start values, the capabilities the
host required, the overridable parameters, the effective overrides, and the
SHA-256 digests that bind it to the extracted FMU. The runtime refuses a
descriptor whose digests no longer match the files it would load.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import platform
import shutil
import zipfile

from .check import STANDARD_PROFILE, Report, check
from .contract import Contract, load_contract
from .metadata import Metadata, parse, read_model_description

DESCRIPTOR_NAME = "vehicle.json"
EXTRACTED_DIR = "fmu"


class PrepareError(ValueError):
    """The FMU or the requested configuration cannot be used; the message lists every reason."""

    def __init__(self, messages: list[str]):
        super().__init__("\n".join(messages))
        self.messages = messages


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def platform_tuple() -> str:
    machine = {"amd64": "x86_64", "arm64": "aarch64"}.get(platform.machine().lower(),
                                                           platform.machine().lower())
    system = {"darwin": "darwin", "windows": "windows"}.get(platform.system().lower(), "linux")
    return f"{machine}-{system}"


# FMI 2 platform directory names. Only the legacy profile accepts them: some
# legacy exports (FastDyn's pinned Rumoca) used FMI 2 layout inside FMI 3 FMUs.
FMI2_PLATFORM = {"x86_64-linux": "linux64", "x86_64-darwin": "darwin64",
                 "x86_64-windows": "win64"}


def find_library(extracted: Path, identifier: str, profile: str) -> tuple[Path | None, str | None]:
    """The Co-Simulation binary for this platform, and a warning if it needed legacy layout."""
    standard = extracted / "binaries" / platform_tuple() / f"{identifier}.so"
    if standard.is_file():
        return standard, None
    legacy_dir = FMI2_PLATFORM.get(platform_tuple())
    if profile != STANDARD_PROFILE and legacy_dir:
        legacy = extracted / "binaries" / legacy_dir / f"{identifier}.so"
        if legacy.is_file():
            return legacy, (f"binary found under the FMI 2 platform directory "
                            f"binaries/{legacy_dir}/ instead of binaries/{platform_tuple()}/; "
                            "accepted under the legacy profile only")
    return None, None


def overridable_parameters(meta: Metadata) -> dict[str, dict]:
    """Scalar Float64 parameters a host may set before initialization.

    Structural parameters, arrays, calculated parameters, constants and other
    types are not overridable in this version of the runtime.
    """
    result = {}
    for var in meta.variables.values():
        if (var.type == "Float64" and var.causality == "parameter"
                and var.variability in ("fixed", "tunable") and not var.dimensions):
            start = None
            if var.start and len(var.start) == 1:
                try:
                    start = float(var.start[0])
                except ValueError:
                    start = None
            result[var.name] = {"vr": var.value_reference, "start": start}
    return result


def _extract(fmu: Path, destination: Path) -> None:
    if destination.exists():
        shutil.rmtree(destination)
    if fmu.is_dir():
        shutil.copytree(fmu, destination)
        return
    destination.mkdir(parents=True)
    root = destination.resolve()
    with zipfile.ZipFile(fmu) as archive:
        for member in archive.infolist():
            target = (destination / member.filename).resolve()
            if root != target and root not in target.parents:
                raise PrepareError([f"{fmu}: archive member {member.filename!r} escapes the "
                                    "extraction directory"])
        archive.extractall(destination)


def build_descriptor(report: Report, meta: Metadata, contract: Contract, *, fmu: Path,
                     extracted: Path, library: Path, out_dir: Path, required: list[str],
                     overrides: dict[str, float]) -> dict:
    resources = extracted / "resources"
    capabilities = {}
    for cap_name, fields in report.provided.items():
        capabilities[cap_name] = {
            name: None if resolved is None else {
                "vr": resolved.value_reference,
                "count": resolved.count,
                "start": list(resolved.start) if resolved.start is not None else None,
                "unit_declared": resolved.unit_declared,
            }
            for name, resolved in fields.items()
        }
    marker = None
    if report.marker is not None:
        marker = {"name": contract.marker.name, "vr": report.marker.value_reference,
                  "value": contract.marker.value}
    return {
        "descriptor_schema": contract.descriptor_schema,
        "contract": {"name": contract.name, "version": contract.version},
        "profile": report.profile,
        "reference_point": contract.reference_point,
        "warnings": report.warnings,
        "artifact": {
            "source": str(fmu.resolve()),
            "source_sha256": sha256(fmu) if fmu.is_file() else None,
            "model_name": meta.model_name,
            "generation_tool": meta.generation_tool,
            "fmi_version": meta.fmi_version,
            "model_identifier": meta.model_identifier,
            "instantiation_token": meta.instantiation_token,
            "model_description": str((extracted / "modelDescription.xml").relative_to(out_dir)),
            "model_description_sha256": sha256(extracted / "modelDescription.xml"),
            "library": str(library.relative_to(out_dir)),
            "library_sha256": sha256(library),
            "resources": str(resources.relative_to(out_dir)) if resources.is_dir() else None,
        },
        "marker": marker,
        "required_capabilities": list(required),
        "capabilities": capabilities,
        "parameters": overridable_parameters(meta),
        "overrides": overrides,
    }


def prepare(fmu: Path | str, out_dir: Path | str, *, required: list[str] = (),
            profile: str = STANDARD_PROFILE, overrides: dict[str, float] | None = None,
            contract: Contract | None = None) -> tuple[Path, dict, Report]:
    """Check the FMU, extract it under out_dir and write out_dir/vehicle.json.

    Raises PrepareError with every structural error; nothing is written then.
    """
    fmu, out_dir = Path(fmu), Path(out_dir)
    contract = contract or load_contract()
    meta = parse(read_model_description(fmu))
    report = check(meta, contract, list(required), profile)
    overrides = dict(overrides or {})
    allowed = overridable_parameters(meta)
    for name, value in overrides.items():
        if name not in allowed:
            var = meta.variables.get(name)
            why = "does not exist" if var is None else (
                f"is {var.type} {var.causality}/{var.variability}"
                f"{' with dimensions' if var.dimensions else ''}; only scalar Float64 "
                "parameters with variability fixed or tunable can be overridden")
            report.error(f"override {name!r} {why}")
        elif not isinstance(value, (int, float)) or not math.isfinite(value):
            report.error(f"override {name!r} must be a finite number, not {value!r}")
    if not report.ok:
        raise PrepareError(report.errors)
    out_dir.mkdir(parents=True, exist_ok=True)
    extracted = out_dir / EXTRACTED_DIR
    _extract(fmu, extracted)
    library, layout_warning = find_library(extracted, meta.model_identifier, profile)
    if library is None:
        shutil.rmtree(extracted)
        raise PrepareError([f"{report.model}: no Co-Simulation binary for {platform_tuple()} "
                            f"(binaries/{platform_tuple()}/{meta.model_identifier}.so)"])
    if layout_warning:
        report.warn(layout_warning)
    descriptor = build_descriptor(report, meta, contract, fmu=fmu, extracted=extracted,
                                  library=library,
                                  out_dir=out_dir, required=list(required),
                                  overrides={k: float(v) for k, v in overrides.items()})
    path = out_dir / DESCRIPTOR_NAME
    path.write_text(json.dumps(descriptor, indent=2) + "\n")
    return path, descriptor, report
