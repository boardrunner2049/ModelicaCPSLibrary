"""Check exported FMU metadata against the vehicle interface contract.

The check is structural: names, types, causality, variability, shapes, start
values, declared units and the version marker. It cannot establish physical
correctness, frames or channel wiring; the conformance tests do that.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math

from .contract import VARIABLE_LENGTH, Capability, Contract, Field
from .metadata import Metadata, Variable

STANDARD_PROFILE = "standard"


@dataclass
class ResolvedField:
    capability: str
    name: str
    value_reference: int
    count: int
    start: tuple[float, ...] | None
    unit_declared: bool


@dataclass
class Report:
    model: str
    profile: str
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    marker: Variable | None = None
    provided: dict[str, dict[str, ResolvedField | None]] = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return not self.errors

    def error(self, message: str) -> None:
        self.errors.append(f"{self.model}: {message}")

    def warn(self, message: str) -> None:
        self.warnings.append(f"{self.model}: {message}")


def _finite_starts(var: Variable) -> tuple[float, ...] | None:
    if var.start is None:
        return None
    try:
        values = tuple(float(v) for v in var.start)
    except ValueError:
        return None
    return values if all(math.isfinite(v) for v in values) else None


def _check_field(report: Report, spec: Field, var: Variable) -> ResolvedField | None:
    """Compare one exported variable with its contract field; None when it does not conform."""
    where = f"{spec.capability} field {spec.name!r}"
    before = len(report.errors)
    if var.type != spec.type:
        report.error(f"{where} must be {spec.type}; the FMU declares {var.type}")
    if var.causality not in spec.causality:
        report.error(f"{where} must have causality {' or '.join(spec.causality)}; "
                     f"the FMU declares {var.causality}")
    if spec.variability and var.variability not in spec.variability:
        report.error(f"{where} must have variability {' or '.join(spec.variability)}; "
                     f"the FMU declares {var.variability}")
    for problem in var.dimension_errors:
        report.error(f"{where}: {problem} (unresolved dimensions are not supported)")
    count = None
    if not var.dimension_errors:
        if len(var.dimensions) != len(spec.shape):
            expected = "a scalar" if not spec.shape else f"{len(spec.shape)} dimension(s)"
            report.error(f"{where} must be {expected}; the FMU declares "
                         f"{len(var.dimensions)} dimension(s)")
        else:
            for want, have in zip(spec.shape, var.dimensions):
                if want == VARIABLE_LENGTH and have < 1:
                    report.error(f"{where} must have at least one element; the FMU declares {have}")
                elif want != VARIABLE_LENGTH and have != want:
                    report.error(f"{where} must have shape [{want}]; the FMU declares [{have}]")
            count = var.size
    start = None
    if spec.start == "finite":
        start = _finite_starts(var)
        if start is None:
            report.error(f"{where} needs a finite literal start value for every element; "
                         f"the FMU declares {var.start!r}")
        elif count is not None and len(start) != count:
            report.error(f"{where} has {count} element(s) but {len(start)} start value(s); "
                         "an ambiguous initial vector is not accepted")
    elif var.start is not None:
        start = _finite_starts(var)
    if var.unit is not None and spec.unit is not None and var.unit != spec.unit:
        report.error(f"{where} must have unit {spec.unit!r}; the FMU declares {var.unit!r}")
    if len(report.errors) != before:
        return None
    return ResolvedField(spec.capability, spec.name, var.value_reference, count or 1, start,
                         var.unit is not None)


def _check_capability(report: Report, cap: Capability, meta: Metadata,
                      ) -> dict[str, ResolvedField | None] | None:
    present = {f.name for f in cap.fields if f.name in meta.variables}
    missing = [f.name for f in cap.required_fields if f.name not in present]
    if missing:
        # Parameters such as the geodetic origin are shared vocabulary; only an
        # exported input or output shows that the model attempted the capability.
        if any(f.name in present and "parameter" not in f.causality
               for f in cap.required_fields):
            report.error(f"{cap.name} is partially exported; missing required field(s) "
                         f"{', '.join(missing)}")
        return None
    resolved: dict[str, ResolvedField | None] = {}
    conforming = True
    for spec in cap.fields:
        var = meta.variables.get(spec.name)
        if var is None:
            resolved[spec.name] = None
            continue
        result = _check_field(report, spec, var)
        conforming &= result is not None
        resolved[spec.name] = result
    return resolved if conforming else None


def check(meta: Metadata, contract: Contract, required: list[str] | tuple[str, ...] = (),
          profile: str = STANDARD_PROFILE) -> Report:
    """Check metadata; the report lists every independent error, not only the first."""
    report = Report(meta.model_name or meta.model_identifier or "<unnamed FMU>", profile)
    if not meta.fmi_version.startswith(contract.fmi_version_prefix):
        report.error(f"FMI version {meta.fmi_version!r} is not FMI 3")
    if meta.cosimulation is None or not meta.model_identifier:
        report.error("no CoSimulation interface with a modelIdentifier")
    else:
        if meta.flag("needsExecutionTool"):
            report.error("needsExecutionTool=true is not supported (in-process loading only)")
        if not meta.flag("canHandleVariableCommunicationStepSize"):
            report.error("canHandleVariableCommunicationStepSize must be true; "
                         "hosts choose their communication points")
    if not meta.instantiation_token:
        report.error("no instantiationToken")
    for name in meta.duplicate_names:
        report.error(f"variable name {name!r} is declared more than once")

    marker = meta.variables.get(contract.marker.name)
    if profile == STANDARD_PROFILE:
        offered = list(contract.capabilities)
        if marker is None:
            report.error(f"no {contract.marker.name}; an FMU without the marker is accepted only "
                         f"with an explicitly selected legacy profile "
                         f"({', '.join(contract.profiles) or 'none defined'})")
        else:
            report.marker = marker
            m = contract.marker
            if marker.type != m.type:
                report.error(f"{m.name} must be {m.type}; the FMU declares {marker.type}")
            if marker.causality not in m.causality:
                report.error(f"{m.name} must have causality {' or '.join(m.causality)}; "
                             f"the FMU declares {marker.causality}")
            if marker.variability not in m.variability:
                report.error(f"{m.name} must have variability {' or '.join(m.variability)}; "
                             f"the FMU declares {marker.variability}")
            if marker.dimensions:
                report.error(f"{m.name} must be a scalar")
            value = marker.start[0] if marker.start and len(marker.start) == 1 else None
            if value is None:
                report.error(f"{m.name} has no literal value in the metadata")
            elif value != str(m.value):
                report.error(f"{m.name} declares version {value}; this checker supports "
                             f"version {m.value} only")
    else:
        prof = contract.profiles.get(profile)
        if prof is None:
            report.error(f"unknown profile {profile!r}")
            return report
        offered = list(prof.capabilities)
        if marker is not None:
            report.error(f"profile {profile!r} is for FMUs without {contract.marker.name}; "
                         "this FMU declares it, use the standard profile")
        else:
            report.warn(f"no {contract.marker.name}; accepted under the legacy profile "
                        f"{profile!r}: {prof.summary}")

    for name in offered:
        resolved = _check_capability(report, contract.capability(name), meta)
        if resolved is not None:
            report.provided[name] = resolved
    for name in required:
        if name not in contract.capabilities:
            report.error(f"the host requires unknown capability {name!r}")
        elif name not in offered:
            report.error(f"the host requires {name}, which profile {profile!r} cannot provide")
        elif name not in report.provided:
            missing = [f.name for f in contract.capability(name).required_fields
                       if f.name not in meta.variables]
            detail = f" (missing {', '.join(missing)})" if missing else ""
            report.error(f"the host requires {name}, which the FMU does not provide{detail}")
    return report
