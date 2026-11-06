"""Load the normative vehicle interface contract (contract/vehicle_interface.toml)."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import tomllib

LIBRARY_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_CONTRACT = LIBRARY_ROOT / "contract" / "vehicle_interface.toml"
VARIABLE_LENGTH = "n"


@dataclass(frozen=True)
class Field:
    """One exported variable of a capability."""

    capability: str
    name: str
    type: str
    causality: tuple[str, ...]
    variability: tuple[str, ...] | None
    shape: tuple[int | str, ...]
    required: bool
    start: str | None
    unit: str | None
    frame: str | None
    meaning: str


@dataclass(frozen=True)
class Capability:
    name: str
    summary: str
    fields: tuple[Field, ...]

    @property
    def required_fields(self) -> tuple[Field, ...]:
        return tuple(f for f in self.fields if f.required)


@dataclass(frozen=True)
class Marker:
    name: str
    type: str
    causality: tuple[str, ...]
    variability: tuple[str, ...]
    value: int


@dataclass(frozen=True)
class Profile:
    name: str
    summary: str
    capabilities: tuple[str, ...]


@dataclass(frozen=True)
class Contract:
    name: str
    version: int
    descriptor_schema: int
    fmi_version_prefix: str
    reference_point: str
    marker: Marker
    capabilities: dict[str, Capability]
    profiles: dict[str, Profile]

    def capability(self, name: str) -> Capability:
        try:
            return self.capabilities[name]
        except KeyError:
            known = ", ".join(sorted(self.capabilities))
            raise ValueError(f"unknown capability {name!r}; known: {known}") from None


def _tuple(value) -> tuple[str, ...]:
    return tuple(value) if isinstance(value, list) else (value,)


def _shape(value: list, where: str) -> tuple[int | str, ...]:
    if len(value) > 1:
        raise ValueError(f"{where}: only scalars and one-dimensional shapes are defined")
    for item in value:
        if item != VARIABLE_LENGTH and not (isinstance(item, int) and item >= 1):
            raise ValueError(f"{where}: invalid shape entry {item!r}")
    return tuple(value)


def load_contract(path: Path | str = DEFAULT_CONTRACT) -> Contract:
    """Parse and sanity-check the contract table."""
    raw = tomllib.loads(Path(path).read_text())
    head = raw["contract"]
    m = raw["marker"]
    marker = Marker(m["name"], m["type"], _tuple(m["causality"]), _tuple(m["variability"]),
                    int(m["value"]))
    if marker.value != head["version"]:
        raise ValueError("contract: marker value must equal the contract version")
    capabilities: dict[str, Capability] = {}
    seen: dict[str, str] = {marker.name: "marker"}
    for cap_name, cap in raw["capabilities"].items():
        fields = []
        for entry in cap["fields"]:
            where = f"{cap_name}.{entry['name']}"
            if entry["name"] in seen:
                raise ValueError(f"{where}: also declared by {seen[entry['name']]}")
            seen[entry["name"]] = cap_name
            start = entry.get("start")
            if start not in (None, "finite"):
                raise ValueError(f"{where}: unknown start rule {start!r}")
            fields.append(Field(
                capability=cap_name, name=entry["name"], type=entry["type"],
                causality=_tuple(entry["causality"]),
                variability=_tuple(entry["variability"]) if "variability" in entry else None,
                shape=_shape(entry["shape"], where), required=bool(entry["required"]),
                start=start, unit=entry.get("unit"), frame=entry.get("frame"),
                meaning=entry.get("meaning", "")))
        if not any(f.required for f in fields):
            raise ValueError(f"{cap_name}: a capability needs at least one required field")
        capabilities[cap_name] = Capability(cap_name, cap.get("summary", ""), tuple(fields))
    profiles = {}
    for prof_name, prof in raw.get("profiles", {}).items():
        unknown = [c for c in prof["capabilities"] if c not in capabilities]
        if unknown:
            raise ValueError(f"profile {prof_name}: unknown capabilities {unknown}")
        profiles[prof_name] = Profile(prof_name, prof.get("summary", ""),
                                      tuple(prof["capabilities"]))
    return Contract(head["name"], int(head["version"]), int(head["descriptor_schema"]),
                    head["fmi_version"], head["reference_point"], marker, capabilities, profiles)
