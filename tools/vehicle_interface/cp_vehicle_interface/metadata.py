"""Read FMI 3 modelDescription.xml metadata from an .fmu archive or an extracted FMU."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import xml.etree.ElementTree as ET
import zipfile

TYPE_TAGS = frozenset({
    "Float32", "Float64", "Int8", "UInt8", "Int16", "UInt16", "Int32", "UInt32",
    "Int64", "UInt64", "Boolean", "String", "Binary", "Enumeration", "Clock",
})


class MetadataError(ValueError):
    """The model description is missing or malformed."""


@dataclass
class Variable:
    name: str
    type: str
    value_reference: int
    causality: str
    variability: str
    initial: str | None
    start: tuple[str, ...] | None
    dimensions: tuple[int | None, ...]
    unit: str | None
    description: str | None
    # Human-readable reason a dimension could not be resolved, if any.
    dimension_errors: tuple[str, ...] = ()

    @property
    def size(self) -> int | None:
        if any(d is None for d in self.dimensions):
            return None
        size = 1
        for d in self.dimensions:
            size *= d
        return size


@dataclass
class Metadata:
    fmi_version: str
    model_name: str | None
    instantiation_token: str | None
    generation_tool: str | None
    cosimulation: dict[str, str] | None
    variables: dict[str, Variable] = field(default_factory=dict)
    duplicate_names: tuple[str, ...] = ()

    @property
    def model_identifier(self) -> str | None:
        return self.cosimulation.get("modelIdentifier") if self.cosimulation else None

    def flag(self, name: str) -> bool:
        return bool(self.cosimulation) and self.cosimulation.get(name, "false") == "true"


def _local(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def read_model_description(path: Path | str) -> bytes:
    """Return modelDescription.xml from an .fmu archive or an extracted FMU directory."""
    path = Path(path)
    if path.is_dir():
        xml = path / "modelDescription.xml"
        if not xml.is_file():
            raise MetadataError(f"{path}: no modelDescription.xml")
        return xml.read_bytes()
    try:
        with zipfile.ZipFile(path) as archive:
            return archive.read("modelDescription.xml")
    except (zipfile.BadZipFile, KeyError, FileNotFoundError) as exc:
        raise MetadataError(f"{path}: cannot read modelDescription.xml ({exc})") from exc


def parse(xml: bytes) -> Metadata:
    """Parse the parts of an FMI 3 model description the vehicle interface relies on."""
    try:
        root = ET.fromstring(xml)
    except ET.ParseError as exc:
        raise MetadataError(f"malformed modelDescription.xml: {exc}") from exc
    if _local(root.tag) != "fmiModelDescription":
        raise MetadataError("root element is not fmiModelDescription")
    cosim = next((dict(e.attrib) for e in root if _local(e.tag) == "CoSimulation"), None)
    meta = Metadata(root.get("fmiVersion", ""), root.get("modelName"),
                    root.get("instantiationToken"), root.get("generationTool"), cosim)
    variables_node = next((e for e in root if _local(e.tag) == "ModelVariables"), None)
    if variables_node is None:
        raise MetadataError("modelDescription.xml has no ModelVariables")
    by_vr: dict[int, Variable] = {}
    dimension_refs: list[tuple[Variable, int, int]] = []
    duplicates = []
    for element in variables_node:
        kind = _local(element.tag)
        if kind not in TYPE_TAGS:
            continue
        name = element.get("name")
        raw_vr = element.get("valueReference")
        if name is None or raw_vr is None:
            raise MetadataError(f"a {kind} variable lacks a name or valueReference")
        try:
            vr = int(raw_vr)
        except ValueError:
            raise MetadataError(f"variable {name!r}: invalid valueReference {raw_vr!r}") from None
        dims: list[int | None] = []
        pending: list[tuple[int, int]] = []
        for index, child in enumerate(c for c in element if _local(c.tag) == "Dimension"):
            literal, ref = child.get("start"), child.get("valueReference")
            dims.append(int(literal) if literal is not None and literal.isdigit() else None)
            if literal is None and ref is not None and ref.isdigit():
                pending.append((index, int(ref)))
        start = element.get("start")
        var = Variable(
            name=name, type=kind, value_reference=vr,
            causality=element.get("causality", "local"),
            variability=element.get("variability",
                                    "continuous" if kind in ("Float32", "Float64") else "discrete"),
            initial=element.get("initial"),
            start=tuple(start.split()) if start is not None else None,
            dimensions=tuple(dims), unit=element.get("unit"),
            description=element.get("description"))
        dimension_refs.extend((var, index, ref) for index, ref in pending)
        if name in meta.variables:
            duplicates.append(name)
        meta.variables[name] = var
        by_vr.setdefault(vr, var)
    # Dimension valueReference: the size is the referenced structural parameter's
    # start. Only literal integer starts are resolvable before execution.
    for var, index, ref in dimension_refs:
        dims = list(var.dimensions)
        target = by_vr.get(ref)
        if target is None or target.start is None or len(target.start) != 1:
            var.dimension_errors += (f"dimension {index + 1} references valueReference {ref}, "
                                     "which has no literal start value",)
            continue
        try:
            dims[index] = int(target.start[0])
        except ValueError:
            var.dimension_errors += (f"dimension {index + 1}: non-integer size {target.start[0]!r}",)
            continue
        var.dimensions = tuple(dims)
    meta.duplicate_names = tuple(duplicates)
    return meta
