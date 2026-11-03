"""Read the variables of an FMI 3 FMU, with array extents resolved."""
import math, re, zipfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

TYPES = {"Float64", "Float32", "Int8", "UInt8", "Int16", "UInt16", "Int32", "UInt32", "Int64",
         "UInt64", "Boolean", "String", "Enumeration"}


@dataclass
class Variable:
    name: str
    type: str
    causality: str
    variability: str
    start: list[str]
    dims: list[int]
    description: str = ""
    unit: str = ""

    @property
    def size(self) -> int:
        return math.prod(self.dims) if self.dims else 1

    def elements(self) -> list[tuple[str, str]]:
        """(endpoint suffix, start text) per scalar element, row-major, 1-based."""
        if not self.dims:
            return [(self.name, self.start[0] if self.start else "")]
        starts = self.start if len(self.start) == self.size else [self.start[0] if self.start else ""] * self.size
        return [(f"{self.name}[{k + 1}]", starts[k]) for k in range(self.size)]


def read(fmu: Path) -> list[Variable]:
    with zipfile.ZipFile(fmu) as archive:
        root = ET.fromstring(archive.read("modelDescription.xml"))
    nodes = [n for n in root.find("ModelVariables") if n.tag in TYPES]
    by_vr = {n.get("valueReference"): n for n in nodes}
    variables = []
    for n in nodes:
        dims = []
        for d in n.findall("Dimension"):
            if d.get("start") is not None:
                dims.append(int(d.get("start")))
            else:
                dims.append(int(by_vr[d.get("valueReference")].get("start")))
        variables.append(Variable(n.get("name"), n.tag, n.get("causality", "local"),
                                  n.get("variability", "continuous"), (n.get("start") or "").split(),
                                  dims, n.get("description", ""), n.get("unit", "")))
    return variables


C_KEYWORDS = set("""auto break case char const continue default do double else enum extern float for
goto if inline int long register restrict return short signed sizeof static struct switch typedef union
unsigned void volatile while bool true false main""".split())


def alias(endpoint: str, taken: set[str]) -> str:
    """A C identifier for a world binding alias, unique within `taken`."""
    base = re.sub(r"[^A-Za-z0-9]+", "_", endpoint).strip("_").lower() or "v"
    if base[0].isdigit() or base in C_KEYWORDS or base.startswith("wm_"):
        base = "v_" + base
    name, k = base, 2
    while name in taken:
        name, k = f"{base}_{k}", k + 1
    taken.add(name)
    return name
