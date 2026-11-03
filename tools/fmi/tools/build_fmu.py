#!/usr/bin/env python3
"""Turn a Rumoca source FMU into a binary FMI 3 FMU for x86_64-linux.

usage: build_fmu.py SOURCE.fmu OUT.fmu

Compiles the sources named in sources/buildDescription.xml against the FMI 3
headers in lib/fmi3-headers, and adds binaries/x86_64-linux/<modelIdentifier>.so.
"""
import os, re, shutil, subprocess, sys, tempfile, zipfile
from pathlib import Path

HEADERS = Path(os.environ.get("FMI3_HEADERS") or Path(__file__).resolve().parent.parent / "lib" / "fmi3-headers")
CC = os.environ.get("CC", "cc")


def build(source_fmu: Path, out_fmu: Path) -> None:
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        with zipfile.ZipFile(source_fmu) as archive:
            archive.extractall(root)
        build_xml = (root / "sources" / "buildDescription.xml").read_text()
        identifier = re.search(r'modelIdentifier="([^"]+)"', build_xml).group(1)
        sources = [root / "sources" / name for name in re.findall(r'<SourceFile name="([^"]+)"', build_xml)]
        defines = [f"-D{n}={v}" for n, v in re.findall(r'<PreprocessorDefinition name="([^"]+)" value="([^"]*)"', build_xml)]
        binary = root / "binaries" / "x86_64-linux" / f"{identifier}.so"
        binary.parent.mkdir(parents=True)
        cmd = [CC, "-std=c99", "-O2", "-fPIC", "-shared", *defines, f"-I{HEADERS}",
               f"-I{root / 'sources'}", "-o", str(binary), *map(str, sources), "-lm"]
        subprocess.run(cmd, check=True)
        out_fmu.parent.mkdir(parents=True, exist_ok=True)
        partial = out_fmu.with_suffix(".fmu.partial")
        with zipfile.ZipFile(partial, "w", zipfile.ZIP_DEFLATED) as archive:
            for path in sorted(root.rglob("*")):
                if path.is_file():
                    archive.write(path, path.relative_to(root).as_posix())
        partial.replace(out_fmu)


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    build(Path(sys.argv[1]), Path(sys.argv[2]))
