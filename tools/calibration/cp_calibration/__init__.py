"""Calibration and validation of COMPOSE sensor models (docs/sensor-calibration.md).

The harness exercises the library's own sensor models as Rumoca-exported FMI 3
Co-Simulation FMUs through FMPy, fits their parameters to traces with SciPy,
and writes calibration profiles that `fire-compose` overlays on a part
(`[models.KIND] calibration = "..."`) without modifying the part file.
"""
__version__ = "0.1.0"
