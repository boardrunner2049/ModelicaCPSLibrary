# FMI harness for one FIRE model; the build rules are shared.
# KNOWN FAILURE: FMU initialization fails on the parallel ray (division by n.d = 0 under a
# frozen relation with Rumoca 6058a2bb; see checks.toml).
MODEL := Tests.RangePhysics
include ../../harness.mk
