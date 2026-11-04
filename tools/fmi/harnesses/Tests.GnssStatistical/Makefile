# FMI harness for one FIRE model; the build rules are shared.
# KNOWN FAILURE: stops at t = 0.001 s on its own "Independent RNG streams collapsed" assert
# (Rumoca 6058a2bb orders two when-clauses on one sample() wrongly; see checks.toml).
MODEL := Tests.GnssStatistical
include ../../harness.mk
