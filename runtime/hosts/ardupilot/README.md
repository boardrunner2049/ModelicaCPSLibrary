# ArduPilot SITL host (prototype)

The patches add the SITL model `cpvehicle` to ArduPilot (0001), the rover
and fixed-wing examples (0002), switch it to vehicle models linked into the
SITL binary (0003) and build every example vehicle from its TOML
configuration (0004): `--model cpvehicle:NAME` runs a CP Models vehicle
model compiled into this runtime (`prepare --shim NAME`, `cmake
-DCPV_MODELS=...`) in place of SITL's vehicle physics, with nothing loaded at
run time; the autopilot and SITL's sensor simulation are unchanged.

Pinned and tested: ArduPilot tag `Copter-4.6.2` (1ebd4d996e) and Rumoca
`e32964fc`, with ArduCopter and the CP Models quad (`configs/quad.toml`),
ArduRover and the rover (`configs/rover.toml`; also `configs/rover_i2.toml`
with an I2 IMU), ArduPlane and the fixed-wing (`configs/plane.toml`).

```sh
git clone --branch Copter-4.6.2 https://github.com/ArduPilot/ardupilot.git
cd ardupilot && git switch -c cpvehicle-sitl
git am /path/to/ModelicaCPSLibrary/runtime/hosts/ardupilot/000*.patch
```

Build, run and the measured results are in the patch's
`libraries/SITL/examples/CPVehicle/README.md`. This is host code; it belongs
to ArduPilot, not to the vehicle definitions, and is kept here only so the
prototype is reproducible from this repository.
