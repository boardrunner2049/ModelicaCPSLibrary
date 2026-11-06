# ArduPilot SITL host (prototype)

The patches add the SITL model `cpvehicle` to ArduPilot (0001) and the rover
and fixed-wing examples (0002): a CP Models vehicle FMU loaded in-process
through this runtime replaces SITL's vehicle physics; the autopilot and SITL's
sensor simulation are unchanged.

Pinned and tested: ArduPilot tag `Copter-4.6.2` (1ebd4d996e) and Rumoca
`e32964fc`, with ArduCopter and the CP Models quad (`configs/quad.toml`),
ArduRover and `Exports.FMU.Rover`, ArduPlane and `Exports.FMU.FixedWing`.

```sh
git clone --branch Copter-4.6.2 https://github.com/ArduPilot/ardupilot.git
cd ardupilot && git switch -c cpvehicle-sitl
git am /path/to/ModelicaCPSLibrary/runtime/hosts/ardupilot/000*.patch
```

Build, run and the measured results are in the patch's
`libraries/SITL/examples/CPVehicle/README.md`. This is host code; it belongs
to ArduPilot, not to the vehicle definitions, and is kept here only so the
prototype is reproducible from this repository.
