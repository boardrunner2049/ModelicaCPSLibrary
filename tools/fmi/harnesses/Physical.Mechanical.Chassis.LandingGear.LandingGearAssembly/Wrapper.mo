// Harness wrapper for Physical.Mechanical.Chassis.LandingGear.LandingGearAssembly.
//
// The assembly is causal and exports on its own, but every leg asserts (in its
// equation section) that terrainNormal_w is a unit vector, and an FMU input's
// start value is all the model sees during initialization (the harness
// applies its signals only after fmi3ExitInitializationMode). The plain
// export starts every input at 0, so initialization aborts. This wrapper
// changes nothing but the input start values: a level NED ground plane
// (outward normal Up = {0,0,-1}), identity attitude, CG 0.3 m above ground
// (the scenario's first phase). The leg layout is overridden in world.toml.
model FireHarness_LandingGearAssembly
  extends ModelicaCPSLibrary.Physical.Mechanical.Chassis.LandingGear.LandingGearAssembly(
    terrainNormal_w(start = {0, 0, -1}),
    R_wb(start = [1, 0, 0; 0, 1, 0; 0, 0, 1]),
    p_w(start = {0, 0, -0.3}));
end FireHarness_LandingGearAssembly;
