// Harness wrapper for Physical.Mechanical.Chassis.ChassisAssembly.
//
// The chassis is a parameter-only mass budget (no states, no inputs). Exported
// alone it uses its aggregate default (1 kg, diagonal inertia), which
// exercises nothing. Its configuration is structural (part counts, record
// arrays), so this wrapper instantiates it twice with fixed part lists and
// adds no equations:
//   assembled      useAssembledMass = true with an asymmetric parts list in
//                  chassis axes C (FRD), chosen so that every product of
//                  inertia of the result is nonzero:
//                    core      1.2 kg at {0.01,-0.005,0.02}, diag(0.012,0.014,0.022)
//                    arm 1     0.08 kg at {0.12,0.09,0}, rod axes yawed by
//                              atan2(0.6,0.8) (R_Cj), diag(2e-5,6e-4,6e-4)
//                    arm 2     0.08 kg at {-0.10,0.10,0.01}, yawed 135 deg
//                    payload   0.35 kg at {0.15,-0.03,0.09}, full tensor
//                    battery   0.45 kg at {-0.03,0.02,-0.04}, diag(1.5e-4,9e-4,1e-3)
//                    motor     0.06 kg point mass at {0.2,0.2,-0.03}
//   aggregateOnly  useAssembledMass = false with a 2.5 kg aggregate budget
//                  and a (deliberately different) constituent list, which
//                  aggregate mode must ignore.
model FireHarness_ChassisAssembly
  import ModelicaCPSLibrary.Physical.Mechanical.MassProperties;
  import ModelicaCPSLibrary.Physical.Mechanical.Chassis.ChassisAssembly;
  import ModelicaCPSLibrary.Physical.Mechanical.Chassis.Arms.RigidArm;
  import ModelicaCPSLibrary.Physical.Mechanical.Chassis.Payloads.FixedPayload;
  constant Real s = 0.7071067811865476 "cos(45 deg)";
  ChassisAssembly assembled(
    useAssembledMass = true,
    core(mass = 1.2, r_C = {0.01, -0.005, 0.02}, inertia = diagonal({0.012, 0.014, 0.022})),
    nArms = 2,
    arms = {
      RigidArm(mass = 0.08, r_C = {0.12, 0.09, 0},
        R_Cj = [0.8, -0.6, 0; 0.6, 0.8, 0; 0, 0, 1], inertia = diagonal({2e-5, 6e-4, 6e-4})),
      RigidArm(mass = 0.08, r_C = {-0.10, 0.10, 0.01},
        R_Cj = [-s, -s, 0; s, -s, 0; 0, 0, 1], inertia = diagonal({2e-5, 6e-4, 6e-4}))},
    nPayloads = 1,
    payloads = {FixedPayload(mass = 0.35, r_C = {0.15, -0.03, 0.09},
      inertia = [4e-4, 5e-5, -2e-5; 5e-5, 6e-4, 3e-5; -2e-5, 3e-5, 5e-4])},
    nAdditionalParts = 2,
    additionalParts = {
      MassProperties(mass = 0.45, r_C = {-0.03, 0.02, -0.04}, inertia = diagonal({1.5e-4, 9e-4, 1e-3})),
      MassProperties(mass = 0.06, r_C = {0.2, 0.2, -0.03})});
  ChassisAssembly aggregateOnly(
    useAssembledMass = false,
    aggregate(mass = 2.5, r_C = {0.02, -0.01, 0.03}, inertia = [0.02, 0.001, 0; 0.001, 0.03, 0; 0, 0, 0.045]),
    core(mass = 9),
    nPayloads = 1,
    payloads = {FixedPayload(mass = 7, r_C = {1, 1, 1})});
end FireHarness_ChassisAssembly;
