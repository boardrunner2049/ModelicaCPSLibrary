// Harness wrapper for ModelicaCPSLibrary.Systems.Propulsion.HubReaction.
// HubReaction is purely algebraic but J has no default, so it cannot be
// exported alone ("parameter J has neither a binding nor an initial value").
// The wrapper supplies J = 1e-4 (motor rotor), JLoad = 2e-4 kg.m2 (propeller)
// and drives two instances with the same inputs, one per rotor spin sense
// (spinSign = +1 and -1), so the harness can check the sign convention of
// the axial spin-acceleration reaction.
model FireHarness_HubReaction
  input Real aerodynamicMoment_r[3](each unit = "N.m") "Aerodynamic hub moment in the rotor frame";
  input Real acceleration(unit = "rad/s2") "Rotor spin acceleration (magnitude, motor frame)";
  ModelicaCPSLibrary.Systems.Propulsion.HubReaction positive(J = 1e-4, JLoad = 2e-4, spinSign = 1);
  ModelicaCPSLibrary.Systems.Propulsion.HubReaction negative(J = 1e-4, JLoad = 2e-4, spinSign = -1);
  output Real momentPositive_r[3](each unit = "N.m") = positive.moment_r;
  output Real momentNegative_r[3](each unit = "N.m") = negative.moment_r;
equation
  positive.aerodynamicMoment_r = aerodynamicMoment_r;
  positive.acceleration = acceleration;
  negative.aerodynamicMoment_r = aerodynamicMoment_r;
  negative.acceleration = acceleration;
end FireHarness_HubReaction;
