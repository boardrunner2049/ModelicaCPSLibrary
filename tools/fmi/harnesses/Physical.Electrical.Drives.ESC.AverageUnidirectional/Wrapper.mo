within ;
model FireHarness_AverageUnidirectional
  "AverageUnidirectional driving the inductive motor load its documentation requires"
  // The ESC's p/n pins cannot stay open, and its coast/blocking policy only
  // makes sense on an inductive load that reports back-EMF and projects its
  // winding current at the zero crossing. This rig wires it exactly as the
  // documentation prescribes: openCircuitVoltage = motor back-EMF and
  // coast -> motor.currentZero. The motor (R 0.2 ohm, L 1 mH, k 0.02,
  // J + JLoad = 2e-4, b = 1e-5) carries a quadratic blade-like drag
  // kQ*omega^2 so the rig has a loaded equilibrium and a coast-down. The
  // two energy integrals are bookkeeping for the harness checks only.
  input Real escCommand(start = 0) "ESC command (clamped to [0,1] by the ESC)";
  input Real dcVoltage(start = 16) "DC bus voltage (>= 0 asserted by the ESC)";
  parameter Real kQ = 1.5e-7 "Quadratic drag coefficient [N.m.s2]";
  ModelicaCPSLibrary.Physical.Electrical.Drives.ESC.AverageUnidirectional esc;
  ModelicaCPSLibrary.Systems.Propulsion.Motors.DynamicDCEquivalent motor(
    R = 0.2, L = 1e-3, k = 0.02, J = 1e-4, JLoad = 1e-4, b = 1e-5);
  output Real omega = motor.omega;
  output Real current = motor.current;
  output Real backEMF = motor.backEMF;
  output Real storedEnergy = motor.storedEnergy;
  Real inputEnergy(start = 0, fixed = true) "Integral of the ESC's DC-side power";
  Real lossEnergy(start = 0, fixed = true) "Integral of copper, friction and drag losses";
equation
  esc.escCommand = escCommand;
  esc.dcVoltage = dcVoltage;
  esc.openCircuitVoltage = motor.backEMF;
  motor.currentZero = esc.coast;
  connect(esc.p, motor.p);
  connect(esc.n, motor.n);
  motor.loadTorque = kQ * motor.omega ^ 2;
  der(inputEnergy) = esc.electricalPower;
  der(lossEnergy) = motor.copperLoss + motor.frictionLoss + motor.loadPower;
end FireHarness_AverageUnidirectional;
