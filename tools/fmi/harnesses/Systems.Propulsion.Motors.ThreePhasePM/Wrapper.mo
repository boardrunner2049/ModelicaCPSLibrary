// Harness wrapper for ModelicaCPSLibrary.Systems.Propulsion.Motors.ThreePhasePM.
// The motor's three phase pins are open acausal connectors and its
// parameters have no values, so it cannot be exported alone. This wrapper:
//   - gives it RPhase = 0.2 Ohm, LPhase = 0.1 mH, ke = 0.01 V s/rad,
//     7 pole pairs, sinusoidal back-EMF (trapezoidal = false, so the steady
//     state has a closed form), J = 8e-6 and JLoad = 2e-6 kg m^2 (an
//     asymmetric split: only J + JLoad = 1e-5 is right), b = 1e-5 N m s/rad,
//     and non-default start values: theta0 = 0.3 rad, omega_start = 50 rad/s,
//     i_start = {1, -0.4, -0.6} A (floating star: phase 3 follows from KCL);
//   - drives the phases with an ideal sine-commutated voltage source: three
//     MSL signal voltage sources from a common ground, v_j =
//     voltageAmplitude*sin(theta_e - (j-1)*2*pi/3), aligned with the back-EMF
//     (q axis) using the motor's own electricalAngle output; the star point of
//     the motor stays floating, as in the library;
//   - adds a common-mode voltage (input commonMode) to all three sources: a
//     floating star must reject it (phase currents unchanged, the star point
//     follows it), which a grounded or mis-wired neutral would not;
//   - passes the loadTorque input straight through;
//   - integrates the terminal power and the reported losses so the harness
//     can check the energy balance (energyError), and reports the floating-star
//     KCL sum of the phase currents (starCurrentSum).
model FireHarness_ThreePhasePM
  input Real voltageAmplitude(unit = "V") "Peak phase voltage, q-axis aligned";
  input Real loadTorque(unit = "N.m") "Load torque opposing positive rotation";
  input Real commonMode(unit = "V") "Voltage added to all three phases";
  ModelicaCPSLibrary.Systems.Propulsion.Motors.ThreePhasePM motor(
    RPhase = 0.2, LPhase = 1e-4, ke = 0.01, polePairs = 7, trapezoidal = false,
    J = 8e-6, JLoad = 2e-6, b = 1e-5, theta0 = 0.3, omega_start = 50,
    i_start = {1.0, -0.4, -0.6});
  Modelica.Electrical.Analog.Sources.SignalVoltage source[3];
  Modelica.Electrical.Analog.Basic.Ground ground;
  Modelica.Units.SI.Energy terminalEnergy(start = 0, fixed = true) "Integral of sum(v_j*i_j) into the motor";
  Modelica.Units.SI.Energy dissipatedEnergy(start = 0, fixed = true) "Integral of copper, friction and load power";
  Modelica.Units.SI.Energy initialStoredEnergy "Stored (magnetic + kinetic) energy at t = 0, held";
  output Modelica.Units.SI.AngularVelocity omega = motor.omega;
  output Modelica.Units.SI.Torque torque = motor.electromagneticTorque;
  output Modelica.Units.SI.Power copperLoss = motor.copperLoss;
  output Modelica.Units.SI.Current starCurrentSum = sum(motor.phaseCurrent);
  output Modelica.Units.SI.Voltage starVoltage = motor.emf.neutral.v "Floating star-point potential";
  output Modelica.Units.SI.Energy storedEnergy = motor.storedEnergy;
  output Modelica.Units.SI.Energy energyError "Terminal energy minus dissipated energy minus the change of stored energy";
initial equation
  initialStoredEnergy = motor.storedEnergy;
equation
  der(initialStoredEnergy) = 0;
  for j in 1:3 loop
    source[j].v = voltageAmplitude*sin(motor.electricalAngle - (j - 1)*2*Modelica.Constants.pi/3) + commonMode;
    connect(source[j].p, motor.phase[j]);
    connect(source[j].n, ground.p);
  end for;
  motor.loadTorque = loadTorque;
  der(terminalEnergy) = sum(motor.phase.v .* motor.phase.i);
  der(dissipatedEnergy) = motor.copperLoss + motor.frictionLoss + motor.loadPower;
  energyError = terminalEnergy - dissipatedEnergy - (motor.storedEnergy - initialStoredEnergy);
end FireHarness_ThreePhasePM;
