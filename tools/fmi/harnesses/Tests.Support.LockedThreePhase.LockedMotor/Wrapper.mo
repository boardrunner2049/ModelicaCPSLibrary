// Harness wrapper for ModelicaCPSLibrary.Tests.Support.LockedThreePhase.LockedMotor.
// LockedMotor is the ThreePhasePM motor with its shaft bolted to a fixed
// flange (the locked-rotor benchmark motor of Tests.ThreePhaseAveraging).
// Its phase pins are open acausal connectors and its parameters only get
// values from the bench, so it cannot be exported alone. This wrapper:
//   - gives it the bench's values RPhase = 0.2 Ohm, LPhase = 1 mH,
//     ke = 0.01 V s/rad, 7 pole pairs, J = JLoad = 1e-4 kg m^2, default
//     trapezoidal back-EMF, but theta0 = 0.3 rad instead of the bench's 0 so
//     that the locked-rotor shape {trapezoid(0.3), -1, +1} = {0.5730, -1, 1}
//     weights every phase differently (with theta0 = 0 phase a carries no
//     torque and a/b/c transpositions could hide);
//   - drives each phase pin from an MSL signal voltage source to a common
//     ground (inputs va, vb, vc); the motor's star point stays floating;
//   - passes loadTorque through;
//   - exposes the phase currents, the electromagnetic torque, the lock's
//     reaction torque, the star-point potential and an integrated energy
//     balance (energyError).
model FireHarness_LockedMotor
  input Real va(unit = "V");
  input Real vb(unit = "V");
  input Real vc(unit = "V");
  input Real loadTorque(unit = "N.m");
  ModelicaCPSLibrary.Tests.Support.LockedThreePhase.LockedMotor motor(
    RPhase = 0.2, LPhase = 0.001, ke = 0.01, polePairs = 7, J = 0.0001, JLoad = 0.0001,
    theta0 = 0.3);
  Modelica.Electrical.Analog.Sources.SignalVoltage source[3];
  Modelica.Electrical.Analog.Basic.Ground ground;
  Modelica.Units.SI.Energy terminalEnergy(start = 0, fixed = true) "Integral of sum(v_j*i_j) into the motor";
  Modelica.Units.SI.Energy copperEnergy(start = 0, fixed = true) "Integral of the reported copper loss";
  output Modelica.Units.SI.Current ia = motor.phaseCurrent[1];
  output Modelica.Units.SI.Current ib = motor.phaseCurrent[2];
  output Modelica.Units.SI.Current ic = motor.phaseCurrent[3];
  output Modelica.Units.SI.Torque torque = motor.electromagneticTorque;
  output Modelica.Units.SI.Torque lockTorque = motor.lock.flange.tau "Torque the lock takes from the shaft";
  output Modelica.Units.SI.Voltage starVoltage = motor.emf.neutral.v "Floating star-point potential";
  output Modelica.Units.SI.Energy energyError "Terminal energy minus copper loss energy minus stored energy";
equation
  source[1].v = va;
  source[2].v = vb;
  source[3].v = vc;
  for j in 1:3 loop
    connect(source[j].p, motor.phase[j]);
    connect(source[j].n, ground.p);
  end for;
  motor.loadTorque = loadTorque;
  der(terminalEnergy) = sum(motor.phase.v .* motor.phase.i);
  der(copperEnergy) = motor.copperLoss;
  energyError = terminalEnergy - copperEnergy - motor.storedEnergy;
end FireHarness_LockedMotor;
