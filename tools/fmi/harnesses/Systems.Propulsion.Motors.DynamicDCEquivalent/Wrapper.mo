within ;
model DynamicDCEquivalentRig "DynamicDCEquivalent fed by a commanded terminal voltage"
  // The motor's pins cannot stay open: an unconnected top-level pin pins its
  // flow to zero, which leaves the winding current no freedom and makes the
  // currentZero reinit ill-posed (Rumoca ES017; OpenModelica refuses it too).
  input Real voltage(start = 0) "Terminal voltage command";
  input Boolean currentZero(start = false) "Drive-reported current zero crossing";
  input Real loadTorque(start = 0) "Torque opposing positive shaft rotation";
  ModelicaCPSLibrary.Systems.Propulsion.Motors.DynamicDCEquivalent motor(
    R = 0.5, L = 2e-3, k = 0.02, J = 2e-5, JLoad = 1e-5, b = 1e-6);
  Modelica.Electrical.Analog.Sources.SignalVoltage source;
  Modelica.Electrical.Analog.Basic.Ground ground;
  output Real current = motor.current;
  output Real omega = motor.omega;
  output Real backEMF = motor.backEMF;
  output Real electromagneticTorque = motor.electromagneticTorque;
  output Real copperLoss = motor.copperLoss;
  output Real storedEnergy = motor.storedEnergy;
equation
  source.v = voltage;
  connect(source.p, motor.p);
  connect(source.n, motor.n);
  connect(source.n, ground.p);
  motor.currentZero = currentZero;
  motor.loadTorque = loadTorque;
end DynamicDCEquivalentRig;
