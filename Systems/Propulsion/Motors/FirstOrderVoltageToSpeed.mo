within ModelicaCPSLibrary.Systems.Propulsion.Motors;
model FirstOrderVoltageToSpeed "Load-independent voltage-to-speed surrogate"
  parameter Real Kv(unit="rad/(s.V)") = 1000/16 "SI speed gain, not rpm/V";
  parameter Modelica.Units.SI.Time tauOmega = 0.03;
  parameter Modelica.Units.SI.AngularVelocity omega_start = 0;
  input Modelica.Units.SI.Voltage equivalentVoltage;
  output Modelica.Units.SI.AngularVelocity omega(start=omega_start, fixed=true);
initial equation
  assert(Kv > 0 and tauOmega > 0, "FirstOrderVoltageToSpeed requires positive Kv and tauOmega");
  assert(omega_start >= 0, "FirstOrderVoltageToSpeed requires nonnegative initial speed");
equation
  assert(equivalentVoltage >= 0, "FirstOrderVoltageToSpeed requires nonnegative equivalentVoltage");
  der(omega) = (Kv*equivalentVoltage - omega)/tauOmega;
  annotation(Documentation(info="<html><p>Single continuous speed state with no additional speed cap. Zero voltage gives exponential coast-down; reverse rotation, current, electromagnetic torque, load feedback, braking and power accounting are outside this model.</p></html>"));
end FirstOrderVoltageToSpeed;
