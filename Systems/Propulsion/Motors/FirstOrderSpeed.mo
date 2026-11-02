within ModelicaCPSLibrary.Systems.Propulsion.Motors;
model FirstOrderSpeed "Load-independent first-order rotor speed response"
  parameter Modelica.Units.SI.Time tau = 0.02;
  parameter Modelica.Units.SI.AngularVelocity omegaMax = 1000;
  parameter Modelica.Units.SI.AngularVelocity omega_start = 0;
  input Real demand(unit="1") "Normalized speed command";
  output Modelica.Units.SI.AngularVelocity omega(start = omega_start, fixed = true);
protected
  Modelica.Units.SI.AngularVelocity omegaCommand;
initial equation
  assert(tau > 0 and omegaMax > 0, "Speed response requires positive tau and omegaMax");
  assert(omega_start >= 0 and omega_start <= omegaMax,
    "Initial rotor speed must lie in [0,omegaMax]");
equation
  omegaCommand = omegaMax * noEvent(min(1, max(0, demand)));
  der(omega) = (omegaCommand - omega) / tau;
end FirstOrderSpeed;
