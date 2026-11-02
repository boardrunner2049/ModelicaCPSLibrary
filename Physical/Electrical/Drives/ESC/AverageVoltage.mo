within ModelicaCPSLibrary.Physical.Electrical.Drives.ESC;
model AverageVoltage "Continuous average-voltage ESC with unipolar modulation command"
  input Real escCommand(unit="1") "Voltage modulation command; clamped to [0,1]";
  input Modelica.Units.SI.Voltage dcVoltage "Nonnegative DC supply voltage";
  output Real modulation(unit="1");
  output Modelica.Units.SI.Voltage equivalentVoltage "DC-equivalent drive voltage";
equation
  assert(dcVoltage >= 0, "AverageVoltage requires nonnegative dcVoltage");
  modulation = noEvent(min(1, max(0, escCommand)));
  equivalentVoltage = modulation * dcVoltage;
  annotation(Documentation(info="<html><p>Command mode is unipolar voltage modulation, not a speed or thrust target. Continuous algebraic voltage-only model: no switching, delay, current, losses or power accounting. No physical conserving electrical connector is advertised.</p></html>"));
end AverageVoltage;
