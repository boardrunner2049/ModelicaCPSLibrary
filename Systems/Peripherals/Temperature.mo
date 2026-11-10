within ModelicaCPSLibrary.Systems.Peripherals;
model Temperature "T0: air temperature at the mount (ISA troposphere from the origin), plus a fixed self-heating offset; continuous"
  extends PartialPeripheral;
  parameter Real temperature0(unit="K") = 288.15 "Ambient temperature at the origin";
  parameter Real selfHeating(unit="K") = 0;
  output Real temperature(unit="K");
  output Boolean valid = true;
  output Real validity = 1 "1 if valid, else 0";
protected
  Real pressure(unit="Pa");
equation
  (pressure, temperature) = Worlds.Environment.troposphere(-mountPosition_w[3], 101325,
    temperature0 + selfHeating);
end Temperature;
