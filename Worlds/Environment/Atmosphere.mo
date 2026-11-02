within ModelicaCPSLibrary.Worlds.Environment;

model Atmosphere
  parameter Real temperature0 = 288.15 "Temperature [K]";
  parameter Real pressure0 = 101325.0 "Pressure [Pa]";
  parameter Real density0 = 1.225 "Density [kg/m3]";
  Modelica.Blocks.Interfaces.RealOutput temperature;
  Modelica.Blocks.Interfaces.RealOutput pressure;
  Modelica.Blocks.Interfaces.RealOutput density;
equation
  temperature = temperature0;
  pressure = pressure0;
  density = density0;
end Atmosphere;
