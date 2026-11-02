within ModelicaCPSLibrary.Worlds.Environment;

model MagneticField
  parameter Real magneticField0[3] = {0.0, 21.0e-6, -43.0e-6}
    "Nominal world-frame magnetic field [T]";
  Modelica.Blocks.Interfaces.RealOutput magneticField[3];
equation
  magneticField = magneticField0;
end MagneticField;
