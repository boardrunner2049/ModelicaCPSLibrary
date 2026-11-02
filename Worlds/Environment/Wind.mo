within ModelicaCPSLibrary.Worlds.Environment;

model Wind
  parameter Real wind0[3] = {0, 0, 0} "Constant wind in world frame [m/s]";
  Modelica.Blocks.Interfaces.RealOutput wind[3];
equation
  wind = wind0;
end Wind;
