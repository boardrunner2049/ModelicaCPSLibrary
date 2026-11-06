within ModelicaCPSLibrary.Exports.FMU.VehicleInterface;
partial model Barometer "barometer_v1: latest barometer reading"
  output Real baro_pressure_pa(unit="Pa") "Static pressure [Pa]";
  output Real baro_temperature_c(unit="degC") "Sensor temperature [degC]";
  output Real baro_altitude_m(unit="m") "Up-positive altitude above the ground plane [m]";
  output Real baro_climb_rate_mps(unit="m/s") "Up-positive climb rate [m/s]";
end Barometer;
