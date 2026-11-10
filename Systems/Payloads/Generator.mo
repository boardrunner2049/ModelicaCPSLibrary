within ModelicaCPSLibrary.Systems.Payloads;
model Generator "An onboard generator (engine or fuel cell) delivering the share of the vehicle's electrical demand the power module draws from it, up to its rating, burning fuel in proportion; the fuel's weight loads the vehicle"
  extends PartialPayload;
  parameter Real ratedPower(unit="W") = 2000 "[datasheet]";
  parameter Real fuelRate(unit="l/h") = 1.5 "At rated power [datasheet]";
  parameter Real tankVolume(unit="l") = 3 "[datasheet]";
  parameter Real fuel_start(unit="l") = 3 "[configuration]";
  parameter Real fuelDensity(unit="kg/l") = 0.74 "[physics]";
  input Real utilization = 0 "Fraction of its capacity the vehicle draws (the power module's)";
  output Real capacity(unit="W") "Power it can deliver";
  output Real supply(unit="W") "Power delivered";
  output Real fuelFlow(unit="l/h");
  output Real fuelLevel "Fraction of the tank";
  output Real fuel(unit="l");
protected
  Real tank(unit="l", start=fuel_start, fixed=true);
equation
  capacity = if tank > 0 then ratedPower else 0;
  supply = capacity*utilization;
  fuelFlow = fuelRate*supply/ratedPower;
  der(tank) = -fuelFlow/3600;
  fuel = max(tank, 0);
  fuelLevel = fuel/tankVolume;
  force_b = transpose(truth.R_wb)*{0, 0, fuelDensity*fuel*gravity};
  moment_b = cross(position_b, force_b);
end Generator;
