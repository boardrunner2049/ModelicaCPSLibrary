within ModelicaCPSLibrary.Systems.Peripherals;
partial model PartialPowerModule "A battery pack (Power.Battery) feeding the vehicle, and the module measuring its voltage and current"
  parameter Real avionicsCurrent(unit="A") = 0.5 "Constant load besides propulsion [assumed]";
  parameter Integer nCurrents(min=1) = 1 "Electrical propulsion currents [configuration]";
  parameter Integer nShafts(min=1) = 1 "Shafts without an electrical model [configuration]";
  parameter Real efficiency = 0.8 "Electrical to shaft power (motor and ESC), for shaft power [assumed]";
  parameter Real propellerDiskArea(unit="m2") = 0.05 "Propeller disk, for a thrust-only propulsion model [assumed]";
  parameter Real figureOfMerit = 0.7 "Ideal (momentum) power over shaft power, for a thrust-only propulsion model [assumed]";
  Power.Battery battery;
  input Real propulsionCurrent[nCurrents](each unit="A") = zeros(nCurrents)
    "Electrical propulsion currents (magnitudes are summed)";
  input Real shaftTorque[nShafts](each unit="N.m") = zeros(nShafts) "Load torques of shafts without an electrical model";
  input Real shaftSpeed[nShafts](each unit="rad/s") = zeros(nShafts);
  input Real thrust(unit="N") = 0 "Thrust of a propulsion model with neither current nor shaft torque";
  input Real airspeed(unit="m/s") = 0 "Axial inflow of that propeller";
  parameter Integer nSupplies(min=1) = 1 "Power sources besides the pack [configuration]";
  input Real supplyPower[nSupplies](each unit="W") = zeros(nSupplies) "What a generator or a power tether can deliver";
  output Real voltage(unit="V") "Measured";
  output Real current(unit="A") "Measured";
  output Real consumed(unit="A.h") "Diagnostic: truth";
  output Real remaining "Diagnostic: state of charge";
  output Boolean valid;
  output Real validity "1 if valid, else 0";
  output Real load(unit="A") "Diagnostic: total discharge current";
  output Real power(unit="W") "Diagnostic: electrical power of the shafts and the propeller";
  output Real demand(unit="W") "The vehicle's electrical demand, for a generator or tether to follow";
  output Real packCurrent(unit="A") "Out of the pack: the load less what a generator or tether supplies";
  output Real utilization "Fraction of the sources' capacity drawn: the demand, up to all of it";
protected
  Real ideal(unit="W") "Momentum-theory power of the thrust";
equation
  ideal = noEvent(if thrust > 0 then thrust*(airspeed/2 + sqrt(airspeed^2/4 + thrust/(2*1.225*propellerDiskArea))) else 0);
  power = (max(shaftTorque*shaftSpeed, 0) + ideal/figureOfMerit)/efficiency;
  load = avionicsCurrent + magnitudeSum(propulsionCurrent) + power/max(battery.voltage, 1);
  demand = load*battery.voltage;
  utilization = min(1, demand/max(magnitudeSum(supplyPower), 1e-6));
  packCurrent = load - utilization*magnitudeSum(supplyPower)/max(battery.voltage, 1);
  battery.current = packCurrent;
  consumed = battery.consumed;
  remaining = battery.soc;
  validity = if valid then 1 else 0;
end PartialPowerModule;
