within ModelicaCPSLibrary.Systems.Peripherals;
partial model PartialPowerModule "A battery pack (Power.Battery) feeding the vehicle, and the module measuring its voltage and current"
  parameter Real avionicsCurrent(unit="A") = 0.5 "Constant load besides propulsion";
  parameter Integer nCurrents(min=1) = 1 "Electrical propulsion currents";
  parameter Integer nShafts(min=1) = 1 "Shafts without an electrical model";
  parameter Real efficiency = 0.8 "Electrical to shaft power (motor and ESC), for shaft power";
  parameter Real propellerDiskArea(unit="m2") = 0.05 "Propeller disk, for a thrust-only propulsion model";
  parameter Real figureOfMerit = 0.7 "Ideal (momentum) power over shaft power, for a thrust-only propulsion model";
  Power.Battery battery;
  input Real propulsionCurrent[nCurrents](each unit="A") = zeros(nCurrents)
    "Electrical propulsion currents (magnitudes are summed)";
  input Real shaftTorque[nShafts](each unit="N.m") = zeros(nShafts) "Load torques of shafts without an electrical model";
  input Real shaftSpeed[nShafts](each unit="rad/s") = zeros(nShafts);
  input Real thrust(unit="N") = 0 "Thrust of a propulsion model with neither current nor shaft torque";
  input Real airspeed(unit="m/s") = 0 "Axial inflow of that propeller";
  output Real voltage(unit="V") "Measured";
  output Real current(unit="A") "Measured";
  output Real consumed(unit="A.h") "Diagnostic: truth";
  output Real remaining "Diagnostic: state of charge";
  output Boolean valid;
  output Real validity "1 if valid, else 0";
  output Real load(unit="A") "Diagnostic: total discharge current";
  output Real power(unit="W") "Diagnostic: electrical power of the shafts and the propeller";
protected
  Real ideal(unit="W") "Momentum-theory power of the thrust";
equation
  ideal = noEvent(if thrust > 0 then thrust*(airspeed/2 + sqrt(airspeed^2/4 + thrust/(2*1.225*propellerDiskArea))) else 0);
  power = (max(shaftTorque*shaftSpeed, 0) + ideal/figureOfMerit)/efficiency;
  load = avionicsCurrent + magnitudeSum(propulsionCurrent) + power/max(battery.voltage, 1);
  battery.current = load;
  consumed = battery.consumed;
  remaining = battery.soc;
  validity = if valid then 1 else 0;
end PartialPowerModule;
