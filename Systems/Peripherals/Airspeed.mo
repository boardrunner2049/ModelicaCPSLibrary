within ModelicaCPSLibrary.Systems.Peripherals;
model Airspeed "A0: pitot-static differential pressure 1/2 rho u|u| of the air velocity u along the device x axis, and the static air temperature at the mount (ISA troposphere); continuous"
  extends PartialPeripheral;
  parameter Real pressure0(unit="Pa") = 101325 "Ambient pressure at the origin [environment]";
  parameter Real temperature0(unit="K") = 288.15 "Ambient temperature at the origin [environment]";
  input Real wind_w[3](each unit="m/s") = zeros(3) "Air velocity over ground, NED";
  output Real differentialPressure(unit="Pa") "Total minus static pressure";
  output Real temperature(unit="K") "Static air temperature";
  output Boolean valid = true;
  output Real validity = 1 "1 if valid, else 0";
  output Real axialAirspeed(unit="m/s") "Diagnostic: true air velocity along the device x axis";
protected
  Real air_s[3](each unit="m/s") "Air-relative velocity of the mount, device axes";
  Real pressure(unit="Pa");
  Real density(unit="kg/m3");
equation
  air_s=transpose(R_bs)*(mountVelocity_b-transpose(truth.R_wb)*wind_w);
  (pressure,temperature)=Worlds.Environment.troposphere(-mountPosition_w[3],pressure0,temperature0);
  density=pressure/(287.05287*temperature);
  axialAirspeed=air_s[1];
  differentialPressure=0.5*density*air_s[1]*abs(air_s[1]);
end Airspeed;
