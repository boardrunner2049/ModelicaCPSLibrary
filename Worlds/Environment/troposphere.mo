within ModelicaCPSLibrary.Worlds.Environment;
function troposphere "Pressure and temperature at a height above a reference point, ISA troposphere lapse rate"
  input Real height(unit="m") "Above the reference point";
  input Real pressure0(unit="Pa") "At the reference point";
  input Real temperature0(unit="K") "At the reference point";
  output Real pressure(unit="Pa");
  output Real temperature(unit="K");
protected
  constant Real lapseRate(unit="K/m") = 0.0065;
  constant Real exponent = 5.255877 "g0*M/(R*L) of the ISA";
algorithm
  temperature := temperature0 - lapseRate*height;
  pressure := pressure0*(temperature/temperature0)^exponent;
end troposphere;
