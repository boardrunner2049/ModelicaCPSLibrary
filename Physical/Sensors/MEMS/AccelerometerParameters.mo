within ModelicaCPSLibrary.Physical.Sensors.MEMS;
record AccelerometerParameters "Illustrative capacitive proof mass, NOT vendor calibration"
  parameter Real mass(unit="kg")=1e-8;
  parameter Real frequency(unit="Hz")=2000;
  parameter Real quality=0.7;
  parameter Real readC0(unit="F")=2e-13;
  parameter Real readLength(unit="m")=5e-6;
  parameter Real carrierVoltage(unit="V")=1;
  parameter Real feedbackCapacitance(unit="F")=1e-12;
  parameter Real afeTau(unit="s")=1e-5;
  parameter Real rail(unit="V")=3.3;
end AccelerometerParameters;
