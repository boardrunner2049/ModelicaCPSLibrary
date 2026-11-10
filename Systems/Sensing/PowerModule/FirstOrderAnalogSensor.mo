within ModelicaCPSLibrary.Systems.Sensing.PowerModule;
model FirstOrderAnalogSensor "A2: A1 with a first-order output response per channel (the module's filtering or amplifier bandwidth)"
  extends PartialAnalogSensor;
  parameter Real voltageGain(unit="V/V") = 1/18.182 "[datasheet]";
  parameter Real voltageOffset(unit="V") = 0 "[datasheet]";
  parameter Real currentGain(unit="V/A") = 1/36.364 "[datasheet]";
  parameter Real currentOffset(unit="V") = 0 "[datasheet]";
  parameter Real voltageTimeConstant(unit="s") = 1e-3 "Output time constant, 1/(2 pi bandwidth) [datasheet]";
  parameter Real currentTimeConstant(unit="s") = 1e-3 "[datasheet]";
  Real voltageFiltered(unit="V", start=0, fixed=false) "Before clipping";
  Real currentFiltered(unit="V", start=0, fixed=false);
initial equation
  assert(voltageTimeConstant > 0 and currentTimeConstant > 0, "Time constants must be positive (A1 is the static level)");
  voltageFiltered = voltageGain*voltage + voltageOffset;
  currentFiltered = currentGain*current + currentOffset;
equation
  voltageTimeConstant*der(voltageFiltered) = voltageGain*voltage + voltageOffset - voltageFiltered;
  currentTimeConstant*der(currentFiltered) = currentGain*current + currentOffset - currentFiltered;
  voltageSense = min(outputMax, max(outputMin, voltageFiltered));
  currentSense = min(outputMax, max(outputMin, currentFiltered));
  annotation(Documentation(info="<html><p>Starts in steady state with the initial inputs.</p></html>"));
end FirstOrderAnalogSensor;
