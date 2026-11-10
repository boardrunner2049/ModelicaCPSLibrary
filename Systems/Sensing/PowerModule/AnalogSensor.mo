within ModelicaCPSLibrary.Systems.Sensing.PowerModule;
model AnalogSensor "A1: static transfer functions, gain and offset per channel, clipped; optional current-gain temperature coefficient"
  extends PartialAnalogSensor;
  parameter Real voltageGain(unit="V/V") = 1/18.182 "Voltage-sense volts per battery volt (1/divider) [datasheet]";
  parameter Real voltageOffset(unit="V") = 0 "Voltage-sense output at zero battery voltage [datasheet]";
  parameter Real currentGain(unit="V/A") = 1/36.364 "Current-sense volts per ampere [datasheet]";
  parameter Real currentOffset(unit="V") = 0 "Current-sense output at zero current [datasheet]";
  parameter Real currentGainTempco(unit="1/K") = 0 "Relative current-gain change per kelvin of module temperature [datasheet]";
  parameter Real referenceTemperature(unit="K") = 298.15 "Temperature of the stated gains [datasheet]";
equation
  voltageSense = min(outputMax, max(outputMin, voltageGain*voltage + voltageOffset));
  currentSense = min(outputMax, max(outputMin,
    currentGain*(1 + currentGainTempco*(temperature - referenceTemperature))*current + currentOffset));
end AnalogSensor;
