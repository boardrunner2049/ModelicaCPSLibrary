within ModelicaCPSLibrary.Systems.Sensing.PowerModule;
model ShuntAnalogSensor "A3: component level: resistor divider, shunt with temperature coefficient and self-heating, current-sense amplifier"
  extends PartialAnalogSensor;
  parameter Real dividerTop(unit="Ohm") = 17182 "Divider resistor to the battery [datasheet]";
  parameter Real dividerBottom(unit="Ohm") = 1000 "Divider resistor to ground [datasheet]";
  parameter Real voltageOffset(unit="V") = 0 "[datasheet]";
  parameter Real shuntResistance(unit="Ohm") = 0.0005 "At the reference temperature [datasheet]";
  parameter Real shuntTempco(unit="1/K") = 0 "Relative resistance change per kelvin [datasheet]";
  parameter Real amplifierGain(unit="1") = 55 "Current-sense amplifier voltage gain [datasheet]";
  parameter Real amplifierOffset(unit="V") = 0 "Output at zero shunt voltage [datasheet]";
  parameter Real thermalResistance(unit="K/W") = 20 "Shunt to ambient [assumed]";
  parameter Real thermalCapacitance(unit="J/K") = 2 "Shunt and its copper [assumed]";
  parameter Real referenceTemperature(unit="K") = 298.15 "[datasheet]";
  Real shuntTemperature(unit="K", start=298.15, fixed=false);
  Real shuntActual(unit="Ohm") "Resistance at the shunt temperature";
initial equation
  assert(dividerTop >= 0 and dividerBottom > 0 and shuntResistance > 0 and thermalResistance > 0
    and thermalCapacitance > 0, "Invalid shunt sensor components");
  shuntTemperature = temperature;
equation
  shuntActual = shuntResistance*(1 + shuntTempco*(shuntTemperature - referenceTemperature));
  thermalCapacitance*der(shuntTemperature) = current^2*shuntActual - (shuntTemperature - temperature)/thermalResistance;
  voltageSense = min(outputMax, max(outputMin, dividerBottom/(dividerTop + dividerBottom)*voltage + voltageOffset));
  currentSense = min(outputMax, max(outputMin, amplifierGain*shuntActual*current + amplifierOffset));
  annotation(Documentation(info="<html><p>From terminal measurements only the divider ratio and the
product amplifierGain*shuntResistance are identifiable, not the factors. The shunt starts at the module
temperature. The defaults reproduce the PM02 V3 reference scales (18.182 V/V, 36.364 A/V).</p></html>"));
end ShuntAnalogSensor;
