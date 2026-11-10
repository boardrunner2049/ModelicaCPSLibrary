within ModelicaCPSLibrary.Systems.Sensing.PowerModule;
partial model PartialAnalogSensor "Power-module contract: true voltage and current in, voltage-sense and current-sense outputs (V)"
  parameter Real outputMin(unit="V") = 0 "Lowest output voltage (rail) [datasheet]";
  parameter Real outputMax(unit="V") = 3.3 "Highest output voltage (rail, or the ADC range it is designed for) [datasheet]";
  input Real voltage(unit="V") "True battery voltage";
  input Real current(unit="A") "True battery current, positive discharging";
  input Real temperature(unit="K") = 298.15 "Module (ambient) temperature";
  output Real voltageSense(unit="V") "Voltage-sense output";
  output Real currentSense(unit="V") "Current-sense output";
  annotation(Documentation(info="<html><p>Continuous analog outputs, valid from the start: there is
no sampling inside the module. Both are clipped to <code>outputMin</code>..<code>outputMax</code>.
Downstream ADC conversion and scaling belong to the consumer.</p></html>"));
end PartialAnalogSensor;
