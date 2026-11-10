within ModelicaCPSLibrary.Systems.Sensing;
package PowerModule "Analog power-module sensing: true battery voltage and current in, the module's two sense voltages out"
  annotation(Documentation(info="<html><p>The sensor ends at its analog outputs. The autopilot's ADC and
its scaling (ArduPilot BATT_VOLT_MULT, BATT_AMP_PERVLT, BATT_AMP_OFFSET) are outside it:
<code>Systems.Peripherals.AnalogPowerModule</code> adds them for a vehicle.</p></html>"));
end PowerModule;
