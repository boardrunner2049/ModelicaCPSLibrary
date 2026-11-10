within ModelicaCPSLibrary.Systems.Peripherals;
model AnalogPowerModule "W2: the module's analog sense voltages (Sensing.PowerModule.AnalogSensor) read through the autopilot's ADC and scaling"
  extends PartialPowerModule;
  parameter Real samplePeriod(unit="s") = 0.1 "ADC read period [configuration]";
  parameter Real startTime(unit="s") = 0 "[configuration]";
  parameter Integer latencySamples(min=0) = 0 "[configuration]";
  parameter Integer seed = 1 "[configuration]";
  parameter Integer streamId = 352 "[configuration]";
  parameter Real voltageGain(unit="V/V") = 1/18.182 "Voltage-sense volts per battery volt [datasheet]";
  parameter Real voltageOffset(unit="V") = 0 "[datasheet]";
  parameter Real currentGain(unit="V/A") = 1/36.364 "Current-sense volts per ampere [datasheet]";
  parameter Real currentOffset(unit="V") = 0 "Current-sense output at zero current [datasheet]";
  parameter Real currentGainTempco(unit="1/K") = 0 "[datasheet]";
  parameter Real outputMin(unit="V") = 0 "[datasheet]";
  parameter Real outputMax(unit="V") = 3.3 "[datasheet]";
  parameter Real adcReference(unit="V") = 3.3 "ADC full scale [configuration]";
  parameter Integer adcBits(min=1) = 12 "[configuration]";
  parameter Real adcNoiseStd(unit="V") = 0 "ADC input-referred noise per read [assumed]";
  parameter Real voltageMultiplier = 18.182 "The autopilot's BATT_VOLT_MULT [configuration]";
  parameter Real ampsPerVolt(unit="A/V") = 36.364 "The autopilot's BATT_AMP_PERVLT [configuration]";
  parameter Real ampsOffset(unit="V") = 0 "The autopilot's BATT_AMP_OFFSET [configuration]";
  input Real moduleTemperature(unit="K") = 298.15 "Module temperature";
  output Real voltageSense(unit="V") "The module's voltage-sense output";
  output Real currentSense(unit="V") "The module's current-sense output";
protected
  Sensing.PowerModule.AnalogSensor sense(voltageGain=voltageGain, voltageOffset=voltageOffset,
    currentGain=currentGain, currentOffset=currentOffset, currentGainTempco=currentGainTempco,
    outputMin=outputMin, outputMax=outputMax, temperature=moduleTemperature);
  Sampler adc(n=2, samplePeriod=samplePeriod, startTime=startTime, latencySamples=latencySamples,
    seed=seed, streamId=streamId, noiseStd=fill(adcNoiseStd, 2), quantum=fill(adcReference/2^adcBits, 2),
    lower=zeros(2), upper=fill(adcReference, 2));
equation
  sense.voltage = battery.voltage;
  sense.current = packCurrent;
  voltageSense = sense.voltageSense;
  currentSense = sense.currentSense;
  adc.u = {voltageSense, currentSense};
  voltage = adc.y[1]*voltageMultiplier;
  current = (adc.y[2] - ampsOffset)*ampsPerVolt;
  valid = adc.valid;
  annotation(Documentation(info="<html><p>The sensor ends at <code>voltageSense</code> and
<code>currentSense</code> (exported as <code>power_module_analog_v1</code>); the ADC and the scaling
are the consumer's, here as ArduPilot applies BATT_VOLT_MULT, BATT_AMP_PERVLT and BATT_AMP_OFFSET.
A mismatch between the module's true gains and the configured scaling appears in
<code>voltage</code>/<code>current</code> as it would on the vehicle.</p></html>"));
end AnalogPowerModule;
