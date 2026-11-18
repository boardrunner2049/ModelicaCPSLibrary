within ModelicaCPSLibrary.Systems.Sensing.Barometer.VendorConfig;
partial model PartialModel "Measured-value boundary; no truth, mounting or physical biases"
  input Real measuredPressure(unit="Pa");
  input Real measuredTemperature(unit="K");
  input Real measuredSampleTime(unit="s");
  input Real measuredDeliveryTime(unit="s");
  input Integer measuredSequence;
  input Boolean measuredValid;
  output Real pressure(unit="Pa");
  output Real temperature(unit="K");
  output Real sampleTime(unit="s");
  output Real deliveryTime(unit="s");
  output Integer sequence;
  output Boolean valid;
end PartialModel;
