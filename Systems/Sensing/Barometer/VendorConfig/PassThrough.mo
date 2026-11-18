within ModelicaCPSLibrary.Systems.Sensing.Barometer.VendorConfig;
model PassThrough "Forward measured values and timing without adding state or a clock"
  extends PartialModel;
equation
  pressure=measuredPressure;
  temperature=measuredTemperature;
  sampleTime=measuredSampleTime;
  deliveryTime=measuredDeliveryTime;
  sequence=measuredSequence;
  valid=measuredValid;
end PassThrough;
