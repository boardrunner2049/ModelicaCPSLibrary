within ModelicaCPSLibrary.Systems.Sensing.Barometer;
model Device "Barometer physics followed by optional vendor configuration"
  extends PartialSensor;
  replaceable model PhysicsModel=Sensor constrainedby PartialSensor;
  replaceable model VendorConfigModel=VendorConfig.PassThrough
    constrainedby VendorConfig.PartialModel;
  PhysicsModel physics(pressureBias=pressureBias,temperatureBias=temperatureBias,
    altitudeBias=altitudeBias);
  VendorConfigModel vendorConfig;
equation
  physics.ambientPressure=ambientPressure;
  physics.ambientTemperature=ambientTemperature;
  physics.altitude_w=altitude_w;
  physics.climbRate_w=climbRate_w;
  vendorConfig.measuredPressure=physics.pressure;
  vendorConfig.measuredTemperature=physics.temperature;
  vendorConfig.measuredSampleTime=physics.sampleTime;
  vendorConfig.measuredDeliveryTime=physics.deliveryTime;
  vendorConfig.measuredSequence=physics.sequence;
  vendorConfig.measuredValid=physics.valid;
  pressure=vendorConfig.pressure;
  temperature=vendorConfig.temperature;
  sampleTime=vendorConfig.sampleTime;
  deliveryTime=vendorConfig.deliveryTime;
  sequence=vendorConfig.sequence;
  valid=vendorConfig.valid;
  altitude=physics.altitude;
  climbRate=physics.climbRate;
end Device;
