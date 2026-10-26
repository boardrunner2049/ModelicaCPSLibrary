within fire_modelica_models.Systems.Sensing.IMU;
partial model PartialDeviceSensor "Device response before ADC; stochastic errors are ADC-equivalent"
  extends PartialSampledSensor(final deviceReadout=true);
protected
  Real acousticError[6];
  Real analogState[6];
  Real drive[6];
initial equation
  analogState=drive "Steady input initialization; subsequent steps retain dynamics";
equation
  drive=calibrated+(if enableErrors then p.temperatureCoefficient*(sensorTemperature-p.referenceTemperature) else zeros(6))+acousticError;
  p.analogTau .* der(analogState)+analogState=drive;
  frontEnd=analogState;
end PartialDeviceSensor;
