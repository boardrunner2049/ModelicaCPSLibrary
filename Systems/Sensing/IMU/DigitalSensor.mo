within fire_modelica_models.Systems.Sensing.IMU;
model DigitalSensor "I1: deterministic bandwidth, ADC, IIR, decimation and delivery"
  extends PartialDeviceSensor(final enableErrors=false);
  input Real equivalentError[6] = zeros(6) "External vulnerability before readout, sensor-local SI";
initial equation
  assert(max(abs(accelBias))+max(abs(gyroBias))+max(abs(p.scale))+max(abs(p.noiseStd))+
    max(abs(p.biasStd))+max(abs(p.temperatureCoefficient))+sum(abs(p.accelMisalignment))+
    sum(abs(p.gyroMisalignment))+sum(abs(p.gyroGSensitivity))<=0,
    "I1 does not apply calibration/noise; select ErrorSensor (I2)");
equation
  acousticError=equivalentError;
end DigitalSensor;
