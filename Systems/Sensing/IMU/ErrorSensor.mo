within ModelicaCPSLibrary.Systems.Sensing.IMU;
model ErrorSensor "I2: I1 plus calibration, temperature, Gauss-Markov bias and noise"
  extends PartialDeviceSensor(final enableErrors=true);
  input Real equivalentError[6] = zeros(6) "External vulnerability, separate from nominal errors";
equation
  acousticError=equivalentError;
end ErrorSensor;
