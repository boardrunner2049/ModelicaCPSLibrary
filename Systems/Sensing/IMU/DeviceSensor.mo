within ModelicaCPSLibrary.Systems.Sensing.IMU;
model DeviceSensor "Legacy I2: temperature, analog response, sampled ADC, digital filter and decimation"
  extends PartialDeviceSensor;
equation
  acousticError=zeros(6);
end DeviceSensor;
