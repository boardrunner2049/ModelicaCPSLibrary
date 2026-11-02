within ModelicaCPSLibrary.Systems.Sensing.IMU;
model ElectromechanicalSensor "I3: six physical axes, shared sampled output; external forces belong to vulnerabilities"
  extends PartialSampledSensor(final deviceReadout=true);
  parameter ModelicaCPSLibrary.Physical.Sensors.MEMS.GyroParameters gyroPhysics;
  parameter ModelicaCPSLibrary.Physical.Sensors.MEMS.AccelerometerParameters accelPhysics;
  input Real gyroDriveForce[3](each unit="N")=zeros(3);
  input Real gyroSenseForce[3](each unit="N")=zeros(3);
  input Real accelForce[3](each unit="N")=zeros(3);
  output Boolean gyroLocked[3] "Loop/readout diagnostic, separate from sample delivery validity";
  VibratoryGyro gyroscopes[3](each p=gyroPhysics,externalDriveForce=gyroDriveForce,externalSenseForce=gyroSenseForce);
  CapacitiveAccelerometer accelerometers[3](each p=accelPhysics,externalForce=accelForce);
protected
  Real physicalAcceleration[3];
  Real physicalRate[3];
equation
  for j in 1:3 loop
    gyroscopes[j].angularRate=ideal.gyro[j];
    accelerometers[j].specificForce=ideal.acceleration[j];
    physicalAcceleration[j]=accelerometers[j].measuredAcceleration;
    physicalRate[j]=gyroscopes[j].measuredRate;
    gyroLocked[j]=gyroscopes[j].locked;
  end for;
  // Residual calibration only; true angular rate is never added to the readout.
  frontEnd[1:3]=(identity(3)+diagonal(p.scale[1:3])+p.accelMisalignment)*physicalAcceleration+accelBias
    +p.temperatureCoefficient[1:3]*(sensorTemperature-p.referenceTemperature);
  frontEnd[4:6]=(identity(3)+diagonal(p.scale[4:6])+p.gyroMisalignment)*physicalRate
    +p.gyroGSensitivity*physicalAcceleration+gyroBias+p.temperatureCoefficient[4:6]*(sensorTemperature-p.referenceTemperature);
end ElectromechanicalSensor;
