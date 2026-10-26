within fire_modelica_models.Systems.Sensing.IMU;
model CapacitiveAccelerometer "One proof mass and capacitive readout; no gyro AGC/PLL"
  parameter fire_modelica_models.Physical.Sensors.MEMS.AccelerometerParameters p;
  input Real specificForce(unit="m/s2");
  input Real externalForce(unit="N")=0;
  output Real measuredAcceleration(unit="m/s2");
  fire_modelica_models.Physical.Sensors.MEMS.AccelerometerMechanics mechanics(p=p);
  fire_modelica_models.Physical.Sensors.Electronics.CapacitiveReadout readout(
    C0=p.readC0,length=p.readLength,carrierVoltage=p.carrierVoltage,Cf=p.feedbackCapacitance,tau=p.afeTau,rail=p.rail);
equation
  mechanics.specificForce=specificForce;
  mechanics.externalForce=externalForce;
  readout.displacement=mechanics.displacement;
  measuredAcceleration=-(2*Modelica.Constants.pi*p.frequency)^2*readout.measuredDisplacement;
end CapacitiveAccelerometer;
