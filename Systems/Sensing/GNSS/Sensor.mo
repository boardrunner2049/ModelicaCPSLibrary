within ModelicaCPSLibrary.Systems.Sensing.GNSS;
model Sensor "G0: ideal continuous local NED antenna position and velocity"
  extends PartialSensor;
equation
  position=position_w+positionBias;
  velocity=velocity_w+velocityBias;
end Sensor;
