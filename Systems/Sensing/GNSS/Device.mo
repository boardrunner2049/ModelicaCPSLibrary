within ModelicaCPSLibrary.Systems.Sensing.GNSS;
model Device "GNSS physics followed by optional vendor configuration"
  extends PartialSensor;
  replaceable model PhysicsModel=Sensor constrainedby PartialSensor;
  replaceable model VendorConfigModel=VendorConfig.PassThrough
    constrainedby VendorConfig.PartialModel;
  PhysicsModel physics(positionBias=positionBias,velocityBias=velocityBias);
  VendorConfigModel vendorConfig;
equation
  physics.position_w=position_w;
  physics.velocity_w=velocity_w;
  vendorConfig.measuredPosition=physics.position;
  vendorConfig.measuredVelocity=physics.velocity;
  position=vendorConfig.position;
  velocity=vendorConfig.velocity;
end Device;
