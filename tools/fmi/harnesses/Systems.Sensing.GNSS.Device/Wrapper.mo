model FireHarness_GNSSDevice "Coherent moving antenna for M9N configuration checks"
  parameter Real initialPdopLimit=3;
  parameter Real initialStaticSpeed=1;
  parameter Real initialStaticDistance=2;
  input Real command[20](start=zeros(20));
  ModelicaCPSLibrary.Systems.Sensing.GNSS.Device sensor(
    redeclare model VendorConfigModel=ModelicaCPSLibrary.Systems.Sensing.GNSS.VendorConfig.M9N,
    vendorConfig(configCommand=command,reportedPdop=1,pdopLimit=initialPdopLimit,
      staticHoldSpeedThreshold=initialStaticSpeed,staticHoldMaxDistance=initialStaticDistance));
  output Real response[20]=sensor.vendorConfig.configResponse;
  output Real navPvt[14]=sensor.vendorConfig.navPvt;
  output Real position[3]=sensor.position;
  output Real velocity[3]=sensor.velocity;
equation
  sensor.position_w={10+.25*time,-4+.5*time,1-.125*time};
  sensor.velocity_w={.25,.5,-.125};
end FireHarness_GNSSDevice;
