within ModelicaCPSLibrary.Tests;
model M9N "M9N acquisition and output validity over moving G0 measurements"
  parameter Real pdopLimit=3 "Select before initialization; compare separate runs with 3 and 1";
  model M9NDevice=Systems.Sensing.GNSS.Device(
    redeclare model VendorConfigModel=Systems.Sensing.GNSS.VendorConfig.M9N);
  M9NDevice sensor(vendorConfig(pdopLimit=pdopLimit));
  M9NDevice biased(
    positionBias={1,2,3},velocityBias={0.1,-0.2,0.3},vendorConfig(pdopLimit=pdopLimit));
  M9NDevice equalLimit(vendorConfig(pdopLimit=2));
  M9NDevice aboveLimit(vendorConfig(pdopLimit=2.1));
equation
  sensor.position_w={10+2*time,-4+3*time,1-time};
  sensor.velocity_w={2,3,-1};
  biased.position_w=sensor.position_w;
  biased.velocity_w=sensor.velocity_w;
  equalLimit.position_w=sensor.position_w;
  equalLimit.velocity_w=sensor.velocity_w;
  aboveLimit.position_w=sensor.position_w;
  aboveLimit.velocity_w=sensor.velocity_w;
  when sample(0.05,0.2) then
    assert(abs(sensor.vendorConfig.navPvt[2]-(sensor.vendorConfig.navPvt[1]-1)*0.2)<1e-8,
      "M9N sample epoch must agree with sequence");
    assert(max(abs(sensor.position-{10+2*sensor.vendorConfig.navPvt[2],-4+3*sensor.vendorConfig.navPvt[2],1-sensor.vendorConfig.navPvt[2]}))<1e-8,
      "M9N position must be held at the acquisition epoch even when validity is rejected");
    assert(max(abs(sensor.velocity-{2,3,-1}))<1e-8,
      "M9N velocity must be retained when validity is rejected");
    assert(max(abs(biased.position-sensor.position-{1,2,3}))<1e-8 and
      max(abs(biased.velocity-sensor.velocity-{0.1,-0.2,0.3}))<1e-8,
      "The physics component must apply position and velocity biases exactly once");
    assert(abs(sensor.vendorConfig.navPvt[4]-3)<1e-12 and abs(equalLimit.vendorConfig.navPvt[5])<1e-12 and abs(aboveLimit.vendorConfig.navPvt[5]-1)<1e-12,
      "M9N fix type and strict PDOP acceptance must remain distinct");
    assert(abs(sensor.vendorConfig.navPvt[5]-(if 2<pdopLimit then 1 else 0))<1e-12,
      "M9N acceptance must follow the fixed per-run PDOP limit");
  end when;
  annotation(experiment(StopTime=1.05,Interval=0.01,Tolerance=1e-9));
end M9N;
