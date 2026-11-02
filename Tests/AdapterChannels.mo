within ModelicaCPSLibrary.Tests;
model AdapterChannels "Stateless eight-channel PWM clipping and sensor mount transformations"
  Adapters.VirtualIO.PwmInput commands(nChannels=8);
  Adapters.VirtualIO.SensorValues values(geometry=Vehicles.Copter.Presets.OctoX(
    R_bImu=[0,-1,0;1,0,0;0,0,1],R_bMag=diagonal({1,-1,-1})));
equation
  commands.pulseWidth_us={900,1100,1200,1300,1400,1500,1600,2100};
  values.measurements.acceleration={1,2,3};
  values.measurements.gyro={4,5,6};
  values.measurements.magneticField={1e-5,2e-5,3e-5};
  values.measurements.position={time,2*time,-3*time};
  values.measurements.velocity={1,2,-3};
  values.measurements.pressure=101325;
  values.measurements.temperature=288.15;
  values.measurements.altitude=3*time;
  values.measurements.climbRate=3;
  assert(max(abs(commands.normalizedCommand-{0,0.1,0.2,0.3,0.4,0.5,0.6,1}))<1e-12,"PWM channels must not be trimmed or truncated");
  assert(max(abs(values.acceleration_frd-{-2,1,3}))<1e-12,"IMU mount conversion failed");
  assert(max(abs(values.gyro_frd-{-5,4,6}))<1e-12,"Gyro mount conversion failed");
  assert(max(abs(values.magneticField_frd-{1e-5,-2e-5,-3e-5}))<1e-12,"Magnetometer mount conversion failed");
  assert(max(abs(values.position_ned-{time,2*time,-3*time}))<1e-12,"NED position must pass through continuously");
  assert(max(abs(values.velocity_ned-{1,2,-3}))<1e-12,"NED velocity must not be converted twice");
  annotation(experiment(StopTime=0.035,Tolerance=1e-9,Interval=0.0007));
end AdapterChannels;
