within ModelicaCPSLibrary.Tests;
model MultirotorPayload "Fixed payload rebases all rotor and sensor mounts"
  Vehicles.Copter.MultirotorWithSensors vehicle(useAssembledMass=true,
    geometry(imuPosition_C={0.05,0,0},gnssPosition_C={0,0,0.1},barometerPosition_C={0,0,0.02}),
    core(mass=2,inertia=identity(3)),arms(each mass=0,each inertia=zeros(3,3)),
    nPayloads=1,payloads={Physical.Mechanical.Chassis.Payloads.FixedPayload(
      mass=1,r_C={0.3,0,-0.15},inertia=diagonal({0.1,0.1,0.1}))},
    gravity_w=zeros(3));
initial equation
  assert(abs(vehicle.chassis.mass-3)<1e-12,"Payload must enter mass budget once");
  assert(max(abs(vehicle.chassis.cg_C-{0.1,0,-0.05}))<1e-12,"Payload CG mismatch");
  assert(max(abs(vehicle.chassis.inertia-[1.115,0,0.03;0,1.175,0;0.03,0,1.16]))<1e-12,"Payload full inertia mismatch");
  for i in 1:vehicle.geometry.nRotors loop
    assert(max(abs(vehicle.rotorPosition_b[i,:]-(vehicle.geometry.rotorPosition_C[i,:]-{0.1,0,-0.05})))<1e-12,"Rotor mount not rebased");
  end for;
  for i in 1:4 loop
    assert(max(abs(vehicle.legPosition_b[i,:]-(vehicle.landingGear.position_C[i,:]-{0.1,0,-0.05})))<1e-12,
      "Landing leg mount must be rebased by the combined payload CG");
  end for;
  assert(max(abs(vehicle.sensors.rImu_b-{-0.05,0,0.05}))<1e-12,"IMU lever arm not rebased");
  assert(max(abs(vehicle.sensors.rGnss_b-{-0.1,0,0.15}))<1e-12,"GNSS mount not rebased");
  assert(max(abs(vehicle.sensors.rBarometer_b-{-0.1,0,0.07}))<1e-12,"Barometer mount not rebased");
equation
  vehicle.rotorForce_r=zeros(vehicle.geometry.nRotors,3);
  vehicle.rotorMoment_r=zeros(vehicle.geometry.nRotors,3);
  annotation(experiment(StopTime=0.05,Tolerance=1e-9,Interval=0.001));
end MultirotorPayload;
