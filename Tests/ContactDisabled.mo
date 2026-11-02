within ModelicaCPSLibrary.Tests;

model ContactDisabled "Disabling landing gear preserves freefall through the ground plane"
  Vehicles.Copter.MultirotorWithSensors vehicle(
    landingGear(enabled=false),p_start={0,0,-0.12});
equation
  vehicle.rotorForce_r=zeros(vehicle.geometry.nRotors,3);
  vehicle.rotorMoment_r=zeros(vehicle.geometry.nRotors,3);
  assert(max(abs(vehicle.gear.force_b))+max(abs(vehicle.gear.moment_b))<1e-12,
    "Disabled landing gear must contribute no wrench");
  assert(max(abs(vehicle.measurements.acceleration))<1e-9,"Freefall accelerometer must read zero");
  when time>=0.5 then
    assert(abs(vehicle.body.p_w[3]-(-0.12+0.5*9.80665*time^2))<1e-6,
      "Disabled contact must not arrest freefall below the plane");
  end when;
  annotation(experiment(StopTime=0.6,Tolerance=1e-9,Interval=0.002));
end ContactDisabled;
