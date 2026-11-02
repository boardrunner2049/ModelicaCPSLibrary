within ModelicaCPSLibrary.Tests;

model ContactTiltedLanding "Uneven touchdown restores roll/pitch through separate contact moments"
  Vehicles.Copter.MultirotorPlant vehicle(
    p_start={0,0,-0.3},q_start={cos(0.08),sin(0.08),0,0});
equation
  vehicle.rotorForce_r=zeros(vehicle.geometry.nRotors,3);
  vehicle.rotorMoment_r=zeros(vehicle.geometry.nRotors,3);
  when time>=2.5 then
    assert(abs(vehicle.body.R_wb[3,1])+abs(vehicle.body.R_wb[3,2])<1e-5,
      "Four independent contacts must settle the tilted vehicle level");
    assert(max(abs(vehicle.body.v_b))+max(abs(vehicle.body.omega_b))<1e-5,
      "Contact damping must settle translational and rotational motion");
    assert(min(vehicle.gear.normalForce)>0,"All four feet must support the settled vehicle");
  end when;
  annotation(experiment(StopTime=3,Tolerance=1e-8,Interval=0.002));
end ContactTiltedLanding;
