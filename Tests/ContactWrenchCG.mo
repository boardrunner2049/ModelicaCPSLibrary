within ModelicaCPSLibrary.Tests;

model ContactWrenchCG "Contact moment is about the combined CG, with ground height independent of origin"
  parameter Real weight=2*9.80665;
  Vehicles.Copter.MultirotorPlant vehicle(
    aggregate(mass=2,r_C={0.04,-0.03,0.02}),
    landingGear(groundZ=1.2),q_start={1,0,0,0},
    p_start={0,0,1.2-0.08+weight/12000},
    externalMoment_b={0.03*weight,0.04*weight,0});
equation
  vehicle.rotorForce_r=zeros(vehicle.geometry.nRotors,3);
  vehicle.rotorMoment_r=zeros(vehicle.geometry.nRotors,3);
  assert(max(abs(vehicle.gear.force_b-{0,0,-weight}))<1e-5,"NED contact force must point up and equal weight");
  assert(max(abs(vehicle.gear.moment_b-{-0.03*weight,-0.04*weight,0}))<1e-5,
    "Contact torque must include the CG shift exactly once");
  assert(max(abs(vehicle.body.v_b))+max(abs(vehicle.body.omega_b))<1e-5,
    "Contact wrench plus external balancing moment must preserve equilibrium");
  annotation(experiment(StopTime=0.2,Tolerance=1e-9,Interval=0.002));
end ContactWrenchCG;
