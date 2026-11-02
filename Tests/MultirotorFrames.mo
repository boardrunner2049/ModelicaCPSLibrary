within ModelicaCPSLibrary.Tests;
model MultirotorFrames "Tilted rotor, shifted CG, hub torque exactly once"
  Vehicles.Copter.MultirotorPlant vehicle(
    geometry(nArms=1,nRotors=2,nActuators=3,
      rotorArmIndex={1,1},actuatorIndex={3,1},spinSign={1,-1},
      rotorPosition_C=[0.2,0.1,0;-0.1,0.05,0.05],
      R_br={{{0,0,1},{0,1,0},{-1,0,0}},identity(3)}),
    aggregate(r_C={0.03,-0.02,0.01}),gravity_w=zeros(3));
equation
  vehicle.rotorForce_r=[0,0,1.6;0,0,0.4];
  vehicle.rotorMoment_r=[0,0,-0.024;0,0,0.006];
  assert(max(abs(vehicle.force_b-{1.6,0,0.4}))<1e-9,"Rotor local-to-body force mapping failed");
  assert(max(abs(vehicle.moment_b-{0.004,0.036,-0.186}))<1e-9,"Moment arm must use combined CG and be added exactly once");
  annotation(experiment(StopTime=0.1,Tolerance=1e-9,Interval=0.001));
end MultirotorFrames;
