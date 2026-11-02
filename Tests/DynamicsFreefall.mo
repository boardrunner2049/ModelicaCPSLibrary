within ModelicaCPSLibrary.Tests;
model DynamicsFreefall "Ballistic trajectory and zero CG specific force"
  ModelicaCPSLibrary.Physical.Mechanical.Dynamics.RigidBody6DOF body(
    mass = 2, p_start = {2, 1, 5}, v_start = {1, 2, 3});
equation
  body.force_b = zeros(3);
  body.moment_b = zeros(3);
  body.gravity_w = {0, 0, 9.81};
  assert(max(abs(body.p_w - {2 + time, 1 + 2 * time, 5 + 3 * time + 4.905 * time ^ 2})) < 1e-6,
    "Freefall position does not match the ballistic solution");
  assert(max(abs(body.v_w - {1, 2, 3 + 9.81 * time})) < 1e-6,
    "Freefall world velocity is incorrect");
  assert(max(abs(body.specificForce_b)) < 1e-10, "Freefall CG accelerometer must read zero");
  annotation(experiment(StopTime = 1, Tolerance = 1e-9, Interval = 0.01));
end DynamicsFreefall;
