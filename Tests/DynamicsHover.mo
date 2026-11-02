within ModelicaCPSLibrary.Tests;
model DynamicsHover "Weight balanced by applied thrust"
  ModelicaCPSLibrary.Physical.Mechanical.Dynamics.RigidBody6DOF body(
    mass = 2, p_start = {0, 0, -1});
equation
  body.force_b = {0, 0, -19.62};
  body.moment_b = zeros(3);
  body.gravity_w = {0, 0, 9.81};
  assert(max(abs(body.p_w - {0, 0, -1})) < 1e-9 and max(abs(body.v_w)) < 1e-9,
    "Hover must preserve position and velocity");
  assert(max(abs(body.specificForce_b - {0, 0, -9.81})) < 1e-9,
    "Level hover specific force must be -g on body z");
  assert(max(abs(body.a_w)) < 1e-9, "Hover inertial acceleration must be zero");
  annotation(experiment(StopTime = 1, Tolerance = 1e-9, Interval = 0.01));
end DynamicsHover;
