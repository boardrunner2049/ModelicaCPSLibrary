within ModelicaCPSLibrary.Tests;
model DynamicsQuaternion "Hamilton body-rate composition and velocity transport"
  constant Real s = sqrt(0.5);
  ModelicaCPSLibrary.Physical.Mechanical.Dynamics.RigidBody6DOF body(
    q_start = {s, s, 0, 0}, omega_start = {0, 0, 2}, v_start = {1, 0, 0});
  Real expectedQuaternion[4];
initial equation
  assert(max(abs(Utilities.Math.quaternionToRotationMatrix({2,-3,4,-5})-
    [-14,-2,23;-22,-7,-14;7,-26,2]/27))<1e-12,"Nonunit quaternion rotation changed");
  assert(max(abs(Utilities.Math.quaternionToRotationMatrix({2,2,0,0})-
    [1,0,0;0,0,-1;0,1,0]))<1e-12,"Scaled quarter-roll rotation changed");
  assert(max(abs(Utilities.Math.quaternionToRotationMatrix(zeros(4))-identity(3)))<1e-12,
    "Legacy utility zero-quaternion behavior changed (body initialization still rejects zero)");
  assert(max(abs(Utilities.Math.quaternionToRotationMatrix({0,0.5e-9,0,0})-
    diagonal({1,0.5,0.5})))<1e-12,"Legacy utility epsilon clamp changed");
equation
  body.force_b = zeros(3);
  body.moment_b = zeros(3);
  body.gravity_w = zeros(3);
  // Initial roll rotation composed on the right with rotation about body z.
  expectedQuaternion = {s * cos(time), s * cos(time), -s * sin(time), s * sin(time)};
  assert(max(abs(body.q_wb - expectedQuaternion)) < 2e-6,
    "Quaternion must compose initial attitude with body angular velocity on the right");
  assert(abs(body.q_wb * body.q_wb - 1) < 1e-6, "Quaternion norm drift");
  assert(max(abs(body.v_b - {cos(2 * time), -sin(2 * time), 0})) < 2e-6,
    "Body velocity transport term is incorrect");
  assert(max(abs(body.v_w - {1, 0, 0})) < 2e-6,
    "Force-free translation must maintain world velocity under body rotation");
  assert(max(abs(body.p_w - {time, 0, 0})) < 2e-6,
    "Force-free rotating body must follow a straight trajectory");
  annotation(experiment(StopTime = 1, Tolerance = 1e-10, Interval = 0.01));
end DynamicsQuaternion;
