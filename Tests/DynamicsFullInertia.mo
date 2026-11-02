within ModelicaCPSLibrary.Tests;
model DynamicsFullInertia "Manufactured angular trajectory with off-diagonal inertia"
  parameter Real J[3, 3] = [2, 0.3, -0.2; 0.3, 3, 0.4; -0.2, 0.4, 4];
  ModelicaCPSLibrary.Physical.Mechanical.Dynamics.RigidBody6DOF body(
    inertia = J, omega_start = {0.1, 0.2, -0.1});
  Real expectedOmega[3];
equation
  expectedOmega = {0.1 + 0.2 * time, 0.2 - 0.15 * time, -0.1 + 0.1 * time};
  body.force_b = zeros(3);
  body.gravity_w = zeros(3);
  // J*{0.2,-0.15,0.1} evaluated independently, plus transport of angular momentum.
  body.moment_b = {0.335, -0.35, 0.3} + cross(expectedOmega, J * expectedOmega);
  assert(max(abs(body.omega_b - expectedOmega)) < 1e-6,
    "Full-inertia angular velocity differs from the manufactured trajectory");
  assert(max(abs(body.alpha_b - {0.2, -0.15, 0.1})) < 1e-6,
    "Full-inertia angular acceleration is incorrect");
  annotation(experiment(StopTime = 1, Tolerance = 1e-10, Interval = 0.01));
end DynamicsFullInertia;
