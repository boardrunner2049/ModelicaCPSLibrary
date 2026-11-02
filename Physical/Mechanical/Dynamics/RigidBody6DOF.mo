within ModelicaCPSLibrary.Physical.Mechanical.Dynamics;
model RigidBody6DOF "Single rigid body; all applied moments are about its CG"
  parameter Modelica.Units.SI.Mass mass = 1;
  parameter Modelica.Units.SI.Inertia inertia[3, 3] = identity(3)
    "Symmetric positive-definite inertia about CG, resolved in body FRD";
  parameter Modelica.Units.SI.Position p_start[3] = zeros(3);
  parameter Modelica.Units.SI.Velocity v_start[3] = zeros(3)
    "Initial CG velocity resolved in body FRD";
  parameter Real q_start[4] = {1, 0, 0, 0}
    "Initial Hamilton quaternion {w,x,y,z}, body to world; normalized at initialization";
  parameter Modelica.Units.SI.AngularVelocity omega_start[3] = zeros(3);
  parameter Real quaternionNormGain(unit = "1/s", min = 0) = 1
    "Continuous quaternion norm stabilization gain";

  input Modelica.Units.SI.Force force_b[3] "Sum of non-gravity forces in body";
  input Modelica.Units.SI.Torque moment_b[3] "Sum of moments about CG in body";
  input Modelica.Units.SI.Acceleration gravity_w[3] "Gravity in world NED";

  output Modelica.Units.SI.Position p_w[3](start = p_start, each fixed = true);
  output Modelica.Units.SI.Velocity v_b[3](start = v_start, each fixed = true);
  output Real q_wb[4](start = q_initial, each fixed = true);
  output Modelica.Units.SI.AngularVelocity omega_b[3](
    start = omega_start, each fixed = true);
  output Real R_wb[3, 3] "Body-to-world rotation matrix";
  output Modelica.Units.SI.Velocity v_w[3];
  output Modelica.Units.SI.Acceleration a_w[3] "Inertial CG acceleration";
  output Modelica.Units.SI.AngularAcceleration alpha_b[3];
  output Modelica.Units.SI.Acceleration specificForce_b[3]
    "CG accelerometer specific force; excludes gravity";
  output Modelica.Units.SI.Angle euler[3]
    "Display-only 3-2-1 angles {roll,pitch,yaw}; singular at pitch +/-pi/2";

protected
  final parameter Real q_start_norm = sqrt(q_start[1] * q_start[1]
    + q_start[2] * q_start[2] + q_start[3] * q_start[3] + q_start[4] * q_start[4]);
  final parameter Real q_initial[4] = q_start / max(q_start_norm, 1e-15);
  final parameter Real inertiaScale = max(abs(inertia));
  final parameter Real determinant =
    inertia[1, 1] * (inertia[2, 2] * inertia[3, 3] - inertia[2, 3] * inertia[3, 2])
    - inertia[1, 2] * (inertia[2, 1] * inertia[3, 3] - inertia[2, 3] * inertia[3, 1])
    + inertia[1, 3] * (inertia[2, 1] * inertia[3, 2] - inertia[2, 2] * inertia[3, 1]);
  // Expand the 3x3 inverse in parameter arithmetic so explicit ODE exporters
  // need neither an algebraic solver nor an external matrix-inverse function.
  // Invalid inertia is rejected below; keep its parameter evaluation finite
  // so the positive-definiteness assertion supplies the diagnostic.
  final parameter Real inverseInertia[3, 3] = if determinant > 0 then
    {{inertia[2, 2] * inertia[3, 3] - inertia[2, 3] * inertia[3, 2],
     inertia[1, 3] * inertia[3, 2] - inertia[1, 2] * inertia[3, 3],
     inertia[1, 2] * inertia[2, 3] - inertia[1, 3] * inertia[2, 2]},
     {inertia[2, 3] * inertia[3, 1] - inertia[2, 1] * inertia[3, 3],
     inertia[1, 1] * inertia[3, 3] - inertia[1, 3] * inertia[3, 1],
     inertia[1, 3] * inertia[2, 1] - inertia[1, 1] * inertia[2, 3]},
     {inertia[2, 1] * inertia[3, 2] - inertia[2, 2] * inertia[3, 1],
     inertia[1, 2] * inertia[3, 1] - inertia[1, 1] * inertia[3, 2],
     inertia[1, 1] * inertia[2, 2] - inertia[1, 2] * inertia[2, 1]}} / determinant
    else zeros(3, 3);

initial equation
  assert(mass > 0, "RigidBody6DOF requires positive mass");
  assert(q_start_norm > 1e-12, "Initial quaternion must be nonzero");
  assert(quaternionNormGain >= 0, "Quaternion norm gain must be nonnegative");
  assert(max(abs(inertia - transpose(inertia))) <= 1e-12 * max(inertiaScale, 1e-30),
    "Inertia must be symmetric");
  assert(inertia[1, 1] > 0 and
    inertia[1, 1] * inertia[2, 2] - inertia[1, 2] * inertia[2, 1] > 0 and determinant > 0,
    "Inertia must be positive definite (positive leading principal minors)");

equation
  for i in 1:3 loop
    for j in 1:3 loop
      R_wb[i,j] = ModelicaCPSLibrary.Utilities.Math.quaternionRotationElement(q_wb,i,j);
    end for;
  end for;
  specificForce_b = force_b / mass;
  v_w = R_wb * v_b;
  a_w = R_wb * specificForce_b + gravity_w;
  der(p_w) = v_w;
  der(v_b) = specificForce_b + transpose(R_wb) * gravity_w - cross(omega_b, v_b);
  alpha_b = inverseInertia * (moment_b - cross(omega_b, inertia * omega_b));
  der(omega_b) = alpha_b;
  der(q_wb) = 0.5 * ModelicaCPSLibrary.Utilities.Math.quaternionProduct(
    q_wb, {0, omega_b[1], omega_b[2], omega_b[3]})
    + quaternionNormGain * (1 - q_wb * q_wb) * q_wb;
  euler = {atan2(R_wb[3, 2], R_wb[3, 3]),
    asin(noEvent(min(1, max(-1, -R_wb[3, 1])))),
    atan2(R_wb[2, 1], R_wb[1, 1])};
end RigidBody6DOF;
