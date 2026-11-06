within ModelicaCPSLibrary.Vehicles.Rover;

model RoverPlant "Four-wheel Ackermann rover: steered front wheels, rear open differential driven by a DC motor"
  import Modelica.Units.SI;
  parameter SI.Mass mass = 3.0 "Total mass, including wheels and drivetrain";
  parameter SI.Inertia inertia[3, 3] = diagonal({0.028, 0.056, 0.073})
    "About the CG in body FRD (0.45 x 0.30 x 0.15 m uniform box)";
  parameter SI.Length wheelbase = 0.32;
  parameter SI.Length track = 0.26;
  parameter SI.Length cgToFrontAxle = 0.16 "Longitudinal distance from CG to the front axle";
  parameter SI.Length cgHeight = 0.09 "CG height above the ground with unloaded suspension";
  parameter SI.Length wheelRadius = 0.055;
  parameter Real suspensionStiffness(unit = "N/m") = 1500 "Per wheel, tyre and suspension combined";
  parameter Real suspensionDamping(unit = "N.s/m") = 40 "Per wheel";
  parameter Real frictionCoefficient = 0.9;
  parameter Real longitudinalStiffness = 15 "Per unit slip and normal load";
  parameter Real corneringStiffness(unit = "1/rad") = 10 "Per radian and unit normal load";
  parameter Real rollingResistance = 0.015;
  parameter SI.Angle maxSteerAngle = 0.45 "Equivalent single-track steer angle at full command";
  parameter SI.Time steeringTimeConstant = 0.08 "First-order steering servo";
  parameter SI.Torque motorStallTorque = 1.4 "Axle torque at zero speed and full command";
  parameter SI.AngularVelocity motorNoLoadSpeed = 182 "Axle speed at zero load and full command";
  parameter SI.Inertia wheelInertia = 1.5e-3 "Per rear wheel, with its share of the reflected motor and gear inertia";
  parameter Real dragArea(unit = "m2") = 0.03 "Drag coefficient times frontal area";
  parameter SI.Density airDensity = 1.225;
  parameter SI.Position groundZ = 0 "NED down coordinate of the ground plane";
  parameter SI.Position p_start[3] = {0, 0, -cgHeight};
  parameter SI.Velocity v_start[3] = zeros(3) "Body velocity at CG";
  parameter Real q_start[4] = {1, 0, 0, 0} "Identity faces north";
  parameter SI.AngularVelocity omega_start[3] = zeros(3);

  input Real steeringCommand(unit = "1") "-1 full left .. +1 full right; clamped";
  input Real throttleCommand(unit = "1") "-1 full reverse .. +1 full forward; clamped";
  input Real gravity_w[3](each unit = "m/s2") = {0, 0, 9.80665};
  input Real wind_w[3](each unit = "m/s") = zeros(3);
  input Real externalForce_b[3](each unit = "N") = zeros(3) "Non-gravity force at CG";
  input Real externalMoment_b[3](each unit = "N.m") = zeros(3) "Moment about CG";

  final parameter SI.Length cgToRearAxle = wheelbase - cgToFrontAxle;
  final parameter SI.Length hubHeight_b = cgHeight - wheelRadius "Hub below CG (FRD z)";
  final parameter Real hubPosition_b[4, 3] = {
    {cgToFrontAxle, -track / 2, hubHeight_b}, {cgToFrontAxle, track / 2, hubHeight_b},
    {-cgToRearAxle, -track / 2, hubHeight_b}, {-cgToRearAxle, track / 2, hubHeight_b}}
    "Front left, front right, rear left, rear right";

  Physical.Mechanical.Dynamics.RigidBody6DOF body(mass = mass, inertia = inertia,
    p_start = p_start, v_start = v_start, q_start = q_start, omega_start = omega_start);
  Physical.Mechanical.Wheels.CompliantWheel wheels[4](
    rHub_b = hubPosition_b,
    each radius = wheelRadius,
    each stiffness = suspensionStiffness,
    each damping = suspensionDamping,
    each frictionCoefficient = frictionCoefficient,
    each longitudinalStiffness = longitudinalStiffness,
    each corneringStiffness = corneringStiffness,
    each rollingResistance = rollingResistance,
    driven = {false, false, true, true});

  SI.Angle steerAngle(start = 0, fixed = true) "Single-track steer angle";
  SI.AngularVelocity wheelSpeed[2](each start = v_start[1] / wheelRadius, each fixed = true)
    "Rear left, rear right spin";
  output SI.Angle wheelSteer[2] "Front left, front right (Ackermann)";
  output SI.Torque motorTorque "At the differential input (axle)";
  output Interfaces.VehicleState truth;
  output SI.Force force_b[3] "Sum of non-gravity loads";
  output SI.Torque moment_b[3] "Sum about CG";
protected
  Real steering(unit = "1");
  Real throttle(unit = "1");
  SI.Velocity airVelocity_b[3];
  SI.Force drag_b[3];
initial equation
  assert(wheelbase > 0 and track > 0 and wheelRadius > 0 and cgHeight > wheelRadius,
    "Rover geometry must be positive with the CG above the hubs");
  assert(maxSteerAngle > 0 and track / 2 * tan(maxSteerAngle) < wheelbase,
    "Ackermann geometry needs the inner wheel turn centre ahead of the rear axle");
equation
  steering = noEvent(min(1, max(-1, steeringCommand)));
  throttle = noEvent(min(1, max(-1, throttleCommand)));
  der(steerAngle) = (maxSteerAngle * steering - steerAngle) / steeringTimeConstant;
  // Ackermann: both front wheels point at the turn centre on the rear axle line.
  wheelSteer[1] = atan(wheelbase * tan(steerAngle) / (wheelbase + track / 2 * tan(steerAngle)));
  wheelSteer[2] = atan(wheelbase * tan(steerAngle) / (wheelbase - track / 2 * tan(steerAngle)));

  // Voltage-commanded DC motor through an open differential: equal torque to
  // both rear wheels, motor speed follows their mean. Zero command brakes.
  motorTorque = motorStallTorque * (throttle - (wheelSpeed[1] + wheelSpeed[2]) / 2 / motorNoLoadSpeed);
  for i in 1:2 loop
    wheelInertia * der(wheelSpeed[i]) = motorTorque / 2 - wheels[2 + i].longitudinalForce * wheelRadius;
  end for;

  for i in 1:4 loop
    wheels[i].p_w = body.p_w;
    wheels[i].v_b = body.v_b;
    wheels[i].omega_b = body.omega_b;
    wheels[i].R_wb = body.R_wb;
    wheels[i].terrainPoint_w = {0, 0, groundZ};
    wheels[i].terrainNormal_w = {0, 0, -1};
    wheels[i].terrainVelocity_w = zeros(3);
  end for;
  wheels[1].steerAngle = wheelSteer[1];
  wheels[2].steerAngle = wheelSteer[2];
  wheels[3].steerAngle = 0;
  wheels[4].steerAngle = 0;
  wheels[1].surfaceSpeed = 0;
  wheels[2].surfaceSpeed = 0;
  wheels[3].surfaceSpeed = wheelSpeed[1] * wheelRadius;
  wheels[4].surfaceSpeed = wheelSpeed[2] * wheelRadius;

  airVelocity_b = body.v_b - transpose(body.R_wb) * wind_w;
  drag_b = -0.5 * airDensity * dragArea * sqrt(airVelocity_b * airVelocity_b + 1e-4) * airVelocity_b;
  for j in 1:3 loop
    force_b[j] = sum(wheels[i].force_b[j] for i in 1:4) + drag_b[j] + externalForce_b[j];
    moment_b[j] = sum(wheels[i].moment_b[j] for i in 1:4) + externalMoment_b[j];
  end for;
  body.force_b = force_b;
  body.moment_b = moment_b;
  body.gravity_w = gravity_w;
  truth.p_w = body.p_w;
  truth.v_w = body.v_w;
  truth.v_b = body.v_b;
  truth.q_wb = body.q_wb;
  truth.R_wb = body.R_wb;
  truth.omega_b = body.omega_b;
  truth.a_w = body.a_w;
  truth.alpha_b = body.alpha_b;
  annotation(Documentation(info = "<html><p>A 1/10-scale class electric rover. Six rigid-body degrees
of freedom on four compliant wheels, so pitch and roll on the suspension, load transfer, wheel spin,
skids and rollover come out of the contact forces; none are imposed. Wheel order: front left, front
right, rear left, rear right. Aerodynamic drag acts at the CG. The default values are representative,
not a measured vehicle.</p></html>"));
end RoverPlant;
