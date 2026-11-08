within ModelicaCPSLibrary.Vehicles.FixedWing;

model FixedWingPlant "Conventional fixed-wing: aileron, elevator, rudder, tractor propeller, tricycle gear"
  import Modelica.Units.SI;
  parameter SI.Mass mass = 2.0;
  parameter SI.Inertia inertia[3, 3] = diagonal({0.15, 0.10, 0.24})
    "About the CG in body FRD (estimated for a 1.9 m span, 2 kg airframe)";
  parameter Physical.Mechanical.Aerodynamics.Airframe.Coefficients aero;
  parameter Real aeroReference_b[3](each unit = "m") = {-0.05, 0, 0}
    "Moment reference point of the coefficients relative to the CG; 5 cm aft trims neutral elevator near 15 m/s";
  parameter SI.Density airDensity = 1.225;
  parameter SI.Angle maxAileron = 0.3491 "Deflection at full command";
  parameter SI.Angle maxElevator = 0.3491;
  parameter SI.Angle maxRudder = 0.3491;
  parameter SI.Time servoTimeConstant = 0.04 "First-order control-surface servos";
  parameter Integer servoModel = 0
    "0: first-order servos (S0); 1: first-order servos with rate limit and deadband (S1)"
    annotation(Evaluate = true);
  parameter SI.AngularVelocity maxServoRate = 7.0 "S1: surface rate limit (about 0.15 s per 60 degrees)";
  parameter SI.Angle servoDeadband = 0.002 "S1: no motion while within this of the command";
  parameter SI.Force maxThrust = 30 "Static thrust at full throttle";
  parameter SI.Velocity propellerPitchSpeed = 60 "Axial airspeed at which thrust falls to zero";
  parameter SI.Time motorTimeConstant = 0.1 "Throttle to thrust lag";
  parameter Real thrustPosition_b[3](each unit = "m") = zeros(3)
    "Thrust line point relative to the CG; thrust acts along body x";
  parameter SI.Length cgHeight = 0.15 "CG height above the ground with unloaded gear";
  parameter SI.Length wheelRadius = 0.035;
  parameter SI.Length noseWheelX = 0.25 "Nose wheel ahead of the CG";
  parameter SI.Length mainWheelX = -0.05 "Main wheels relative to the CG (behind: negative)";
  parameter SI.Length mainWheelTrack = 0.44;
  parameter SI.Angle maxNoseWheelSteer = 0.35 "Nose wheel angle at full rudder";
  parameter Real gearStiffness(unit = "N/m") = 2500 "Per wheel";
  parameter Real gearDamping(unit = "N.s/m") = 60 "Per wheel";
  parameter Real frictionCoefficient = 0.8;
  parameter Real corneringStiffness(unit = "1/rad") = 8;
  parameter Real rollingResistance = 0.03;
  parameter SI.Position groundZ = 0 "NED down coordinate of the ground plane";
  parameter SI.Position p_start[3] = {0, 0, -cgHeight};
  parameter SI.Velocity v_start[3] = zeros(3) "Body velocity at CG";
  parameter Real q_start[4] = {1, 0, 0, 0} "Identity faces north";
  parameter SI.AngularVelocity omega_start[3] = zeros(3);
  parameter Real throttle_start(min = 0, max = 1) = 0 "Initial thrust state";

  input Real aileronCommand(unit = "1") "-1 .. +1 (+ rolls right); clamped";
  input Real elevatorCommand(unit = "1") "-1 .. +1 (+ pitches nose up); clamped";
  input Real rudderCommand(unit = "1") "-1 .. +1 (+ yaws nose right, steers right); clamped";
  input Real throttleCommand(unit = "1") "0 .. 1; clamped";
  input Real gravity_w[3](each unit = "m/s2") = {0, 0, 9.80665};
  input Real wind_w[3](each unit = "m/s") = zeros(3);
  input Real externalForce_b[3](each unit = "N") = zeros(3) "Non-gravity force at CG";
  input Real externalMoment_b[3](each unit = "N.m") = zeros(3) "Moment about CG";

  final parameter Real wheelPosition_b[3, 3] = {
    {noseWheelX, 0, cgHeight - wheelRadius},
    {mainWheelX, -mainWheelTrack / 2, cgHeight - wheelRadius},
    {mainWheelX, mainWheelTrack / 2, cgHeight - wheelRadius}} "Nose, main left, main right hubs";

  Physical.Mechanical.Dynamics.RigidBody6DOF body(mass = mass, inertia = inertia,
    p_start = p_start, v_start = v_start, q_start = q_start, omega_start = omega_start);
  Physical.Mechanical.Aerodynamics.Airframe.StabilityDerivatives airframe(
    coefficients = aero, airDensity = airDensity);
  Physical.Mechanical.Wheels.CompliantWheel gear[3](
    rHub_b = wheelPosition_b,
    each radius = wheelRadius,
    each stiffness = gearStiffness,
    each damping = gearDamping,
    each frictionCoefficient = frictionCoefficient,
    each corneringStiffness = corneringStiffness,
    each rollingResistance = rollingResistance,
    each driven = false);

  output SI.Angle aileron(start = 0, fixed = true);
  output SI.Angle elevator(start = 0, fixed = true);
  output SI.Angle rudder(start = 0, fixed = true);
  output Real throttle(start = throttle_start, fixed = true) "Thrust state, 0 .. 1";
  output SI.Force thrust;
  output SI.Velocity airspeed "Magnitude of the air-relative CG velocity";
  output Interfaces.VehicleState truth;
  output SI.Force force_b[3] "Sum of non-gravity loads";
  output SI.Torque moment_b[3] "Sum about CG";
protected
  SI.Velocity airVelocity_b[3];
initial equation
  assert(servoModel >= 0 and servoModel <= 1 and maxServoRate > 0 and servoDeadband >= 0,
    "servoModel is 0 or 1, with a positive rate limit");
  assert(cgHeight > wheelRadius and noseWheelX > mainWheelX,
    "Gear geometry: hubs below the CG, nose wheel ahead of the main wheels");
equation
  der(aileron) = if servoModel == 1 then Utilities.Math.rateLimitedLag(
    maxAileron * noEvent(min(1, max(-1, aileronCommand))), aileron, servoTimeConstant, maxServoRate, servoDeadband)
    else (maxAileron * noEvent(min(1, max(-1, aileronCommand))) - aileron) / servoTimeConstant;
  der(elevator) = if servoModel == 1 then Utilities.Math.rateLimitedLag(
    maxElevator * noEvent(min(1, max(-1, elevatorCommand))), elevator, servoTimeConstant, maxServoRate, servoDeadband)
    else (maxElevator * noEvent(min(1, max(-1, elevatorCommand))) - elevator) / servoTimeConstant;
  der(rudder) = if servoModel == 1 then Utilities.Math.rateLimitedLag(
    maxRudder * noEvent(min(1, max(-1, rudderCommand))), rudder, servoTimeConstant, maxServoRate, servoDeadband)
    else (maxRudder * noEvent(min(1, max(-1, rudderCommand))) - rudder) / servoTimeConstant;
  der(throttle) = (noEvent(min(1, max(0, throttleCommand))) - throttle) / motorTimeConstant;

  airVelocity_b = body.v_b - transpose(body.R_wb) * wind_w;
  airspeed = sqrt(airVelocity_b * airVelocity_b + 1e-12);
  airframe.vAir_b = airVelocity_b + cross(body.omega_b, aeroReference_b);
  airframe.omega_b = body.omega_b;
  airframe.aileron = aileron;
  airframe.elevator = elevator;
  airframe.rudder = rudder;
  // Thrust falls linearly with axial airspeed (fixed-pitch propeller).
  thrust = maxThrust * throttle * (1 - airVelocity_b[1] / propellerPitchSpeed);

  for i in 1:3 loop
    gear[i].p_w = body.p_w;
    gear[i].v_b = body.v_b;
    gear[i].omega_b = body.omega_b;
    gear[i].R_wb = body.R_wb;
    gear[i].terrainPoint_w = {0, 0, groundZ};
    gear[i].terrainNormal_w = {0, 0, -1};
    gear[i].terrainVelocity_w = zeros(3);
    gear[i].surfaceSpeed = 0;
  end for;
  gear[1].steerAngle = maxNoseWheelSteer * rudder / maxRudder;
  gear[2].steerAngle = 0;
  gear[3].steerAngle = 0;

  for j in 1:3 loop
    force_b[j] = airframe.force_b[j] + (if j == 1 then thrust else 0)
      + sum(gear[i].force_b[j] for i in 1:3) + externalForce_b[j];
  end for;
  moment_b = airframe.moment_b + cross(aeroReference_b, airframe.force_b)
    + cross(thrustPosition_b, {thrust, 0, 0})
    + gear[1].moment_b + gear[2].moment_b + gear[3].moment_b + externalMoment_b;
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
  annotation(Documentation(info = "<html><p>A small electric fixed-wing on a flat runway. The airframe
is <code>StabilityDerivatives</code> (default: the last_letter Skywalker 2013 coefficients); the nose
wheel steers with the rudder. Propeller torque, slipstream, ground effect and wind gradients are
neglected. Mass and inertia are estimates, not measurements.</p></html>"));
end FixedWingPlant;
