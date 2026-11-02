within ModelicaCPSLibrary.Vehicles.Copter;
model MultirotorPlant "Common NED/FRD mass, rigid body, mounting and contact"
  parameter Vehicles.Copter.Geometry geometry;
  parameter Physical.Mechanical.Chassis.LandingGear.Parameters landingGear;
  parameter Boolean useAssembledMass = false;
  parameter Physical.Mechanical.MassProperties aggregate(
    mass=1.5,inertia=diagonal({0.02,0.02,0.04}));
  parameter Physical.Mechanical.Chassis.CenterBody.RigidCenterBody core(mass=1.2,inertia=diagonal({0.015,0.015,0.025}));
  parameter Physical.Mechanical.Chassis.Arms.RigidArm arms[geometry.nArms](
    each mass=0.05,r_C=geometry.armMount/2,each inertia=diagonal({0.0001,0.0001,0.0002}));
  parameter Integer nPayloads(min=0) = 0;
  parameter Physical.Mechanical.Chassis.Payloads.FixedPayload payloads[nPayloads];
  parameter Integer nAdditionalParts(min=0) = 0;
  parameter Physical.Mechanical.MassProperties additionalParts[nAdditionalParts];
  parameter Real p_start[3] = {0,0,-1};
  parameter Real v_start[3] = zeros(3) "Body velocity at CG";
  parameter Real q_start[4] = {sqrt(0.5),0,0,sqrt(0.5)} "East-facing baseline; identity faces North";
  parameter Real omega_start[3] = zeros(3);
  input Real gravity_w[3](each unit="m/s2") = {0,0,9.80665};
  input Real wind_w[3](each unit="m/s") = zeros(3);
  input Real externalForce_b[3](each unit="N") = zeros(3) "Non-gravity force at CG";
  input Real externalMoment_b[3](each unit="N.m") = zeros(3) "Moment about combined CG";
  Physical.Mechanical.Chassis.ChassisAssembly chassis(
    useAssembledMass=useAssembledMass,aggregate=aggregate,core=core,
    nArms=geometry.nArms,arms=arms,nPayloads=nPayloads,payloads=payloads,
    nAdditionalParts=nAdditionalParts,additionalParts=additionalParts);
  final parameter Real rotorPosition_b[geometry.nRotors,3] =
    {geometry.rotorPosition_C[i,:]-chassis.cg_C for i in 1:geometry.nRotors};
  final parameter Integer nLandingLegs = if landingGear.enabled then 4 else 0;
  final parameter Real legPosition_b[nLandingLegs,3] =
    {landingGear.position_C[i,:]-chassis.cg_C for i in 1:nLandingLegs};
  Physical.Mechanical.Chassis.LandingGear.LandingGearAssembly gear(
    nLegs=nLandingLegs,rLeg_b=legPosition_b,
    stiffness=landingGear.stiffness,damping=landingGear.damping,
    tangentialDamping=landingGear.tangentialDamping,
    frictionCoefficient=landingGear.frictionCoefficient);
  Physical.Mechanical.Dynamics.RigidBody6DOF body(mass=chassis.mass,inertia=chassis.inertia,
    p_start=p_start,v_start=v_start,q_start=q_start,omega_start=omega_start);
  output Interfaces.VehicleState truth;
  output Real force_b[3](each unit="N") "Sum of non-gravity loads";
  output Real moment_b[3](each unit="N.m") "Sum about combined CG";
  output Real rotorForce_b[geometry.nRotors,3](each unit="N");
  output Real rotorMoment_b[geometry.nRotors,3](each unit="N.m");
  input Real rotorForce_r[geometry.nRotors,3](each unit="N") "Rotor-local force at each hub";
  input Real rotorMoment_r[geometry.nRotors,3](each unit="N.m") "Rotor-local hub moment, excluding the CG moment arm";
  output Real rotorAirVelocity_r[geometry.nRotors,3](each unit="m/s") "Hub velocity relative to air in rotor axes";
  output Real rotorOmegaBody_r[geometry.nRotors,3](each unit="rad/s") "Carrier angular velocity in rotor axes";
initial equation
  for i in 1:geometry.nRotors loop
    assert(geometry.rotorArmIndex[i]>=1 and geometry.rotorArmIndex[i]<=geometry.nArms,"Invalid rotor arm index");
    assert(geometry.actuatorIndex[i]>=1 and geometry.actuatorIndex[i]<=geometry.nActuators,"Invalid rotor actuator index");
    assert(max(abs(transpose(geometry.R_br[i,:,:])*geometry.R_br[i,:,:]-identity(3)))<1e-9,"Rotor mount must be a proper rotation");
    //  and abs(Modelica.Math.Matrices.det(geometry.R_br[i,:,:])-1)<1e-9
  end for;
equation
  gear.p_w = body.p_w;
  gear.v_b = body.v_b;
  gear.omega_b = body.omega_b;
  gear.R_wb = body.R_wb;
  gear.terrainPoint_w = {0,0,landingGear.groundZ};
  gear.terrainNormal_w = {0,0,-1};
  gear.terrainVelocity_w = zeros(3);
  for i in 1:geometry.nRotors loop
    rotorAirVelocity_r[i,:] = transpose(geometry.R_br[i,:,:])*(body.v_b+
      cross(body.omega_b,rotorPosition_b[i,:])-transpose(body.R_wb)*wind_w);
    rotorOmegaBody_r[i,:] = transpose(geometry.R_br[i,:,:])*body.omega_b;
    rotorForce_b[i,:] = geometry.R_br[i,:,:]*rotorForce_r[i,:];
    rotorMoment_b[i,:] = cross(rotorPosition_b[i,:],rotorForce_b[i,:])+
      geometry.R_br[i,:,:]*rotorMoment_r[i,:];
  end for;
  for j in 1:3 loop
    force_b[j] = sum(rotorForce_b[:,j])+gear.force_b[j]+externalForce_b[j];
    moment_b[j] = sum(rotorMoment_b[:,j])+gear.moment_b[j]+externalMoment_b[j];
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
end MultirotorPlant;
