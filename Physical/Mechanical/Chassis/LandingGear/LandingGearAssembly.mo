within ModelicaCPSLibrary.Physical.Mechanical.Chassis.LandingGear;

model LandingGearAssembly "Independent contacts on an arbitrary number of legs"
  parameter Integer nLegs(min = 0) = 4;
  parameter Real rLeg_b[nLegs, 3](each unit = "m") = ModelicaCPSLibrary.Utilities.Math.regularPolygon(
      nLegs, sqrt(2)*0.17, 0.1, Modelica.Constants.pi/4)
    "Tip offsets from body CG in FRD; independent of rotor count";
  parameter Real stiffness(min = 0, unit = "N/m") = 3000;
  parameter Real damping(min = 0, unit = "N.s/m") = 150;
  parameter Real tangentialDamping(min = 0, unit = "N.s/m") = 25;
  parameter Real frictionCoefficient(min = 0) = 0.6;
  input Real p_w[3](each unit = "m");
  input Real v_b[3](each unit = "m/s");
  input Real omega_b[3](each unit = "rad/s");
  input Real R_wb[3, 3];
  input Real terrainPoint_w[3](each unit = "m");
  input Real terrainNormal_w[3];
  input Real terrainVelocity_w[3](each unit = "m/s");
  output Real force_b[3](each unit = "N");
  output Real moment_b[3](each unit = "N.m");
  output Boolean contact[nLegs];
  output Real gap[nLegs](each unit = "m");
  output Real normalForce[nLegs](each unit = "N");
  CompliantPointLeg legs[nLegs](
    rLeg_b = rLeg_b,
    each stiffness = stiffness,
    each damping = damping,
    each tangentialDamping = tangentialDamping,
    each frictionCoefficient = frictionCoefficient);
equation
  for i in 1:nLegs loop
    legs[i].p_w = p_w;
    legs[i].v_b = v_b;
    legs[i].omega_b = omega_b;
    legs[i].R_wb = R_wb;
    legs[i].terrainPoint_w = terrainPoint_w;
    legs[i].terrainNormal_w = terrainNormal_w;
    legs[i].terrainVelocity_w = terrainVelocity_w;
    contact[i] = legs[i].contact;
    gap[i] = legs[i].gap;
    normalForce[i] = legs[i].normalForce;
  end for;
  for j in 1:3 loop
    force_b[j] = sum(legs[i].force_b[j] for i in 1:nLegs);
    moment_b[j] = sum(legs[i].moment_b[j] for i in 1:nLegs);
  end for;
end LandingGearAssembly;
