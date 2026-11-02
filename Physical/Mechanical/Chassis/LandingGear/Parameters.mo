within ModelicaCPSLibrary.Physical.Mechanical.Chassis.LandingGear;

record Parameters "Four fixed contact points in chassis FRD and a stationary horizontal NED plane"
  parameter Boolean enabled = true "False removes all contact points at translation";
  parameter Real position_C[4,3](each unit="m") =
    [0.17,0.17,0.10; -0.17,-0.17,0.10; 0.17,-0.17,0.10; -0.17,0.17,0.10]
    "Uncompressed tips relative to chassis reference C, not combined CG";
  parameter Real groundZ(unit="m") = 0 "NED Down coordinate of the ground plane";
  parameter Real stiffness(min=0,unit="N/m") = 3000 "Per contact point";
  parameter Real damping(min=0,unit="N.s/m") = 150 "Per contact point";
  parameter Real tangentialDamping(min=0,unit="N.s/m") = 25;
  parameter Real frictionCoefficient(min=0) = 0.6;
  annotation(Documentation(info="<html><p>The rigid vehicle supplies the sprung mass. Include any physical landing-gear mass in the existing aggregate or assembled mass budget exactly once. This record adds neither mass nor motion states. A horizontal NED plane has outward normal {0,0,-1}; positive body Z places feet below the chassis.</p></html>"));
end Parameters;
