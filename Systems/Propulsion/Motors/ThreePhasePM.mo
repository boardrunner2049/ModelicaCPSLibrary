within ModelicaCPSLibrary.Systems.Propulsion.Motors;
model ThreePhasePM "Floating-star three-phase PM motor and rigid shaft; shared by P3 and P4"
  parameter Modelica.Units.SI.Resistance RPhase;
  parameter Modelica.Units.SI.Inductance LPhase "Effective phase inductance in the balanced subspace";
  parameter Modelica.Units.SI.ElectricalTorqueConstant ke;
  parameter Integer polePairs(min=1);
  parameter Boolean trapezoidal = true;
  parameter Modelica.Units.SI.Inertia J;
  parameter Modelica.Units.SI.Inertia JLoad = 0;
  parameter Modelica.Units.SI.RotationalDampingConstant b = 0;
  parameter Modelica.Units.SI.Angle theta0 = 0;
  parameter Modelica.Units.SI.Current i_start[3] = zeros(3);
  parameter Modelica.Units.SI.AngularVelocity omega_start = 0;
  Modelica.Electrical.Analog.Interfaces.PositivePin phase[3];
  input Modelica.Units.SI.Torque loadTorque;
  Physical.Electrical.Passives.SeriesRL winding[3](each R=RPhase,each L=LPhase,
    i_start=i_start,fixedCurrentStart={true,true,false});
  Physical.Electrical.Electromechanical.PermanentMagnet3Phase emf(
    ke=ke,polePairs=polePairs,theta0=theta0,trapezoidal=trapezoidal);
  Physical.Mechanical.Rotational.RigidShaft shaft(J=J+JLoad,b=b,omega_start=omega_start);
  output Modelica.Units.SI.Current phaseCurrent[3];
  output Modelica.Units.SI.Voltage backEMF[3];
  output Modelica.Units.SI.Angle electricalAngle;
  output Modelica.Units.SI.AngularVelocity omega;
  output Modelica.Units.SI.AngularAcceleration acceleration;
  output Modelica.Units.SI.Torque electromagneticTorque;
  output Modelica.Units.SI.Power copperLoss;
  output Modelica.Units.SI.Power frictionLoss;
  output Modelica.Units.SI.Power loadPower;
  output Modelica.Units.SI.Energy storedEnergy;
protected
  Modelica.Mechanics.Rotational.Sources.Torque load(useSupport=false);
  Modelica.Mechanics.Rotational.Components.Fixed housing;
initial equation
  assert(RPhase > 0 and LPhase > 0 and J > 0 and JLoad >= 0 and b >= 0,
    "ThreePhasePM requires positive RPhase,LPhase,J and nonnegative JLoad,b");
  assert(abs(sum(i_start)) < 1e-10,"Floating-star initial currents must sum to zero");
equation
  connect(phase,winding.p);
  connect(winding.n,emf.p);
  // Unconnected neutral has zero flow, not zero voltage. Only two currents are independent.
  connect(emf.flange,shaft.drive);
  connect(emf.support,housing.flange);
  connect(shaft.support,housing.flange);
  connect(shaft.load,load.flange);
  load.tau = -loadTorque;
  phaseCurrent = winding.i;
  backEMF = emf.backEMF;
  electricalAngle = emf.electricalAngle;
  omega = shaft.omega;
  acceleration = shaft.acceleration;
  electromagneticTorque = emf.torque;
  copperLoss = sum(winding.copperLoss);
  frictionLoss = shaft.frictionLoss;
  loadPower = loadTorque*omega;
  storedEnergy = sum(winding.magneticEnergy)+shaft.kineticEnergy;
  annotation(Documentation(info="<html><p>Three identical effective SeriesRL windings and a floating star neutral. Two independent current states, shaft speed and angle. Parameters are per-phase SI, not P2 equivalents. ABC current signs are unrestricted; the controller chooses forward motoring. Fixed-carrier mechanics and HubReaction conventions are the same as P2. No current state reset at cutoff.</p></html>"));
end ThreePhasePM;
