within ModelicaCPSLibrary.Systems.Propulsion.Motors;
model DynamicDCEquivalent "RL motor with back-EMF, shaft inertia and external load"
  parameter Modelica.Units.SI.Resistance R;
  parameter Modelica.Units.SI.Inductance L;
  parameter Modelica.Units.SI.ElectricalTorqueConstant k;
  parameter Modelica.Units.SI.Inertia J "Motor rotor inertia";
  parameter Modelica.Units.SI.Inertia JLoad = 0 "Rigidly attached propeller inertia";
  parameter Modelica.Units.SI.RotationalDampingConstant b = 0;
  parameter Modelica.Units.SI.Current i_start = 0;
  parameter Modelica.Units.SI.AngularVelocity omega_start = 0;
  final parameter Modelica.Units.SI.Inertia JTotal = J+JLoad;
  Modelica.Electrical.Analog.Interfaces.PositivePin p;
  Modelica.Electrical.Analog.Interfaces.NegativePin n;
  input Boolean currentZero "Drive-reported current zero crossing; not a cutoff command";
  input Modelica.Units.SI.Torque loadTorque "Positive torque opposing positive shaft rotation";
  Physical.Electrical.Passives.SeriesRL winding(R=R,L=L,i_start=i_start);
  Modelica.Electrical.Analog.Basic.RotationalEMF emf(k=k,useSupport=true);
  Physical.Mechanical.Rotational.RigidShaft shaft(J=JTotal,b=b,omega_start=omega_start);
  output Modelica.Units.SI.Current current;
  output Modelica.Units.SI.Voltage backEMF;
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
  assert(R > 0 and L > 0 and k > 0 and J > 0 and JLoad >= 0 and b >= 0,
    "DynamicDCEquivalent requires positive R,L,k,J and nonnegative JLoad,b");
equation
  connect(p,winding.p);
  connect(winding.n,emf.p);
  connect(emf.n,n);
  connect(emf.flange,shaft.drive);
  connect(emf.support,housing.flange);
  connect(shaft.support,housing.flange);
  connect(shaft.load,load.flange);
  load.tau = -loadTorque;
  current = winding.i;
  backEMF = k*omega;
  omega = shaft.omega;
  acceleration = shaft.acceleration;
  electromagneticTorque = k*current;
  copperLoss = winding.copperLoss;
  frictionLoss = shaft.frictionLoss;
  loadPower = loadTorque*omega;
  storedEnergy = winding.magneticEnergy+shaft.kineticEnergy;
  when currentZero then
    reinit(winding.inductor.i,0);
  end when;
  annotation(Documentation(info="<html><p>Fixed-carrier, rigid-shaft DC-equivalent motor. Electrical current and mechanical speed dynamics are composed from reusable passives, ideal energy conversion and shaft mechanics. The ESC owns current blocking/coast policy. Its currentZero event projects the actual winding-current state only upon reaching zero, never on command cutoff. A BLDC application requires consistently converted equivalent R,L,k and current definitions. Carrier rotation/gyroscopic coupling, saturation, thermal dynamics and current protection are not modeled. The vehicle mass budget includes physical rotor masses once; J/JLoad describe relative spin energy. HubReaction supplies the additional axial spin-acceleration moment under the fixed-carrier approximation.</p></html>"));
end DynamicDCEquivalent;
