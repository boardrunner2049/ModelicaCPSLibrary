within ModelicaCPSLibrary.Tests;
model ElectromechanicalCoast "Current continuity, freewheel, open coast, blocked command and restart"
  Physical.Electrical.Drives.ESC.AverageUnidirectional esc;
  Systems.Propulsion.Motors.DynamicDCEquivalent motor(
    R=0.2,L=0.001,k=0.02,J=0.0001,JLoad=0.0001,i_start=1.2,omega_start=400);
  Physical.Mechanical.Aerodynamics.Blades.QuadraticBlade blade(kQ=1.5e-7,spinSign=-1);
  Systems.Propulsion.HubReaction reaction(J=0.0001,JLoad=0.0001,spinSign=-1);
  Physical.Electrical.Drives.ESC.AverageUnidirectional unpowered;
  Systems.Propulsion.Motors.DynamicDCEquivalent spinning(
    R=0.2,L=0.001,k=0.02,J=0.0002,b=0.0001,omega_start=200);
  Systems.Propulsion.HubReaction frictionReaction(J=0.0002,spinSign=1);
  Real inputEnergy(start=0,fixed=true,unit="J");
  Real dissipatedEnergy(start=0,fixed=true,unit="J");
  discrete Real coastSpeed(start=0,fixed=true,unit="rad/s");
  Real coastExpected(unit="rad/s");
equation
  unpowered.dcVoltage=0;
  unpowered.escCommand=1;
  unpowered.openCircuitVoltage=spinning.backEMF;
  spinning.currentZero=unpowered.coast;
  connect(unpowered.p,spinning.p);
  connect(unpowered.n,spinning.n);
  spinning.loadTorque=0;
  frictionReaction.aerodynamicMoment_r=zeros(3);
  frictionReaction.acceleration=spinning.acceleration;
  assert(abs(spinning.current)<1e-8 and abs(unpowered.dcCurrent)<1e-8,
    "Rotating zero-current initialization must coast with zero supply");
  assert(abs(spinning.omega-200*exp(-time/2))<1e-5,"Viscous coast solution failed");
  assert(abs(frictionReaction.moment_r[3]-0.0001*spinning.omega)<1e-8,
    "Bearing friction reaction must act on the carrier once");
  esc.dcVoltage=16;
  esc.escCommand=if time<0.1 then 0.515 else if time<0.5 then 0 else if time<0.6 then 0.1 else if time<0.9 then 0.8 else 0;
  motor.currentZero=esc.coast;
  esc.openCircuitVoltage=motor.backEMF;
  connect(esc.p,motor.p);
  connect(esc.n,motor.n);
  blade.omega=motor.omega;
  blade.airVelocity_r=zeros(3);
  blade.omegaBody_r=zeros(3);
  motor.loadTorque=blade.loadTorque;
  reaction.aerodynamicMoment_r=blade.moment_r;
  reaction.acceleration=motor.acceleration;
  der(inputEnergy)=esc.electricalPower;
  der(dissipatedEnergy)=motor.copperLoss+motor.frictionLoss+motor.loadPower;
  when time>=0.1 then
    assert(abs(motor.current-1.2)<1e-5,"Cutoff must not reset inductive current");
  end when;
  when time>=0.11 then
    coastSpeed=motor.omega;
  end when;
  coastExpected=coastSpeed/(1+1.5e-7*coastSpeed*(time-0.11)/0.0002);
  assert(motor.current>=-1e-6 and esc.dcCurrent>=-1e-6,"Reverse current is forbidden");
  assert(abs(motor.storedEnergy+dissipatedEnergy-inputEnergy-(16+0.00072))<2e-5,
    "Motor/ESC energy accounting failed");
  assert(time<=0.10001 or time>=0.1001 or (motor.current>0 and esc.freewheel and abs(esc.dcCurrent)<1e-8),
    "Residual current must circulate after command cutoff");
  assert(time<=0.12 or time>=0.6 or (abs(motor.current)<1e-6 and esc.coast
    and abs(esc.terminalVoltage-0.02*motor.omega)<1e-6),"Coast must be open circuit, not zero terminal voltage");
  assert(time<=0.12 or time>=0.6 or abs(motor.omega-coastExpected)<1e-4,"Quadratic-load coast solution changed");
  assert(time<=0.12 or time>=0.6 or abs(reaction.moment_r[3])<1e-7,"Free rotor drag must not be counted twice on carrier");
  assert(abs(reaction.moment_r[3]-motor.electromagneticTorque)<1e-7,"Hub reaction sign or inertia bookkeeping failed");
  assert(time<=0.7 or time>=0.9 or motor.current>0,"Drive must restart after coast");
  annotation(experiment(StopTime=1.1,Tolerance=1e-10,Interval=0.0001));
end ElectromechanicalCoast;
