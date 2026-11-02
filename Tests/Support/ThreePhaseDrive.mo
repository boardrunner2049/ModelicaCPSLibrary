within ModelicaCPSLibrary.Tests.Support;
model ThreePhaseDrive "Reusable bus + ESC + motor + R0 load bench"
  parameter Modelica.Units.SI.AngularVelocity omega_start = 0;
  parameter Real kQ = 1.5e-7;
  input Real command;
  Systems.Power.DCLink bus(sourceResistance=0.02,dcLinkCapacitance=0.002,voltage_start=16,sourceVoltage=16);
  replaceable Systems.Propulsion.Motors.ThreePhasePM motor
    constrainedby Systems.Propulsion.Motors.ThreePhasePM(RPhase=0.2,LPhase=0.001,ke=0.01,
    polePairs=7,J=0.0001,JLoad=0.0001,omega_start=omega_start);
  replaceable Physical.Electrical.Drives.ESC.AverageThreePhase esc
    constrainedby Physical.Electrical.Drives.ESC.PartialThreePhase(Ron=0.005,Rdiode=0.005,Vdiode=0.5);
  Modelica.Units.SI.Energy netEnergy(start=0,fixed=true);
  output Modelica.Units.SI.Energy energyError;
  output Modelica.Units.SI.AngularVelocity speed;
  output Modelica.Units.SI.Current phaseCurrent[3];
equation
  connect(bus.p,esc.dc_p);
  connect(bus.n,esc.dc_n);
  connect(esc.phase,motor.phase);
  esc.electricalAngle = motor.electricalAngle;
  esc.escCommand = command;
  motor.loadTorque = kQ*motor.omega*abs(motor.omega);
  der(netEnergy) = bus.sourcePower-bus.lossPower-esc.lossPower-motor.copperLoss
    -motor.frictionLoss-motor.loadPower;
  energyError = netEnergy-(motor.storedEnergy+bus.storedEnergy
    -0.002*16*16/2-0.0002*omega_start*omega_start/2);
  speed = motor.omega;
  phaseCurrent = motor.phaseCurrent;
  assert(abs(sum(phaseCurrent)) < 1e-7,"Floating-star KCL failed");
  assert(esc.lossPower > -1e-6,"ESC generated energy internally");
  assert(abs(energyError) < 1e-4,"Integrated electrical/mechanical energy balance failed");
end ThreePhaseDrive;
