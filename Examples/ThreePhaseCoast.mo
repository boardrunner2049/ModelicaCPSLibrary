within ModelicaCPSLibrary.Examples;
model ThreePhaseCoast "P3 single-motor startup, current decay, coast and restart"
  Systems.Power.DCLink bus(sourceResistance=0.02,dcLinkCapacitance=0.002,voltage_start=16,sourceVoltage=16);
  Systems.Propulsion.Motors.ThreePhasePM motor(RPhase=0.2,LPhase=0.001,ke=0.01,
    polePairs=7,J=0.0001,JLoad=0.0001);
  replaceable Physical.Electrical.Drives.ESC.AverageThreePhase esc
    constrainedby Physical.Electrical.Drives.ESC.PartialThreePhase(Ron=0.005,Rdiode=0.005,Vdiode=0.5);
  Physical.Mechanical.Aerodynamics.Blades.QuadraticBlade rotor;
equation
  connect(bus.p,esc.dc_p);
  connect(bus.n,esc.dc_n);
  connect(esc.phase,motor.phase);
  esc.electricalAngle = motor.electricalAngle;
  esc.escCommand = if time < 0.03 then 0.6 else if time < 0.05 then 0 else 0.7;
  rotor.omega = motor.omega;
  rotor.airVelocity_r = zeros(3);
  rotor.omegaBody_r = zeros(3);
  motor.loadTorque = rotor.loadTorque;
  annotation(experiment(StopTime=0.08,Interval=0.00001,Tolerance=1e-8));
end ThreePhaseCoast;
