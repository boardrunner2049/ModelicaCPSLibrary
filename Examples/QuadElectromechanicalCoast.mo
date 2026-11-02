within ModelicaCPSLibrary.Examples;
model QuadElectromechanicalCoast "Illustrative P2 quad: hover, cutoff, residual current and restart"
  parameter Real kT=1e-5;
  parameter Real kQ=1.5e-7;
  parameter Real hoverSpeed=sqrt(1.5*9.80665/(4*kT));
  parameter Real hoverCurrent=kQ*hoverSpeed^2/0.02;
  parameter Real hoverCommand=(0.02*hoverSpeed+0.2*hoverCurrent)/16;
  Vehicles.Copter.MultirotorWithSensors vehicle(
    geometry=Vehicles.Copter.Presets.QuadX(),landingGear(enabled=false));
  Physical.Electrical.Drives.ESC.AverageUnidirectional escs[4];
  Systems.Propulsion.Motors.DynamicDCEquivalent motors[4](each R=0.2,each L=0.001,
    each k=0.02,each J=0.0001,each JLoad=0.0001,
    each omega_start=hoverSpeed,each i_start=hoverCurrent);
  Physical.Mechanical.Aerodynamics.Blades.QuadraticBlade rotors[4](
    each kT=kT,each kQ=kQ,spinSign=vehicle.geometry.spinSign);
  Systems.Propulsion.HubReaction reactions[4](each J=0.0001,each JLoad=0.0001,
    spinSign=vehicle.geometry.spinSign);
  output Real altitude(unit="m");
equation
  escs.dcVoltage=fill(16,4);
  escs.escCommand=fill(if time<0.3 then hoverCommand else if time<0.6 then 0 else 0.95,4);
  motors.currentZero=escs.coast;
  escs.openCircuitVoltage=motors.backEMF;
  connect(escs.p,motors.p);
  connect(escs.n,motors.n);
  motors.loadTorque=rotors.loadTorque;
  rotors.omega=motors.omega;
  rotors.airVelocity_r=vehicle.rotorAirVelocity_r;
  rotors.omegaBody_r=vehicle.rotorOmegaBody_r;
  reactions.aerodynamicMoment_r=rotors.moment_r;
  reactions.acceleration=motors.acceleration;
  vehicle.rotorForce_r=rotors.force_r;
  vehicle.rotorMoment_r=reactions.moment_r;
  altitude=-vehicle.truth.p_w[3];
  annotation(experiment(StopTime=1,Tolerance=1e-9,Interval=0.0001),
    Documentation(info="<html><p>Illustrative parameters only, not a calibrated BLDC or ESC. Plot motors.current, motors.omega, escs.coast, escs.freewheel and escs.electricalPower.</p></html>"));
end QuadElectromechanicalCoast;
