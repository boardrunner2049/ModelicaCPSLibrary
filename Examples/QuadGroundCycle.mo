within ModelicaCPSLibrary.Examples;

model QuadGroundCycle "Drop, settle, open-loop takeoff, motor cutoff and landing in NED/FRD"
  Vehicles.Copter.MultirotorWithSensors vehicle(
    landingGear(enabled=true),p_start={0,0,-0.12});
  output Real altitude(unit="m") = -vehicle.body.p_w[3];
  output Real verticalVelocity(unit="m/s") = vehicle.body.v_w[3] "Down positive";
  output Real normalLoad(unit="N") = sum(vehicle.gear.normalForce);
  output Integer contactCount = sum(if vehicle.gear.contact[i] then 1 else 0 for i in 1:4);
  output Real acceleration_frd[3](each unit="m/s2") =
    vehicle.geometry.R_bImu*vehicle.measurements.acceleration;
  Physical.Electrical.Drives.ESC.PassThrough escs[vehicle.geometry.nRotors];
  Systems.Propulsion.Motors.FirstOrderSpeed motors[vehicle.geometry.nRotors](each omegaMax=1000,each tau=0.03);
  Physical.Mechanical.Aerodynamics.Blades.QuadraticBlade rotors[vehicle.geometry.nRotors](
    each kT=1e-5,each kQ=1.5e-7,spinSign=vehicle.geometry.spinSign);
equation
  rotors.omega=motors.omega;
  rotors.airVelocity_r=vehicle.rotorAirVelocity_r;
  rotors.omegaBody_r=vehicle.rotorOmegaBody_r;
  vehicle.rotorForce_r=rotors.force_r;
  vehicle.rotorMoment_r=rotors.moment_r;
  motors.demand=escs.normalizedCommand;
  escs.escCommand = {if time < 2 or time >= 2.7 then 0 else
    sqrt(1.3*vehicle.chassis.mass*9.80665/(vehicle.geometry.nRotors*rotors[i].kT))/motors[i].omegaMax
    for i in 1:vehicle.geometry.nRotors};
  annotation(experiment(StopTime=6,Tolerance=1e-8,Interval=0.002),
    Documentation(info="<html><p>Unpowered feet start 2 cm above the plane. At 2 s a symmetric speed command requests 1.3 times weight in steady thrust. Motors switch off at 2.7 s, allowing a second landing. Plot altitude, verticalVelocity, normalLoad, contactCount and acceleration_frd[3]. This is an open-loop physics demonstration, not an ArduCopter flight controller.</p></html>"));
end QuadGroundCycle;
