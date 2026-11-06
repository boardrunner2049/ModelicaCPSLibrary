within ModelicaCPSLibrary.Exports.FMU;

model Rover "Ackermann rover, vehicle interface v1: pwm[1] steering, pwm[3] throttle (ArduRover servo order)"
  extends PartialSensedVehicleInterface(final R_bImu = plant.R_bImu, final R_bMag = plant.R_bMag);
  extends VehicleInterface.PwmActuation(final nPwm = 4, pwm_min = 1000, pwm_max = 2000,
    pwm(each start = 1500));
  Vehicles.Rover.RoverWithSensors plant(
    steeringCommand = 2 * commands.normalizedCommand[1] - 1,
    throttleCommand = 2 * commands.normalizedCommand[3] - 1,
    externalForce_b = external_force_frd,
    externalMoment_b = external_moment_frd);
  Adapters.VirtualIO.PwmInput commands(nChannels = nPwm,
    pwmMin = fill(pwm_min, nPwm), pwmMax = fill(pwm_max, nPwm));
  output Real wheelSpeed[2](each unit = "rad/s") "Diagnostic rear wheel spin (left, right)";
equation
  commands.pulseWidth_us = pwm;
  wheelSpeed = plant.wheelSpeed;
  truth = plant.truth;
  measurements = plant.measurements;
  annotation(Documentation(info = "<html><p>Steering and throttle are centred channels: the midpoint
of <code>pwm_min</code>..<code>pwm_max</code> is straight ahead and zero throttle; <code>pwm_max</code>
is full right and full forward, <code>pwm_min</code> full left and full reverse. Elements 2 and 4
are not connected. All channels start at neutral (1500 us).</p></html>"));
end Rover;
