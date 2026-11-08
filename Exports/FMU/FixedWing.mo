within ModelicaCPSLibrary.Exports.FMU;

model FixedWing "Conventional fixed-wing, vehicle interface v1: pwm[1..4] aileron, elevator, throttle, rudder (ArduPlane servo order)"
  extends PartialSensedVehicleInterface(final R_bImu = plant.R_bImu, final R_bMag = plant.R_bMag);
  extends VehicleInterface.PwmActuation(final nPwm = 4, pwm_min = 1000, pwm_max = 2000,
    pwm(start = {1500, 1500, 1000, 1500}));
  parameter Real start_altitude_m(unit = "m") = 0
    "Initial CG height above its unloaded on-ground height; 0 starts on the runway";
  parameter Real start_speed_mps(unit = "m/s") = 0 "Initial forward (body x) speed, wings level";
  Vehicles.FixedWing.FixedWingWithSensors plant(
    magneticField_w = {earth_field_north_T, earth_field_east_T, earth_field_down_T},
    p_start = {0, 0, -plant.cgHeight - start_altitude_m},
    v_start = {start_speed_mps, 0, 0},
    aileronCommand = 2 * commands.normalizedCommand[1] - 1,
    elevatorCommand = 2 * commands.normalizedCommand[2] - 1,
    throttleCommand = commands.normalizedCommand[3],
    rudderCommand = 2 * commands.normalizedCommand[4] - 1,
    externalForce_b = external_force_frd,
    externalMoment_b = external_moment_frd);
  Adapters.VirtualIO.PwmInput commands(nChannels = nPwm,
    pwmMin = fill(pwm_min, nPwm), pwmMax = fill(pwm_max, nPwm));
  output Real airspeed(unit = "m/s") "Diagnostic true airspeed";
  output Real thrust(unit = "N") "Diagnostic propeller thrust";
equation
  commands.pulseWidth_us = pwm;
  airspeed = plant.airspeed;
  thrust = plant.thrust;
  truth = plant.truth;
  measurements = plant.measurements;
  annotation(Documentation(info = "<html><p>Control surfaces are centred channels (midpoint of
<code>pwm_min</code>..<code>pwm_max</code> is neutral, <code>pwm_max</code> full positive deflection:
roll right, nose up, nose right). Throttle runs from <code>pwm_min</code> (off) to
<code>pwm_max</code> (full). Surfaces start neutral and throttle off.</p></html>"));
end FixedWing;
