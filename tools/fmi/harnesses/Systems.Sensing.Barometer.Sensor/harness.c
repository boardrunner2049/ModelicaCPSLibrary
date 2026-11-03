/* FMI harness: Systems.Sensing.Barometer.Sensor.
 *
 * Pass-through barometer: pressure/temperature are the ambient values plus fixed
 * biases, altitude is the geometric (Up-positive) altitude plus a bias, climb rate
 * passes through unbiased; there is no pressure-to-height inversion.
 * The harness plays the environment for a vertical flight profile
 *   [0,1) on the ground, [1,11) climb at 10 m/s to 100 m, [11,13) hover,
 *   [13,18) descend at 8 m/s to 60 m, [18,20] hover,
 * feeding ISA-troposphere ambient conditions T = 288.15 - 0.0065 h,
 * p = 101325 (T/288.15)^5.25588. world.toml sets pressureBias = 12.5 Pa,
 * temperatureBias = -0.4 K, altitudeBias = 2 m. checks.toml verifies each
 * channel against these closed forms, the bias bookkeeping (climb rate carries
 * none), cross-channel consistency (hydrostatic pressure drop, climb rate = slope of
 * altitude, lapse rate) and the physical ordering (pressure falls with height). */
#include "fire_harness.h"

#include <math.h>

static double height(double t) {
  if (t < 1.0)
    return 0.0;
  if (t < 11.0)
    return 10.0 * (t - 1.0);
  if (t < 13.0)
    return 100.0;
  if (t < 18.0)
    return 100.0 - 8.0 * (t - 13.0);
  return 60.0;
}

static double climb(double t) {
  if (t >= 1.0 && t < 11.0)
    return 10.0;
  if (t >= 13.0 && t < 18.0)
    return -8.0;
  return 0.0;
}

static double isa_temperature(double t) { return 288.15 - 0.0065 * height(t); }
static double isa_pressure(double t) { return 101325.0 * pow(isa_temperature(t) / 288.15, 5.25588); }

static const fh_input_t inputs[] = {
    FH_INPUT(ambientpressure, FH_FUNCTION(isa_pressure)),
    FH_INPUT(ambienttemperature, FH_FUNCTION(isa_temperature)),
    FH_INPUT(altitude_w, FH_FUNCTION(height)),
    FH_INPUT(climbrate_w, FH_FUNCTION(climb)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(pressure),
    FH_OUTPUT(temperature),
    FH_OUTPUT(altitude),
    FH_OUTPUT(climbrate),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Systems.Sensing.Barometer.Sensor",
      .stop_s = 20.0,
      .step_s = 0.01,
      .print_s = 1.0,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
