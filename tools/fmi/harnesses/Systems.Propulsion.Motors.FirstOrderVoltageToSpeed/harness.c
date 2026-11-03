/* FMI harness: Systems.Propulsion.Motors.FirstOrderVoltageToSpeed.
 *
 * omega' = (Kv*V - omega)/tauOmega, Kv = 62.5 rad/(s.V), tauOmega = 30 ms;
 * world.toml overrides omega_start = 50 rad/s so the initial-state binding
 * is visible. No speed cap. Equivalent voltage (>= 0 is asserted; a
 * negative voltage must fail the step, exercised outside this scenario):
 *   0.0-0.1  0 V        coast-down from 50 rad/s
 *   0.1-0.4  16 V       step: target 1000 rad/s
 *   0.4-0.6  16 -> 4 V  ramp (zero-order held per 1 ms step)
 *   0.6-0.8  4 V        settle at 250 rad/s
 *   0.8-0.9  24 V       above the nominal bus: 1500 rad/s target, no cap
 *   0.9-1.3  0 V        exponential coast-down
 * checks.toml compares against the exact solution of the ZOH-sampled
 * input (computed in closed form per step). */
#include "fire_harness.h"

static double voltage_profile(double t) {
  if (t < 0.1)
    return 0.0;
  if (t < 0.4)
    return 16.0;
  if (t < 0.6)
    return 16.0 - (t - 0.4) / 0.2 * 12.0;
  if (t < 0.8)
    return 4.0;
  if (t < 0.9)
    return 24.0;
  return 0.0;
}

static const fh_input_t inputs[] = {
    FH_INPUT(equivalentvoltage, FH_FUNCTION(voltage_profile)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(omega),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Systems.Propulsion.Motors.FirstOrderVoltageToSpeed",
      .stop_s = 1.3,
      .step_s = 0.001,
      .print_s = 0.05,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
