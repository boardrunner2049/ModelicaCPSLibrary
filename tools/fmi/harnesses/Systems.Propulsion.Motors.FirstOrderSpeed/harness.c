/* FMI harness: Systems.Propulsion.Motors.FirstOrderSpeed.
 *
 * First-order speed lag omega' = (omegaMax*clamp(demand,0,1) - omega)/tau with
 * tau = 20 ms, omegaMax = 1000 rad/s. The demand profile exercises the linear
 * range (step to 0.5), the upper clamp (ramp past 1) and the lower clamp
 * (step to -0.2); checks.toml compares against the closed-form response. */
#include "fire_harness.h"

static double demand_profile(double t) {
  if (t < 0.1)
    return 0.0;
  if (t < 0.3)
    return 0.5;
  if (t < 0.4)
    return 0.5 + (t - 0.3) / 0.1 * 0.7; /* ramp to 1.2, saturating at 1 */
  if (t < 0.6)
    return 1.2;
  return -0.2;
}

static const fh_input_t inputs[] = {
    FH_INPUT(demand, FH_FUNCTION(demand_profile)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(omega),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Systems.Propulsion.Motors.FirstOrderSpeed",
      .stop_s = 1.0,
      .step_s = 0.001,
      .print_s = 0.05,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
