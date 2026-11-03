/* FMI harness: Systems.Power.DCLink (through Wrapper.mo).
 *
 * Shared DC bus: ideal source -> Rs = 20 mOhm -> bus with C = 2 mF
 * (tau = Rs*C = 40 us), starting charged at 16 V, loaded by an ideal current
 * sink. Scenario (1 us communication step, 3 ms):
 *   0.2 ms  load steps 0 -> 20 A        bus sags by Rs*i = 0.4 V with tau
 *   0.6 ms  source steps 16 -> 12 V      bus follows to 11.6 V
 *   1.0 ms  load reverses to -10 A       regeneration: bus rises above the
 *                                        source (12.2 V), source absorbs power
 *   1.5-2.5 ms source ramps 12 -> 20 V   bus tracks with lag slope*tau =
 *                                        0.32 V; capacitor current C*dV/dt
 * checks.toml compares the bus voltage with the closed-form first-order
 * response (evaluated exactly over the zero-order-hold steps), and checks
 * KCL, steady power balance source = loss + load, stored energy C v^2/2 and
 * the sign of power flow during regeneration. */
#include "fire_harness.h"

static double source_profile(double t) {
  if (t < 0.6e-3 - 1e-12)
    return 16.0;
  if (t < 1.5e-3 - 1e-12)
    return 12.0;
  if (t < 2.5e-3 - 1e-12)
    return 12.0 + (t - 1.5e-3) / 1e-3 * 8.0;
  return 20.0;
}

static double load_profile(double t) {
  if (t < 0.2e-3 - 1e-12)
    return 0.0;
  if (t < 1.0e-3 - 1e-12)
    return 20.0;
  return -10.0;
}

static const fh_input_t inputs[] = {
    FH_INPUT(sourcevoltage, FH_FUNCTION(source_profile)),
    FH_INPUT(loadcurrent, FH_FUNCTION(load_profile)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(voltage),
    FH_OUTPUT(sourcecurrent),
    FH_OUTPUT(sourcepower),
    FH_OUTPUT(storedenergy),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Systems.Power.DCLink",
      .stop_s = 0.003,
      .step_s = 0.000001,
      .print_s = 0.0002,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
