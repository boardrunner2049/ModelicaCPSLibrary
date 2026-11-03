/* FMI harness: Physical.Electrical.Drives.ESC.AverageVoltage.
 *
 * Algebraic unipolar voltage ESC: modulation = clamp(escCommand, 0, 1),
 * equivalentVoltage = modulation * dcVoltage (dcVoltage >= 0 is asserted;
 * a negative supply must fail the step, exercised outside this scenario).
 *
 * escCommand: 0 until 0.1 s, ramp -0.5 -> 1.5 over 0.1-0.5 s (crossing the
 * lower clamp, the linear range and the upper clamp), 0.25 on 0.5-0.7 s,
 * -1 (lower clamp) on 0.7-0.8 s, 0.6 afterwards.
 * dcVoltage: 16 V until 0.6 s, sags to 8 V on 0.6-0.8 s, ramps 8 -> 0 V on
 * 0.8-0.9 s, 0 V (dead bus) from 0.9 s. */
#include "fire_harness.h"

static double command_profile(double t) {
  if (t < 0.1)
    return 0.0;
  if (t < 0.5)
    return -0.5 + (t - 0.1) / 0.4 * 2.0;
  if (t < 0.7)
    return 0.25;
  if (t < 0.8)
    return -1.0;
  return 0.6;
}

static double supply_profile(double t) {
  if (t < 0.6)
    return 16.0;
  if (t < 0.8)
    return 8.0;
  if (t < 0.9)
    return 8.0 - (t - 0.8) / 0.1 * 8.0;
  return 0.0;
}

static const fh_input_t inputs[] = {
    FH_INPUT(esccommand, FH_FUNCTION(command_profile)),
    FH_INPUT(dcvoltage, FH_FUNCTION(supply_profile)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(modulation),
    FH_OUTPUT(equivalentvoltage),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Physical.Electrical.Drives.ESC.AverageVoltage",
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
