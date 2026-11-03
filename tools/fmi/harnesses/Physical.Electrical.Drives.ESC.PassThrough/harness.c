/* FMI harness: Physical.Electrical.Drives.ESC.PassThrough.
 *
 * Stateless forwarding normalizedCommand = escCommand, valid only on [0,1]
 * (the model asserts the range; out-of-range commands must make the step
 * fail, which is exercised outside this scenario). The command walks the
 * whole valid range: rest at 0, ramp 0 -> 1 over 0.1-0.3 s, hold at the
 * upper bound 1, step down to 0.35, a 0.5 +- 0.5 sine that touches both
 * bounds exactly, and a final step to 0. */
#include "fire_harness.h"

#include <math.h>

#define PI 3.14159265358979323846

static double command_profile(double t) {
  if (t < 0.1)
    return 0.0;
  if (t < 0.3)
    return (t - 0.1) / 0.2; /* ramp 0 -> 1 */
  if (t < 0.4)
    return 1.0;
  if (t < 0.5)
    return 0.35;
  if (t < 0.9)
    return 0.5 - 0.5 * cos(2.0 * PI * 5.0 * (t - 0.5)); /* 0 .. 1 .. 0, two periods */
  return 0.0;
}

static const fh_input_t inputs[] = {
    FH_INPUT(esccommand, FH_FUNCTION(command_profile)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(normalizedcommand),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Physical.Electrical.Drives.ESC.PassThrough",
      .stop_s = 1.0,
      .step_s = 0.001,
      .print_s = 0.1,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
