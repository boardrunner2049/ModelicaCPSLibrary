/* FMI harness: Physical.Mechanical.Contact.ContactMode.
 *
 * contact = gap <= 0 (closed surface: touching counts as contact). The gap
 * profile starts at its start value 0 (initially touching), lifts to +5 mm,
 * ramps down through zero (crossing at 0.30 s), steps to -5 mm, holds exactly 0
 * (touching, not penetrating), steps to -1 mm and ramps up through zero
 * (crossing at 0.70 s) to +2 mm,
 * then a 1 um penetration pulse at 0.85 s. checks.toml pins the flag at
 * every phase and the exact switching samples. */
#include "fire_harness.h"

static double gap_profile(double t) {
  if (t < 0.1)
    return 0.005;
  if (t < 0.4)
    return 0.005 - (t - 0.1) / 0.2 * 0.005; /* 0 at 0.30, -2.5 mm at 0.4 */
  if (t < 0.5)
    return -0.005;
  if (t < 0.6)
    return 0.0;
  if (t < 0.8)
    return (t - 0.6) / 0.1 * 0.001 - 0.001; /* 0 at 0.70 */
  if (t >= 0.85 && t < 0.86)
    return -1e-6;
  return 0.002;
}

static const fh_input_t inputs[] = {
    FH_INPUT(gap, FH_FUNCTION(gap_profile)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT_BOOL(contact),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Physical.Mechanical.Contact.ContactMode",
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
