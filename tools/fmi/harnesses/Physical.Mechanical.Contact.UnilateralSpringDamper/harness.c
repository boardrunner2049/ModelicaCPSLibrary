/* FMI harness: Physical.Mechanical.Contact.UnilateralSpringDamper.
 *
 * normalForce = if active and gap <= 0 then max(0, k*max(0,-gap) - c*v_n)
 * else 0, k = 3000 N/m, c = 150 N.s/m, v_n > 0 separating. Phases:
 *   0.00-0.10  active, 10 mm above the surface: no force
 *   0.10-0.30  2 mm in, at rest: spring only, 6 N
 *   0.30-0.40  approaching at 0.1 m/s: 6 + 15 = 21 N
 *   0.40-0.50  separating at 0.1 m/s: 6 - 15 < 0 -> clipped to 0 (no pull)
 *   0.50-0.60  separating at 0.02 m/s: 6 - 3 = 3 N
 *   0.60-0.70  exactly touching (gap 0), approaching 0.1 m/s: damper only, 15 N
 *   0.70-0.75  2 mm in, at rest: 6 N again
 *   0.75-0.80  active = false (alone, nothing else changes): no force
 *   0.80-1.00  active again; gap ramps -2 mm -> +2 mm at v_n = +0.02 m/s
 *              (crossing 0.90 s): F = max(0, k*pen - 3)
 *   1.00-1.20  outside the surface, approaching 1 m/s: no force (gap > 0) */
#include "fire_harness.h"

static double active_profile(double t) { return (t >= 0.75 && t < 0.8) ? 0.0 : 1.0; }

static double gap_profile(double t) {
  if (t < 0.1)
    return 0.01;
  if (t < 0.6)
    return -0.002;
  if (t < 0.7)
    return 0.0;
  if (t < 0.8)
    return -0.002;
  if (t < 1.0)
    return -0.002 + 0.02 * (t - 0.8);
  return 0.002;
}

static double vn_profile(double t) {
  if (t >= 0.3 && t < 0.4)
    return -0.1;
  if (t >= 0.4 && t < 0.5)
    return 0.1;
  if (t >= 0.5 && t < 0.6)
    return 0.02;
  if (t >= 0.6 && t < 0.7)
    return -0.1;
  if (t >= 0.8 && t < 1.0)
    return 0.02;
  if (t >= 1.0)
    return -1.0;
  return 0.0;
}

static const fh_input_t inputs[] = {
    FH_INPUT_BOOL(active, FH_FUNCTION(active_profile)),
    FH_INPUT(gap, FH_FUNCTION(gap_profile)),
    FH_INPUT(normalvelocity, FH_FUNCTION(vn_profile)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(penetration),
    FH_OUTPUT(normalforce),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Physical.Mechanical.Contact.UnilateralSpringDamper",
      .stop_s = 1.2,
      .step_s = 0.001,
      .print_s = 0.05,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
