/* FMI harness: Physical.Mechanical.Contact.TangentialFriction.
 *
 * |F| = min(c_t*|v|, mu*N) if active and N > 0 else 0, F opposite v (with
 * v/max(|v|, 1e-6) as direction), power = F.v. c_t = 25 N.s/m, mu = 0.6.
 *   0.00-0.10  active, N = 20 N, at rest: no force
 *   0.10-0.20  v = (0.2,0,0): viscous 5 N < mu*N = 12 N
 *   0.20-0.30  v = (0.3,-0.4,0), |v| = 0.5: viscous 12.5 N capped at 12 N
 *   0.30-0.40  v = (0.1,0.2,-0.2), |v| = 0.3: viscous 7.5 N, all three axes
 *   0.40-0.50  N = -5 N (tensile normal load): no friction
 *   0.50-0.55  N = 20 N again, same v: 7.5 N
 *   0.55-0.58  active = false alone: no friction
 *   0.58-0.60  active = true alone: 7.5 N again
 *   0.60-0.80  active again, v = (1e-8,0,0) below the 1e-6 m/s regularization
 *              speed: F = -c_t*|v|^2/1e-6 = -2.5e-9 N, continuous through 0
 *   0.80-1.00  v = (0,-1,0), N ramps 0 -> 30 N: Coulomb-capped 0.6*N */
#include "fire_harness.h"

static double active_profile(double t) { return (t >= 0.55 && t < 0.58) ? 0.0 : 1.0; }

static double n_profile(double t) {
  if (t >= 0.4 && t < 0.5)
    return -5.0;
  if (t >= 0.8)
    return 30.0 * (t - 0.8) / 0.2;
  return 20.0;
}

static double v1(double t) {
  if (t >= 0.1 && t < 0.2)
    return 0.2;
  if (t >= 0.2 && t < 0.3)
    return 0.3;
  if (t >= 0.3 && t < 0.6)
    return 0.1;
  if (t >= 0.6 && t < 0.8)
    return 1e-8;
  return 0.0;
}

static double v2(double t) {
  if (t >= 0.2 && t < 0.3)
    return -0.4;
  if (t >= 0.3 && t < 0.6)
    return 0.2;
  if (t >= 0.8)
    return -1.0;
  return 0.0;
}

static double v3(double t) { return (t >= 0.3 && t < 0.6) ? -0.2 : 0.0; }

static const fh_input_t inputs[] = {
    FH_INPUT_BOOL(active, FH_FUNCTION(active_profile)),
    FH_INPUT(normalforce, FH_FUNCTION(n_profile)),
    FH_INPUT(velocitytangential_w_1, FH_FUNCTION(v1)),
    FH_INPUT(velocitytangential_w_2, FH_FUNCTION(v2)),
    FH_INPUT(velocitytangential_w_3, FH_FUNCTION(v3)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(force_w_1),
    FH_OUTPUT(force_w_2),
    FH_OUTPUT(force_w_3),
    FH_OUTPUT(power),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Physical.Mechanical.Contact.TangentialFriction",
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
