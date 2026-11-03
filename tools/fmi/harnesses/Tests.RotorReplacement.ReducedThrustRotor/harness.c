/* FMI harness: Tests.RotorReplacement.ReducedThrustRotor.
 *
 * QuadraticBlade with the replacement coefficient kT = 0.5e-5 bound by the
 * extends modifier (half the library default); kQ = 1e-7 and spinSign = +1
 * stay at their defaults (no world.toml overrides, so the modifier binding
 * itself is under test). omega (>= 0 asserted; exercised outside this
 * scenario):
 *   0.0-0.1  0
 *   0.1-0.3  100 rad/s   the RotorReplacement operating point: 0.05 N per
 *                        rotor, 0.2 N for the quad that test asserts on
 *   0.3-0.7  ramp 100 -> 900 rad/s
 *   0.7-0.9  900 rad/s
 *   0.9-1.0  0           stop: wrench returns to zero
 * airVelocity_r / omegaBody_r carry nonzero constants the static model must
 * ignore. */
#include "fire_harness.h"

static double omega_profile(double t) {
  if (t < 0.1)
    return 0.0;
  if (t < 0.3)
    return 100.0;
  if (t < 0.7)
    return 100.0 + (t - 0.3) / 0.4 * 800.0;
  if (t < 0.9)
    return 900.0;
  return 0.0;
}

static const fh_input_t inputs[] = {
    FH_INPUT(omega, FH_FUNCTION(omega_profile)),
    FH_INPUT(airvelocity_r_1, FH_CONSTANT(4.0)),
    FH_INPUT(airvelocity_r_2, FH_CONSTANT(-2.0)),
    FH_INPUT(airvelocity_r_3, FH_CONSTANT(7.0)),
    FH_INPUT(omegabody_r_1, FH_CONSTANT(0.5)),
    FH_INPUT(omegabody_r_2, FH_CONSTANT(-1.5)),
    FH_INPUT(omegabody_r_3, FH_CONSTANT(2.0)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(force_r_3),
    FH_OUTPUT(loadtorque),
    FH_OUTPUT(moment_r_3),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.RotorReplacement.ReducedThrustRotor",
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
