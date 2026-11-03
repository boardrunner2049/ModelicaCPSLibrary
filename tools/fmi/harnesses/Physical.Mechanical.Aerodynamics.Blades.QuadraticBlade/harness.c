/* FMI harness: Physical.Mechanical.Aerodynamics.Blades.QuadraticBlade.
 *
 * Static quadratic rotor: force_r = {0,0,kT w^2}, loadTorque = kQ w^2,
 * moment_r = {0,0,-spinSign kQ w^2}. world.toml overrides kQ = 1.5e-7 and
 * spinSign = -1 (clockwise rotor), so the reaction moment is +z; kT stays
 * at its 1e-5 default. omega (>= 0 is asserted; reverse rotation must
 * fail the step, exercised outside this scenario):
 *   0.0-0.1  0            at rest
 *   0.1-0.5  0 -> 1000    ramp (quadratic growth of thrust and torque)
 *   0.5-0.7  400          step down
 *   0.7-1.0  600 + 300 sin(2 pi 5 t)
 * airVelocity_r and omegaBody_r are driven with nonzero sines throughout:
 * the static model must ignore them. */
#include "fire_harness.h"

#include <math.h>

#define PI 3.14159265358979323846

static double omega_profile(double t) {
  if (t < 0.1)
    return 0.0;
  if (t < 0.5)
    return (t - 0.1) / 0.4 * 1000.0;
  if (t < 0.7)
    return 400.0;
  return 600.0 + 300.0 * sin(2.0 * PI * 5.0 * t);
}

static const fh_input_t inputs[] = {
    FH_INPUT(omega, FH_FUNCTION(omega_profile)),
    FH_INPUT(airvelocity_r_1, FH_SINE(3.0, 2.0, 1.0, 0.0)),
    FH_INPUT(airvelocity_r_2, FH_SINE(-1.0, 4.0, 2.0, 0.5)),
    FH_INPUT(airvelocity_r_3, FH_SINE(5.0, 6.0, 3.0, 1.0)),
    FH_INPUT(omegabody_r_1, FH_SINE(0.2, 1.0, 1.5, 0.0)),
    FH_INPUT(omegabody_r_2, FH_SINE(-0.3, 2.0, 2.5, 0.3)),
    FH_INPUT(omegabody_r_3, FH_SINE(0.5, 3.0, 0.7, 0.9)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(force_r_3),
    FH_OUTPUT(loadtorque),
    FH_OUTPUT(moment_r_3),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Physical.Mechanical.Aerodynamics.Blades.QuadraticBlade",
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
