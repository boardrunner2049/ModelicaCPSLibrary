/* FMI harness: Tests.ContactWrenchCG.
 *
 * A closed test model: a 2 kg MultirotorPlant (aggregate CG shifted to
 * {0.04,-0.03,0.02} in chassis axes) resting on its four compliant legs on
 * ground at Down = 1.2 m, rotors off, with an external body moment
 * {0.03W, 0.04W, 0} (W = 2 g) that exactly cancels the contact moment about
 * the shifted CG. p_start is chosen so each leg is compressed W/(4k): the
 * vehicle starts in exact static equilibrium. Equation-section asserts require
 * the contact force (0,0,-W), the contact moment (-0.03W,-0.04W,0) and zero
 * motion at every instant. There are no inputs; the scenario runs 1 s (5x the
 * model's StopTime) at 1 ms to show the equilibrium is held, not just met.
 *
 * checks.toml establishes: total contact force equals the weight and points
 * Up, the contact moment about the combined CG includes the CG shift exactly
 * once, equal leg loads W/4 and compressions W/(4k), and no drift of
 * position, velocity or attitude rate.
 *
 * Two Rumoca defects blocked this harness when it was written, both fixed
 * as of a4dd197b: initialization-time contact relations, and products of
 * inertia about the chassis origin instead of the combined CG. Per-leg lever
 * arms and moments are checked individually because the equal loads make the
 * sums permutation-invariant; the model's asserts forbid asymmetric
 * loading. */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.ContactWrenchCG",
      .stop_s = 1.0,
      .step_s = 0.001,
      .print_s = 0.1,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
