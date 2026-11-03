/* FMI harness: Tests.ContactPhysics.
 *
 * Closed test model (no inputs): four CompliantPointLeg instances with
 * prescribed kinematics -- an inclined plane with Coulomb-capped friction, a
 * rising leg whose damper would pull, a leg with an offset tip and body rate
 * (omega x r), and a leg separating at 0.2 m/s. The scenario is the run
 * itself: 0.1 s (the model's StopTime) at 1 ms. checks.toml restates the
 * model's asserts as closed forms and adds the separation timing (force
 * clips to zero at 0.03 s, contact ends at 0.05 s). */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.ContactPhysics",
      .stop_s = 0.1,
      .step_s = 0.001,
      .print_s = 0.01,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
