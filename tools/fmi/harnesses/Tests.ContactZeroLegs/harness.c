/* FMI harness: Tests.ContactZeroLegs.
 *
 * Closed test model (no inputs): a LandingGearAssembly with nLegs = 0 placed
 * at the ground plane. The scenario is the run itself: 0.01 s (the model's
 * StopTime) at 1 ms. The empty leg arrays must export and the wrench must be
 * exactly zero. */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.ContactZeroLegs",
      .stop_s = 0.01,
      .step_s = 0.001,
      .print_s = 0.001,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
