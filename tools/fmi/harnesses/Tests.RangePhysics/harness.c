/* FMI harness: Tests.RangePhysics (closed test model, no inputs).
 *
 * Scenario: the test itself, 1 s at its 2 ms interval: L0 ray/plane ranges
 * (tilted, scaled normal, parallel, behind, too close, too far) and delayed L1
 * packets with no-return and device-error statuses. */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.RangePhysics",
      .stop_s = 1,
      .step_s = 0.002,
      .print_s = 0.1,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
