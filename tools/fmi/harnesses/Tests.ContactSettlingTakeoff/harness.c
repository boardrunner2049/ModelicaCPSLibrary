/* FMI harness: Tests.ContactSettlingTakeoff.
 *
 * Closed test model (no inputs): a quad (1.5 kg) and a HexaX (2.2 kg)
 * Examples.QuadGroundCycle side by side: drop 2 cm onto four compliant legs,
 * settle, open-loop takeoff at 1.3 m g on [2, 2.7) s, motor cutoff, second
 * landing. The scenario is the run itself: 6 s (the model's StopTime) on
 * its 2 ms output grid. checks.toml compares against an independent 1-D
 * reference ODE (and native rumoca sim), and restates the model's asserts. */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.ContactSettlingTakeoff",
      .stop_s = 6.0,
      .step_s = 0.002,
      .print_s = 0.6,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
