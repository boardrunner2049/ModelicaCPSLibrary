/* FMI harness: Tests.GnssStatistical (closed test model, no inputs).
 *
 * Scenario: the test itself, 4 s at its 2 ms interval: G0 and delayed G1
 * receivers with a fix outage, seed replay and independent streams, and the
 * SensorSuite GNSS lever arm. */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.GnssStatistical",
      .stop_s = 4,
      .step_s = 0.002,
      .print_s = 0.5,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
