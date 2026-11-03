/* FMI harness: Tests.SpiSignals (closed test model, no inputs).
 *
 * Scenario: the test itself. Eight 1 MHz master/slave pairs cover all four
 * (CPOL, CPHA) modes with MSB-first (pairs 1-4) and LSB-first (pairs 5-8)
 * bit order. CS is held on [1, 65) us; requests 1..5 are submitted at 2, 14,
 * 26, 38, 50 us with data 150, 150 (repeated byte), 53, 0, 255; every slave
 * answers 105. Run to the experiment's 70 us at a 50 ns step. The model's own
 * asserts (11, 23, 35, 47, 59, 66 us) abort the FMU if violated. */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.SpiSignals",
      .stop_s = 70e-6,
      .step_s = 50e-9,
      .print_s = 5e-6,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
