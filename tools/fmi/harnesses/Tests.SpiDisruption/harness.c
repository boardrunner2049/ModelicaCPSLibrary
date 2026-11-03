/* FMI harness: Tests.SpiDisruption (closed test model, no inputs).
 *
 * Scenario: the test itself, three 1 MHz mode-0 links side by side:
 *   rejected  - request 1 at 2 us with CS never asserted (must be refused);
 *   cancelled - CS on [1, 6) us drops mid-byte during request 1 (abort), then
 *               CS on [11, 24) us carries request 2 (0x35 = 53) to completion;
 *   disturbed - request 1 (0x96 = 150) with MOSI inverted on [2.25, 2.75) us
 *               around the first sampling edge (slave must get 0x16 = 22),
 *               while the MISO path (0x69 = 105) is unaffected.
 * Run to the experiment's 26 us at a 50 ns step. The model's own asserts
 * (at 3, 7, 11, 22 us) abort the FMU if violated; checks.toml restates them
 * on the trace and adds timing of the abort, recovery and bus idling. */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.SpiDisruption",
      .stop_s = 26e-6,
      .step_s = 50e-9,
      .print_s = 2e-6,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
