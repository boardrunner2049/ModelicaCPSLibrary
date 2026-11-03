/* FMI harness: Examples.SpiTransfer (closed example model, no inputs).
 *
 * Scenario: the example itself. An SpiTransfer master (1 MHz, mode 0, MSB
 * first) asserts CS on [1, 25) us, submits request 1 (0xA5 = 165) at 2 us and
 * request 2 (0x3C = 60) at 14 us; a ByteSlave answers 0x5A = 90 to both. Run
 * to the experiment's 30 us at a 50 ns communication step (a tenth of an SCLK
 * half period, so every clock edge and MOSI bit shows in the trace).
 *
 * checks.toml establishes: CS timing; each byte takes 16 half periods (8 us)
 * and retires with status 1; the master receives 90 and the slave receives
 * 165 then 60 (receivedCount 1 then 2); SCLK toggles at the right instants,
 * idles low; MOSI carries the MSB-first bit pattern; MISO goes Hi-Z after CS
 * release. */
#include "fire_harness.h"

static const fh_output_t outputs[] = {
    FH_OUTPUT_INT(receivedcommand),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Examples.SpiTransfer",
      .stop_s = 30e-6,
      .step_s = 50e-9,
      .print_s = 2e-6,
      .inputs = NULL,
      .input_count = 0,
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
