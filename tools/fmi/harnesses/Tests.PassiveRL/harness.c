/* FMI harness: Tests.PassiveRL.
 *
 * Closed bench (no inputs): SeriesRL (R = 2 ohm, L = 20 mH, tau = 10 ms,
 * i_start = -1 A) on a signal voltage of -4 V until 0.1 s and +4 V after,
 * so the current relaxes from -1 A to -2 A and then reverses through zero
 * to +2 A. A separate state integrates v*i - R i^2 and must equal the
 * stored magnetic energy L i^2/2. The model's own asserts (closed-form
 * current within 1e-5, energy balance within 1e-7) run inside the FMU.
 * Communication step 0.1 ms matches the experiment Interval; checks.toml
 * compares against the closed form and Kirchhoff's voltage law per row
 * (native rumoca sim agrees to 2e-9 A). */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.PassiveRL",
      .stop_s = 0.2,
      .step_s = 0.0001,
      .print_s = 0.02,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
