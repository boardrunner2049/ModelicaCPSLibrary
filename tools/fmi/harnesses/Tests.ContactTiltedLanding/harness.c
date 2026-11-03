/* FMI harness: Tests.ContactTiltedLanding.
 *
 * Closed test model (no inputs): a 1.5 kg MultirotorPlant, rotors off,
 * dropped from CG Down -0.3 m with a 0.16 rad roll onto level ground; the two
 * right-hand legs touch first and the contact moments roll it level. The
 * scenario is the run itself: 3 s (the model's StopTime) at 1 ms.
 * checks.toml: closed-form free fall and touchdown, energy never increasing,
 * the settled static state, and the transient against native rumoca sim. */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.ContactTiltedLanding",
      .stop_s = 3.0,
      .step_s = 0.001,
      .print_s = 0.3,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
