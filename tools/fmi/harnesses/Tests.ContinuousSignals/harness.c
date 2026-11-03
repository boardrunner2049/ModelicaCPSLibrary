/* FMI harness: Tests.ContinuousSignals (closed test model, no inputs).
 *
 * Scenario: a GNSS sensor, a barometer and a six-channel PwmInput are fed
 * time functions inside the model (position {t,2t,-3t}, pressure
 * 101325 - 10t, temperature 288.15 + t, altitude 3t, PWM 1000 + 1000t us).
 * Run to 0.98 s (experiment: 0.035 s) at the experiment's 0.7 ms interval so
 * the PWM ramp sweeps almost the whole range without reaching the clamp that
 * the in-model assert forbids. world.toml sets the two biases the asserts
 * leave free: GNSS velocityBias {0.25,-0.5,0.125} and temperatureBias -1.5 K.
 *
 * checks.toml establishes: every output tracks its input continuously (values
 * at the off-grid instant 0.0217 s, at 0.49 s and at the end; no
 * sample-and-hold), NED/up sign conventions (down -3t vs altitude +3t), the
 * per-axis velocity bias and the temperature bias are applied with sign and
 * not transposed, and PWM normalization (pw - 1000)/1000 on all six channels.
 *
 * No outputs exist, so the console prints only time; values are in the trace. */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.ContinuousSignals",
      .stop_s = 0.98,
      .step_s = 0.0007,
      .print_s = 0.07,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
