/* FMI harness: Adapters.VirtualIO.PwmInput.
 *
 * Four PWM channels; the output is clamp((pw - pwmMin)/(pwmMax - pwmMin), 0, 1),
 * stateless. world.toml gives each channel its own range: ch1, ch2 1000..2000
 * us (defaults), ch3 1100..1900 us, ch4 1050..2050 us. Scenario (3 s, 1 ms):
 *   ch1  ramp 800 -> 2200 us over [0.5, 2.5] s: crosses the lower clamp, the
 *        whole linear range and the upper clamp;
 *   ch2  staircase 1000, 1250, 1500, 1750, 2000, 0 us (exact endpoints and a
 *        "no signal" 0 us pulse that must read 0, not negative);
 *   ch3  sine 1500 +/- 700 us at 1 Hz: saturates at both peaks, earlier than
 *        with the default range;
 *   ch4  constant 1500 us (0.45 with its range).
 * checks.toml establishes the linear map at known points, both clamps, the
 * output bound [0,1] over the whole run, per-channel ranges (honoured and not
 * transposed) and channel independence (ch4 constant at every instant).
 * Input/trace timing: the row at t shows the input held from t - 1 ms. */
#include "fire_harness.h"

static double staircase(double t) {
  if (t < 0.5)
    return 1000.0;
  if (t < 1.0)
    return 1250.0;
  if (t < 1.5)
    return 1500.0;
  if (t < 2.0)
    return 1750.0;
  if (t < 2.5)
    return 2000.0;
  return 0.0;
}

static const fh_input_t inputs[] = {
    FH_INPUT(pulsewidth_us_1, FH_RAMP(0.5, 800.0, 2.5, 2200.0)),
    FH_INPUT(pulsewidth_us_2, FH_FUNCTION(staircase)),
    FH_INPUT(pulsewidth_us_3, FH_SINE(1500.0, 700.0, 1.0, 0.0)),
    FH_INPUT(pulsewidth_us_4, FH_CONSTANT(1500.0)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(normalizedcommand_1),
    FH_OUTPUT(normalizedcommand_2),
    FH_OUTPUT(normalizedcommand_3),
    FH_OUTPUT(normalizedcommand_4),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Adapters.VirtualIO.PwmInput",
      .stop_s = 3.0,
      .step_s = 0.001,
      .print_s = 0.25,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
