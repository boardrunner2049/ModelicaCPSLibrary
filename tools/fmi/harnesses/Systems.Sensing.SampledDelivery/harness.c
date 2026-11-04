/* FMI harness: Systems.Sensing.SampledDelivery (atomic acquisition + delivery queue).
 *
 * Default clock: samplePeriod 0.1 s from startTime 0, latency 0, n = 1 (the
 * clock parameters are left at their defaults: with Rumoca 6058a2bb an FMI
 * override of samplePeriod/startTime changes the reported sampleTime but not
 * the sample() events, which keep the compiled 0.1 s / 0 s; latencySamples and
 * n are structural).
 * Inputs: u = 1 + 2 t (a ramp, so a delivered value identifies its acquisition
 * time); acquisitionStatus 3 with acquisitionValid true, except status 7 and
 * valid false on [0.5, 0.7) s. A sample on a communication point acquires the
 * inputs held over the preceding 10 ms step (the left limit), so the invalid
 * window is acquired at 0.6 and 0.7 s and y = 1 + 2 (sampleTime - 0.01). */
#include "fire_harness.h"

static double ramp(double t) { return 1.0 + 2.0 * t; }

static const fh_input_t inputs[] = {
    FH_INPUT(u_1, FH_FUNCTION(ramp)),
    FH_INPUT_INT(acquisitionstatus, FH_PULSE(0.5, 0.2, 3.0, 7.0)),
    FH_INPUT_BOOL(acquisitionvalid, FH_PULSE(0.5, 0.2, 1.0, 0.0)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(y_1),        FH_OUTPUT_INT(status),   FH_OUTPUT_BOOL(valid),
    FH_OUTPUT(sampletime), FH_OUTPUT(deliverytime), FH_OUTPUT_INT(sequence),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Systems.Sensing.SampledDelivery",
      .stop_s = 2.0,
      .step_s = 0.01,
      .print_s = 0.1,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
