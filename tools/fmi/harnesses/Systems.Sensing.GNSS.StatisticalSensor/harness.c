/* FMI harness: Systems.Sensing.GNSS.StatisticalSensor (G1 statistical PVT).
 *
 * Antenna truth as in the Systems.Sensing.GNSS.Sensor harness: a circle of
 * radius 20 m at 0.5 rad/s (North -> East) climbing 1 m/s from 10 m height,
 *   p = (20 cos wt, 20 sin wt, -10 - t),  v = (-10 sin wt, 10 cos wt, -1).
 * world.toml: fixed NED biases, correlated per-epoch noise (noiseFactor),
 * 60 s Gauss-Markov biases (biasFactor), default 0.2 s epochs from t = 0,
 * latency 0. fixAvailable drops on [10, 12) s: those epochs are delivered
 * invalid with fixType 0 and a zero payload; the epoch counter keeps running.
 * A sample on a communication point acquires the inputs held over the 10 ms
 * step before it, so the outage covers the epochs 10.2 .. 12.0 s.
 * checks.toml: epochs, validity, covariance, position/velocity consistency
 * with the truth geometry within the configured noise. */
#include "fire_harness.h"

#include <math.h>

#define RADIUS 20.0
#define OMEGA 0.5

static double pn(double t) { return RADIUS * cos(OMEGA * t); }
static double pe(double t) { return RADIUS * sin(OMEGA * t); }
static double pd(double t) { return -10.0 - 1.0 * t; }
static double vn(double t) { return -RADIUS * OMEGA * sin(OMEGA * t); }
static double ve(double t) { return RADIUS * OMEGA * cos(OMEGA * t); }

static const fh_input_t inputs[] = {
    FH_INPUT(position_w_1, FH_FUNCTION(pn)),
    FH_INPUT(position_w_2, FH_FUNCTION(pe)),
    FH_INPUT(position_w_3, FH_FUNCTION(pd)),
    FH_INPUT(velocity_w_1, FH_FUNCTION(vn)),
    FH_INPUT(velocity_w_2, FH_FUNCTION(ve)),
    FH_INPUT(velocity_w_3, FH_CONSTANT(-1.0)),
    FH_INPUT_BOOL(fixavailable, FH_PULSE(10.0, 2.0, 1.0, 0.0)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(sampletime), FH_OUTPUT_INT(sequence), FH_OUTPUT_BOOL(valid), FH_OUTPUT_INT(fixtype),
    FH_OUTPUT(position_1), FH_OUTPUT(position_2), FH_OUTPUT(position_3), FH_OUTPUT(velocity_3),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Systems.Sensing.GNSS.StatisticalSensor",
      .stop_s = 20.0,
      .step_s = 0.01,
      .print_s = 2.0,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
