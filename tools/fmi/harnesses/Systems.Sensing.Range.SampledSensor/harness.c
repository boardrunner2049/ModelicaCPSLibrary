/* FMI harness: Systems.Sensing.Range.SampledSensor (L1 sampled rangefinder).
 *
 * A rangefinder descends over flat ground (plane z = 0, normal Down): height
 * h = 10 - 2.4 t m over 4 s, the ray tilted 36.87 deg forward,
 * direction (0.6, 0, 0.8), so the slant range is h / 0.8 = 12.5 .. 0.5 m.
 * world.toml: valid window [1, 12] m, bias 5 cm, noise 2 cm + 0.2 % of range,
 * 1 cm LSB, 2 mm/K at 10 K above reference (+2 cm). Scenario events:
 *   t < 0.17 s       range > 12 m          -> TooFar (4)
 *   [1.0, 1.2) s     returnAvailable false -> NoReturn (2)
 *   [1.5, 1.6) s     deviceError           -> DeviceError (5)
 *   [2.0, 2.2) s     ray tilted upward (0.6, 0, -0.8), away from the ground -> NoReturn (2)
 *   t > 3.83 s       range < 1 m           -> TooClose (3)
 * Default 20 ms epochs from 0, latency 0; a sample on a communication point
 * acquires the inputs held over the 5 ms step before it. signalQuality 0.8.
 * Not exercised: a ray parallel to the plane (Tests.RangePhysics covers it).
 * Before Rumoca 13724e65 the parallelRay relation stayed frozen after an input
 * write, so switching the ray to (1, 0, 0) mid-run divided by n.d = 0. */
#include "fire_harness.h"

static double down(double t) { return -10.0 + 2.4 * t; }
static double ray_z(double t) { return t >= 2.0 && t < 2.2 ? -0.8 : 0.8; }

static const fh_input_t inputs[] = {
    FH_INPUT(position_w_1, FH_CONSTANT(0.0)),
    FH_INPUT(position_w_2, FH_CONSTANT(0.0)),
    FH_INPUT(position_w_3, FH_FUNCTION(down)),
    FH_INPUT(direction_w_1, FH_CONSTANT(0.6)),
    FH_INPUT(direction_w_2, FH_CONSTANT(0.0)),
    FH_INPUT(direction_w_3, FH_FUNCTION(ray_z)),
    FH_INPUT(planenormal_w_1, FH_CONSTANT(0.0)),
    FH_INPUT(planenormal_w_2, FH_CONSTANT(0.0)),
    FH_INPUT(planenormal_w_3, FH_CONSTANT(1.0)),
    FH_INPUT(planeoffset, FH_CONSTANT(0.0)),
    FH_INPUT(sensortemperature, FH_CONSTANT(303.15)),
    FH_INPUT_BOOL(returnavailable, FH_PULSE(1.0, 0.2, 1.0, 0.0)),
    FH_INPUT_BOOL(deviceerror, FH_PULSE(1.5, 0.1, 0.0, 1.0)),
    FH_INPUT(signalquality, FH_CONSTANT(0.8)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(sampletime), FH_OUTPUT_INT(sequence), FH_OUTPUT(distance),
    FH_OUTPUT_BOOL(valid), FH_OUTPUT_INT(status),   FH_OUTPUT(quality),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Systems.Sensing.Range.SampledSensor",
      .stop_s = 4.0,
      .step_s = 0.005,
      .print_s = 0.25,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
