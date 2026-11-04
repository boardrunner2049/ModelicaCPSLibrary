/* FMI harness across models: the Systems.Sensing.IMU fidelity ladder.
 *
 * Wrapper.mo drives the ideal Sensor and the six sampled fidelities
 * (StatisticalSensor, DigitalSensor, DeviceSensor, ErrorSensor,
 * ElectromechanicalSensor, AcousticSensor) with one motion
 * (lib/imu_motion.h: level and at rest for 1 s, then 0.5 rad/s roll), on a
 * shared 2 ms output clock with one sample of latency. The errored fidelities
 * share one stochastic error set and RNG stream, so checks.toml compares them
 * sample by sample: equal where the models must be equal (ErrorSensor with no
 * injected error, AcousticSensor with no pressure, both against DeviceSensor),
 * and within the readout/physics budget otherwise. 0.25 ms communication step
 * for the electromechanical gyros (see that model's harness). */
#include "fire_harness.h"
#include "imu_motion.h"

static const fh_input_t inputs[] = {
    IMU_MOTION_INPUTS,
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Systems.Sensing.IMU.FidelityComparison",
      .stop_s = 3.0,
      .step_s = 0.00025,
      .print_s = 0.5,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
