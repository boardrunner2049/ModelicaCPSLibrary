/* FMI harness: Systems.Sensing.IMU.DigitalSensor (I1: deterministic device readout).
 *
 * Motion (lib/imu_motion.h): level and at rest for 1 s, then 0.5 rad/s roll.
 * Readout (world.toml): first-order analog response tau = 2 ms, ADC with
 * symmetric clipping at fullScale (16 g, 2000 deg/s defaults) and LSB
 * 0.01 m/s2 / 0.001 rad/s, a 3 ms matched-pole IIR at the 1 ms ADC rate, no
 * calibration or noise (I1 asserts that none is configured).
 * equivalentError (the vulnerability input, added before the analog stage):
 *   accel x +0.5 m/s2 on [0.5, 0.7) s  -> passes through the readout
 *   gyro z  +50 rad/s on [0.3, 0.35) s  -> exceeds fullScale, clipped to 34.9066
 * Everything is deterministic, so checks.toml uses exact ADC codes where the
 * signals have settled, and a lag-compensated comparison while rolling. */
#include "fire_harness.h"
#include "imu_motion.h"

static const fh_input_t inputs[] = {
    IMU_MOTION_INPUTS,
    FH_INPUT(sensortemperature, FH_CONSTANT(293.15)),
    FH_INPUT(equivalenterror_1, FH_PULSE(0.5, 0.2, 0.0, 0.5)),
    FH_INPUT(equivalenterror_2, FH_CONSTANT(0.0)),
    FH_INPUT(equivalenterror_3, FH_CONSTANT(0.0)),
    FH_INPUT(equivalenterror_4, FH_CONSTANT(0.0)),
    FH_INPUT(equivalenterror_5, FH_CONSTANT(0.0)),
    FH_INPUT(equivalenterror_6, FH_PULSE(0.3, 0.05, 0.0, 50.0)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(sampletime),     FH_OUTPUT(acceleration_1), FH_OUTPUT(acceleration_2),
    FH_OUTPUT(acceleration_3), FH_OUTPUT(gyro_1),         FH_OUTPUT(gyro_3),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Systems.Sensing.IMU.DigitalSensor",
      .stop_s = 3.0,
      .step_s = 0.0004,
      .print_s = 0.25,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
