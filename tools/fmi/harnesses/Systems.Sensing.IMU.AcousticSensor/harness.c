/* FMI harness: Systems.Sensing.IMU.AcousticSensor (legacy I3: acoustic MEMS coupling).
 *
 * Motion (lib/imu_motion.h): level and at rest for 1 s, then 0.5 rad/s roll.
 * world.toml: a 100 Hz, damping 0.1 mode on every channel, driven only on
 * accel x with static gain 1 m/Pa and read out 1:1, 0.1 ms analog response,
 * no noise. acousticPressure: a 1 Pa, 110 Hz tone on [0.3, 0.8) s, silence
 * otherwise. Off resonance by r = 1.1 the mode amplifies by
 * 1/sqrt((1 - r^2)^2 + (2 zeta r)^2) = 3.288, so accel x carries a 3.28 m/s2
 * (analog roll-off 0.998) 110 Hz error during the tone; every other channel
 * must read the motion exactly as the DigitalSensor would. */
#include "fire_harness.h"
#include "imu_motion.h"

static double tone(double t) {
  return t >= 0.3 && t < 0.8 ? sin(2.0 * 3.14159265358979323846 * 110.0 * (t - 0.3)) : 0.0;
}

static const fh_input_t inputs[] = {
    IMU_MOTION_INPUTS,
    FH_INPUT(sensortemperature, FH_CONSTANT(293.15)),
    FH_INPUT(acousticpressure, FH_FUNCTION(tone)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(sampletime), FH_OUTPUT(acceleration_1), FH_OUTPUT(acceleration_2),
    FH_OUTPUT(acceleration_3), FH_OUTPUT(gyro_1),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Systems.Sensing.IMU.AcousticSensor",
      .stop_s = 3.0,
      .step_s = 0.0002,
      .print_s = 0.25,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
