/* FMI harness: Adapters.VirtualIO.SensorValues.
 *
 * Stateless sensor-local -> FRD conversion: acceleration_frd = R_bImu*a,
 * gyro_frd = R_bImu*w, magneticField_frd = R_bMag*B; GNSS position/velocity
 * are already NED and pass through. world.toml sets R_bImu to the cyclic
 * rotation [0 0 1; 1 0 0; 0 1 0] (det +1, not symmetric, so a transposed
 * matrix shows); R_bMag keeps its default diag{1,-1,-1} (sensor flipped
 * about x). Every channel gets a distinct time-varying signal (sines of
 * different frequency, ramps, steps), and the barometer fields, which the
 * adapter ignores, are driven too. Run 1 s at a 1 ms step. */
#include "fire_harness.h"

static const fh_input_t inputs[] = {
    FH_INPUT(measurements_acceleration_1, FH_SINE(0.5, 2.0, 3.0, 0.0)),
    FH_INPUT(measurements_acceleration_2, FH_SINE(-1.0, 1.5, 5.0, 1.0)),
    FH_INPUT(measurements_acceleration_3, FH_STEP(0.4, -9.81, -12.0)),
    FH_INPUT(measurements_gyro_1, FH_RAMP(0.2, 0.0, 0.8, 3.0)),
    FH_INPUT(measurements_gyro_2, FH_SINE(0.0, 0.7, 2.0, 0.5)),
    FH_INPUT(measurements_gyro_3, FH_STEP(0.5, 0.1, -0.4)),
    FH_INPUT(measurements_magneticfield_1, FH_CONSTANT(2.1e-5)),
    FH_INPUT(measurements_magneticfield_2, FH_SINE(0.0, 1e-5, 1.0, 0.0)),
    FH_INPUT(measurements_magneticfield_3, FH_STEP(0.6, 4.3e-5, -4.3e-5)),
    FH_INPUT(measurements_position_1, FH_RAMP(0.0, 10.0, 1.0, 13.0)),
    FH_INPUT(measurements_position_2, FH_RAMP(0.0, -5.0, 1.0, -7.0)),
    FH_INPUT(measurements_position_3, FH_RAMP(0.0, -20.0, 1.0, -21.0)),
    FH_INPUT(measurements_velocity_1, FH_CONSTANT(3.0)),
    FH_INPUT(measurements_velocity_2, FH_CONSTANT(-2.0)),
    FH_INPUT(measurements_velocity_3, FH_CONSTANT(-1.0)),
    FH_INPUT(measurements_pressure, FH_CONSTANT(101325.0)),
    FH_INPUT(measurements_temperature, FH_CONSTANT(288.15)),
    FH_INPUT(measurements_altitude, FH_RAMP(0.0, 20.0, 1.0, 21.0)),
    FH_INPUT(measurements_climbrate, FH_CONSTANT(1.0)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(acceleration_frd_1), FH_OUTPUT(acceleration_frd_2), FH_OUTPUT(acceleration_frd_3),
    FH_OUTPUT(gyro_frd_1),         FH_OUTPUT(gyro_frd_2),         FH_OUTPUT(gyro_frd_3),
    FH_OUTPUT(magneticfield_frd_1), FH_OUTPUT(magneticfield_frd_2), FH_OUTPUT(magneticfield_frd_3),
    FH_OUTPUT(position_ned_1),     FH_OUTPUT(position_ned_2),     FH_OUTPUT(position_ned_3),
    FH_OUTPUT(velocity_ned_1),     FH_OUTPUT(velocity_ned_2),     FH_OUTPUT(velocity_ned_3),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Adapters.VirtualIO.SensorValues",
      .stop_s = 1.0,
      .step_s = 0.001,
      .print_s = 0.1,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
