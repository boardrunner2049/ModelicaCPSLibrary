/* FMI harness: Systems.Sensing.Magnetometer.Sensor.
 *
 * magneticField = R_bs' * R_wb' * magneticField_w + bias  (Tesla, NED world, FRD body).
 * A constant, Earth-like NED field (N 20 uT, E -4 uT, D 47 uT: mid-latitude northern
 * hemisphere, dipping ~66 deg) is observed while the body changes attitude:
 *   [0,1) level, nose North      -> body field equals the world field
 *   [1,2) yaw 90 deg right (East) -> North component appears on body -y
 *   [2,3) pitch up 30 deg         -> Down component leaks onto body -x
 *   [3,4] yaw sweep at pi/2 rad/s -> horizontal field rotates, vertical stays put
 * world.toml mounts the sensor 90 deg about z (sensor x along body right,
 * R_bs = [0 -1 0; 1 0 0; 0 0 1]) with a bias of (1, -2, 0.5) uT. checks.toml holds
 * closed-form values (cross-checked against a native `rumoca sim` of the same profile)
 * and the rotation invariant: the bias-corrected reading keeps the field magnitude. */
#include "fire_harness.h"

#include <math.h>

#define PI 3.14159265358979323846

static void attitude(double t, double R[9]) {
  double c, s;
  if (t < 1.0) {
    c = 1.0, s = 0.0; /* yaw 0 */
  } else if (t < 2.0) {
    c = 0.0, s = 1.0; /* yaw 90 deg */
  } else if (t < 3.0) {
    double th = 30.0 * PI / 180.0; /* pitch: R = Ry(theta) */
    double m[9] = {cos(th), 0, sin(th), 0, 1, 0, -sin(th), 0, cos(th)};
    for (int i = 0; i < 9; ++i)
      R[i] = m[i];
    return;
  } else {
    double psi = 0.5 * PI * (t - 3.0);
    c = cos(psi), s = sin(psi);
  }
  double m[9] = {c, -s, 0, s, c, 0, 0, 0, 1}; /* Rz(psi) */
  for (int i = 0; i < 9; ++i)
    R[i] = m[i];
}

#define R_FN(i)                                                                                    \
  static double r_wb_##i##_fn(double t) {                                                          \
    double R[9];                                                                                   \
    attitude(t, R);                                                                                \
    return R[(i) - 1];                                                                             \
  }
R_FN(1) R_FN(2) R_FN(3) R_FN(4) R_FN(5) R_FN(6) R_FN(7) R_FN(8) R_FN(9)

static const fh_input_t inputs[] = {
    FH_INPUT(r_wb_1, FH_FUNCTION(r_wb_1_fn)),
    FH_INPUT(r_wb_2, FH_FUNCTION(r_wb_2_fn)),
    FH_INPUT(r_wb_3, FH_FUNCTION(r_wb_3_fn)),
    FH_INPUT(r_wb_4, FH_FUNCTION(r_wb_4_fn)),
    FH_INPUT(r_wb_5, FH_FUNCTION(r_wb_5_fn)),
    FH_INPUT(r_wb_6, FH_FUNCTION(r_wb_6_fn)),
    FH_INPUT(r_wb_7, FH_FUNCTION(r_wb_7_fn)),
    FH_INPUT(r_wb_8, FH_FUNCTION(r_wb_8_fn)),
    FH_INPUT(r_wb_9, FH_FUNCTION(r_wb_9_fn)),
    FH_INPUT(magneticfield_w_1, FH_CONSTANT(20.0e-6)),
    FH_INPUT(magneticfield_w_2, FH_CONSTANT(-4.0e-6)),
    FH_INPUT(magneticfield_w_3, FH_CONSTANT(47.0e-6)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(magneticfield_1),
    FH_OUTPUT(magneticfield_2),
    FH_OUTPUT(magneticfield_3),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Systems.Sensing.Magnetometer.Sensor",
      .stop_s = 4.0,
      .step_s = 0.01,
      .print_s = 0.25,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
