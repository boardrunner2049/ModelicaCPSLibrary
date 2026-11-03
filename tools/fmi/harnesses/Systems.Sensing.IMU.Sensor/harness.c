/* FMI harness: Systems.Sensing.IMU.Sensor.
 *
 * Ideal strapdown IMU at a lever arm on an FRD body in a NED world:
 *   acceleration = R_bs' * (R_wb'*(a_w - gravity_w) + alpha_b x r_b + omega_b x (omega_b x r_b)) + accelBias
 *   gyro         = R_bs' * omega_b + gyroBias
 * world.toml sets non-default parameters: lever arm r_b = (0.10,-0.05,0.02) m,
 * the vehicle mount R_bs = Rx(90deg)*Rz(90deg) = [0 -1 0; 0 0 -1; 1 0 0] and non-zero
 * accelerometer/gyro biases. The mount is a proper but NON-symmetric rotation, so the
 * documented contract v_b = R_bs*v_s is actually tested: the sensor reads
 * (b_z, -b_x, -b_y), whereas a missing/extra transpose or a column-major flattening
 * would give (-b_y, -b_z, b_x).
 * Phases (within each phase R_wb is consistent with omega_b/alpha_b; the attitude is
 * reset at the phase boundaries t = 1, 2, 5 without a matching rate -- harmless,
 * the sensor is algebraic):
 *   [0,1) level and still              -> -g appears on sensor x (= body z)
 *   [1,2) static 30 deg right roll     -> gravity projects onto body y, routed to sensor z
 *   [2,4) yaw at 2 rad/s, a_w = 1.5 m/s2 North -> world acceleration rotates in the body frame,
 *                                         centripetal term -w^2*(rx,ry,0) from the lever arm,
 *                                         yaw rate sensed on sensor x
 *   [4,5) yaw spin-up alpha = 3 rad/s2 -> tangential term alpha x r_b appears
 *   [5,6) roll at 1 rad/s about body x -> roll rate sensed negated on sensor y
 * checks.toml compares against closed-form values computed from these kinematics
 * (cross-checked with a native `rumoca sim` of the same profile). */
#include "fire_harness.h"

#include <math.h>

#define G0 9.80665
#define DEG (3.14159265358979323846 / 180.0)

typedef struct {
  double R[9]; /* R_wb, row-major */
  double a[3]; /* a_w */
  double w[3]; /* omega_b */
  double al[3]; /* alpha_b */
} kin_t;

static void rot_x(double phi, double R[9]) {
  double c = cos(phi), s = sin(phi);
  double m[9] = {1, 0, 0, 0, c, -s, 0, s, c};
  for (int i = 0; i < 9; ++i)
    R[i] = m[i];
}

static void rot_z(double psi, double R[9]) {
  double c = cos(psi), s = sin(psi);
  double m[9] = {c, -s, 0, s, c, 0, 0, 0, 1};
  for (int i = 0; i < 9; ++i)
    R[i] = m[i];
}

static kin_t kinematics(double t) {
  kin_t k = {{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  if (t < 1.0) {
    /* level, at rest */
  } else if (t < 2.0) {
    rot_x(30.0 * DEG, k.R);
  } else if (t < 4.0) {
    double tau = t - 2.0;
    rot_z(2.0 * tau, k.R);
    k.a[0] = 1.5;
    k.w[2] = 2.0;
  } else if (t < 5.0) {
    double tau = t - 4.0;
    rot_z(4.0 + 2.0 * tau + 1.5 * tau * tau, k.R);
    k.w[2] = 2.0 + 3.0 * tau;
    k.al[2] = 3.0;
  } else {
    rot_x(1.0 * (t - 5.0), k.R);
    k.w[0] = 1.0;
  }
  return k;
}

#define R_FN(i)                                                                                    \
  static double r_wb_##i##_fn(double t) { return kinematics(t).R[(i) - 1]; }
#define V_FN(name, field, i)                                                                       \
  static double name##_##i##_fn(double t) { return kinematics(t).field[(i) - 1]; }
R_FN(1) R_FN(2) R_FN(3) R_FN(4) R_FN(5) R_FN(6) R_FN(7) R_FN(8) R_FN(9)
V_FN(a_w, a, 1) V_FN(a_w, a, 2) V_FN(a_w, a, 3)
V_FN(omega_b, w, 1) V_FN(omega_b, w, 2) V_FN(omega_b, w, 3)
V_FN(alpha_b, al, 1) V_FN(alpha_b, al, 2) V_FN(alpha_b, al, 3)

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
    FH_INPUT(a_w_1, FH_FUNCTION(a_w_1_fn)),
    FH_INPUT(a_w_2, FH_FUNCTION(a_w_2_fn)),
    FH_INPUT(a_w_3, FH_FUNCTION(a_w_3_fn)),
    FH_INPUT(gravity_w_1, FH_CONSTANT(0.0)),
    FH_INPUT(gravity_w_2, FH_CONSTANT(0.0)),
    FH_INPUT(gravity_w_3, FH_CONSTANT(G0)), /* NED: gravity points Down */
    FH_INPUT(omega_b_1, FH_FUNCTION(omega_b_1_fn)),
    FH_INPUT(omega_b_2, FH_FUNCTION(omega_b_2_fn)),
    FH_INPUT(omega_b_3, FH_FUNCTION(omega_b_3_fn)),
    FH_INPUT(alpha_b_1, FH_FUNCTION(alpha_b_1_fn)),
    FH_INPUT(alpha_b_2, FH_FUNCTION(alpha_b_2_fn)),
    FH_INPUT(alpha_b_3, FH_FUNCTION(alpha_b_3_fn)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(acceleration_1), FH_OUTPUT(acceleration_2), FH_OUTPUT(acceleration_3),
    FH_OUTPUT(gyro_1),         FH_OUTPUT(gyro_2),         FH_OUTPUT(gyro_3),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Systems.Sensing.IMU.Sensor",
      .stop_s = 6.0,
      .step_s = 0.001,
      .print_s = 0.5,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
