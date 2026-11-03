/* FMI harness: Systems.Sensing.SensorSuite.
 *
 * Four ideal sensors on one rigid FRD body in a NED world: IMU and magnetometer
 * with DISTINCT, NON-symmetric vehicle mounts (R_bImu = Rx(90)*Rz(90), R_bMag = Rz(90)),
 * so swapped mount wiring or a missing/extra transpose in either sub-sensor changes
 * the readings; GNSS antenna and barometer at lever arms the suite converts into
 * world position/velocity (p + R*r, v + R*(omega x r)); barometer altitude/climb
 * rate Up-positive. world.toml sets distinct lever arms and biases so each suite
 * parameter can be traced to the sub-sensor it must reach.
 * Phases (inside each phase p/v/a_w and R_wb/omega_b/alpha_b are mutually
 * consistent; the attitude is reset without a matching rate at t = 6, where the
 * pitch jumps 0 -> -10 deg -- harmless, every sensor is algebraic):
 *   [0,2)  hover at p = (10,-5,-20) m, level, nose North
 *   [2,6)  pirouette: yaw at 0.5 rad/s in place -> antenna circles the CG
 *          (horizontal lever-arm velocity), IMU sees centripetal -w^2*rx, field rotates
 *   [6,8)  climb at 2 m/s, yaw held at 2 rad, pitched 10 deg nose-down (pitch
 *          moves the lever arms' heights; barometer climb rate = +2 m/s)
 *   [8,10] pull-up: pitch rate q = 0.2 + 0.6*tau rad/s (alpha_b = 0.6 rad/s2 about
 *          body y) while the CG accelerates a_w = (0.8, 0, -1.0) m/s2 (North and up).
 *          A pitch rate moves the fore/aft lever arms VERTICALLY, so the barometer
 *          climb rate and GNSS Down velocity carry the R_wb*cross(omega_b, r) term, and
 *          the IMU sees a_w and alpha_b x r (both inputs are non-zero only here).
 * The environment inputs are standard gravity, a mid-latitude NED field
 * (20,-4,47) uT and ISA pressure/temperature at the CG height.
 * checks.toml holds closed-form expectations (also matched by a native
 * `rumoca sim` of the same profile).
 * NOTE: the suite's single output is a record (`output SensorMeasurements
 * measurements`); Rumoca exports its fields with causality "local", so the FMU
 * has no outputs to print (suspected compiler bug, reported). The checks read
 * the measurements.* locals from the trace instead. */
#include "fire_harness.h"

#include <math.h>

#define G0 9.80665
#define PI 3.14159265358979323846

typedef struct {
  double p[3], v[3], a[3], R[9], w[3], al[3];
} kin_t;

static kin_t kinematics(double t) {
  kin_t k = {{10.0, -5.0, -20.0}, {0, 0, 0}, {0, 0, 0}, {1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}, {0, 0, 0}};
  double psi = 0.0, th = 0.0;
  if (t < 2.0) {
  } else if (t < 6.0) {
    psi = 0.5 * (t - 2.0);
    k.w[2] = 0.5;
  } else if (t < 8.0) {
    psi = 2.0;
    th = -10.0 * PI / 180.0; /* nose down */
    k.p[2] = -20.0 - 2.0 * (t - 6.0);
    k.v[2] = -2.0;
  } else {
    double u = t - 8.0;
    psi = 2.0;
    th = -10.0 * PI / 180.0 + 0.2 * u + 0.3 * u * u; /* pull-up */
    k.w[1] = 0.2 + 0.6 * u;
    k.al[1] = 0.6;
    k.a[0] = 0.8;
    k.a[2] = -1.0;
    k.v[0] = 0.8 * u;
    k.v[2] = -2.0 - u;
    k.p[0] = 10.0 + 0.4 * u * u;
    k.p[2] = -24.0 - 2.0 * u - 0.5 * u * u;
  }
  /* R = Rz(psi) * Ry(th); omega_b = (0, th', psi') holds because only one angle moves per phase */
  double cz = cos(psi), sz = sin(psi), cy = cos(th), sy = sin(th);
  double R[9] = {cz * cy, -sz, cz * sy, sz * cy, cz, sz * sy, -sy, 0.0, cy};
  for (int i = 0; i < 9; ++i)
    k.R[i] = R[i];
  return k;
}

static double isa_temperature(double t) { return 288.15 + 0.0065 * kinematics(t).p[2]; }
static double isa_pressure(double t) { return 101325.0 * pow(isa_temperature(t) / 288.15, 5.25588); }

#define FN(name, field, i)                                                                         \
  static double name##_##i##_fn(double t) { return kinematics(t).field[(i) - 1]; }
FN(p_w, p, 1) FN(p_w, p, 2) FN(p_w, p, 3)
FN(v_w, v, 1) FN(v_w, v, 2) FN(v_w, v, 3)
FN(r_wb, R, 1) FN(r_wb, R, 2) FN(r_wb, R, 3) FN(r_wb, R, 4) FN(r_wb, R, 5)
FN(r_wb, R, 6) FN(r_wb, R, 7) FN(r_wb, R, 8) FN(r_wb, R, 9)
FN(omega_b, w, 1) FN(omega_b, w, 2) FN(omega_b, w, 3)
FN(a_w, a, 1) FN(a_w, a, 2) FN(a_w, a, 3)
FN(alpha_b, al, 1) FN(alpha_b, al, 2) FN(alpha_b, al, 3)

static const fh_input_t inputs[] = {
    FH_INPUT(p_w_1, FH_FUNCTION(p_w_1_fn)),
    FH_INPUT(p_w_2, FH_FUNCTION(p_w_2_fn)),
    FH_INPUT(p_w_3, FH_FUNCTION(p_w_3_fn)),
    FH_INPUT(v_w_1, FH_FUNCTION(v_w_1_fn)),
    FH_INPUT(v_w_2, FH_FUNCTION(v_w_2_fn)),
    FH_INPUT(v_w_3, FH_FUNCTION(v_w_3_fn)),
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
    FH_INPUT(omega_b_1, FH_FUNCTION(omega_b_1_fn)),
    FH_INPUT(omega_b_2, FH_FUNCTION(omega_b_2_fn)),
    FH_INPUT(omega_b_3, FH_FUNCTION(omega_b_3_fn)),
    FH_INPUT(alpha_b_1, FH_FUNCTION(alpha_b_1_fn)),
    FH_INPUT(alpha_b_2, FH_FUNCTION(alpha_b_2_fn)),
    FH_INPUT(alpha_b_3, FH_FUNCTION(alpha_b_3_fn)),
    FH_INPUT(gravity_w_1, FH_CONSTANT(0.0)),
    FH_INPUT(gravity_w_2, FH_CONSTANT(0.0)),
    FH_INPUT(gravity_w_3, FH_CONSTANT(G0)),
    FH_INPUT(magneticfield_w_1, FH_CONSTANT(20.0e-6)),
    FH_INPUT(magneticfield_w_2, FH_CONSTANT(-4.0e-6)),
    FH_INPUT(magneticfield_w_3, FH_CONSTANT(47.0e-6)),
    FH_INPUT(pressure, FH_FUNCTION(isa_pressure)),
    FH_INPUT(temperature, FH_FUNCTION(isa_temperature)),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Systems.Sensing.SensorSuite",
      .stop_s = 10.0,
      .step_s = 0.01,
      .print_s = 0.0,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = NULL, /* no FMU outputs: see NOTE above */
      .output_count = 0,
  };
  return fh_run(&scenario);
}
