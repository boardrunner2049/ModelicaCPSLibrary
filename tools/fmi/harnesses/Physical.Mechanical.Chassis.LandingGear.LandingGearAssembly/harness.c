/* FMI harness: Physical.Mechanical.Chassis.LandingGear.LandingGearAssembly
 * (through Wrapper.mo, which only gives the inputs valid start values).
 *
 * Four massless legs, k = 3000 N/m, c = 150 N.s/m, tangential damping
 * 25 N.s/m, mu = 0.6, over a level NED ground plane at Down = 0. world.toml
 * replaces the default square layout by an asymmetric one (FRD, from the CG):
 *   leg 1 ( 0.20,  0.15, 0.10)   leg 2 (-0.12,  0.18, 0.12)
 *   leg 3 (-0.15, -0.16, 0.10)   leg 4 ( 0.18, -0.10, 0.11)
 * so the legs touch down at different times, carry different loads, and
 * every component of the summed force and moment is distinct: a swapped
 * leg, index, sign or R_wb transpose would show. The harness plays the
 * vehicle and prescribes the CG kinematics (instantaneous values; the gear is
 * algebraic, so body rates need not be integrated into R_wb):
 *
 *   0.00-0.20  level, CG Down -0.3 m: every leg clear
 *   0.20-0.61  descend at 0.5 m/s; legs touch down in the order 2 (0.56 s),
 *              4 (0.58 s), then 1 and 3 (0.60 s); end at CG Down -0.095 m
 *   0.61-0.90  hold level: 15/75/15/45 N (penetrations 5/25/5/15 mm)
 *   0.90-1.30  static attitude roll 0.06 rad, pitch -0.04 rad (ZYX): leg 3
 *              lifts off, the others share 205 N
 *   1.30-1.70  yawed 30 deg, sliding at body velocity (0.3, -0.5, 0) m/s:
 *              legs 1 and 3 hit the Coulomb cap mu*N = 9 N, legs 2 and 4
 *              stay viscous (25*0.583 = 14.6 N)
 *   1.70-2.00  level, body rates (0.5, -0.8, 0.3) rad/s: leg 3 stays
 *              geometrically in contact but its damper unloads it to 0 N
 *   2.00-2.30  climb at 1 m/s: every leg leaves the ground
 *   2.30-2.70  sloped, heaving deck: the plane passes through
 *              terrainPoint_w = (1.5,-0.8,0.4) with an 8 deg incline toward
 *              azimuth -40 deg and moves at (-0.2,0.15,0.05) m/s; attitude
 *              ZYX (-0.5, 0.1, -0.12) rad, CG 0.085 m above the plane,
 *              v_b (0.6,0.5,0.05), omega_b (0.2,0.3,-0.4). Leg 1 stays clear
 *              (gap 0.79 mm), leg 2 is Coulomb-capped, legs 3 and 4 viscous.
 *              Every terrain input is nonzero, so a dropped or mis-wired
 *              terrain fan-out to any leg would change that leg's values.
 *
 * ref.py (in this directory) is the independent numpy evaluation that
 * produced every expected value. checks.toml compares per-leg normal forces/gaps/contact flags and the
 * summed body force and moment about the CG with an independent Python
 * evaluation of the leg laws for this geometry, plus invariants (unilateral
 * normal force, dissipative friction, ground can only push Up). */
#include "fire_harness.h"

#include <math.h>

#define T_DESCEND 0.20
#define T_HOLD 0.61
#define T_TILT 0.90
#define T_SLIDE 1.30
#define T_RATES 1.70
#define T_LIFT 2.00
#define T_SLOPE 2.30
#define PI 3.14159265358979323846

static int in(double t, double a, double b) { return t >= a && t < b; }
static int slope(double t) { return t >= T_SLOPE; }

/* Sloped-deck geometry (same construction as ref.py). */
static const double slope_point[3] = {1.5, -0.8, 0.4};
static const double slope_vel[3] = {-0.2, 0.15, 0.05};
static void slope_n(double n[3]) {
  const double inc = 8.0 * PI / 180.0, az = -40.0 * PI / 180.0;
  n[0] = -sin(inc) * cos(az);
  n[1] = -sin(inc) * sin(az);
  n[2] = -cos(inc);
}
/* CG: terrain point + tangential offset (0.25,-0.1,0) projected on the
 * plane + 0.085 m along the outward normal. */
static double slope_cg(int i) {
  double n[3];
  const double off[3] = {0.25, -0.1, 0.0};
  slope_n(n);
  const double on = off[0] * n[0] + off[1] * n[1] + off[2] * n[2];
  return slope_point[i] + off[i] - on * n[i] + 0.085 * n[i];
}

static double down(double t) {
  if (slope(t))
    return slope_cg(2);
  if (t < T_DESCEND)
    return -0.3;
  if (t < T_HOLD)
    return -0.3 + 0.5 * (t - T_DESCEND);
  if (t < T_LIFT)
    return -0.095;
  return -0.095 - 1.0 * (t - T_LIFT);
}

static double down_rate(double t) {
  if (slope(t))
    return 0.05;
  if (t >= T_DESCEND && t < T_HOLD)
    return 0.5;
  if (t >= T_LIFT)
    return -1.0;
  return 0.0;
}

/* Attitude per phase as ZYX Euler angles; R_wb = Rz(yaw) Ry(pitch) Rx(roll). */
static void euler(double t, double *yaw, double *pitch, double *roll) {
  if (slope(t)) {
    *yaw = -0.5;
    *pitch = 0.1;
    *roll = -0.12;
    return;
  }
  *yaw = in(t, T_SLIDE, T_RATES) ? PI / 6 : 0.0;
  *pitch = in(t, T_TILT, T_SLIDE) ? -0.04 : 0.0;
  *roll = in(t, T_TILT, T_SLIDE) ? 0.06 : 0.0;
}

static double rotation(double t, int i, int j) {
  double y, p, r;
  euler(t, &y, &p, &r);
  const double cz = cos(y), sz = sin(y), cy = cos(p), sy = sin(p), cx = cos(r), sx = sin(r);
  const double R[3][3] = {
      {cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx},
      {sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx},
      {-sy, cy * sx, cy * cx},
  };
  return R[i][j];
}

#define R_ELEMENT(i, j)                                                                            \
  static double r##i##j(double t) { return rotation(t, i - 1, j - 1); }
R_ELEMENT(1, 1)
R_ELEMENT(1, 2)
R_ELEMENT(1, 3)
R_ELEMENT(2, 1)
R_ELEMENT(2, 2)
R_ELEMENT(2, 3)
R_ELEMENT(3, 1)
R_ELEMENT(3, 2)
R_ELEMENT(3, 3)

static double p1(double t) { return slope(t) ? slope_cg(0) : 0.0; }
static double p2(double t) { return slope(t) ? slope_cg(1) : 0.0; }
static double p3(double t) { return down(t); }
static double v1(double t) { return slope(t) ? 0.6 : in(t, T_SLIDE, T_RATES) ? 0.3 : 0.0; }
static double v2(double t) { return slope(t) ? 0.5 : in(t, T_SLIDE, T_RATES) ? -0.5 : 0.0; }
static double v3(double t) { return down_rate(t); }
static double w1(double t) { return slope(t) ? 0.2 : in(t, T_RATES, T_LIFT) ? 0.5 : 0.0; }
static double w2(double t) { return slope(t) ? 0.3 : in(t, T_RATES, T_LIFT) ? -0.8 : 0.0; }
static double w3(double t) { return slope(t) ? -0.4 : in(t, T_RATES, T_LIFT) ? 0.3 : 0.0; }
static double tp1(double t) { return slope(t) ? slope_point[0] : 0.0; }
static double tp2(double t) { return slope(t) ? slope_point[1] : 0.0; }
static double tp3(double t) { return slope(t) ? slope_point[2] : 0.0; }
static double tn(double t, int i) {
  double n[3] = {0.0, 0.0, -1.0};
  if (slope(t))
    slope_n(n);
  return n[i];
}
static double tn1(double t) { return tn(t, 0); }
static double tn2(double t) { return tn(t, 1); }
static double tn3(double t) { return tn(t, 2); }
static double tv1(double t) { return slope(t) ? slope_vel[0] : 0.0; }
static double tv2(double t) { return slope(t) ? slope_vel[1] : 0.0; }
static double tv3(double t) { return slope(t) ? slope_vel[2] : 0.0; }

static const fh_input_t inputs[] = {
    FH_INPUT(p_w_1, FH_FUNCTION(p1)),
    FH_INPUT(p_w_2, FH_FUNCTION(p2)),
    FH_INPUT(p_w_3, FH_FUNCTION(p3)),
    FH_INPUT(v_b_1, FH_FUNCTION(v1)),
    FH_INPUT(v_b_2, FH_FUNCTION(v2)),
    FH_INPUT(v_b_3, FH_FUNCTION(v3)),
    FH_INPUT(omega_b_1, FH_FUNCTION(w1)),
    FH_INPUT(omega_b_2, FH_FUNCTION(w2)),
    FH_INPUT(omega_b_3, FH_FUNCTION(w3)),
    FH_INPUT(r_wb_1, FH_FUNCTION(r11)),
    FH_INPUT(r_wb_2, FH_FUNCTION(r12)),
    FH_INPUT(r_wb_3, FH_FUNCTION(r13)),
    FH_INPUT(r_wb_4, FH_FUNCTION(r21)),
    FH_INPUT(r_wb_5, FH_FUNCTION(r22)),
    FH_INPUT(r_wb_6, FH_FUNCTION(r23)),
    FH_INPUT(r_wb_7, FH_FUNCTION(r31)),
    FH_INPUT(r_wb_8, FH_FUNCTION(r32)),
    FH_INPUT(r_wb_9, FH_FUNCTION(r33)),
    FH_INPUT(terrainpoint_w_1, FH_FUNCTION(tp1)),
    FH_INPUT(terrainpoint_w_2, FH_FUNCTION(tp2)),
    FH_INPUT(terrainpoint_w_3, FH_FUNCTION(tp3)),
    FH_INPUT(terrainnormal_w_1, FH_FUNCTION(tn1)),
    FH_INPUT(terrainnormal_w_2, FH_FUNCTION(tn2)),
    FH_INPUT(terrainnormal_w_3, FH_FUNCTION(tn3)),
    FH_INPUT(terrainvelocity_w_1, FH_FUNCTION(tv1)),
    FH_INPUT(terrainvelocity_w_2, FH_FUNCTION(tv2)),
    FH_INPUT(terrainvelocity_w_3, FH_FUNCTION(tv3)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(normalforce_1), FH_OUTPUT(normalforce_2), FH_OUTPUT(normalforce_3),
    FH_OUTPUT(normalforce_4), FH_OUTPUT(force_b_1),     FH_OUTPUT(force_b_2),
    FH_OUTPUT(force_b_3),     FH_OUTPUT(moment_b_1),    FH_OUTPUT(moment_b_2),
    FH_OUTPUT(moment_b_3),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Physical.Mechanical.Chassis.LandingGear.LandingGearAssembly",
      .stop_s = 2.7,
      .step_s = 0.001,
      .print_s = 0.1,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
