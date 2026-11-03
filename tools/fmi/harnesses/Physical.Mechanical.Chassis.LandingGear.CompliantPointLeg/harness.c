/* FMI harness: Physical.Mechanical.Chassis.LandingGear.CompliantPointLeg
 * (through Wrapper.mo, which only gives the inputs valid start values).
 *
 * The leg is massless and stateless: the harness plays the vehicle and
 * prescribes consistent CG kinematics (position, body velocity, body rate,
 * attitude) over a level NED ground plane at Down = 0 (outward normal
 * {0,0,-1}). world.toml overrides the leg tip to the asymmetric
 * rLeg_b = {0.12,-0.08,0.1} (forward, left, 0.1 m below the CG) so every
 * component of r x F and of the R_wb mapping is nonzero and a swapped index,
 * sign or transpose would show. k = 3000 N/m, c = 150 N.s/m, tangential
 * damping 25 N.s/m, mu = 0.6. The kinematics are prescribed instants (the
 * leg is algebraic), so a body rate need not be integrated into R_wb.
 *
 *   0.00-0.20  hover 0.3 m above the ground (gap 0.2 m, no contact)
 *   0.20-0.62  descend at 0.5 m/s; touchdown at 0.60 s, 10 mm penetration
 *   0.62-0.80  hold: spring only, 30 N up
 *   0.80-1.00  ground (a moving deck) slides at -0.4 m/s North under the leg
 *   1.00-1.40  vehicle slides North at 0.4 m/s: viscous friction 10 N
 *   1.40-1.80  vehicle slides at 1.0 m/s: friction capped at mu*N = 18 N
 *   1.80-2.20  yaw rate 2 rad/s about the CG: tip moves at w x r =
 *              (0.16,0.24,0) m/s, viscous friction (-4,-6,0) N
 *   2.20-2.60  yawed 90 deg (nose East), body-forward 0.4 m/s: the friction
 *              acts West-East in world, body -x in body axes
 *   2.60-3.00  lift off at 0.5 m/s: the damper cannot pull (force stays 0)
 *   3.00-3.30  sloped terrain: the ground plane passes through
 *              terrainPoint_w = (2,-1,0.5) with a 10 deg incline (outward
 *              normal n = (-sin10 cos30, -sin10 sin30, -cos10)) and moves at
 *              (0.1,0.05,0) m/s; the vehicle sits at a general attitude
 *              (ZYX yaw 0.7, pitch -0.15, roll 0.2 rad, so all nine R_wb
 *              entries are nonzero) with the tip 8 mm into the slope, body
 *              velocity (0.3,-0.2,0.1) and rate (0.4,-0.6,0.5): every input
 *              component is nonzero; viscous friction
 *   3.30-3.60  same pose, body velocity (2.5,-1.8,-0.1): friction capped
 *
 * checks.toml compares forces, moments and tangential power against the
 * closed-form penalty/friction laws and the lever arm r x F, checks the
 * world/body frame mapping in the yawed phase, unilateral behaviour, and
 * dissipativity (tangential power never positive). The sloped phases pin the
 * terrainPoint term of the gap, the normal/tangential velocity split for a
 * non-vertical normal, and the full R_wb / R_wb' mapping (incl. roll and
 * pitch entries) against an independent numpy evaluation of the leg law. */
#include "fire_harness.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Phase boundaries [s]. */
#define T_DESCEND 0.20
#define T_HOLD 0.62
#define T_DECK 0.80
#define T_SLIDE 1.00
#define T_FAST 1.40
#define T_PITCH 1.80 /* yaw-rate phase */
#define T_YAW 2.20
#define T_LIFT 2.60
#define T_SLOPE 3.00
#define T_SLOPE_FAST 3.30

/* Sloped-terrain phase geometry (fixed pose, prescribed instants). */
static const double r_leg[3] = {0.12, -0.08, 0.1};
static const double terrain_point[3] = {2.0, -1.0, 0.5};
static const double terrain_vel[3] = {0.1, 0.05, 0.0};
static const double slope_omega[3] = {0.4, -0.6, 0.5};
static int slope(double t) { return t >= T_SLOPE; }

static void slope_r(double R[9]) { /* R = Rz(0.7) Ry(-0.15) Rx(0.2) */
  const double cy = cos(0.7), sy = sin(0.7), cp = cos(-0.15), sp = sin(-0.15);
  const double cr = cos(0.2), sr = sin(0.2);
  R[0] = cy * cp; R[1] = cy * sp * sr - sy * cr; R[2] = cy * sp * cr + sy * sr;
  R[3] = sy * cp; R[4] = sy * sp * sr + cy * cr; R[5] = sy * sp * cr - cy * sr;
  R[6] = -sp;     R[7] = cp * sr;                R[8] = cp * cr;
}

static void slope_n(double n[3]) {
  const double inc = 10.0 * M_PI / 180.0, az = 30.0 * M_PI / 180.0;
  n[0] = -sin(inc) * cos(az); n[1] = -sin(inc) * sin(az); n[2] = -cos(inc);
}

/* CG position: tip 8 mm into the slope, offset tangentially from
 * terrainPoint by (0.3,0.2,0) projected onto the plane. */
static double slope_p(int i) {
  double R[9], n[3], off[3] = {0.3, 0.2, 0.0};
  slope_r(R);
  slope_n(n);
  const double on = off[0] * n[0] + off[1] * n[1] + off[2] * n[2];
  const double rr = R[3 * i] * r_leg[0] + R[3 * i + 1] * r_leg[1] + R[3 * i + 2] * r_leg[2];
  return terrain_point[i] + (off[i] - on * n[i]) - 0.008 * n[i] - rr;
}

static double down(double t) { /* CG Down coordinate */
  if (slope(t))
    return slope_p(2);
  if (t < T_DESCEND)
    return -0.3;
  if (t < T_HOLD)
    return -0.3 + 0.5 * (t - T_DESCEND);
  if (t < T_LIFT)
    return -0.09;
  return -0.09 - 0.5 * (t - T_LIFT);
}

static double down_rate(double t) {
  if (t >= T_DESCEND && t < T_HOLD)
    return 0.5;
  if (t >= T_LIFT)
    return -0.5;
  return 0.0;
}

static double forward_speed(double t) { /* body x velocity */
  if (t >= T_SLIDE && t < T_FAST)
    return 0.4;
  if (t >= T_FAST && t < T_PITCH)
    return 1.0;
  if (t >= T_YAW && t < T_LIFT)
    return 0.4;
  return 0.0;
}

static int yawed(double t) { return t >= T_YAW && t < T_LIFT; }

/* World CG position integrates the world velocity of the phases above. */
static double north(double t) {
  if (slope(t))
    return slope_p(0);
  double x = 0.0;
  if (t > T_SLIDE)
    x += 0.4 * (fmin(t, T_FAST) - T_SLIDE);
  if (t > T_FAST)
    x += 1.0 * (fmin(t, T_PITCH) - T_FAST);
  return x;
}

static double east(double t) {
  if (slope(t))
    return slope_p(1);
  return t > T_YAW ? 0.4 * (fmin(t, T_LIFT) - T_YAW) : 0.0; }

static double p1(double t) { return north(t); }
static double p2(double t) { return east(t); }
static double p3(double t) { return down(t); }
static double v1(double t) { return slope(t) ? (t < T_SLOPE_FAST ? 0.3 : 2.5) : forward_speed(t); }
static double v2(double t) { return slope(t) ? (t < T_SLOPE_FAST ? -0.2 : -1.8) : 0.0; }
static double v3(double t) { return slope(t) ? (t < T_SLOPE_FAST ? 0.1 : -0.1) : down_rate(t); }
static double w1(double t) { return slope(t) ? slope_omega[0] : 0.0; }
static double w2(double t) { return slope(t) ? slope_omega[1] : 0.0; }
static double w3(double t) {
  if (slope(t))
    return slope_omega[2];
  return t >= T_PITCH && t < T_YAW ? 2.0 : 0.0;
}
/* Body-to-world rotation, row-major: identity, a 90-degree yaw (nose East),
 * or the general sloped-phase attitude. */
static double r_entry(double t, int k) {
  static const double ident[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  static const double yaw90[9] = {0, -1, 0, 1, 0, 0, 0, 0, 1};
  if (slope(t)) {
    double R[9];
    slope_r(R);
    return R[k];
  }
  return yawed(t) ? yaw90[k] : ident[k];
}
#define R_FN(k)                                                                                    \
  static double r##k(double t) { return r_entry(t, k); }
R_FN(0)
R_FN(1)
R_FN(2)
R_FN(3)
R_FN(4)
R_FN(5)
R_FN(6)
R_FN(7)
R_FN(8)
static double tp(double t, int i) { return slope(t) ? terrain_point[i] : 0.0; }
static double tp1(double t) { return tp(t, 0); }
static double tp2(double t) { return tp(t, 1); }
static double tp3(double t) { return tp(t, 2); }
static double tn(double t, int i) {
  double n[3] = {0.0, 0.0, -1.0};
  if (slope(t))
    slope_n(n);
  return n[i];
}
static double tn1(double t) { return tn(t, 0); }
static double tn2(double t) { return tn(t, 1); }
static double tn3(double t) { return tn(t, 2); }
static double deck(double t) {
  if (slope(t))
    return terrain_vel[0];
  return t >= T_DECK && t < T_SLIDE ? -0.4 : 0.0;
}
static double tv2(double t) { return slope(t) ? terrain_vel[1] : 0.0; }

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
    FH_INPUT(r_wb_1, FH_FUNCTION(r0)),
    FH_INPUT(r_wb_2, FH_FUNCTION(r1)),
    FH_INPUT(r_wb_3, FH_FUNCTION(r2)),
    FH_INPUT(r_wb_4, FH_FUNCTION(r3)),
    FH_INPUT(r_wb_5, FH_FUNCTION(r4)),
    FH_INPUT(r_wb_6, FH_FUNCTION(r5)),
    FH_INPUT(r_wb_7, FH_FUNCTION(r6)),
    FH_INPUT(r_wb_8, FH_FUNCTION(r7)),
    FH_INPUT(r_wb_9, FH_FUNCTION(r8)),
    FH_INPUT(terrainpoint_w_1, FH_FUNCTION(tp1)),
    FH_INPUT(terrainpoint_w_2, FH_FUNCTION(tp2)),
    FH_INPUT(terrainpoint_w_3, FH_FUNCTION(tp3)),
    FH_INPUT(terrainnormal_w_1, FH_FUNCTION(tn1)),
    FH_INPUT(terrainnormal_w_2, FH_FUNCTION(tn2)),
    FH_INPUT(terrainnormal_w_3, FH_FUNCTION(tn3)),
    FH_INPUT(terrainvelocity_w_1, FH_FUNCTION(deck)),
    FH_INPUT(terrainvelocity_w_2, FH_FUNCTION(tv2)),
    FH_INPUT(terrainvelocity_w_3, FH_CONSTANT(0.0)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(gap),           FH_OUTPUT(normalforce), FH_OUTPUT(force_b_1),
    FH_OUTPUT(force_b_3),     FH_OUTPUT(force_w_2),   FH_OUTPUT(moment_b_1), FH_OUTPUT(moment_b_2), FH_OUTPUT(moment_b_3),
    FH_OUTPUT(tangentialpower),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Physical.Mechanical.Chassis.LandingGear.CompliantPointLeg",
      .stop_s = 3.6,
      .step_s = 0.001,
      .print_s = 0.1,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
