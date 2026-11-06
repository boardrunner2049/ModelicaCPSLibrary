/* Known-answer tests of the rover and fixed-wing vehicle FMUs.
 *
 * usage: test_ground_air VEHICLES_DIR
 *   VEHICLES_DIR/rover/vehicle.json  Exports.FMU.Rover, default parameters (configs/rover.toml)
 *   VEHICLES_DIR/plane/vehicle.json  Exports.FMU.FixedWing, default parameters (configs/plane.toml)
 *
 * Each physical check compares the simulation with a value computed here
 * from the model's published parameters (Vehicles.Rover.RoverPlant,
 * Vehicles.FixedWing.FixedWingPlant, Physical.Mechanical.Aerodynamics.
 * Airframe.Coefficients): static loads, a drivetrain force balance, the
 * single-track turn rate, the glide equilibrium, the instantaneous
 * aerodynamic force and the sign of every control channel.
 *
 * Runs with invalid-operation, divide-by-zero and overflow traps enabled, as
 * ArduPilot SITL does: a model that computes 0/0 even in an unselected branch
 * aborts such a host.
 */
#define _GNU_SOURCE /* feenableexcept */
#include "cpvehicle/cpvehicle.h"

#include <fenv.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define G 9.80665
#define PI 3.14159265358979323846
#define DT 0.002

static int failures, checks;
static char rover[1024], plane[1024];

#define CHECK(cond, ...)                                                                           \
  do {                                                                                             \
    checks++;                                                                                      \
    if (!(cond)) {                                                                                 \
      fprintf(stderr, "%s:%d: FAILED %s: ", __FILE__, __LINE__, #cond);                            \
      fprintf(stderr, __VA_ARGS__);                                                                \
      fputc('\n', stderr);                                                                         \
      failures++;                                                                                  \
    }                                                                                              \
  } while (0)
#define NEAR(a, b, tol) (fabs((a) - (b)) <= (tol))

/* Rover defaults (Vehicles.Rover.RoverPlant). */
#define R_MASS 3.0
#define R_WHEELBASE 0.32
#define R_CG_FRONT 0.16
#define R_CG_HEIGHT 0.09
#define R_RADIUS 0.055
#define R_K 1500.0
#define R_KX 15.0
#define R_CRR 0.015
#define R_MAX_STEER 0.45
#define R_STALL 1.4
#define R_NOLOAD 182.0
#define R_CDA 0.03
#define RHO 1.225
#define V_RELAX 0.5

/* Fixed-wing defaults (Vehicles.FixedWing.FixedWingPlant, Airframe.Coefficients). */
#define P_MASS 2.0
#define P_S 0.45
#define P_B 1.88
#define P_C 0.24
#define P_CL0 0.56
#define P_CLA 6.9
#define P_M 50.0
#define P_ASTALL 0.4712
#define P_E 0.9
#define P_CDP 0.1
#define P_CM0 0.045
#define P_CMA (-0.7)
#define P_XREF (-0.05)
#define P_CG_HEIGHT 0.15
#define P_RADIUS 0.035
#define P_NOSE_X 0.25
#define P_MAIN_X (-0.05)
#define P_GEAR_K 2500.0

typedef struct {
  cpv_vehicle_t *v;
  double t;
} sim_t;

static int sim_open(sim_t *s, const char *path, const char *const *names, const double *values,
                    size_t n) {
  cpv_options_t o;
  cpv_options_init(&o);
  const char *required[] = {"pwm_actuation_v1", "kinematic_state_v1", "imu_v1"};
  o.required_capabilities = required;
  o.required_count = 3;
  char error[1024];
  s->v = NULL;
  s->t = 0;
  cpv_status_t st = cpv_open(path, &o, &s->v, error, sizeof(error));
  CHECK(st == CPV_OK, "cpv_open(%s) -> %s: %s", path, cpv_status_string(st), error);
  if (st != CPV_OK)
    return 0;
  for (size_t i = 0; i < n; i++) {
    st = cpv_set_parameter(s->v, names[i], values[i]);
    CHECK(st == CPV_OK, "set %s: %s", names[i], cpv_last_error(s->v));
  }
  st = cpv_initialize(s->v, 0.0);
  CHECK(st == CPV_OK, "initialize: %s", cpv_last_error(s->v));
  return st == CPV_OK;
}

static void sim_pwm(sim_t *s, double p1, double p2, double p3, double p4) {
  const double pwm[4] = {p1, p2, p3, p4};
  cpv_status_t st = cpv_set_pwm_vector(s->v, pwm, 4);
  CHECK(st == CPV_OK, "set pwm: %s", cpv_last_error(s->v));
}

static int sim_run(sim_t *s, double until) {
  while (s->t < until - 1e-9) {
    double reached = 0;
    s->t += DT;
    cpv_status_t st = cpv_advance_to(s->v, s->t, &reached);
    if (st != CPV_OK) {
      CHECK(st == CPV_OK, "advance to %.3f: %s", s->t, cpv_last_error(s->v));
      return 0;
    }
  }
  return 1;
}

static void sim_state(sim_t *s, cpv_kinematic_state_t *k) {
  cpv_status_t st = cpv_get_state(s->v, k);
  if (st != CPV_OK)
    CHECK(st == CPV_OK, "state: %s", cpv_last_error(s->v));
}

static double pitch_of(const double q[4]) {
  return asin(2 * (q[0] * q[2] - q[3] * q[1]));
}
static double yaw_of(const double q[4]) {
  return atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
}
static double roll_of(const double q[4]) {
  return atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]));
}

/* ---- rover ---------------------------------------------------------------- */

/* On four equally loaded wheels: sink m g / 4k, level, specific force -g. */
static void test_rover_rest(void) {
  sim_t s;
  if (!sim_open(&s, rover, NULL, NULL, 0))
    return;
  sim_run(&s, 3.0);
  cpv_kinematic_state_t k;
  cpv_imu_t imu;
  sim_state(&s, &k);
  cpv_get_imu(s.v, &imu);
  const double z = -R_CG_HEIGHT + R_MASS * G / (4 * R_K);
  CHECK(NEAR(k.position_ned[2], z, 2e-6), "rest height %.7f, expected %.7f", k.position_ned[2], z);
  CHECK(NEAR(k.position_ned[0], 0, 1e-6) && NEAR(k.position_ned[1], 0, 1e-6),
        "rover creeps at neutral: %g %g", k.position_ned[0], k.position_ned[1]);
  CHECK(NEAR(imu.accel_frd[2], -G, 1e-4) && NEAR(imu.accel_frd[0], 0, 1e-4),
        "rest specific force %g %g %g", imu.accel_frd[0], imu.accel_frd[1], imu.accel_frd[2]);
  CHECK(NEAR(k.quaternion_wb[0], 1, 1e-9), "rest attitude q0 %g", k.quaternion_wb[0]);
  cpv_close(s.v);
}

/* Full throttle, straight: the steady speed at which the voltage-driven DC
 * motor's torque, through the rear tyres' slip, balances front rolling
 * resistance and drag (with the pitch load transfer of drag at CG height). */
static double rover_top_speed(void) {
  const double a = R_CG_FRONT, h = R_CG_HEIGHT - R_MASS * G / (4 * R_K);
  double lo = 1.0, hi = 9.99;
  for (int i = 0; i < 200; i++) {
    double v = 0.5 * (lo + hi), sc = sqrt(v * v + V_RELAX * V_RELAX);
    double drag = 0.5 * RHO * R_CDA * sqrt(v * v + 1e-4) * v;
    double transfer = h * drag / (2 * a); /* N_rear - N_front */
    double nf = R_MASS * G / 4 - transfer / 2, nr = R_MASS * G / 4 + transfer / 2;
    double fx = (2 * nf * R_CRR * v / sc + drag) / 2; /* per rear wheel */
    double omega = v / R_RADIUS + fx * sc / (nr * R_KX * R_RADIUS);
    double residual = R_STALL * (1 - omega / R_NOLOAD) - 2 * fx * R_RADIUS;
    if (residual > 0)
      lo = v;
    else
      hi = v;
  }
  return 0.5 * (lo + hi);
}

static void test_rover_drive(void) {
  sim_t s;
  if (!sim_open(&s, rover, NULL, NULL, 0))
    return;
  sim_pwm(&s, 1500, 1500, 2000, 1500);
  sim_run(&s, 20.0);
  cpv_kinematic_state_t k;
  sim_state(&s, &k);
  const double expected = rover_top_speed();
  CHECK(NEAR(k.velocity_ned[0], expected, 2e-3 * expected), "top speed %.4f, expected %.4f",
        k.velocity_ned[0], expected);
  CHECK(fabs(k.velocity_ned[1]) < 1e-6 && fabs(yaw_of(k.quaternion_wb)) < 1e-6,
        "straight run drifts: vy %g yaw %g", k.velocity_ned[1], yaw_of(k.quaternion_wb));
  /* Accelerating, the body squats on the rear wheels: nose up. */
  cpv_close(s.v);

  if (!sim_open(&s, rover, NULL, NULL, 0))
    return;
  sim_pwm(&s, 1500, 1500, 2000, 1500);
  sim_run(&s, 0.5);
  sim_state(&s, &k);
  CHECK(pitch_of(k.quaternion_wb) > 0, "no rear squat under acceleration: pitch %g",
        pitch_of(k.quaternion_wb));
  sim_pwm(&s, 1500, 1500, 1000, 1500); /* full reverse */
  sim_run(&s, 4.0);
  sim_state(&s, &k);
  CHECK(k.velocity_ned[0] < -2.0, "full reverse does not reverse: vx %g", k.velocity_ned[0]);
  cpv_close(s.v);
}

/* Half right steering at walking speed: the single-track (Ackermann) yaw rate
 * v tan(delta)/L, slightly reduced by tyre slip (understeer), to the right. */
static void test_rover_turn(void) {
  sim_t s;
  if (!sim_open(&s, rover, NULL, NULL, 0))
    return;
  sim_pwm(&s, 1750, 1500, 1550, 1500);
  sim_run(&s, 10.0);
  cpv_kinematic_state_t k;
  sim_state(&s, &k);
  const double v = hypot(k.velocity_ned[0], k.velocity_ned[1]);
  const double kinematic = v * tan(0.5 * R_MAX_STEER) / R_WHEELBASE;
  const double r = k.angular_velocity_frd[2];
  CHECK(r > 0 && r <= kinematic && r > 0.97 * kinematic,
        "turn rate %.4f rad/s, single-track %.4f at %.3f m/s", r, kinematic, v);
  CHECK(roll_of(k.quaternion_wb) < 0, "body does not roll out of a right turn: %g",
        roll_of(k.quaternion_wb));
  cpv_close(s.v);
}

/* Elements 2 and 4 are not connected: extreme values change nothing. */
static void test_rover_unused_channels(void) {
  sim_t a, b;
  if (!sim_open(&a, rover, NULL, NULL, 0) || !sim_open(&b, rover, NULL, NULL, 0))
    return;
  sim_pwm(&a, 1600, 1500, 1700, 1500);
  sim_pwm(&b, 1600, 1000, 1700, 2000);
  sim_run(&a, 2.0);
  sim_run(&b, 2.0);
  cpv_kinematic_state_t ka, kb;
  sim_state(&a, &ka);
  sim_state(&b, &kb);
  CHECK(memcmp(&ka, &kb, sizeof(ka)) == 0, "unused channels change the rover: %g vs %g",
        ka.position_ned[0], kb.position_ned[0]);
  cpv_close(a.v);
  cpv_close(b.v);
}

/* ---- fixed-wing ----------------------------------------------------------- */

static void aero_coefficients(double alpha, double *cl, double *cd) {
  double e1 = exp(-P_M * (alpha - P_ASTALL)), e2 = exp(P_M * (alpha + P_ASTALL));
  double sigma = (1 + e1 + e2) / ((1 + e1) * (1 + e2));
  double attached = P_CL0 + P_CLA * alpha;
  *cl = (1 - sigma) * attached + sigma * 2 * sin(alpha) * fabs(sin(alpha)) * cos(alpha);
  *cd = P_CDP + attached * attached / (PI * P_E * P_B * P_B / P_S);
}

/* Pitch moment about the CG at zero rates and neutral controls, per qS. */
static double pitch_moment(double alpha) {
  double cl, cd;
  aero_coefficients(alpha, &cl, &cd);
  double cz = -cd * sin(alpha) - cl * cos(alpha);
  return P_C * (P_CM0 + P_CMA * alpha) - P_XREF * cz; /* r x F, r = (x_ref, 0, 0) */
}

/* At rest on the tricycle gear: solve the two static equations (vertical
 * force, pitch moment) for the CG height and pitch with the wheel contact
 * points directly below the hubs. */
static void plane_rest(double *z_out, double *theta_out) {
  const double xs[3] = {P_NOSE_X, P_MAIN_X, P_MAIN_X}, zh = P_CG_HEIGHT - P_RADIUS;
  double z = -P_CG_HEIGHT + 0.003, th = 0;
  for (int it = 0; it < 100; it++) {
    double f[2] = {0, 0}, j[2][2] = {{0, 0}, {0, 0}};
    for (int i = 0; i < 3; i++) {
      double pen = z - xs[i] * sin(th) + zh * cos(th) + P_RADIUS;
      double dx = xs[i] * cos(th) + zh * sin(th);
      double n = P_GEAR_K * pen;
      double dpen_dth = -xs[i] * cos(th) - zh * sin(th), ddx_dth = -xs[i] * sin(th) + zh * cos(th);
      f[0] += n;
      f[1] += dx * n;
      j[0][0] += P_GEAR_K;
      j[0][1] += P_GEAR_K * dpen_dth;
      j[1][0] += dx * P_GEAR_K;
      j[1][1] += ddx_dth * n + dx * P_GEAR_K * dpen_dth;
    }
    f[0] -= P_MASS * G;
    double det = j[0][0] * j[1][1] - j[0][1] * j[1][0];
    z -= (j[1][1] * f[0] - j[0][1] * f[1]) / det;
    th -= (-j[1][0] * f[0] + j[0][0] * f[1]) / det;
  }
  *z_out = z;
  *theta_out = th;
}

static void test_plane_rest(void) {
  sim_t s;
  if (!sim_open(&s, plane, NULL, NULL, 0))
    return;
  sim_run(&s, 4.0);
  cpv_kinematic_state_t k;
  cpv_imu_t imu;
  sim_state(&s, &k);
  cpv_get_imu(s.v, &imu);
  double z, th;
  plane_rest(&z, &th);
  CHECK(NEAR(pitch_of(k.quaternion_wb), th, 2e-5), "rest pitch %.6f, expected %.6f",
        pitch_of(k.quaternion_wb), th);
  CHECK(NEAR(k.position_ned[2], z, 2e-6), "rest height %.7f, expected %.7f", k.position_ned[2], z);
  CHECK(fabs(k.position_ned[0]) < 1e-3, "plane creeps at idle: %g", k.position_ned[0]);
  /* Specific force at rest is R^T (0, 0, -g) = (g sin(theta), 0, -g cos(theta)). */
  CHECK(NEAR(imu.accel_frd[0], G * sin(th), 1e-4) && NEAR(imu.accel_frd[2], -G * cos(th), 1e-4),
        "rest specific force %g %g, expected %g %g", imu.accel_frd[0], imu.accel_frd[2],
        G * sin(th), -G * cos(th));
  cpv_close(s.v);
}

/* Air start at 18 m/s, wings level, zero angle of attack, idle: the first
 * specific force is the attached-flow drag and lift, -qS (CD, 0, CL)/m. */
static void test_plane_air_start(void) {
  const char *names[] = {"start_altitude_m", "start_speed_mps"};
  const double values[] = {300, 18};
  sim_t s;
  if (!sim_open(&s, plane, names, values, 2))
    return;
  cpv_imu_t imu;
  cpv_get_imu(s.v, &imu);
  double cl, cd;
  aero_coefficients(0, &cl, &cd);
  const double qs = 0.5 * RHO * 18 * 18 * P_S;
  CHECK(NEAR(imu.accel_frd[0], -qs * cd / P_MASS, 1e-9) &&
            NEAR(imu.accel_frd[2], -qs * cl / P_MASS, 1e-9) && NEAR(imu.accel_frd[1], 0, 1e-12),
        "air-start specific force %.9f %.9f %.9f, expected %.9f 0 %.9f", imu.accel_frd[0],
        imu.accel_frd[1], imu.accel_frd[2], -qs * cd / P_MASS, -qs * cl / P_MASS);
  cpv_close(s.v);
}

/* Idle glide with neutral controls converges to the equilibrium where the
 * pitch moment vanishes (alpha*) and the aerodynamic force balances weight:
 * speed sqrt(2 m g / (rho S |C|)) and descent angle atan(CD/CL). */
static void test_plane_glide(void) {
  double lo = -0.2, hi = 0.2;
  for (int i = 0; i < 200; i++) {
    double mid = 0.5 * (lo + hi);
    if (pitch_moment(mid) > 0)
      lo = mid;
    else
      hi = mid;
  }
  /* The model's alpha is atan2(w, sqrt(u^2 + v0^2)); alpha is set by the
   * moment balance, so the forces follow from it directly. */
  const double alpha_model = 0.5 * (lo + hi);
  double cl, cd;
  aero_coefficients(alpha_model, &cl, &cd);
  const double cx = -cd * cos(alpha_model) + cl * sin(alpha_model);
  const double cz = -cd * sin(alpha_model) - cl * cos(alpha_model);
  const double speed = sqrt(2 * P_MASS * G / (RHO * P_S * hypot(cx, cz)));
  const double u = speed * cos(alpha_model);
  const double tan_alpha_true = tan(alpha_model) * sqrt(u * u + V_RELAX * V_RELAX) / u;
  const double theta = atan2(cx, -cz); /* body pitch at which F_aero = -m g */
  const double descent = atan(tan_alpha_true) - theta;

  const char *names[] = {"start_altitude_m", "start_speed_mps"};
  const double values[] = {400, 18};
  sim_t s;
  if (!sim_open(&s, plane, names, values, 2))
    return;
  sim_run(&s, 60.0);
  cpv_kinematic_state_t k;
  sim_state(&s, &k);
  const double v = sqrt(k.velocity_ned[0] * k.velocity_ned[0] + k.velocity_ned[2] * k.velocity_ned[2]);
  const double path = atan2(k.velocity_ned[2], k.velocity_ned[0]);
  CHECK(NEAR(v, speed, 1e-3 * speed), "glide speed %.4f, expected %.4f", v, speed);
  CHECK(NEAR(path, descent, 2e-4), "glide descent angle %.5f rad, expected %.5f", path, descent);
  CHECK(NEAR(pitch_of(k.quaternion_wb), theta, 2e-4), "glide pitch %.5f, expected %.5f",
        pitch_of(k.quaternion_wb), theta);
  CHECK(fabs(k.velocity_ned[1]) < 1e-6 && fabs(roll_of(k.quaternion_wb)) < 1e-6,
        "symmetric glide leaves the plane: vy %g roll %g", k.velocity_ned[1],
        roll_of(k.quaternion_wb));
  cpv_close(s.v);
}

/* Each surface channel, above its midpoint, gives its documented sense:
 * aileron rolls right, elevator pitches nose up, rudder yaws nose right;
 * throttle increases thrust. Compared with a neutral run at the same time. */
static void test_plane_controls(void) {
  const char *names[] = {"start_altitude_m", "start_speed_mps"};
  const double values[] = {300, 18};
  double rates[5][3];
  const double pwm[5][4] = {{1500, 1500, 1000, 1500},
                            {1750, 1500, 1000, 1500},
                            {1500, 1750, 1000, 1500},
                            {1500, 1500, 1000, 1750},
                            {1500, 1500, 2000, 1500}};
  double ax[5];
  for (int c = 0; c < 5; c++) {
    sim_t s;
    if (!sim_open(&s, plane, names, values, 2))
      return;
    sim_pwm(&s, pwm[c][0], pwm[c][1], pwm[c][2], pwm[c][3]);
    sim_run(&s, 0.3);
    cpv_kinematic_state_t k;
    cpv_imu_t imu;
    sim_state(&s, &k);
    cpv_get_imu(s.v, &imu);
    memcpy(rates[c], k.angular_velocity_frd, sizeof(rates[c]));
    ax[c] = imu.accel_frd[0];
    cpv_close(s.v);
  }
  CHECK(rates[1][0] - rates[0][0] > 0.5, "aileron above midpoint does not roll right: %g",
        rates[1][0] - rates[0][0]);
  CHECK(rates[2][1] - rates[0][1] > 0.5, "elevator above midpoint does not pitch up: %g",
        rates[2][1] - rates[0][1]);
  CHECK(rates[3][2] - rates[0][2] > 0.05, "rudder above midpoint does not yaw right: %g",
        rates[3][2] - rates[0][2]);
  CHECK(ax[4] - ax[0] > 5, "throttle does not add thrust: %g", ax[4] - ax[0]);
}

/* Full throttle on the runway: tracks the centreline and cannot leave the
 * ground before the lift at zero angle of attack (CL0) carries the weight.
 * With neutral elevator the aerodynamic nose-down moment holds the nose
 * wheel down; rotating with up elevator lifts off sooner. */
static double takeoff(double elevator_pwm, double *lateral, double *yaw) {
  sim_t s;
  if (!sim_open(&s, plane, NULL, NULL, 0))
    return -1;
  double z_rest, th;
  plane_rest(&z_rest, &th);
  sim_pwm(&s, 1500, elevator_pwm, 2000, 1500);
  double liftoff_speed = -1;
  cpv_kinematic_state_t k;
  memset(&k, 0, sizeof(k));
  while (s.t < 8.0 && sim_run(&s, s.t + DT)) {
    sim_state(&s, &k);
    if (k.position_ned[2] < z_rest - 0.05) {
      liftoff_speed = hypot(k.velocity_ned[0], k.velocity_ned[2]);
      break;
    }
  }
  *lateral = k.position_ned[1];
  *yaw = yaw_of(k.quaternion_wb);
  cpv_close(s.v);
  return liftoff_speed;
}

static void test_plane_takeoff(void) {
  const double cl0_speed = sqrt(2 * P_MASS * G / (RHO * P_S * P_CL0));
  double lateral, yaw;
  const double neutral = takeoff(1500, &lateral, &yaw);
  CHECK(neutral > cl0_speed && neutral < 25, "neutral-elevator liftoff at %.2f m/s (CL0: %.2f)",
        neutral, cl0_speed);
  CHECK(fabs(lateral) < 1e-6 && fabs(yaw) < 1e-6, "takeoff roll leaves the centreline: y %g yaw %g",
        lateral, yaw);
  const double rotated = takeoff(1700, &lateral, &yaw);
  CHECK(rotated > 0 && rotated < neutral, "up elevator does not lift off sooner: %.2f vs %.2f m/s",
        rotated, neutral);
}

int main(int argc, char **argv) {
  if (argc != 2) {
    fprintf(stderr, "usage: %s VEHICLES_DIR\n", argv[0]);
    return 2;
  }
  feenableexcept(FE_INVALID | FE_DIVBYZERO | FE_OVERFLOW);
  snprintf(rover, sizeof(rover), "%s/rover/vehicle.json", argv[1]);
  snprintf(plane, sizeof(plane), "%s/plane/vehicle.json", argv[1]);
  test_rover_rest();
  test_rover_drive();
  test_rover_turn();
  test_rover_unused_channels();
  test_plane_rest();
  test_plane_air_start();
  test_plane_glide();
  test_plane_controls();
  test_plane_takeoff();
  printf("ground/air: %d checks, %d failure(s)\n", checks, failures);
  return failures != 0;
}
