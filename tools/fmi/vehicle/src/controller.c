/* Generic multirotor flight controller; see controller.h for the structure. */
#include "controller.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const double DEG = M_PI / 180.0;

/* ---- small vector / matrix helpers ------------------------------------- */

static double clampd(double x, double lo, double hi) { return x < lo ? lo : x > hi ? hi : x; }
static double dot3(const double a[3], const double b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
static double norm3(const double a[3]) { return sqrt(dot3(a, a)); }
static void cross3(const double a[3], const double b[3], double out[3]) {
  double r[3] = {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
  memcpy(out, r, sizeof r);
}
/* m: 3x3 row-major (&M[0][0]) */
static void matvec(const double *m, const double v[3], double out[3]) {
  double r[3];
  for (int i = 0; i < 3; ++i)
    r[i] = m[3 * i] * v[0] + m[3 * i + 1] * v[1] + m[3 * i + 2] * v[2];
  memcpy(out, r, sizeof r);
}
static double wrap_pi(double a) { return atan2(sin(a), cos(a)); }

static void quat_to_dcm(const double q[4], double R[3][3]) {
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  R[0][0] = 1 - 2 * (y * y + z * z), R[0][1] = 2 * (x * y - w * z), R[0][2] = 2 * (x * z + w * y);
  R[1][0] = 2 * (x * y + w * z), R[1][1] = 1 - 2 * (x * x + z * z), R[1][2] = 2 * (y * z - w * x);
  R[2][0] = 2 * (x * z - w * y), R[2][1] = 2 * (y * z + w * x), R[2][2] = 1 - 2 * (x * x + y * y);
}

/* ZYX Euler angles (roll, pitch, yaw) to a body->NED quaternion. */
static void euler_to_quat(double roll, double pitch, double yaw, double q[4]) {
  const double cr = cos(roll / 2), sr = sin(roll / 2), cp = cos(pitch / 2), sp = sin(pitch / 2);
  const double cy = cos(yaw / 2), sy = sin(yaw / 2);
  q[0] = cr * cp * cy + sr * sp * sy;
  q[1] = sr * cp * cy - cr * sp * sy;
  q[2] = cr * sp * cy + sr * cp * sy;
  q[3] = cr * cp * sy - sr * sp * cy;
}

/* q <- q (x) exp(w dt / 2): body-rate propagation. */
static void quat_propagate(double q[4], const double w[3], double dt) {
  const double angle = norm3(w) * dt;
  double d[4] = {1, 0, 0, 0};
  if (angle > 1e-12) {
    const double s = sin(angle / 2) / norm3(w);
    d[0] = cos(angle / 2), d[1] = w[0] * s, d[2] = w[1] * s, d[3] = w[2] * s;
  }
  const double r[4] = {q[0] * d[0] - q[1] * d[1] - q[2] * d[2] - q[3] * d[3],
                       q[0] * d[1] + q[1] * d[0] + q[2] * d[3] - q[3] * d[2],
                       q[0] * d[2] - q[1] * d[3] + q[2] * d[0] + q[3] * d[1],
                       q[0] * d[3] + q[1] * d[2] - q[2] * d[1] + q[3] * d[0]};
  const double n = sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2] + r[3] * r[3]);
  for (int i = 0; i < 4; ++i)
    q[i] = r[i] / n;
}

static double heading(const fc_t *fc) { return atan2(fc->R[1][0], fc->R[0][0]); }

/* In-place inverse of a 4x4 matrix (Gauss-Jordan, partial pivoting). */
static int invert4(double a[4][4]) {
  double inv[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
  for (int c = 0; c < 4; ++c) {
    int p = c;
    for (int r = c + 1; r < 4; ++r)
      if (fabs(a[r][c]) > fabs(a[p][c]))
        p = r;
    if (fabs(a[p][c]) < 1e-12)
      return -1;
    for (int k = 0; k < 4; ++k) {
      double t = a[c][k];
      a[c][k] = a[p][k], a[p][k] = t;
      t = inv[c][k];
      inv[c][k] = inv[p][k], inv[p][k] = t;
    }
    const double d = a[c][c];
    for (int k = 0; k < 4; ++k)
      a[c][k] /= d, inv[c][k] /= d;
    for (int r = 0; r < 4; ++r)
      if (r != c) {
        const double f = a[r][c];
        for (int k = 0; k < 4; ++k)
          a[r][k] -= f * a[c][k], inv[r][k] -= f * inv[c][k];
      }
  }
  memcpy(a, inv, sizeof inv);
  return 0;
}

/* ---- setup ---------------------------------------------------------------- */

static double rotor_command(const vehicle_t *v, int i, double omega) {
  if (omega <= 0)
    return 0;
  return clampd(v->cmd_c0[i] + v->cmd_c1[i] * omega + v->cmd_c2[i] * omega * omega, 0, 1);
}

int fc_init(fc_t *fc, const vehicle_t *v, const mission_t *mission, double dt) {
  memset(fc, 0, sizeof *fc);
  fc->v = v, fc->mission = mission, fc->dt = dt;

  /* Bandwidths. The rate loop sees the actuator lag tau (after the lead
   * compensation of slower motors): for a P rate loop on 1/(s (tau s + 1)) a
   * gain of 0.5/tau gives damping ~0.7. The outer loops are separated from it
   * by ~3-4x each. */
  const double tau_m = v->motor_tau > 0 ? v->motor_tau : FC_ACTUATOR_TAU;
  const double tau = fc->tau_a = fmin(tau_m, fmax(FC_ACTUATOR_TAU, tau_m / FC_MAX_LEAD));
  fc_gains_t *k = &fc->k;
  k->rate = clampd(0.5 / tau, 4.0, 20.0);
  k->rate_i = 0.3 * k->rate;
  k->att = k->rate / 2.5;
  k->att_yaw = 0.5 * k->att;
  k->vel_xy = k->att / 3.0;
  k->vel_xy_i = k->vel_xy * k->vel_xy / 40.0; /* slow trim only: it winds up while tracking */
  k->pos_xy = k->vel_xy / 3.0;
  k->vel_z = clampd(0.35 / tau, 1.5, 4.0);
  k->vel_z_i = 0.25 * k->vel_z * k->vel_z; /* critically damped PI */
  k->pos_z = k->vel_z / 3.0;
  k->ref = k->att; /* the reference moves no faster than the attitude can follow */
  k->max_rate[0] = k->max_rate[1] = 3.0, k->max_rate[2] = 1.5;
  k->est_acc = 0.5, k->est_mag = 0.3;

  /* Effectiveness B (4 x n): collective thrust along body -z, then the body
   * moment about the CG per newton of rotor thrust (arm x direction plus the
   * drag reaction). Allocation = pseudo-inverse B' (B B')^-1. */
  double B[4][VEHICLE_MAX_ROTORS];
  for (int i = 0; i < v->n_rotors; ++i) {
    double arm[3];
    cross3(v->position[i], v->thrust_dir[i], arm);
    B[0][i] = -v->thrust_dir[i][2];
    for (int a = 0; a < 3; ++a)
      B[1 + a][i] = arm[a] + v->drag_moment[i][a];
    fc->thrust_max[i] = 0.95 * v->kT[i] * v->omega_max[i] * v->omega_max[i];
    fc->thrust_min[i] = 0.0;
  }
  double M[4][4];
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c) {
      M[r][c] = 0;
      for (int i = 0; i < v->n_rotors; ++i)
        M[r][c] += B[r][i] * B[c][i];
    }
  if (invert4(M))
    return -1;
  for (int i = 0; i < v->n_rotors; ++i)
    for (int c = 0; c < 4; ++c) {
      fc->alloc[i][c] = 0;
      for (int r = 0; r < 4; ++r)
        fc->alloc[i][c] += B[r][i] * M[r][c];
    }
  fc->yaw_scale = 1;
  return 0;
}

/* ---- estimator ------------------------------------------------------------- */

static void gnss_to_ned(const vehicle_t *v, const double gps[3], double ned[3]) {
  ned[0] = (gps[0] - v->lat0_deg) * DEG * v->earth_radius_m;
  ned[1] = (gps[1] - v->lon0_deg) * DEG * v->earth_radius_m * cos(v->lat0_deg * DEG);
  ned[2] = v->ground_alt_m - gps[2];
}

/* CG position (absolute local NED, height from the barometer) and velocity. */
static void cg_state(const fc_t *fc, const fc_sensors_t *s, double p[3], double vel[3]) {
  const vehicle_t *v = fc->v;
  double lever[3], w_x_r[3];
  gnss_to_ned(v, s->gps, p);
  matvec(&fc->R[0][0], v->gnss_position, lever);
  cross3(s->gyro, v->gnss_position, w_x_r);
  matvec(&fc->R[0][0], w_x_r, w_x_r);
  for (int a = 0; a < 3; ++a) {
    p[a] -= lever[a];
    vel[a] = s->vel_ned[a] - w_x_r[a];
  }
  matvec(&fc->R[0][0], v->baro_position, lever);
  p[2] = -s->baro_alt - lever[2];
}

static void estimator_init(fc_t *fc, const fc_sensors_t *s) {
  /* Tilt from the specific force (at rest f = -g R' e3), heading from the
   * FMU's heading output; the magnetometer reference is learned here. */
  const double f = norm3(s->accel);
  if (fabs(f - fc->v->gravity) > 0.1 * fc->v->gravity)
    return; /* not at rest (e.g. still dropping onto the gear): wait */
  const double d[3] = {-s->accel[0] / f, -s->accel[1] / f, -s->accel[2] / f};
  euler_to_quat(atan2(d[1], d[2]), -asin(clampd(d[0], -1, 1)), s->yaw_deg * DEG, fc->q);
  quat_to_dcm(fc->q, fc->R);
  matvec(&fc->R[0][0], s->mag, fc->mag_ref);
  fc->mag_ref[2] = 0;
  fc->initialized = true;
}

static void estimator_update(fc_t *fc, const fc_sensors_t *s) {
  const double dt = fc->dt, g = fc->v->gravity;
  double w[3] = {s->gyro[0], s->gyro[1], s->gyro[2]};
  /* Accelerometer: measured down direction vs. estimated R' e3 (Mahony). */
  const double f = norm3(s->accel);
  if (fabs(f - g) < 0.15 * g) {
    const double meas[3] = {-s->accel[0] / f, -s->accel[1] / f, -s->accel[2] / f};
    const double est[3] = {fc->R[2][0], fc->R[2][1], fc->R[2][2]};
    double e[3];
    cross3(meas, est, e);
    for (int a = 0; a < 3; ++a)
      w[a] += fc->k.est_acc * e[a];
  }
  /* Magnetometer: heading error of the horizontal field, about world down. */
  double m_w[3];
  matvec(&fc->R[0][0], s->mag, m_w);
  const double hm = hypot(m_w[0], m_w[1]), hr = hypot(fc->mag_ref[0], fc->mag_ref[1]);
  if (hm > 1e-9 && hr > 1e-9) {
    const double e = (m_w[0] * fc->mag_ref[1] - m_w[1] * fc->mag_ref[0]) / (hm * hr);
    for (int a = 0; a < 3; ++a)
      w[a] += fc->k.est_mag * e * fc->R[2][a];
  }
  quat_propagate(fc->q, w, dt);
  quat_to_dcm(fc->q, fc->R);
}

/* ---- reference ------------------------------------------------------------- */

/* Moves ref (n axes) toward target with a speed limit and an acceleration
 * limit that also brakes in time; writes the reference velocity. */
static void approach(double *ref, double *vel, const double *target, int n, double vmax, double amax,
                     double dt) {
  double d[3], dist = 0, speed = 0;
  for (int a = 0; a < n; ++a) {
    d[a] = target[a] - ref[a];
    dist += d[a] * d[a];
    speed += vel[a] * vel[a];
  }
  dist = sqrt(dist), speed = sqrt(speed);
  if (dist < 1e-9) {
    for (int a = 0; a < n; ++a)
      ref[a] = target[a], vel[a] = 0;
    return;
  }
  const double want = fmin(vmax, sqrt(2 * amax * dist));
  speed = clampd(want, speed - amax * dt, speed + amax * dt);
  const double move = fmin(speed * dt, dist);
  for (int a = 0; a < n; ++a) {
    ref[a] += d[a] / dist * move;
    vel[a] = d[a] / dist * move / dt;
  }
}

static void arm(fc_t *fc, const fc_sensors_t *s) {
  double p[3], vel[3];
  cg_state(fc, s, p, vel);
  memcpy(fc->home_gnss, p, sizeof p);
  memset(fc->ref_pos, 0, sizeof fc->ref_pos);
  memset(fc->ref_vel, 0, sizeof fc->ref_vel);
  memset(fc->path_pos, 0, sizeof fc->path_pos);
  memset(fc->path_vel, 0, sizeof fc->path_vel);
  memset(fc->int_vel, 0, sizeof fc->int_vel);
  memset(fc->int_rate, 0, sizeof fc->int_rate);
  memset(fc->lead_state, 0, sizeof fc->lead_state); /* rotors at rest */
  fc->yaw0 = fc->ref_yaw = heading(fc);
  fc->armed = true, fc->landing = false, fc->landed = false, fc->land_timer = 0;
}

static void reference_update(fc_t *fc, const mission_phase_t *ph) {
  const mission_t *m = fc->mission;
  const double dt = fc->dt, w = fc->k.ref;
  const double target[3] = {ph->target[0], ph->target[1], -ph->target[2]};
  approach(fc->path_pos, fc->path_vel, target, 2, m->speed_h, m->accel, dt);
  if (ph->kind == PHASE_LAND) {
    /* Descend at land_speed (accelerating gently); the ground ends it. */
    fc->path_vel[2] = fmin(m->land_speed, fc->path_vel[2] + m->accel * dt);
    fc->path_pos[2] = fc->ref_pos[2] + fc->path_vel[2] * dt;
  } else {
    approach(&fc->path_pos[2], &fc->path_vel[2], &target[2], 1, m->speed_v, m->accel, dt);
  }
  /* ref'' = w^2 (path - ref) + 2 w (path' - ref'): critically damped, so the
   * reference never overshoots the path and its acceleration is continuous. */
  for (int a = 0; a < 3; ++a) {
    fc->ref_acc[a] = w * w * (fc->path_pos[a] - fc->ref_pos[a]) + 2 * w * (fc->path_vel[a] - fc->ref_vel[a]);
    fc->ref_vel[a] += fc->ref_acc[a] * dt;
    fc->ref_pos[a] += fc->ref_vel[a] * dt;
  }
  const double yaw_target = fc->yaw0 + ph->yaw_deg * DEG;
  const double step = m->yaw_rate_deg * DEG * dt;
  fc->ref_yaw = wrap_pi(fc->ref_yaw + clampd(wrap_pi(yaw_target - fc->ref_yaw), -step, step));
}

/* ---- control ------------------------------------------------------------- */

static void allocate(fc_t *fc, const double wrench[4]) {
  const vehicle_t *v = fc->v;
  double base[VEHICLE_MAX_ROTORS], yaw[VEHICLE_MAX_ROTORS], s = 1;
  for (int i = 0; i < v->n_rotors; ++i) {
    base[i] = fc->alloc[i][0] * wrench[0] + fc->alloc[i][1] * wrench[1] + fc->alloc[i][2] * wrench[2];
    yaw[i] = fc->alloc[i][3] * wrench[3];
    /* Largest yaw fraction that keeps this rotor inside its thrust range. */
    const double t = base[i] + yaw[i];
    if (t > fc->thrust_max[i] && yaw[i] > 0)
      s = fmin(s, fmax(0, (fc->thrust_max[i] - base[i]) / yaw[i]));
    if (t < fc->thrust_min[i] && yaw[i] < 0)
      s = fmin(s, fmax(0, (fc->thrust_min[i] - base[i]) / yaw[i]));
  }
  fc->yaw_scale = s;
  fc->saturated = 0;
  for (int i = 0; i < v->n_rotors; ++i) {
    double t = base[i] + s * yaw[i];
    if (t > fc->thrust_max[i] || t < fc->thrust_min[i])
      fc->saturated++;
    t = clampd(t, fc->thrust_min[i], fc->thrust_max[i]);
    fc->rotor_thrust[i] = t;
    /* Lead-lag (tau_m s + 1)/(tau_a s + 1) on the speed demand: the lagged
     * state x follows the demand w with tau_a; the output
     * (tau_m/tau_a) w + (1 - tau_m/tau_a) x cancels the motor pole. */
    double w = sqrt(t / v->kT[i]);
    if (v->motor_tau > fc->tau_a) {
      const double ratio = v->motor_tau / fc->tau_a;
      const double x = fc->lead_state[i];
      fc->lead_state[i] = x + (w - x) * (1 - exp(-fc->dt / fc->tau_a));
      w = fmax(0, ratio * w + (1 - ratio) * x);
    }
    fc->command[i] = rotor_command(v, i, w);
  }
}

static void control(fc_t *fc, const fc_sensors_t *s) {
  const vehicle_t *v = fc->v;
  const fc_gains_t *k = &fc->k;
  const double dt = fc->dt, g = v->gravity, m = v->mass;
  const bool airborne = -fc->pos[2] > 0.3;

  /* Position -> velocity setpoint. */
  double vsp[3];
  vsp[0] = fc->ref_vel[0] + k->pos_xy * (fc->ref_pos[0] - fc->pos[0]);
  vsp[1] = fc->ref_vel[1] + k->pos_xy * (fc->ref_pos[1] - fc->pos[1]);
  vsp[2] = fc->landing ? fc->ref_vel[2] : fc->ref_vel[2] + k->pos_z * (fc->ref_pos[2] - fc->pos[2]);
  const double vh = hypot(vsp[0], vsp[1]), vh_max = 2.0 * fc->mission->speed_h;
  if (vh > vh_max)
    vsp[0] *= vh_max / vh, vsp[1] *= vh_max / vh;
  vsp[2] = clampd(vsp[2], -2.0 * fc->mission->speed_v, 2.0 * fc->mission->speed_v);

  /* Velocity -> acceleration (PI + reference feedforward). */
  double acc[3];
  for (int a = 0; a < 3; ++a) {
    const double e = vsp[a] - fc->vel[a];
    const bool z = a == 2;
    const double ki = z ? k->vel_z_i : k->vel_xy_i, kp = z ? k->vel_z : k->vel_xy;
    const double limit = z ? 0.4 * g : 2.0;
    if (z || airborne)
      fc->int_vel[a] = clampd(fc->int_vel[a] + e * dt, -limit / ki, limit / ki);
    acc[a] = fc->ref_acc[a] + kp * e + ki * fc->int_vel[a];
  }

  /* Thrust vector F = m (a - g e3) with a tilt limit, then the desired
   * attitude: body z along -F, heading ref_yaw. */
  double F[3] = {m * acc[0], m * acc[1], m * (acc[2] - g)};
  F[2] = fmin(F[2], -0.1 * m * g);
  const double fh = hypot(F[0], F[1]), fh_max = -F[2] * tan(fc->mission->tilt_limit_deg * DEG);
  if (fh > fh_max)
    F[0] *= fh_max / fh, F[1] *= fh_max / fh;
  const double fn = norm3(F);
  const double b3[3] = {-F[0] / fn, -F[1] / fn, -F[2] / fn};
  const double b3_now[3] = {fc->R[0][2], fc->R[1][2], fc->R[2][2]};
  fc->thrust = fmax(0, -dot3(F, b3_now));
  const double b1c[3] = {cos(fc->ref_yaw), sin(fc->ref_yaw), 0};
  double b2[3], b1[3];
  cross3(b3, b1c, b2);
  const double n2 = norm3(b2);
  for (int a = 0; a < 3; ++a)
    b2[a] /= n2;
  cross3(b2, b3, b1);
  const double Rd[3][3] = {{b1[0], b2[0], b3[0]}, {b1[1], b2[1], b3[1]}, {b1[2], b2[2], b3[2]}};
  fc->tilt_deg = acos(clampd(fc->R[2][2], -1, 1)) / DEG;

  /* Attitude error e_R = vee(Rd' R - R' Rd) / 2 -> body-rate setpoint. */
  double E[3][3];
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      double a = 0, b = 0;
      for (int l = 0; l < 3; ++l)
        a += Rd[l][i] * fc->R[l][j], b += fc->R[l][i] * Rd[l][j];
      E[i][j] = 0.5 * (a - b);
    }
  const double eR[3] = {E[2][1], E[0][2], E[1][0]};
  const double katt[3] = {k->att, k->att, k->att_yaw};
  for (int a = 0; a < 3; ++a)
    fc->rate_sp[a] = clampd(-katt[a] * eR[a], -k->max_rate[a], k->max_rate[a]);

  /* Rate PI -> torque J alpha + w x J w. */
  double alpha[3], Jw[3], wJw[3];
  for (int a = 0; a < 3; ++a) {
    const double e = fc->rate_sp[a] - s->gyro[a];
    if (airborne)
      fc->int_rate[a] = clampd(fc->int_rate[a] + e * dt, -1.0, 1.0);
    alpha[a] = k->rate * e + k->rate_i * fc->int_rate[a];
  }
  matvec(&v->inertia[0][0], alpha, fc->torque);
  matvec(&v->inertia[0][0], s->gyro, Jw);
  cross3(s->gyro, Jw, wJw);
  for (int a = 0; a < 3; ++a)
    fc->torque[a] += wJw[a];

  const double wrench[4] = {fc->thrust, fc->torque[0], fc->torque[1], fc->torque[2]};
  allocate(fc, wrench);
}

void fc_update(fc_t *fc, const fc_sensors_t *s, const mission_phase_t *ph, double pwm[]) {
  const vehicle_t *v = fc->v;
  if (!fc->initialized)
    estimator_init(fc, s);
  else
    estimator_update(fc, s);
  for (int c = 0; c < v->n_actuators; ++c)
    pwm[c] = v->pwm_min;
  if (!fc->initialized)
    return;

  const bool want_armed = ph->kind != PHASE_IDLE && !fc->landed;
  if (want_armed && !fc->armed)
    arm(fc, s);
  if (ph->kind == PHASE_IDLE) {
    fc->armed = false;
    fc->landed = false; /* a later flying phase may arm again */
  }

  double p[3];
  cg_state(fc, s, p, fc->vel);
  for (int a = 0; a < 3; ++a)
    fc->pos[a] = p[a] - fc->home_gnss[a];

  if (!fc->armed) {
    for (int i = 0; i < v->n_rotors; ++i)
      fc->command[i] = 0, fc->rotor_thrust[i] = 0;
    fc->thrust = 0;
    return;
  }

  fc->landing = ph->kind == PHASE_LAND;
  reference_update(fc, ph);
  control(fc, s);

  /* Touchdown: within 15 cm of home height and not moving vertically for
   * 0.5 s while descending -> disarm. */
  if (fc->landing) {
    const bool still = -fc->pos[2] < 0.15 && fabs(fc->vel[2]) < 0.1;
    fc->land_timer = still ? fc->land_timer + fc->dt : 0;
    if (fc->land_timer >= 0.5) {
      fc->armed = false, fc->landed = true;
      return;
    }
  }

  /* Rotor commands -> PWM channels (rotors sharing a channel are averaged). */
  double sum[VEHICLE_MAX_ACTUATORS] = {0};
  int count[VEHICLE_MAX_ACTUATORS] = {0};
  for (int i = 0; i < v->n_rotors; ++i)
    sum[v->actuator[i]] += fc->command[i], count[v->actuator[i]]++;
  for (int c = 0; c < v->n_actuators; ++c)
    if (count[c])
      pwm[c] = v->pwm_min + sum[c] / count[c] * (v->pwm_max - v->pwm_min);
}
