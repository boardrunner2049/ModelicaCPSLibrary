/* Generic multirotor flight controller (firmware-in-the-loop style).
 *
 * Input: the FMU's sensor outputs only (IMU, magnetometer, GNSS, barometer,
 * heading); never the truth record. Output: one PWM pulse width per channel.
 * Everything vehicle-specific comes from a vehicle_t (vehicle.h):
 *
 *   estimator   complementary attitude filter (gyro integration, accelerometer
 *               tilt and magnetometer heading corrections); position and
 *               velocity from GNSS, height from the barometer, both corrected
 *               for the sensor lever arms
 *   reference   speed/acceleration-limited path to the phase target, smoothed
 *               by a critically damped 2nd-order filter at the attitude
 *               bandwidth; it supplies continuous velocity and acceleration
 *               feedforward
 *   position    P   -> velocity setpoint
 *   velocity    PI  -> acceleration setpoint -> thrust vector m (a - g)
 *   attitude    P on the SO(3) error (Lee et al.) -> body-rate setpoint
 *   rate        PI  -> angular acceleration -> torque J alpha + w x J w
 *   allocation  pseudo-inverse of the rotor effectiveness matrix (collective
 *               thrust and three torques per rotor thrust); yaw is scaled
 *               back first when a rotor would saturate
 *   motor map   thrust -> speed sqrt(T/kT) -> lead compensation of the motor
 *               lag -> steady-state command of the propulsion fidelity -> PWM,
 *               routed through actuatorIndex
 *
 * Gains are bandwidths [1/s]; mass and inertia turn them into forces and
 * torques, so the same law flies every configuration. Motors slower than
 * FC_ACTUATOR_TAU (the voltage-driven P2/P3 machines: tau = J/(k^2/R + 2 kQ w))
 * get a per-rotor lead-lag (tau s + 1)/(tau_a s + 1) on the speed demand, so
 * the loops see an actuator lag tau_a and keep the same bandwidths; the rate
 * bandwidth is derived from tau_a. The vertical-velocity and body-rate
 * integrators absorb the difference between the nominal command map and the
 * actual propulsion (voltage-driven P1..P3 motors, supply droop, CG offsets).
 */

/* Target actuator time constant [s] and the largest lead ratio tau/tau_a. */
#define FC_ACTUATOR_TAU 0.03
#define FC_MAX_LEAD 5.0
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "vehicle.h"

#include <stdbool.h>

typedef struct {
  double accel[3];   /* specific force, body FRD [m/s2] */
  double gyro[3];    /* body FRD [rad/s] */
  double mag[3];     /* body FRD [Gauss] */
  double gps[3];     /* latitude, longitude [deg], WGS84 altitude [m] */
  double vel_ned[3]; /* GNSS velocity [m/s] */
  double yaw_deg;    /* heading output of the FMU (used once, to initialize) */
  double baro_alt;   /* barometric altitude, up [m] */
} fc_sensors_t;

typedef struct {
  /* bandwidths [1/s] (integral gains [1/s2]) */
  double pos_xy, vel_xy, vel_xy_i, pos_z, vel_z, vel_z_i, att, att_yaw, rate, rate_i;
  double ref; /* reference filter natural frequency */
  double max_rate[3];  /* body-rate setpoint limit [rad/s] */
  double est_acc, est_mag; /* complementary filter gains [1/s] */
} fc_gains_t;

typedef struct {
  const vehicle_t *v;
  const mission_t *mission;
  fc_gains_t k;
  double dt;
  double alloc[VEHICLE_MAX_ROTORS][4]; /* pseudo-inverse: rotor thrust per (T, Mx, My, Mz) */
  double thrust_max[VEHICLE_MAX_ROTORS], thrust_min[VEHICLE_MAX_ROTORS];

  /* estimator */
  bool initialized;
  double q[4];        /* body -> NED, scalar first */
  double R[3][3];     /* same as a matrix */
  double mag_ref[3];  /* horizontal reference field, NED (learned at initialization) */
  double pos[3], vel[3]; /* CG relative to home, NED */
  double home_gnss[3], home_baro;
  double yaw0;        /* heading at arming [rad] */

  /* reference */
  bool armed, landing, landed;
  double path_pos[3], path_vel[3]; /* speed/acceleration-limited path to the target */
  double ref_pos[3], ref_vel[3], ref_acc[3], ref_yaw; /* the path through a 2nd-order filter */
  double land_timer;

  /* control state */
  double int_vel[3], int_rate[3];
  double thrust;          /* commanded collective thrust [N] */
  double rate_sp[3], torque[3], tilt_deg;
  double tau_a;           /* actuator lag the loops are designed for [s] */
  double lead_state[VEHICLE_MAX_ROTORS]; /* lagged speed demand of the lead-lag [rad/s] */
  double rotor_thrust[VEHICLE_MAX_ROTORS], command[VEHICLE_MAX_ROTORS];
  int saturated;          /* rotors clipped in the last allocation */
  double yaw_scale;       /* fraction of the yaw torque the allocation kept */
} fc_t;

/* Derives gains and the allocation; returns 0, or -1 if the effectiveness
 * matrix is singular (the vehicle cannot control all four axes). */
int fc_init(fc_t *fc, const vehicle_t *v, const mission_t *mission, double dt);

/* One controller period. Runs the estimator; while armed, flies `phase`.
 * Writes one pulse width per PWM channel into pwm[0..n_actuators-1]. */
void fc_update(fc_t *fc, const fc_sensors_t *s, const mission_phase_t *phase, double pwm[]);

#endif
