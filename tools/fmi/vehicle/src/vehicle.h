/* Vehicle and mission descriptions for the generic multirotor harness.
 *
 * Both are plain data. tools/vehicle.py fills them per configuration into the
 * generated header vehicle_config.h (VEHICLE from the fire-compose manifest,
 * MISSION from mission.toml); nothing here depends on a particular airframe.
 *
 * Frames follow the library (docs/architecture.md): world NED, body FRD, all
 * body positions relative to the combined centre of gravity (CG).
 */
#ifndef VEHICLE_H
#define VEHICLE_H

#define VEHICLE_MAX_ROTORS 16
#define VEHICLE_MAX_ACTUATORS 32
#define MISSION_MAX_PHASES 32

typedef struct {
  const char *name;       /* Modelica model name, e.g. ConfiguredQuad */
  const char *config;     /* configuration file it was composed from */
  const char *propulsion; /* fidelity P0..P4 */
  int n_rotors;
  int n_actuators; /* PWM channels of the FMU (pwm[1..n_actuators]) */

  double mass;          /* total mass [kg] */
  double inertia[3][3]; /* about the CG, body axes [kg m2] */
  double gravity;       /* [m/s2] */

  /* Per rotor i (0-based): */
  double position[VEHICLE_MAX_ROTORS][3];   /* hub relative to the CG, body [m] */
  double thrust_dir[VEHICLE_MAX_ROTORS][3]; /* unit thrust direction, body (R_br e3) */
  double drag_moment[VEHICLE_MAX_ROTORS][3]; /* reaction moment per newton of thrust, body
                                                (R_br (0,0,-spinSign) kQ/kT) [m] */
  int spin_sign[VEHICLE_MAX_ROTORS];
  int actuator[VEHICLE_MAX_ROTORS]; /* PWM channel, 0-based (geometry.actuatorIndex - 1) */
  double kT[VEHICLE_MAX_ROTORS];    /* thrust = kT omega^2 [N s2] */
  double kQ[VEHICLE_MAX_ROTORS];    /* drag torque = kQ omega^2 [N m s2] */
  /* Steady-state normalized command for a rotor speed omega (from the ESC and
   * motor models of the propulsion fidelity, at nominal supply):
   *   command = cmd_c0 + cmd_c1 omega + cmd_c2 omega^2 */
  double cmd_c0[VEHICLE_MAX_ROTORS], cmd_c1[VEHICLE_MAX_ROTORS], cmd_c2[VEHICLE_MAX_ROTORS];
  double omega_max[VEHICLE_MAX_ROTORS]; /* steady speed at command 1 [rad/s] */
  double hover_omega[VEHICLE_MAX_ROTORS]; /* allocation of m g, level hover [rad/s] */
  double hover_command[VEHICLE_MAX_ROTORS];
  double motor_tau; /* estimated small-signal speed time constant at hover [s] */

  double pwm_min, pwm_max; /* PWM calibration [us]: command = (pwm - min)/(max - min) */

  /* GNSS: local NED from lat/lon/alt (inverse of Exports.FMU.PartialCopterInterface). */
  double lat0_deg, lon0_deg, ground_alt_m, earth_radius_m;
  double gnss_position[3]; /* antenna relative to the CG, body [m] */
  double baro_position[3]; /* barometer relative to the CG, body [m] */
} vehicle_t;

typedef enum {
  PHASE_IDLE, /* disarmed: every channel at pwm_min */
  PHASE_FLY,  /* armed: fly to the target and hold it */
  PHASE_LAND  /* armed: descend at land_speed, disarm on touchdown */
} phase_kind_t;

typedef struct {
  const char *name;
  phase_kind_t kind;
  double start_s, duration_s;
  double target[3]; /* north, east, up relative to the arming point (home) [m] */
  double yaw_deg;   /* heading relative to the heading at arming, clockwise [deg] */
} mission_phase_t;

typedef struct {
  const char *name;
  double step_s;         /* controller period = communication step [s] */
  double stop_s;         /* end of the last phase [s] */
  double speed_h;        /* reference horizontal speed limit [m/s] */
  double speed_v;        /* reference vertical speed limit [m/s] */
  double accel;          /* reference acceleration limit [m/s2] */
  double yaw_rate_deg;   /* reference yaw rate limit [deg/s] */
  double land_speed;     /* descent rate in PHASE_LAND [m/s] */
  double tilt_limit_deg; /* commanded tilt limit [deg] */
  int phase_count;
  mission_phase_t phases[MISSION_MAX_PHASES];
} mission_t;

#endif
