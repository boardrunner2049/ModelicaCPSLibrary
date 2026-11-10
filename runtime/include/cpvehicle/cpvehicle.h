/* CP Models vehicle runtime: load a vehicle FMU into the host process and
 * drive it through the vehicle interface contract (contract/vehicle_interface.toml).
 *
 * The runtime consumes a descriptor written by the checker
 * (python -m cp_vehicle_interface prepare MODEL.fmu OUT_DIR). It verifies that
 * the descriptor is bound to the files it loads (SHA-256), that the host's
 * required capabilities are provided, and that the instance declares the
 * contract version it was checked against. It never guesses a value reference.
 * Alternatively a model can be linked into the program and opened with
 * cpv_open_model (cpvehicle_model.h); everything after opening is the same.
 *
 * Lifecycle:  cpv_open -> [cpv_set_parameter] -> cpv_initialize ->
 *             { set inputs; cpv_advance_to; read outputs } -> [cpv_reset] -> cpv_close
 *
 * Concurrency: an instance is not reentrant; a host that shares one instance
 * between threads serializes every call on it. Distinct instances are
 * independent as far as the FMU itself keeps per-instance state; the runtime
 * keeps no global mutable state.
 */
#ifndef CPVEHICLE_H
#define CPVEHICLE_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CPV_CONTRACT_NAME "cp_vehicle_interface"
#define CPV_CONTRACT_VERSION 1
#define CPV_DESCRIPTOR_SCHEMA 1

typedef struct cpv_vehicle cpv_vehicle_t;

typedef enum {
  CPV_OK = 0,
  CPV_WARNING,          /* completed; the FMU reported fmi3Warning (see cpv_last_error) */
  CPV_UNSUPPORTED,      /* the model does not provide this capability or field */
  CPV_ERR_ARGUMENT,     /* invalid argument: null pointer, wrong count, non-finite, past time */
  CPV_ERR_DESCRIPTOR,   /* malformed descriptor, unknown version, or not bound to the files */
  CPV_ERR_CAPABILITY,   /* a capability the host requires is not provided */
  CPV_ERR_LOAD,         /* shared library or FMI symbols could not be loaded or instantiated */
  CPV_ERR_STATE,        /* the call is not valid in the instance's lifecycle state */
  CPV_ERR_FMU,          /* the FMU returned an error, discard or fatal status, or terminated */
  CPV_ERR_NOMEM
} cpv_status_t;

/* FMU log messages. fmi_status is the fmi3Status of the message (0 OK .. 4 Fatal). */
typedef void (*cpv_log_fn)(void *user, int fmi_status, const char *category, const char *message);

typedef struct {
  /* Capabilities the host needs, e.g. "pwm_actuation_v1". Opening fails if any is missing. */
  const char *const *required_capabilities;
  size_t required_count;
  const char *instance_name; /* NULL: "cpvehicle" */
  /* Largest single fmi3DoStep; cpv_advance_to splits longer intervals. 0: no limit. */
  double max_step_s;
  /* Accept a descriptor prepared with a legacy profile (no version marker). */
  int allow_legacy_profile;
  cpv_log_fn log; /* NULL: messages with status >= warning go to stderr */
  void *log_user;
} cpv_options_t;

/* Fills defaults: no required capabilities, 2 ms maximum step, no legacy profile. */
void cpv_options_init(cpv_options_t *options);

/* Validate the descriptor, load the library and instantiate the FMU.
 * On failure *out is NULL and error (if non-NULL) holds the reasons. */
cpv_status_t cpv_open(const char *descriptor_path, const cpv_options_t *options,
                      cpv_vehicle_t **out, char *error, size_t error_size);

/* Before cpv_initialize: override a scalar Float64 parameter the descriptor
 * lists as overridable. Applied on top of the descriptor's own overrides. */
cpv_status_t cpv_set_parameter(cpv_vehicle_t *vehicle, const char *name, double value);

/* Enter initialization at start_time_s, apply overrides and the current
 * input vectors (initially the model's start values), exit initialization,
 * and check the version marker of the live instance. */
cpv_status_t cpv_initialize(cpv_vehicle_t *vehicle, double start_time_s);

/* Advance to target_time_s (>= the current time). *reached (if non-NULL) is
 * always set to the time the FMU actually reached. On an FMU error the
 * instance enters a failed state; only cpv_reset and cpv_close are valid then. */
cpv_status_t cpv_advance_to(cpv_vehicle_t *vehicle, double target_time_s, double *reached);

double cpv_time(const cpv_vehicle_t *vehicle);
int cpv_has_capability(const cpv_vehicle_t *vehicle, const char *capability);
const char *cpv_profile(const cpv_vehicle_t *vehicle);
const char *cpv_model_name(const cpv_vehicle_t *vehicle);

/* pwm_actuation_v1. Channel k is element k+1 of the exported pwm vector.
 * Values are held until changed and applied before the next step. */
size_t cpv_pwm_count(const cpv_vehicle_t *vehicle);
cpv_status_t cpv_pwm_start(const cpv_vehicle_t *vehicle, size_t channel, double *pulse_us);
cpv_status_t cpv_set_pwm(cpv_vehicle_t *vehicle, size_t channel, double pulse_us);
cpv_status_t cpv_set_pwm_vector(cpv_vehicle_t *vehicle, const double *pulse_us, size_t count);
/* The model's pwm_min/pwm_max, when it exports them; CPV_UNSUPPORTED otherwise. */
cpv_status_t cpv_pwm_range(cpv_vehicle_t *vehicle, double *min_us, double *max_us);

/* payload_actuation_v1: pulse widths of the payloads' servos (gripper, winch,
 * parachute, ...). Element k is host servo output 9+k. Held like pwm. */
size_t cpv_aux_count(const cpv_vehicle_t *vehicle);
cpv_status_t cpv_set_aux_pwm(cpv_vehicle_t *vehicle, size_t channel, double pulse_us);

/* external_wrench_v1: force [N] at the combined CG and moment [N.m] about it, body FRD. */
cpv_status_t cpv_set_external_wrench(cpv_vehicle_t *vehicle, const double force_frd[3],
                                     const double moment_frd[3]);

/* kinematic_state_v1: truth at the combined CG at the current time. */
typedef struct {
  double position_ned[3];       /* m, relative to the geodetic origin */
  double velocity_ned[3];       /* m/s */
  double quaternion_wb[4];      /* scalar-first, body FRD -> NED */
  double angular_velocity_frd[3]; /* rad/s */
} cpv_kinematic_state_t;

typedef struct {
  double latitude_deg;
  double longitude_deg;
  double altitude_wgs84_m; /* NED down is zero here */
} cpv_origin_t;

typedef struct {
  double accel_frd[3]; /* specific force, m/s2 */
  double gyro_frd[3];  /* rad/s */
} cpv_imu_t;

typedef struct {
  double field_frd_gauss[3];
} cpv_mag_t;

typedef struct {
  double latitude_deg;
  double longitude_deg;
  double altitude_wgs84_m;
  double velocity_ned[3];
  int has_yaw;
  double yaw_deg; /* clockwise from north; valid when has_yaw */
} cpv_gnss_t;

typedef struct {
  double pressure_pa;
  double temperature_c;
  int has_altitude;
  double altitude_m;
  int has_climb_rate;
  double climb_rate_mps;
} cpv_baro_t;

cpv_status_t cpv_get_state(cpv_vehicle_t *vehicle, cpv_kinematic_state_t *state);
cpv_status_t cpv_get_origin(cpv_vehicle_t *vehicle, cpv_origin_t *origin);
cpv_status_t cpv_get_imu(cpv_vehicle_t *vehicle, cpv_imu_t *imu);
cpv_status_t cpv_get_mag(cpv_vehicle_t *vehicle, cpv_mag_t *mag);
cpv_status_t cpv_get_gnss(cpv_vehicle_t *vehicle, cpv_gnss_t *gnss);
cpv_status_t cpv_get_baro(cpv_vehicle_t *vehicle, cpv_baro_t *baro);

/* Any contract field the model provides, by its exported name; count must equal its size. */
cpv_status_t cpv_get_field(cpv_vehicle_t *vehicle, const char *name, double *values, size_t count);

/* Return to the instantiated state (fmi3Reset). Parameter overrides and input
 * vectors return to the descriptor's; call cpv_initialize again. */
cpv_status_t cpv_reset(cpv_vehicle_t *vehicle);

void cpv_close(cpv_vehicle_t *vehicle);

/* The last error or warning message of this instance ("" if none). */
const char *cpv_last_error(const cpv_vehicle_t *vehicle);
const char *cpv_status_string(cpv_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* CPVEHICLE_H */
