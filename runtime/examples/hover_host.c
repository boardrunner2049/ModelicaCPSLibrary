/* A minimal standalone host: load a vehicle FMU in-process through its
 * descriptor and fly a vertical climb-and-hold on collective PWM.
 *
 * usage: cpv_hover_host VEHICLE.json [TARGET_ALTITUDE_M] [DURATION_S]
 *
 * It uses only contract capabilities (pwm_actuation_v1, kinematic_state_v1,
 * imu_v1): no model name, no value reference, no rotor count. The controller
 * knows nothing about the vehicle except what it measures, so it adapts its
 * hover command online. It is an architectural test of the runtime boundary,
 * not a flight controller.
 */
#include "cpvehicle/cpvehicle.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv) {
  if (argc < 2 || argc > 4) {
    fprintf(stderr, "usage: %s VEHICLE.json [TARGET_ALTITUDE_M] [DURATION_S]\n", argv[0]);
    return 2;
  }
  double target = argc > 2 ? atof(argv[2]) : 1.0;
  double duration = argc > 3 ? atof(argv[3]) : 12.0;
  const char *required[] = {"pwm_actuation_v1", "kinematic_state_v1", "imu_v1"};
  cpv_options_t options;
  cpv_options_init(&options);
  options.required_capabilities = required;
  options.required_count = 3;
  cpv_vehicle_t *v = NULL;
  char error[1024];
  cpv_status_t s = cpv_open(argv[1], &options, &v, error, sizeof(error));
  if (s != CPV_OK) {
    fprintf(stderr, "open: %s: %s\n", cpv_status_string(s), error);
    return 1;
  }
  if ((s = cpv_initialize(v, 0.0)) != CPV_OK) {
    fprintf(stderr, "initialize: %s: %s\n", cpv_status_string(s), cpv_last_error(v));
    cpv_close(v);
    return 1;
  }
  double lo = 1100, hi = 1900;
  if (cpv_pwm_range(v, &lo, &hi) == CPV_UNSUPPORTED)
    printf("model exports no pwm range; assuming %.0f..%.0f us\n", lo, hi);
  size_t channels = cpv_pwm_count(v);
  printf("%s: %zu PWM channel(s), %.0f..%.0f us, profile %s\n", cpv_model_name(v), channels, lo,
         hi, cpv_profile(v));

  const double dt = 0.0025; /* 400 Hz host loop */
  double hover = 0.5, integral = 0.0;
  cpv_kinematic_state_t st;
  int rc = 0;
  for (int k = 1; k * dt <= duration + 1e-12; k++) {
    if ((s = cpv_get_state(v, &st)) >= CPV_UNSUPPORTED)
      break;
    double altitude = -st.position_ned[2], climb = -st.velocity_ned[2];
    /* Critically damped altitude loop on collective, with integral trim. */
    double error_m = target - altitude;
    integral += error_m * dt;
    double command = hover + 0.25 * error_m - 0.25 * climb + 0.08 * integral;
    command = fmin(1.0, fmax(0.0, command));
    for (size_t c = 0; c < channels; c++)
      cpv_set_pwm(v, c, lo + (hi - lo) * command);
    double reached;
    if ((s = cpv_advance_to(v, k * dt, &reached)) >= CPV_UNSUPPORTED)
      break;
    if (k % 400 == 0) {
      cpv_imu_t imu;
      cpv_get_imu(v, &imu);
      printf("t=%5.2f s  altitude=%7.3f m  climb=%7.3f m/s  command=%.3f  accel_z=%7.3f\n",
             reached, altitude, climb, command, imu.accel_frd[2]);
    }
  }
  if (s >= CPV_UNSUPPORTED) {
    fprintf(stderr, "stopped at t=%.6f s: %s: %s\n", cpv_time(v), cpv_status_string(s),
            cpv_last_error(v));
    rc = 1;
  } else {
    cpv_get_state(v, &st);
    double altitude = -st.position_ned[2];
    printf("final: t=%.3f s altitude=%.3f m (target %.3f)\n", cpv_time(v), altitude, target);
    if (fabs(altitude - target) > 0.1) {
      fprintf(stderr, "altitude hold missed the target by %.3f m\n", altitude - target);
      rc = 1;
    }
  }
  cpv_close(v);
  return rc;
}
