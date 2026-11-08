/* A vehicle model's sensor outputs over time, as CSV.
 *
 *   cpv_sensors MODEL SECONDS [PWM_US...]
 *
 * MODEL is a linked model name or a prepared descriptor (vehicle.json).
 * PWM_US values are held on channels 1, 2, ...; the rest keep their start
 * values. Steps at 1200 Hz and prints one row every 10 ms: time, NED truth,
 * accelerometer and gyro, magnetometer (Gauss), GNSS (lat, lon, alt, NED
 * velocity) and barometer (pressure, temperature, altitude).
 */
#include "cpvehicle/cpvehicle.h"
#include "cpvehicle/cpvehicle_model.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv) {
  if (argc < 3) {
    fprintf(stderr, "usage: cpv_sensors MODEL SECONDS [PWM_US...]\n");
    return 2;
  }
  const double seconds = atof(argv[2]);
  cpv_options_t options;
  cpv_options_init(&options);
  cpv_vehicle_t *vehicle = NULL;
  char error[512] = "";
  const cpv_model_t *linked = cpv_find_model(argv[1]);
  const cpv_status_t opened = linked ? cpv_open_model(linked, &options, &vehicle, error, sizeof error)
                                     : cpv_open(argv[1], &options, &vehicle, error, sizeof error);
  if (opened != CPV_OK || cpv_initialize(vehicle, 0) > CPV_WARNING) {
    fprintf(stderr, "cpv_sensors: %s\n", vehicle ? cpv_last_error(vehicle) : error);
    return 1;
  }
  for (int i = 3; i < argc && (size_t)(i - 3) < cpv_pwm_count(vehicle); i++)
    cpv_set_pwm(vehicle, (size_t)(i - 3), atof(argv[i]));
  printf("t,n,e,d,ax,ay,az,gx,gy,gz,mx,my,mz,lat,lon,alt,vn,ve,vd,p_pa,t_c,baro_alt\n");
  const double hz = 1200;
  for (long k = 1; k / hz <= seconds + 1e-9; k++) {
    double reached = 0;
    if (cpv_advance_to(vehicle, k / hz, &reached) > CPV_WARNING) {
      fprintf(stderr, "cpv_sensors: %s\n", cpv_last_error(vehicle));
      return 1;
    }
    if (k % 12)
      continue;
    cpv_kinematic_state_t s;
    cpv_imu_t imu;
    cpv_mag_t mag;
    cpv_gnss_t gnss;
    cpv_baro_t baro;
    memset(&mag, 0, sizeof mag);
    memset(&gnss, 0, sizeof gnss);
    memset(&baro, 0, sizeof baro);
    cpv_get_state(vehicle, &s);
    cpv_get_imu(vehicle, &imu);
    cpv_get_mag(vehicle, &mag);
    cpv_get_gnss(vehicle, &gnss);
    cpv_get_baro(vehicle, &baro);
    printf("%.3f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.9f,%.9f,%.4f,"
           "%.4f,%.4f,%.4f,%.3f,%.3f,%.4f\n",
           reached, s.position_ned[0], s.position_ned[1], s.position_ned[2], imu.accel_frd[0],
           imu.accel_frd[1], imu.accel_frd[2], imu.gyro_frd[0], imu.gyro_frd[1], imu.gyro_frd[2],
           mag.field_frd_gauss[0], mag.field_frd_gauss[1], mag.field_frd_gauss[2],
           gnss.latitude_deg, gnss.longitude_deg, gnss.altitude_wgs84_m, gnss.velocity_ned[0],
           gnss.velocity_ned[1], gnss.velocity_ned[2], baro.pressure_pa, baro.temperature_c,
           baro.altitude_m);
  }
  cpv_close(vehicle);
  return 0;
}
