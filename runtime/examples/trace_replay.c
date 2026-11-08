/* Drive a vehicle model with the PWM commands of a trace; print its own trace.
 *
 *   cpv_replay MODEL INPUT.csv [--rate HZ|input] [--every N] [--set NAME=VALUE]...
 *
 * MODEL is a linked model name or a prepared descriptor (vehicle.json).
 * INPUT.csv is a trace (tools/difftest): a header row naming its columns, then
 * one row per sample. Its `t` column (s) and `pwm1`, `pwm2`, ... columns (us)
 * are used; the others are ignored. Each row's commands are held from its
 * time until the next row's (zero-order hold, as an autopilot's outputs are),
 * starting at the first row's time, which becomes the model's time zero.
 * As in the ArduPilot adapter, model channel i takes servo output i + 1
 * (`pwm<i+1>`) unless CPV_CHANNEL_MAP (comma-separated zero-based servo per
 * channel, -1 for none) says otherwise, and a 0 us command means the channel
 * is not driven: it keeps its value. Channels the input does not name keep
 * their start values. Output `pwm` columns are named by servo, as the input's.
 *
 * Steps the model at HZ (default 1200), or with `--rate input` from each input
 * row's time to the next one's (a frame-by-frame trace, as SITL's: the same
 * steps as the run that recorded it), and prints a trace row every N steps
 * (default 12) on stdout, time shifted back to the input's: truth
 * (NED position and velocity, body-to-NED quaternion and Euler angles, body
 * rates), IMU, magnetometer (Gauss), barometer and the commands applied.
 * --set sets a model parameter before initialization (lat0, lon0,
 * earth_field_north_T, ...), as a host would.
 */
#include "cpvehicle/cpvehicle.h"
#include "cpvehicle/cpvehicle_model.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_COLUMNS 256
#define MAX_PWM 32

typedef struct {
  double t;
  double pwm[MAX_PWM];
} row_t;

static int split(char *line, char **fields) {
  int n = 0;
  for (char *p = strtok(line, ",\r\n"); p && n < MAX_COLUMNS; p = strtok(NULL, ",\r\n"))
    fields[n++] = p;
  return n;
}

/* Reads INPUT: rows of time and commands. Returns the row count, or -1. */
static long read_input(const char *path, row_t **rows_out, int *pwm_columns) {
  FILE *f = fopen(path, "r");
  if (!f) {
    perror(path);
    return -1;
  }
  char line[16384];
  char *fields[MAX_COLUMNS];
  int t_col = -1, pwm_col[MAX_PWM];
  for (int i = 0; i < MAX_PWM; i++)
    pwm_col[i] = -1;
  if (!fgets(line, sizeof line, f)) {
    fprintf(stderr, "cpv_replay: %s is empty\n", path);
    fclose(f);
    return -1;
  }
  const int columns = split(line, fields);
  for (int c = 0; c < columns; c++) {
    if (!strcmp(fields[c], "t"))
      t_col = c;
    else if (!strncmp(fields[c], "pwm", 3)) {
      const int k = atoi(fields[c] + 3);
      if (k >= 1 && k <= MAX_PWM)
        pwm_col[k - 1] = c;
    }
  }
  if (t_col < 0) {
    fprintf(stderr, "cpv_replay: %s has no t column\n", path);
    fclose(f);
    return -1;
  }
  *pwm_columns = 0;
  for (int i = 0; i < MAX_PWM; i++)
    if (pwm_col[i] >= 0)
      *pwm_columns = i + 1;
  long n = 0, capacity = 1024;
  row_t *rows = malloc(sizeof *rows * (size_t)capacity);
  while (fgets(line, sizeof line, f)) {
    /* Empty fields are dropped by strtok: re-split keeping them. */
    char *values[MAX_COLUMNS];
    int count = 0;
    for (char *p = line; count < MAX_COLUMNS;) {
      values[count++] = p;
      char *comma = strchr(p, ',');
      if (!comma)
        break;
      *comma = '\0';
      p = comma + 1;
    }
    if (count <= t_col || !*values[t_col] || values[t_col][0] == '\n')
      continue;
    if (n == capacity)
      rows = realloc(rows, sizeof *rows * (size_t)(capacity *= 2));
    rows[n].t = atof(values[t_col]);
    for (int i = 0; i < MAX_PWM; i++)
      rows[n].pwm[i] = (pwm_col[i] >= 0 && pwm_col[i] < count && *values[pwm_col[i]] &&
                        values[pwm_col[i]][0] != '\n')
                           ? atof(values[pwm_col[i]])
                           : NAN;
    n++;
  }
  fclose(f);
  *rows_out = rows;
  return n;
}

static void print_row(cpv_vehicle_t *vehicle, double time_s, const double *held,
                      const int *source, size_t channels) {
  cpv_kinematic_state_t s;
  cpv_imu_t imu;
  cpv_mag_t mag;
  cpv_baro_t baro;
  memset(&imu, 0, sizeof imu);
  memset(&mag, 0, sizeof mag);
  memset(&baro, 0, sizeof baro);
  cpv_get_state(vehicle, &s);
  const int has_imu = cpv_get_imu(vehicle, &imu) == CPV_OK;
  const int has_mag = cpv_get_mag(vehicle, &mag) == CPV_OK;
  const int has_baro = cpv_get_baro(vehicle, &baro) == CPV_OK;
  const double w = s.quaternion_wb[0], x = s.quaternion_wb[1], y = s.quaternion_wb[2],
               z = s.quaternion_wb[3];
  const double roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const double sp = 2 * (w * y - z * x);
  const double pitch = asin(sp > 1 ? 1 : sp < -1 ? -1 : sp);
  const double yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  printf("%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.8f,%.8f,%.8f,%.8f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f",
         time_s, s.position_ned[0], s.position_ned[1], s.position_ned[2], s.velocity_ned[0],
         s.velocity_ned[1], s.velocity_ned[2], w, x, y, z, roll, pitch, yaw,
         s.angular_velocity_frd[0], s.angular_velocity_frd[1], s.angular_velocity_frd[2]);
  if (has_imu)
    printf(",%.6f,%.6f,%.6f,%.7f,%.7f,%.7f", imu.accel_frd[0], imu.accel_frd[1],
           imu.accel_frd[2], imu.gyro_frd[0], imu.gyro_frd[1], imu.gyro_frd[2]);
  else
    printf(",,,,,,");
  if (has_mag)
    printf(",%.6f,%.6f,%.6f", mag.field_frd_gauss[0], mag.field_frd_gauss[1],
           mag.field_frd_gauss[2]);
  else
    printf(",,,");
  if (has_baro)
    printf(",%.3f", baro.pressure_pa);
  else
    printf(",");
  if (has_baro && baro.has_altitude)
    printf(",%.4f", baro.altitude_m);
  else
    printf(",");
  for (size_t i = 0; i < channels; i++)
    if (source[i] >= 0)
      printf(",%.1f", held[i]);
  printf("\n");
}

int main(int argc, char **argv) {
  if (argc < 3) {
    fprintf(stderr, "usage: cpv_replay MODEL INPUT.csv [--rate HZ|input] [--every N] [--set NAME=VALUE]...\n");
    return 2;
  }
  double hz = 1200; /* 0: the input's own times */
  long every = 12;
  cpv_options_t options;
  cpv_options_init(&options);
  cpv_vehicle_t *vehicle = NULL;
  char error[512] = "";
  const cpv_model_t *linked = cpv_find_model(argv[1]);
  const cpv_status_t opened = linked ? cpv_open_model(linked, &options, &vehicle, error, sizeof error)
                                     : cpv_open(argv[1], &options, &vehicle, error, sizeof error);
  if (opened != CPV_OK) {
    fprintf(stderr, "cpv_replay: %s\n", error);
    return 1;
  }
  for (int i = 3; i < argc; i++) {
    if (!strcmp(argv[i], "--rate") && i + 1 < argc) {
      i++;
      hz = strcmp(argv[i], "input") ? atof(argv[i]) : 0;
      if (hz <= 0 && strcmp(argv[i], "input")) {
        fprintf(stderr, "cpv_replay: --rate must be positive or input\n");
        return 2;
      }
    }
    else if (!strcmp(argv[i], "--every") && i + 1 < argc)
      every = atol(argv[++i]);
    else if (!strcmp(argv[i], "--set") && i + 1 < argc) {
      char *eq = strchr(argv[++i], '=');
      if (!eq) {
        fprintf(stderr, "cpv_replay: --set NAME=VALUE\n");
        return 2;
      }
      *eq = '\0';
      if (cpv_set_parameter(vehicle, argv[i], atof(eq + 1)) != CPV_OK) {
        fprintf(stderr, "cpv_replay: %s: %s\n", argv[i], cpv_last_error(vehicle));
        return 1;
      }
    } else {
      fprintf(stderr, "cpv_replay: unknown option %s\n", argv[i]);
      return 2;
    }
  }
  if (every < 1) {
    fprintf(stderr, "cpv_replay: --every must be positive\n");
    return 2;
  }
  row_t *rows = NULL;
  int pwm_columns = 0;
  const long n = read_input(argv[2], &rows, &pwm_columns);
  if (n < 0)
    return 1;
  if (n == 0) {
    fprintf(stderr, "cpv_replay: %s has no rows\n", argv[2]);
    return 1;
  }
  if (cpv_initialize(vehicle, 0) > CPV_WARNING) {
    fprintf(stderr, "cpv_replay: %s\n", cpv_last_error(vehicle));
    return 1;
  }
  size_t channels = cpv_pwm_count(vehicle);
  if (channels > MAX_PWM)
    channels = MAX_PWM;
  int source[MAX_PWM];
  for (size_t i = 0; i < channels; i++)
    source[i] = (int)i;
  const char *map = getenv("CPV_CHANNEL_MAP");
  if (map && *map) {
    size_t i = 0;
    for (const char *p = map; *p && i < channels; i++) {
      char *end = NULL;
      source[i] = (int)strtol(p, &end, 10);
      if (end == p || source[i] < -1 || source[i] >= MAX_PWM) {
        fprintf(stderr, "cpv_replay: invalid CPV_CHANNEL_MAP '%s'\n", map);
        return 2;
      }
      p = *end == ',' ? end + 1 : end;
    }
    if (i != channels) {
      fprintf(stderr, "cpv_replay: CPV_CHANNEL_MAP has %zu entries; the model has %zu channels\n",
              i, channels);
      return 2;
    }
  }
  double held[MAX_PWM];
  for (size_t i = 0; i < channels; i++) {
    cpv_pwm_start(vehicle, i, &held[i]);
    if (source[i] >= pwm_columns)
      source[i] = -1; /* not in the input: keeps its start value */
  }

  printf("t,pos_n,pos_e,pos_d,vel_n,vel_e,vel_d,qw,qx,qy,qz,roll,pitch,yaw,rate_x,rate_y,rate_z,"
         "accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,baro_press,baro_alt");
  for (size_t i = 0; i < channels; i++)
    if (source[i] >= 0)
      printf(",pwm%d", source[i] + 1);
  printf("\n");

  const double t0 = rows[0].t, end = rows[n - 1].t - t0;
  const long steps = hz > 0 ? (long)floor(end * hz + 1e-6) + 1 : n;
  long next = 0;
  for (long k = 0; k < steps; k++) {
    const double now = hz > 0 ? k / hz : rows[k].t - t0;
    /* Every row due by now applies; its values hold until the next one. */
    while (next < n && rows[next].t - t0 <= now + 1e-9) {
      for (size_t i = 0; i < channels; i++) {
        const double us = source[i] >= 0 ? rows[next].pwm[source[i]] : NAN;
        if (!isnan(us) && us != 0)
          held[i] = us;
      }
      next++;
    }
    if (channels && cpv_set_pwm_vector(vehicle, held, channels) > CPV_WARNING) {
      fprintf(stderr, "cpv_replay: %s\n", cpv_last_error(vehicle));
      return 1;
    }
    /* The row: the state at `now` and the commands held from `now` on. */
    if (k % every == 0)
      print_row(vehicle, now + t0, held, source, channels);
    if (k + 1 < steps) {
      const double then = hz > 0 ? (k + 1) / hz : rows[k + 1].t - t0;
      double reached = 0;
      if (cpv_advance_to(vehicle, then, &reached) > CPV_WARNING) {
        fprintf(stderr, "cpv_replay: t=%.3f: %s\n", then, cpv_last_error(vehicle));
        return 1;
      }
    }
  }
  free(rows);
  cpv_close(vehicle);
  return 0;
}
