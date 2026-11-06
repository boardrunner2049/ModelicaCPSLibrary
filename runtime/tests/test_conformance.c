/* Conformance and runtime tests against prepared vehicle descriptors.
 *
 * usage: test_conformance VEHICLES_DIR
 *   VEHICLES_DIR/quad/vehicle.json    configs/quad.toml (1.5 kg, Izz 0.04, yawed 90 deg)
 *   VEHICLES_DIR/hex/vehicle.json     configs/hex.toml (2.2 kg, Izz 0.075, 8 channels, 6 used)
 *   VEHICLES_DIR/legacy/vehicle.json  an FMU without the marker, prepared with fastdyn_legacy
 *
 * Every physical check compares against a value known in advance (gravity,
 * the configured initial attitude, F = m a, M = I alpha); none compares the
 * model with itself only.
 */
#include "cpvehicle/cpvehicle.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define G 9.80665
#define QUAD_MASS 1.5
#define HEX_MASS 2.2
#define HEX_IZZ 0.075

static int failures, checks;
static char quad[1024], hex[1024], legacy[1024], vehicles[1024];

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
#define OK(call)                                                                                   \
  do {                                                                                             \
    cpv_status_t s_ = (call);                                                                      \
    CHECK(s_ == CPV_OK, "%s -> %s (%s)", #call, cpv_status_string(s_), cpv_last_error(v));         \
  } while (0)

static cpv_vehicle_t *open_vehicle(const char *path, const char *const *required, size_t n) {
  cpv_options_t o;
  cpv_options_init(&o);
  o.required_capabilities = required;
  o.required_count = n;
  cpv_vehicle_t *v = NULL;
  char error[1024];
  cpv_status_t s = cpv_open(path, &o, &v, error, sizeof(error));
  CHECK(s == CPV_OK && v, "cpv_open(%s) -> %s: %s", path, cpv_status_string(s), error);
  return v;
}

/* Rotate a body FRD vector into NED with the scalar-first quaternion q. */
static void rotate(const double q[4], const double b[3], double n[3]) {
  double w = q[0], x = q[1], y = q[2], z = q[3];
  double r[3][3] = {{1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)},
                    {2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)},
                    {2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)}};
  for (int i = 0; i < 3; i++)
    n[i] = r[i][0] * b[0] + r[i][1] * b[1] + r[i][2] * b[2];
}

static const char *const ALL[] = {"pwm_actuation_v1", "kinematic_state_v1", "imu_v1",
                                  "magnetometer_v1", "gnss_v1", "barometer_v1",
                                  "external_wrench_v1"};

/* At rest on its gear: gravity, the configured attitude, consistent sensors. */
static void test_rest_and_rotation(void) {
  cpv_vehicle_t *v = open_vehicle(quad, ALL, 7);
  if (!v)
    return;
  OK(cpv_initialize(v, 0.0));
  double reached = -1;
  OK(cpv_advance_to(v, 3.0, &reached));
  CHECK(NEAR(reached, 3.0, 1e-9), "reached %.9f", reached);
  cpv_kinematic_state_t st;
  cpv_imu_t imu;
  cpv_gnss_t gnss;
  cpv_origin_t origin;
  OK(cpv_get_state(v, &st));
  OK(cpv_get_imu(v, &imu));
  OK(cpv_get_gnss(v, &gnss));
  OK(cpv_get_origin(v, &origin));
  double norm = sqrt(st.quaternion_wb[0] * st.quaternion_wb[0] + st.quaternion_wb[1] * st.quaternion_wb[1] +
                     st.quaternion_wb[2] * st.quaternion_wb[2] + st.quaternion_wb[3] * st.quaternion_wb[3]);
  CHECK(NEAR(norm, 1.0, 1e-6), "quaternion norm %.9f", norm);
  /* quad.toml: q_start = 90 deg about body down, i.e. nose east. */
  double fwd[3] = {1, 0, 0}, right[3] = {0, 1, 0}, n[3];
  rotate(st.quaternion_wb, fwd, n);
  CHECK(NEAR(n[0], 0, 1e-3) && NEAR(n[1], 1, 1e-3) && NEAR(n[2], 0, 1e-3),
        "body forward in NED = (%.4f %.4f %.4f), expected east", n[0], n[1], n[2]);
  rotate(st.quaternion_wb, right, n);
  CHECK(NEAR(n[0], -1, 1e-3) && NEAR(n[1], 0, 1e-3), "body right in NED = (%.4f %.4f %.4f), "
        "expected south", n[0], n[1], n[2]);
  CHECK(gnss.has_yaw && NEAR(gnss.yaw_deg, 90, 0.1), "yaw_deg %.4f, expected 90", gnss.yaw_deg);
  /* Settled on the gear: at rest, specific force is -g along body down. */
  for (int i = 0; i < 3; i++) {
    CHECK(NEAR(st.velocity_ned[i], 0, 1e-3), "velocity[%d] %.6f", i, st.velocity_ned[i]);
    CHECK(NEAR(st.angular_velocity_frd[i], 0, 1e-3), "omega[%d] %.6f", i, st.angular_velocity_frd[i]);
    CHECK(NEAR(imu.gyro_frd[i], 0, 1e-3), "gyro[%d] %.6f", i, imu.gyro_frd[i]);
  }
  CHECK(NEAR(imu.accel_frd[0], 0, 0.01) && NEAR(imu.accel_frd[1], 0, 0.01) &&
            NEAR(imu.accel_frd[2], -G, 0.01),
        "accel (%.4f %.4f %.4f), expected (0 0 %.4f)", imu.accel_frd[0], imu.accel_frd[1],
        imu.accel_frd[2], -G);
  /* GNSS agrees with the truth through the declared origin: a sensor read through
   * the contract, compared with the state read through the contract. */
  CHECK(NEAR(gnss.altitude_wgs84_m, origin.altitude_wgs84_m - st.position_ned[2], 1e-6),
        "gnss altitude %.6f vs origin %.6f - down %.6f", gnss.altitude_wgs84_m,
        origin.altitude_wgs84_m, st.position_ned[2]);
  CHECK(NEAR(gnss.latitude_deg, origin.latitude_deg, 1e-7), "gnss lat %.9f", gnss.latitude_deg);
  cpv_close(v);
}

/* A known external force and moment (external_wrench_v1) give F = m a and M = I alpha. */
static void test_external_wrench(void) {
  cpv_vehicle_t *v = open_vehicle(hex, ALL, 7);
  if (!v)
    return;
  OK(cpv_initialize(v, 0.0));
  OK(cpv_advance_to(v, 2.0, NULL));
  /* Body FRD z is down while level: -2 m g lifts the vehicle at g; a yaw moment spins it. */
  double force[3] = {0, 0, -2 * HEX_MASS * G}, moment[3] = {0, 0, 0.3};
  OK(cpv_set_external_wrench(v, force, moment));
  OK(cpv_advance_to(v, 2.5, NULL)); /* off the gear */
  cpv_kinematic_state_t a, b;
  OK(cpv_get_state(v, &a));
  OK(cpv_advance_to(v, 3.0, NULL));
  OK(cpv_get_state(v, &b));
  double az = (b.velocity_ned[2] - a.velocity_ned[2]) / 0.5;
  CHECK(NEAR(az, -G, 0.02), "vertical acceleration %.5f, expected %.5f (up)", az, -G);
  double alpha = (b.angular_velocity_frd[2] - a.angular_velocity_frd[2]) / 0.5;
  CHECK(NEAR(alpha, 0.3 / HEX_IZZ, 0.01 * 0.3 / HEX_IZZ), "yaw angular acceleration %.5f, "
        "expected %.5f", alpha, 0.3 / HEX_IZZ);
  cpv_imu_t imu;
  OK(cpv_get_imu(v, &imu));
  /* Specific force is the applied non-gravity force over mass, in body FRD. */
  CHECK(NEAR(imu.accel_frd[2], -2 * G, 0.05), "accel z %.4f, expected %.4f", imu.accel_frd[2],
        -2 * G);
  cpv_close(v);
}

/* Every channel reaches its own element; unused channels are valid; start values hold. */
static void test_actuator_channels(void) {
  cpv_vehicle_t *v = open_vehicle(hex, ALL, 7);
  if (!v)
    return;
  size_t n = cpv_pwm_count(v);
  CHECK(n == 8, "hex exports %zu PWM channels, expected 8", n);
  double start[8], back[8];
  for (size_t i = 0; i < n && i < 8; i++)
    OK(cpv_pwm_start(v, i, &start[i]));
  CHECK(cpv_set_pwm(v, n, 1500) == CPV_ERR_ARGUMENT, "channel %zu accepted", n);
  CHECK(cpv_set_pwm(v, 0, NAN) == CPV_ERR_ARGUMENT, "NaN accepted");
  OK(cpv_initialize(v, 0.0));
  OK(cpv_get_field(v, "pwm", back, 8));
  for (size_t i = 0; i < 8; i++)
    CHECK(back[i] == start[i], "channel %zu before any write: %.3f, start %.3f", i, back[i], start[i]);
  OK(cpv_set_pwm(v, 3, 1234.5));
  OK(cpv_set_pwm(v, 7, 1777.25));
  OK(cpv_advance_to(v, 0.01, NULL));
  OK(cpv_get_field(v, "pwm", back, 8));
  for (size_t i = 0; i < 8; i++) {
    double want = i == 3 ? 1234.5 : i == 7 ? 1777.25 : start[i];
    CHECK(back[i] == want, "channel %zu: %.3f, expected %.3f", i, back[i], want);
  }
  double lo, hi;
  OK(cpv_pwm_range(v, &lo, &hi));
  CHECK(lo == 1100 && hi == 1900, "pwm range %.1f..%.1f", lo, hi);
  cpv_close(v);

  /* Physical wiring of hex.toml: actuatorIndex = {1,2,3,4,6,8}; channels 4 and 6
   * (0-based) are unused. Hover on an external force and drive one channel. */
  int used[8] = {1, 1, 1, 1, 0, 1, 0, 1};
  for (size_t ch = 0; ch < 8; ch++) {
    v = open_vehicle(hex, ALL, 7);
    if (!v)
      return;
    double lift[3] = {0, 0, -HEX_MASS * G}, none[3] = {0, 0, 0};
    OK(cpv_set_external_wrench(v, lift, none));
    OK(cpv_initialize(v, 0.0));
    for (size_t i = 0; i < 8; i++)
      OK(cpv_set_pwm(v, i, 1100));
    OK(cpv_set_pwm(v, ch, 1700));
    OK(cpv_advance_to(v, 0.5, NULL));
    cpv_kinematic_state_t st;
    OK(cpv_get_state(v, &st));
    double w = fabs(st.angular_velocity_frd[0]) + fabs(st.angular_velocity_frd[1]) +
               fabs(st.angular_velocity_frd[2]);
    if (used[ch])
      CHECK(w > 0.05, "channel %zu drives a rotor but |omega| = %.6f", ch, w);
    else
      CHECK(w < 1e-9, "channel %zu is unused but |omega| = %.9f", ch, w);
    cpv_close(v);
  }
}

static void run_scenario(cpv_vehicle_t *v, double force_down, cpv_kinematic_state_t *out) {
  double force[3] = {0, 0, force_down}, none[3] = {0, 0, 0};
  OK(cpv_set_external_wrench(v, force, none));
  OK(cpv_advance_to(v, 1.0, NULL));
  OK(cpv_get_state(v, out));
}

/* Two instances in one process are independent; reset reproduces the first run. */
static void test_instances_and_reset(void) {
  cpv_vehicle_t *v = open_vehicle(quad, ALL, 7);
  cpv_vehicle_t *w = open_vehicle(quad, ALL, 7);
  if (!v || !w)
    return;
  OK(cpv_initialize(v, 0.0));
  OK(cpv_initialize(w, 0.0));
  cpv_kinematic_state_t a, b, again;
  /* Interleave the two instances step by step. */
  double up[3] = {0, 0, -1.5 * QUAD_MASS * G}, none[3] = {0, 0, 0};
  OK(cpv_set_external_wrench(v, up, none));
  for (int k = 1; k <= 100; k++) {
    OK(cpv_advance_to(v, k * 0.01, NULL));
    OK(cpv_advance_to(w, k * 0.01, NULL));
  }
  OK(cpv_get_state(v, &a));
  OK(cpv_get_state(w, &b));
  CHECK(a.position_ned[2] < -0.5, "lifted instance down position %.4f", a.position_ned[2]);
  CHECK(fabs(b.velocity_ned[2]) < 1e-3, "resting instance moved: vz %.6f", b.velocity_ned[2]);
  cpv_close(w);

  OK(cpv_reset(v));
  CHECK(cpv_time(v) == 0, "time after reset %.9f", cpv_time(v));
  CHECK(cpv_advance_to(v, 1.0, NULL) == CPV_ERR_STATE, "stepping before re-initialize accepted");
  OK(cpv_initialize(v, 0.0));
  run_scenario(v, -1.5 * QUAD_MASS * G, &again);
  for (int i = 0; i < 3; i++)
    CHECK(fabs(again.position_ned[i] - a.position_ned[i]) < 1e-9,
          "after reset position[%d] %.12f, first run %.12f", i, again.position_ned[i],
          a.position_ned[i]);
  CHECK(cpv_advance_to(v, 0.5, NULL) == CPV_ERR_ARGUMENT, "backward time accepted");
  cpv_close(v);
}

/* Overrides apply through the descriptor's parameter table; others are refused. */
static void test_parameters(void) {
  cpv_vehicle_t *v = open_vehicle(quad, ALL, 7);
  if (!v)
    return;
  OK(cpv_set_parameter(v, "lat0", 47.0));
  CHECK(cpv_set_parameter(v, "cp_vehicle_interface_version", 2) == CPV_ERR_ARGUMENT,
        "marker override accepted");
  CHECK(cpv_set_parameter(v, "nPwm", 6) == CPV_ERR_ARGUMENT, "structural override accepted");
  CHECK(cpv_set_parameter(v, "no_such_parameter", 1) == CPV_ERR_ARGUMENT, "unknown accepted");
  OK(cpv_initialize(v, 0.0));
  CHECK(cpv_set_parameter(v, "lat0", 48.0) == CPV_ERR_STATE, "override after initialize accepted");
  OK(cpv_advance_to(v, 0.1, NULL));
  cpv_origin_t origin;
  cpv_gnss_t gnss;
  OK(cpv_get_origin(v, &origin));
  OK(cpv_get_gnss(v, &gnss));
  CHECK(origin.latitude_deg == 47.0, "origin latitude %.6f", origin.latitude_deg);
  CHECK(NEAR(gnss.latitude_deg, 47.0, 1e-6), "gnss latitude %.6f", gnss.latitude_deg);
  /* Reset returns to the descriptor's parameters. */
  OK(cpv_reset(v));
  OK(cpv_initialize(v, 0.0));
  OK(cpv_get_origin(v, &origin));
  CHECK(NEAR(origin.latitude_deg, 40.414929, 1e-9), "latitude after reset %.6f",
        origin.latitude_deg);
  cpv_close(v);
}

static int write_text(const char *path, const char *text) {
  FILE *f = fopen(path, "w");
  if (!f)
    return -1;
  fputs(text, f);
  return fclose(f);
}

static char *slurp(const char *path) {
  FILE *f = fopen(path, "rb");
  if (!f)
    return NULL;
  fseek(f, 0, SEEK_END);
  long n = ftell(f);
  rewind(f);
  char *s = malloc((size_t)n + 1);
  if (s && fread(s, 1, (size_t)n, f) != (size_t)n) {
    free(s);
    s = NULL;
  }
  if (s)
    s[n] = '\0';
  fclose(f);
  return s;
}

/* Descriptors that must be refused, each with a reason in the error text. */
static void test_refusals(void) {
  cpv_options_t o;
  cpv_vehicle_t *v = NULL;
  char error[1024], path[1200];
  cpv_options_init(&o);

  const char *wanted = "rangefinder_v1";
  o.required_capabilities = &wanted;
  o.required_count = 1;
  CHECK(cpv_open(hex, &o, &v, error, sizeof(error)) == CPV_ERR_CAPABILITY && !v,
        "missing capability accepted: %s", error);
  CHECK(strstr(error, "rangefinder_v1") != NULL, "error does not name it: %s", error);

  FILE *probe = fopen(legacy, "r");
  if (!probe) {
    printf("SKIPPED legacy-profile checks: %s not prepared (set CPV_LEGACY_FMU)\n", legacy);
    goto edits;
  }
  fclose(probe);
  cpv_options_init(&o);
  CHECK(cpv_open(legacy, &o, &v, error, sizeof(error)) == CPV_ERR_DESCRIPTOR && !v,
        "legacy profile accepted without opt-in: %s", error);
  o.allow_legacy_profile = 1;
  const char *needs_state = "kinematic_state_v1";
  o.required_capabilities = &needs_state;
  o.required_count = 1;
  CHECK(cpv_open(legacy, &o, &v, error, sizeof(error)) == CPV_ERR_CAPABILITY && !v,
        "legacy FMU claimed kinematic state: %s", error);
  o.required_count = 0;
  cpv_status_t s = cpv_open(legacy, &o, &v, error, sizeof(error));
  CHECK(s == CPV_OK && v, "legacy with opt-in refused: %s", error);
  if (v) {
    CHECK(cpv_initialize(v, 0) == CPV_OK, "legacy initialize: %s", cpv_last_error(v));
    cpv_kinematic_state_t st;
    CHECK(cpv_get_state(v, &st) == CPV_UNSUPPORTED, "legacy state read did not report unsupported");
    cpv_imu_t imu;
    CHECK(cpv_get_imu(v, &imu) == CPV_OK, "legacy imu: %s", cpv_last_error(v));
    cpv_close(v);
    v = NULL;
  }

edits:;
  /* A stale digest, an unknown schema and an unknown contract version. */
  char *text = slurp(quad);
  CHECK(text != NULL, "cannot read %s", quad);
  if (!text)
    return;
  struct {
    const char *from, *to, *expect;
  } edits[] = {
      {"\"library_sha256\": \"", "\"library_sha256\": \"0", "stale"},
      {"\"descriptor_schema\": 1", "\"descriptor_schema\": 2", "descriptor_schema"},
      {"\"version\": 1", "\"version\": 2", "version"},
      {"\"profile\": \"standard\"", "\"profile\": \"bogus\"", "profile"},
  };
  snprintf(path, sizeof(path), "%s/quad/edited.json", vehicles);
  for (size_t i = 0; i < sizeof(edits) / sizeof(edits[0]); i++) {
    char *at = strstr(text, edits[i].from);
    CHECK(at != NULL, "descriptor lacks %s", edits[i].from);
    if (!at)
      continue;
    size_t head = (size_t)(at - text), from = strlen(edits[i].from), to = strlen(edits[i].to);
    char *edited = malloc(strlen(text) + to + 1);
    memcpy(edited, text, head);
    memcpy(edited + head, edits[i].to, to);
    strcpy(edited + head + to, at + from);
    write_text(path, edited);
    free(edited);
    cpv_options_init(&o);
    s = cpv_open(path, &o, &v, error, sizeof(error));
    CHECK(s == CPV_ERR_DESCRIPTOR && !v, "edit %zu accepted (%s)", i, cpv_status_string(s));
    CHECK(strstr(error, edits[i].expect) != NULL, "edit %zu: error lacks '%s': %s", i,
          edits[i].expect, error);
    if (v)
      cpv_close(v);
    v = NULL;
  }
  /* A wrong instantiation token fails cleanly at instantiation (partial-open cleanup). */
  char *at = strstr(text, "\"instantiation_token\": \"");
  if (at) {
    at += strlen("\"instantiation_token\": \"");
    char saved = *at;
    *at = saved == 'x' ? 'y' : 'x';
    write_text(path, text);
    *at = saved;
    s = cpv_open(path, &o, &v, error, sizeof(error));
    CHECK(s == CPV_ERR_LOAD && !v, "wrong token accepted (%s): %s", cpv_status_string(s), error);
    if (v)
      cpv_close(v);
  }
  remove(path);
  free(text);
}

int main(int argc, char **argv) {
  if (argc != 2) {
    fprintf(stderr, "usage: %s VEHICLES_DIR\n", argv[0]);
    return 2;
  }
  snprintf(vehicles, sizeof(vehicles), "%s", argv[1]);
  snprintf(quad, sizeof(quad), "%s/quad/vehicle.json", argv[1]);
  snprintf(hex, sizeof(hex), "%s/hex/vehicle.json", argv[1]);
  snprintf(legacy, sizeof(legacy), "%s/legacy/vehicle.json", argv[1]);
  test_rest_and_rotation();
  test_external_wrench();
  test_actuator_channels();
  test_instances_and_reset();
  test_parameters();
  test_refusals();
  printf("conformance: %d checks, %d failure(s)\n", checks, failures);
  return failures != 0;
}
