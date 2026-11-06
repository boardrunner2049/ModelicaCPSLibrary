/* Tests of vehicle models linked into the runtime (prepare --shim, cmake -DCPV_MODELS).
 *
 * usage: test_linked_models VEHICLES_DIR
 *   Every linked model NAME must also have VEHICLES_DIR/NAME/vehicle.json, the
 *   descriptor prepared from the same FMU. The linked model (compiled from the
 *   FMU's sources) and the descriptor (the FMU's own shared library) are driven
 *   with the same inputs, parameter override and wrench, and must agree bit for
 *   bit: same capabilities, fields, start values and every output at every frame.
 *   Then: lookup, refusals, and instances of several linked models at once.
 */
#include "cpvehicle/cpvehicle.h"
#include "cpvehicle/cpvehicle_model.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures, checks;
static const char *vehicles;

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

/* Every Float64 output the contract defines, read through the public API. */
typedef struct {
  cpv_kinematic_state_t state;
  cpv_imu_t imu;
  cpv_mag_t mag;
  cpv_gnss_t gnss;
  cpv_baro_t baro;
  cpv_origin_t origin;
} snapshot_t;

static cpv_status_t snapshot(cpv_vehicle_t *v, snapshot_t *s) {
  memset(s, 0, sizeof(*s));
  cpv_status_t worst = cpv_get_state(v, &s->state), r;
#define TAKE(call, cap)                                                                            \
  if (cpv_has_capability(v, cap) && (r = (call)) > worst)                                          \
  worst = r
  TAKE(cpv_get_imu(v, &s->imu), "imu_v1");
  TAKE(cpv_get_mag(v, &s->mag), "magnetometer_v1");
  TAKE(cpv_get_gnss(v, &s->gnss), "gnss_v1");
  TAKE(cpv_get_baro(v, &s->baro), "barometer_v1");
  TAKE(cpv_get_origin(v, &s->origin), "kinematic_state_v1");
#undef TAKE
  return worst;
}

static cpv_vehicle_t *open_linked(const cpv_model_t *m) {
  cpv_vehicle_t *v = NULL;
  char error[1024];
  cpv_status_t s = cpv_open_model(m, NULL, &v, error, sizeof(error));
  CHECK(s == CPV_OK && v, "cpv_open_model(%s) -> %s: %s", m->name, cpv_status_string(s), error);
  return v;
}

static cpv_vehicle_t *open_descriptor(const char *name) {
  char path[1024];
  snprintf(path, sizeof(path), "%s/%s/vehicle.json", vehicles, name);
  cpv_vehicle_t *v = NULL;
  char error[1024];
  cpv_status_t s = cpv_open(path, NULL, &v, error, sizeof(error));
  CHECK(s == CPV_OK && v, "cpv_open(%s) -> %s: %s", path, cpv_status_string(s), error);
  return v;
}

/* The same description: model, profile, capabilities, PWM layout and start values. */
static void compare_description(const cpv_model_t *m, cpv_vehicle_t *a, cpv_vehicle_t *b) {
  CHECK(strcmp(cpv_model_name(a), cpv_model_name(b)) == 0, "%s: model %s vs %s", m->name,
        cpv_model_name(a), cpv_model_name(b));
  CHECK(strcmp(cpv_profile(a), cpv_profile(b)) == 0, "%s: profile", m->name);
  static const char *const caps[] = {"pwm_actuation_v1", "kinematic_state_v1", "imu_v1",
                                     "magnetometer_v1",  "gnss_v1",            "barometer_v1",
                                     "external_wrench_v1"};
  for (size_t i = 0; i < sizeof(caps) / sizeof(caps[0]); i++)
    CHECK(cpv_has_capability(a, caps[i]) == cpv_has_capability(b, caps[i]), "%s: %s", m->name,
          caps[i]);
  CHECK(cpv_pwm_count(a) == cpv_pwm_count(b), "%s: %zu vs %zu PWM channels", m->name,
        cpv_pwm_count(a), cpv_pwm_count(b));
  for (size_t k = 0; k < cpv_pwm_count(a) && k < cpv_pwm_count(b); k++) {
    double x = NAN, y = NAN;
    cpv_pwm_start(a, k, &x);
    cpv_pwm_start(b, k, &y);
    CHECK(x == y, "%s: pwm[%zu] start %g vs %g", m->name, k + 1, x, y);
  }
}

/* Drive both with identical inputs and compare every output at every frame. */
static void compare_run(const cpv_model_t *m) {
  cpv_vehicle_t *a = open_linked(m), *b = open_descriptor(m->name);
  if (!a || !b) {
    cpv_close(a);
    cpv_close(b);
    return;
  }
  compare_description(m, a, b);
  /* A parameter override goes through both paths the same way. */
  if (cpv_has_capability(a, "kinematic_state_v1")) {
    CHECK(cpv_set_parameter(a, "lat0", 40.5) == CPV_OK, "%s: %s", m->name, cpv_last_error(a));
    CHECK(cpv_set_parameter(b, "lat0", 40.5) == CPV_OK, "%s: %s", m->name, cpv_last_error(b));
  }
  CHECK(cpv_initialize(a, 0.0) == CPV_OK, "%s: %s", m->name, cpv_last_error(a));
  CHECK(cpv_initialize(b, 0.0) == CPV_OK, "%s: %s", m->name, cpv_last_error(b));
  size_t n = cpv_pwm_count(a);
  double pwm[16];
  int frames = 0, mismatches = 0;
  for (int i = 1; i <= 750 && n <= 16; i++) { /* 3 s at 250 Hz */
    double t = i * 0.004;
    for (size_t k = 0; k < n; k++) /* every channel moves, inside 1000..2000 us */
      pwm[k] = 1500 + 450 * sin(1.3 * t + 0.7 * (double)k);
    cpv_set_pwm_vector(a, pwm, n);
    cpv_set_pwm_vector(b, pwm, n);
    if (cpv_has_capability(a, "external_wrench_v1")) {
      double f[3] = {0.5 * sin(t), 0.2, -0.1 * t}, mo[3] = {0.01, -0.02 * sin(2 * t), 0.005};
      cpv_set_external_wrench(a, f, mo);
      cpv_set_external_wrench(b, f, mo);
    }
    cpv_status_t sa = cpv_advance_to(a, t, NULL), sb = cpv_advance_to(b, t, NULL);
    if (sa > CPV_WARNING || sb > CPV_WARNING) {
      CHECK(0, "%s: step at t=%.3f: %s / %s", m->name, t, cpv_last_error(a), cpv_last_error(b));
      break;
    }
    snapshot_t x, y;
    cpv_status_t ra = snapshot(a, &x), rb = snapshot(b, &y);
    if (ra > CPV_WARNING || rb > CPV_WARNING) {
      CHECK(0, "%s: outputs at t=%.3f: %s / %s", m->name, t, cpv_last_error(a), cpv_last_error(b));
      break;
    }
    frames++;
    if (memcmp(&x, &y, sizeof(x)) != 0 && mismatches++ == 0)
      fprintf(stderr, "%s: first difference at t=%.3f s: z %.17g vs %.17g\n", m->name, t,
              x.state.position_ned[2], y.state.position_ned[2]);
  }
  CHECK(frames == 750, "%s: ran %d of 750 frames", m->name, frames);
  CHECK(mismatches == 0, "%s: linked model differs from the descriptor in %d of %d frames",
        m->name, mismatches, frames);
  /* Moving, so the comparison was not of two vehicles at rest. */
  cpv_kinematic_state_t s;
  cpv_get_state(a, &s);
  double moved = fabs(s.velocity_ned[0]) + fabs(s.velocity_ned[1]) + fabs(s.velocity_ned[2]) +
                 fabs(s.angular_velocity_frd[0]) + fabs(s.angular_velocity_frd[1]) +
                 fabs(s.angular_velocity_frd[2]);
  CHECK(moved > 1e-3, "%s: vehicle did not move (|v|+|w| = %g)", m->name, moved);
  /* Reset returns the linked model to its start like the descriptor path. */
  CHECK(cpv_reset(a) == CPV_OK && cpv_initialize(a, 0.0) == CPV_OK, "%s: reset: %s", m->name,
        cpv_last_error(a));
  CHECK(cpv_reset(b) == CPV_OK && cpv_initialize(b, 0.0) == CPV_OK, "%s: reset: %s", m->name,
        cpv_last_error(b));
  snapshot_t x, y;
  snapshot(a, &x);
  snapshot(b, &y);
  CHECK(memcmp(&x, &y, sizeof(x)) == 0, "%s: differs after reset", m->name);
  cpv_close(a);
  cpv_close(b);
}

static void test_lookup_and_refusals(void) {
  CHECK(cpv_find_model("no_such_model") == NULL, "unknown name found");
  CHECK(cpv_find_model(NULL) == NULL, "NULL name found");
  CHECK(cpv_model_at(cpv_model_count()) == NULL, "index past the end");
  cpv_vehicle_t *v = (cpv_vehicle_t *)1;
  char error[512];
  CHECK(cpv_open_model(NULL, NULL, &v, error, sizeof(error)) == CPV_ERR_ARGUMENT && v == NULL,
        "NULL model accepted");

  const cpv_model_t *m = cpv_model_at(0);
  cpv_model_t bad = *m;
  bad.descriptor_schema = 99;
  CHECK(cpv_open_model(&bad, NULL, &v, error, sizeof(error)) == CPV_ERR_DESCRIPTOR && !v,
        "future schema accepted");
  bad = *m;
  bad.contract_version = 2;
  CHECK(cpv_open_model(&bad, NULL, &v, error, sizeof(error)) == CPV_ERR_DESCRIPTOR && !v,
        "contract version 2 accepted");
  bad = *m;
  bad.marker_value = 7;
  CHECK(cpv_open_model(&bad, NULL, &v, error, sizeof(error)) == CPV_ERR_DESCRIPTOR && !v,
        "wrong marker accepted");
  bad = *m;
  bad.fmi3.do_step = NULL;
  CHECK(cpv_open_model(&bad, NULL, &v, error, sizeof(error)) == CPV_ERR_LOAD && !v &&
            strstr(error, "fmi3DoStep"),
        "missing fmi3DoStep: %s", error);
  bad = *m;
  bad.instantiation_token = "00000000-0000-0000-0000-000000000000";
  CHECK(cpv_open_model(&bad, NULL, &v, error, sizeof(error)) == CPV_ERR_LOAD && !v,
        "foreign instantiation token accepted");
  /* The host's required capabilities apply to linked models too. */
  cpv_options_t o;
  cpv_options_init(&o);
  const char *const need[] = {"rotor_speed_v9"};
  o.required_capabilities = need;
  o.required_count = 1;
  CHECK(cpv_open_model(m, &o, &v, error, sizeof(error)) == CPV_ERR_CAPABILITY && !v,
        "missing host capability accepted");
}

/* Every linked model open at once, stepped interleaved: no shared state. */
static void test_all_at_once(void) {
  size_t n = cpv_model_count();
  cpv_vehicle_t *v[16] = {0};
  for (size_t i = 0; i < n && i < 16; i++) {
    v[i] = open_linked(cpv_model_at(i));
    if (v[i])
      CHECK(cpv_initialize(v[i], 0.0) == CPV_OK, "%s", cpv_last_error(v[i]));
  }
  for (int f = 1; f <= 50; f++)
    for (size_t i = 0; i < n && i < 16; i++)
      if (v[i])
        CHECK(cpv_advance_to(v[i], f * 0.004, NULL) <= CPV_WARNING, "%s: %s",
              cpv_model_at(i)->name, cpv_last_error(v[i]));
  for (size_t i = 0; i < n && i < 16; i++) {
    if (!v[i])
      continue;
    CHECK(fabs(cpv_time(v[i]) - 0.2) < 1e-9, "%s at t=%g", cpv_model_at(i)->name, cpv_time(v[i]));
    cpv_close(v[i]);
  }
}

int main(int argc, char **argv) {
  if (argc != 2) {
    fprintf(stderr, "usage: %s VEHICLES_DIR\n", argv[0]);
    return 2;
  }
  vehicles = argv[1];
  if (cpv_model_count() == 0) {
    fprintf(stderr, "no models are linked (configure with -DCPV_MODELS=...)\n");
    return 1;
  }
  for (size_t i = 0; i < cpv_model_count(); i++) {
    const cpv_model_t *m = cpv_model_at(i);
    CHECK(cpv_find_model(m->name) == m, "cpv_find_model(%s)", m->name);
    compare_run(m);
  }
  test_lookup_and_refusals();
  test_all_at_once();
  printf("test_linked_models: %zu model(s), %d checks, %d failures\n", cpv_model_count(), checks,
         failures);
  return failures != 0;
}
