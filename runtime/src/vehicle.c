/* CP Models vehicle runtime: model validation (descriptor file or linked model),
 * FMU lifecycle, ports, stepping. */
#define _POSIX_C_SOURCE 200809L
#include "cpvehicle/cpvehicle.h"
#include "cpvehicle/cpvehicle_model.h"

#include "fmi3FunctionTypes.h"
#include "json.h"
#include "sha256.h"

#include <dlfcn.h>
#include <limits.h>
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ERROR_SIZE 1024
#define LEGACY_PREFIX "fastdyn_legacy"
#define STANDARD_PROFILE "standard"
/* Communication points closer than this are treated as equal (1 ns). */
#define TIME_EPSILON 1e-9

typedef struct {
  char *capability;
  char *name;
  fmi3ValueReference vr;
  size_t count;
  double *start; /* count values, or NULL when the descriptor records none */
} field_t;

typedef struct {
  char *name;
  fmi3ValueReference vr;
  double value;
} parameter_t;

typedef enum { STATE_INSTANTIATED, STATE_INITIALIZED, STATE_FAILED, STATE_DEAD } lifecycle_t;

struct cpv_vehicle {
  char error[ERROR_SIZE];
  char fmu_message[ERROR_SIZE]; /* last FMU log message at warning or above */
  char *profile;
  char *model_name;
  char *instance_name;
  char **capabilities;
  size_t capability_count;
  field_t *fields;
  size_t field_count;
  parameter_t *overridable; /* every overridable parameter, value unused */
  size_t overridable_count;
  parameter_t *overrides; /* descriptor overrides followed by host overrides */
  size_t override_count, override_capacity;
  size_t descriptor_override_count; /* the first entries came from the descriptor */
  double *descriptor_values;        /* their descriptor values, restored by cpv_reset */
  int has_marker;
  fmi3ValueReference marker_vr;
  int marker_value;

  const field_t *pwm, *pwm_min, *pwm_max, *force, *moment;
  double *pwm_values;
  int pwm_dirty;
  double wrench[6];
  int wrench_dirty;

  double max_step_s;
  double time_s;
  lifecycle_t state;
  cpv_log_fn log;
  void *log_user;

  void *library;
  fmi3Instance instance;
  fmi3InstantiateCoSimulationTYPE *instantiate;
  fmi3FreeInstanceTYPE *free_instance;
  fmi3EnterInitializationModeTYPE *enter_initialization;
  fmi3ExitInitializationModeTYPE *exit_initialization;
  fmi3DoStepTYPE *do_step;
  fmi3GetFloat64TYPE *get_float64;
  fmi3SetFloat64TYPE *set_float64;
  fmi3GetInt32TYPE *get_int32;
  fmi3ResetTYPE *reset;
  fmi3TerminateTYPE *terminate;
};

/* ------------------------------------------------------------------ helpers */

static void set_error(cpv_vehicle_t *v, const char *format, ...) {
  va_list args;
  va_start(args, format);
  vsnprintf(v->error, sizeof(v->error), format, args);
  va_end(args);
}

static char *copy_string(const char *s) {
  if (!s)
    return NULL;
  size_t n = strlen(s) + 1;
  char *copy = malloc(n);
  if (copy)
    memcpy(copy, s, n);
  return copy;
}

static const char *json_string(const cpv_json_t *object, const char *key) {
  const cpv_json_t *v = cpv_json_get(object, key);
  return v && v->type == CPV_JSON_STRING ? v->string : NULL;
}

static int json_uint(const cpv_json_t *v, uint64_t max, uint64_t *out) {
  if (!v || v->type != CPV_JSON_NUMBER || v->number < 0 || v->number > (double)max ||
      v->number != floor(v->number))
    return -1;
  *out = (uint64_t)v->number;
  return 0;
}

static char *read_file(const char *path, size_t *length) {
  FILE *f = fopen(path, "rb");
  if (!f)
    return NULL;
  char *data = NULL;
  size_t size = 0, cap = 0, n;
  char chunk[8192];
  while ((n = fread(chunk, 1, sizeof(chunk), f)) > 0) {
    if (size + n + 1 > cap) {
      cap = (size + n + 1) * 2;
      char *grown = realloc(data, cap);
      if (!grown) {
        free(data);
        fclose(f);
        return NULL;
      }
      data = grown;
    }
    memcpy(data + size, chunk, n);
    size += n;
  }
  int failed = ferror(f);
  fclose(f);
  if (failed || !data) {
    free(data);
    return NULL;
  }
  data[size] = '\0';
  *length = size;
  return data;
}

/* dir/relative; relative paths in a descriptor are relative to its directory. */
static char *join_path(const char *descriptor_path, const char *relative) {
  if (relative[0] == '/')
    return copy_string(relative);
  const char *slash = strrchr(descriptor_path, '/');
  size_t dir = slash ? (size_t)(slash - descriptor_path) : 1;
  const char *base = slash ? descriptor_path : ".";
  size_t n = dir + 1 + strlen(relative) + 1;
  char *path = malloc(n);
  if (path)
    snprintf(path, n, "%.*s/%s", (int)dir, base, relative);
  return path;
}

static int finite_all(const double *values, size_t n) {
  for (size_t i = 0; i < n; i++)
    if (!isfinite(values[i]))
      return 0;
  return 1;
}

static const field_t *find_field(const cpv_vehicle_t *v, const char *name) {
  for (size_t i = 0; i < v->field_count; i++)
    if (strcmp(v->fields[i].name, name) == 0)
      return &v->fields[i];
  return NULL;
}

static int has_capability(const cpv_vehicle_t *v, const char *name) {
  for (size_t i = 0; i < v->capability_count; i++)
    if (strcmp(v->capabilities[i], name) == 0)
      return 1;
  return 0;
}

static void fmu_log(fmi3InstanceEnvironment env, fmi3Status status, fmi3String category,
                    fmi3String message) {
  cpv_vehicle_t *v = env;
  if (status >= fmi3Warning)
    snprintf(v->fmu_message, sizeof(v->fmu_message), "%s: %s", category ? category : "log",
             message ? message : "");
  if (v->log)
    v->log(v->log_user, (int)status, category ? category : "", message ? message : "");
  else if (status >= fmi3Warning)
    fprintf(stderr, "cpvehicle[%s] %s: %s\n", v->instance_name, category ? category : "log",
            message ? message : "");
}

/* -------------------------------------------------------------------- model */

static cpv_status_t add_override(cpv_vehicle_t *v, const char *name, double value) {
  const parameter_t *p = NULL;
  for (size_t i = 0; i < v->overridable_count; i++)
    if (strcmp(v->overridable[i].name, name) == 0)
      p = &v->overridable[i];
  if (!p) {
    set_error(v, "parameter %s is not overridable (only scalar Float64 parameters the "
                 "descriptor lists can be set)", name);
    return CPV_ERR_ARGUMENT;
  }
  if (!isfinite(value)) {
    set_error(v, "parameter %s: value must be finite", name);
    return CPV_ERR_ARGUMENT;
  }
  for (size_t i = 0; i < v->override_count; i++)
    if (strcmp(v->overrides[i].name, name) == 0) {
      v->overrides[i].value = value;
      return CPV_OK;
    }
  if (v->override_count == v->override_capacity) {
    size_t cap = v->override_capacity ? v->override_capacity * 2 : 8;
    parameter_t *grown = realloc(v->overrides, cap * sizeof(*grown));
    if (!grown)
      return CPV_ERR_NOMEM;
    v->overrides = grown;
    v->override_capacity = cap;
  }
  v->overrides[v->override_count] = (parameter_t){copy_string(name), p->vr, value};
  if (!v->overrides[v->override_count].name)
    return CPV_ERR_NOMEM;
  v->override_count++;
  return CPV_OK;
}

static cpv_status_t load_field(cpv_vehicle_t *v, const cpv_model_field_t *mf) {
  if (!mf->capability || !mf->name || mf->count == 0 || mf->count > (1u << 20)) {
    set_error(v, "descriptor: field %s.%s needs a valid vr and count",
              mf->capability ? mf->capability : "?", mf->name ? mf->name : "?");
    return CPV_ERR_DESCRIPTOR;
  }
  if (!has_capability(v, mf->capability)) {
    set_error(v, "descriptor: field %s belongs to undescribed capability %s", mf->name,
              mf->capability);
    return CPV_ERR_DESCRIPTOR;
  }
  field_t *f = &v->fields[v->field_count++];
  f->capability = copy_string(mf->capability);
  f->name = copy_string(mf->name);
  f->vr = (fmi3ValueReference)mf->vr;
  f->count = mf->count;
  if (!f->capability || !f->name)
    return CPV_ERR_NOMEM;
  if (mf->start) {
    if (!finite_all(mf->start, mf->count)) {
      set_error(v, "descriptor: field %s has a non-finite start value", mf->name);
      return CPV_ERR_DESCRIPTOR;
    }
    if (!(f->start = malloc(f->count * sizeof(double))))
      return CPV_ERR_NOMEM;
    memcpy(f->start, mf->start, f->count * sizeof(double));
  }
  return CPV_OK;
}

/* Validate a model description, from a descriptor file or linked into the
 * program, and copy it into the instance. The FMI functions are bound later. */
static cpv_status_t load_model(cpv_vehicle_t *v, const cpv_model_t *m, const cpv_options_t *o) {
  cpv_status_t s;
  if (m->descriptor_schema != CPV_DESCRIPTOR_SCHEMA) {
    set_error(v, "descriptor: unsupported descriptor_schema (this runtime reads %d)",
              CPV_DESCRIPTOR_SCHEMA);
    return CPV_ERR_DESCRIPTOR;
  }
  if (!m->contract_name || strcmp(m->contract_name, CPV_CONTRACT_NAME) != 0 ||
      m->contract_version != CPV_CONTRACT_VERSION) {
    set_error(v, "descriptor: not a %s version %d descriptor", CPV_CONTRACT_NAME,
              CPV_CONTRACT_VERSION);
    return CPV_ERR_DESCRIPTOR;
  }
  if (!m->profile) {
    set_error(v, "descriptor: profile is required");
    return CPV_ERR_DESCRIPTOR;
  }
  int legacy = strcmp(m->profile, STANDARD_PROFILE) != 0;
  if (legacy && strncmp(m->profile, LEGACY_PREFIX, strlen(LEGACY_PREFIX)) != 0) {
    set_error(v, "descriptor: unknown profile %s", m->profile);
    return CPV_ERR_DESCRIPTOR;
  }
  if (legacy && !o->allow_legacy_profile) {
    set_error(v, "descriptor uses the legacy profile %s (no version marker); the host must "
                 "explicitly allow legacy profiles", m->profile);
    return CPV_ERR_DESCRIPTOR;
  }
  if (!(v->profile = copy_string(m->profile)))
    return CPV_ERR_NOMEM;
  if (!legacy) {
    if (!m->has_marker || m->marker_value != CPV_CONTRACT_VERSION) {
      set_error(v, "descriptor: the standard profile requires a version %d marker",
                CPV_CONTRACT_VERSION);
      return CPV_ERR_DESCRIPTOR;
    }
    v->has_marker = 1;
    v->marker_vr = (fmi3ValueReference)m->marker_vr;
    v->marker_value = m->marker_value;
  }
  if (!m->instantiation_token) {
    set_error(v, "descriptor: artifact.instantiation_token is required");
    return CPV_ERR_DESCRIPTOR;
  }
  if (!(v->model_name = copy_string(m->model_name ? m->model_name : "")))
    return CPV_ERR_NOMEM;

  v->capabilities = calloc(m->capability_count ? m->capability_count : 1, sizeof(char *));
  v->fields = calloc(m->field_count ? m->field_count : 1, sizeof(field_t));
  if (!v->capabilities || !v->fields)
    return CPV_ERR_NOMEM;
  for (size_t i = 0; i < m->capability_count; i++) {
    if (!m->capabilities[i]) {
      set_error(v, "descriptor: capability %zu is malformed", i);
      return CPV_ERR_DESCRIPTOR;
    }
    if (!(v->capabilities[v->capability_count++] = copy_string(m->capabilities[i])))
      return CPV_ERR_NOMEM;
  }
  for (size_t i = 0; i < m->field_count; i++)
    if ((s = load_field(v, &m->fields[i])))
      return s;

  /* Capabilities required at prepare time and by this host. */
  for (size_t i = 0; i < m->required_count; i++)
    if (m->required_capabilities[i] && !has_capability(v, m->required_capabilities[i])) {
      set_error(v, "descriptor requires %s, which it does not describe",
                m->required_capabilities[i]);
      return CPV_ERR_DESCRIPTOR;
    }
  for (size_t i = 0; i < o->required_count; i++)
    if (!o->required_capabilities[i] || !has_capability(v, o->required_capabilities[i])) {
      set_error(v, "model %s does not provide %s, which the host requires (profile %s)",
                v->model_name, o->required_capabilities[i] ? o->required_capabilities[i] : "(null)",
                v->profile);
      return CPV_ERR_CAPABILITY;
    }

  v->overridable = calloc(m->parameter_count ? m->parameter_count : 1, sizeof(parameter_t));
  if (!v->overridable)
    return CPV_ERR_NOMEM;
  for (size_t i = 0; i < m->parameter_count; i++) {
    if (!m->parameters[i].name) {
      set_error(v, "descriptor: parameter %zu has no name", i);
      return CPV_ERR_DESCRIPTOR;
    }
    parameter_t *p = &v->overridable[v->overridable_count++];
    p->vr = (fmi3ValueReference)m->parameters[i].vr;
    if (!(p->name = copy_string(m->parameters[i].name)))
      return CPV_ERR_NOMEM;
  }
  for (size_t i = 0; i < m->override_count; i++) {
    if (!m->overrides[i].name) {
      set_error(v, "descriptor: override %zu has no name", i);
      return CPV_ERR_DESCRIPTOR;
    }
    if ((s = add_override(v, m->overrides[i].name, m->overrides[i].value)))
      return s == CPV_ERR_ARGUMENT ? CPV_ERR_DESCRIPTOR : s;
  }
  v->descriptor_override_count = v->override_count;
  if (v->override_count) {
    if (!(v->descriptor_values = malloc(v->override_count * sizeof(double))))
      return CPV_ERR_NOMEM;
    for (size_t i = 0; i < v->override_count; i++)
      v->descriptor_values[i] = v->overrides[i].value;
  }

  /* Ports the runtime drives directly. */
  v->pwm = find_field(v, "pwm");
  v->pwm_min = find_field(v, "pwm_min");
  v->pwm_max = find_field(v, "pwm_max");
  v->force = find_field(v, "external_force_frd");
  v->moment = find_field(v, "external_moment_frd");
  if (has_capability(v, "pwm_actuation_v1") && (!v->pwm || !v->pwm->start)) {
    set_error(v, "descriptor: pwm_actuation_v1 needs pwm with a complete start vector");
    return CPV_ERR_DESCRIPTOR;
  }
  if (has_capability(v, "external_wrench_v1") &&
      (!v->force || !v->moment || v->force->count != 3 || v->moment->count != 3 ||
       !v->force->start || !v->moment->start)) {
    set_error(v, "descriptor: external_wrench_v1 needs 3-element force and moment with starts");
    return CPV_ERR_DESCRIPTOR;
  }
  return CPV_OK;
}

#define BIND(field, member, type, name)                                                            \
  do {                                                                                             \
    if (!f->member) {                                                                              \
      set_error(v, "FMU library lacks %s", name);                                                  \
      return CPV_ERR_LOAD;                                                                         \
    }                                                                                              \
    v->field = (type *)f->member;                                                                  \
  } while (0)

static cpv_status_t bind_functions(cpv_vehicle_t *v, const cpv_fmi3_functions_t *f) {
  BIND(instantiate, instantiate_co_simulation, fmi3InstantiateCoSimulationTYPE,
       "fmi3InstantiateCoSimulation");
  BIND(free_instance, free_instance, fmi3FreeInstanceTYPE, "fmi3FreeInstance");
  BIND(enter_initialization, enter_initialization_mode, fmi3EnterInitializationModeTYPE,
       "fmi3EnterInitializationMode");
  BIND(exit_initialization, exit_initialization_mode, fmi3ExitInitializationModeTYPE,
       "fmi3ExitInitializationMode");
  BIND(do_step, do_step, fmi3DoStepTYPE, "fmi3DoStep");
  BIND(get_float64, get_float64, fmi3GetFloat64TYPE, "fmi3GetFloat64");
  BIND(set_float64, set_float64, fmi3SetFloat64TYPE, "fmi3SetFloat64");
  BIND(get_int32, get_int32, fmi3GetInt32TYPE, "fmi3GetInt32");
  BIND(reset, reset, fmi3ResetTYPE, "fmi3Reset");
  BIND(terminate, terminate, fmi3TerminateTYPE, "fmi3Terminate");
  return CPV_OK;
}

static void restore_inputs(cpv_vehicle_t *v) {
  if (v->pwm)
    memcpy(v->pwm_values, v->pwm->start, v->pwm->count * sizeof(double));
  v->pwm_dirty = v->pwm != NULL;
  if (v->force) {
    memcpy(v->wrench, v->force->start, 3 * sizeof(double));
    memcpy(v->wrench + 3, v->moment->start, 3 * sizeof(double));
  }
  v->wrench_dirty = v->force != NULL;
}

/* ----------------------------------------------------------------- descriptor */

/* A model description read from a descriptor file. Strings point into the
 * parsed JSON; the arrays are owned here. */
typedef struct {
  cpv_model_t model;
  const char **capabilities;
  const char **required;
  cpv_model_field_t *fields;
  cpv_model_parameter_t *parameters;
  cpv_model_override_t *overrides;
} json_model_t;

static void json_model_free(json_model_t *j) {
  for (size_t i = 0; j->fields && i < j->model.field_count; i++)
    free((void *)j->fields[i].start);
  free(j->capabilities);
  free(j->required);
  free(j->fields);
  free(j->parameters);
  free(j->overrides);
}

static cpv_status_t verify_digest(cpv_vehicle_t *v, const char *descriptor_path,
                                  const cpv_json_t *artifact, const char *path_key,
                                  const char *digest_key, char **resolved) {
  const char *relative = json_string(artifact, path_key);
  const char *expected = json_string(artifact, digest_key);
  if (!relative || !expected) {
    set_error(v, "descriptor: artifact.%s and artifact.%s are required", path_key, digest_key);
    return CPV_ERR_DESCRIPTOR;
  }
  char *path = join_path(descriptor_path, relative);
  if (!path)
    return CPV_ERR_NOMEM;
  char actual[65];
  if (cpv_sha256_file(path, actual)) {
    set_error(v, "descriptor: cannot read %s", path);
    free(path);
    return CPV_ERR_DESCRIPTOR;
  }
  if (strcmp(actual, expected) != 0) {
    set_error(v, "descriptor is stale: %s changed since it was prepared (sha256 %s, descriptor "
                 "records %s); prepare it again", path, actual, expected);
    free(path);
    return CPV_ERR_DESCRIPTOR;
  }
  if (resolved)
    *resolved = path;
  else
    free(path);
  return CPV_OK;
}

static cpv_status_t json_field(cpv_vehicle_t *v, const char *capability, const char *name,
                               const cpv_json_t *entry, cpv_model_field_t *f) {
  uint64_t vr, count;
  if (entry->type != CPV_JSON_OBJECT ||
      json_uint(cpv_json_get(entry, "vr"), UINT32_MAX, &vr) ||
      json_uint(cpv_json_get(entry, "count"), 1u << 20, &count) || count == 0) {
    set_error(v, "descriptor: field %s.%s needs a valid vr and count", capability, name);
    return CPV_ERR_DESCRIPTOR;
  }
  *f = (cpv_model_field_t){capability, name, (unsigned int)vr, (size_t)count, NULL};
  const cpv_json_t *start = cpv_json_get(entry, "start");
  if (start && start->type == CPV_JSON_ARRAY) {
    if (start->count != f->count) {
      set_error(v, "descriptor: field %s has %zu element(s) but %zu start value(s)", name,
                f->count, start->count);
      return CPV_ERR_DESCRIPTOR;
    }
    double *values = malloc(f->count * sizeof(double));
    if (!values)
      return CPV_ERR_NOMEM;
    f->start = values;
    for (size_t i = 0; i < f->count; i++) {
      if (start->items[i].type != CPV_JSON_NUMBER || !isfinite(start->items[i].number)) {
        set_error(v, "descriptor: field %s has a non-finite start value", name);
        return CPV_ERR_DESCRIPTOR;
      }
      values[i] = start->items[i].number;
    }
  } else if (start && start->type != CPV_JSON_NULL) {
    set_error(v, "descriptor: field %s has a malformed start", name);
    return CPV_ERR_DESCRIPTOR;
  }
  return CPV_OK;
}

/* Read the descriptor into a model description and check that it is bound to
 * the extracted files (SHA-256) it would load. */
static cpv_status_t model_from_json(cpv_vehicle_t *v, const char *path, const cpv_json_t *root,
                                    json_model_t *j, char **library, char **resources) {
  cpv_model_t *m = &j->model;
  uint64_t number;
  cpv_status_t s;
  if (root->type != CPV_JSON_OBJECT) {
    set_error(v, "descriptor: top level is not an object");
    return CPV_ERR_DESCRIPTOR;
  }
  m->descriptor_schema =
      json_uint(cpv_json_get(root, "descriptor_schema"), INT_MAX, &number) ? -1 : (int)number;
  const cpv_json_t *contract = cpv_json_get(root, "contract");
  m->contract_name = json_string(contract, "name");
  m->contract_version =
      json_uint(cpv_json_get(contract, "version"), INT_MAX, &number) ? -1 : (int)number;
  m->profile = json_string(root, "profile");
  const cpv_json_t *marker = cpv_json_get(root, "marker");
  uint64_t vr, value;
  if (marker && !json_uint(cpv_json_get(marker, "vr"), UINT32_MAX, &vr) &&
      !json_uint(cpv_json_get(marker, "value"), INT_MAX, &value)) {
    m->has_marker = 1;
    m->marker_vr = (unsigned int)vr;
    m->marker_value = (int)value;
  }
  /* Header problems are reported before the files are read. */
  if (m->descriptor_schema != CPV_DESCRIPTOR_SCHEMA || !m->contract_name ||
      strcmp(m->contract_name, CPV_CONTRACT_NAME) != 0 ||
      m->contract_version != CPV_CONTRACT_VERSION || !m->profile)
    return CPV_OK;

  const cpv_json_t *artifact = cpv_json_get(root, "artifact");
  if ((s = verify_digest(v, path, artifact, "model_description", "model_description_sha256",
                         NULL)) ||
      (s = verify_digest(v, path, artifact, "library", "library_sha256", library)))
    return s;
  m->instantiation_token = json_string(artifact, "instantiation_token");
  const char *res = json_string(artifact, "resources");
  if (res && !(*resources = join_path(path, res)))
    return CPV_ERR_NOMEM;
  m->model_name = json_string(artifact, "model_name");
  m->source_sha256 = json_string(artifact, "source_sha256");

  const cpv_json_t *caps = cpv_json_get(root, "capabilities");
  if (!caps || caps->type != CPV_JSON_OBJECT) {
    set_error(v, "descriptor: capabilities must be an object");
    return CPV_ERR_DESCRIPTOR;
  }
  size_t total = 0;
  for (size_t i = 0; i < caps->count; i++) {
    if (caps->items[i].type != CPV_JSON_OBJECT) {
      set_error(v, "descriptor: capability %s is malformed", caps->keys[i]);
      return CPV_ERR_DESCRIPTOR;
    }
    total += caps->items[i].count;
  }
  j->capabilities = calloc(caps->count ? caps->count : 1, sizeof(char *));
  j->fields = calloc(total ? total : 1, sizeof(cpv_model_field_t));
  if (!j->capabilities || !j->fields)
    return CPV_ERR_NOMEM;
  m->capabilities = j->capabilities;
  m->fields = j->fields;
  for (size_t i = 0; i < caps->count; i++) {
    j->capabilities[m->capability_count++] = caps->keys[i];
    const cpv_json_t *fields = &caps->items[i];
    for (size_t k = 0; k < fields->count; k++) {
      if (fields->items[k].type == CPV_JSON_NULL)
        continue; /* optional field the model does not export */
      s = json_field(v, caps->keys[i], fields->keys[k], &fields->items[k],
                     &j->fields[m->field_count]);
      if (j->fields[m->field_count].name)
        m->field_count++; /* owns its start array from here on */
      if (s)
        return s;
    }
  }

  const cpv_json_t *required = cpv_json_get(root, "required_capabilities");
  if (required && required->type == CPV_JSON_ARRAY) {
    if (!(j->required = calloc(required->count ? required->count : 1, sizeof(char *))))
      return CPV_ERR_NOMEM;
    m->required_capabilities = j->required;
    for (size_t i = 0; i < required->count; i++)
      if (required->items[i].type == CPV_JSON_STRING)
        j->required[m->required_count++] = required->items[i].string;
  }

  const cpv_json_t *params = cpv_json_get(root, "parameters");
  if (params && params->type == CPV_JSON_OBJECT) {
    if (!(j->parameters = calloc(params->count ? params->count : 1, sizeof(*j->parameters))))
      return CPV_ERR_NOMEM;
    m->parameters = j->parameters;
    for (size_t i = 0; i < params->count; i++) {
      if (json_uint(cpv_json_get(&params->items[i], "vr"), UINT32_MAX, &vr)) {
        set_error(v, "descriptor: parameter %s has no valid vr", params->keys[i]);
        return CPV_ERR_DESCRIPTOR;
      }
      j->parameters[m->parameter_count++] = (cpv_model_parameter_t){params->keys[i],
                                                                    (unsigned int)vr};
    }
  }
  const cpv_json_t *overrides = cpv_json_get(root, "overrides");
  if (overrides && overrides->type == CPV_JSON_OBJECT) {
    if (!(j->overrides = calloc(overrides->count ? overrides->count : 1, sizeof(*j->overrides))))
      return CPV_ERR_NOMEM;
    m->overrides = j->overrides;
    for (size_t i = 0; i < overrides->count; i++) {
      if (overrides->items[i].type != CPV_JSON_NUMBER) {
        set_error(v, "descriptor: override %s is not a number", overrides->keys[i]);
        return CPV_ERR_DESCRIPTOR;
      }
      j->overrides[m->override_count++] =
          (cpv_model_override_t){overrides->keys[i], overrides->items[i].number};
    }
  }
  return CPV_OK;
}

#define LOAD_SYMBOL(member, name)                                                                  \
  do {                                                                                             \
    *(void **)(&f->member) = dlsym(*handle, name);                                                 \
    if (!f->member) {                                                                              \
      set_error(v, "FMU library lacks %s", name);                                                  \
      return CPV_ERR_LOAD;                                                                         \
    }                                                                                              \
  } while (0)

static cpv_status_t load_library(cpv_vehicle_t *v, const char *library, void **handle,
                                 cpv_fmi3_functions_t *f) {
  *handle = dlopen(library, RTLD_NOW | RTLD_LOCAL);
  if (!*handle) {
    set_error(v, "cannot load %s: %s", library, dlerror());
    return CPV_ERR_LOAD;
  }
  LOAD_SYMBOL(instantiate_co_simulation, "fmi3InstantiateCoSimulation");
  LOAD_SYMBOL(free_instance, "fmi3FreeInstance");
  LOAD_SYMBOL(enter_initialization_mode, "fmi3EnterInitializationMode");
  LOAD_SYMBOL(exit_initialization_mode, "fmi3ExitInitializationMode");
  LOAD_SYMBOL(do_step, "fmi3DoStep");
  LOAD_SYMBOL(get_float64, "fmi3GetFloat64");
  LOAD_SYMBOL(set_float64, "fmi3SetFloat64");
  LOAD_SYMBOL(get_int32, "fmi3GetInt32");
  LOAD_SYMBOL(reset, "fmi3Reset");
  LOAD_SYMBOL(terminate, "fmi3Terminate");
  return CPV_OK;
}

/* ----------------------------------------------------------------- lifecycle */

void cpv_options_init(cpv_options_t *o) {
  memset(o, 0, sizeof(*o));
  o->max_step_s = 0.002;
}

static void destroy(cpv_vehicle_t *v) {
  if (!v)
    return;
  if (v->instance) {
    if (v->state == STATE_INITIALIZED && v->terminate)
      v->terminate(v->instance);
    if (v->free_instance)
      v->free_instance(v->instance);
  }
  if (v->library)
    dlclose(v->library);
  for (size_t i = 0; i < v->field_count; i++) {
    free(v->fields[i].capability);
    free(v->fields[i].name);
    free(v->fields[i].start);
  }
  for (size_t i = 0; i < v->capability_count; i++)
    free(v->capabilities[i]);
  for (size_t i = 0; i < v->overridable_count; i++)
    free(v->overridable[i].name);
  for (size_t i = 0; i < v->override_count; i++)
    free(v->overrides[i].name);
  free(v->fields);
  free(v->capabilities);
  free(v->overridable);
  free(v->overrides);
  free(v->descriptor_values);
  free(v->pwm_values);
  free(v->profile);
  free(v->model_name);
  free(v->instance_name);
  free(v);
}

static cpv_vehicle_t *create(const cpv_options_t *o, cpv_status_t *s) {
  cpv_vehicle_t *v = calloc(1, sizeof(*v));
  if (!v) {
    *s = CPV_ERR_NOMEM;
    return NULL;
  }
  v->instance_name = copy_string(o->instance_name ? o->instance_name : "cpvehicle");
  v->max_step_s = o->max_step_s;
  v->log = o->log;
  v->log_user = o->log_user;
  if (!v->instance_name)
    *s = CPV_ERR_NOMEM;
  else if (!(o->max_step_s >= 0) || !isfinite(o->max_step_s)) {
    set_error(v, "max_step_s must be finite and non-negative");
    *s = CPV_ERR_ARGUMENT;
  }
  return v;
}

/* After the model is loaded and its functions bound: input buffers and the instance. */
static cpv_status_t instantiate(cpv_vehicle_t *v, const char *token, const char *resource_uri) {
  if (v->pwm && !(v->pwm_values = malloc(v->pwm->count * sizeof(double))))
    return CPV_ERR_NOMEM;
  restore_inputs(v);
  v->instance = v->instantiate(v->instance_name, token, resource_uri, fmi3False, fmi3False,
                               fmi3False /* eventModeUsed */, fmi3False /* earlyReturnAllowed */,
                               NULL, 0, v, fmu_log, NULL);
  if (!v->instance) {
    set_error(v, "fmi3InstantiateCoSimulation failed for %s%s%s", v->model_name,
              v->fmu_message[0] ? ": " : "", v->fmu_message);
    return CPV_ERR_LOAD;
  }
  v->state = STATE_INSTANTIATED;
  return CPV_OK;
}

static cpv_status_t finish(cpv_vehicle_t *v, cpv_status_t s, cpv_vehicle_t **out, char *error,
                           size_t error_size) {
  if (s != CPV_OK) {
    if (error && error_size)
      snprintf(error, error_size, "%s", v && v->error[0] ? v->error : cpv_status_string(s));
    destroy(v);
    return s;
  }
  *out = v;
  return CPV_OK;
}

cpv_status_t cpv_open(const char *descriptor_path, const cpv_options_t *options,
                      cpv_vehicle_t **out, char *error, size_t error_size) {
  if (out)
    *out = NULL;
  if (error && error_size)
    error[0] = '\0';
  if (!descriptor_path || !out) {
    if (error && error_size)
      snprintf(error, error_size, "cpv_open: descriptor path and out are required");
    return CPV_ERR_ARGUMENT;
  }
  cpv_options_t defaults;
  cpv_options_init(&defaults);
  const cpv_options_t *o = options ? options : &defaults;
  cpv_status_t s = CPV_OK;
  cpv_vehicle_t *v = create(o, &s);
  char *library = NULL, *resources = NULL, *text = NULL, *resource_uri = NULL;
  cpv_json_t *root = NULL;
  json_model_t j = {0};
  size_t length = 0;
  if (s)
    goto done;
  if (!(text = read_file(descriptor_path, &length))) {
    set_error(v, "cannot read descriptor %s", descriptor_path);
    s = CPV_ERR_DESCRIPTOR;
    goto done;
  }
  if (!(root = cpv_json_parse(text, length, v->error, sizeof(v->error)))) {
    s = CPV_ERR_DESCRIPTOR;
    goto done;
  }
  if ((s = model_from_json(v, descriptor_path, root, &j, &library, &resources)) ||
      (s = load_model(v, &j.model, o)) ||
      (s = load_library(v, library, &v->library, &j.model.fmi3)) ||
      (s = bind_functions(v, &j.model.fmi3)))
    goto done;
  /* FMI 3 expects a file URI to the resources directory, ending in '/'. */
  if (resources) {
    size_t n = strlen(resources) + 9;
    if (!(resource_uri = malloc(n))) {
      s = CPV_ERR_NOMEM;
      goto done;
    }
    snprintf(resource_uri, n, "file://%s/", resources);
  }
  s = instantiate(v, j.model.instantiation_token, resource_uri);
done:
  json_model_free(&j);
  free(resource_uri);
  free(library);
  free(resources);
  free(text);
  cpv_json_free(root);
  return finish(v, s, out, error, error_size);
}

cpv_status_t cpv_open_model(const cpv_model_t *model, const cpv_options_t *options,
                            cpv_vehicle_t **out, char *error, size_t error_size) {
  if (out)
    *out = NULL;
  if (error && error_size)
    error[0] = '\0';
  if (!model || !out) {
    if (error && error_size)
      snprintf(error, error_size, "cpv_open_model: model and out are required");
    return CPV_ERR_ARGUMENT;
  }
  cpv_options_t defaults;
  cpv_options_init(&defaults);
  const cpv_options_t *o = options ? options : &defaults;
  cpv_status_t s = CPV_OK;
  cpv_vehicle_t *v = create(o, &s);
  if (!s && !(s = load_model(v, model, o)) && !(s = bind_functions(v, &model->fmi3)))
    s = instantiate(v, model->instantiation_token, model->resources_uri);
  return finish(v, s, out, error, error_size);
}

void cpv_close(cpv_vehicle_t *v) { destroy(v); }

cpv_status_t cpv_set_parameter(cpv_vehicle_t *v, const char *name, double value) {
  if (!v || !name)
    return CPV_ERR_ARGUMENT;
  if (v->state != STATE_INSTANTIATED) {
    set_error(v, "parameters can only be set before cpv_initialize");
    return CPV_ERR_STATE;
  }
  return add_override(v, name, value);
}

/* Map an FMI status; on anything but OK the message names the call and the
 * FMU's own last log message. Error and discard leave the instance failed. */
static cpv_status_t fmi_result(cpv_vehicle_t *v, fmi3Status status, const char *what) {
  static const char *names[] = {"fmi3OK", "fmi3Warning", "fmi3Discard", "fmi3Error", "fmi3Fatal"};
  if (status == fmi3OK) {
    v->fmu_message[0] = '\0';
    return CPV_OK;
  }
  const char *name = (unsigned)status <= fmi3Fatal ? names[status] : "an unknown status";
  set_error(v, "%s returned %s%s%s", what, name, v->fmu_message[0] ? ": " : "", v->fmu_message);
  v->fmu_message[0] = '\0';
  if (status == fmi3Warning)
    return CPV_WARNING;
  v->state = status == fmi3Fatal ? STATE_DEAD : STATE_FAILED;
  return CPV_ERR_FMU;
}

static cpv_status_t flush_inputs(cpv_vehicle_t *v, const char *phase) {
  cpv_status_t worst = CPV_OK, s;
  char what[96];
  if (v->pwm && v->pwm_dirty) {
    snprintf(what, sizeof(what), "fmi3SetFloat64(pwm) %s", phase);
    s = fmi_result(v, v->set_float64(v->instance, &v->pwm->vr, 1, v->pwm_values, v->pwm->count),
                   what);
    if (s >= CPV_ERR_ARGUMENT)
      return s;
    worst = s > worst ? s : worst;
    v->pwm_dirty = 0;
  }
  if (v->force && v->wrench_dirty) {
    snprintf(what, sizeof(what), "fmi3SetFloat64(external wrench) %s", phase);
    s = fmi_result(v, v->set_float64(v->instance, &v->force->vr, 1, v->wrench, 3), what);
    if (s >= CPV_ERR_ARGUMENT)
      return s;
    worst = s > worst ? s : worst;
    s = fmi_result(v, v->set_float64(v->instance, &v->moment->vr, 1, v->wrench + 3, 3), what);
    if (s >= CPV_ERR_ARGUMENT)
      return s;
    worst = s > worst ? s : worst;
    v->wrench_dirty = 0;
  }
  return worst;
}

cpv_status_t cpv_initialize(cpv_vehicle_t *v, double start_time_s) {
  if (!v)
    return CPV_ERR_ARGUMENT;
  if (v->state != STATE_INSTANTIATED) {
    set_error(v, "cpv_initialize requires a freshly opened or reset instance");
    return CPV_ERR_STATE;
  }
  if (!isfinite(start_time_s)) {
    set_error(v, "start time must be finite");
    return CPV_ERR_ARGUMENT;
  }
  v->error[0] = '\0';
  cpv_status_t worst = CPV_OK, s;
  s = fmi_result(v, v->enter_initialization(v->instance, fmi3False, 0.0, start_time_s, fmi3False,
                                            0.0),
                 "fmi3EnterInitializationMode");
  if (s >= CPV_ERR_ARGUMENT)
    return s;
  worst = s;
  for (size_t i = 0; i < v->override_count; i++) {
    char what[160];
    snprintf(what, sizeof(what), "fmi3SetFloat64(parameter %s)", v->overrides[i].name);
    s = fmi_result(v, v->set_float64(v->instance, &v->overrides[i].vr, 1, &v->overrides[i].value, 1),
                   what);
    if (s >= CPV_ERR_ARGUMENT)
      return s;
    worst = s > worst ? s : worst;
  }
  v->pwm_dirty = v->pwm != NULL;
  v->wrench_dirty = v->force != NULL;
  if ((s = flush_inputs(v, "during initialization")) >= CPV_ERR_ARGUMENT)
    return s;
  worst = s > worst ? s : worst;
  s = fmi_result(v, v->exit_initialization(v->instance), "fmi3ExitInitializationMode");
  if (s >= CPV_ERR_ARGUMENT)
    return s;
  worst = s > worst ? s : worst;
  v->state = STATE_INITIALIZED;
  v->time_s = start_time_s;
  if (v->has_marker) {
    fmi3Int32 value = 0;
    s = fmi_result(v, v->get_int32(v->instance, &v->marker_vr, 1, &value, 1),
                   "fmi3GetInt32(cp_vehicle_interface_version)");
    if (s >= CPV_ERR_ARGUMENT)
      return s;
    if (value != v->marker_value) {
      set_error(v, "the live instance reports cp_vehicle_interface_version %d; the descriptor "
                   "was checked for %d", (int)value, v->marker_value);
      v->state = STATE_FAILED;
      return CPV_ERR_DESCRIPTOR;
    }
  }
  return worst;
}

cpv_status_t cpv_advance_to(cpv_vehicle_t *v, double target, double *reached) {
  if (!v)
    return CPV_ERR_ARGUMENT;
  if (reached)
    *reached = v->time_s;
  if (v->state != STATE_INITIALIZED) {
    set_error(v, "cpv_advance_to requires an initialized, healthy instance");
    return CPV_ERR_STATE;
  }
  if (!isfinite(target) || target < v->time_s - TIME_EPSILON) {
    set_error(v, "cannot advance from t=%.9f s to t=%.9f s", v->time_s, target);
    return CPV_ERR_ARGUMENT;
  }
  cpv_status_t worst = CPV_OK, s;
  while (v->time_s + TIME_EPSILON < target) {
    double h = target - v->time_s;
    if (v->max_step_s > 0 && h > v->max_step_s)
      h = v->max_step_s;
    if ((s = flush_inputs(v, "before a step")) >= CPV_ERR_ARGUMENT)
      return s;
    worst = s > worst ? s : worst;
    fmi3Boolean event = fmi3False, terminate = fmi3False, early = fmi3False;
    fmi3Float64 last = v->time_s;
    char what[96];
    snprintf(what, sizeof(what), "fmi3DoStep at t=%.9f s, h=%.9f s", v->time_s, h);
    s = fmi_result(v, v->do_step(v->instance, v->time_s, h, fmi3True, &event, &terminate, &early,
                                 &last),
                   what);
    if (s >= CPV_ERR_ARGUMENT) {
      /* The step did not complete; the reached time is the last complete point. */
      if (reached)
        *reached = v->time_s;
      return s;
    }
    worst = s > worst ? s : worst;
    if (early) {
      /* earlyReturnAllowed was false: returning early is outside the supported subset. */
      if (last > v->time_s && last < v->time_s + h)
        v->time_s = last;
      set_error(v, "FMU returned early at t=%.9f s although early return was not allowed",
                v->time_s);
      v->state = STATE_FAILED;
      if (reached)
        *reached = v->time_s;
      return CPV_UNSUPPORTED;
    }
    v->time_s += h;
    if (reached)
      *reached = v->time_s;
    if (terminate) {
      set_error(v, "FMU requested termination at t=%.9f s", v->time_s);
      v->state = STATE_FAILED;
      return CPV_ERR_FMU;
    }
    if (event) {
      set_error(v, "FMU requested event handling at t=%.9f s; event mode is not supported",
                v->time_s);
      v->state = STATE_FAILED;
      return CPV_UNSUPPORTED;
    }
  }
  return worst;
}

cpv_status_t cpv_reset(cpv_vehicle_t *v) {
  if (!v)
    return CPV_ERR_ARGUMENT;
  if (v->state == STATE_DEAD) {
    set_error(v, "the FMU returned fmi3Fatal; the instance can only be closed");
    return CPV_ERR_STATE;
  }
  if (v->reset(v->instance) != fmi3OK) {
    set_error(v, "fmi3Reset failed");
    v->state = STATE_DEAD;
    return CPV_ERR_FMU;
  }
  /* Back to the descriptor's overrides and the model's start inputs. Host
   * overrides follow the descriptor's; a host override of a descriptor
   * parameter replaced its value in place. */
  for (size_t i = v->descriptor_override_count; i < v->override_count; i++)
    free(v->overrides[i].name);
  v->override_count = v->descriptor_override_count;
  for (size_t i = 0; i < v->override_count; i++)
    v->overrides[i].value = v->descriptor_values[i];
  v->state = STATE_INSTANTIATED;
  v->time_s = 0;
  v->error[0] = '\0';
  restore_inputs(v);
  return CPV_OK;
}

/* ------------------------------------------------------------------- ports */

static cpv_status_t read_field(cpv_vehicle_t *v, const field_t *f, double *values) {
  if (v->state != STATE_INITIALIZED) {
    set_error(v, "outputs are only valid on an initialized, healthy instance");
    return CPV_ERR_STATE;
  }
  char what[160];
  snprintf(what, sizeof(what), "fmi3GetFloat64(%s)", f->name);
  cpv_status_t s = fmi_result(v, v->get_float64(v->instance, &f->vr, 1, values, f->count), what);
  if (s >= CPV_ERR_ARGUMENT)
    return s;
  if (!finite_all(values, f->count)) {
    set_error(v, "%s is not finite at t=%.9f s", f->name, v->time_s);
    return CPV_ERR_FMU;
  }
  return s;
}

static cpv_status_t read_named(cpv_vehicle_t *v, const char *capability, const char *name,
                               double *values, size_t count) {
  if (!has_capability(v, capability)) {
    set_error(v, "model %s does not provide %s", v->model_name, capability);
    return CPV_UNSUPPORTED;
  }
  const field_t *f = find_field(v, name);
  if (!f) {
    set_error(v, "model %s does not export %s", v->model_name, name);
    return CPV_UNSUPPORTED;
  }
  if (f->count != count) {
    set_error(v, "%s has %zu element(s), not %zu", name, f->count, count);
    return CPV_ERR_ARGUMENT;
  }
  return read_field(v, f, values);
}

#define WORST(a, b) ((a) > (b) ? (a) : (b))
#define READ(cap, name, dst, n)                                                                    \
  do {                                                                                             \
    cpv_status_t r_ = read_named(v, cap, name, dst, n);                                            \
    if (r_ >= CPV_UNSUPPORTED)                                                                     \
      return r_;                                                                                   \
    worst = WORST(worst, r_);                                                                      \
  } while (0)

double cpv_time(const cpv_vehicle_t *v) { return v ? v->time_s : NAN; }
int cpv_has_capability(const cpv_vehicle_t *v, const char *c) {
  return v && c && has_capability(v, c);
}
const char *cpv_profile(const cpv_vehicle_t *v) { return v ? v->profile : NULL; }
const char *cpv_model_name(const cpv_vehicle_t *v) { return v ? v->model_name : NULL; }

size_t cpv_pwm_count(const cpv_vehicle_t *v) { return v && v->pwm ? v->pwm->count : 0; }

cpv_status_t cpv_pwm_start(const cpv_vehicle_t *v, size_t channel, double *pulse_us) {
  if (!v || !pulse_us)
    return CPV_ERR_ARGUMENT;
  if (!v->pwm)
    return CPV_UNSUPPORTED;
  if (channel >= v->pwm->count)
    return CPV_ERR_ARGUMENT;
  *pulse_us = v->pwm->start[channel];
  return CPV_OK;
}

cpv_status_t cpv_set_pwm(cpv_vehicle_t *v, size_t channel, double pulse_us) {
  if (!v)
    return CPV_ERR_ARGUMENT;
  if (!v->pwm) {
    set_error(v, "model %s does not provide pwm_actuation_v1", v->model_name);
    return CPV_UNSUPPORTED;
  }
  if (channel >= v->pwm->count) {
    set_error(v, "channel %zu is outside the model's %zu PWM channel(s)", channel, v->pwm->count);
    return CPV_ERR_ARGUMENT;
  }
  if (!isfinite(pulse_us)) {
    set_error(v, "channel %zu: pulse width must be finite", channel);
    return CPV_ERR_ARGUMENT;
  }
  if (v->pwm_values[channel] != pulse_us) {
    v->pwm_values[channel] = pulse_us;
    v->pwm_dirty = 1;
  }
  return CPV_OK;
}

cpv_status_t cpv_set_pwm_vector(cpv_vehicle_t *v, const double *pulse_us, size_t count) {
  if (!v || !pulse_us)
    return CPV_ERR_ARGUMENT;
  if (!v->pwm)
    return CPV_UNSUPPORTED;
  if (count != v->pwm->count || !finite_all(pulse_us, count)) {
    set_error(v, "pwm vector must have %zu finite element(s)", v->pwm->count);
    return CPV_ERR_ARGUMENT;
  }
  memcpy(v->pwm_values, pulse_us, count * sizeof(double));
  v->pwm_dirty = 1;
  return CPV_OK;
}

cpv_status_t cpv_pwm_range(cpv_vehicle_t *v, double *min_us, double *max_us) {
  if (!v || !min_us || !max_us)
    return CPV_ERR_ARGUMENT;
  if (!v->pwm_min || !v->pwm_max) {
    set_error(v, "model %s does not export pwm_min and pwm_max", v->model_name);
    return CPV_UNSUPPORTED;
  }
  cpv_status_t a = read_field(v, v->pwm_min, min_us);
  if (a >= CPV_ERR_ARGUMENT)
    return a;
  cpv_status_t b = read_field(v, v->pwm_max, max_us);
  return WORST(a, b);
}

cpv_status_t cpv_set_external_wrench(cpv_vehicle_t *v, const double force[3],
                                     const double moment[3]) {
  if (!v || !force || !moment)
    return CPV_ERR_ARGUMENT;
  if (!v->force) {
    set_error(v, "model %s does not provide external_wrench_v1", v->model_name);
    return CPV_UNSUPPORTED;
  }
  if (!finite_all(force, 3) || !finite_all(moment, 3)) {
    set_error(v, "external wrench must be finite");
    return CPV_ERR_ARGUMENT;
  }
  memcpy(v->wrench, force, 3 * sizeof(double));
  memcpy(v->wrench + 3, moment, 3 * sizeof(double));
  v->wrench_dirty = 1;
  return CPV_OK;
}

cpv_status_t cpv_get_state(cpv_vehicle_t *v, cpv_kinematic_state_t *out) {
  if (!v || !out)
    return CPV_ERR_ARGUMENT;
  cpv_status_t worst = CPV_OK;
  READ("kinematic_state_v1", "state.position_ned", out->position_ned, 3);
  READ("kinematic_state_v1", "state.velocity_ned", out->velocity_ned, 3);
  READ("kinematic_state_v1", "state.quaternion_wb", out->quaternion_wb, 4);
  READ("kinematic_state_v1", "state.angular_velocity_frd", out->angular_velocity_frd, 3);
  return worst;
}

cpv_status_t cpv_get_origin(cpv_vehicle_t *v, cpv_origin_t *out) {
  if (!v || !out)
    return CPV_ERR_ARGUMENT;
  cpv_status_t worst = CPV_OK;
  READ("kinematic_state_v1", "lat0", &out->latitude_deg, 1);
  READ("kinematic_state_v1", "lon0", &out->longitude_deg, 1);
  READ("kinematic_state_v1", "ground_alt_wgs84", &out->altitude_wgs84_m, 1);
  return worst;
}

cpv_status_t cpv_get_imu(cpv_vehicle_t *v, cpv_imu_t *out) {
  if (!v || !out)
    return CPV_ERR_ARGUMENT;
  cpv_status_t worst = CPV_OK;
  READ("imu_v1", "accel", out->accel_frd, 3);
  READ("imu_v1", "gyro", out->gyro_frd, 3);
  return worst;
}

cpv_status_t cpv_get_mag(cpv_vehicle_t *v, cpv_mag_t *out) {
  if (!v || !out)
    return CPV_ERR_ARGUMENT;
  cpv_status_t worst = CPV_OK;
  READ("magnetometer_v1", "mag", out->field_frd_gauss, 3);
  return worst;
}

cpv_status_t cpv_get_gnss(cpv_vehicle_t *v, cpv_gnss_t *out) {
  if (!v || !out)
    return CPV_ERR_ARGUMENT;
  cpv_status_t worst = CPV_OK;
  double gps[3];
  READ("gnss_v1", "gps", gps, 3);
  READ("gnss_v1", "vel_ned", out->velocity_ned, 3);
  out->latitude_deg = gps[0];
  out->longitude_deg = gps[1];
  out->altitude_wgs84_m = gps[2];
  out->has_yaw = find_field(v, "yaw_deg") != NULL;
  out->yaw_deg = NAN;
  if (out->has_yaw)
    READ("gnss_v1", "yaw_deg", &out->yaw_deg, 1);
  return worst;
}

cpv_status_t cpv_get_baro(cpv_vehicle_t *v, cpv_baro_t *out) {
  if (!v || !out)
    return CPV_ERR_ARGUMENT;
  cpv_status_t worst = CPV_OK;
  READ("barometer_v1", "baro_pressure_pa", &out->pressure_pa, 1);
  READ("barometer_v1", "baro_temperature_c", &out->temperature_c, 1);
  out->has_altitude = find_field(v, "baro_altitude_m") != NULL;
  out->altitude_m = NAN;
  if (out->has_altitude)
    READ("barometer_v1", "baro_altitude_m", &out->altitude_m, 1);
  out->has_climb_rate = find_field(v, "baro_climb_rate_mps") != NULL;
  out->climb_rate_mps = NAN;
  if (out->has_climb_rate)
    READ("barometer_v1", "baro_climb_rate_mps", &out->climb_rate_mps, 1);
  return worst;
}

cpv_status_t cpv_get_field(cpv_vehicle_t *v, const char *name, double *values, size_t count) {
  if (!v || !name || !values)
    return CPV_ERR_ARGUMENT;
  const field_t *f = find_field(v, name);
  if (!f) {
    set_error(v, "model %s does not export contract field %s", v->model_name, name);
    return CPV_UNSUPPORTED;
  }
  if (f->count != count) {
    set_error(v, "%s has %zu element(s), not %zu", name, f->count, count);
    return CPV_ERR_ARGUMENT;
  }
  return read_field(v, f, values);
}

const char *cpv_last_error(const cpv_vehicle_t *v) { return v ? v->error : ""; }

const char *cpv_status_string(cpv_status_t s) {
  switch (s) {
  case CPV_OK: return "ok";
  case CPV_WARNING: return "warning";
  case CPV_UNSUPPORTED: return "unsupported";
  case CPV_ERR_ARGUMENT: return "invalid argument";
  case CPV_ERR_DESCRIPTOR: return "descriptor error";
  case CPV_ERR_CAPABILITY: return "missing capability";
  case CPV_ERR_LOAD: return "load error";
  case CPV_ERR_STATE: return "invalid state";
  case CPV_ERR_FMU: return "FMU error";
  case CPV_ERR_NOMEM: return "out of memory";
  }
  return "unknown status";
}
