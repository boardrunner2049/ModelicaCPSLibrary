/* CP Models vehicle runtime: vehicle models linked into the host program.
 *
 * `python -m cp_vehicle_interface prepare MODEL.fmu OUT_DIR --shim NAME` checks
 * the FMU exactly as for a descriptor and also writes OUT_DIR/shim/: the FMU's
 * C sources and cpv_model_NAME.c, which defines `const cpv_model_t
 * cpv_model_NAME` (the resolved value references, shapes, start values,
 * capabilities and parameters, and the model's FMI 3 entry points).
 *
 * Building the runtime with -DCPV_MODELS="OUT_DIR/shim;..." compiles those
 * models into libcpvehicle with every model symbol except cpv_model_NAME made
 * local, so several models link into one program. The host then needs no
 * descriptor file and no shared library at run time:
 *
 *   cpv_open_model(cpv_find_model("rover"), &options, &vehicle, error, size);
 *
 * Everything after opening is the same API as cpvehicle.h.
 */
#ifndef CPVEHICLE_MODEL_H
#define CPVEHICLE_MODEL_H

#include "cpvehicle/cpvehicle.h"

#ifdef __cplusplus
extern "C" {
#endif

/* An FMI 3 function of the linked model; the runtime casts it back to its type. */
typedef void (*cpv_fmi3_fn)(void);

typedef struct {
  cpv_fmi3_fn instantiate_co_simulation;
  cpv_fmi3_fn free_instance;
  cpv_fmi3_fn enter_initialization_mode;
  cpv_fmi3_fn exit_initialization_mode;
  cpv_fmi3_fn do_step;
  cpv_fmi3_fn get_float64;
  cpv_fmi3_fn set_float64;
  cpv_fmi3_fn get_int32;
  cpv_fmi3_fn reset;
  cpv_fmi3_fn terminate;
} cpv_fmi3_functions_t;

typedef struct {
  const char *capability;
  const char *name;
  unsigned int vr;
  size_t count;
  const double *start; /* count values, or NULL */
} cpv_model_field_t;

typedef struct {
  const char *name;
  unsigned int vr;
} cpv_model_parameter_t;

typedef struct {
  const char *name;
  double value;
} cpv_model_override_t;

typedef struct cpv_model {
  int descriptor_schema;     /* CPV_DESCRIPTOR_SCHEMA the shim was generated for */
  const char *contract_name; /* CPV_CONTRACT_NAME */
  int contract_version;      /* CPV_CONTRACT_VERSION */
  const char *name;          /* the --shim NAME */
  const char *profile;
  const char *model_name;
  const char *instantiation_token;
  const char *resources_uri; /* NULL: the model has no resources */
  const char *source_sha256; /* provenance: the FMU the shim was generated from */
  int has_marker;
  unsigned int marker_vr;
  int marker_value;
  const char *const *capabilities;
  size_t capability_count;
  const char *const *required_capabilities; /* required when the shim was generated */
  size_t required_count;
  const cpv_model_field_t *fields;
  size_t field_count;
  const cpv_model_parameter_t *parameters; /* overridable scalar Float64 parameters */
  size_t parameter_count;
  const cpv_model_override_t *overrides; /* prepare --set values */
  size_t override_count;
  cpv_fmi3_functions_t fmi3;
} cpv_model_t;

/* Validate the model description and instantiate it, as cpv_open does for a descriptor. */
cpv_status_t cpv_open_model(const cpv_model_t *model, const cpv_options_t *options,
                            cpv_vehicle_t **out, char *error, size_t error_size);

/* Models linked into this program (cmake -DCPV_MODELS=...), in build order. */
size_t cpv_model_count(void);
const cpv_model_t *cpv_model_at(size_t index);
/* The linked model with this --shim name, or NULL. */
const cpv_model_t *cpv_find_model(const char *name);

#ifdef __cplusplus
}
#endif

#endif /* CPVEHICLE_MODEL_H */
