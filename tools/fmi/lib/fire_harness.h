/* Shared driver for the FIRE FMI harnesses.
 *
 * A harness is one FIRE model exported as an FMI 3 Co-Simulation FMU, loaded
 * through the world runtime's generated bindings (world_bindings.h). Each
 * harness.c only declares its scenario: which world inputs are driven by
 * which signal, which world outputs are printed, and how long to run.
 * fh_run() owns initialization, the time loop, input application, output
 * sampling, finiteness checks and teardown. Physical acceptance checks live
 * in checks.toml and are evaluated over the trace by tools/check_trace.py.
 */
#ifndef FIRE_HARNESS_H
#define FIRE_HARNESS_H

#include "world_bindings.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
  FH_SIGNAL_CONSTANT, /* a */
  FH_SIGNAL_STEP,     /* before a: b, from a on: c */
  FH_SIGNAL_RAMP,     /* b before a, linear to d at c, d afterwards */
  FH_SIGNAL_SINE,     /* a + b * sin(2 pi c t + d) */
  FH_SIGNAL_PULSE,    /* c, except d on [a, a + b) */
  FH_SIGNAL_FUNCTION  /* fn(t) */
} fh_signal_kind_t;

typedef struct {
  fh_signal_kind_t kind;
  double a, b, c, d;
  double (*fn)(double t);
} fh_signal_t;

#define FH_CONSTANT(value) {FH_SIGNAL_CONSTANT, (value), 0, 0, 0, NULL}
#define FH_STEP(at, before, after) {FH_SIGNAL_STEP, (at), (before), (after), 0, NULL}
#define FH_RAMP(t0, v0, t1, v1) {FH_SIGNAL_RAMP, (t0), (v0), (t1), (v1), NULL}
#define FH_SINE(offset, amplitude, hz, phase)                                                      \
  {FH_SIGNAL_SINE, (offset), (amplitude), (hz), (phase), NULL}
#define FH_PULSE(at, width, base, high) {FH_SIGNAL_PULSE, (at), (width), (base), (high), NULL}
#define FH_FUNCTION(function) {FH_SIGNAL_FUNCTION, 0, 0, 0, 0, (function)}

double fh_signal_value(fh_signal_t signal, double t);

/* One driven world input. Exactly one setter is non-null; Integer inputs take
 * the rounded signal, Boolean inputs take signal > 0.5. */
typedef struct {
  const char *name;
  wm_status_t (*set_f64)(double);
  wm_status_t (*set_i32)(int32_t);
  wm_status_t (*set_bool)(bool);
  fh_signal_t signal;
} fh_input_t;

#define FH_INPUT(alias, signal) {#alias, set_##alias, NULL, NULL, signal}
#define FH_INPUT_INT(alias, signal) {#alias, NULL, set_##alias, NULL, signal}
#define FH_INPUT_BOOL(alias, signal) {#alias, NULL, NULL, set_##alias, signal}

/* One printed world output. Exactly one getter is non-null. */
typedef struct {
  const char *name;
  wm_status_t (*get_f64)(double *);
  wm_status_t (*get_i32)(int32_t *);
  wm_status_t (*get_bool)(bool *);
} fh_output_t;

#define FH_OUTPUT(alias) {#alias, get_##alias, NULL, NULL}
#define FH_OUTPUT_INT(alias) {#alias, NULL, get_##alias, NULL}
#define FH_OUTPUT_BOOL(alias) {#alias, NULL, NULL, get_##alias}

typedef struct {
  const char *title;
  double stop_s;  /* simulated duration */
  double step_s;  /* communication step; a multiple of [World].step */
  double print_s; /* console sampling period; 0 prints only the final row */
  const fh_input_t *inputs;
  size_t input_count;
  const fh_output_t *outputs;
  size_t output_count;
  /* Optional closed loop: called at every communication point t (after the
   * outputs are sampled, before the inputs for [t, t + step) are applied) with
   * `control_context`. It may read world outputs and write world inputs through
   * the generated accessors; inputs it writes hold over the step like the
   * signals above. A nonzero return ends the run as a runtime failure. */
  int (*control)(double t, void *control_context);
  void *control_context;
} fh_scenario_t;

#define FH_COUNT(array) (sizeof(array) / sizeof((array)[0]))
#define FH_NONE NULL, 0

/* Runs the scenario. Returns 0 on success, 1 if the runtime or FMU failed,
 * 2 if an output became non-finite. */
int fh_run(const fh_scenario_t *scenario);

#endif
