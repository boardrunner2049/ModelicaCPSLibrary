#define _POSIX_C_SOURCE 200809L
#include "fire_harness.h"

#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

double fh_signal_value(fh_signal_t s, double t) {
  switch (s.kind) {
  case FH_SIGNAL_CONSTANT:
    return s.a;
  case FH_SIGNAL_STEP:
    return t < s.a ? s.b : s.c;
  case FH_SIGNAL_RAMP:
    if (t <= s.a)
      return s.b;
    if (t >= s.c)
      return s.d;
    return s.b + (s.d - s.b) * (t - s.a) / (s.c - s.a);
  case FH_SIGNAL_SINE:
    return s.a + s.b * sin(2.0 * M_PI * s.c * t + s.d);
  case FH_SIGNAL_PULSE:
    return (t >= s.a && t < s.a + s.b) ? s.d : s.c;
  case FH_SIGNAL_FUNCTION:
    return s.fn ? s.fn(t) : NAN;
  }
  return NAN;
}

static uint64_t to_ns(double seconds) { return (uint64_t)llround(seconds * 1e9); }

static int apply_inputs(const fh_scenario_t *sc, double t) {
  for (size_t i = 0; i < sc->input_count; ++i) {
    const fh_input_t *in = &sc->inputs[i];
    double value = fh_signal_value(in->signal, t);
    wm_status_t s = WM_ERR_STATE;
    if (in->set_f64)
      s = in->set_f64(value);
    else if (in->set_i32)
      s = in->set_i32((int32_t)lround(value));
    else if (in->set_bool)
      s = in->set_bool(value > 0.5);
    if (s != WM_OK) {
      fprintf(stderr, "input %s = %g at t=%g s failed: %s\n", in->name, value, t, wm_last_error());
      return 1;
    }
  }
  return 0;
}

/* Reads one output as a double; returns nonzero on runtime failure. */
static int read_output(const fh_output_t *out, double *value) {
  wm_status_t s = WM_ERR_STATE;
  if (out->get_f64) {
    s = out->get_f64(value);
  } else if (out->get_i32) {
    int32_t v = 0;
    s = out->get_i32(&v);
    *value = v;
  } else if (out->get_bool) {
    bool v = false;
    s = out->get_bool(&v);
    *value = v ? 1.0 : 0.0;
  }
  if (s != WM_OK) {
    fprintf(stderr, "output %s read failed: %s\n", out->name, wm_last_error());
    return 1;
  }
  return 0;
}

static void print_header(const fh_scenario_t *sc) {
  printf("%10s", "t [s]");
  for (size_t i = 0; i < sc->output_count; ++i)
    printf(" %14.14s", sc->outputs[i].name);
  putchar('\n');
}

/* Samples every output at t; prints the row when `print` is set. Returns 1 on
 * runtime failure, 2 when an output is not finite. */
static int sample_outputs(const fh_scenario_t *sc, double t, bool print) {
  int result = 0;
  if (print)
    printf("%10.4f", t);
  for (size_t i = 0; i < sc->output_count; ++i) {
    double value = NAN;
    if (read_output(&sc->outputs[i], &value))
      return 1;
    if (!isfinite(value)) {
      fprintf(stderr, "output %s is not finite at t=%g s\n", sc->outputs[i].name, t);
      result = 2;
    }
    if (print)
      printf(" % 14.6g", value);
  }
  if (print)
    putchar('\n');
  return result;
}

static int run_loop(const fh_scenario_t *sc) {
  const uint64_t stop = to_ns(sc->stop_s), step = to_ns(sc->step_s);
  const uint64_t print = sc->print_s > 0 ? to_ns(sc->print_s) : 0;
  uint64_t next_print = 0;
  if (step == 0) {
    fprintf(stderr, "scenario step must be positive\n");
    return 1;
  }
  print_header(sc);
  for (uint64_t now = 0;; now += step) {
    if (now > stop)
      now = stop;
    double t = (double)now * 1e-9;
    if (now > 0 && wm_advance_to(now) != WM_OK) {
      fprintf(stderr, "advance to t=%g s failed: %s\n", t, wm_last_error());
      return 1;
    }
    bool show = now == stop || (print && now >= next_print);
    if (show && print)
      next_print += print;
    int sampled = sample_outputs(sc, t, show);
    if (sampled)
      return sampled;
    if (now == stop)
      return 0;
    /* Inputs hold over [now, now + step): a zero-order hold at the
     * communication points, as a co-simulation master applies them. */
    if (sc->control && sc->control(t, sc->control_context)) {
      fprintf(stderr, "controller failed at t=%g s\n", t);
      return 1;
    }
    if (apply_inputs(sc, t))
      return 1;
  }
}

int fh_run(const fh_scenario_t *sc) {
  printf("== %s (%.3g s, step %.3g s)\n", sc->title, sc->stop_s, sc->step_s);
  if (wm_init() != WM_OK) {
    fprintf(stderr, "world initialization failed: %s\n", wm_last_error());
    return 1;
  }
  wm_log_event(0, "fire-harness", "lifecycle", sc->title);
  int result = apply_inputs(sc, 0.0);
  if (!result)
    result = run_loop(sc);
  wm_log_event(to_ns(sc->stop_s), "fire-harness", "lifecycle",
               result ? "scenario failed" : "scenario completed");
  wm_destroy();
  printf("== %s: %s\n", sc->title,
         result == 0 ? "ran to completion" : result == 2 ? "NON-FINITE OUTPUT" : "RUNTIME FAILURE");
  return result;
}
