/* Wrapper.mo owns coherent antenna truth; this scenario sends receiver commands.
 * Shared fire_harness owns stepping and tracing. No live state is set directly. */
#include "fire_harness.h"

#include <stdio.h>

#define NAV(ID, OP, MASK, DYN, ELEV, PDOP, SPEED, DIST) \
  {ID, 6, 36, OP, MASK, DYN, 3, 0, 1, ELEV, PDOP, 25, 100, 350, SPEED, 60, 0, 0, DIST, 0}

static const struct {
  double at;
  double command[20];
  int response_status;
} requests[] = {
    {.05, NAV(1, 0, 0, 0, 5, 3, 1, 2), 1},
    {.09, NAV(2, 1, 3, 8, 7, 3, 1, 2), 2},
    {.13, NAV(3, 0, 0, 0, 5, 3, 1, 2), 1},
    {.65, NAV(4, 1, 16, 0, 5, 1, 1, 2), 2},
    {.69, NAV(5, 0, 0, 0, 5, 3, 1, 2), 1},
    {.85, NAV(6, 1, 16, 0, 5, 3, 1, 2), 2},
    {1.25, NAV(7, 1, 64, 0, 5, 3, 0, 0), 2},
    {1.29, NAV(8, 0, 0, 0, 5, 3, 1, 2), 1},
    {1.45, NAV(9, 1, 64, 0, 5, 3, 2.55, 65535), 2},
    {1.49, NAV(10, 0, 0, 0, 5, 3, 1, 2), 1},
    {1.65, NAV(11, 1, 5, 3, 5, 3, 1, 2), 4},
    {1.69, NAV(12, 0, 0, 0, 5, 3, 1, 2), 1},
    {1.735, {13, 6, 0, 0}, 1},
    {1.775, {14, 10, 4, 0}, 1},
    {1.835, NAV(15, 1, 0, 0, 5, 0, 2.55, 0), 2},
};

static wm_status_t (*const setters[20])(double) = {
    set_command_1, set_command_2, set_command_3, set_command_4, set_command_5,
    set_command_6, set_command_7, set_command_8, set_command_9, set_command_10,
    set_command_11, set_command_12, set_command_13, set_command_14, set_command_15,
    set_command_16, set_command_17, set_command_18, set_command_19, set_command_20,
};

static int control(double t, void *context) {
  size_t *next = context;
  if (*next) {
    double id, status;
    if (get_response_1(&id) != WM_OK || get_response_4(&status) != WM_OK ||
        id != requests[*next - 1].command[0] || status != requests[*next - 1].response_status) {
      fprintf(stderr, "previous M9N request has no matching response at %.6f s\n", t);
      return 1;
    }
  }
  if (*next == FH_COUNT(requests) || t + 1e-12 < requests[*next].at)
    return 0;
  /* Payload first, event identity last; hold the entire request until the next one. */
  for (size_t i = 1; i < 20; ++i)
    if (setters[i](requests[*next].command[i]) != WM_OK)
      return 1;
  if (setters[0](requests[*next].command[0]) != WM_OK)
    return 1;
  ++*next;
  return 0;
}

static const fh_output_t outputs[] = {
    FH_OUTPUT(navpvt_1), FH_OUTPUT(navpvt_5), FH_OUTPUT(navpvt_12),
    FH_OUTPUT(position_1), FH_OUTPUT(position_2), FH_OUTPUT(position_3),
    FH_OUTPUT(velocity_1), FH_OUTPUT(response_1), FH_OUTPUT(response_4),
};

int main(void) {
  size_t next = 0;
  const fh_scenario_t scenario = {
      .title = "Systems.Sensing.GNSS.Device",
      .stop_s = 1.91,
      .step_s = .002,
      .print_s = .2,
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
      .control = control,
      .control_context = &next,
  };
  return fh_run(&scenario);
}
