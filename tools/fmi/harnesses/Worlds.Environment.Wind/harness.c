/* FMI harness: Worlds.Environment.Wind.
 *
 * Constant wind source: wind = wind0, the air velocity in the NED world frame.
 * The default is calm ({0,0,0}), which would make every check vacuous, so world.toml
 * sets wind0 = {3, 4, -0.5} m/s: a 5 m/s horizontal wind blowing toward 053 deg
 * (i.e. from the South-West, 233 deg) with a 0.5 m/s updraft (Up is negative Down).
 * The checks establish that the tunable parameter reaches the output
 * unchanged, with NED signs, and stays constant for the whole run. */
#include "fire_harness.h"

static const fh_output_t outputs[] = {
    FH_OUTPUT(wind_1),
    FH_OUTPUT(wind_2),
    FH_OUTPUT(wind_3),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Worlds.Environment.Wind",
      .stop_s = 1.0,
      .step_s = 0.01,
      .print_s = 0.5,
      .inputs = NULL,
      .input_count = 0,
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
