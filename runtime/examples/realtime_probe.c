/* How fast a linked vehicle model runs: steps it at a host's frame rate and
 * reports simulated seconds per wall-clock second.
 *
 *   cpv_realtime NAME [FRAME_HZ]     (default 1200 Hz, ArduPilot SITL's rate)
 *
 * Steps for at most 0.5 s of simulated time or 3 s of wall time, whichever
 * comes first. Prints "NAME FACTOR" (FACTOR < 1: slower than real time).
 */
#include "cpvehicle/cpvehicle_model.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

static double now(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return t.tv_sec + 1e-9 * t.tv_nsec;
}

int main(int argc, char **argv) {
  if (argc < 2) {
    fprintf(stderr, "usage: cpv_realtime NAME [FRAME_HZ]\n");
    return 2;
  }
  const double hz = argc > 2 ? atof(argv[2]) : 1200.0;
  const cpv_model_t *model = cpv_find_model(argv[1]);
  if (!model || hz <= 0) {
    fprintf(stderr, "cpv_realtime: no linked model %s\n", argv[1]);
    return 2;
  }
  cpv_options_t options;
  cpv_options_init(&options);
  cpv_vehicle_t *vehicle = NULL;
  char error[512] = "";
  if (cpv_open_model(model, &options, &vehicle, error, sizeof error) != CPV_OK ||
      cpv_initialize(vehicle, 0) != CPV_OK) {
    fprintf(stderr, "cpv_realtime: %s\n", vehicle ? cpv_last_error(vehicle) : error);
    return 1;
  }
  const double start = now();
  double simulated = 0, reached = 0;
  for (long k = 1; simulated < 0.5 && now() - start < 3.0; k++) {
    if (cpv_advance_to(vehicle, k / hz, &reached) > CPV_WARNING) {
      fprintf(stderr, "cpv_realtime: %s\n", cpv_last_error(vehicle));
      return 1;
    }
    simulated = k / hz;
  }
  printf("%s %.3g\n", argv[1], simulated / (now() - start));
  cpv_close(vehicle);
  return 0;
}
