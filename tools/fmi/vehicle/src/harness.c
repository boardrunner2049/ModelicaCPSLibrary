/* Generic vehicle harness: any fire-compose multirotor (VirtualIO profile) as
 * an FMI 3 Co-Simulation FMU in the world runtime, flown through a scripted
 * mission by the generic flight controller (controller.c).
 *
 * vehicle_config.h is generated per configuration by tools/vehicle.py: the
 * vehicle description (VEHICLE), the mission (MISSION) and the glue between
 * the controller's sensor/PWM arrays and the world bindings (read_sensors,
 * write_pwm). The shared driver (lib/fire_harness.c) owns the time loop and
 * calls control() at every communication point; the truth record is only
 * traced, for checks.toml, never read here.
 *
 * Also writes out/controller.csv (estimate, reference, commands; every 10 ms).
 */
#include "controller.h"
#include "fire_harness.h"
#include "vehicle_config.h"

#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
  fc_t fc;
  FILE *log;
  int phase;
  long steps;
  double stuck_s; /* time armed on the ground while commanding more than 1.3 m g */
  bool warned;
} harness_t;

/* Diagnoses propulsion that cannot lift the vehicle (e.g. a winding reactance
 * that caps the rotor speed below hover): still on the ground after 2 s of
 * commanding a collective thrust above 1.3 m g. Returns nonzero (ends the run:
 * the mission cannot be flown) once diagnosed. */
static int check_lift(harness_t *h, double t, const mission_phase_t *ph) {
  const bool full = h->fc.armed && ph->kind == PHASE_FLY && -h->fc.pos[2] < 0.05 &&
                    h->fc.thrust > 1.3 * VEHICLE.mass * VEHICLE.gravity;
  h->stuck_s = full ? h->stuck_s + MISSION.step_s : 0;
  if (h->stuck_s >= 2.0 && !h->warned) {
    h->warned = true;
    printf("insufficient thrust: t = %.2f s: still on the ground after 2 s at a commanded thrust of %.2f m g "
           "(commands %.3f); the propulsion cannot lift the vehicle (hover needs %.0f rad/s per rotor)\n",
           t, h->fc.thrust / (VEHICLE.mass * VEHICLE.gravity), h->fc.command[0], VEHICLE.hover_omega[0]);
    wm_log_event((uint64_t)llround(t * 1e9), "vehicle-harness", "mission", "insufficient thrust");
  }
  return h->warned;
}

static const mission_phase_t *phase_at(harness_t *h, double t) {
  while (h->phase + 1 < MISSION.phase_count && t >= MISSION.phases[h->phase + 1].start_s - 1e-9) {
    h->phase++;
    char text[96];
    snprintf(text, sizeof text, "phase %s", MISSION.phases[h->phase].name);
    wm_log_event((uint64_t)llround(t * 1e9), "vehicle-harness", "mission", text);
  }
  return &MISSION.phases[h->phase];
}

static void log_row(harness_t *h, double t, const char *phase) {
  const fc_t *fc = &h->fc;
  fprintf(h->log, "%.4f,%s,%d,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,%.3f,%d,%.3f",
          t, phase, fc->armed, fc->ref_pos[0], fc->ref_pos[1], -fc->ref_pos[2], fc->pos[0], fc->pos[1],
          -fc->pos[2], fc->vel[0], fc->vel[1], -fc->vel[2], fc->ref_yaw * 180 / M_PI,
          atan2(fc->R[1][0], fc->R[0][0]) * 180 / M_PI, fc->tilt_deg, fc->thrust, fc->saturated,
          fc->yaw_scale);
  for (int i = 0; i < VEHICLE.n_rotors; ++i)
    fprintf(h->log, ",%.5f", fc->command[i]);
  fputc('\n', h->log);
}

static int control(double t, void *context) {
  harness_t *h = context;
  fc_sensors_t s;
  double pwm[VEHICLE_MAX_ACTUATORS];
  if (read_sensors(&s))
    return 1;
  const mission_phase_t *ph = phase_at(h, t);
  const bool was_armed = h->fc.armed;
  fc_update(&h->fc, &s, ph, pwm);
  if (check_lift(h, t, ph))
    return 1;
  if (was_armed != h->fc.armed)
    wm_log_event((uint64_t)llround(t * 1e9), "vehicle-harness", "mission",
                 h->fc.armed ? "armed" : "disarmed");
  if (h->log && h->steps++ % (long)fmax(1, llround(0.01 / MISSION.step_s)) == 0)
    log_row(h, t, ph->name);
  return write_pwm(pwm);
}

int main(void) {
  static harness_t h;
  if (fc_init(&h.fc, &VEHICLE, &MISSION, MISSION.step_s)) {
    fprintf(stderr, "%s: rotor effectiveness matrix is singular\n", VEHICLE.name);
    return 1;
  }
  printf("vehicle %s (%s, %s): %d rotors on %d channels, m = %.4g kg, motor tau ~ %.3g s\n",
         VEHICLE.name, VEHICLE.config, VEHICLE.propulsion, VEHICLE.n_rotors, VEHICLE.n_actuators,
         VEHICLE.mass, VEHICLE.motor_tau);
  printf("gains [1/s]: pos %.3g/%.3g  vel %.3g/%.3g  att %.3g  rate %.3g (actuator lag %.3g s%s)\n",
         h.fc.k.pos_xy, h.fc.k.pos_z, h.fc.k.vel_xy, h.fc.k.vel_z, h.fc.k.att, h.fc.k.rate, h.fc.tau_a,
         VEHICLE.motor_tau > h.fc.tau_a ? ", lead-compensated" : "");
  printf("mission %s: %d phases, %.1f s, step %g s\n", MISSION.name, MISSION.phase_count, MISSION.stop_s,
         MISSION.step_s);
  h.log = fopen("out/controller.csv", "w");
  if (h.log) {
    setvbuf(h.log, NULL, _IOLBF, 0); /* readable while a slow (P3) flight runs */
    fprintf(h.log, "t,phase,armed,ref_n,ref_e,ref_up,n,e,up,vn,ve,vup,ref_yaw_deg,yaw_deg,tilt_deg,"
                   "thrust,saturated,yaw_scale");
    for (int i = 0; i < VEHICLE.n_rotors; ++i)
      fprintf(h.log, ",cmd%d", i + 1);
    fputc('\n', h.log);
  }
  const fh_scenario_t scenario = {
      .title = VEHICLE.name,
      .stop_s = MISSION.stop_s,
      .step_s = MISSION.step_s,
      .print_s = 2.0,
      .inputs = NULL,
      .input_count = 0,
      .outputs = PRINTED,
      .output_count = FH_COUNT(PRINTED),
      .control = control,
      .control_context = &h,
  };
  const int result = fh_run(&scenario);
  if (h.log)
    fclose(h.log);
  return result;
}
