/* FMI harness: Systems.Sensing.GNSS.Sensor.
 *
 * Ideal GNSS: antenna position/velocity in local NED plus fixed biases
 * (world.toml: positionBias = (1.0, -0.5, 0.3) m, velocityBias = (0.05, -0.02, 0.10) m/s).
 * The antenna flies a clockwise-from-above circle (North -> East) of radius 20 m at
 * 0.5 rad/s while climbing 1 m/s from 10 m height:
 *   p = (20 cos wt, 20 sin wt, -10 - t),  v = dp/dt = (-10 sin wt, 10 cos wt, -1).
 * checks.toml compares positions/velocities at quarter turns against this closed
 * form and, at every sample, the bias-corrected kinematics: radius 20 m, speed 10 m/s,
 * velocity tangential to the circle, r x v = +R^2 w (North -> East sense), and the
 * Down velocity equal to the slope of the Down position (climb is negative Down). */
#include "fire_harness.h"

#include <math.h>

#define RADIUS 20.0
#define OMEGA 0.5

static double pn(double t) { return RADIUS * cos(OMEGA * t); }
static double pe(double t) { return RADIUS * sin(OMEGA * t); }
static double pd(double t) { return -10.0 - 1.0 * t; }
static double vn(double t) { return -RADIUS * OMEGA * sin(OMEGA * t); }
static double ve(double t) { return RADIUS * OMEGA * cos(OMEGA * t); }

static const fh_input_t inputs[] = {
    FH_INPUT(position_w_1, FH_FUNCTION(pn)),
    FH_INPUT(position_w_2, FH_FUNCTION(pe)),
    FH_INPUT(position_w_3, FH_FUNCTION(pd)),
    FH_INPUT(velocity_w_1, FH_FUNCTION(vn)),
    FH_INPUT(velocity_w_2, FH_FUNCTION(ve)),
    FH_INPUT(velocity_w_3, FH_CONSTANT(-1.0)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(position_1), FH_OUTPUT(position_2), FH_OUTPUT(position_3),
    FH_OUTPUT(velocity_1), FH_OUTPUT(velocity_2), FH_OUTPUT(velocity_3),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Systems.Sensing.GNSS.Sensor",
      .stop_s = 13.0,
      .step_s = 0.01,
      .print_s = 1.0,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
