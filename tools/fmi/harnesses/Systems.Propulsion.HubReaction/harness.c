/* FMI harness: Systems.Propulsion.HubReaction (through Wrapper.mo).
 *
 * HubReaction adds the axial spin-acceleration reaction of the rotor to the
 * aerodynamic hub moment (fixed-carrier approximation):
 *   moment_r = aerodynamicMoment_r - {0, 0, spinSign*(J+JLoad)*acceleration}
 * with J + JLoad = 1e-4 + 2e-4 = 3e-4 kg.m2. The wrapper evaluates the same
 * inputs on a spinSign = +1 and a spinSign = -1 instance.
 * Scenario: roll/pitch aerodynamic moments (1 Hz sine, constant) that must
 * pass through untouched, a slow yaw-moment ramp, and spin acceleration
 * steps 0 -> +1000 -> -2000 -> 0 rad/s2 (spin-up, hard braking, constant
 * speed). checks.toml verifies the closed-form yaw moment for each spin
 * sense (opposite reactions, equal at zero acceleration), and that roll and
 * pitch are unaffected. */
#include "fire_harness.h"

static double spin_acceleration(double t) {
  if (t < 0.2 - 1e-12)
    return 0.0;
  if (t < 0.5 - 1e-12)
    return 1000.0;
  if (t < 0.8 - 1e-12)
    return -2000.0;
  return 0.0;
}

static const fh_input_t inputs[] = {
    FH_INPUT(aerodynamicmoment_r_1, FH_SINE(0.0, 0.01, 1.0, 0.0)),
    FH_INPUT(aerodynamicmoment_r_2, FH_CONSTANT(-0.02)),
    FH_INPUT(aerodynamicmoment_r_3, FH_RAMP(0.0, -0.005, 1.0, 0.005)),
    FH_INPUT(acceleration, FH_FUNCTION(spin_acceleration)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(momentpositive_r_1), FH_OUTPUT(momentpositive_r_3),
    FH_OUTPUT(momentnegative_r_1), FH_OUTPUT(momentnegative_r_3),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Systems.Propulsion.HubReaction",
      .stop_s = 1.0,
      .step_s = 0.001,
      .print_s = 0.1,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
