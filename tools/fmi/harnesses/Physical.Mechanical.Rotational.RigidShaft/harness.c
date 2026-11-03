/* FMI harness: Physical.Mechanical.Rotational.RigidShaft (through Wrapper.mo).
 *
 * Rigid shaft J = 2e-3 kg.m2 with viscous friction b = 0.01 N.m.s/rad to a
 * fixed housing (time constant J/b = 0.2 s), driven by a prescribed drive
 * torque and an opposing load torque. The shaft starts spinning backwards,
 * omega_start = -2 rad/s (tunable-parameter override in world.toml):
 *   0-0.1 s  no torque                  friction decays the initial spin
 *   0.1 s  drive 0 -> 0.1 N.m          spin-up through zero to 10 rad/s
 *   1.5 s  load 0 -> 0.15 N.m          net torque -0.05: reversal to -5 rad/s
 *   2.5 s  both torques removed         free coast-down, exp(-t/0.2)
 * checks.toml compares speed and angle against the closed-form first-order
 * solution J w' = tau_d - tau_l - b w, the initial acceleration tau/J,
 * friction loss b w^2, kinetic energy J w^2/2, and that the housing reaction
 * equals the rate of change of shaft angular momentum (zero at steady state),
 * and that the omega_start parameter sets the initial state. Not covered:
 * drive and load act on one rigid inertia, so a swap of the drive and load
 * flange connections is unobservable (inherent to a rigid shaft). */
#include "fire_harness.h"

static const fh_input_t inputs[] = {
    FH_INPUT(drivetorque, FH_PULSE(0.1, 2.4, 0.0, 0.1)),
    FH_INPUT(loadtorque, FH_PULSE(1.5, 1.0, 0.0, 0.15)),
};

static const fh_output_t outputs[] = {
    FH_OUTPUT(omega),
    FH_OUTPUT(acceleration),
    FH_OUTPUT(frictionloss),
    FH_OUTPUT(kineticenergy),
};

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Physical.Mechanical.Rotational.RigidShaft",
      .stop_s = 3.5,
      .step_s = 0.001,
      .print_s = 0.1,
      .inputs = inputs,
      .input_count = FH_COUNT(inputs),
      .outputs = outputs,
      .output_count = FH_COUNT(outputs),
  };
  return fh_run(&scenario);
}
