/* FMI harness: Tests.NedFrdContract.
 *
 * A closed test model of the library's frame convention migration: the same
 * physical rigid-body motion integrated twice, once in the legacy ENU world /
 * FLU body basis (oldBody, oldImu) and once in NED / FRD (body, sensors,
 * VirtualIO adapter values), with the inputs of the second derived from the
 * first through W (ENU->NED) and B = diag(1,-1,-1) (FLU->FRD): full inertia
 * B J B, attitude, gravity, force/moment, sensor mounts, GNSS lever arm and
 * the rotor hub wrench. Equation-section asserts compare both at each
 * instant. There are no inputs; the scenario runs 2 s (20x the model's
 * StopTime) at 1 ms so the bodies tumble and translate substantially.
 *
 * checks.toml re-establishes the whole contract at every sample from the
 * trace (attitude, position, velocity, acceleration, rates, IMU, adapter,
 * magnetometer, GNSS position/velocity, barometric altitude sign, hub
 * wrench), and adds the physics both bodies must obey on their own:
 * Newton's law with the correct gravity sign per basis and Euler's
 * rotational equation with the full inertia tensor. Because both bodies use
 * the same RigidBody6DOF, the relative checks cannot see a common-mode
 * integration fault, so the initial state and the 2 s state of both bodies
 * are also pinned against ref_traj.py (this directory), an independent scipy
 * integration of the legacy-basis body (the FMU matches it to ~1e-12). */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.NedFrdContract",
      .stop_s = 2.0,
      .step_s = 0.001,
      .print_s = 0.25,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
