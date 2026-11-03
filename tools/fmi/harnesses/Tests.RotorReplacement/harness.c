/* FMI harness: Tests.RotorReplacement (closed test model, no inputs).
 *
 * Scenario: a QuadX MultirotorPlant (1.5 kg, gravity off, east-facing start)
 * carries four replacement blades (a QuadraticBlade subclass with kT = 0.5e-5)
 * held at 100 rad/s; 1 s (experiment: 0.05 s) at a 1 ms step so the motion is
 * measurable. world.toml makes the configuration non-symmetric: rotor 1 gets
 * kQ = 3e-7 (others 1e-7), giving a net +2 mN.m yaw moment, and the start
 * position is moved to {0.5, -0.3, -1}.
 *
 * checks.toml establishes: the class-modifier kT reaches every blade (0.05 N
 * each, -0.2 N total along body -z); each rotor's arm moment r x F has the
 * sign/size of its own QuadX position (so a permuted position or a transposed
 * R_br would show); each yaw reaction has its rotor's spin sign and kQ (the
 * overridden one only on rotor 1); roll/pitch cancel, net yaw 2 mN.m gives
 * alpha_z = Mz/Jzz and yaw = pi/2 + 0.025 t^2; the closed-form climb with no
 * horizontal drift from the overridden start; hub air velocity (climb rate
 * along the thrust axis, w_z r in plane); legs never touch (gap, contact flags,
 * zero gear force).
 *
 * No outputs exist, so the console prints only time; values are in the trace. */
#include "fire_harness.h"

int main(void) {
  const fh_scenario_t scenario = {
      .title = "Tests.RotorReplacement",
      .stop_s = 1.0,
      .step_s = 0.001,
      .print_s = 0.1,
      .inputs = NULL,
      .input_count = 0,
      .outputs = NULL,
      .output_count = 0,
  };
  return fh_run(&scenario);
}
