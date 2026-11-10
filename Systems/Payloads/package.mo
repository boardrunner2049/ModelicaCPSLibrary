within ModelicaCPSLibrary.Systems;
package Payloads "Payloads and accessories on a vehicle (ArduPilot's optional hardware that moves or loads the airframe): gripper, winch, parachute, sprayer, retractable landing gear, gimbal, power tether, generator"
  extends Modelica.Icons.Package;
  annotation(Documentation(info="<html><p>Each payload is attached at <code>position_b</code>, reads the
vehicle's truth and its servo pulse(s), and returns the force and moment it puts on the vehicle
(<code>force_b</code>, <code>moment_b</code> about the combined CG, body FRD). The composer sums
them with the host's external wrench (<code>Bus</code>) into the plant. Power sources (generator,
tether) supply the power module's pack instead. See README.md.</p></html>"));
end Payloads;
