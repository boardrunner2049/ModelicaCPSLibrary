within ModelicaCPSLibrary.Systems;
package Peripherals "Optional devices on a vehicle: sensors the autopilot reads (rangefinder, airspeed, optical flow, RPM, power module, temperature, external AHRS, visual odometry, beacons, precision landing) and their sampled device models"
  extends Modelica.Icons.Package;
  annotation(Documentation(info="<html><p>Each kind has an ideal level (continuous, the
measurand at the mount from the vehicle's truth) and a sampled level (the ideal measurand through
<code>Sampler</code>: sample period, latency, bias, scale error, Gauss-Markov drift, white noise,
quantization and clipping). A device is rigidly mounted: <code>position_b</code> from the combined
CG and <code>R_bs</code>, its axes to body FRD. The composer (tools/fire-compose) adds the selected
devices to a vehicle export and exports their readings as the vehicle interface's optional
capabilities (contract/vehicle_interface.toml). See README.md.</p></html>"));
end Peripherals;
