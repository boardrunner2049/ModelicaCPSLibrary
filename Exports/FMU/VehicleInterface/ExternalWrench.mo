within ModelicaCPSLibrary.Exports.FMU.VehicleInterface;
partial model ExternalWrench "external_wrench_v1: host-supplied non-gravity wrench at the combined CG"
  input Real external_force_frd[3](each unit="N", each start=0)
    "Force applied at the combined CG, resolved in body FRD [N]";
  input Real external_moment_frd[3](each unit="N.m", each start=0)
    "Moment about the combined CG, resolved in body FRD [N.m]";
end ExternalWrench;
