within ModelicaCPSLibrary.Exports.FMU;
package VehicleInterface "Reference partials of the CP Models vehicle interface; see docs/vehicle-interface.md"
  extends Modelica.Icons.Package;
  annotation(Documentation(info="<html><p>Each partial declares the exported variables of one
capability of the vehicle interface (contract/vehicle_interface.toml). A vehicle export extends
Base and the capabilities it provides and supplies the equations. Conformance is decided on the
exported FMU metadata, not on inheritance from these classes.</p></html>"));
end VehicleInterface;
