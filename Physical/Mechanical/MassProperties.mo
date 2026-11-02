within ModelicaCPSLibrary.Physical.Mechanical;

record MassProperties "A physical part's mass and inertia about its own centre of mass"
  Real mass(min = 0, unit = "kg") = 0;
  Real r_C[3](each unit = "m") = {0, 0, 0}
    "Part centre of mass in the fixed chassis reference frame C";
  Real R_Cj[3, 3] = identity(3)
    "Rotation from the part's inertia axes j to chassis axes C";
  Real inertia[3, 3](each unit = "kg.m2") = zeros(3, 3)
    "Inertia about the part centre of mass, resolved in axes j";
  annotation(Documentation(info = "<html><p>This record contains only numerical mass properties. Physical-part componentId metadata belongs to the composer configuration and manifest; fire-compose rejects duplicate nonempty IDs before generating Modelica. Direct Modelica callers must account for each physical part exactly once.</p></html>"));
end MassProperties;
