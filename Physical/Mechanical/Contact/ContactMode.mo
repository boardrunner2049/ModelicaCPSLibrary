within ModelicaCPSLibrary.Physical.Mechanical.Contact;

model ContactMode "Geometric contact state, including initially touching points"
  input Real gap(unit = "m") "Positive outside the surface";
  output Boolean contact;
equation
  contact = gap <= 0;
end ContactMode;
