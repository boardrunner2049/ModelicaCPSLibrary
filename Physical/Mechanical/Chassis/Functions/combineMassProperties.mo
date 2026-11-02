within ModelicaCPSLibrary.Physical.Mechanical.Chassis.Functions;

function combineMassProperties "Rotate and translate each part's full inertia tensor"
  input ModelicaCPSLibrary.Physical.Mechanical.MassProperties parts[:];
  output ModelicaCPSLibrary.Physical.Mechanical.MassProperties combined;
protected
  Real offset[3];
  Real rotatedInertia[3, 3];
  Real orthogonality[3, 3];
  Real rotationDeterminant;
  Real inertiaScale;
  Real secondMoment[3, 3]
    "Normalized integral of r*r', required positive semidefinite for a physical inertia";
  Real secondMomentDeterminant;
algorithm
  combined.mass := 0;
  combined.r_C := zeros(3);
  combined.R_Cj := identity(3);
  combined.inertia := zeros(3, 3);

  for i in 1:size(parts, 1) loop
    assert(parts[i].mass >= 0, "Part mass must be nonnegative");
    inertiaScale := max(abs(parts[i].inertia));
    assert(parts[i].mass > 0 or inertiaScale == 0,
      "A zero-mass part cannot carry nonzero mass inertia");
    orthogonality := transpose(parts[i].R_Cj) * parts[i].R_Cj;
    rotationDeterminant :=
      parts[i].R_Cj[1, 1] * (parts[i].R_Cj[2, 2] * parts[i].R_Cj[3, 3] - parts[i].R_Cj[2, 3] * parts[i].R_Cj[3, 2])
      - parts[i].R_Cj[1, 2] * (parts[i].R_Cj[2, 1] * parts[i].R_Cj[3, 3] - parts[i].R_Cj[2, 3] * parts[i].R_Cj[3, 1])
      + parts[i].R_Cj[1, 3] * (parts[i].R_Cj[2, 1] * parts[i].R_Cj[3, 2] - parts[i].R_Cj[2, 2] * parts[i].R_Cj[3, 1]);
    assert(abs(rotationDeterminant - 1) < 1e-8,
      "Part inertia axes must be a right-handed rotation");
    for j in 1:3 loop
      assert(parts[i].inertia[j, j] >= 0, "Part diagonal inertia must be nonnegative");
      for k in 1:3 loop
        assert(abs(orthogonality[j, k] - (if j == k then 1 else 0)) < 1e-8,
          "Part inertia rotation must be orthonormal");
        assert(abs(parts[i].inertia[j, k] - parts[i].inertia[k, j]) <= 1e-10 * max(inertiaScale, 1e-30),
          "Part inertia tensor must be symmetric");
      end for;
    end for;

    // J = integral(r*r') = trace(I)/2*identity(3) - I. J >= 0
    // also enforces the principal-moment triangle inequalities, unlike I > 0 alone.
    secondMoment := (0.5 * sum(parts[i].inertia[j, j] for j in 1:3) * identity(3)
      - parts[i].inertia) / max(inertiaScale, 1e-30);
    secondMomentDeterminant :=
      secondMoment[1, 1] * (secondMoment[2, 2] * secondMoment[3, 3] - secondMoment[2, 3] * secondMoment[3, 2])
      - secondMoment[1, 2] * (secondMoment[2, 1] * secondMoment[3, 3] - secondMoment[2, 3] * secondMoment[3, 1])
      + secondMoment[1, 3] * (secondMoment[2, 1] * secondMoment[3, 2] - secondMoment[2, 2] * secondMoment[3, 1]);
    for j in 1:3 loop
      assert(secondMoment[j, j] >= -1e-10,
        "Part inertia must represent a physical mass distribution (principal-moment triangle inequalities)");
      for k in (j + 1):3 loop
        assert(secondMoment[j, j] * secondMoment[k, k] - secondMoment[j, k] * secondMoment[k, j] >= -1e-10,
          "Part inertia must represent a physical mass distribution (nonnegative second-moment principal minors)");
      end for;
    end for;
    assert(secondMomentDeterminant >= -1e-10,
      "Part inertia must represent a physical mass distribution (nonnegative second-moment determinant)");
    combined.mass := combined.mass + parts[i].mass;
    combined.r_C := combined.r_C + parts[i].mass * parts[i].r_C;
  end for;

  if combined.mass > 0 then
    combined.r_C := combined.r_C / combined.mass;
  end if;

  for i in 1:size(parts, 1) loop
    offset := parts[i].r_C - combined.r_C;
    rotatedInertia := parts[i].R_Cj * parts[i].inertia * transpose(parts[i].R_Cj);
    for j in 1:3 loop
      for k in 1:3 loop
        combined.inertia[j, k] := combined.inertia[j, k] + rotatedInertia[j, k]
          + parts[i].mass * ((if j == k then offset * offset else 0) - offset[j] * offset[k]);
      end for;
    end for;
  end for;
end combineMassProperties;
