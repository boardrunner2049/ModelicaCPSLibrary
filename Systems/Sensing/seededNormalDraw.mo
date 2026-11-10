within ModelicaCPSLibrary.Systems.Sensing;
function seededNormalDraw "normalDraw that seeds the xorshift128+ stream on its first call"
  input Integer n;
  input Integer stateIn[4] "Xorshift128plus state; ignored until seeded";
  input Boolean seeded "false on the first draw: start from initialState(streamId, seed)";
  input Integer streamId "Local seed of Xorshift128plus.initialState";
  input Integer seed "Global seed of Xorshift128plus.initialState";
  output Real z[n];
  output Integer stateOut[4];
protected
  Integer s[4];
  Integer a[2];
  Integer b[2];
  Real r;
algorithm
  // Modelica.Math.Random.Generators.Xorshift128plus.initialState(streamId, seed),
  // written with whole-array assignments and scalar conditionals only (what
  // Rumoca generates code for): Xorshift64star.initialState, then one more
  // Xorshift64star draw for the second pair.
  a := {if streamId == 0 and seed == 0 then 126247697 else streamId, seed};
  for i in 1:10 loop
    (r, a) := Modelica.Math.Random.Generators.Xorshift64star.random(a);
  end for;
  (r, b) := Modelica.Math.Random.Generators.Xorshift64star.random(a);
  s := {if seeded then stateIn[1] else a[1], if seeded then stateIn[2] else a[2],
        if seeded then stateIn[3] else b[1], if seeded then stateIn[4] else b[2]};
  (z, stateOut) := normalDraw(n, s);
  annotation(Documentation(info="<html><p>Seeding inside the first draw, instead of by the state's
start value, keeps <code>seed</code> and <code>streamId</code> settable in an exported FMU: Rumoca
evaluates start values at compile time. The draws equal those of a start-value seeding with
<code>Xorshift128plus.initialState(streamId, seed)</code>.</p></html>"));
end seededNormalDraw;
