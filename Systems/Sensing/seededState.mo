within ModelicaCPSLibrary.Systems.Sensing;
function seededState "The xorshift128+ state to draw from: initialState(streamId, seed) on the first draw, else the running state"
  input Integer stateIn[4] "Xorshift128plus state; ignored until seeded";
  input Boolean seeded "false on the first draw";
  input Integer streamId "Local seed of Xorshift128plus.initialState";
  input Integer seed "Global seed of Xorshift128plus.initialState";
  output Integer state[4];
protected
  Integer a[2];
  Integer b[2];
  Real r;
algorithm
  // Xorshift128plus.initialState(streamId, seed) with whole-array assignments
  // and scalar conditionals (what Rumoca generates code for).
  a := {if streamId == 0 and seed == 0 then 126247697 else streamId, seed};
  for i in 1:10 loop
    (r, a) := Modelica.Math.Random.Generators.Xorshift64star.random(a);
  end for;
  (r, b) := Modelica.Math.Random.Generators.Xorshift64star.random(a);
  state := {if seeded then stateIn[1] else a[1], if seeded then stateIn[2] else a[2],
            if seeded then stateIn[3] else b[1], if seeded then stateIn[4] else b[2]};
  annotation(Documentation(info="<html><p>Seeding at the first draw, instead of by the state's start
value, keeps <code>seed</code> and <code>streamId</code> settable in an exported FMU: Rumoca
evaluates start values at compile time. The draws equal those of a start-value seeding.</p></html>"));
end seededState;
