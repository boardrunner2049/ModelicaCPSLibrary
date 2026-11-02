within fire_modelica_models.Systems.Sensing;
function normalDraw "Explicit local RNG; n independent standard normal variates"
  input Integer n;
  input Integer stateIn[Modelica.Math.Random.Generators.Xorshift128plus.nState];
  output Real z[n];
  output Integer stateOut[Modelica.Math.Random.Generators.Xorshift128plus.nState];
protected
  Real u1;
  Real u2;
algorithm
  stateOut := stateIn;
  for j in 1:n loop
    (u1,stateOut) := Modelica.Math.Random.Generators.Xorshift128plus.random(stateOut);
    (u2,stateOut) := Modelica.Math.Random.Generators.Xorshift128plus.random(stateOut);
    z[j] := sqrt(-2*log(max(u1,Modelica.Constants.small)))*cos(2*Modelica.Constants.pi*u2);
  end for;
end normalDraw;
