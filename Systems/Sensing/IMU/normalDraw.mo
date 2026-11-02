within ModelicaCPSLibrary.Systems.Sensing.IMU;
function normalDraw "Twelve independent standard normals and explicit next RNG state"
  input Integer stateIn[Modelica.Math.Random.Generators.Xorshift128plus.nState];
  output Real z[12];
  output Integer stateOut[Modelica.Math.Random.Generators.Xorshift128plus.nState];
protected
  Real u1;
  Real u2;
  Real radius;
algorithm
  stateOut := stateIn;
  for j in 1:6 loop
    (u1,stateOut) := Modelica.Math.Random.Generators.Xorshift128plus.random(stateOut);
    (u2,stateOut) := Modelica.Math.Random.Generators.Xorshift128plus.random(stateOut);
    radius := sqrt(-2*log(max(u1,Modelica.Constants.small)));
    z[2*j-1] := radius*cos(2*Modelica.Constants.pi*u2);
    z[2*j] := radius*sin(2*Modelica.Constants.pi*u2);
  end for;
end normalDraw;
