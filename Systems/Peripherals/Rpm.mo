within ModelicaCPSLibrary.Systems.Peripherals;
model Rpm "R0: shaft speeds in revolutions per minute, continuous"
  parameter Integer n(min=1) = 1;
  input Real speed[n](each unit="rad/s");
  output Real rpm[n];
  output Boolean valid = true;
  output Real validity = 1 "1 if valid, else 0";
equation
  for i in 1:n loop
    rpm[i]=abs(speed[i])*30/Modelica.Constants.pi;
  end for;
end Rpm;
