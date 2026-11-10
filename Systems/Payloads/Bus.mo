within ModelicaCPSLibrary.Systems.Payloads;
model Bus "The payloads' forces and moments summed with the host's external wrench, for the plant"
  parameter Integer n(min=1) = 1;
  input Real hostForce[3](each unit="N") = zeros(3);
  input Real hostMoment[3](each unit="N.m") = zeros(3);
  input Real force[n, 3](each unit="N");
  input Real moment[n, 3](each unit="N.m");
  output Real totalForce[3](each unit="N");
  output Real totalMoment[3](each unit="N.m");
equation
  totalForce = hostForce + rowSum(force);
  totalMoment = hostMoment + rowSum(moment);
end Bus;
