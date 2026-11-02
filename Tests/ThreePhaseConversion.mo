within ModelicaCPSLibrary.Tests;
model ThreePhaseConversion "ABC angle, peak ke, torque and signed-current conversion at prescribed speed"
  Physical.Electrical.Electromechanical.PermanentMagnet3Phase emf(
    polePairs=3,ke=0.02,trapezoidal=false,theta0=0.2);
  Modelica.Electrical.Analog.Sources.SignalCurrent currents[3];
  Modelica.Electrical.Analog.Basic.Ground ground;
  Modelica.Mechanics.Rotational.Sources.Speed speed(exact=true,useSupport=true);
  Modelica.Mechanics.Rotational.Components.Fixed housing;
  Modelica.Units.SI.Angle theta(start=0,fixed=true);
  Modelica.Units.SI.Power electricalPower;
equation
  connect(emf.flange,speed.flange);
  connect(emf.support,housing.flange);
  connect(speed.support,housing.flange);
  connect(emf.neutral,ground.p);
  speed.w_ref = 10;
  der(theta) = 10;
  for j in 1:3 loop
    connect(currents[j].n,emf.p[j]);
    connect(currents[j].p,ground.p);
    currents[j].i = 2*sin(3*theta+0.2-(j-1)*2*Modelica.Constants.pi/3);
    assert(abs(emf.backEMF[j]-0.2*sin(3*theta+0.2-(j-1)*2*Modelica.Constants.pi/3)) < 1e-7,
      "Electrical angle or phase-peak ke convention mismatch");
  end for;
  electricalPower = sum(emf.backEMF .* currents.i);
  assert(abs(emf.torque-0.06) < 1e-7,"Sinusoidal abc torque must equal 3/2*ke*Ipeak");
  assert(abs(electricalPower-emf.torque*10) < 1e-8,"Lossless conversion power mismatch");
  annotation(experiment(StopTime=0.1,Interval=0.001,Tolerance=1e-9));
end ThreePhaseConversion;
