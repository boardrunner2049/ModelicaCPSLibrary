within ModelicaCPSLibrary.Tests;
model ElectromechanicalLoad "Load, inertia and inductance independently affect motor response"
  Physical.Electrical.Drives.ESC.AverageUnidirectional esc[4];
  Systems.Propulsion.Motors.DynamicDCEquivalent motors[4](
    each R=0.2,L={0.001,0.001,0.001,0.002},each k=0.02,
    J={0.0001,0.0001,0.0004,0.0001});
  parameter Real kQ[4]={1.5e-7,3e-7,1.5e-7,1.5e-7};
  Real expectedSpeed[4](each unit="rad/s");
equation
  esc.escCommand=fill(0.5,4);
  esc.dcVoltage=fill(16,4);
  motors.currentZero=esc.coast;
  esc.openCircuitVoltage=motors.backEMF;
  connect(esc.p,motors.p);
  connect(esc.n,motors.n);
  for i in 1:4 loop
    motors[i].loadTorque=kQ[i]*motors[i].omega^2;
    expectedSpeed[i]=(-0.02+sqrt(0.02^2+4*0.2*kQ[i]*8/0.02))/(2*0.2*kQ[i]/0.02);
    assert(motors[i].current>=-1e-6,"Negative current in unidirectional drive");
    assert(time<3 or abs(motors[i].omega-expectedSpeed[i])<0.01,"Loaded equilibrium speed failed");
    assert(time<3 or abs(0.02*motors[i].current-motors[i].loadTorque)<1e-5,"Steady torque balance failed");
  end for;
  assert(time<=0.00005 or time>=0.00015 or motors[4].current<0.6*motors[1].current,"L must affect startup current slope");
  assert(time<=0.05 or time>=0.15 or motors[3].omega<motors[1].omega,"Inertia must slow acceleration");
  assert(time<3 or (motors[2].omega<motors[1].omega and motors[2].current>motors[1].current),"Load feedback failed");
  annotation(experiment(StopTime=3.1,Tolerance=1e-9,Interval=0.001));
end ElectromechanicalLoad;
