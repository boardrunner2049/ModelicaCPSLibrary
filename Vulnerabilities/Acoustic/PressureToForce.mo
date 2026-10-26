within fire_modelica_models.Vulnerabilities.Acoustic;
model PressureToForce "Static package susceptibility: pressure to generalized MEMS forces"
  parameter Real effectiveArea(unit="m2")=1e-10 "Identified area, independent of drive amplitude";
  parameter Real incidenceAngle(unit="rad")=0 "Zero is sense direction, matching MFQuadrotor convention";
  parameter Real driveCoupling[3]=zeros(3);
  parameter Real senseCoupling[3]={0,0,1};
  parameter Real accelCoupling[3]=zeros(3) "Already projected on accelerometer modal axes";
  input Real pressure(unit="Pa")=0;
  output Real gyroDriveForce[3](each unit="N");
  output Real gyroSenseForce[3](each unit="N");
  output Real accelForce[3](each unit="N");
initial equation
  assert(effectiveArea>0,"Effective area must be positive");
equation
  gyroDriveForce=effectiveArea*pressure*sin(incidenceAngle)*driveCoupling;
  gyroSenseForce=effectiveArea*pressure*cos(incidenceAngle)*senseCoupling;
  accelForce=effectiveArea*pressure*accelCoupling;
end PressureToForce;
