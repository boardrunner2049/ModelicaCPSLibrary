within ModelicaCPSLibrary.Vehicles.Copter;
record Geometry "Fixed mounts resolved in chassis reference C; SI units"
  parameter Integer nArms(min=1) = 4;
  parameter Integer nRotors(min=1) = nArms;
  parameter Integer nActuators(min=1) = nRotors;
  parameter Real armMount[nArms,3] = Utilities.Math.regularPolygon(nArms,0.25,0,Modelica.Constants.pi/4)*diagonal({1,-1,-1})
    "Arm tip positions relative to C; arm mass location is specified separately";
  parameter Integer rotorArmIndex[nRotors] = {1+mod(i-1,nArms) for i in 1:nRotors};
  parameter Real rotorPosition_C[nRotors,3] = {armMount[rotorArmIndex[i],:] for i in 1:nRotors};
  parameter Real R_br[nRotors,3,3] = {diagonal({1,-1,-1}) for i in 1:nRotors}
    "Body-to-rotor relative attitude; v_b = R_br*v_r, local +z thrust";
  parameter Integer spinSign[nRotors] = {if mod(i,2)==1 then 1 else -1 for i in 1:nRotors};
  parameter Integer actuatorIndex[nRotors] = {i for i in 1:nRotors};
  parameter Real imuPosition_C[3] = zeros(3);
  parameter Real gnssPosition_C[3] = zeros(3);
  parameter Real barometerPosition_C[3] = zeros(3);
  parameter Real R_bImu[3,3] = diagonal({1,-1,-1});
  parameter Real R_bMag[3,3] = diagonal({1,-1,-1});
end Geometry;
