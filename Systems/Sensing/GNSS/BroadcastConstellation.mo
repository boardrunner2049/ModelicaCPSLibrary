within fire_modelica_models.Systems.Sensing.GNSS;
model BroadcastConstellation "Ephemeris provider at reception epoch; supply parsed broadcast records"
  parameter Integer nSat(min=1)=4;
  parameter BroadcastEphemeris ephemeris[nSat];
  parameter Real epoch0(unit="s")=0 "GNSS time at simulation time zero";
  output Real position_e[nSat,3](each unit="m");
  output Real velocity_e[nSat,3](each unit="m/s");
  output Real clockBias[nSat](each unit="s");
  output Real clockDrift[nSat];
  output Boolean available[nSat];
equation
  (position_e,velocity_e,clockBias,clockDrift,available)=broadcastStates(ephemeris,epoch0+time);
end BroadcastConstellation;
