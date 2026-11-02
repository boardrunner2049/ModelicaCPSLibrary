within fire_modelica_models.Systems.Sensing.GNSS;
function constellationGeometry "Batch signal geometry; keeps tuple array results inside function scope"
  input Real receiver[3];
  input Real satellite[:,3];
  input Real satelliteVelocity[size(satellite,1),3];
  output Real range[size(satellite,1)];
  output Real gradient[size(satellite,1),3];
  output Real satelliteRate[size(satellite,1)];
  output Real lineOfSight[size(satellite,1),3];
protected
  Real g[3];
  Real los[3];
  Real r;
  Real rate;
algorithm
  for i in 1:size(satellite,1) loop
    (r,g,rate,los):=signalGeometry(receiver,satellite[i,:],satelliteVelocity[i,:]);
    range[i]:=r;
    gradient[i,:]:=g;
    satelliteRate[i]:=rate;
    lineOfSight[i,:]:=los;
  end for;
end constellationGeometry;
