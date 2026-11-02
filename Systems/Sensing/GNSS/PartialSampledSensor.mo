within fire_modelica_models.Systems.Sensing.GNSS;
partial model PartialSampledSensor "Common G1/G2 PVT, covariance and epoch delivery contract"
  extends PartialSensor;
  parameter Real samplePeriod(unit="s")=0.2;
  parameter Real startTime(unit="s")=0;
  parameter Integer latencySamples(min=0)=0;
  output Real sampleTime(unit="s");
  output Real deliveryTime(unit="s");
  output Integer sequence;
  output Boolean valid;
  output Integer fixType "0=no fix, 3=3D fix; no RTK claim";
  output Real covariance[6,6] "Reported NED PVT covariance; meaningful only when valid";
protected
  Real acquiredPvt[6];
  Real acquiredCovariance[6,6];
  Boolean acquiredValid;
  SampledDelivery delivery(n=42,samplePeriod=samplePeriod,startTime=startTime,latencySamples=latencySamples);
equation
  delivery.u[1:6]=if acquiredValid then acquiredPvt else zeros(6);
  for i in 1:6 loop
    for j in 1:6 loop
      delivery.u[6+(i-1)*6+j]=if acquiredValid then acquiredCovariance[i,j] else 0;
      covariance[i,j]=delivery.y[6+(i-1)*6+j];
    end for;
  end for;
  delivery.acquisitionValid=acquiredValid;
  delivery.acquisitionStatus=if acquiredValid then 3 else 0;
  position=delivery.y[1:3];
  velocity=delivery.y[4:6];
  sampleTime=delivery.sampleTime;
  deliveryTime=delivery.deliveryTime;
  sequence=delivery.sequence;
  valid=delivery.valid;
  fixType=delivery.status;
end PartialSampledSensor;
