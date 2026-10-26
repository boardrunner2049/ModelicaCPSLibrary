within fire_modelica_models.Systems.Sensing.IMU;
partial model PartialSampledSensor "Local RNG, clocked errors, ADC/IIR and fixed sample delivery queue"
  extends PartialSensor;
  parameter Parameters p;
  parameter Boolean deviceReadout = false;
  parameter Boolean enableErrors = true "False for deterministic I1; legacy behavior retained";
  final parameter Integer decimation = if deviceReadout then p.adcDecimation else 1;
  final parameter Real tickPeriod = p.samplePeriod/decimation;
  input Real sensorTemperature(unit="K") = p.referenceTemperature "Device temperature, not automatically ambient";
  output Real sampleTime(unit="s", start=-1, fixed=true) "Delivered sample's acquisition epoch; -1 before valid";
  output Real deliveryTime(unit="s", start=-1, fixed=true);
  output Integer sequence(start=0, fixed=true) "One-based delivered sample sequence";
  output Boolean valid(start=false, fixed=true);
  output Real biasState[6](each start=0, each fixed=true) "Diagnostic only; never estimator truth";
protected
  Sensor ideal(r_b=r_b,R_bs=R_bs);
  Real calibrated[6];
  Real frontEnd[6] "Supplied by the selected sensor physics";
  discrete Integer rngState[Modelica.Math.Random.Generators.Xorshift128plus.nState](
    start=Modelica.Math.Random.Generators.Xorshift128plus.initialState(p.streamId,p.seed), each fixed=true);
  discrete Integer tick(start=-1,fixed=true);
  discrete Integer acquired(start=0,fixed=true);
  discrete Real normal[12](each start=0,each fixed=true);
  discrete Real adc[6](each start=0,each fixed=true);
  discrete Real filtered[6](each start=0,each fixed=true);
  discrete Real history[p.latencySamples+1,6](each start=0,each fixed=true);
  discrete Real delivered[6](each start=0,each fixed=true);
  final parameter Real phi[6] = exp(-tickPeriod ./ p.biasTau);
  final parameter Real pole[6] = {if p.digitalTau[j]>0 then exp(-tickPeriod/p.digitalTau[j]) else 0 for j in 1:6};
initial equation
  assert(p.samplePeriod>0 and p.latencySamples>=0 and p.adcDecimation>=1,"Invalid IMU clock/delay");
  assert(min(p.biasTau)>0 and min(p.biasStd)>=0 and min(p.noiseStd)>=0,"Invalid IMU stochastic parameters");
  assert(not deviceReadout or (min(p.analogTau)>0 and min(p.fullScale)>0 and min(p.quantum)>=0 and min(p.digitalTau)>=0),
    "Invalid IMU device readout parameters");
equation
  ideal.R_wb=R_wb;
  ideal.a_w=a_w;
  ideal.gravity_w=gravity_w;
  ideal.omega_b=omega_b;
  ideal.alpha_b=alpha_b;
  calibrated[1:3]=if enableErrors then (identity(3)+diagonal(p.scale[1:3])+p.accelMisalignment)*ideal.acceleration+accelBias else ideal.acceleration;
  calibrated[4:6]=if enableErrors then (identity(3)+diagonal(p.scale[4:6])+p.gyroMisalignment)*ideal.gyro
    +p.gyroGSensitivity*ideal.acceleration+gyroBias else ideal.gyro;
  acceleration=delivered[1:3];
  gyro=delivered[4:6];
  when sample(0,tickPeriod) then
    tick=pre(tick)+1;
    if enableErrors then
      (normal,rngState)=normalDraw(pre(rngState));
    else
      normal=zeros(12);
      rngState=pre(rngState);
    end if;
    for j in 1:6 loop
      biasState[j]=if not enableErrors then 0 else if pre(tick)<0 then p.biasStd[j]*normal[j]
        else phi[j]*pre(biasState[j])+p.biasStd[j]*sqrt(1-phi[j]^2)*normal[j];
      adc[j]=if deviceReadout then quantize(frontEnd[j]+biasState[j]+p.noiseStd[j]*normal[j+6],p.fullScale[j],p.quantum[j])
        else frontEnd[j]+biasState[j]+p.noiseStd[j]*normal[j+6];
      filtered[j]=if not deviceReadout or pre(tick)<0 then adc[j]
        else pole[j]*pre(filtered[j])+(1-pole[j])*adc[j];
    end for;
    acquired=if mod(tick,decimation)==0 then pre(acquired)+1 else pre(acquired);
    history[1,:]=if mod(tick,decimation)==0 then filtered else pre(history[1,:]);
    for j in 2:p.latencySamples+1 loop
      history[j,:]=if mod(tick,decimation)==0 then pre(history[j-1,:]) else pre(history[j,:]);
    end for;
    delivered=if mod(tick,decimation)==0 and acquired>p.latencySamples then history[p.latencySamples+1,:] else pre(delivered);
    sequence=if mod(tick,decimation)==0 and acquired>p.latencySamples then acquired-p.latencySamples else pre(sequence);
    sampleTime=if mod(tick,decimation)==0 and acquired>p.latencySamples then time-p.latencySamples*p.samplePeriod else pre(sampleTime);
    deliveryTime=if mod(tick,decimation)==0 and acquired>p.latencySamples then time else pre(deliveryTime);
    valid=acquired>p.latencySamples;
  end when;
end PartialSampledSensor;
