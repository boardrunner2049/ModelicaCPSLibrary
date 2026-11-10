within ModelicaCPSLibrary.Systems.Peripherals;
model SampledVisualOdometry "V1: V0 at the output rate; position drift growing with the distance travelled (driftFraction of each step, in a slowly varying direction), heading random walk, noise, latency"
  extends PartialPeripheral;
  parameter Real samplePeriod(unit="s") = 0.033;
  parameter Real startTime(unit="s") = 0;
  parameter Integer latencySamples(min=0) = 0;
  parameter Integer seed = 1;
  parameter Integer streamId = 381;
  parameter Real driftFraction = 0 "Position drift per metre travelled (0.01: 1 %)";
  parameter Real driftTau(unit="s") = 60 "Correlation time of the drift direction";
  parameter Real headingWalk = 0 "Heading random walk [rad/sqrt(s)]";
  parameter Real positionNoiseStd(unit="m") = 0;
  parameter Real velocityNoiseStd(unit="m/s") = 0;
  parameter Real attitudeNoiseStd(unit="rad") = 0;
  output Real position[3](each unit="m");
  output Real velocity[3](each unit="m/s");
  output Real attitude[3](each unit="rad");
  output Real quaternion[4];
  output Boolean valid;
  output Real validity = if valid then 1 else 0 "1 if valid, else 0";
  output Real travelled(unit="m") "Diagnostic: path length";
protected
  VisualOdometry ideal(position_b=position_b,R_bs=R_bs);
  discrete Integer rngState[Modelica.Math.Random.Generators.Xorshift128plus.nState](
    start=Modelica.Math.Random.Generators.Xorshift128plus.initialState(streamId,seed),each fixed=true);
  discrete Real normal[4](each start=0,each fixed=true);
  discrete Real direction[3](each start=0,each fixed=true);
  discrete Real drift[3](each start=0,each fixed=true);
  discrete Real heading(start=0,fixed=true);
  discrete Real last[3](each start=0,each fixed=true);
  discrete Real path(start=0,fixed=true);
  discrete Real step(start=0,fixed=true);
  discrete Boolean initialized(start=false,fixed=true);
  Sampler sampler(n=9,samplePeriod=samplePeriod,startTime=startTime,latencySamples=latencySamples,
    seed=seed,streamId=streamId+1,
    noiseStd=cat(1,fill(positionNoiseStd,3),fill(velocityNoiseStd,3),fill(attitudeNoiseStd,3)));
equation
  ideal.truth = truth;
  when sample(startTime, samplePeriod) then
    (normal, rngState) = Sensing.normalDraw(4, pre(rngState));
    step = if pre(initialized) then sqrt((ideal.position - pre(last))*(ideal.position - pre(last))) else 0;
    for i in 1:3 loop
      direction[i] = if not pre(initialized) then normal[i]
        else exp(-samplePeriod/driftTau)*pre(direction[i]) + sqrt(1 - exp(-2*samplePeriod/driftTau))*normal[i];
      drift[i] = pre(drift[i]) + driftFraction*step*direction[i];
      last[i] = ideal.position[i];
    end for;
    heading = pre(heading) + headingWalk*sqrt(samplePeriod)*normal[4];
    path = pre(path) + step;
    initialized = true;
  end when;
  sampler.u = cat(1, ideal.position + drift, ideal.velocity, ideal.attitude + {0, 0, heading});
  position = sampler.y[1:3];
  velocity = sampler.y[4:6];
  attitude = sampler.y[7:9];
  quaternion = eulerQuaternion(sampler.y[7:9]);
  valid = sampler.valid;
  travelled = path;
end SampledVisualOdometry;
