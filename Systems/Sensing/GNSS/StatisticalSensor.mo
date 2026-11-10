within ModelicaCPSLibrary.Systems.Sensing.GNSS;
model StatisticalSensor "G1: correlated PVT white noise and stationary Gauss-Markov errors"
  extends PartialSampledSensor;
  parameter Integer seed=1;
  parameter Integer streamId=201;
  parameter Real noiseFactor[6,6]=zeros(6,6) "Per-sample covariance square root; PVT row units";
  parameter Real biasFactor[6,6]=zeros(6,6) "Stationary bias covariance square root";
  parameter Real biasTau[6](each unit="s")=fill(100,6) "Time constants of independent latent bias modes";
  input Boolean fixAvailable=true "Scenario receiver availability at acquisition";
  output Real biasState[6] "Diagnostic truth, not receiver quality";
protected
  // Seeded at the first draw (Systems.Sensing.seededState), not by a start
  // value, so seed and streamId stay settable in an exported FMU.
  discrete Integer rngState[Modelica.Math.Random.Generators.Xorshift128plus.nState](each start=0,each fixed=true);
  discrete Integer rngSeed[Modelica.Math.Random.Generators.Xorshift128plus.nState](each start=0,each fixed=true);
  discrete Boolean rngSeeded(start=false,fixed=true);
  discrete Real normal[12](each start=0,each fixed=true);
  discrete Real latentBias[6](each start=0,each fixed=true);
  discrete Boolean initialized(start=false,fixed=true);
  final parameter Real phi[6]=exp(-samplePeriod ./ biasTau);
initial equation
  assert(min(biasTau)>0,"GNSS bias time constants must be positive");
equation
  when sample(startTime,samplePeriod) then
    rngSeed=seededState(pre(rngState),pre(rngSeeded),streamId,seed);
    (normal,rngState)=normalDraw(12,rngSeed);
    rngSeeded=true;
    for j in 1:6 loop
      latentBias[j]=if not pre(initialized) then normal[j]
        else phi[j]*pre(latentBias[j])+sqrt(1-phi[j]^2)*normal[j];
    end for;
    initialized=true;
  end when;
  biasState=biasFactor*latentBias;
  acquiredPvt=cat(1,position_w+positionBias,velocity_w+velocityBias)+biasState+noiseFactor*normal[7:12];
  acquiredCovariance=noiseFactor*transpose(noiseFactor)+biasFactor*transpose(biasFactor);
  acquiredValid=fixAvailable;
end StatisticalSensor;
