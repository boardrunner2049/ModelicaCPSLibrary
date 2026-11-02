within ModelicaCPSLibrary.Adapters.VirtualIO;
model PwmInput "Continuous PWM pulse-width value to normalized command; independent of plant semantics"
  parameter Integer nChannels(min=1)=4;
  parameter Real pwmMin[nChannels]=fill(1000,nChannels) "microseconds";
  parameter Real pwmMax[nChannels]=fill(2000,nChannels) "microseconds";
  input Real pulseWidth_us[nChannels] "microseconds; numeric pulse width, no edge decoding";
  output Real normalizedCommand[nChannels](each unit="1") "Clamped PWM position; plant defines its physical meaning";
initial equation
  for i in 1:nChannels loop
    assert(pwmMax[i]>pwmMin[i],"PWM range must be positive");
  end for;
equation
  for i in 1:nChannels loop
    normalizedCommand[i]=noEvent(min(1,max(0,(pulseWidth_us[i]-pwmMin[i])/(pwmMax[i]-pwmMin[i]))));
  end for;
end PwmInput;
