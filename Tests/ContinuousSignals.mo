within ModelicaCPSLibrary.Tests;
model ContinuousSignals "Sensor values and PWM normalization change between former acquisition instants"
  Systems.Sensing.GNSS.Sensor gnss;
  Systems.Sensing.Barometer.Sensor barometer;
  Adapters.VirtualIO.PwmInput commands(nChannels=6);
equation
  gnss.position_w={time,2*time,-3*time};
  gnss.velocity_w={1,2,-3};
  barometer.ambientPressure=101325-10*time;
  barometer.ambientTemperature=288.15+time;
  barometer.altitude_w=3*time;
  barometer.climbRate_w=3;
  commands.pulseWidth_us=fill(1000+1000*time,6);
  assert(max(abs(gnss.position-{time,2*time,-3*time}))<1e-12,"GNSS must not hold outputs");
  assert(abs(barometer.pressure-(101325-10*time))<1e-9 and abs(barometer.altitude-3*time)<1e-12,"Barometer must be continuous");
  assert(max(abs(commands.normalizedCommand-fill(time,6)))<1e-12,"Every PWM channel must normalize continuously");
  annotation(experiment(StopTime=0.035,Tolerance=1e-9,Interval=0.0007));
end ContinuousSignals;
