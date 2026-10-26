within fire_modelica_models.Tests;
model ImuDevice "Analog response, temperature, ADC clipping/rounding, IIR and independent output clock"
  Systems.Sensing.IMU.DeviceSensor analog(p(samplePeriod=0.004,adcDecimation=4,analogTau=fill(0.01,6)));
  Systems.Sensing.IMU.DeviceSensor digital(p(samplePeriod=0.004,adcDecimation=4,analogTau=fill(0.01,6),digitalTau=fill(0.002,6)));
  Systems.Sensing.IMU.DeviceSensor adc(p(samplePeriod=0.004,adcDecimation=4,fullScale=fill(10,6),quantum=fill(0.25,6)));
  Systems.Sensing.IMU.DeviceSensor thermal(p(samplePeriod=0.004,adcDecimation=4,temperatureCoefficient={1,0,0,0,0,0}),sensorTemperature=298.15);
protected
  final parameter Real a=exp(-0.001/0.002);
  final parameter Real b=exp(-0.001/0.01);
  final parameter Real referenceIIR=10*((1-a^10)-(1-a)*b*(a^10-b^10)/(a-b));
equation
  analog.R_wb=identity(3);
  analog.a_w={if time<0.01 then 0 else 10,0,0};
  analog.gravity_w=zeros(3);
  analog.omega_b=zeros(3);
  analog.alpha_b=zeros(3);
  digital.R_wb=identity(3);
  digital.a_w={if time<0.01 then 0 else 10,0,0};
  digital.gravity_w=zeros(3);
  digital.omega_b=zeros(3);
  digital.alpha_b=zeros(3);
  adc.R_wb=identity(3);
  adc.a_w={12.34,-0.37,0};
  adc.gravity_w=zeros(3);
  adc.omega_b=zeros(3);
  adc.alpha_b=zeros(3);
  thermal.R_wb=identity(3);
  thermal.a_w=zeros(3);
  thermal.gravity_w=zeros(3);
  thermal.omega_b=zeros(3);
  thermal.alpha_b=zeros(3);
  when time>=0.0201 then
    assert(abs(analog.sampleTime-0.020)<1e-10,"Output clock must decimate ADC ticks");
    assert(abs(analog.acceleration[1]-10*(1-exp(-1)))<1e-5,"Analog bandwidth response is incorrect");
    assert(abs(digital.acceleration[1]-referenceIIR)<1e-5,"Digital IIR must run at ADC rate before decimation");
    assert(abs(adc.acceleration[1]-10)<1e-12 and abs(adc.acceleration[2]+0.25)<1e-12,"ADC saturation or nearest quantization is incorrect");
    assert(abs(thermal.acceleration[1]-5)<1e-9,"Device temperature coefficient was not applied");
  end when;
  when time>=0.023 then
    assert(abs(analog.sampleTime-0.020)<1e-10,"Sampled device output changed between output ticks");
  end when;
  annotation(experiment(StopTime=0.05,Interval=0.0001,Tolerance=1e-10));
end ImuDevice;
