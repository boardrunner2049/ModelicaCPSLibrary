within ModelicaCPSLibrary.Systems.Sensing.Barometer.VendorConfig;
model BMP280 "BMP280 normal-mode acquisition and configurable raw-ADC filter"
  extends PartialModel(sequence(start=0,fixed=true));
  // An experimental policy for the manual's 'may be ignored', not a device setting.
  parameter Boolean normalModeConfigWritesAccepted=true "Experimental policy: accept normal-mode CONFIG writes; not a physical configuration key";
  parameter Integer initialFilter=0 "Initial CONFIG filter coefficient: 0 (off), 2, 4, 8 or 16; nonzero means a preconfigured, still-powered device";
  parameter Real pressureNoiseStd(unit="Pa")=0 "ADC pressure noise RMS before filtering; Bosch ultra-high-resolution typical is 1.3 Pa; zero keeps an analytic ideal ADC";
  input Real commandSequence "Increasing write ID; zero means no command yet";
  input Real commandRegister "Register address, 0..255";
  input Real commandValue "Register byte, 0..255";
  discrete output Real processedCommandSequence(start=0,fixed=true);
  discrete output Real commandStatus(start=0,fixed=true) "0 idle, 1 applied, 2 ignored, 3 emulator coverage gap; not a wire ACK";
  discrete output Real ctrlMeas(start=0,fixed=true) "Live register 0xF4; initially asleep";
  discrete output Real config(start=0,fixed=false) "Live register 0xF5; initialized from initialFilter";
  discrete output Real filterCoefficient(start=0,fixed=true);
  output Real converting "Conversion-active diagnostic, not the complete STATUS register";
  discrete output Real rawPressure(start=524288,fixed=true) "Held unsigned 20-bit pressure count, register 0xF7..0xF9";
  discrete output Real rawTemperature(start=524288,fixed=true) "Held unsigned 20-bit temperature count, register 0xFA..0xFC";
  final output Real chipId=88 "BMP280 identity, 0x58";
  // Synthetic immutable ROM for this arithmetic bench, not measured factory trim.
  final output Real calibration[12]={16384,16384,0,16384,0,0,0,0,0,0,0,16384}
    "T1,T2,T3,P1..P9; the codec reads this same fixture";
  discrete output Real nextTransition(start=Modelica.Constants.inf,fixed=true) "Next device event [s]; the host may split integration here without creating another sensor clock";
protected
  constant Real conversionTime=0.0375 "Typical T x2/P x16 conversion, seconds";
  constant Real standbyTime=0.0005;
  discrete Integer rngState[4](each start=0,each fixed=true);
  discrete Real noise[1](each start=0,each fixed=true);
  discrete Boolean noiseInitialized(start=false,fixed=true);
  discrete Boolean measurementRunning(start=false,fixed=true);
  Boolean commandArrived(start=false,fixed=true) "A new decoded register write arrived";
  Boolean conversionFinished(start=false,fixed=true) "The running ADC conversion has completed";
  Boolean scheduledStart(start=false,fixed=true) "Normal mode's standby interval has ended";
  Boolean ctrlRequest(start=false,fixed=true), ctrlUnchanged(start=false,fixed=true), ctrlChangeAccepted(start=false,fixed=true);
  Boolean configRequest(start=false,fixed=true), configIgnored(start=false,fixed=true), configAccepted(start=false,fixed=true);
  Boolean startConversion(start=false,fixed=true) "Capture one ADC input for a scheduled or commanded start";
  discrete Boolean filterInitialized(start=false,fixed=true);
  discrete Real pendingPressure(start=0,fixed=true), pendingTemperature(start=0,fixed=true);
  discrete Real pendingSampleTime(start=0,fixed=true);
  discrete Real lastSampleTime(start=-1,fixed=true), lastDeliveryTime(start=-1,fixed=true);
  discrete Real filteredPressure(start=0,fixed=true), filteredTemperature(start=0,fixed=true);
  Real pressureBeforeSecondOrder;
  Integer writeId(start=0,fixed=true)=integer(commandSequence);
initial equation
  assert(initialFilter==0 or initialFilter==2 or initialFilter==4 or initialFilter==8 or initialFilter==16,
    "BMP280 initialFilter must be 0, 2, 4, 8 or 16");
  assert(pressureNoiseStd>=0,"BMP280 ADC pressure noise must be nonnegative");
  config=if initialFilter==0 then 0 else if initialFilter==2 then 4 else if initialFilter==4 then 8 else if initialFilter==8 then 12 else 16;
equation
  assert(noEvent(commandSequence>=0 and commandSequence<=2147483647 and
    abs(commandSequence-floor(commandSequence))<=0),"BMP280 write ID must be an integer in 0..2147483647");
  // Continuous B0 has no delivery clock: its sequence stays zero and valid.
  assert(noEvent(measuredSequence==0 and measuredValid),
    "BMP280 currently requires continuous B0 physics; sampled physics would add a second acquisition clock");
  assert(noEvent(measuredPressure>=30000 and measuredPressure<=110000 and
    measuredTemperature>=273.15 and measuredTemperature<=338.15),
    "BMP280 bench input envelope is 30000..110000 Pa and 273.15..338.15 K");
  converting=if measurementRunning then 1 else 0;
  // A derived register readback; this event adds no acquisition clock.
  when {initial(),change(config)} then
    filterCoefficient=if config<=0 then 0 else 2^min(4,integer(config/4));
  end when;
  valid=sequence>0;
  sampleTime=lastSampleTime;
  deliveryTime=lastDeliveryTime;
  // Decode the same rounded counts sent to the native driver, using this ROM.
  pressureBeforeSecondOrder=(1048576-rawPressure)*6250/16384;
  pressure=pressureBeforeSecondOrder+pressureBeforeSecondOrder^2/2097152;
  temperature=(rawTemperature-262144)/5120+273.15;
  // Capture the ADC input and one noise draw at each real conversion start.
  when startConversion then
    (noise,rngState)=ModelicaCPSLibrary.Systems.Sensing.seededNormalDraw(1,pre(rngState),pre(noiseInitialized),401,1);
    noiseInitialized=true;
    // Inverse of Bosch's compensation polynomial for the fixed ROM above.
    pendingPressure=1048576-(2*(measuredPressure+pressureNoiseStd*noise[1])/
      (1+sqrt(1+4*(measuredPressure+pressureNoiseStd*noise[1])/2097152)))*16384/6250;
    pendingTemperature=262144+5120*(measuredTemperature-273.15);
    pendingSampleTime=time;
  end when;
algorithm
  when {change(writeId),time>=pre(nextTransition)} then
    // Decide from the state before this event; each decision is assigned once.
    commandArrived:=change(writeId);
    conversionFinished:=time>=pre(nextTransition) and pre(measurementRunning);
    scheduledStart:=time>=pre(nextTransition) and not pre(measurementRunning) and integer(pre(ctrlMeas))==87;
    ctrlRequest:=commandArrived and integer(commandRegister)==244 and
      (integer(commandValue)==0 or integer(commandValue)==84 or integer(commandValue)==87);
    ctrlUnchanged:=ctrlRequest and abs(commandValue-pre(ctrlMeas))<=0;
    ctrlChangeAccepted:=ctrlRequest and not ctrlUnchanged and
      (not pre(measurementRunning) or time>=pre(nextTransition)) and not scheduledStart;
    configRequest:=commandArrived and integer(commandRegister)==245 and
      integer(mod(commandValue,4))==0 and commandValue<=28;
    configIgnored:=configRequest and integer(pre(ctrlMeas))==87 and not normalModeConfigWritesAccepted;
    configAccepted:=configRequest and not configIgnored;
    startConversion:=scheduledStart or (ctrlChangeAccepted and integer(commandValue)==87);
    // Publish the completed conversion before applying this event's register write.
    if conversionFinished then
      filteredPressure:=if not pre(filterInitialized) or pre(filterCoefficient)<=0 then pre(pendingPressure)
        else (pre(filteredPressure)*(pre(filterCoefficient)-1)+pre(pendingPressure))/pre(filterCoefficient);
      filteredTemperature:=if not pre(filterInitialized) or pre(filterCoefficient)<=0 then pre(pendingTemperature)
        else (pre(filteredTemperature)*(pre(filterCoefficient)-1)+pre(pendingTemperature))/pre(filterCoefficient);
      rawPressure:=floor(filteredPressure+0.5);
      rawTemperature:=floor(filteredTemperature+0.5);
      sequence:=pre(sequence)+1;
      lastSampleTime:=pre(pendingSampleTime);
      lastDeliveryTime:=time;
    else
      lastSampleTime:=pre(lastSampleTime);
      lastDeliveryTime:=pre(lastDeliveryTime);
    end if;
    if commandArrived then
      assert(commandSequence>pre(processedCommandSequence) and commandSequence<=2147483647 and
        abs(commandSequence-floor(commandSequence))<=0 and commandRegister>=0 and commandRegister<=255 and
        abs(commandRegister-floor(commandRegister))<=0 and commandValue>=0 and commandValue<=255 and
        abs(commandValue-floor(commandValue))<=0,"BMP280 writes require increasing integer IDs and register bytes");
      processedCommandSequence:=commandSequence;
      commandStatus:=if ctrlUnchanged or ctrlChangeAccepted or configAccepted then 1
        else if configIgnored then 2 else 3;
    end if;
    if ctrlChangeAccepted then
      ctrlMeas:=commandValue;
    end if;
    if configAccepted then
      config:=commandValue;
    end if;
    filterInitialized:=if configAccepted then false else if conversionFinished then true else pre(filterInitialized);
    measurementRunning:=if startConversion then true else if conversionFinished then false else pre(measurementRunning);
    nextTransition:=if startConversion then time+conversionTime
      else if ctrlChangeAccepted then Modelica.Constants.inf
      else if conversionFinished then time+standbyTime else pre(nextTransition);
  end when;
end BMP280;
