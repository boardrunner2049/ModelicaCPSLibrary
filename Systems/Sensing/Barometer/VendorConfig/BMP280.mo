within ModelicaCPSLibrary.Systems.Sensing.Barometer.VendorConfig;
model BMP280 "BMP280 normal-mode acquisition and configurable raw-ADC filter; bench subset"
  extends PartialModel(sequence(start=0,fixed=true));
  // An experimental policy for the manual's 'may be ignored', not a device setting.
  parameter Boolean normalModeConfigWritesAccepted=true;
  input Real commandSequence "Increasing write ID; zero means no command yet";
  input Real commandRegister "Register address, 0..255";
  input Real commandValue "Register byte, 0..255";
  discrete output Real processedCommandSequence(start=0,fixed=true);
  discrete output Real commandStatus(start=0,fixed=true) "0 idle, 1 applied, 2 ignored, 3 emulator coverage gap; not a wire ACK";
  discrete output Real ctrlMeas(start=0,fixed=true) "Live register 0xF4; initially asleep";
  discrete output Real config(start=0,fixed=true) "Live register 0xF5; initially filter off";
  output Real filterCoefficient;
  output Real converting "Conversion-active diagnostic, not the complete STATUS register";
  discrete output Real rawPressure(start=524288,fixed=true) "Held unsigned 20-bit pressure count, register 0xF7..0xF9";
  discrete output Real rawTemperature(start=524288,fixed=true) "Held unsigned 20-bit temperature count, register 0xFA..0xFC";
  final output Real chipId=88 "BMP280 identity, 0x58";
  // Synthetic immutable ROM for this arithmetic bench, not measured factory trim.
  final output Real calibration[12]={16384,16384,0,16384,0,0,0,0,0,0,0,16384}
    "T1,T2,T3,P1..P9; the codec reads this same fixture";
protected
  constant Real conversionTime=0.0375 "Typical T x2/P x16 conversion, seconds";
  constant Real standbyTime=0.0005;
  discrete Real nextTransition(start=Modelica.Constants.inf,fixed=true);
  discrete Boolean measurementRunning(start=false,fixed=true);
  discrete Boolean startConversion(start=false,fixed=true);
  discrete Boolean filterInitialized(start=false,fixed=true);
  discrete Real pendingPressure(start=0,fixed=true), pendingTemperature(start=0,fixed=true);
  discrete Real pendingSampleTime(start=0,fixed=true);
  discrete Real lastSampleTime(start=-1,fixed=true), lastDeliveryTime(start=-1,fixed=true);
  discrete Real filteredPressure(start=0,fixed=true), filteredTemperature(start=0,fixed=true);
  Real pressureBeforeSecondOrder;
  Integer writeId(start=0,fixed=true)=integer(commandSequence);
equation
  assert(noEvent(commandSequence>=0 and commandSequence<=2147483647 and
    abs(commandSequence-floor(commandSequence))<=0),"BMP280 write ID must be an integer in 0..2147483647");
  assert(noEvent(measuredSequence==0 and measuredValid and
    abs(measuredSampleTime-time)<=0 and abs(measuredDeliveryTime-time)<=0),
    "BMP280 currently requires continuous B0 physics; sampled physics would add a second acquisition clock");
  assert(noEvent(measuredPressure>=30000 and measuredPressure<=110000 and
    measuredTemperature>=273.15 and measuredTemperature<=338.15),
    "BMP280 bench input envelope is 30000..110000 Pa and 273.15..338.15 K");
  converting=if measurementRunning then 1 else 0;
  filterCoefficient=if config<=0 then 0 else 2^min(4,integer(config/4));
  valid=sequence>0;
  sampleTime=lastSampleTime;
  deliveryTime=lastDeliveryTime;
  // Decode the same rounded counts sent to the native driver, using this ROM.
  pressureBeforeSecondOrder=(1048576-rawPressure)*6250/16384;
  pressure=pressureBeforeSecondOrder+pressureBeforeSecondOrder^2/2097152;
  temperature=(rawTemperature-262144)/5120+273.15;
algorithm
  when {change(writeId),time>=pre(nextTransition)} then
    lastSampleTime:=pre(lastSampleTime);
    lastDeliveryTime:=pre(lastDeliveryTime);
    startConversion:=false;
    // A completed conversion is visible before a write completing at the same time.
    if time>=pre(nextTransition) then
      if pre(measurementRunning) then
        filteredPressure:=if not pre(filterInitialized) or pre(filterCoefficient)<=0 then pre(pendingPressure)
          else (pre(filteredPressure)*(pre(filterCoefficient)-1)+pre(pendingPressure))/pre(filterCoefficient);
        filteredTemperature:=if not pre(filterInitialized) or pre(filterCoefficient)<=0 then pre(pendingTemperature)
          else (pre(filteredTemperature)*(pre(filterCoefficient)-1)+pre(pendingTemperature))/pre(filterCoefficient);
        filterInitialized:=true;
        rawPressure:=floor(filteredPressure+0.5);
        rawTemperature:=floor(filteredTemperature+0.5);
        sequence:=pre(sequence)+1;
        lastSampleTime:=pre(pendingSampleTime);
        lastDeliveryTime:=time;
        measurementRunning:=false;
        nextTransition:=time+standbyTime;
      else
        startConversion:=integer(pre(ctrlMeas))==87;
      end if;
    end if;
    if change(writeId) then
      assert(commandSequence>pre(processedCommandSequence) and commandSequence<=2147483647 and
        abs(commandSequence-floor(commandSequence))<=0 and commandRegister>=0 and commandRegister<=255 and
        abs(commandRegister-floor(commandRegister))<=0 and commandValue>=0 and commandValue<=255 and
        abs(commandValue-floor(commandValue))<=0,"BMP280 writes require increasing integer IDs and register bytes");
      processedCommandSequence:=commandSequence;
      commandStatus:=3;
      if integer(commandRegister)==244 and (integer(commandValue)==0 or integer(commandValue)==84 or integer(commandValue)==87) then
        if abs(commandValue-pre(ctrlMeas))<=0 then
          commandStatus:=1;
        elseif (not pre(measurementRunning) or time>=pre(nextTransition)) and not startConversion then
          ctrlMeas:=commandValue;
          commandStatus:=1;
          startConversion:=integer(commandValue)==87;
          nextTransition:=Modelica.Constants.inf;
        end if;
      elseif integer(commandRegister)==245 and integer(mod(commandValue,4))==0 and commandValue<=28 then
        if integer(pre(ctrlMeas))==87 and not normalModeConfigWritesAccepted then
          commandStatus:=2;
        else
          config:=commandValue;
          filterInitialized:=false;
          commandStatus:=1;
        end if;
      end if;
    end if;
    if startConversion then
      // Inverse of Bosch's compensation polynomial for the fixed ROM above.
      pendingPressure:=1048576-(2*measuredPressure/
        (1+sqrt(1+4*measuredPressure/2097152)))*16384/6250;
      pendingTemperature:=262144+5120*(measuredTemperature-273.15);
      pendingSampleTime:=time;
      measurementRunning:=true;
      nextTransition:=time+conversionTime;
    end if;
  end when;
end BMP280;
