within ModelicaCPSLibrary.Exports.FMU;
partial model CopterInterface "Common P0/P1 VirtualIO PWM and sensor boundary; composer binds plant semantics"
  extends PartialCopterInterface;
  Adapters.VirtualIO.PwmInput commands(nChannels=nPwm,
    pwmMin=fill(pwm_min,nPwm),pwmMax=fill(pwm_max,nPwm));
equation
  commands.pulseWidth_us=pwm;
end CopterInterface;
