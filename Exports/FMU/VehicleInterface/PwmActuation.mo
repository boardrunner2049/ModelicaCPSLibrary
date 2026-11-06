within ModelicaCPSLibrary.Exports.FMU.VehicleInterface;
partial model PwmActuation "pwm_actuation_v1: pulse-width commands, one element per mapped host output channel"
  parameter Integer nPwm(min=1) "Number of PWM channels; the exported dimension of pwm";
  parameter Real pwm_min = 1100 "Pulse width of normalized command 0 (zero, or full negative on a centred channel) [us]";
  parameter Real pwm_max = 1900 "Pulse width of normalized command 1 (full, or full positive on a centred channel) [us]";
  input Real pwm[nPwm](each unit="us", each start=1000)
    "Pulse width [us]; element k+1 is mapped host output channel k";
end PwmActuation;
