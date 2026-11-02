within ModelicaCPSLibrary.Systems.Sensing.IMU;
function quantize "Symmetric clipping, then nearest ADC code (ties toward positive infinity)"
  input Real u;
  input Real fullScale;
  input Real quantum;
  output Real y;
protected
  Real clipped;
algorithm
  clipped:=min(fullScale,max(-fullScale,u));
  y:=if quantum>0 then min(fullScale,max(-fullScale,quantum*floor(clipped/quantum+0.5))) else clipped;
end quantize;
