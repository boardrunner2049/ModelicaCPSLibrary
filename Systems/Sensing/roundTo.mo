within ModelicaCPSLibrary.Systems.Sensing;
function roundTo "Nearest multiple of quantum (ties toward positive infinity); quantum <= 0 leaves u unchanged"
  input Real u;
  input Real quantum;
  output Real y;
algorithm
  y:=if quantum>0 then quantum*floor(u/quantum+0.5) else u;
end roundTo;
