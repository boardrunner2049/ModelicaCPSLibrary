within ModelicaCPSLibrary.Vehicles.Copter.Presets;
record HexaX
  extends Geometry(nArms=6,nRotors=6,nActuators=6,
    armMount=Utilities.Math.regularPolygon(6,0.3,0,Modelica.Constants.pi/6)*diagonal({1,-1,-1}));
end HexaX;
