within ModelicaCPSLibrary.Systems.Peripherals;
model OpticalFlow "F0: image flow rates about the device x and y axes (ArduPilot's convention: x = -v_y/r + p, y = v_x/r + q, r the slant range along the boresight, device +z) and the device's x and y body rates; valid within the range band; continuous"
  extends PartialPeripheral;
  parameter Real minRange(unit="m") = 0.08 "Closest resolvable surface";
  parameter Real maxRange(unit="m") = 30 "Farthest resolvable surface";
  parameter Real groundDown(unit="m") = 0 "Ground plane, NED down coordinate";
  output Real flowRate[2](each unit="rad/s");
  output Real bodyRate[2](each unit="rad/s");
  output Boolean valid "A surface is in range";
  output Real validity = if valid then 1 else 0 "1 if valid, else 0";
  output Real range(unit="m") "Diagnostic: slant range along the boresight (1e30 if none)";
protected
  Real boresightDown "Down component of the boresight";
  Real v_s[3](each unit="m/s");
  Real w_s[3](each unit="rad/s");
equation
  boresightDown=truth.R_wb[3,:]*R_bs[:,3];
  range=noEvent(if boresightDown>0.05 then max(groundDown-mountPosition_w[3],0)/boresightDown else 1e30);
  valid=range>=minRange and range<=maxRange;
  v_s=transpose(R_bs)*mountVelocity_b;
  w_s=transpose(R_bs)*truth.omega_b;
  flowRate[1]=noEvent(if valid then -v_s[2]/range+w_s[1] else w_s[1]);
  flowRate[2]=noEvent(if valid then v_s[1]/range+w_s[2] else w_s[2]);
  bodyRate={w_s[1],w_s[2]};
end OpticalFlow;
